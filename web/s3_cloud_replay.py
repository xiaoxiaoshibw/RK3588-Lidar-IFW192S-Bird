"""Replay original BIRDRAW point clouds alongside their matched S3 UART results."""

import argparse
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np

from bird_audit.raw_capture import read_capture
from .s3_live_server import LiveHub, make_handler


def _json_lines(path):
    with path.open('rb') as stream:
        for line in stream:
            yield json.loads(line)


def _index_rows(rows, key):
    indexed = {}
    for row in rows:
        value = row.get(key)
        if type(value) is not int or value < 0 or value in indexed:
            raise ValueError(f'invalid or repeated {key}: {value}')
        indexed[value] = row
    return indexed


def _paired_rows(timings, uart):
    by_source = _index_rows(_json_lines(timings), 'input_ordinal')
    _index_rows(by_source.values(), 'seq')
    by_ordinal = _index_rows(by_source.values(), 'ordinal')
    rows = list(_json_lines(uart))
    frames = _index_rows((row for row in rows if row.get('type') == 'frame'), 'ordinal')
    paced = _index_rows((row for row in rows if row.get('type') == 'paced_timing'), 'ordinal')
    active = {ordinal for ordinal, row in frames.items() if row['result']['phase'] == 'ACTIVE'}
    if not by_source or by_ordinal.keys() != active or paced.keys() != active:
        raise ValueError('active UART results, paced timing and source mapping must have the same ordinals')
    for ordinal, mapping in by_ordinal.items():
        frame = frames[ordinal]
        result = frame['result']
        if (len(result['locked_birds']) != mapping['locked']
                or frame['tracker_us'] != mapping['tracker_us']
                or paced[ordinal]['ready_us'] != mapping['ready_us']):
            raise ValueError(f'S3 UART result disagrees with mapped timing at ordinal {ordinal}')
    return {source: (mapping, frames[mapping['ordinal']]['result']) for source, mapping in by_source.items()}


def _cloud_snapshot(mapping, result, rows):
    xyz = np.column_stack([rows[axis] for axis in 'xyz'])
    measured = np.isfinite(xyz).all(axis=1) & (xyz != 0).any(axis=1)
    tracks = []
    for track in result['locked_birds']:
        indices = np.asarray(track['source_rows'])
        cells = np.asarray(track['cells'])
        count = track['point_count']
        center = track['centroid_xyz_m']
        if (not count or indices.shape != (count,) or indices.dtype.kind not in 'iu'
                or np.unique(indices).size != count or np.any(indices < 0) or np.any(indices >= len(rows))
                or cells.shape != (count,) or cells.dtype.kind not in 'iu'
                or np.any(cells < 0) or np.any(cells >= 192 * 256)
                or not measured[indices].all() or not np.array_equal(rows['ring'][indices], cells // 256)
                or not np.allclose(xyz[indices].mean(axis=0, dtype=np.float64), center, rtol=0, atol=1e-6)):
            raise ValueError(f"S3 track source rows do not match original cloud at seq {mapping['seq']}")
        tracks.append(dict(id=track['track_id'], box=track['grid_box_gx0_gy0_gx1_gy1'],
                           xyz_m=center, points=count, range_m=round(math.hypot(*center), 4)))
    packed_xyz = xyz[measured].astype('<f4', copy=False)
    return dict(seq=mapping['seq'], state='active', warmup=0, locked=len(tracks),
                candidates=result['dynamic_candidates'], tentative=result['tentative_count'],
                latency_ms=mapping['ready_us'] / 1000, tracker_ms=mapping['tracker_us'] / 1000,
                cloud_points=len(packed_xyz), input_ordinal=mapping['input_ordinal'], tracks=tracks), packed_xyz


def load_replay(capture, timings, uart, reference):
    paired = _paired_rows(timings, uart)
    sys.path.insert(0, str(reference / 'src/human_fall_detection'))
    from core.range_live import pointcloud_rows
    from core.range_wire import parse_cloud

    header, messages = read_capture(capture)
    channel = header.get('channel', {})
    if (channel.get('topic'), channel.get('encoding'), channel.get('schemaName')) != (
            '/innolidar_points', 'ros1', 'sensor_msgs/PointCloud2'):
        raise ValueError('capture channel must be ROS1 /innolidar_points PointCloud2')
    frames = []
    cloud_frames = {}
    minimum = np.full(3, np.inf)
    maximum = np.full(3, -np.inf)
    for ordinal, message in enumerate(messages):
        if ordinal not in paired:
            continue
        mapping, result = paired[ordinal]
        cloud = parse_cloud(message['payload'])
        if cloud.header.seq != mapping['seq']:
            raise ValueError(f'original cloud sequence mismatch at input ordinal {ordinal}')
        frame, packed_xyz = _cloud_snapshot(mapping, result, pointcloud_rows(cloud))
        frames.append(frame)
        cloud_frames[frame['seq']] = packed_xyz.tobytes()
        if len(packed_xyz):
            minimum = np.minimum(minimum, packed_xyz.min(axis=0))
            maximum = np.maximum(maximum, packed_xyz.max(axis=0))
        if len(frames) == len(paired):
            break
    if len(frames) != len(paired):
        raise ValueError('capture is missing one or more source ordinals in the replay mapping')
    if not np.isfinite(minimum).all():
        raise ValueError('capture has no measured XYZ in the mapped replay frames')
    bounds = np.concatenate((minimum, maximum)).tolist()
    for frame in frames:
        frame['cloud_bounds'] = bounds
    return frames, cloud_frames


def _play(frames, hub, fps, stop):
    interval = 1.0 / fps
    deadline = time.monotonic()
    while not stop.is_set():
        for frame in frames:
            if stop.wait(max(0, deadline - time.monotonic())):
                return
            hub.publish(frame)
            deadline += interval
        hub.clear_trails()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('capture', 'timings', 'uart'):
        parser.add_argument(f'--{name}', type=Path, required=True)
    parser.add_argument('--reference', type=Path, default=Path('D:/Code/ldiar'))
    parser.add_argument('--fps', type=float, default=10)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--http-port', type=int, default=8897)
    args = parser.parse_args()
    if not math.isfinite(args.fps) or args.fps <= 0:
        parser.error('--fps must be finite and greater than zero')

    frames, cloud_frames = load_replay(args.capture, args.timings, args.uart, args.reference)
    hub = LiveHub(f'配对回放 · S3 板测 · {args.capture.stem}')
    stop = threading.Event()
    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(hub, cloud_frames=cloud_frames))
    thread = threading.Thread(target=_play, args=(frames, hub, args.fps, stop), daemon=True)
    thread.start()
    print(f'Paired S3 cloud replay: {len(frames)} frames at http://{args.host}:{server.server_port}/', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        thread.join()


if __name__ == '__main__':
    main()
