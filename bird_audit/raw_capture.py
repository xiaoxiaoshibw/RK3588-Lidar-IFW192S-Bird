"""Independent Foxglove ROS1 payload recorder and checksummed replay.

This is not a rosbag. The exact subscribed ROS1 message bytes are retained.
Incomplete files lack a footer and are rejected by the reader.
"""
import argparse
import asyncio
import hashlib
import json
import os
import shutil
import struct
import sys
import time
from pathlib import Path

MAGIC = b'BIRDRAW1'
MAX_HEADER = 2 * 1024 * 1024
MAX_MESSAGE = 32 * 1024 * 1024
RECORD = struct.Struct('<QQQI')
DEFAULT_MIN_FREE_MB = 2048


def _take(f, size):
    data = f.read(size)
    if len(data) != size:
        raise ValueError('truncated capture')
    return data


class Writer:
    def __init__(self, path, header, min_free_mb=DEFAULT_MIN_FREE_MB):
        raw = json.dumps(header, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
        if len(raw) > MAX_HEADER:
            raise ValueError('oversized header')
        parent = Path(path).resolve().parent
        free_mb = shutil.disk_usage(parent).free // (1024 * 1024)
        if free_mb < min_free_mb:
            raise ValueError('only %d MiB free on %s; refusing to record below %d MiB'
                             % (free_mb, parent, min_free_mb))
        self.file = open(path, 'xb')
        self.hash = hashlib.sha256()
        self.count = 0
        self._write(MAGIC + struct.pack('<I', len(raw)) + raw)

    def _write(self, data):
        self.file.write(data)
        self.hash.update(data)

    def append(self, payload, log_time_ns, receive_wall_ns, receive_monotonic_ns):
        if not payload or len(payload) > MAX_MESSAGE:
            raise ValueError('invalid payload length')
        self._write(b'R' + RECORD.pack(log_time_ns, receive_wall_ns,
                                     receive_monotonic_ns, len(payload)) + payload)
        self.count += 1

    def finish(self):
        self.file.write(b'F' + struct.pack('<Q', self.count) + self.hash.digest())
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()

    def abort(self):
        self.file.close()


def _scan(path):
    """Validate the complete stream before any frame is offered to callers."""
    with open(path, 'rb') as f:
        h = hashlib.sha256()
        def take(size):
            data = _take(f, size)
            h.update(data)
            return data
        if take(len(MAGIC)) != MAGIC:
            raise ValueError('not a BIRDRAW1 capture')
        n, = struct.unpack('<I', take(4))
        if n > MAX_HEADER:
            raise ValueError('oversized header')
        header = json.loads(take(n).decode('utf-8'))
        count = 0
        while True:
            tag = _take(f, 1)
            if tag == b'F':
                expected, = struct.unpack('<Q', _take(f, 8))
                digest = _take(f, 32)
                if count != expected or digest != h.digest() or f.read(1):
                    raise ValueError('capture checksum, count or trailing bytes invalid')
                return header, count
            if tag != b'R':
                raise ValueError('invalid capture record')
            h.update(tag)
            _, _, _, size = RECORD.unpack(take(RECORD.size))
            if not size or size > MAX_MESSAGE:
                raise ValueError('invalid payload length')
            take(size)
            count += 1


def read_capture(path):
    """Return (header, iterator of raw payload and three timestamps).

    Input must remain unchanged while the iterator is consumed.
    """
    header, _ = _scan(path)
    def records():
        with open(path, 'rb') as f:
            _take(f, len(MAGIC))
            n, = struct.unpack('<I', _take(f, 4))
            _take(f, n)
            while _take(f, 1) == b'R':
                log, wall, mono, size = RECORD.unpack(_take(f, RECORD.size))
                yield {'payload': _take(f, size), 'log_time_ns': log,
                       'receive_wall_ns': wall, 'receive_monotonic_ns': mono}
    return header, records()


def inspect_clouds(path, reference):
    """Summarize original decoded ROS headers; timestamps remain in their own domains."""
    sys.path.insert(0, str(Path(reference) / 'src/human_fall_detection'))
    from core.range_wire import parse_cloud
    _, records = read_capture(path)
    first = last = None
    previous_stamp = None
    previous_seq = None
    backwards = gaps = 0
    layouts = []
    count = 0
    for item in records:
        cloud = parse_cloud(item['payload'])
        stamp = cloud.header.stamp.secs * 1000000000 + cloud.header.stamp.nsecs
        info = {'seq': cloud.header.seq, 'frame_id': cloud.header.frame_id,
                'stamp': [cloud.header.stamp.secs, cloud.header.stamp.nsecs],
                'log_time_ns': item['log_time_ns'], 'receive_wall_ns': item['receive_wall_ns'],
                'receive_monotonic_ns': item['receive_monotonic_ns'],
                'height': cloud.height, 'width': cloud.width,
                'point_step': cloud.point_step, 'row_step': cloud.row_step,
                'data_bytes': len(cloud.data)}
        layout = {'fields': [{'name': f.name, 'offset': f.offset, 'datatype': f.datatype,
                              'count': f.count} for f in cloud.fields],
                  'is_bigendian': cloud.is_bigendian, 'point_step': cloud.point_step,
                  'is_dense': cloud.is_dense}
        if layout not in layouts:
            if len(layouts) >= 128:
                raise ValueError('too many different message layouts')
            layouts.append(layout)
        if previous_stamp is not None and stamp <= previous_stamp:
            backwards += 1
        if previous_seq is not None and cloud.header.seq != (previous_seq + 1) % (2 ** 32):
            gaps += 1
        previous_stamp, previous_seq = stamp, cloud.header.seq
        if first is None:
            first = info
        last = info
        count += 1
    elapsed = ((last['receive_monotonic_ns'] - first['receive_monotonic_ns']) / 1e9
               if count > 1 else 0)
    return {'count': count, 'first': first, 'last': last, 'layouts': layouts,
            'nonincreasing_header_stamps': backwards, 'sequence_discontinuities': gaps,
            'capture_elapsed_s_first_to_last_receive': elapsed,
            'file_bytes_per_capture_second': os.path.getsize(path) / elapsed if elapsed > 0 else None,
            'clock_domains': 'UNKNOWN; no cross-clock subtraction',
            'note': 'File bytes / receive span is observed capture rate, not a disk benchmark. Sequence gaps do not establish where loss occurred.'}


async def record(path, url, seconds, min_free_mb=DEFAULT_MIN_FREE_MB):
    import websockets
    if seconds <= 0:
        raise ValueError('duration must be positive')
    async with websockets.connect(url, subprotocols=['foxglove.websocket.v1'],
                                  max_size=MAX_MESSAGE + 13, max_queue=16,
                                  open_timeout=5, proxy=None) as ws:
        deadline = time.monotonic() + seconds
        writer = None
        channel_id = None
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    message = await asyncio.wait_for(ws.recv(), remaining)
                except asyncio.TimeoutError:
                    break
                wall, mono = time.time_ns(), time.monotonic_ns()
                if isinstance(message, str):
                    event = json.loads(message)
                    if event.get('op') == 'advertise' and writer is None:
                        channel = next((c for c in event.get('channels', [])
                                        if c.get('topic') == '/innolidar_points'), None)
                        if channel:
                            if channel.get('encoding') != 'ros1' or channel.get('schemaName') != 'sensor_msgs/PointCloud2':
                                raise ValueError('unexpected channel schema or encoding')
                            channel_id = channel['id']
                            writer = Writer(path, {'advertise': event, 'channel': channel,
                                                   'url': url, 'started_wall_ns': wall},
                                            min_free_mb=min_free_mb)
                            await ws.send(json.dumps({'op': 'subscribe', 'subscriptions': [
                                {'id': 1, 'channelId': channel_id}]}))
                    elif event.get('op') == 'unadvertise' and channel_id in event.get('channelIds', []):
                        raise ValueError('channel unadvertised; incomplete capture')
                elif len(message) >= 13 and message[0] == 1:
                    subscription, log_time = struct.unpack_from('<IQ', message, 1)
                    if subscription == 1 and writer is not None:
                        writer.append(message[13:], log_time, wall, mono)
            if writer is None or writer.count == 0:
                raise ValueError('no PointCloud2 messages; incomplete capture')
            writer.finish()
        except BaseException:
            if writer is not None and not writer.file.closed:
                writer.abort()
            raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    capture = sub.add_parser('record')
    capture.add_argument('path', help='new file, never overwrites')
    capture.add_argument('--seconds', type=float, required=True)
    capture.add_argument('--url', default='ws://192.168.31.58:8765')
    capture.add_argument('--min-free-mb', type=int, default=DEFAULT_MIN_FREE_MB,
                         help='refuse to record when the target disk has less free space')
    check = sub.add_parser('check')
    check.add_argument('path')
    check.add_argument('--reference', help='reference repository to decode PointCloud2 headers and fields')
    args = p.parse_args()
    if args.command == 'record':
        asyncio.run(record(args.path, args.url, args.seconds, args.min_free_mb))
    header, count = _scan(args.path)
    digest = hashlib.sha256()
    with open(args.path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(chunk)
    result = {'path': str(Path(args.path).resolve()), 'sha256': digest.hexdigest(),
              'messages': count, 'bytes': os.path.getsize(args.path), 'header': header}
    if args.command == 'check' and args.reference:
        result['clouds'] = inspect_clouds(args.path, args.reference)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
