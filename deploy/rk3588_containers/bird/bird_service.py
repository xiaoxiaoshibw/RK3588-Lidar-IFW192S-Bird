"""RK3588 native C++17 bird recognition service subscribing read-only to Foxglove."""
import argparse
from http import HTTPStatus
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
from queue import Empty
import signal
import sys
import threading
import time

# Support running both inside Docker (/app) and directly from repo root.
_HERE = Path(__file__).resolve().parent
_CANDIDATES = [_HERE]
if len(_HERE.parents) > 2:
    _CANDIDATES.extend((_HERE.parents[2] / "web", _HERE.parents[2]))
for _candidate in _CANDIDATES:
    if _candidate.is_dir() and str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from foxglove_client import FoxgloveError, stream_pointcloud2
from native_tracker import NativeBirdTracker
from pointcloud_grid import IFW192SScanGrid
from s3_live_server import LiveHub, make_handler

DEFAULT_FOXGLOVE_URL = "ws://127.0.0.1:8765"
DEFAULT_TOPIC = "/innolidar_points"
DEFAULT_HTTP_PORT = 8895
DEFAULT_LIB_PATH = _HERE / ("libbird_tracker.dll" if os.name == "nt" else "libbird_tracker.so")


class LatestSlot:
    """Single-slot drop-old buffer so recognition never builds lag behind Foxglove."""

    def __init__(self):
        self._cond = threading.Condition()
        self._item = None
        self._closed = False
        self.received = 0
        self.overwritten = 0

    def put(self, item):
        with self._cond:
            self.received += 1
            if self._item is not None:
                self.overwritten += 1
            self._item = item
            self._cond.notify()

    def take(self, timeout=0.25):
        with self._cond:
            if self._item is None and not self._closed:
                self._cond.wait(timeout=timeout)
            item = self._item
            self._item = None
            return item

    def clear(self):
        with self._cond:
            self._item = None

    def close(self):
        with self._cond:
            self._closed = True
            self._cond.notify_all()


class BirdRealtimeHub(LiveHub):
    """LiveHub extended with atomic UNKNOWN reset, background reset, and /healthz metrics."""

    def __init__(self, source_label, *, foxglove_url, topic, calibration_sha256):
        super().__init__(source_label)
        self._foxglove_url = foxglove_url
        self._topic = topic
        self._calibration_sha256 = calibration_sha256
        self._started_wall_s = round(time.time(), 3)
        self._reset_bg_event = threading.Event()
        self._metrics = {
            "received": 0,
            "processed": 0,
            "overwritten": 0,
            "seq_gaps": 0,
            "errors": 0,
            "last_error": None,
            "last_decode_ms": None,
            "last_tracker_ms": None,
            "last_total_ms": None,
            "last_valid_cells": None,
            "last_frame_mono_s": None,
        }

    def request_background_reset(self):
        self._reset_bg_event.set()
        self.clear_trails()

    def consume_background_reset(self):
        if self._reset_bg_event.is_set():
            self._reset_bg_event.clear()
            return True
        return False

    def update_metrics(self, **kwargs):
        with self._lock:
            self._metrics.update(kwargs)

    def reset_input(self, status, *, error=None):
        with self._lock:
            self._input_status = status
            self._trails.clear()
            self._frame_times.clear()
            if error is not None:
                self._metrics["errors"] += 1
                self._metrics["last_error"] = str(error)
            self._latest = {
                "seq": None,
                "state": "UNKNOWN",
                "warmup": 0,
                "locked": 0,
                "candidates": 0,
                "tentative": 0,
                "tracks": [],
                "source": self.source_label,
            }
            snapshot = {
                "source": self.source_label,
                "input_status": status,
                "latest": self._latest,
                "trails": {},
            }
            status_event = json.dumps({"type": "input_status", "status": status}, separators=(",", ":"))
            reset_event = status_event + "\n\nevent: init\ndata: " + json.dumps(snapshot, separators=(",", ":"))
            for subscriber in self._subscribers:
                while True:
                    try:
                        subscriber.get_nowait()
                    except Empty:
                        break
                subscriber.put_nowait(reset_event)

    def health(self):
        now_mono = time.monotonic()
        with self._lock:
            latest = dict(self._latest) if isinstance(self._latest, dict) else None
            metrics = dict(self._metrics)
            last_mono = metrics.pop("last_frame_mono_s", None)
            frame_age_ms = round((now_mono - last_mono) * 1000.0, 1) if last_mono is not None else None
            return {
                "ok": True,
                "service": "lidar-bird-rk3588",
                "runtime": "rk3588-native-cpp17",
                "source": self.source_label,
                "foxglove_url": self._foxglove_url,
                "topic": self._topic,
                "input_status": self._input_status or "waiting",
                "calibration_sha256": self._calibration_sha256,
                "started_wall_s": self._started_wall_s,
                "frame_age_ms": frame_age_ms,
                "state": latest.get("state") if latest else "UNKNOWN",
                "seq": latest.get("seq") if latest else None,
                "warmup": latest.get("warmup") if latest else 0,
                "locked": latest.get("locked") if latest else 0,
                "fps": latest.get("fps") if latest else 0.0,
                "semantic_truth": "UNKNOWN",
                "metrics": metrics,
            }


def make_bird_handler(hub):
    """Wrap s3_live_server handler with /healthz and POST /reset-background."""
    BaseHandler = make_handler(hub)

    class BirdHandler(BaseHandler):
        def do_GET(self):
            if self.path == "/healthz":
                body = json.dumps(hub.health(), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            super().do_GET()

        def do_POST(self):
            if self.path == "/reset-background":
                hub.request_background_reset()
                body = b'{"ok":true,"action":"reset_background"}'
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            super().do_POST()

    return BirdHandler


def load_scan_grid(calibration=None, azimuth_csv=None, elevation_csv=None):
    if calibration:
        return IFW192SScanGrid.from_calibration_bin(calibration)
    if azimuth_csv and elevation_csv:
        return IFW192SScanGrid.from_csv(azimuth_csv, elevation_csv)
    raise ValueError("Provide either --calibration <file.bin> or both --azimuth and --elevation CSV files")


def process_cloud_payload(raw_payload, grid, tracker, *, recv_wall_ms=None, recv_mono_ms=None,
                          previous_seq=None, include_provenance=False):
    """Decode one ROS1 PointCloud2 frame and advance the native C++17 bird tracker."""
    cloud, points, presence, decode_stats = grid.decode_cloud_bytes(raw_payload)
    seq = int(cloud.header.seq)
    seq_gap = previous_seq is not None and seq != (previous_seq + 1) % (2 ** 32)
    if seq_gap:
        tracker.clear_tracks()

    step_res = tracker.step(points, presence, include_provenance=include_provenance)
    if not step_res["ok"]:
        tracker.clear_tracks()

    decode_ms = decode_stats["decode_ms"]
    tracker_ms = step_res["tracker_ms"]
    total_ms = round(decode_ms + tracker_ms, 3)

    frame = {
        "seq": seq,
        "state": step_res["state"],
        "warmup": step_res["warmup"],
        "locked": step_res["locked"],
        "candidates": step_res["candidates"],
        "tentative": step_res["tentative"],
        "records": decode_stats["valid_cells"],
        "materialize_ms": decode_ms,
        "tracker_ms": tracker_ms,
        "latency_ms": total_ms,
        "tracks": step_res["tracks"],
    }
    if recv_wall_ms is not None:
        frame["host_wall_ms"] = recv_wall_ms
    if recv_mono_ms is not None:
        frame["host_mono_ms"] = recv_mono_ms
    return frame, seq, seq_gap, decode_stats


def _worker_loop(slot, grid, tracker, hub, stop_event, *, include_provenance=False):
    previous_seq = None
    processed = 0
    seq_gaps = 0
    streaming = False

    while not stop_event.is_set():
        item = slot.take(timeout=0.25)
        if item is None:
            if hub.consume_background_reset():
                tracker.reset()
                previous_seq = None
            continue

        if item == "RESET_TRACKS":
            tracker.clear_tracks()
            previous_seq = None
            streaming = False
            continue

        _log_time_ns, recv_wall_ms, recv_mono_ms, raw_payload = item
        if hub.consume_background_reset():
            tracker.reset()
            previous_seq = None

        try:
            frame, seq, had_gap, decode_stats = process_cloud_payload(
                raw_payload,
                grid,
                tracker,
                recv_wall_ms=recv_wall_ms,
                recv_mono_ms=recv_mono_ms,
                previous_seq=previous_seq,
                include_provenance=include_provenance,
            )
            previous_seq = seq
            processed += 1
            if had_gap:
                seq_gaps += 1
            if not streaming:
                hub.input_status("streaming")
                streaming = True
            hub.update_metrics(
                received=slot.received,
                overwritten=slot.overwritten,
                processed=processed,
                seq_gaps=seq_gaps,
                last_decode_ms=decode_stats["decode_ms"],
                last_tracker_ms=frame["tracker_ms"],
                last_total_ms=frame["latency_ms"],
                last_valid_cells=decode_stats["valid_cells"],
                last_frame_mono_s=time.monotonic(),
            )
            hub.publish(frame)
        except Exception as exc:
            tracker.clear_tracks()
            previous_seq = None
            hub.reset_input("error", error=exc)
            streaming = False


def _foxglove_reader_loop(url, topic, slot, hub, stop_event, *,
                          first_frame_timeout=15.0, frame_timeout=3.0, reconnect_delay=1.5):
    hub.reset_input("waiting")
    while not stop_event.is_set():
        try:
            for msg in stream_pointcloud2(
                url,
                topic,
                stop_event,
                first_frame_timeout=first_frame_timeout,
                frame_timeout=frame_timeout,
            ):
                slot.put(msg)
            if stop_event.is_set():
                return
        except (OSError, TimeoutError, FoxgloveError, ValueError) as exc:
            if stop_event.is_set():
                return
            print(f"Foxglove stream unavailable ({url} {topic}): {exc}", flush=True)
            slot.clear()
            slot.put("RESET_TRACKS")
            hub.reset_input("closed", error=exc)
        stop_event.wait(reconnect_delay)


def _birdraw_replay_loop(capture_path, slot, hub, stop_event, *, fps=9.6, loop=False):
    from bird_audit.raw_capture import read_capture

    interval = 1.0 / max(fps, 0.1)
    hub.reset_input("waiting")
    while not stop_event.is_set():
        _, records = read_capture(str(capture_path))
        for rec in records:
            if stop_event.is_set():
                return
            wall_ms = round(time.time() * 1000.0, 3)
            mono_ms = round(time.monotonic() * 1000.0, 3)
            slot.put((rec["log_time_ns"], wall_ms, mono_ms, rec["payload"]))
            if stop_event.wait(interval):
                return
        if not loop:
            return
        slot.put("RESET_TRACKS")
        hub.request_background_reset()


def _env_path(name):
    val = os.environ.get(name, "").strip()
    return Path(val) if val else None


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--foxglove-url", default=os.environ.get("BIRD_FOXGLOVE_URL", DEFAULT_FOXGLOVE_URL))
    parser.add_argument("--topic", default=os.environ.get("BIRD_INPUT_TOPIC", DEFAULT_TOPIC))
    parser.add_argument("--calibration", type=Path, default=_env_path("BIRD_CALIBRATION"))
    parser.add_argument("--azimuth", type=Path, default=_env_path("BIRD_AZIMUTH_CSV"))
    parser.add_argument("--elevation", type=Path, default=_env_path("BIRD_ELEVATION_CSV"))
    parser.add_argument("--lib", type=Path, default=_env_path("BIRD_TRACKER_LIB") or DEFAULT_LIB_PATH)
    parser.add_argument("--host", default=os.environ.get("BIRD_HTTP_HOST", "0.0.0.0"))
    parser.add_argument("--http-port", type=int, default=int(os.environ.get("BIRD_HTTP_PORT", str(DEFAULT_HTTP_PORT))))
    parser.add_argument("--first-frame-timeout", type=float, default=float(os.environ.get("BIRD_FIRST_FRAME_TIMEOUT", "15.0")))
    parser.add_argument("--frame-timeout", type=float, default=float(os.environ.get("BIRD_FRAME_TIMEOUT", "3.0")))
    parser.add_argument("--replay-birdraw", type=Path, default=None, help="Offline BIRDRAW replay instead of Foxglove")
    parser.add_argument("--fps", type=float, default=9.6)
    parser.add_argument("--loop", action="store_true")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    grid = load_scan_grid(args.calibration, args.azimuth, args.elevation)
    source_label = (
        f"回放 · {args.replay_birdraw.name}"
        if args.replay_birdraw
        else "实时 · RK3588 C++17 (Foxglove)"
    )
    hub = BirdRealtimeHub(
        source_label,
        foxglove_url=str(args.replay_birdraw) if args.replay_birdraw else args.foxglove_url,
        topic=args.topic,
        calibration_sha256=grid.sha256,
    )

    slot = LatestSlot()
    stop_event = threading.Event()

    def handle_signal(signum, _frame):
        stop_event.set()
        slot.close()

    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, handle_signal)
        signal.signal(signal.SIGINT, handle_signal)

    with NativeBirdTracker(args.lib, grid.unit_rays) as tracker:
        worker = threading.Thread(
            target=_worker_loop,
            args=(slot, grid, tracker, hub, stop_event),
            daemon=True,
        )
        worker.start()

        if args.replay_birdraw:
            reader = threading.Thread(
                target=_birdraw_replay_loop,
                args=(args.replay_birdraw, slot, hub, stop_event),
                kwargs={"fps": args.fps, "loop": args.loop},
                daemon=True,
            )
        else:
            reader = threading.Thread(
                target=_foxglove_reader_loop,
                args=(args.foxglove_url, args.topic, slot, hub, stop_event),
                kwargs={
                    "first_frame_timeout": args.first_frame_timeout,
                    "frame_timeout": args.frame_timeout,
                },
                daemon=True,
            )
        reader.start()

        server = ThreadingHTTPServer((args.host, args.http_port), make_bird_handler(hub))
        server.timeout = 0.5
        print(
            f"RK3588 bird service listening on http://{args.host}:{args.http_port}/ "
            f"(source={source_label}, cal_sha={grid.sha256[:12]})",
            flush=True,
        )
        try:
            while not stop_event.is_set():
                server.handle_request()
        except KeyboardInterrupt:
            pass
        finally:
            stop_event.set()
            slot.close()
            server.server_close()
            reader.join(timeout=2.0)
            worker.join(timeout=2.0)


if __name__ == "__main__":
    main()
