"""Serve ESP32-S3 IFW192S JSON tracking output over HTTP/SSE for live 2D/3D display."""
import argparse
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import sys
import threading
import time

HTML_PATH = Path(__file__).with_name("s3_live.html")
MAX_TRAIL_POINTS = 120


def _compute_range_m(xyz):
    if not isinstance(xyz, list) or len(xyz) != 3:
        return None
    x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    return round(math.sqrt(x * x + y * y + z * z), 4)


class FrameStreamAssembler:
    """Group S3 `frame` and `track` JSON lines into atomic frame snapshots."""

    def __init__(self):
        self._pending = None
        self._expected_tracks = 0

    def _finalize(self):
        frame = self._pending
        self._pending = None
        self._expected_tracks = 0
        return frame

    def push_message(self, msg):
        """Consume one parsed JSON dict; return a list of 0..2 completed frame snapshots."""
        if not isinstance(msg, dict):
            return []
        msg_type = msg.get("type")
        completed = []
        if msg_type == "frame":
            if self._pending is not None:
                completed.append(self._finalize())
            state = str(msg.get("state", "UNKNOWN"))
            locked = int(msg.get("locked", 0)) if state == "active" else 0
            tail_us = msg.get("tail_read_us")
            grid_us = msg.get("grid_ready_us")
            start_us = msg.get("tracker_start_us")
            ready_us = msg.get("result_ready_us")
            latency_ms = None
            tracker_ms = None
            materialize_ms = None
            if isinstance(tail_us, (int, float)) and isinstance(ready_us, (int, float)):
                latency_ms = round((ready_us - tail_us) / 1000.0, 3)
            if isinstance(start_us, (int, float)) and isinstance(ready_us, (int, float)):
                tracker_ms = round((ready_us - start_us) / 1000.0, 3)
            if isinstance(tail_us, (int, float)) and isinstance(grid_us, (int, float)):
                materialize_ms = round((grid_us - tail_us) / 1000.0, 3)
            snapshot = {
                "seq": msg.get("seq"),
                "state": state,
                "warmup": int(msg.get("warmup", 0)),
                "locked": locked,
                "candidates": int(msg.get("candidates", 0)),
                "tentative": int(msg.get("tentative", 0)),
                "records": msg.get("records"),
                "latency_ms": latency_ms,
                "tracker_ms": tracker_ms,
                "materialize_ms": materialize_ms,
                "tracks": [],
            }
            if "host_wall_ms" in msg:
                snapshot["host_wall_ms"] = msg["host_wall_ms"]
            if "host_mono_ms" in msg:
                snapshot["host_mono_ms"] = msg["host_mono_ms"]
            if state == "active" and locked > 0:
                self._pending = snapshot
                self._expected_tracks = locked
            else:
                completed.append(snapshot)
            return completed

        if msg_type == "track" and self._pending is not None:
            if msg.get("seq") == self._pending.get("seq"):
                xyz = msg.get("xyz_m")
                self._pending["tracks"].append({
                    "id": int(msg.get("id", 0)),
                    "box": msg.get("box", [0, 0, 0, 0]),
                    "xyz_m": xyz,
                    "points": int(msg.get("points", 0)),
                    "range_m": _compute_range_m(xyz),
                })
                if len(self._pending["tracks"]) >= self._expected_tracks:
                    completed.append(self._finalize())
            return completed

        if msg_type in ("summary", "ethernet_done", "error") and self._pending is not None:
            completed.append(self._finalize())
        return completed

    def flush(self):
        return [self._finalize()] if self._pending is not None else []


class LiveHub:
    """Thread-safe hub storing latest frame, per-track trails, and SSE subscribers."""

    def __init__(self, source_label):
        self.source_label = source_label
        self._lock = threading.Lock()
        self._latest = None
        self._input_status = None
        self._trails = {}
        self._subscribers = set()
        self._frame_times = deque(maxlen=30)

    def publish(self, frame):
        now = time.monotonic()
        with self._lock:
            self._frame_times.append(now)
            fps = 0.0
            if len(self._frame_times) >= 2:
                span = self._frame_times[-1] - self._frame_times[0]
                if span > 0:
                    fps = round((len(self._frame_times) - 1) / span, 2)
            payload = dict(frame)
            payload["fps"] = fps
            payload["source"] = self.source_label
            if payload.get("state") == "active":
                for track in payload.get("tracks", []):
                    tid = str(track["id"])
                    trail = self._trails.setdefault(tid, deque(maxlen=MAX_TRAIL_POINTS))
                    xyz = track.get("xyz_m")
                    if isinstance(xyz, list) and len(xyz) == 3:
                        trail.append({
                            "seq": payload.get("seq"),
                            "xyz_m": xyz,
                            "range_m": track.get("range_m"),
                            "points": track.get("points"),
                        })
            self._latest = payload
            self._send_locked(payload)

    def _send_locked(self, payload):
        encoded = json.dumps(payload, separators=(",", ":"))
        for queue in self._subscribers:
            try:
                queue.put_nowait(encoded)
            except Full:
                try:
                    queue.get_nowait()
                except Empty:
                    pass
                queue.put_nowait(encoded)

    def input_status(self, status):
        with self._lock:
            self._input_status = status
            self._send_locked({"type": "input_status", "status": status})

    def clear_trails(self):
        with self._lock:
            self._trails.clear()

    def snapshot(self):
        with self._lock:
            return {
                "source": self.source_label,
                "input_status": self._input_status,
                "latest": self._latest,
                "trails": {tid: list(pts) for tid, pts in self._trails.items()},
            }

    def subscribe(self):
        queue = Queue(maxsize=32)
        with self._lock:
            self._subscribers.add(queue)
        return queue

    def unsubscribe(self, queue):
        with self._lock:
            self._subscribers.discard(queue)


def feed_jsonl_replay(path, hub, fps, loop, stop_event):
    interval = 1.0 / max(fps, 0.1)
    while not stop_event.is_set():
        assembler = FrameStreamAssembler()
        with Path(path).open("r", encoding="utf-8") as stream:
            for line in stream:
                if stop_event.is_set():
                    return
                line = line.strip()
                if not line.startswith("{"):
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue
                for frame in assembler.push_message(msg):
                    hub.publish(frame)
                    if stop_event.wait(interval):
                        return
        for frame in assembler.flush():
            hub.publish(frame)
        if not loop:
            return
        hub.clear_trails()


def feed_serial(port, baud, hub, stop_event):
    import serial  # Deferred so --replay mode works without pyserial.

    assembler = FrameStreamAssembler()
    while not stop_event.is_set():
        try:
            with serial.Serial(port=port, baudrate=baud, timeout=0.25) as link:
                pending = bytearray()
                while not stop_event.is_set():
                    line = link.readline()
                    if not line:
                        continue
                    pending.extend(line)
                    if not line.endswith(b"\n"):
                        continue
                    raw = bytes(pending).strip()
                    pending.clear()
                    if not raw.startswith(b"{"):
                        continue
                    try:
                        msg = json.loads(raw.decode("utf-8", errors="ignore"))
                    except json.JSONDecodeError:
                        continue
                    for frame in assembler.push_message(msg):
                        hub.publish(frame)
        except Exception:
            if stop_event.wait(1.0):
                return


def feed_jsonl_live(stream, hub):
    """Forward the current collector's NDJSON without re-timing or looping it."""
    assembler = FrameStreamAssembler()
    hub.input_status("waiting")
    status = "closed"
    started = False
    for line in stream:
        try:
            message = json.loads(line)
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if not isinstance(message, dict):
            continue
        if message.get("type") == "frame" and not started:
            hub.input_status("streaming")
            started = True
        for frame in assembler.push_message(message):
            hub.publish(frame)
        if message.get("type") in ("summary", "error"):
            status = "finished" if message["type"] == "summary" else "error"
            break
    for frame in assembler.flush():
        hub.publish(frame)
    hub.input_status(status)


def make_handler(hub, *, cloud_frames=None):
    html_bytes = HTML_PATH.read_bytes()
    clouds = {f"/cloud/{seq}": data for seq, data in (cloud_frames or {}).items()}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def handle(self):
            try:
                super().handle()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(html_bytes)))
                self.end_headers()
                self.wfile.write(html_bytes)
                return

            if self.path == "/state":
                body = json.dumps(hub.snapshot(), separators=(",", ":")).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path in clouds:
                body = clouds[self.path]
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/events":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                queue = hub.subscribe()
                try:
                    snap = hub.snapshot()
                    init_line = f"event: init\ndata: {json.dumps(snap, separators=(',', ':'))}\n\n"
                    self.wfile.write(init_line.encode("utf-8"))
                    self.wfile.flush()
                    while True:
                        try:
                            encoded = queue.get(timeout=15.0)
                            self.wfile.write(f"data: {encoded}\n\n".encode("utf-8"))
                        except Empty:
                            self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    hub.unsubscribe(queue)
                return

            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            if self.path == "/clear-trails":
                hub.clear_trails()
                body = b'{"ok":true}'
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_error(HTTPStatus.NOT_FOUND)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Serial device path (e.g. /dev/ttyACM4 or COM5)")
    source.add_argument("--replay", type=Path, help="Replay an S3 live_w5500_timing.jsonl file")
    source.add_argument("--stdin", action="store_true", help="Read fresh S3 JSON lines from the capture process")
    parser.add_argument("--baud", type=int, default=2_000_000, help="Serial baud rate")
    parser.add_argument("--fps", type=float, default=9.6, help="Replay rate in frames per second")
    parser.add_argument("--loop", action="store_true", help="Loop replay continuously")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind address")
    parser.add_argument("--http-port", type=int, default=8898, help="HTTP port")
    args = parser.parse_args()

    if args.stdin:
        label = "实时 · S3 JSON"
    else:
        label = f"serial:{args.port}" if args.port else f"replay:{args.replay.name}"
    hub = LiveHub(label)
    stop_event = threading.Event()
    if args.port:
        worker = threading.Thread(
            target=feed_serial, args=(args.port, args.baud, hub, stop_event), daemon=True,
        )
    elif args.stdin:
        worker = threading.Thread(target=feed_jsonl_live, args=(sys.stdin.buffer, hub), daemon=True)
    else:
        worker = threading.Thread(
            target=feed_jsonl_replay, args=(args.replay, hub, args.fps, args.loop, stop_event), daemon=True,
        )
    worker.start()

    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(hub))
    print(f"S3 live viewer listening on http://{args.host}:{args.http_port}/ ({label})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.server_close()


if __name__ == "__main__":
    main()
