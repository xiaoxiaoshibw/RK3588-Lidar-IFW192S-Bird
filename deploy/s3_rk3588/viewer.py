"""Serve fresh ESP32-S3 tracking over HTTP on the RK3588."""

import argparse
import json
import os
from http.server import ThreadingHTTPServer
from pathlib import Path
from queue import Empty
from threading import Event, Thread
from time import monotonic, time
import zlib

from measure_live import CALIBRATION_BYTES, HEADER, RUN, open_link, wait_reply
from s3_live_server import FrameStreamAssembler, LiveHub, make_handler


DEFAULT_PORT = "/dev/ttyACM0"
FIRST_FRAME_TIMEOUT = 18.0
FRAME_TIMEOUT = 3.0


class RealtimeHub(LiveHub):
    def reset_input(self, status):
        with self._lock:
            self._input_status = status
            self._trails.clear()
            self._frame_times.clear()
            self._latest = {
                "seq": None, "state": "UNKNOWN", "locked": 0,
                "tracks": [], "source": self.source_label,
            }
            snapshot = {
                "source": self.source_label, "input_status": status,
                "latest": self._latest, "trails": {},
            }
            # Send the viewer's existing init event to replace its cached frame and trails.
            # The inherited SSE handler supplies the leading data field and trailing blank line.
            status_event = json.dumps({"type": "input_status", "status": status})
            reset_event = status_event + "\n\nevent: init\ndata: " + json.dumps(snapshot)
            for subscriber in self._subscribers:
                while True:
                    try:
                        subscriber.get_nowait()
                    except Empty:
                        break
                subscriber.put_nowait(reset_event)


class DiscardUart:
    def open(self, mode):
        return open(os.devnull, "wb")


def device_identity(device):
    stat = device.stat()
    return stat.st_dev, stat.st_ino, stat.st_ctime_ns


def stream_connection(device, calibration, hub, stop):
    original_device = device_identity(device)
    with open(os.devnull, "w") as replies, open_link(str(device), DiscardUart()) as (link, messages):
        ready = wait_reply(messages, "ready", 10, replies)
        if ready.get("protocol") != 2:
            raise RuntimeError("ESP32-S3 protocol 2 required")
        link.write(HEADER.pack(b"CAL2", 0, len(calibration), zlib.crc32(calibration)))
        for offset in range(0, len(calibration), 8192):
            if stop.is_set():
                return
            link.write(calibration[offset:offset + 8192])
        wait_reply(messages, "calibrated", 20, replies)
        link.write(RUN.pack(b"UDP2", 80_000_000))

        assembler = FrameStreamAssembler()
        streaming = False
        deadline = monotonic() + FIRST_FRAME_TIMEOUT
        while not stop.is_set():
            if device_identity(device) != original_device:
                raise ConnectionError("Serial device was replaced")
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise TimeoutError("ESP32-S3 stopped producing complete frames")
            try:
                row = messages.get(timeout=min(remaining, 0.25))
            except Empty:
                continue
            if not isinstance(row, dict):
                continue
            if row.get("type") in ("ready", "error", "ethernet_done", "summary"):
                raise ConnectionError("ESP32-S3 stream ended ({})".format(row["type"]))
            row["host_wall_ms"] = round(time() * 1000, 3)
            row["host_mono_ms"] = round(monotonic() * 1000, 3)
            for frame in assembler.push_message(row):
                if frame["locked"] != len(frame["tracks"]):
                    continue
                if not streaming:
                    hub.input_status("streaming")
                    streaming = True
                hub.publish(frame)
                deadline = monotonic() + FRAME_TIMEOUT


def follow_device(device, calibration, hub, stop):
    hub.reset_input("waiting")
    while not stop.is_set():
        if device.exists():
            try:
                stream_connection(device, calibration, hub, stop)
            except (OSError, RuntimeError, TypeError, ValueError) as error:
                if hub.snapshot()["input_status"] != "closed":
                    print("ESP32-S3 link unavailable: {}".format(error), flush=True)
                hub.reset_input("closed")
        stop.wait(1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT, type=Path)
    parser.add_argument("--calibration", default=Path(__file__).with_name("calibration_sdk_geometry.bin"), type=Path)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", default=8895, type=int)
    args = parser.parse_args()

    calibration = args.calibration.read_bytes()
    if len(calibration) != CALIBRATION_BYTES:
        raise ValueError("Wrong IFW192S calibration size")

    hub = RealtimeHub("实时 · S3 JSON")
    stop = Event()
    with ThreadingHTTPServer((args.host, args.http_port), make_handler(hub)) as server:
        worker = Thread(target=follow_device, args=(args.port, calibration, hub, stop), daemon=True)
        worker.start()
        print("ESP32-S3 viewer listening on {}:{}".format(args.host, args.http_port), flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()


if __name__ == "__main__":
    main()
