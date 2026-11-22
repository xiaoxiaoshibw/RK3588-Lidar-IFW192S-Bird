"""Configure the IFW192S timing probe on an ESP32-S3 and retain all USB replies."""
import argparse
from contextlib import contextmanager
import json
from pathlib import Path
from queue import Empty, Queue
import struct
from threading import Event, Thread
from time import monotonic, sleep, time as wall_time
import zlib

import serial

HEADER = struct.Struct("<4sIII")
RUN = struct.Struct("<4sI")
PACKET_BYTES = 1104
CALIBRATION_BYTES = 16 + 49152 * 16


def receive_lines(link, stream, messages, stopped):
    pending = bytearray()
    while not stopped.is_set():
        line = link.readline()
        if not line:
            continue
        stream.write(line)
        stream.flush()
        pending.extend(line)
        if not line.endswith(b"\n"):
            continue
        if pending.lstrip().startswith(b"{"):
            try:
                messages.put(json.loads(pending))
            except (UnicodeDecodeError, json.JSONDecodeError):
                messages.put({"type": "error", "message": "Invalid JSON reply"})
        pending.clear()


@contextmanager
def open_link(device, uart_log):
    with uart_log.open("xb") as stream, serial.Serial(
        baudrate=2_000_000, timeout=0.25, write_timeout=8, exclusive=True,
    ) as link:
        link.port = device
        link.dtr = link.rts = False
        link.open()
        link.reset_input_buffer()
        link.rts = True
        sleep(0.1)
        link.rts = False
        messages = Queue()
        stopped = Event()
        reader = Thread(target=receive_lines, args=(link, stream, messages, stopped))
        reader.start()
        try:
            yield link, messages
        finally:
            stopped.set()
            reader.join()


def write_row(rows, row, stamp_host_time):
    if stamp_host_time and isinstance(row, dict):
        row = dict(row)
        row["host_wall_ms"] = round(wall_time() * 1000.0, 3)
        row["host_mono_ms"] = round(monotonic() * 1000.0, 3)
    rows.write(json.dumps(row) + "\n")
    rows.flush()


def wait_reply(messages, name, timeout, rows, stamp_host_time=False):
    deadline = monotonic() + timeout
    while True:
        try:
            row = messages.get(timeout=max(0.0, deadline - monotonic()))
        except Empty as error:
            raise TimeoutError("No {} reply from the S3".format(name)) from error
        write_row(rows, row, stamp_host_time)
        if row.get("type") == "error":
            raise RuntimeError(row)
        if row.get("type") == name:
            return row


def timings(rows, start_key):
    ordered = sorted((row["result_ready_us"] - row[start_key], row["seq"]) for row in rows)
    if not ordered:
        return None

    def percentile(quantile):
        index = (len(ordered) - 1) * quantile
        lower = int(index)
        floor = ordered[lower][0]
        ceil = ordered[min(lower + 1, len(ordered) - 1)][0]
        return (floor + (ceil - floor) * (index - lower)) / 1000

    return {
        "frames": len(ordered), "p50_ms": percentile(0.5), "p95_ms": percentile(0.95),
        "max_ms": ordered[-1][0] / 1000, "max_seq": ordered[-1][1],
        "over_100_ms": sum(value > 100000 for value, _ in ordered),
    }


def run(args):
    calibration = args.calibration.read_bytes()
    if len(calibration) != CALIBRATION_BYTES:
        raise ValueError("Wrong IFW192S calibration size")
    rows_out = args.output.with_suffix(".jsonl")
    raw_out = args.output.with_suffix(".uart.log")
    summary_out = args.output.with_suffix(".json")
    if any(path.exists() for path in (rows_out, raw_out, summary_out)):
        raise FileExistsError("Choose a new output prefix")
    frame_rows = []
    stamp_host_time = bool(getattr(args, "stamp_host_time", False))
    with rows_out.open("x", encoding="utf-8") as rows, open_link(args.port, raw_out) as (link, messages):
        ready = wait_reply(messages, "ready", 10, rows, stamp_host_time=stamp_host_time)
        if ready.get("protocol") != 2:
            raise RuntimeError("Unexpected timing probe protocol")
        link.write(HEADER.pack(b"CAL2", 0, len(calibration), zlib.crc32(calibration)))
        for offset in range(0, len(calibration), 8192):
            try:
                link.write(calibration[offset:offset + 8192])
            except serial.SerialTimeoutException as error:
                raise TimeoutError("Calibration write stalled after {} bytes".format(offset)) from error
        memory = wait_reply(messages, "calibrated", 20, rows, stamp_host_time=stamp_host_time)

        if args.payloads:
            size = args.payloads.stat().st_size
            if size <= 0 or size % PACKET_BYTES or size // PACKET_BYTES > 200_000:
                raise ValueError("Expected a stream of complete 1104-byte datagrams")
            link.write(RUN.pack(b"USB2", size // PACKET_BYTES))
            with args.payloads.open("rb") as packets:
                while chunk := packets.read(PACKET_BYTES * 16):
                    link.write(chunk)
        else:
            link.write(RUN.pack(b"UDP2", args.udp * 1_000_000))

        deadline = monotonic() + (25 if args.payloads else 65)
        while True:
            try:
                reply = messages.get(timeout=max(0.0, deadline - monotonic()))
            except Empty as error:
                raise TimeoutError("No summary from the S3") from error
            write_row(rows, reply, stamp_host_time)
            if reply["type"] == "error":
                raise RuntimeError(reply)
            if reply["type"] == "frame":
                frame_rows.append(reply)
            if reply["type"] == "summary":
                break
    active = [row for row in frame_rows if row["state"] == "active"]
    summary = {
        "transport": "uart_pcap" if args.payloads else "live_w5500",
        "profile": ready["profile"], "memory": memory, "ingress": reply,
        "warmup_frames": sum(row["state"] == "warmup" for row in frame_rows),
        "active_frames": len(active),
        "locked_frames": sum(row["locked"] > 0 for row in active),
        "output_unknown_frames": sum(row["state"] == "UNKNOWN" for row in frame_rows),
        "tail_read_to_result_ready": timings(active, "tail_read_us"),
        "grid_ready_to_result_ready": timings(active, "grid_ready_us"),
        "grid_tracker": timings(active, "tracker_start_us"),
    }
    with summary_out.open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)
        stream.write("\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--payloads", type=Path)
    modes.add_argument("--udp", type=int, choices=(40, 80), help="SPI clock in MHz")
    parser.add_argument(
        "--stamp-host-time",
        action="store_true",
        help="Attach host_wall_ms and host_mono_ms to each JSONL record for multi-sensor sync",
    )
    run(parser.parse_args())
