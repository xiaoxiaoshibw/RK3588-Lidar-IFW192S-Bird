"""Exercise the replay wire protocol against a host build of bird_host."""

import argparse
import json
from pathlib import Path
import struct
import subprocess
import zlib


def packet(command, ordinal=0, count=0, payload=b"", bad_crc=False):
    crc = zlib.crc32(payload) ^ int(bad_crc)
    return struct.pack("<4sIII", command, ordinal, count, crc) + payload


def measurement(cell, x, source=70000):
    return struct.pack("<HfffI", cell, x, 0.0, 0.0, source)


def check_protocol(executable):
    calibration = struct.pack("<ddd", 1.0, 0.0, 0.0) * 49152
    first = measurement(0, 2.0)
    second = measurement(1, 2.0, 70001)
    checks = (
        ("ping", packet(b"PING"), "ready"),
        ("unknown command", packet(b"JUNK"), "error"),
        ("missing calibration", packet(b"BIRD", 0, 1, first), "error"),
        ("corrupt calibration", packet(b"CAL1", 0, 49152, calibration, bad_crc=True), "error"),
        ("calibration retry", packet(b"CAL1", 0, 49152, calibration), "calibrated"),
        ("first frame", packet(b"BIRD", 0, 1, first), "frame"),
        ("corrupt frame", packet(b"BIRD", 1, 1, second, bad_crc=True), "error"),
        ("duplicate cell", packet(b"BIRD", 1, 2, second + second), "error"),
        ("nonfinite measurement", packet(b"BIRD", 1, 1, measurement(1, float("nan"))), "error"),
        ("out-of-grid cell", packet(b"BIRD", 1, 1, measurement(49152, 2.0)), "error"),
        ("future ordinal", packet(b"BIRD", 2, 1, second), "error"),
        ("second frame retry", packet(b"BIRD", 1, 1, second), "frame"),
        ("reset", packet(b"RSET"), "reset"),
        ("first frame after reset", packet(b"BIRD", 0, 1, first), "frame"),
    )
    run = subprocess.run(
        [str(executable.resolve())], input=b"".join(payload for _, payload, _ in checks),
        capture_output=True, check=True, timeout=15,
    )
    replies = [json.loads(line) for line in run.stdout.splitlines()]
    if len(replies) != len(checks):
        raise AssertionError(f"Expected {len(checks)} responses, got {len(replies)}")
    for (name, _, expected), reply in zip(checks, replies, strict=True):
        if reply["type"] != expected:
            raise AssertionError(f"{name}: expected {expected}, got {reply}")
    if replies[0]["protocol"] != 1:
        raise AssertionError(f"Unexpected protocol version: {replies[0]}")
    steps = [(r["ordinal"], r["result"]["warmup_progress"]) for r in replies if r["type"] == "frame"]
    if steps != [(0, "1/10"), (1, "2/10"), (0, "1/10")]:
        raise AssertionError(f"Unexpected state progression after retries and reset: {steps}")
    print(f"Replay protocol passed all {len(checks)} checks.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path, help="Path to the host bird_host executable")
    check_protocol(parser.parse_args().executable)
