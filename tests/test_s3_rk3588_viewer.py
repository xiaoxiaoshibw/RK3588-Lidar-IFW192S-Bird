import contextlib
from io import StringIO
import json
from pathlib import Path
from queue import Queue
import sys
from tempfile import TemporaryDirectory
from threading import Event
import unittest
from unittest.mock import Mock, patch
import zlib


REPO_ROOT = Path(__file__).resolve().parents[1]
with patch.object(sys, "path", [
    str(REPO_ROOT / "web"), str(REPO_ROOT / "firmware" / "esp32s3_ifw192s"), *sys.path,
]):
    try:
        from deploy.s3_rk3588 import viewer
    except ModuleNotFoundError as error:
        if error.name != "serial":
            raise
        raise unittest.SkipTest("pyserial is required by the RK3588 viewer") from error


class RK3588ViewerTests(unittest.TestCase):
    def test_handshake_and_only_complete_current_frames(self):
        link = Mock()
        messages = Queue()
        for row in (
            {"type": "ready", "protocol": 2},
            {"type": "calibrated"},
            {"type": "frame", "seq": 9, "state": "active", "locked": 1},
            {"type": "track", "seq": 9, "id": 5, "xyz_m": [1, 2, 2], "points": 27},
            {"type": "frame", "seq": 10, "state": "active", "locked": 1},
            {"type": "summary"},
        ):
            messages.put(row)

        calibration = b"0" * 8200
        hub = viewer.RealtimeHub("live")
        with TemporaryDirectory() as folder:
            port = Path(folder) / "serial"
            port.touch()
            connection = contextlib.nullcontext((link, messages))
            with patch.object(viewer, "open_link", return_value=connection):
                with self.assertRaisesRegex(ConnectionError, "summary"):
                    viewer.stream_connection(port, calibration, hub, Event())

        writes = [call.args[0] for call in link.write.call_args_list]
        self.assertEqual(viewer.HEADER.unpack(writes[0]), (b"CAL2", 0, len(calibration), zlib.crc32(calibration)))
        self.assertEqual(writes[1:-1], [calibration[:8192], calibration[8192:]])
        self.assertEqual(viewer.RUN.unpack(writes[-1]), (b"UDP2", 80_000_000))
        snap = hub.snapshot()
        self.assertEqual(snap["input_status"], "streaming")
        self.assertEqual(snap["latest"]["seq"], 9)
        self.assertEqual(snap["latest"]["tracks"][0]["range_m"], 3)
        self.assertIn("host_mono_ms", snap["latest"])

    def test_disconnect_replaces_buffered_frames_and_later_reconnects(self):
        stop = Event()
        hub = viewer.RealtimeHub("live")
        subscriber = hub.subscribe()

        def publish(seq):
            hub.input_status("streaming")
            hub.publish({
                "seq": seq, "state": "active", "locked": 1,
                "tracks": [{"id": 5, "xyz_m": [1, 2, 2], "points": 27}],
            })

        def unplug(*args):
            publish(9)
            raise FileNotFoundError("serial disappeared")

        def replug(*args):
            snap = hub.snapshot()
            self.assertEqual(snap["input_status"], "closed")
            self.assertEqual(snap["latest"]["state"], "UNKNOWN")
            self.assertIsNone(snap["latest"]["seq"])
            self.assertEqual(snap["trails"], {})
            status, resync = subscriber.get_nowait().split("\n\nevent: init\ndata: ", 1)
            self.assertEqual(json.loads(status), {"type": "input_status", "status": "closed"})
            self.assertEqual(json.loads(resync), snap)
            self.assertTrue(subscriber.empty())
            publish(10)
            stop.set()

        callbacks = iter([unplug, replug])
        device = Mock()
        with patch.object(viewer, "stream_connection", side_effect=lambda *args: next(callbacks)(*args)):
            with patch.object(stop, "wait", return_value=False), contextlib.redirect_stdout(StringIO()):
                viewer.follow_device(device, b"", hub, stop)

        self.assertEqual(hub.snapshot()["latest"]["seq"], 10)
        self.assertEqual([pt["seq"] for pt in hub.snapshot()["trails"]["5"]], [10])

    def test_detects_silent_stream_after_handshake(self):
        messages = Queue()
        messages.put({"type": "ready", "protocol": 2})
        messages.put({"type": "calibrated"})
        with TemporaryDirectory() as folder:
            port = Path(folder) / "serial"
            port.touch()
            connection = contextlib.nullcontext((Mock(), messages))
            with patch.object(viewer, "open_link", return_value=connection):
                with patch.object(viewer, "FIRST_FRAME_TIMEOUT", 0):
                    with self.assertRaisesRegex(TimeoutError, "stopped producing complete frames"):
                        viewer.stream_connection(port, b"", viewer.RealtimeHub("live"), Event())


if __name__ == "__main__":
    unittest.main()
