import http.client
from io import BytesIO
import json
from pathlib import Path
import threading
import unittest
from http.server import ThreadingHTTPServer

from web.s3_live_server import FrameStreamAssembler, LiveHub, feed_jsonl_live, make_handler


class TestS3LiveServer(unittest.TestCase):
    def test_assembler_groups_tracks_and_computes_metrics(self):
        asm = FrameStreamAssembler()
        self.assertEqual(asm.push_message({
            "type": "frame", "seq": 1000, "state": "active", "warmup": 0,
            "locked": 2, "candidates": 2, "tentative": 0,
            "tail_read_us": 10_000, "grid_ready_us": 40_000,
            "tracker_start_us": 40_005, "result_ready_us": 55_005,
        }), [])
        self.assertEqual(asm.push_message({
            "type": "track", "seq": 1000, "id": 1, "box": [10, 20, 15, 25],
            "xyz_m": [3.0, 4.0, 0.0], "points": 20,
        }), [])
        done = asm.push_message({
            "type": "track", "seq": 1000, "id": 2, "box": [30, 40, 35, 45],
            "xyz_m": [0.0, 0.0, 2.0], "points": 14,
        })
        self.assertEqual(len(done), 1)
        frame = done[0]
        self.assertEqual(frame["seq"], 1000)
        self.assertEqual(frame["state"], "active")
        self.assertAlmostEqual(frame["latency_ms"], 45.005)
        self.assertAlmostEqual(frame["tracker_ms"], 15.0)
        self.assertAlmostEqual(frame["materialize_ms"], 30.0)
        self.assertEqual(len(frame["tracks"]), 2)
        self.assertAlmostEqual(frame["tracks"][0]["range_m"], 5.0)
        self.assertAlmostEqual(frame["tracks"][1]["range_m"], 2.0)

    def test_assembler_preserves_unknown_without_tracks(self):
        asm = FrameStreamAssembler()
        done = asm.push_message({"type": "frame", "seq": 1768, "state": "UNKNOWN", "records": 1234})
        self.assertEqual(len(done), 1)
        self.assertEqual(done[0]["state"], "UNKNOWN")
        self.assertEqual(done[0]["locked"], 0)
        self.assertEqual(done[0]["tracks"], [])

    def test_http_endpoints_and_trail_clearing(self):
        hub = LiveHub("test")
        hub.publish({
            "seq": 768, "state": "active", "warmup": 0, "locked": 1,
            "candidates": 1, "tentative": 0, "latency_ms": 48.0, "tracker_ms": 15.0,
            "tracks": [{"id": 7, "box": [1, 2, 3, 4], "xyz_m": [2.0, 0.1, 0.5], "points": 18, "range_m": 2.064}],
        })
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(hub))
        port = server.server_address[1]
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
            conn.request("GET", "/")
            resp = conn.getresponse()
            body = resp.read()
            self.assertEqual(resp.status, 200)
            self.assertIn(b"ESP32-S3", body)

            conn.request("GET", "/state")
            resp = conn.getresponse()
            snap = json.loads(resp.read().decode("utf-8"))
            self.assertEqual(snap["latest"]["seq"], 768)
            self.assertIn("7", snap["trails"])

            conn.request("POST", "/clear-trails")
            resp = conn.getresponse()
            self.assertEqual(resp.status, 200)
            resp.read()

            conn.request("GET", "/state")
            resp = conn.getresponse()
            snap = json.loads(resp.read().decode("utf-8"))
            self.assertEqual(snap["trails"], {})
            conn.close()
        finally:
            server.shutdown()
            server.server_close()

    def test_cloud_binary_serves_only_the_requested_frame(self):
        cloud = bytes.fromhex("0000803f0000004000004040")
        with ThreadingHTTPServer(("127.0.0.1", 0), make_handler(
            LiveHub("paired replay"), cloud_frames={42: cloud}
        )) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            conn = http.client.HTTPConnection("127.0.0.1", server.server_address[1], timeout=5)
            try:
                conn.request("GET", "/cloud/42")
                response = conn.getresponse()
                self.assertEqual(response.status, 200)
                self.assertEqual(response.getheader("Content-Type"), "application/octet-stream")
                self.assertEqual(response.getheader("Cache-Control"), "no-store")
                self.assertEqual(response.read(), cloud)

                conn.request("GET", "/cloud/43")
                response = conn.getresponse()
                self.assertEqual(response.status, 404)
                response.read()
            finally:
                conn.close()
                server.shutdown()

    def test_live_stdin_ingests_current_frames_and_marks_summary(self):
        hub = LiveHub("live")
        events = hub.subscribe()
        feed_jsonl_live(BytesIO(
            b'boot log\n{"type":"ready","protocol":2}\n'
            b'{"type":"frame","seq":76,"state":"active","locked":1}\n'
            b'{"type":"track","seq":76,"id":2,"xyz_m":[1,2,2],"points":3}\n'
            b'{"type":"summary"}\n{"type":"frame","seq":99,"state":"active"}\n'
        ), hub)
        self.assertEqual([json.loads(events.get_nowait()).get("status") for _ in range(4)],
                         ["waiting", "streaming", None, "finished"])
        self.assertEqual(hub.snapshot()["input_status"], "finished")
        self.assertEqual(hub.snapshot()["latest"]["seq"], 76)
        self.assertEqual(hub.snapshot()["latest"]["tracks"][0]["range_m"], 3)

    def test_real_run10_jsonl_if_present(self):
        sample = Path("data/s3_ifw192s_live_20261120_10/live_w5500_timing.jsonl")
        if not sample.exists():
            self.skipTest("local run 10 jsonl not present")
        asm = FrameStreamAssembler()
        frames = []
        for line in sample.read_text(encoding="utf-8").splitlines():
            if line.startswith("{"):
                frames.extend(asm.push_message(json.loads(line)))
        frames.extend(asm.flush())
        locked_frames = [f for f in frames if f["tracks"]]
        self.assertEqual(len(locked_frames), 9)


if __name__ == "__main__":
    unittest.main()
