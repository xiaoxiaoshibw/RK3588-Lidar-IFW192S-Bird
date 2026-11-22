"""Regression tests for S3 JSON rendering, camera wall-clock alignment, and overwrite protection."""
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from web.s3_live_server import FrameStreamAssembler
from web.s3_sync_video import (
    align_timestamp_to_s3,
    camera_frame_wall_ms,
    load_s3_frames,
    render_s3_standalone,
    render_synced_side_by_side,
)


def _write_synthetic_jsonl(path, base_wall_ms=1_700_000_000_000.0):
    rows = [
        {"type": "ready", "protocol": 2, "profile": "geometry_only", "host_wall_ms": base_wall_ms - 500.0},
        {
            "type": "frame", "seq": 100, "state": "warmup", "warmup": 1,
            "locked": 0, "candidates": 0, "tentative": 0,
            "tail_read_us": 1000, "grid_ready_us": 31000, "tracker_start_us": 31005, "result_ready_us": 46000,
            "host_wall_ms": base_wall_ms + 0.0,
        },
        {
            "type": "frame", "seq": 101, "state": "active", "warmup": 0,
            "locked": 0, "candidates": 1, "tentative": 1,
            "tail_read_us": 105000, "grid_ready_us": 135000, "tracker_start_us": 135005, "result_ready_us": 150000,
            "host_wall_ms": base_wall_ms + 100.0,
        },
        {
            "type": "frame", "seq": 102, "state": "active", "warmup": 0,
            "locked": 1, "candidates": 1, "tentative": 0,
            "tail_read_us": 209000, "grid_ready_us": 239000, "tracker_start_us": 239005, "result_ready_us": 254000,
            "host_wall_ms": base_wall_ms + 200.0,
        },
        {
            "type": "track", "seq": 102, "id": 7, "box": [110, 80, 124, 92],
            "xyz_m": [2.45, -0.18, 0.32], "points": 64,
            "host_wall_ms": base_wall_ms + 201.0,
        },
        {
            "type": "frame", "seq": 103, "state": "active", "warmup": 0,
            "locked": 1, "candidates": 1, "tentative": 0,
            "tail_read_us": 313000, "grid_ready_us": 343000, "tracker_start_us": 343005, "result_ready_us": 358000,
            "host_wall_ms": base_wall_ms + 300.0,
        },
        {
            "type": "track", "seq": 103, "id": 7, "box": [112, 81, 126, 93],
            "xyz_m": [2.52, -0.10, 0.36], "points": 68,
            "host_wall_ms": base_wall_ms + 301.0,
        },
        {
            "type": "frame", "seq": 104, "state": "UNKNOWN", "warmup": 0,
            "locked": 0, "candidates": 0, "tentative": 0,
            "host_wall_ms": base_wall_ms + 400.0,
        },
    ]
    with Path(path).open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")


def _write_synthetic_camera(video_path, meta_path, base_wall_ms=1_700_000_000_000.0, num_frames=12, fps=20.0):
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(video_path), fourcc, fps, (320, 240))
    assert writer.isOpened()
    for i in range(num_frames):
        img = np.full((240, 320, 3), (30, 40, 50), dtype=np.uint8)
        cv2.circle(img, (40 + i * 20, 120), 14, (80, 220, 120), -1)
        writer.write(img)
    writer.release()

    # Start 300ms before S3 frame 0 so early frames test NO_S3_SYNC behavior.
    start_wall_ms = base_wall_ms - 300.0
    meta = {
        "device": "/dev/video0",
        "resolution": "320x240",
        "fps": fps,
        "start_wall_ms": start_wall_ms,
        "frame_pts_s": [round(i / fps, 6) for i in range(num_frames)],
    }
    Path(meta_path).write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")


class S3SyncVideoTests(unittest.TestCase):
    def test_assembler_preserves_host_wall_ms(self):
        asm = FrameStreamAssembler()
        frames = asm.push_message({
            "type": "frame", "seq": 1, "state": "active", "warmup": 0,
            "locked": 0, "candidates": 0, "tentative": 0,
            "host_wall_ms": 123456.789, "host_mono_ms": 42.5,
        })
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["host_wall_ms"], 123456.789)
        self.assertEqual(frames[0]["host_mono_ms"], 42.5)

    def test_align_timestamp_respects_max_skew(self):
        times = [1000.0, 1100.0, 1200.0]
        idx, dt = align_timestamp_to_s3(1080.0, times, max_skew_ms=50.0)
        self.assertEqual(idx, 1)
        self.assertEqual(dt, 20.0)

        idx, dt = align_timestamp_to_s3(700.0, times, max_skew_ms=250.0)
        self.assertIsNone(idx)
        self.assertIsNone(dt)

    def test_camera_frame_wall_ms_prefers_pts(self):
        meta = {"start_wall_ms": 5000.0, "fps": 30.0, "frame_pts_s": [0.0, 0.035, 0.071]}
        self.assertAlmostEqual(camera_frame_wall_ms(meta, 1, 30.0), 5035.0)
        self.assertAlmostEqual(camera_frame_wall_ms(meta, 2, 30.0), 5071.0)

    def test_standalone_and_synced_render_and_overwrite_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            jsonl_path = root / "live.jsonl"
            cam_path = root / "cam.mp4"
            meta_path = root / "camera_meta.json"
            s3_mp4 = root / "s3_render.mp4"
            sync_mp4 = root / "sync.mp4"
            summary_json = root / "sync_summary.json"

            _write_synthetic_jsonl(jsonl_path)
            _write_synthetic_camera(cam_path, meta_path)

            frames = load_s3_frames(jsonl_path)
            self.assertEqual(len(frames), 5)
            self.assertEqual(frames[2]["tracks"][0]["id"], 7)

            s3_stats = render_s3_standalone(jsonl_path, s3_mp4, fps=10.0)
            self.assertTrue(s3_mp4.exists() and s3_mp4.stat().st_size > 0)
            self.assertEqual(s3_stats["locked_frames"], 2)
            self.assertEqual(s3_stats["tracks"][0]["track_id"], 7)

            with self.assertRaises(FileExistsError):
                render_s3_standalone(jsonl_path, s3_mp4, fps=10.0)

            sync_stats = render_synced_side_by_side(
                jsonl_path, cam_path, meta_path, sync_mp4,
                summary_json_path=summary_json, max_skew_ms=120.0,
            )
            self.assertTrue(sync_mp4.exists() and sync_mp4.stat().st_size > 0)
            self.assertTrue(summary_json.exists())
            self.assertEqual(sync_stats["camera_frames"], 12)
            # Early camera frames at -300ms/-250ms/-200ms/-150ms are outside 120ms skew -> unsynced
            self.assertLess(sync_stats["synced_camera_frames"], 12)
            self.assertGreater(sync_stats["locked_synced_camera_frames"], 0)

            with self.assertRaises(FileExistsError):
                render_synced_side_by_side(jsonl_path, cam_path, meta_path, sync_mp4)


if __name__ == "__main__":
    unittest.main()
