"""End-to-end tests for RK3588 two-container bird recognition & deployment pipeline."""
import asyncio
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
CONTAINER_ROOT = REPO_ROOT / "deploy" / "rk3588_containers"
BIRD_DIR = CONTAINER_ROOT / "bird"
CORE_DIR = REPO_ROOT / "firmware" / "esp32s3_bird_core"

for path in (BIRD_DIR, CONTAINER_ROOT, REPO_ROOT / "web", REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import bird_service
import deploy_to_rk3588
from foxglove_client import FoxgloveError, stream_pointcloud2
from native_tracker import K_CELLS, K_HEIGHT, K_WIDTH, NativeBirdTracker
from pointcloud_grid import CALIBRATION_HEADER_BYTES, IFW192SScanGrid, parse_pointcloud2


def _make_synthetic_angles():
    """Create a realistic 192x256 azimuth/elevation table matching IFW192S ordering."""
    az_1d = np.linspace(75.0, -75.0, K_WIDTH, dtype=np.float64)
    el_1d = np.linspace(-35.0, 35.0, K_HEIGHT, dtype=np.float64)
    az = np.broadcast_to(az_1d[None, :], (K_HEIGHT, K_WIDTH)).copy()
    el = np.broadcast_to(el_1d[:, None], (K_HEIGHT, K_WIDTH)).copy()
    return az, el


def _pack_ros_string(text):
    raw = text.encode("utf-8")
    return struct.pack("<I", len(raw)) + raw


def _serialize_pointcloud2(seq, x, y, z, ring, *, frame_id="innolidar", secs=1700000000, nsecs=0):
    """Serialize arrays into a valid ROS1 sensor_msgs/PointCloud2 (28-byte stride)."""
    n = len(x)
    assert len(y) == n and len(z) == n and len(ring) == n
    row_dtype = np.dtype([
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("ring", "<u2"),
        ("_pad1", "<u2"),
        ("timestamp", "<f4"),
        ("_pad2", "<f4"),
    ])
    buf = np.zeros(n, dtype=row_dtype)
    buf["x"] = x
    buf["y"] = y
    buf["z"] = z
    buf["intensity"] = 100.0
    buf["ring"] = ring
    data_bytes = buf.tobytes()

    fields = [
        ("x", 0, 7, 1),
        ("y", 4, 7, 1),
        ("z", 8, 7, 1),
        ("intensity", 12, 7, 1),
        ("ring", 16, 4, 1),
        ("timestamp", 20, 7, 1),
    ]
    out = bytearray()
    out.extend(struct.pack("<III", seq, secs, nsecs))
    out.extend(_pack_ros_string(frame_id))
    out.extend(struct.pack("<III", 1, n, len(fields)))
    for name, offset, datatype, count in fields:
        out.extend(_pack_ros_string(name))
        out.extend(struct.pack("<IBI", offset, datatype, count))
    out.extend(struct.pack("<?III", False, 28, n * 28, len(data_bytes)))
    out.extend(data_bytes)
    out.extend(struct.pack("<?", False))
    return bytes(out)


def _compile_native_lib(build_dir):
    gxx = shutil.which("g++")
    if not gxx:
        return None
    ext = ".dll" if os.name == "nt" else ".so"
    out_lib = Path(build_dir) / f"libbird_tracker{ext}"
    cmd = [
        gxx, "-O3", "-std=c++17", "-shared", "-Wall", "-Wextra", "-Werror",
    ]
    if os.name != "nt":
        cmd.append("-fPIC")
    else:
        cmd.append("-static")
    cmd.extend([
        f"-I{CORE_DIR}",
        str(BIRD_DIR / "bird_core_c_api.cpp"),
        str(CORE_DIR / "grid_algorithm.cpp"),
        str(CORE_DIR / "tracker.cpp"),
        str(CORE_DIR / "geometry.cpp"),
        "-o", str(out_lib),
    ])
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise AssertionError(f"g++ compile failed:\n{res.stdout}\n{res.stderr}")
    return out_lib


def _make_grid_scene_cloud(seq, grid, bg_range_m=4.2, target_box=None, target_range_m=2.5, depth_m=0.09):
    """Generate a full-grid ROS1 PointCloud2 with optional 3D ellipsoid bird-sized patch."""
    ranges = np.full((K_HEIGHT, K_WIDTH), bg_range_m, dtype=np.float64)
    if target_box is not None:
        r0, r1, c0, c1 = target_box
        rr, cc = np.meshgrid(np.arange(r0, r1), np.arange(c0, c1), indexing="ij")
        # Front-facing 3D ellipsoid so radial depth >= 0.04 m and sphericity >= 0.085
        rc = 0.5 * (r0 + r1 - 1)
        cc_mid = 0.5 * (c0 + c1 - 1)
        dist_sq = ((rr - rc) / max(1.0, (r1 - r0) * 0.5)) ** 2 + ((cc - cc_mid) / max(1.0, (c1 - c0) * 0.5)) ** 2
        inside = dist_sq <= 1.0
        hemi = 1.0 - np.sqrt(np.maximum(0.0, 1.0 - np.minimum(dist_sq, 1.0)))
        patch = np.where(inside, target_range_m + depth_m * hemi, bg_range_m)
        ranges[r0:r1, c0:c1] = patch

    rays = grid.unit_rays.reshape(K_HEIGHT, K_WIDTH, 3)
    xyz = (rays * ranges[:, :, None]).astype(np.float32)
    rings = np.broadcast_to(np.arange(K_HEIGHT, dtype=np.uint16)[:, None], (K_HEIGHT, K_WIDTH))
    x = xyz[:, :, 0].ravel()
    y = xyz[:, :, 1].ravel()
    z = xyz[:, :, 2].ravel()
    r = rings.ravel()
    return _serialize_pointcloud2(seq, x, y, z, r)


class TestRK3588TwoContainers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory(prefix="rk3588_bird_test_", ignore_cleanup_errors=True)
        cls.tmp_path = Path(cls._tmp.name)
        cls.native_lib = _compile_native_lib(cls.tmp_path)
        az, el = _make_synthetic_angles()
        cls.grid = IFW192SScanGrid(az, el)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_scan_grid_csv_and_bin_parity_and_nearest_echo(self):
        az, el = _make_synthetic_angles()
        az_csv = self.tmp_path / "az.csv"
        el_csv = self.tmp_path / "el.csv"
        cal_bin = self.tmp_path / "cal.bin"
        np.savetxt(az_csv, az, delimiter=",", fmt="%.6f")
        np.savetxt(el_csv, el, delimiter=",", fmt="%.6f")
        az_parsed = np.loadtxt(az_csv, delimiter=",")
        el_parsed = np.loadtxt(el_csv, delimiter=",")
        angles_interleaved = np.stack([az_parsed, el_parsed], axis=-1).astype("<f8").tobytes()
        cal_bin.write_bytes(b"\x00" * CALIBRATION_HEADER_BYTES + angles_interleaved)

        grid_csv = IFW192SScanGrid.from_csv(az_csv, el_csv)
        grid_bin = IFW192SScanGrid.from_calibration_bin(cal_bin)
        self.assertEqual(grid_csv.sha256, grid_bin.sha256)
        np.testing.assert_allclose(grid_csv.unit_rays, grid_bin.unit_rays, atol=1e-12)

        cell_a = 10 * K_WIDTH + 20
        cell_b = 10 * K_WIDTH + 21
        ray_a = grid_bin.unit_rays[cell_a]
        ray_b = grid_bin.unit_rays[cell_b]

        pts = [
            (float("nan"), 0.0, 0.0, 10),                              # row 0: NaN (rejected)
            (0.0, 0.0, 0.0, 10),                                       # row 1: zero range (rejected)
            (*(ray_a * 3.5), 10),                                      # row 2: farther echo on cell_a
            (*(ray_a * 2.2), 10),                                      # row 3: nearest echo on cell_a (wins)
            (*(ray_a * 2.2), 10),                                      # row 4: same range, larger row (loses tie-break)
            (*(ray_b * 2.8), 10),                                      # row 5: valid echo on cell_b
        ]
        x = np.array([p[0] for p in pts], dtype="<f4")
        y = np.array([p[1] for p in pts], dtype="<f4")
        z = np.array([p[2] for p in pts], dtype="<f4")
        ring = np.array([p[3] for p in pts], dtype="<u2")

        raw = _serialize_pointcloud2(42, x, y, z, ring)
        cloud, measurements, presence, stats = grid_bin.decode_cloud_bytes(raw)
        self.assertEqual(cloud.header.seq, 42)
        self.assertEqual(stats["valid_cells"], 2)
        self.assertEqual(stats["duplicate_points"], 2)
        self.assertEqual(int(measurements[cell_a]["source_row"]), 3)
        self.assertEqual(int(measurements[cell_b]["source_row"]), 5)
        self.assertAlmostEqual(float(measurements[cell_a]["x"]), float(x[3]), places=6)
        self.assertNotEqual(int(presence[cell_a // 8]) & (1 << (cell_a % 8)), 0)
        self.assertNotEqual(int(presence[cell_b // 8]) & (1 << (cell_b % 8)), 0)

    def test_native_cpp17_tracker_warmup_lock_retain_and_provenance(self):
        if self.native_lib is None:
            self.skipTest("g++ compiler not available")

        with NativeBirdTracker(self.native_lib, self.grid.unit_rays) as tracker:
            # 10 warmup frames with static background
            for seq in range(10):
                raw = _make_grid_scene_cloud(seq, self.grid, bg_range_m=4.2)
                frame, parsed_seq, seq_gap, _ = bird_service.process_cloud_payload(
                    raw, self.grid, tracker, previous_seq=seq - 1 if seq > 0 else None, include_provenance=True,
                )
                self.assertEqual(parsed_seq, seq)
                self.assertFalse(seq_gap)
                self.assertEqual(frame["state"], "warmup")
                self.assertEqual(frame["warmup"], seq + 1 if seq < 9 else 0)
                self.assertEqual(frame["locked"], 0)

            # Move a bird-sized cluster across active frames (>4.5 cm displacement, >=5 hits)
            # At 2.5 m, 1 column ~ 0.588 deg ~ 2.56 cm, 1 ring ~ 0.366 deg ~ 1.60 cm.
            # Patch 9 rings (~14.4 cm) x 6 cols (~15.4 cm) ~ 40 cells.
            locked_seen = []
            prev_seq = 9
            final_box = None
            for step_idx in range(8):
                seq = 10 + step_idx
                c0 = 120 + step_idx
                final_box = (92, 101, c0, c0 + 6)
                raw = _make_grid_scene_cloud(seq, self.grid, bg_range_m=4.2, target_box=final_box, target_range_m=2.5)
                frame, parsed_seq, seq_gap, _ = bird_service.process_cloud_payload(
                    raw, self.grid, tracker, previous_seq=prev_seq, include_provenance=True,
                )
                prev_seq = parsed_seq
                self.assertFalse(seq_gap)
                self.assertEqual(frame["state"], "active")
                if frame["locked"] > 0:
                    locked_seen.append(frame)

            self.assertGreaterEqual(len(locked_seen), 2, "Moving bird cluster must establish lock")
            last_locked = locked_seen[-1]
            track = last_locked["tracks"][0]
            locked_id = track["id"]
            self.assertEqual(track["reason"], "dynamic")
            self.assertGreaterEqual(track["points"], 15)
            self.assertEqual(len(track["cells"]), track["points"])
            self.assertEqual(len(track["source_rows"]), track["points"])
            # Every cell's source_row equals the cell index because full-grid scene emits rows in cell order
            self.assertEqual(track["cells"], track["source_rows"])

            # Stationary continuation at final_box keeps the same locked track ID
            for extra in range(3):
                seq = 18 + extra
                raw = _make_grid_scene_cloud(seq, self.grid, bg_range_m=4.2, target_box=final_box, target_range_m=2.5)
                frame, prev_seq, _, _ = bird_service.process_cloud_payload(
                    raw, self.grid, tracker, previous_seq=prev_seq, include_provenance=True,
                )
                self.assertEqual(frame["state"], "active")
                self.assertEqual(frame["locked"], 1)
                self.assertEqual(frame["tracks"][0]["id"], locked_id)

            # Sequence gap clears tracks while keeping background active
            raw_gap = _make_grid_scene_cloud(99, self.grid, bg_range_m=4.2, target_box=final_box, target_range_m=2.5)
            frame_gap, _, seq_gap, _ = bird_service.process_cloud_payload(
                raw_gap, self.grid, tracker, previous_seq=prev_seq,
            )
            self.assertTrue(seq_gap)
            self.assertEqual(frame_gap["state"], "active")
            self.assertEqual(frame_gap["locked"], 0)

            # Full reset restarts 10-frame warmup
            tracker.reset()
            raw_reset = _make_grid_scene_cloud(100, self.grid, bg_range_m=4.2)
            frame_reset, _, _, _ = bird_service.process_cloud_payload(raw_reset, self.grid, tracker)
            self.assertEqual(frame_reset["state"], "warmup")
            self.assertEqual(frame_reset["warmup"], 1)

    def test_foxglove_client_and_bird_service_http_sse_disconnect_reset(self):
        if self.native_lib is None:
            self.skipTest("g++ compiler not available")
        import websockets

        # Prepare static PointCloud2 frames to stream continuously until disconnect
        base_payload = _make_grid_scene_cloud(0, self.grid, bg_range_m=4.2)
        subscribed_topics = []
        server_port = {}
        server_ready = threading.Event()
        release_disconnect = threading.Event()

        async def handler(ws):
            await ws.send(json.dumps({
                "op": "advertise",
                "channels": [{
                    "id": 7,
                    "topic": "/innolidar_points",
                    "encoding": "ros1",
                    "schemaName": "sensor_msgs/PointCloud2",
                }],
            }))
            sub_raw = await asyncio.wait_for(ws.recv(), timeout=5.0)
            sub_msg = json.loads(sub_raw)
            self.assertEqual(sub_msg.get("op"), "subscribe")
            subscribed_topics.append(sub_msg)
            await ws.ping(b"hb")
            seq = 0
            while not release_disconnect.is_set():
                # Update only the 4-byte ROS header seq in-place for fast realistic ~20 Hz streaming
                body = struct.pack("<I", seq) + base_payload[4:]
                frame_bin = struct.pack("<BIQ", 1, 1, 1_700_000_000_000_000_000 + seq * 50_000_000) + body
                await ws.send(frame_bin)
                seq += 1
                await asyncio.sleep(0.05)
            await ws.send(json.dumps({"op": "unadvertise", "channelIds": [7]}))
            await ws.close()

        async def run_ws_server():
            async with websockets.serve(
                handler, "127.0.0.1", 0,
                subprotocols=["foxglove.websocket.v1"],
                max_size=16 * 1024 * 1024,
            ) as srv:
                server_port["port"] = srv.sockets[0].getsockname()[1]
                server_ready.set()
                while not release_disconnect.is_set():
                    await asyncio.sleep(0.05)
                await asyncio.sleep(0.2)

        ws_thread = threading.Thread(target=lambda: asyncio.run(run_ws_server()), daemon=True)
        ws_thread.start()
        self.assertTrue(server_ready.wait(timeout=5.0))
        ws_url = f"ws://127.0.0.1:{server_port['port']}"

        hub = bird_service.BirdRealtimeHub(
            "测试 · RK3588 C++17",
            foxglove_url=ws_url,
            topic="/innolidar_points",
            calibration_sha256=self.grid.sha256,
        )
        slot = bird_service.LatestSlot()
        stop_event = threading.Event()
        server = ThreadingHTTPServer(("127.0.0.1", 0), bird_service.make_bird_handler(hub))
        http_port = server.server_address[1]
        http_thread = threading.Thread(target=server.serve_forever, daemon=True)
        http_thread.start()

        with NativeBirdTracker(self.native_lib, self.grid.unit_rays) as tracker:
            worker = threading.Thread(
                target=bird_service._worker_loop,
                args=(slot, self.grid, tracker, hub, stop_event),
                daemon=True,
            )
            reader = threading.Thread(
                target=bird_service._foxglove_reader_loop,
                args=(ws_url, "/innolidar_points", slot, hub, stop_event),
                kwargs={"first_frame_timeout": 5.0, "frame_timeout": 2.0, "reconnect_delay": 5.0},
                daemon=True,
            )
            worker.start()
            reader.start()

            try:
                # Wait until active state reached (after 10 warmup frames)
                deadline = time.monotonic() + 10.0
                health = None
                while time.monotonic() < deadline:
                    with urlopen(f"http://127.0.0.1:{http_port}/healthz", timeout=2) as resp:
                        health = json.loads(resp.read().decode("utf-8"))
                    if health["state"] == "active" and health["input_status"] == "streaming":
                        break
                    time.sleep(0.05)

                self.assertIsNotNone(health)
                self.assertEqual(health["input_status"], "streaming")
                self.assertEqual(health["state"], "active")
                self.assertEqual(health["semantic_truth"], "UNKNOWN")
                self.assertGreaterEqual(health["metrics"]["processed"], 11)
                self.assertEqual(len(subscribed_topics), 1)

                with urlopen(f"http://127.0.0.1:{http_port}/", timeout=2) as resp:
                    html = resp.read().decode("utf-8")
                    self.assertIn("<html", html.lower())

                with urlopen(f"http://127.0.0.1:{http_port}/state", timeout=2) as resp:
                    snap = json.loads(resp.read().decode("utf-8"))
                    self.assertEqual(snap["latest"]["state"], "active")

                req = Request(f"http://127.0.0.1:{http_port}/reset-background", method="POST")
                with urlopen(req, timeout=2) as resp:
                    reset_reply = json.loads(resp.read().decode("utf-8"))
                    self.assertTrue(reset_reply["ok"])

                # Trigger unadvertise / disconnect and verify UNKNOWN reset
                release_disconnect.set()
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    with urlopen(f"http://127.0.0.1:{http_port}/state", timeout=2) as resp:
                        snap = json.loads(resp.read().decode("utf-8"))
                    if snap["input_status"] == "closed" and snap["latest"]["state"] == "UNKNOWN":
                        break
                    time.sleep(0.05)
                self.assertEqual(snap["input_status"], "closed")
                self.assertEqual(snap["latest"]["state"], "UNKNOWN")
                self.assertEqual(snap["latest"]["tracks"], [])
            finally:
                stop_event.set()
                slot.close()
                server.shutdown()
                server.server_close()
                reader.join(timeout=2.0)
                worker.join(timeout=2.0)
                ws_thread.join(timeout=2.0)

    def test_staging_bundle_and_linux_lf_scripts(self):
        # Verify entrypoint.sh uses pure LF endings
        entrypoint_bytes = (CONTAINER_ROOT / "human_fall" / "entrypoint.sh").read_bytes()
        self.assertNotIn(b"\r\n", entrypoint_bytes)

        # Build a synthetic immutable human-fall bundle tar
        az, el = _make_synthetic_angles()
        az_buf = io.StringIO()
        el_buf = io.StringIO()
        np.savetxt(az_buf, az, delimiter=",", fmt="%.4f")
        np.savetxt(el_buf, el, delimiter=",", fmt="%.4f")

        payload_files = {
            "src/inno_lidar_ros/config/ifw192s/ifw192s_azimuth_192x256.csv": az_buf.getvalue().encode("utf-8"),
            "src/inno_lidar_ros/config/ifw192s/ifw192s_elevation_192x256.csv": el_buf.getvalue().encode("utf-8"),
            "src/human_fall_detection/config/range_service_v1.json": b'{"protocol":"range_recognition/1"}\n',
            "src/human_fall_detection/scripts/range_service_node.py": b"# stub\n",
        }
        manifest = {
            rel: {"bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}
            for rel, raw in payload_files.items()
        }
        release = {
            "release_id": "test_release_1234",
            "entry": "src/human_fall_detection/scripts/range_service_node.py",
            "config": "src/human_fall_detection/config/range_service_v1.json",
            "canonical_state_topic": "/single_target_live/state",
        }
        synth_tar = self.tmp_path / "synth_human_fall.tar"
        with tarfile.open(synth_tar, "w") as tf:
            for rel, raw in payload_files.items():
                deploy_to_rk3588._add_bytes_to_tar(tf, rel, raw)
            deploy_to_rk3588._add_bytes_to_tar(tf, "MANIFEST.json", json.dumps(manifest).encode("utf-8"))
            deploy_to_rk3588._add_bytes_to_tar(tf, "RELEASE.json", json.dumps(release).encode("utf-8"))

        stage_out = self.tmp_path / "stage_synth"
        archive_path, manifest_path, summary = deploy_to_rk3588.create_staging_bundle(
            stage_out,
            human_fall_tar=synth_tar,
            config={
                "remote_root": "/home/wel/.local/opt/lidar-rk3588",
                "bird_http_port": 8895,
                "human_fall_cpuset": "4,5",
                "bird_cpuset": "6,7",
                "human_fall_mode": "instant_start",
                "mount_pitch_deg": 26.0,
                "mount_roll_deg": 0.0,
                "disable_imu": True,
                "with_human_physical_v1": False,
                "single_person_lock": False,
                "human_fall_base_image": "",
                "bird_base_image": "",
            },
        )
        self.assertTrue(archive_path.is_file())
        self.assertTrue(manifest_path.is_file())
        self.assertEqual(summary["human_fall_release"]["release_id"], "test_release_1234")

        with tarfile.open(archive_path, "r:gz") as tf:
            names = set(tf.getnames())
            self.assertIn("remote_deploy.sh", names)
            self.assertIn("human_fall_bundle.tar", names)
            self.assertIn("calibration/ifw192s_azimuth_192x256.csv", names)
            self.assertIn("repo/deploy/rk3588_containers/docker-compose.yml", names)
            remote_sh = tf.extractfile("remote_deploy.sh").read()
            self.assertNotIn(b"\r\n", remote_sh)
            remote_text = remote_sh.decode("utf-8")
            # Build must occur before stopping legacy range_service_node.py
            self.assertLess(
                remote_text.index("docker build"),
                remote_text.index('pgrep -f "[r]ange_service_node.py"'),
            )

        # Also validate the real human-fall bundle if present on this workstation
        if deploy_to_rk3588.DEFAULT_HUMAN_FALL_TAR.is_file():
            real_out = self.tmp_path / "stage_real"
            real_archive, _, real_summary = deploy_to_rk3588.create_staging_bundle(
                real_out,
                human_fall_tar=deploy_to_rk3588.DEFAULT_HUMAN_FALL_TAR,
                config={
                    "remote_root": "/home/wel/.local/opt/lidar-rk3588",
                    "bird_http_port": 8895,
                    "human_fall_cpuset": "4,5",
                    "bird_cpuset": "6,7",
                    "human_fall_mode": "instant_start",
                    "mount_pitch_deg": 26.0,
                    "mount_roll_deg": 0.0,
                    "disable_imu": True,
                    "with_human_physical_v1": False,
                    "single_person_lock": False,
                    "human_fall_base_image": "",
                    "bird_base_image": "",
                },
            )
            self.assertTrue(real_archive.is_file())
            self.assertEqual(real_summary["human_fall_manifest_files"], 86)


if __name__ == "__main__":
    unittest.main()
