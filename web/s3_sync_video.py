"""Render ESP32-S3 JSON tracking video and millisecond-synced 3588 USB camera comparison video."""
import argparse
from bisect import bisect_left
from collections import deque
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from web.s3_live_server import FrameStreamAssembler

PANEL_W = 960
PANEL_H = 720
MAX_TRAIL_POINTS = 120

# BGR color palette matching web/s3_live.html
BG_COLOR = (24, 16, 11)
PANEL_COLOR = (38, 26, 18)
BORDER_COLOR = (68, 48, 35)
GRID_COLOR = (52, 39, 28)
TEXT_COLOR = (247, 236, 228)
MUTED_COLOR = (181, 155, 138)
GREEN_COLOR = (157, 229, 46)
AMBER_COLOR = (77, 184, 255)
RED_COLOR = (119, 92, 255)
BLUE_COLOR = (255, 166, 88)

TRACK_PALETTE = [
    (157, 229, 46),   # #2ee59d
    (255, 166, 88),   # #58a6ff
    (77, 184, 255),   # #ffb84d
    (157, 107, 255),  # #ff6b9d
    (255, 136, 179),  # #b388ff
    (225, 208, 77),   # #4dd0e1
    (79, 213, 255),   # #ffd54f
    (196, 203, 128),  # #80cbc4
]


def _color_for_track(track_id):
    return TRACK_PALETTE[abs(int(track_id)) % len(TRACK_PALETTE)]


def load_s3_frames(jsonl_path):
    """Parse an S3 live_w5500_timing.jsonl file into ordered atomic frame snapshots."""
    assembler = FrameStreamAssembler()
    frames = []
    with Path(jsonl_path).open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue
            for frame in assembler.push_message(msg):
                frames.append(frame)
    for frame in assembler.flush():
        frames.append(frame)
    return frames


def load_camera_meta(meta_path):
    """Load camera_meta.json describing start_wall_ms, fps, and optional per-frame PTS."""
    with Path(meta_path).open("r", encoding="utf-8") as stream:
        meta = json.load(stream)
    if "start_wall_ms" not in meta:
        raise ValueError("camera_meta.json missing required 'start_wall_ms'")
    return meta


def camera_frame_wall_ms(meta, frame_index, fallback_fps):
    """Compute the 3588 wall-clock timestamp (ms) of camera frame_index."""
    start_ms = float(meta["start_wall_ms"])
    pts_list = meta.get("frame_pts_s")
    if isinstance(pts_list, list) and frame_index < len(pts_list) and pts_list[frame_index] is not None:
        return start_ms + float(pts_list[frame_index]) * 1000.0
    fps = float(meta.get("fps") or fallback_fps or 30.0)
    return start_ms + (frame_index / max(fps, 1e-3)) * 1000.0


def align_timestamp_to_s3(target_wall_ms, s3_times_ms, max_skew_ms=250.0):
    """Return (index, signed_dt_ms) of the nearest S3 frame within max_skew_ms, or (None, None)."""
    if not s3_times_ms:
        return None, None
    pos = bisect_left(s3_times_ms, target_wall_ms)
    best_idx = None
    best_abs = None
    best_dt = None
    for cand in (pos - 1, pos):
        if 0 <= cand < len(s3_times_ms):
            dt = s3_times_ms[cand] - target_wall_ms
            abs_dt = abs(dt)
            if best_abs is None or abs_dt < best_abs:
                best_abs = abs_dt
                best_idx = cand
                best_dt = dt
    if best_abs is not None and best_abs <= max_skew_ms:
        return best_idx, round(best_dt, 3)
    return None, None


def _draw_card(canvas, x0, y0, x1, y1, title, subtitle=""):
    cv2.rectangle(canvas, (x0, y0), (x1, y1), PANEL_COLOR, -1)
    cv2.rectangle(canvas, (x0, y0), (x1, y1), BORDER_COLOR, 1)
    cv2.line(canvas, (x0, y0 + 28), (x1, y0 + 28), BORDER_COLOR, 1)
    cv2.putText(canvas, title, (x0 + 10, y0 + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.48, TEXT_COLOR, 1, cv2.LINE_AA)
    if subtitle:
        (tw, _), _ = cv2.getTextSize(subtitle, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
        cv2.putText(canvas, subtitle, (x1 - tw - 10, y0 + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.40, MUTED_COLOR, 1, cv2.LINE_AA)


def _draw_top_down(canvas, rect, frame, trails):
    x0, y0, x1, y1 = rect
    _draw_card(canvas, x0, y0, x1, y1, "Top-Down (X-Y)", "Work 1.5-4.5m")
    px0, py0, px1, py1 = x0 + 36, y0 + 38, x1 - 14, y1 - 24
    pw, ph = px1 - px0, py1 - py0

    def to_px(xm, ym):
        # X: 0..5.0m (bottom to top), Y: -2.5..+2.5m (right to left, or left=+Y to match radar view)
        u = px0 + int(round((2.5 - ym) / 5.0 * pw))
        v = py1 - int(round(xm / 5.0 * ph))
        return u, v

    # Grid lines
    for xm in range(1, 6):
        _, gy = to_px(xm, 0.0)
        cv2.line(canvas, (px0, gy), (px1, gy), GRID_COLOR, 1)
        cv2.putText(canvas, f"{xm}m", (x0 + 6, gy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.36, MUTED_COLOR, 1, cv2.LINE_AA)
    for ym in (-2, -1, 0, 1, 2):
        gx, _ = to_px(0.0, ym)
        cv2.line(canvas, (gx, py0), (gx, py1), GRID_COLOR, 1)
        cv2.putText(canvas, f"{ym:+d}", (gx - 10, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.36, MUTED_COLOR, 1, cv2.LINE_AA)

    # Working range arcs (1.5m and 4.5m)
    ox, oy = to_px(0.0, 0.0)
    for r_m, col in ((1.5, AMBER_COLOR), (4.5, BLUE_COLOR)):
        pts = []
        for deg in range(-65, 66, 5):
            rad = math.radians(deg)
            rx = r_m * math.cos(rad)
            ry = r_m * math.sin(rad)
            u, v = to_px(rx, ry)
            if px0 <= u <= px1 and py0 <= v <= py1:
                pts.append([u, v])
        if len(pts) >= 2:
            cv2.polylines(canvas, [np.array(pts, dtype=np.int32)], False, col, 1, cv2.LINE_AA)

    cv2.circle(canvas, (ox, oy), 4, TEXT_COLOR, -1, cv2.LINE_AA)

    # Trails (measured historical positions)
    for tid, pts in trails.items():
        if len(pts) >= 2:
            col = _color_for_track(tid)
            poly = []
            for pt in pts:
                xyz = pt["xyz_m"]
                u, v = to_px(xyz[0], xyz[1])
                poly.append([np.clip(u, px0, px1), np.clip(v, py0, py1)])
            cv2.polylines(canvas, [np.array(poly, dtype=np.int32)], False, col, 1, cv2.LINE_AA)

    # Current locked tracks
    if frame and frame.get("state") == "active":
        for tr in frame.get("tracks", []):
            xyz = tr.get("xyz_m")
            if not isinstance(xyz, list) or len(xyz) != 3:
                continue
            col = _color_for_track(tr["id"])
            u, v = to_px(xyz[0], xyz[1])
            u = int(np.clip(u, px0, px1))
            v = int(np.clip(v, py0, py1))
            cv2.circle(canvas, (u, v), 6, col, -1, cv2.LINE_AA)
            cv2.circle(canvas, (u, v), 9, col, 1, cv2.LINE_AA)
            lbl = f"ID{tr['id']} {tr.get('range_m', 0):.2f}m"
            cv2.putText(canvas, lbl, (min(u + 10, px1 - 80), max(v - 6, py0 + 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)


def _draw_side_view(canvas, rect, frame, trails):
    x0, y0, x1, y1 = rect
    _draw_card(canvas, x0, y0, x1, y1, "Side View (X-Z)", "X:0-5m Z:-1.5..+2.5m")
    px0, py0, px1, py1 = x0 + 38, y0 + 38, x1 - 14, y1 - 24
    pw, ph = px1 - px0, py1 - py0

    def to_px(xm, zm):
        u = px0 + int(round(xm / 5.0 * pw))
        v = py1 - int(round((zm - (-1.5)) / 4.0 * ph))
        return u, v

    for xm in (1, 2, 3, 4, 5):
        gx, _ = to_px(xm, 0.0)
        cv2.line(canvas, (gx, py0), (gx, py1), GRID_COLOR, 1)
        cv2.putText(canvas, f"{xm}m", (gx - 8, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.36, MUTED_COLOR, 1, cv2.LINE_AA)
    for zm in (-1.0, 0.0, 1.0, 2.0):
        _, gy = to_px(0.0, zm)
        col = BORDER_COLOR if zm == 0.0 else GRID_COLOR
        cv2.line(canvas, (px0, gy), (px1, gy), col, 1)
        cv2.putText(canvas, f"{zm:+.0f}", (x0 + 6, gy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.36, MUTED_COLOR, 1, cv2.LINE_AA)

    for r_m, col in ((1.5, AMBER_COLOR), (4.5, BLUE_COLOR)):
        gx, _ = to_px(r_m, 0.0)
        cv2.line(canvas, (gx, py0), (gx, py1), col, 1)

    for tid, pts in trails.items():
        if len(pts) >= 2:
            col = _color_for_track(tid)
            poly = []
            for pt in pts:
                xyz = pt["xyz_m"]
                u, v = to_px(xyz[0], xyz[2])
                poly.append([np.clip(u, px0, px1), np.clip(v, py0, py1)])
            cv2.polylines(canvas, [np.array(poly, dtype=np.int32)], False, col, 1, cv2.LINE_AA)

    if frame and frame.get("state") == "active":
        for tr in frame.get("tracks", []):
            xyz = tr.get("xyz_m")
            if not isinstance(xyz, list) or len(xyz) != 3:
                continue
            col = _color_for_track(tr["id"])
            u, v = to_px(xyz[0], xyz[2])
            u = int(np.clip(u, px0, px1))
            v = int(np.clip(v, py0, py1))
            cv2.circle(canvas, (u, v), 5, col, -1, cv2.LINE_AA)
            cv2.putText(canvas, f"ID{tr['id']} z={xyz[2]:+.2f}", (min(u + 8, px1 - 85), max(v - 4, py0 + 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, col, 1, cv2.LINE_AA)


def _draw_grid_view(canvas, rect, frame):
    x0, y0, x1, y1 = rect
    _draw_card(canvas, x0, y0, x1, y1, "256x192 Scan Grid (2D Box)", "Locked boxes")
    avail_w = (x1 - x0) - 24
    avail_h = (y1 - y0) - 44
    scale = min(avail_w / 256.0, avail_h / 192.0)
    gw = int(round(256 * scale))
    gh = int(round(192 * scale))
    gx0 = x0 + ((x1 - x0) - gw) // 2
    gy0 = y0 + 34 + (avail_h - gh) // 2
    cv2.rectangle(canvas, (gx0, gy0), (gx0 + gw, gy0 + gh), (18, 13, 9), -1)
    cv2.rectangle(canvas, (gx0, gy0), (gx0 + gw, gy0 + gh), BORDER_COLOR, 1)

    # Subtle 4x4 grid
    for i in range(1, 4):
        cv2.line(canvas, (gx0 + gw * i // 4, gy0), (gx0 + gw * i // 4, gy0 + gh), GRID_COLOR, 1)
        cv2.line(canvas, (gx0, gy0 + gh * i // 4), (gx0 + gw, gy0 + gh * i // 4), GRID_COLOR, 1)

    if frame and frame.get("state") == "active":
        for tr in frame.get("tracks", []):
            box = tr.get("box")
            if not isinstance(box, list) or len(box) != 4:
                continue
            bx0 = gx0 + int(round(box[0] * scale))
            by0 = gy0 + int(round(box[1] * scale))
            bx1 = gx0 + int(round((box[2] + 1) * scale))
            by1 = gy0 + int(round((box[3] + 1) * scale))
            col = _color_for_track(tr["id"])
            cv2.rectangle(canvas, (bx0, by0), (bx1, by1), col, 2)
            lbl = f"ID{tr['id']} ({tr.get('points', 0)}pts)"
            cv2.putText(canvas, lbl, (bx0, max(by0 - 5, gy0 + 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)


def _draw_track_table(canvas, rect, frame, sync_note):
    x0, y0, x1, y1 = rect
    _draw_card(canvas, x0, y0, x1, y1, "Measured Locked Targets", sync_note)
    headers = "ID   X(m)   Y(m)   Z(m)   R(m)  Pts   Box"
    cv2.putText(canvas, headers, (x0 + 12, y0 + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.39, MUTED_COLOR, 1, cv2.LINE_AA)
    cv2.line(canvas, (x0 + 10, y0 + 55), (x1 - 10, y0 + 55), BORDER_COLOR, 1)

    if frame is None:
        cv2.putText(canvas, "NO_S3_SYNC (waiting for S3 frame)", (x0 + 14, y0 + 84),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, AMBER_COLOR, 1, cv2.LINE_AA)
        return
    state = frame.get("state", "UNKNOWN")
    if state == "warmup":
        cv2.putText(canvas, f"WARMUP ({frame.get('warmup', 0)}/10) building static background...",
                    (x0 + 14, y0 + 84), cv2.FONT_HERSHEY_SIMPLEX, 0.44, AMBER_COLOR, 1, cv2.LINE_AA)
        return
    if state != "active":
        cv2.putText(canvas, f"STATE: {state}", (x0 + 14, y0 + 84),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, RED_COLOR, 1, cv2.LINE_AA)
        return

    tracks = frame.get("tracks", [])
    if not tracks:
        cv2.putText(canvas, "No locked targets (cand={}, tent={})".format(
            frame.get("candidates", 0), frame.get("tentative", 0)),
            (x0 + 14, y0 + 84), cv2.FONT_HERSHEY_SIMPLEX, 0.44, MUTED_COLOR, 1, cv2.LINE_AA)
        return

    y = y0 + 78
    for tr in tracks[:6]:
        xyz = tr.get("xyz_m") or [0.0, 0.0, 0.0]
        box = tr.get("box") or [0, 0, 0, 0]
        col = _color_for_track(tr["id"])
        row_txt = (
            f"#{tr['id']:<2d} {xyz[0]:5.2f} {xyz[1]:+6.2f} {xyz[2]:+6.2f} "
            f"{(tr.get('range_m') or 0.0):5.2f} {tr.get('points', 0):4d}  "
            f"[{box[0]},{box[1]},{box[2]},{box[3]}]"
        )
        cv2.putText(canvas, row_txt, (x0 + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.39, col, 1, cv2.LINE_AA)
        y += 24


def render_s3_panel(frame, trails, *, elapsed_s=None, sync_dt_ms=None, title="ESP32-S3 Live JSON"):
    """Render one 960x720 BGR dashboard image for an S3 frame snapshot (or None if unsynced)."""
    canvas = np.full((PANEL_H, PANEL_W, 3), BG_COLOR, dtype=np.uint8)

    # Header bar
    cv2.rectangle(canvas, (0, 0), (PANEL_W, 56), PANEL_COLOR, -1)
    cv2.line(canvas, (0, 56), (PANEL_W, 56), BORDER_COLOR, 1)
    cv2.putText(canvas, title, (14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, TEXT_COLOR, 1, cv2.LINE_AA)

    if frame is None:
        badge_txt, badge_col = "NO_S3_SYNC", AMBER_COLOR
        metrics = "seq=-  locked=0  cand=0  tent=0  lat=-"
        sync_note = "Unsynced"
    else:
        state = str(frame.get("state", "UNKNOWN"))
        if state == "active":
            badge_txt = f"ACTIVE · LOCKED {frame.get('locked', 0)}"
            badge_col = GREEN_COLOR if frame.get("locked", 0) > 0 else BLUE_COLOR
        elif state == "warmup":
            badge_txt = f"WARMUP {frame.get('warmup', 0)}/10"
            badge_col = AMBER_COLOR
        else:
            badge_txt = state.upper()
            badge_col = RED_COLOR
        lat = frame.get("latency_ms")
        trk = frame.get("tracker_ms")
        lat_str = f"{lat:.1f}ms" if isinstance(lat, (int, float)) else "-"
        trk_str = f"{trk:.1f}ms" if isinstance(trk, (int, float)) else "-"
        metrics = (
            f"seq={frame.get('seq')}  locked={frame.get('locked', 0)}  "
            f"cand={frame.get('candidates', 0)}  tent={frame.get('tentative', 0)}  "
            f"lat={lat_str}  trk={trk_str}"
        )
        sync_note = f"dt={sync_dt_ms:+.1f}ms" if isinstance(sync_dt_ms, (int, float)) else "Standalone"

    if isinstance(elapsed_s, (int, float)):
        metrics = f"t={elapsed_s:6.2f}s  " + metrics

    cv2.rectangle(canvas, (235, 8), (430, 32), (32, 24, 18), -1)
    cv2.rectangle(canvas, (235, 8), (430, 32), badge_col, 1)
    cv2.putText(canvas, badge_txt, (245, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, badge_col, 1, cv2.LINE_AA)
    cv2.putText(canvas, metrics, (14, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.42, MUTED_COLOR, 1, cv2.LINE_AA)

    # 2x2 grid of cards
    _draw_top_down(canvas, (12, 68, 474, 412), frame, trails)
    _draw_grid_view(canvas, (486, 68, 948, 412), frame)
    _draw_side_view(canvas, (12, 424, 474, 708), frame, trails)
    _draw_track_table(canvas, (486, 424, 948, 708), frame, sync_note)
    return canvas


def _fit_camera_panel(bgr_frame, target_w=PANEL_W, target_h=PANEL_H):
    """Letterbox-fit a BGR camera frame into target_w x target_h without distorting aspect ratio."""
    canvas = np.full((target_h, target_w, 3), BG_COLOR, dtype=np.uint8)
    h, w = bgr_frame.shape[:2]
    if h <= 0 or w <= 0:
        return canvas
    scale = min(target_w / float(w), target_h / float(h))
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    resized = cv2.resize(bgr_frame, (nw, nh), interpolation=cv2.INTER_AREA)
    x0 = (target_w - nw) // 2
    y0 = (target_h - nh) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = resized
    return canvas


def _overlay_camera_hud(panel, elapsed_s, s3_frame, dt_ms):
    cv2.rectangle(panel, (0, 0), (panel.shape[1], 44), (20, 14, 10), -1)
    cv2.line(panel, (0, 44), (panel.shape[1], 44), BORDER_COLOR, 1)
    left_txt = f"RK3588 USB Camera  |  t={elapsed_s:6.2f}s"
    cv2.putText(panel, left_txt, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.54, TEXT_COLOR, 1, cv2.LINE_AA)
    if s3_frame is None:
        right_txt, col = "S3: NO_S3_SYNC", AMBER_COLOR
    elif s3_frame.get("state") == "warmup":
        right_txt, col = f"S3: WARMUP {s3_frame.get('warmup', 0)}/10", AMBER_COLOR
    elif s3_frame.get("state") == "active":
        locked = int(s3_frame.get("locked", 0))
        right_txt = f"S3: LOCKED={locked}  dt={dt_ms:+.0f}ms"
        col = GREEN_COLOR if locked > 0 else BLUE_COLOR
    else:
        right_txt, col = f"S3: {s3_frame.get('state', 'UNKNOWN')}", RED_COLOR
    (tw, _), _ = cv2.getTextSize(right_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 1)
    cv2.putText(panel, right_txt, (panel.shape[1] - tw - 14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.50, col, 1, cv2.LINE_AA)
    return panel


class _VideoSink:
    """Write BGR frames to MP4 via ffmpeg H.264 if available, else OpenCV mp4v."""

    def __init__(self, output_path, width, height, fps):
        self.output_path = Path(output_path)
        if self.output_path.exists():
            raise FileExistsError(f"Refusing to overwrite existing output: {self.output_path}")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self._proc = None
        self._cv_writer = None

        ffmpeg_bin = shutil.which("ffmpeg")
        if ffmpeg_bin:
            cmd = [
                ffmpeg_bin, "-y", "-loglevel", "error",
                "-f", "rawvideo", "-vcodec", "rawvideo", "-pix_fmt", "bgr24",
                "-s", f"{self.width}x{self.height}", "-r", f"{self.fps:.3f}",
                "-i", "-",
                "-c:v", "libx264", "-preset", "veryfast", "-crf", "22",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                str(self.output_path),
            ]
            self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        else:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._cv_writer = cv2.VideoWriter(str(self.output_path), fourcc, self.fps, (self.width, self.height))
            if not self._cv_writer.isOpened():
                raise RuntimeError(f"Failed to open VideoWriter for {self.output_path}")

    def write(self, bgr_frame):
        if self._proc is not None:
            self._proc.stdin.write(bgr_frame.tobytes())
        else:
            self._cv_writer.write(bgr_frame)

    def close(self):
        if self._proc is not None:
            if self._proc.stdin:
                self._proc.stdin.close()
            rc = self._proc.wait()
            if rc != 0:
                raise RuntimeError(f"ffmpeg exited with status {rc} writing {self.output_path}")
        if self._cv_writer is not None:
            self._cv_writer.release()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


def _update_trails(trails, frame):
    if frame is None or frame.get("state") != "active":
        return
    for tr in frame.get("tracks", []):
        xyz = tr.get("xyz_m")
        if isinstance(xyz, list) and len(xyz) == 3:
            tid = str(tr["id"])
            trail = trails.setdefault(tid, deque(maxlen=MAX_TRAIL_POINTS))
            trail.append({"seq": frame.get("seq"), "xyz_m": xyz})


def _summarize_tracks(frames):
    stats = {}
    for idx, frame in enumerate(frames):
        if frame.get("state") != "active":
            continue
        for tr in frame.get("tracks", []):
            tid = str(tr["id"])
            rec = stats.setdefault(tid, {
                "track_id": int(tr["id"]),
                "frames": 0,
                "first_seq": frame.get("seq"),
                "last_seq": frame.get("seq"),
                "first_frame_idx": idx,
                "last_frame_idx": idx,
                "min_range_m": None,
                "max_range_m": None,
                "min_points": None,
                "max_points": None,
            })
            rec["frames"] += 1
            rec["last_seq"] = frame.get("seq")
            rec["last_frame_idx"] = idx
            r = tr.get("range_m")
            if isinstance(r, (int, float)):
                rec["min_range_m"] = r if rec["min_range_m"] is None else min(rec["min_range_m"], r)
                rec["max_range_m"] = r if rec["max_range_m"] is None else max(rec["max_range_m"], r)
            pts = tr.get("points")
            if isinstance(pts, int):
                rec["min_points"] = pts if rec["min_points"] is None else min(rec["min_points"], pts)
                rec["max_points"] = pts if rec["max_points"] is None else max(rec["max_points"], pts)
    return list(stats.values())


def render_s3_standalone(jsonl_path, output_mp4, fps=9.65):
    """Render standalone S3 JSON parsed dashboard video."""
    frames = load_s3_frames(jsonl_path)
    if not frames:
        raise ValueError(f"No S3 frames found in {jsonl_path}")
    trails = {}
    t0_ms = frames[0].get("host_wall_ms")
    with _VideoSink(output_mp4, PANEL_W, PANEL_H, fps) as sink:
        for idx, frame in enumerate(frames):
            _update_trails(trails, frame)
            if isinstance(t0_ms, (int, float)) and isinstance(frame.get("host_wall_ms"), (int, float)):
                elapsed_s = (frame["host_wall_ms"] - t0_ms) / 1000.0
            else:
                elapsed_s = idx / max(fps, 1e-3)
            panel = render_s3_panel(frame, trails, elapsed_s=elapsed_s)
            sink.write(panel)
    return {
        "s3_frames": len(frames),
        "active_frames": sum(1 for f in frames if f.get("state") == "active"),
        "locked_frames": sum(1 for f in frames if f.get("state") == "active" and f.get("locked", 0) > 0),
        "tracks": _summarize_tracks(frames),
    }


def render_synced_side_by_side(
    jsonl_path,
    camera_video_path,
    camera_meta_path,
    output_mp4,
    *,
    summary_json_path=None,
    max_skew_ms=250.0,
    output_fps=None,
):
    """Align 3588 USB camera video with S3 JSON frames on shared wall clock and render 1920x720 MP4."""
    frames = load_s3_frames(jsonl_path)
    if not frames:
        raise ValueError(f"No S3 frames found in {jsonl_path}")
    s3_times_ms = [f.get("host_wall_ms") for f in frames]
    if any(not isinstance(t, (int, float)) for t in s3_times_ms):
        raise ValueError("S3 JSONL frames must contain 'host_wall_ms' (run measure_live.py --stamp-host-time)")

    meta = load_camera_meta(camera_meta_path)
    cap = cv2.VideoCapture(str(camera_video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera video: {camera_video_path}")
    src_fps = cap.get(cv2.CAP_PROP_FPS)
    if not src_fps or src_fps <= 0 or math.isnan(src_fps):
        src_fps = float(meta.get("fps") or 30.0)
    fps = float(output_fps or src_fps)

    if summary_json_path is not None and Path(summary_json_path).exists():
        cap.release()
        raise FileExistsError(f"Refusing to overwrite existing summary: {summary_json_path}")

    trails = {}
    last_applied_s3_idx = -1
    cam_frames = 0
    synced_frames = 0
    locked_synced_frames = 0
    abs_skews = []
    t0_ms = float(meta["start_wall_ms"])

    try:
        with _VideoSink(output_mp4, PANEL_W * 2, PANEL_H, fps) as sink:
            while True:
                ok, bgr = cap.read()
                if not ok:
                    break
                cam_ms = camera_frame_wall_ms(meta, cam_frames, src_fps)
                elapsed_s = (cam_ms - t0_ms) / 1000.0
                s3_idx, dt_ms = align_timestamp_to_s3(cam_ms, s3_times_ms, max_skew_ms=max_skew_ms)
                s3_frame = None
                if s3_idx is not None:
                    synced_frames += 1
                    abs_skews.append(abs(dt_ms))
                    if s3_idx > last_applied_s3_idx:
                        for step_i in range(last_applied_s3_idx + 1, s3_idx + 1):
                            _update_trails(trails, frames[step_i])
                        last_applied_s3_idx = s3_idx
                    s3_frame = frames[s3_idx]
                    if s3_frame.get("state") == "active" and s3_frame.get("locked", 0) > 0:
                        locked_synced_frames += 1

                left_panel = _fit_camera_panel(bgr, PANEL_W, PANEL_H)
                _overlay_camera_hud(left_panel, elapsed_s, s3_frame, dt_ms)
                right_panel = render_s3_panel(
                    s3_frame, trails, elapsed_s=elapsed_s, sync_dt_ms=dt_ms,
                )
                combined = np.hstack((left_panel, right_panel))
                sink.write(combined)
                cam_frames += 1
    finally:
        cap.release()

    abs_skews.sort()
    summary = {
        "camera_frames": cam_frames,
        "camera_fps": round(fps, 3),
        "s3_frames": len(frames),
        "s3_active_frames": sum(1 for f in frames if f.get("state") == "active"),
        "s3_locked_frames": sum(1 for f in frames if f.get("state") == "active" and f.get("locked", 0) > 0),
        "synced_camera_frames": synced_frames,
        "locked_synced_camera_frames": locked_synced_frames,
        "max_skew_ms_limit": max_skew_ms,
        "skew_p50_ms": round(abs_skews[len(abs_skews) // 2], 3) if abs_skews else None,
        "skew_p95_ms": round(abs_skews[int((len(abs_skews) - 1) * 0.95)], 3) if abs_skews else None,
        "skew_max_ms": round(abs_skews[-1], 3) if abs_skews else None,
        "tracks": _summarize_tracks(frames),
    }
    if summary_json_path is not None:
        Path(summary_json_path).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jsonl", required=True, type=Path, help="Path to S3 live_w5500_timing.jsonl")
    parser.add_argument("--output-s3", type=Path, help="Output MP4 path for standalone S3 JSON render")
    parser.add_argument("--s3-fps", type=float, default=9.65, help="FPS for standalone S3 video")
    parser.add_argument("--camera-video", type=Path, help="Path to 3588 USB camera video (MKV/MP4)")
    parser.add_argument("--camera-meta", type=Path, help="Path to camera_meta.json with start_wall_ms")
    parser.add_argument("--output-sync", type=Path, help="Output MP4 path for side-by-side synced video")
    parser.add_argument("--summary-json", type=Path, help="Optional path to write sync_summary.json")
    parser.add_argument("--max-skew-ms", type=float, default=250.0, help="Max allowed sync skew in ms")
    args = parser.parse_args()

    if not args.output_s3 and not args.output_sync:
        parser.error("Specify at least one of --output-s3 or --output-sync")

    results = {}
    if args.output_s3:
        results["s3_standalone"] = render_s3_standalone(args.jsonl, args.output_s3, fps=args.s3_fps)
    if args.output_sync:
        if not args.camera_video or not args.camera_meta:
            parser.error("--output-sync requires both --camera-video and --camera-meta")
        results["sync"] = render_synced_side_by_side(
            args.jsonl,
            args.camera_video,
            args.camera_meta,
            args.output_sync,
            summary_json_path=args.summary_json,
            max_skew_ms=args.max_skew_ms,
        )
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
