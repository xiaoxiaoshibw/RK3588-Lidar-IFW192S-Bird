"""ctypes binding for the native C++17 esp32s3_bird_core shared library."""
import ctypes
from pathlib import Path
from time import perf_counter
import numpy as np

K_WIDTH = 256
K_HEIGHT = 192
K_CELLS = K_WIDTH * K_HEIGHT
K_PRESENCE_BYTES = K_CELLS // 8

MEASUREMENT_DTYPE = np.dtype([
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("source_row", "<u4"),
])
assert MEASUREMENT_DTYPE.itemsize == 16


class BirdTrackC(ctypes.Structure):
    _fields_ = [
        ("track_id", ctypes.c_uint32),
        ("reason", ctypes.c_uint32),
        ("box", ctypes.c_int32 * 4),
        ("center", ctypes.c_double * 3),
        ("range_m", ctypes.c_double),
        ("max_disp_m", ctypes.c_double),
        ("point_count", ctypes.c_uint32),
        ("cell_offset", ctypes.c_uint32),
    ]


class BirdStepResultC(ctypes.Structure):
    _fields_ = [
        ("ok", ctypes.c_uint32),
        ("warming_up", ctypes.c_uint32),
        ("warmup_progress", ctypes.c_int32),
        ("dynamic_candidates", ctypes.c_int32),
        ("skipped_large_parents", ctypes.c_int32),
        ("tentative_count", ctypes.c_int32),
        ("locked_count", ctypes.c_uint32),
        ("total_track_cells", ctypes.c_uint32),
    ]


assert ctypes.sizeof(BirdTrackC) == 72
assert ctypes.sizeof(BirdStepResultC) == 32


def _bind_library(lib_path):
    lib = ctypes.CDLL(str(lib_path))
    lib.bird_tracker_create.argtypes = []
    lib.bird_tracker_create.restype = ctypes.c_void_p

    lib.bird_tracker_destroy.argtypes = [ctypes.c_void_p]
    lib.bird_tracker_destroy.restype = None

    lib.bird_tracker_reset.argtypes = [ctypes.c_void_p]
    lib.bird_tracker_reset.restype = None

    lib.bird_tracker_clear_tracks.argtypes = [ctypes.c_void_p]
    lib.bird_tracker_clear_tracks.restype = None

    lib.bird_tracker_step.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.POINTER(BirdStepResultC),
    ]
    lib.bird_tracker_step.restype = ctypes.c_int

    lib.bird_tracker_copy_tracks.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(BirdTrackC),
        ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_uint16),
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.c_uint32,
    ]
    lib.bird_tracker_copy_tracks.restype = ctypes.c_uint32
    return lib


class NativeBirdTracker:
    """In-process wrapper around GridTracker from firmware/esp32s3_bird_core."""

    def __init__(self, lib_path, unit_rays):
        rays = np.ascontiguousarray(unit_rays, dtype="<f8")
        if rays.shape != (K_CELLS, 3) or not np.isfinite(rays).all():
            raise ValueError(f"Expected finite ({K_CELLS}, 3) float64 unit_rays, got {rays.shape}")
        self._lib_path = Path(lib_path)
        self._lib = _bind_library(self._lib_path)
        self._rays = rays
        self._handle = self._lib.bird_tracker_create()
        if not self._handle:
            raise MemoryError("Failed to allocate native BirdTracker handle")

    def close(self):
        if getattr(self, "_handle", None):
            self._lib.bird_tracker_destroy(self._handle)
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def reset(self):
        if not self._handle:
            raise RuntimeError("NativeBirdTracker is closed")
        self._lib.bird_tracker_reset(self._handle)

    def clear_tracks(self):
        if not self._handle:
            raise RuntimeError("NativeBirdTracker is closed")
        self._lib.bird_tracker_clear_tracks(self._handle)

    def step(self, points, presence, *, include_provenance=False):
        if not self._handle:
            raise RuntimeError("NativeBirdTracker is closed")
        pts = np.ascontiguousarray(points, dtype=MEASUREMENT_DTYPE)
        if pts.shape != (K_CELLS,):
            raise ValueError(f"Expected ({K_CELLS},) Measurement array, got {pts.shape}")
        pres = np.ascontiguousarray(presence, dtype=np.uint8)
        if pres.shape != (K_PRESENCE_BYTES,):
            raise ValueError(f"Expected ({K_PRESENCE_BYTES},) presence bitmap, got {pres.shape}")

        raw_result = BirdStepResultC()
        t0 = perf_counter()
        ok = self._lib.bird_tracker_step(
            self._handle,
            pts.ctypes.data,
            pres.ctypes.data,
            self._rays.ctypes.data,
            ctypes.byref(raw_result),
        )
        tracker_ms = round((perf_counter() - t0) * 1000.0, 3)

        if not ok or not raw_result.ok:
            return {
                "ok": False,
                "state": "UNKNOWN",
                "warmup": 0,
                "candidates": 0,
                "tentative": 0,
                "skipped_large_parents": 0,
                "locked": 0,
                "tracks": [],
                "tracker_ms": tracker_ms,
            }

        locked_count = int(raw_result.locked_count)
        total_cells = int(raw_result.total_track_cells)
        tracks = []
        if locked_count > 0:
            tracks_buf = (BirdTrackC * locked_count)()
            if include_provenance and total_cells > 0:
                cells_buf = (ctypes.c_uint16 * total_cells)()
                rows_buf = (ctypes.c_uint32 * total_cells)()
                self._lib.bird_tracker_copy_tracks(
                    self._handle, tracks_buf, locked_count, cells_buf, rows_buf, total_cells,
                )
            else:
                cells_buf = None
                rows_buf = None
                self._lib.bird_tracker_copy_tracks(
                    self._handle, tracks_buf, locked_count, None, None, 0,
                )
            for idx in range(locked_count):
                item = tracks_buf[idx]
                track = {
                    "id": int(item.track_id),
                    "reason": "retained" if item.reason == 1 else "dynamic",
                    "box": [int(item.box[i]) for i in range(4)],
                    "xyz_m": [round(float(item.center[i]), 6) for i in range(3)],
                    "range_m": round(float(item.range_m), 4),
                    "max_disp_m": round(float(item.max_disp_m), 4),
                    "points": int(item.point_count),
                }
                if include_provenance and cells_buf is not None and rows_buf is not None:
                    start = int(item.cell_offset)
                    end = start + int(item.point_count)
                    track["cells"] = [int(cells_buf[i]) for i in range(start, end)]
                    track["source_rows"] = [int(rows_buf[i]) for i in range(start, end)]
                tracks.append(track)

        return {
            "ok": True,
            "state": "warmup" if raw_result.warming_up else "active",
            "warmup": int(raw_result.warmup_progress),
            "candidates": int(raw_result.dynamic_candidates),
            "tentative": int(raw_result.tentative_count),
            "skipped_large_parents": int(raw_result.skipped_large_parents),
            "locked": locked_count,
            "tracks": tracks,
            "tracker_ms": tracker_ms,
        }
