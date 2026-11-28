"""Self-contained ROS1 PointCloud2 parser and IFW192S 192x256 native grid mapper.

Depends only on Python stdlib + NumPy (compatible with Python 3.8+ / NumPy 1.17.4+).
Preserves original measured XYZ coordinates and exact source_row indices without interpolation.
"""
import hashlib
import json
from pathlib import Path
import struct
from time import perf_counter
from types import SimpleNamespace

import numpy as np

from native_tracker import K_CELLS, K_HEIGHT, K_PRESENCE_BYTES, K_WIDTH, MEASUREMENT_DTYPE

CALIBRATION_HEADER_BYTES = 16
CALIBRATION_BIN_BYTES = CALIBRATION_HEADER_BYTES + K_CELLS * 2 * 8
MAX_CLOUD_BYTES = 32 * 1024 * 1024
MAX_POINTS = 262_144
_BIT_WEIGHTS = np.array([1, 2, 4, 8, 16, 32, 64, 128], dtype=np.uint8)


def parse_pointcloud2(raw):
    """Parse a serialized ROS1 sensor_msgs/PointCloud2 payload with strict bounds."""
    if not isinstance(raw, (bytes, bytearray, memoryview)):
        raise ValueError("PointCloud2 payload must be bytes-like")
    total_len = len(raw)
    if total_len > MAX_CLOUD_BYTES:
        raise ValueError("oversized PointCloud2 payload")
    offset = 0

    def take(fmt):
        nonlocal offset
        size = struct.calcsize(fmt)
        if offset + size > total_len:
            raise ValueError("truncated PointCloud2 payload")
        value = struct.unpack_from(fmt, raw, offset)
        offset += size
        return value

    def take_string():
        nonlocal offset
        (size,) = take("<I")
        if size > 4096 or offset + size > total_len:
            raise ValueError("invalid ROS string in PointCloud2")
        text = bytes(raw[offset:offset + size]).decode("utf-8")
        offset += size
        return text

    seq, secs, nsecs = take("<III")
    frame_id = take_string()
    height, width, field_count = take("<III")
    if (field_count > 128 or nsecs >= 1_000_000_000 or not frame_id
            or height == 0 or width == 0 or width * height > MAX_POINTS):
        raise ValueError("invalid PointCloud2 header or dimensions")

    fields = []
    for _ in range(field_count):
        name = take_string()
        field_offset, datatype, count = take("<IBI")
        fields.append(SimpleNamespace(
            name=name, offset=field_offset, datatype=datatype, count=count,
        ))

    is_bigendian, point_step, row_step, data_size = take("<?III")
    if data_size > MAX_CLOUD_BYTES or offset + data_size + 1 != total_len:
        raise ValueError("PointCloud2 data payload length mismatch")
    data = memoryview(raw)[offset:offset + data_size]
    offset += data_size
    (is_dense,) = take("<?")

    return SimpleNamespace(
        header=SimpleNamespace(
            seq=seq,
            frame_id=frame_id,
            stamp=SimpleNamespace(secs=secs, nsecs=nsecs),
        ),
        height=height,
        width=width,
        fields=fields,
        is_bigendian=is_bigendian,
        point_step=point_step,
        row_step=row_step,
        data=data,
        is_dense=is_dense,
    )


def extract_xyz_ring(cloud):
    """Extract contiguous (x, y, z) float32 and ring uint16 preserving original row order."""
    width = int(cloud.width)
    height = int(cloud.height)
    step = int(cloud.point_step)
    stride = int(cloud.row_step)
    if (width < 0 or height <= 0 or width * height > MAX_POINTS or step <= 0
            or stride < width * step or stride * height > 64 * 1024 * 1024):
        raise ValueError("invalid PointCloud2 dimensions or strides")
    data = memoryview(cloud.data)
    if not data.c_contiguous or data.nbytes != stride * height:
        raise ValueError("PointCloud2 buffer contiguity or size mismatch")

    field_map = {}
    for field in cloud.fields:
        if field.name in field_map:
            raise ValueError(f"duplicate PointCloud2 field: {field.name}")
        field_map[field.name] = field

    endian = ">" if cloud.is_bigendian else "<"
    required = (("x", 7, "f4"), ("y", 7, "f4"), ("z", 7, "f4"), ("ring", 4, "u2"))
    names = []
    formats = []
    offsets = []
    intervals = []
    for name, expected_type, fmt_code in required:
        field = field_map.get(name)
        if field is None or field.count != 1 or field.datatype != expected_type:
            raise ValueError(f"missing or unsupported PointCloud2 field: {name}")
        dtype = np.dtype(endian + fmt_code)
        start = int(field.offset)
        end = start + dtype.itemsize
        if start < 0 or end > step or any(start < b and a < end for a, b in intervals):
            raise ValueError(f"out-of-bounds or overlapping field: {name}")
        names.append(name)
        formats.append(dtype)
        offsets.append(start)
        intervals.append((start, end))

    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": step})
    view = np.ndarray((height, width), dtype=dtype, buffer=data, strides=(stride, step))
    x = np.ascontiguousarray(view["x"].reshape(-1), dtype="<f4")
    y = np.ascontiguousarray(view["y"].reshape(-1), dtype="<f4")
    z = np.ascontiguousarray(view["z"].reshape(-1), dtype="<f4")
    ring = np.ascontiguousarray(view["ring"].reshape(-1), dtype="<u2")
    return x, y, z, ring


class IFW192SScanGrid:
    """Calibrated 192x256 IFW192S ray table mapping PointCloud2 rows to native tracker cells."""

    def __init__(self, azimuth_deg, elevation_deg, tolerance_deg=0.02):
        az = np.array(azimuth_deg, dtype=np.float64, copy=True)
        el = np.array(elevation_deg, dtype=np.float64, copy=True)
        tolerance = float(tolerance_deg)
        if (az.shape != (K_HEIGHT, K_WIDTH) or el.shape != (K_HEIGHT, K_WIDTH)
                or not np.isfinite(az).all() or not np.isfinite(el).all()
                or (np.abs(az) >= 180.0).any() or (np.abs(el) > 90.0).any()
                or not np.isfinite(tolerance) or tolerance <= 0.0):
            raise ValueError("invalid IFW192S 192x256 direction table or tolerance")

        order = np.argsort(az, axis=1)
        sorted_az = np.take_along_axis(az, order, axis=1)
        if (np.diff(sorted_az, axis=1) <= 2.0 * tolerance).any():
            raise ValueError("azimuth directions are ambiguous within tolerance")
        if not ((np.diff(az, axis=1) > 0).all() or (np.diff(az, axis=1) < 0).all()):
            raise ValueError("columns must have a consistent angular order")

        self.shape = (K_HEIGHT, K_WIDTH)
        self.tolerance_deg = tolerance
        self.azimuth_deg = az
        self.elevation_deg = el
        self._az_flat = az.ravel()
        self._el_flat = el.ravel()
        self._keys = (sorted_az + 360.0 * np.arange(K_HEIGHT)[:, None]).ravel()
        self._cells = (order + K_WIDTH * np.arange(K_HEIGHT)[:, None]).ravel()

        self._lookup_low = int(np.floor(az.min() * 100.0)) - 2
        lookup_width = int(np.ceil(az.max() * 100.0)) - self._lookup_low + 3
        self._lookup = np.full((K_HEIGHT, lookup_width), -1, dtype=np.int32)
        ring_idx, col_idx = np.indices(self.shape)
        centres = np.rint(az * 100.0).astype(np.int64) - self._lookup_low
        span = int(np.ceil(tolerance * 100.0)) + 1
        for delta in range(-span, span + 1):
            bins = centres + delta
            ok = (bins >= 0) & (bins < lookup_width)
            self._lookup[ring_idx[ok], bins[ok]] = col_idx[ok]

        az_rad = np.radians(self._az_flat)
        el_rad = np.radians(self._el_flat)
        cos_el = np.cos(el_rad)
        self.unit_rays = np.ascontiguousarray(
            np.column_stack((cos_el * np.cos(az_rad), cos_el * np.sin(az_rad), np.sin(el_rad))),
            dtype="<f8",
        )

        digest = hashlib.sha256()
        digest.update(az.astype("<f8").tobytes())
        digest.update(el.astype("<f8").tobytes())
        digest.update(json.dumps({"shape": list(self.shape), "tolerance_deg": tolerance}, sort_keys=True).encode())
        self.sha256 = digest.hexdigest()

        for arr in (self.azimuth_deg, self.elevation_deg, self._keys, self._cells, self._lookup, self.unit_rays):
            arr.flags.writeable = False

    @classmethod
    def from_csv(cls, azimuth_path, elevation_path, tolerance_deg=0.02):
        az = np.loadtxt(Path(azimuth_path), delimiter=",")
        el = np.loadtxt(Path(elevation_path), delimiter=",")
        return cls(az, el, tolerance_deg=tolerance_deg)

    @classmethod
    def from_calibration_bin(cls, bin_path, tolerance_deg=0.02):
        raw = Path(bin_path).read_bytes()
        if len(raw) != CALIBRATION_BIN_BYTES:
            raise ValueError(
                f"Expected {CALIBRATION_BIN_BYTES}-byte calibration binary, got {len(raw)} bytes"
            )
        angles = np.frombuffer(raw[CALIBRATION_HEADER_BYTES:], dtype="<f8").reshape(K_HEIGHT, K_WIDTH, 2)
        return cls(angles[:, :, 0], angles[:, :, 1], tolerance_deg=tolerance_deg)

    def decode_cloud_bytes(self, raw_payload):
        """Parse ROS1 PointCloud2 bytes and project to native tracker arrays."""
        t0 = perf_counter()
        cloud = parse_pointcloud2(raw_payload)
        points, presence, stats = self.decode_cloud(cloud)
        stats["decode_ms"] = round((perf_counter() - t0) * 1000.0, 3)
        return cloud, points, presence, stats

    def decode_cloud(self, cloud):
        """Map parsed PointCloud2 into contiguous Measurement[49152] + presence[6144]."""
        x_f32, y_f32, z_f32, ring_u16 = extract_xyz_ring(cloud)
        return self.encode_xyz_ring(x_f32, y_f32, z_f32, ring_u16)

    def encode_xyz_ring(self, x_f32, y_f32, z_f32, ring_u16):
        """Deterministic nearest-echo mapping; ties broken by smallest source_row."""
        x64 = x_f32.astype(np.float64)
        y64 = y_f32.astype(np.float64)
        z64 = z_f32.astype(np.float64)
        ranges = np.sqrt(x64 * x64 + y64 * y64 + z64 * z64)
        finite = np.isfinite(ranges)
        positive = np.zeros(len(ranges), dtype=bool)
        positive[finite] = ranges[finite] > 0.1
        rings = ring_u16.astype(np.int64)
        eligible = positive & (rings < K_HEIGHT)
        rows = np.flatnonzero(eligible)

        points = np.zeros(K_CELLS, dtype=MEASUREMENT_DTYPE)
        valid_flat = np.zeros(K_CELLS, dtype=np.uint8)
        if len(rows) == 0:
            presence = np.zeros(K_PRESENCE_BYTES, dtype=np.uint8)
            return points, presence, {
                "total_points": int(len(x_f32)),
                "valid_cells": 0,
                "unknown_direction_points": 0,
                "duplicate_points": 0,
            }

        az = np.degrees(np.arctan2(y64[rows], x64[rows]))
        el = np.degrees(np.arctan2(z64[rows], np.hypot(x64[rows], y64[rows])))
        bins = np.rint(az * 100.0).astype(np.int64) - self._lookup_low
        lookup_w = self._lookup.shape[1]
        in_lookup = (bins >= 0) & (bins < lookup_w)
        columns = self._lookup[rings[rows], np.clip(bins, 0, lookup_w - 1)]
        cells = rings[rows] * K_WIDTH + np.maximum(columns, 0)
        known = (
            in_lookup
            & (columns >= 0)
            & (np.abs(self._az_flat[cells] - az) <= self.tolerance_deg)
            & (np.abs(self._el_flat[cells] - el) <= self.tolerance_deg)
        )

        if not known.all():
            bad = np.flatnonzero(~known)
            query = az[bad] + 360.0 * rings[rows[bad]]
            right = np.minimum(np.searchsorted(self._keys, query), len(self._keys) - 1)
            left = np.maximum(right - 1, 0)
            nearest = np.where(
                np.abs(self._keys[left] - query) <= np.abs(self._keys[right] - query),
                left,
                right,
            )
            candidate = self._cells[nearest]
            accepted = (
                (candidate // K_WIDTH == rings[rows[bad]])
                & (np.abs(self._az_flat[candidate] - az[bad]) <= self.tolerance_deg)
                & (np.abs(self._el_flat[candidate] - el[bad]) <= self.tolerance_deg)
            )
            cells[bad[accepted]] = candidate[accepted]
            known[bad[accepted]] = True

        unknown_count = int((~known).sum())
        rows = rows[known]
        cells = cells[known]

        duplicate_count = 0
        if len(cells) > 0:
            counts = np.bincount(cells, minlength=K_CELLS)
            if (counts > 1).any():
                order = np.lexsort((rows, ranges[rows], cells))
                ordered_cells = cells[order]
                first = np.r_[True, ordered_cells[1:] != ordered_cells[:-1]]
                duplicate_count = int((~first).sum())
                rows = rows[order[first]]
                cells = cells[order[first]]

            points["x"][cells] = x_f32[rows]
            points["y"][cells] = y_f32[rows]
            points["z"][cells] = z_f32[rows]
            points["source_row"][cells] = rows.astype(np.uint32)
            valid_flat[cells] = 1

        presence = (valid_flat.reshape(K_PRESENCE_BYTES, 8) * _BIT_WEIGHTS).sum(axis=1, dtype=np.uint8)
        return points, presence, {
            "total_points": int(len(x_f32)),
            "valid_cells": int(len(cells)),
            "unknown_direction_points": unknown_count,
            "duplicate_points": duplicate_count,
        }
