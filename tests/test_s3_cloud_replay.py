import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from web.s3_cloud_replay import _cloud_snapshot, _paired_rows


class TestS3CloudReplay(unittest.TestCase):
    def test_paired_rows_reject_duplicates_and_unmatched_timing(self):
        mapping = dict(input_ordinal=303, ordinal=10, seq=55, locked=0, tracker_us=14000, ready_us=46000)
        result = dict(phase='ACTIVE', locked_birds=[])
        uart_frame = dict(type='frame', ordinal=10, tracker_us=14000, result=result)
        uart_timing = dict(type='paced_timing', ordinal=10, ready_us=46000)
        with tempfile.TemporaryDirectory() as directory:
            timings, uart = (Path(directory) / name for name in ('timings.jsonl', 'uart.bin'))
            timings.write_text(json.dumps(mapping) + '\n', encoding='utf-8')
            uart.write_bytes(b'\n'.join(json.dumps(row).encode() for row in (uart_frame, uart_timing)))
            self.assertEqual(_paired_rows(timings, uart), {303: (mapping, result)})
            timings.write_text((json.dumps(mapping) + '\n') * 2, encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'repeated input_ordinal'):
                _paired_rows(timings, uart)
            timings.write_text(json.dumps(mapping) + '\n', encoding='utf-8')
            uart.write_bytes(json.dumps(uart_frame).encode())
            with self.assertRaisesRegex(ValueError, 'must have the same ordinals'):
                _paired_rows(timings, uart)

    def test_snapshot_keeps_original_measured_xyz_and_checks_uart_source_rows(self):
        rows = np.array([(1, 0, 0, 4), (3, 0, 0, 4), (8, 0, 1, 5), (0, 0, 0, 0), (np.nan, 1, 1, 5)],
                        dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('ring', 'u2')])
        mapping = dict(input_ordinal=307, seq=55, ready_us=47000, tracker_us=16000)
        track = dict(track_id=4, centroid_xyz_m=[2, 0, 0], point_count=2, source_rows=[0, 1],
                     cells=[1024, 1025], grid_box_gx0_gy0_gx1_gy1=[0, 4, 1, 4])
        result = dict(locked_birds=[track], dynamic_candidates=1, tentative_count=0)
        frame, xyz = _cloud_snapshot(mapping, result, rows)
        self.assertEqual(frame['cloud_points'], 3)
        self.assertEqual(frame['tracks'], [dict(id=4, xyz_m=[2, 0, 0], points=2,
                                               box=[0, 4, 1, 4], range_m=2.0)])
        self.assertEqual(frame['latency_ms'], 47)
        self.assertEqual(xyz.dtype, np.dtype('<f4'))
        np.testing.assert_array_equal(xyz, [[1, 0, 0], [3, 0, 0], [8, 0, 1]])
        track['source_rows'] = [0, 2]
        with self.assertRaisesRegex(ValueError, 'source rows do not match'):
            _cloud_snapshot(mapping, result, rows)
