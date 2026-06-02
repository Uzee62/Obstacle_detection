"""
Unit tests for sensors/lidar/preprocessor.py

Tests
-----
- Invalid points are rejected (zero, NaN, inf)
- Range gate works correctly
- FOV mask blanks the right sectors
- Polar to cartesian conversion is correct
- Mounting angle offset is applied
"""

import math
import pytest

from maritime_perception.sensors.lidar.driver import RawScanPoint, LidarScan
from maritime_perception.sensors.lidar.preprocessor import (
    LidarPreprocessor,
    PreprocessorConfig,
)
from maritime_perception.models.common import Header, SensorSource, now_ns


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_header():
    return Header(
        timestamp_ns = now_ns(),
        sensor_id    = "test",
        frame_id     = "lidar",
        source       = SensorSource.LIDAR,
    )


def make_scan(points: list[tuple]) -> LidarScan:
    """
    points: list of (angle_deg, distance_m, quality)
    """
    return LidarScan(
        header  = make_header(),
        points  = [
            RawScanPoint(angle_deg=a, distance_m=d, quality=q)
            for a, d, q in points
        ],
        scan_id = 1,
    )


def default_config(**kwargs) -> PreprocessorConfig:
    base = dict(
        range_min_m      = 0.3,
        range_max_m      = 30.0,
        min_intensity    = 0,
        angle_offset_deg = 0.0,
        fov_mask         = [],
    )
    base.update(kwargs)
    return PreprocessorConfig(**base)


# ---------------------------------------------------------------------------
# Validity gate
# ---------------------------------------------------------------------------

class TestValidityGate:

    def test_zero_distance_rejected(self):
        scan = make_scan([(45.0, 0.0, 15)])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 0

    def test_negative_distance_rejected(self):
        scan = make_scan([(45.0, -1.0, 15)])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 0

    def test_nan_distance_rejected(self):
        scan = make_scan([(45.0, float('nan'), 15)])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 0

    def test_inf_distance_rejected(self):
        scan = make_scan([(45.0, float('inf'), 15)])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 0

    def test_valid_point_passes(self):
        scan = make_scan([(45.0, 5.0, 15)])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 1


# ---------------------------------------------------------------------------
# Range gate
# ---------------------------------------------------------------------------

class TestRangeGate:

    def test_below_min_rejected(self):
        scan = make_scan([(0.0, 0.1, 15)])   # 0.1m < 0.3m min
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 0

    def test_above_max_rejected(self):
        scan = make_scan([(0.0, 35.0, 15)])  # 35m > 30m max
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 0

    def test_at_min_boundary_passes(self):
        scan = make_scan([(0.0, 0.3, 15)])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 1

    def test_at_max_boundary_passes(self):
        scan = make_scan([(0.0, 30.0, 15)])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 1

    def test_mixed_valid_and_invalid(self):
        scan = make_scan([
            (0.0,   0.1, 15),   # too close
            (10.0,  5.0, 15),   # valid
            (20.0, 35.0, 15),   # too far
            (30.0,  8.0, 15),   # valid
        ])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 2


# ---------------------------------------------------------------------------
# FOV mask
# ---------------------------------------------------------------------------

class TestFOVMask:

    def test_masked_angle_rejected(self):
        cfg  = default_config(fov_mask=[(178.0, 182.0)])
        scan = make_scan([(180.0, 5.0, 15)])   # right in the mask
        proc = LidarPreprocessor(cfg)
        out  = proc.process(scan)
        assert len(out) == 0

    def test_outside_mask_passes(self):
        cfg  = default_config(fov_mask=[(178.0, 182.0)])
        scan = make_scan([(90.0, 5.0, 15)])    # nowhere near mask
        proc = LidarPreprocessor(cfg)
        out  = proc.process(scan)
        assert len(out) == 1

    def test_mask_boundary_rejected(self):
        cfg  = default_config(fov_mask=[(178.0, 182.0)])
        scan = make_scan([(178.0, 5.0, 15), (182.0, 5.0, 15)])
        proc = LidarPreprocessor(cfg)
        out  = proc.process(scan)
        assert len(out) == 0

    def test_wraparound_mask(self):
        # sector wraps around 0° — e.g. 350° to 10°
        cfg  = default_config(fov_mask=[(350.0, 10.0)])
        scan = make_scan([
            (0.0,   5.0, 15),   # inside wraparound mask
            (5.0,   5.0, 15),   # inside
            (180.0, 5.0, 15),   # outside
        ])
        proc = LidarPreprocessor(cfg)
        out  = proc.process(scan)
        assert len(out) == 1   # only the 180° point passes


# ---------------------------------------------------------------------------
# Polar to cartesian
# ---------------------------------------------------------------------------

class TestPolarToCartesian:

    def test_zero_degrees_is_forward(self):
        # angle=0° should give x=distance, y≈0
        scan = make_scan([(0.0, 10.0, 15)])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 1
        assert abs(out[0].x - 10.0) < 0.01
        assert abs(out[0].y) < 0.01

    def test_90_degrees_is_port(self):
        # angle=90° should give x≈0, y=distance
        scan = make_scan([(90.0, 10.0, 15)])
        proc = LidarPreprocessor(default_config())
        out  = proc.process(scan)
        assert len(out) == 1
        assert abs(out[0].x) < 0.01
        assert abs(out[0].y - 10.0) < 0.01

    def test_mounting_offset_applied(self):
        # with 90° offset, a 0° reading should map to port
        cfg  = default_config(angle_offset_deg=90.0)
        scan = make_scan([(0.0, 10.0, 15)])
        proc = LidarPreprocessor(cfg)
        out  = proc.process(scan)
        assert len(out) == 1
        # after +90° offset, angle becomes 90° → x≈0, y=10
        assert abs(out[0].x) < 0.01
        assert abs(out[0].y - 10.0) < 0.01


# ---------------------------------------------------------------------------
# Empty scan
# ---------------------------------------------------------------------------

def test_empty_scan_returns_empty():
    scan = make_scan([])
    proc = LidarPreprocessor(default_config())
    out  = proc.process(scan)
    assert out == []