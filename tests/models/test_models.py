"""
Unit tests for the data models.
Verifies contracts — field validation, computed properties.
"""

import math
import pytest

from maritime_perception.models.common import Header, SensorSource, now_ns
from maritime_perception.models.observation import DetectionObservation
from maritime_perception.models.world_model import WorldObject


def make_header():
    return Header(
        timestamp_ns = now_ns(),
        sensor_id    = "test",
        frame_id     = "vessel",
        source       = SensorSource.LIDAR,
    )


class TestDetectionObservation:

    def test_confidence_clamped_above_1(self):
        obs = DetectionObservation(
            header=make_header(), position_x=1.0, position_y=0.0,
            size_m=0.5, range_m=1.0, bearing_deg=0.0,
            point_count=10, confidence=1.5,   # above 1.0
        )
        assert obs.confidence == 1.0

    def test_confidence_clamped_below_0(self):
        obs = DetectionObservation(
            header=make_header(), position_x=1.0, position_y=0.0,
            size_m=0.5, range_m=1.0, bearing_deg=0.0,
            point_count=10, confidence=-0.5,   # below 0.0
        )
        assert obs.confidence == 0.0

    def test_negative_size_raises(self):
        with pytest.raises(ValueError):
            DetectionObservation(
                header=make_header(), position_x=1.0, position_y=0.0,
                size_m=-1.0, range_m=1.0, bearing_deg=0.0,
                point_count=10, confidence=0.5,
            )


class TestWorldObject:

    def make_obj(self, px=3.0, py=4.0, vx=0.0, vy=0.0):
        return WorldObject(
            id=1, header=make_header(),
            position_x=px, position_y=py,
            velocity_x=vx, velocity_y=vy,
            heading_deg=0.0, size_m=1.0,
            confidence=0.9, position_std_m=0.2,
            dynamic=False, coasting=False,
            sources=SensorSource.LIDAR,
        )

    def test_range_computed_correctly(self):
        obj = self.make_obj(px=3.0, py=4.0)
        assert abs(obj.range_m - 5.0) < 0.001   # 3-4-5 triangle

    def test_speed_computed_correctly(self):
        obj = self.make_obj(vx=3.0, vy=4.0)
        assert abs(obj.speed_ms - 5.0) < 0.001

    def test_safety_radius_includes_uncertainty(self):
        obj = self.make_obj()
        # safety_radius = size_m + position_std_m = 1.0 + 0.2 = 1.2
        assert abs(obj.safety_radius_m - 1.2) < 0.001

    def test_bearing_of_object_ahead(self):
        obj = self.make_obj(px=10.0, py=0.0)
        assert abs(obj.bearing_deg) < 1.0   # dead ahead = 0°

    def test_bearing_of_object_to_port(self):
        obj = self.make_obj(px=0.0, py=5.0)
        assert abs(obj.bearing_deg - 90.0) < 1.0   # port = 90°