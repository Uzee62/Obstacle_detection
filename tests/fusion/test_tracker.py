"""
Unit tests for fusion/tracker.py

Tests the track lifecycle:
  TENTATIVE → CONFIRMED → COASTING → DEAD
"""

import pytest
import time

from maritime_perception.fusion.tracker import FusionTracker
from maritime_perception.models.common import Header, SensorSource, now_ns
from maritime_perception.models.observation import DetectionObservation
from maritime_perception.models.track import TrackState


def make_header():
    return Header(
        timestamp_ns = now_ns(),
        sensor_id    = "test_lidar",
        frame_id     = "vessel",
        source       = SensorSource.LIDAR,
    )


def make_obs(x=5.0, y=0.0, size=1.0, conf=0.8):
    import math
    return DetectionObservation(
        header      = make_header(),
        position_x  = x,
        position_y  = y,
        size_m      = size,
        range_m     = math.hypot(x, y),
        bearing_deg = math.degrees(math.atan2(y, x)),
        point_count = 20,
        confidence  = conf,
    )


def make_tracker(min_hits=3, max_miss_tent=3, max_miss_conf=5, max_miss_coast=3):
    cfg = {
        "tracking": {
            "gate_mahalanobis"    : 9.21,
            "min_hits_confirm"    : min_hits,
            "max_misses_tentative": max_miss_tent,
            "max_misses_confirmed": max_miss_conf,
            "max_misses_coast"    : max_miss_coast,
            "min_confidence_output": 0.1,
            "kalman": {
                "process_noise_pos": 0.1,
                "process_noise_vel": 0.5,
                "measurement_noise": 0.3,
            },
        },
        "fusion": {"cycle_rate_hz": 10.0},
    }
    return FusionTracker(cfg)


def cycle_header():
    return Header(
        timestamp_ns = now_ns(),
        sensor_id    = "fusion",
        frame_id     = "vessel",
        source       = SensorSource.LIDAR,
    )


class TestTrackLifecycle:

    def test_track_confirms_after_min_hits(self):
        tracker = make_tracker(min_hits=3)
        obs     = make_obs(x=5.0, y=0.0)

        # Feed same observation 3 times
        for i in range(3):
            world = tracker.update([obs], cycle_header(), scan_id=i)

        # Should now have one confirmed track
        assert tracker.confirmed_count == 1
        assert len(world) == 1

    def test_tentative_track_not_published(self):
        tracker = make_tracker(min_hits=3)
        obs     = make_obs(x=5.0, y=0.0)

        # Only one hit — still tentative
        world = tracker.update([obs], cycle_header(), scan_id=1)
        assert len(world) == 0

    def test_tentative_track_dies_on_misses(self):
        tracker = make_tracker(min_hits=3, max_miss_tent=3)
        obs     = make_obs(x=5.0, y=0.0)

        # One hit to spawn tentative track
        tracker.update([obs], cycle_header(), scan_id=1)
        assert tracker.track_count == 1

        # Three empty scans — should die
        for i in range(2, 5):
            tracker.update([], cycle_header(), scan_id=i)

        assert tracker.track_count == 0

    def test_confirmed_track_enters_coast_on_misses(self):
        tracker = make_tracker(min_hits=3, max_miss_conf=3)
        obs     = make_obs(x=5.0, y=0.0)

        # Confirm the track
        for i in range(3):
            tracker.update([obs], cycle_header(), scan_id=i)
        assert tracker.confirmed_count == 1

        # Miss enough scans to enter coast
        for i in range(3, 7):
            tracker.update([], cycle_header(), scan_id=i)

        # Track should still exist but be coasting
        assert tracker.track_count > 0

    def test_track_recovers_from_coast(self):
        tracker = make_tracker(min_hits=3, max_miss_conf=3)
        obs     = make_obs(x=5.0, y=0.0)

        # Confirm
        for i in range(3):
            tracker.update([obs], cycle_header(), scan_id=i)

        # Miss — enter coast
        for i in range(3, 6):
            tracker.update([], cycle_header(), scan_id=i)

        # Observation returns — should recover
        world = tracker.update([obs], cycle_header(), scan_id=7)
        assert len(world) == 1

    def test_two_separate_obstacles_get_separate_tracks(self):
        tracker = make_tracker(min_hits=3)
        obs_a   = make_obs(x=5.0,  y=0.0)
        obs_b   = make_obs(x=0.0,  y=8.0)

        for i in range(3):
            tracker.update([obs_a, obs_b], cycle_header(), scan_id=i)

        assert tracker.confirmed_count == 2

    def test_reset_clears_all_tracks(self):
        tracker = make_tracker(min_hits=3)
        obs     = make_obs(x=5.0, y=0.0)

        for i in range(3):
            tracker.update([obs], cycle_header(), scan_id=i)

        assert tracker.confirmed_count == 1
        tracker.reset()
        assert tracker.track_count == 0


class TestVelocityEstimation:

    def test_stationary_obstacle_has_near_zero_velocity(self):
        tracker = make_tracker(min_hits=3)

        # Same position every scan
        for i in range(10):
            obs   = make_obs(x=5.0, y=0.0)
            world = tracker.update([obs], cycle_header(), scan_id=i)

        assert len(world) == 1
        obj = world.objects[0]
        assert obj.speed_ms < 0.2   # near zero with some Kalman noise

    def test_moving_obstacle_has_nonzero_velocity(self):
        tracker = make_tracker(min_hits=3)

        # Obstacle moves 0.5m per scan in x direction
        for i in range(15):
            obs   = make_obs(x=5.0 + i * 0.5, y=0.0)
            world = tracker.update([obs], cycle_header(), scan_id=i)

        assert len(world) == 1
        obj = world.objects[0]
        assert obj.speed_ms > 0.5   # should detect movement