"""
Unit tests for sensors/lidar/segmentation.py
"""

import math
import pytest

from maritime_perception.sensors.lidar.preprocessor import CartesianPoint
from maritime_perception.sensors.lidar.segmentation import (
    JumpDistanceSegmenter,
    SegmentationConfig,
)


def default_config(**kwargs):
    base = dict(
        jump_distance_m  = 0.5,
        adaptive_k       = 0.0,   # disable adaptive for predictable tests
        min_points       = 3,
        min_arc_deg      = 0.0,   # disable arc filter for clean tests
        max_points       = 2000,
        merge_distance_m = 0.1,   # tight merge — don't merge test clusters
    )
    base.update(kwargs)
    return SegmentationConfig(**base)


def make_point(angle, dist, x, y):
    return CartesianPoint(
        angle_deg  = angle,
        distance_m = dist,
        x          = x,
        y          = y,
        quality    = 15,
    )


def make_cluster(cx, cy, n=5, spread=0.1):
    """Make n points clustered around (cx, cy)."""
    return [
        make_point(
            angle = math.degrees(math.atan2(cy, cx)),
            dist  = math.hypot(cx, cy),
            x     = cx + i * spread,
            y     = cy,
        )
        for i in range(n)
    ]


class TestJumpDistanceSplit:

    def test_two_clusters_split_correctly(self):
        # cluster A around x=2, cluster B around x=10
        # large gap between them triggers split
        cluster_a = make_cluster(2.0, 0.0, n=5, spread=0.05)
        cluster_b = make_cluster(10.0, 0.0, n=5, spread=0.05)
        points = cluster_a + cluster_b

        seg   = JumpDistanceSegmenter(default_config())
        segs  = seg.segment(points)
        assert len(segs) == 2

    def test_single_cluster_stays_together(self):
        points = make_cluster(5.0, 0.0, n=10, spread=0.05)
        seg    = JumpDistanceSegmenter(default_config())
        segs   = seg.segment(points)
        assert len(segs) == 1

    def test_min_points_filter(self):
        # two clusters but one has fewer than min_points
        cluster_a = make_cluster(2.0, 0.0, n=5)   # passes min_points=3
        cluster_b = make_cluster(10.0, 0.0, n=2)   # fails min_points=3
        points = cluster_a + cluster_b

        seg   = JumpDistanceSegmenter(default_config(min_points=3))
        segs  = seg.segment(points)
        assert len(segs) == 1   # only cluster_a survives

    def test_empty_input_returns_empty(self):
        seg  = JumpDistanceSegmenter(default_config())
        segs = seg.segment([])
        assert segs == []

    def test_three_clusters(self):
        a = make_cluster(2.0,  0.0, n=4)
        b = make_cluster(10.0, 0.0, n=4)
        c = make_cluster(20.0, 0.0, n=4)

        seg  = JumpDistanceSegmenter(default_config())
        segs = seg.segment(a + b + c)
        assert len(segs) == 3