"""
sensors/lidar/segmentation.py

Stage 3 — jump-distance segmentation + KD-tree Euclidean merge.

After the noise filter, we have a set of reliable Cartesian points 
that likely belong to real objects. The purpose of this stage is 
to group those points into obstacle candidates.

Algorithm 1: Jump-distance segmentation

Walk the angularly-ordered point list. If the Euclidean gap between
consecutive points exceeds an adaptive threshold, start a new segment.

Adaptive threshold:
    threshold = jump_distance_m + range_m × adaptive_k

This accounts for the natural angular spreading of points at range —
at 20m, adjacent 0.5° scan lines are further apart than at 5m.

Quality filters applied per segment:
- min_points     : minimum raw point count
- min_arc_deg    : minimum angular span (filters single-direction spikes)
- max_points     : safety cap

Algorithm 2: KD-tree single-linkage merge

After segmentation, nearby segment centroids are merged.
Uses scipy.spatial.cKDTree for O(n log n) nearest-neighbour lookup
instead of the O(n²) pairwise approach.

This handles vessel hulls that produce fragmented segments due to
superstructure geometry or brief scan gaps within one object.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

from .preprocessor import CartesianPoint

log = logging.getLogger(__name__)


# Segmentation Config (stores all parameters that control segmentation, loaded from YAML)


@dataclass(frozen=True)
class SegmentationConfig:
    jump_distance_m  : float
    adaptive_k       : float
    min_points       : int
    min_arc_deg      : float
    max_points       : int
    merge_distance_m : float

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> "SegmentationConfig":
        seg = cfg.get("segmentation", {})
        return cls(
            jump_distance_m  = float(seg.get("jump_distance_m", 0.5)),
            adaptive_k       = float(seg.get("adaptive_k", 0.02)),
            min_points       = int(seg.get("min_points", 3)),
            min_arc_deg      = float(seg.get("min_arc_deg", 0.5)),
            max_points       = int(seg.get("max_points", 2000)),
            merge_distance_m = float(seg.get("merge_distance_m", 1.0)),
        )


# Segment


@dataclass
class Segment:
    points: list[CartesianPoint] = field(default_factory=list)

    @property
    def size(self) -> int:
        return len(self.points)

    @property
    def centroid(self) -> tuple[float, float]:
        xs = [p.x for p in self.points]
        ys = [p.y for p in self.points]
        return sum(xs) / len(xs), sum(ys) / len(ys)

    @property
    def arc_span_deg(self) -> float:
        """Angular span of the segment in degrees."""
        if len(self.points) < 2:
            return 0.0
        angles = [p.angle_deg for p in self.points]
        return max(angles) - min(angles)

    @property
    def mean_range_m(self) -> float:
        return sum(p.distance_m for p in self.points) / len(self.points)


# Jump Distance Segmenter

class JumpDistanceSegmenter:
    """
    Segments an ordered LiDAR point cloud into obstacle candidates.
    Stateless — safe to call from any thread.
    """

    def __init__(self, config: SegmentationConfig) -> None:
        self._cfg = config

    def segment(self, points: list[CartesianPoint]) -> list[Segment]:
        """
        Full segmentation pipeline: split → filter → merge.

        Parameters
        
        points : noise-filtered CartesianPoints in angular order

        Returns
        
        list[Segment] — one per confirmed obstacle candidate.
        """
        if not points:
            return []

        # 1. jump-distance split
        raw_segments = self._split(points)

        # 2. quality filter
        filtered = [
            s for s in raw_segments
            if (self._cfg.min_points <= s.size <= self._cfg.max_points
                and s.arc_span_deg >= self._cfg.min_arc_deg)
        ]

        # 3. KD-tree merge
        merged = self._kdtree_merge(filtered)

        log.debug(
            "segmentation: %d raw → %d filtered → %d merged segments",
            len(raw_segments), len(filtered), len(merged),
        )
        return merged

    # Implementer: This implements jump-distance segmentation.

    def _split(self, points: list[CartesianPoint]) -> list[Segment]:
        """Walk angularly-ordered points, split on adaptive gap threshold."""
        segments = [Segment(points=[points[0]])]

        for i in range(1, len(points)):
            prev = points[i - 1]
            curr = points[i]

            gap = math.hypot(curr.x - prev.x, curr.y - prev.y)
            threshold = (
                self._cfg.jump_distance_m
                + curr.distance_m * self._cfg.adaptive_k
            )

            if gap > threshold:
                segments.append(Segment())
            segments[-1].points.append(curr)

        return segments
        
     # Merges nearby segments using their centroids
    def _kdtree_merge(self, segments: list[Segment]) -> list[Segment]:
        """
        Merge segment pairs whose centroids are within merge_distance_m.
        Uses cKDTree for O(n log n) lookup — handles 500+ segments cleanly.
        """
        if len(segments) <= 1:
            return segments

        # build centroid array
        centroids = np.array([s.centroid for s in segments], dtype=np.float64)
        tree      = cKDTree(centroids)

        # find all pairs within merge distance
        pairs = tree.query_pairs(r=self._cfg.merge_distance_m)

        # union-find to group connected segments
        parent = list(range(len(segments)))

        def find(i: int) -> int:
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i: int, j: int) -> None:
            parent[find(i)] = find(j)

        for i, j in pairs:
            union(i, j)

        # collect merged segments
        groups: dict[int, list[CartesianPoint]] = {}
        for idx, seg in enumerate(segments):
            root = find(idx)
            if root not in groups:
                groups[root] = []
            groups[root].extend(seg.points)

        return [Segment(points=pts) for pts in groups.values()] 