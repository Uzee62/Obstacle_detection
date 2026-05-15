"""
sensors/lidar/extractor.py

Stage 4 — convert confirmed segments into DetectionObservations.

extractor.py converts each segmented LiDAR cluster into a standardized 
DetectionObservation by estimating its centroid, size, range, bearing, 
and confidence for use by the multi-object tracker.

For each segment:
  centroid    = mean of all (x, y) points
  radius      = max distance from centroid (bounding circle)
  range       = distance from vessel origin to centroid
  bearing     = atan2(cy, cx)
  confidence  = f(point_count, range_penalty)

Confidence model

Two components multiplied together:

1. Point count contribution (saturates at confidence_pts_ref points):
       pt_conf = min(1.0, point_count / pts_ref)

2. Range penalty (further = less reliable centroid accuracy):
       range_pen = 1.0 - penalty_factor × (range / max_range)
       clamped to [0.3, 1.0]

Final: confidence = pt_conf × range_pen

This is intentionally simple and explainable. When something goes
wrong at sea you need to be able to reason about why a track has
a given confidence. Opaque ML models fail this requirement.

L-shape ready
   
The extractor is structured so that an L-shape fitting step can be
inserted after the bounding circle computation without restructuring.
L-shape fitting is Phase 2 for close-range vessel detection.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any

from maritime_perception.models.common import Header, SensorSource
from maritime_perception.models.observation import DetectionObservation
from .segmentation import Segment

log = logging.getLogger(__name__)


# Extractor Config (stores all parameters that control extraction, loaded from YAML)

@dataclass(frozen=True)
class ExtractorConfig:
    max_obstacle_radius_m      : float
    confidence_pts_ref         : int
    confidence_range_penalty   : float
    range_max_m                : float   # matches preprocessing range_max_m

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> "ExtractorConfig":
        ex = cfg.get("extraction", {})
        pp = cfg.get("preprocessing", {})
        return cls(
            max_obstacle_radius_m    = float(ex.get("max_obstacle_radius_m", 15.0)),
            confidence_pts_ref       = int(ex.get("confidence_pts_ref", 30)),
            confidence_range_penalty = float(ex.get("confidence_range_penalty", 0.5)),
            range_max_m              = float(pp.get("range_max_m", 30.0)),
        )


# Obstacle Extractor


class ObstacleExtractor:
    """
    Converts a list of Segments into DetectionObservations.
    Stateless — safe to call from any thread.
    """

    def __init__(self, config: ExtractorConfig) -> None:
        self._cfg = config

    def extract(
        self,
        segments  : list[Segment],
        header    : Header,
    ) -> list[DetectionObservation]:
        """
        Convert each valid segment to a DetectionObservation.
        Discards segments whose bounding circle exceeds max_obstacle_radius_m
        (walls, quay faces, extended structures).

        Parameters
        
        segments : confirmed segments from JumpDistanceSegmenter
        header   : Header from the originating LidarScan (preserves timestamp)

        Returns
        
        list[DetectionObservation] — one per valid obstacle cluster.
        """
        observations = []

        for seg in segments:
            obs = self._extract_one(seg, header)
            if obs is None:
                continue
            observations.append(obs)

        log.debug(
            "extractor: %d/%d segments → observations",
            len(observations), len(segments),
        )
        return observations

    

    def _extract_one(
        self,
        seg    : Segment,
        header : Header,
    ) -> DetectionObservation | None:
        """Extract one observation from one segment. Returns None if invalid."""
        pts = seg.points
        n   = len(pts)

        # centroid
        cx = sum(p.x for p in pts) / n
        cy = sum(p.y for p in pts) / n

        # bounding circle radius
        radius = max(
            math.hypot(p.x - cx, p.y - cy)
            for p in pts
        )
        radius = max(radius, 0.05)   # minimum 5cm

        # discard oversized (walls, structures)
        if radius > self._cfg.max_obstacle_radius_m:
            log.debug(
                "extractor: discarding oversized cluster "
                "radius=%.1fm at (%.1f, %.1f)", radius, cx, cy
            )
            return None

        range_m     = math.hypot(cx, cy)
        bearing_deg = math.degrees(math.atan2(cy, cx))

        # confidence
        pt_conf   = min(1.0, n / self._cfg.confidence_pts_ref)
        range_pen = max(
            1.0 - self._cfg.confidence_range_penalty,
            1.0 - self._cfg.confidence_range_penalty
                  * (range_m / self._cfg.range_max_m)
        )
        confidence = pt_conf * range_pen

        return DetectionObservation(
            header      = header,
            position_x  = cx,
            position_y  = cy,
            size_m      = radius,
            range_m     = range_m,
            bearing_deg = bearing_deg,
            point_count = n,
            confidence  = confidence,
        )