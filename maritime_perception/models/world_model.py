"""
models/world_model.py
=====================
WorldObject  — a confirmed obstacle in the world model.
WorldModel   — the complete perception output, single source of truth.

The safety layer and navigation layer consume ONLY these types.
They never see raw scans, sensor observations, or internal tracks.

This separation is the architectural guarantee that perception
and decision-making remain independently upgradeable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .common import Header, SensorSource


@dataclass(slots=True)
class WorldObject:
    """
    A confirmed, fused obstacle ready for safety and navigation consumption.

    All positions are in the vessel frame (bow=+x, port=+y)
    unless frame_id in header indicates otherwise.

    position_std_m : 1-sigma position uncertainty from Kalman covariance.
                     The safety layer should add this to size_m when
                     computing safety margins.
    dynamic        : True if the obstacle is moving above noise floor.
                     Static obstacles still need avoidance but have
                     predictable future positions.
    coasting       : True if track is confirmed but currently unobserved.
                     Confidence will be declining. Treat with extra caution.
    sources        : which sensors have contributed to this track.
                     Used by safety layer to weight trust appropriately.
    """
    id             : int
    header         : Header

    position_x     : float
    position_y     : float
    velocity_x     : float
    velocity_y     : float
    heading_deg    : float
    size_m         : float
    confidence     : float
    position_std_m : float
    dynamic        : bool
    coasting       : bool
    sources        : SensorSource

    @property
    def range_m(self) -> float:
        return math.hypot(self.position_x, self.position_y)

    @property
    def bearing_deg(self) -> float:
        return math.degrees(math.atan2(self.position_y, self.position_x))

    @property
    def speed_ms(self) -> float:
        return math.hypot(self.velocity_x, self.velocity_y)

    @property
    def safety_radius_m(self) -> float:
        """
        Conservative safety margin = physical size + position uncertainty.
        Use this for collision avoidance, not raw size_m.
        """
        return self.size_m + self.position_std_m


@dataclass
class WorldModel:
    """
    Timestamped snapshot of all confirmed obstacles.

    Immutable once published. Safety and navigation read this,
    they never write back into it.

    scan_id   : monotonic scan counter for replay and correlation.
    latency_ms: pipeline processing time for this cycle (monitoring).
    """
    header     : Header
    objects    : list[WorldObject]
    scan_id    : int   = 0
    latency_ms : float = 0.0

    def __len__(self) -> int:
        return len(self.objects)

    def __iter__(self):
        return iter(self.objects)

    def closest(self) -> WorldObject | None:
        """Return the closest obstacle by range, or None if empty."""
        if not self.objects:
            return None
        return min(self.objects, key=lambda o: o.range_m)