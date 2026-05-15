"""
models/track.py

ObjectTrack — a persistent tracked obstacle across multiple scans.
TrackState  — lifecycle state machine enum.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np

from .common import Header, SensorSource


class TrackState(Enum):
    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"
    COASTING  = "coasting"
    DEAD      = "dead"


@dataclass
class ObjectTrack:
    """
    A persistent tracked obstacle.
    state_vec  : Kalman state [x, y, vx, vy]
    covariance : Kalman covariance 4x4
    kalman     : KalmanFilter instance — attached by FusionTracker on spawn
    score      : running quality [0,1], decays on miss, grows on hit
    """
    # required
    track_id       : int
    state          : TrackState
    header         : Header
    state_vec      : np.ndarray        # shape (4,)
    covariance     : np.ndarray        # shape (4,4)

    # optional
    kalman         : Any          = field(default=None, repr=False)
    size_m         : float        = 0.5
    confidence     : float        = 0.0
    score          : float        = 0.3
    hit_count      : int          = 0
    miss_count     : int          = 0
    age_scans      : int          = 0
    sources        : SensorSource = SensorSource.LIDAR
    last_update_ns : int          = 0

    @property
    def position_x(self) -> float:
        return float(self.state_vec[0])

    @property
    def position_y(self) -> float:
        return float(self.state_vec[1])

    @property
    def velocity_x(self) -> float:
        return float(self.state_vec[2])

    @property
    def velocity_y(self) -> float:
        return float(self.state_vec[3])

    @property
    def speed_ms(self) -> float:
        return math.hypot(self.velocity_x, self.velocity_y)

    @property
    def heading_deg(self) -> float:
        if self.speed_ms < 0.1:
            return 0.0
        return math.degrees(math.atan2(self.velocity_y, self.velocity_x))

    @property
    def range_m(self) -> float:
        return math.hypot(self.position_x, self.position_y)

    @property
    def bearing_deg(self) -> float:
        return math.degrees(math.atan2(self.position_y, self.position_x))

    @property
    def is_dynamic(self) -> bool:
        return self.speed_ms > 0.15

    @property
    def position_std_m(self) -> float:
        return float(math.sqrt(
            (self.covariance[0, 0] + self.covariance[1, 1]) / 2.0
        ))