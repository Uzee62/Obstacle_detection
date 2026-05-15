"""
models/pose.py

VesselPose — the vessel state estimate produced by RTK + IMU fusion.

Every sensor module that needs to compensate for vessel motion
reads this struct. Nobody reads raw IMU or RTK data directly.

Coordinate convention

Position : NED (North-East-Down), metres from arbitrary origin
Velocity : NED, metres per second
Heading  : degrees from North, clockwise (0=N, 90=E, 180=S, 270=W)
Roll     : degrees, positive = starboard down
Pitch    : degrees, positive = bow down
Yaw rate : degrees per second, positive = turning starboard
"""

from __future__ import annotations

from dataclasses import dataclass
from .common import Header


@dataclass(slots=True)
class VesselPose:
    """
    Full vessel state estimate at a given timestamp.
    Produced by pose/estimator.py from RTK + IMU fusion.
    """
    header        : Header

    # Position (NED, metres from session origin)
    north_m       : float = 0.0
    east_m        : float = 0.0
    down_m        : float = 0.0

    # Velocity (NED, m/s)
    vel_north_ms  : float = 0.0
    vel_east_ms   : float = 0.0
    vel_down_ms   : float = 0.0

    # Orientation
    heading_deg   : float = 0.0   # true heading, degrees from North
    roll_deg      : float = 0.0   # positive = starboard down
    pitch_deg     : float = 0.0   # positive = bow down
    yaw_rate_dps  : float = 0.0   # degrees per second, + = turning starboard

    # Fix quality
    rtk_fix_type  : int   = 0     # 0=none, 1=single, 2=float, 4=fixed
    hdop          : float = 99.0  # horizontal dilution of precision

    @property
    def speed_ms(self) -> float:
        """Scalar horizontal speed in m/s."""
        return (self.vel_north_ms**2 + self.vel_east_ms**2) ** 0.5

    @property
    def is_rtk_fixed(self) -> bool:
        return self.rtk_fix_type == 4

    @property
    def is_position_valid(self) -> bool:
        return self.rtk_fix_type > 0 and self.hdop < 5.0