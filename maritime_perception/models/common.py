"""
models/common.py
Shared primitives used by every module in the system.
No logic. No imports from within this package.
This file is the foundation — everything else builds on it.

Timestamp convention

All timestamps use time.monotonic_ns() — a monotonically increasing
nanosecond counter that never jumps backwards and is unaffected by
system clock changes (NTP, daylight saving, etc).

This is critical for sensor fusion. When you fuse LiDAR at 10Hz,
IMU at 100Hz, and AIS at irregular intervals, every message must
carry its acquisition timestamp in the same time base so the fusion
layer can correctly order and interpolate readings.

Do NOT use time.time() for internal timestamps. Use now_ns() here.
time.time() is only for human-readable logging and output formatting.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Flag, auto


def now_ns() -> int:
    """
    Current monotonic time in nanoseconds.
    Use this for ALL internal timestamps.
    """
    return time.monotonic_ns()


def ns_to_ms(ns: int) -> float:
    """Convert nanoseconds to milliseconds."""
    return ns / 1_000_000.0


def elapsed_ms(start_ns: int) -> float:
    """Milliseconds elapsed since start_ns."""
    return ns_to_ms(now_ns() - start_ns)


# Sensor source bitmask

class SensorSource(Flag):
    """
    Bitmask identifying which sensors contributed to an observation or track.
    Combine with bitwise OR for multi-sensor tracks.

    Usage:
        src = SensorSource.LIDAR | SensorSource.AIS
        if SensorSource.LIDAR in src: ...
    """
    LIDAR = auto()
    AIS   = auto()
    FLS   = auto()    # Forward Looking Sonar
    MBES  = auto()    # Multibeam Echo Sounder
    RADAR = auto()    # future


# Header — carried by every message in the system

@dataclass(slots=True, frozen=True)
class Header:
    """
    Metadata attached to every message.
    Immutable once created — headers are never modified in transit.

    timestamp_ns : acquisition time (monotonic nanoseconds)
                   Set at the moment data is read from hardware.
                   Never updated downstream — preserves original timing.
    sensor_id    : unique identifier for the source sensor instance
                   e.g. "rplidar_s2_port0", "ais_vhf_ch1"
    frame_id     : coordinate frame of the data
                   "lidar"  — sensor body frame
                   "vessel" — vessel body frame (bow=+x, port=+y)
                   "ned"    — world NED frame (North=+x, East=+y, Down=+z)
    source       : SensorSource bitmask
    """
    timestamp_ns : int
    sensor_id    : str
    frame_id     : str
    source       : SensorSource