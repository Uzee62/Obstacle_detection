"""
models/observation.py

DetectionObservation — the universal sensor interface.

This is the single most important type in the system.
Every sensor module — LiDAR, AIS, FLS, MBES — produces this type
and nothing else. The fusion layer consumes this type and nothing else.

This strict contract is what makes the architecture sensor-agnostic.
Adding a new sensor = writing a new module that produces this type.
The fusion layer never needs to change.

Fields
------
All positional fields are in the coordinate frame specified by header.frame_id.
When produced by a sensor pipeline, frame_id is typically "vessel".

position_x, position_y
    Obstacle centroid in metres.
    Vessel frame: bow=+x, port=+y, origin=vessel centre of rotation.

size_m
    Bounding circle radius in metres.
    Represents the spatial uncertainty + physical size of the obstacle.
    Feeds directly into collision avoidance safety radius calculations.

range_m
    Scalar distance from vessel origin to obstacle centroid.
    Derived from position but pre-computed for convenience.

bearing_deg
    Bearing from vessel bow to obstacle centroid.
    0° = dead ahead, +90° = port beam, -90° = starboard beam.
    Convention: same as maritime bearing relative to heading.

point_count
    Number of raw sensor returns that contributed to this observation.
    Quality proxy — more points = more reliable geometry.
    For AIS this is always 1 (one transmitted message).

confidence
    Normalised quality score in [0.0, 1.0].
    Computed per sensor using sensor-appropriate model.
    LiDAR: f(point_count, range, persistence).
    AIS: f(message_age, vessel_class, signal_quality).
    Do NOT compare confidence across sensor types without normalisation.
"""

from __future__ import annotations

from dataclasses import dataclass
from .common import Header


@dataclass(slots=True)
class DetectionObservation:
    """Universal output of any sensor detection module."""

    header      : Header
    position_x  : float    # metres, vessel frame
    position_y  : float    # metres, vessel frame
    size_m      : float    # bounding circle radius, metres
    range_m     : float    # distance from vessel origin, metres
    bearing_deg : float    # relative bearing, degrees (-180 to +180)
    point_count : int      # raw returns contributing to this observation
    confidence  : float    # quality score [0.0, 1.0]

    def __post_init__(self) -> None:
        self.confidence = float(max(0.0, min(1.0, self.confidence)))
        if self.size_m < 0:
            raise ValueError(f"size_m must be >= 0, got {self.size_m}")
        if self.range_m < 0:
            raise ValueError(f"range_m must be >= 0, got {self.range_m}")