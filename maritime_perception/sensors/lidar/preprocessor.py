"""

First stage of LiDAR perception pipeline.

This file takes the raw output from driver.py and converts it into 
clean Cartesian points that are ready for clustering and tracking.

Steps (in order)

1. Validity gate    — reject zero, NaN, inf distances explicitly
2. Range gate       — reject outside [min_range_m, max_range_m]
3. Intensity gate   — reject low-quality returns (optional)
4. FOV mask         — blank self-hull / superstructure sectors
5. Mounting offset  — apply angular offset from vessel_profile.yaml
6. Polar → cartesian— convert to (x, y) in vessel frame

Output

list of CartesianPoint — validated, converted points ready for noise filter.

Design

Pure functions only. Stateless. No side effects.
Each step is a separate private function for testability.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any

from .driver import LidarScan, RawScanPoint

log = logging.getLogger(__name__)


# Output type


@dataclass(slots=True)
class CartesianPoint:
    """A validated, converted LiDAR point in vessel frame."""
    angle_deg  : float   # corrected angle after mounting offset
    distance_m : float   # original range
    x          : float   # vessel frame: bow=+x
    y          : float   # vessel frame: port=+y
    quality    : int     # original return quality


# Preprocessor Config (Stores all configuration parameters, loaded from YAML.)


@dataclass(frozen=True)
class PreprocessorConfig:
    range_min_m      : float
    range_max_m      : float
    min_intensity    : int
    angle_offset_deg : float
    fov_mask         : list[tuple[float, float]]   # list of (start, end) degree pairs

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> "PreprocessorConfig":
        pp      = cfg.get("preprocessing", {})
        mount   = cfg.get("lidar_mounting", {})
        mask_raw= cfg.get("fov_mask", [])

        return cls(
            range_min_m      = float(pp.get("range_min_m", 0.3)),
            range_max_m      = float(pp.get("range_max_m", 30.0)),
            min_intensity    = int(pp.get("min_intensity", 0)),
            angle_offset_deg = float(mount.get("angle_offset_deg", 0.0)),
            fov_mask         = [
                (float(sector[0]), float(sector[1]))
                for sector in mask_raw
            ],
        )


# Preprocessor


class LidarPreprocessor:
    """
    Stateless preprocessor. Converts a LidarScan to a list of CartesianPoints.
    """

    def __init__(self, config: PreprocessorConfig) -> None:
        self._cfg = config

    def process(self, scan: LidarScan) -> list[CartesianPoint]:
        """
        Apply full preprocessing pipeline to one scan.
        Returns validated, converted points in vessel frame.
        """
        if not scan.points:
            log.warning("preprocessor: empty scan scan_id=%d", scan.scan_id)
            return []

        #Initialise counters for diagnostics
        #These count how many points are rejected at each stage

        t_valid = t_range = t_intensity = t_mask = 0 

        # Output list of CartesianPoints
        out: list[CartesianPoint] = []

        for pt in scan.points:
            # 1. validity gate
            if not self._is_valid(pt):
                t_valid += 1
                continue

            # 2. range gate
            if not (self._cfg.range_min_m <= pt.distance_m <= self._cfg.range_max_m):
                t_range += 1
                continue

            # 3. intensity gate
            if pt.quality < self._cfg.min_intensity:
                t_intensity += 1
                continue

            # 4. mounting offset
            angle = _normalise(pt.angle_deg + self._cfg.angle_offset_deg)

            # 5. FOV mask
            if self._cfg.fov_mask and _in_mask(angle, self._cfg.fov_mask):
                t_mask += 1
                continue

            # 6. polar → cartesian
            rad = math.radians(angle)
            x   = pt.distance_m * math.cos(rad)
            y   = pt.distance_m * math.sin(rad)

            out.append(CartesianPoint(
                angle_deg  = angle,
                distance_m = pt.distance_m,
                x          = x,
                y          = y,
                quality    = pt.quality,
            ))

        log.debug(
            "preprocessor scan_id=%d: %d/%d kept "
            "(invalid=%d range=%d intensity=%d mask=%d)",
            scan.scan_id, len(out), len(scan.points),
            t_valid, t_range, t_intensity, t_mask,
        )
        return out

    #Checks if a point contains a finite positive distance

    @staticmethod
    def _is_valid(pt: RawScanPoint) -> bool:
        """Explicit validity check — never rely on comparison with NaN."""
        if not math.isfinite(pt.distance_m):
            return False
        if pt.distance_m <= 0.0:
            return False
        return True


# Helpers


def _normalise(deg: float) -> float:
    """Wrap angle to [0, 360)."""
    return deg % 360.0


def _in_mask(angle: float, mask: list[tuple[float, float]]) -> bool:
    """
    Return True if angle falls inside any masked sector.
    Handles sectors that wrap around 0° (e.g. 350°–10°).
    """
    for start, end in mask:
        if start <= end:
            if start <= angle <= end:
                return True
        else:   # wraps around 0°
            if angle >= start or angle <= end:
                return True
    return False