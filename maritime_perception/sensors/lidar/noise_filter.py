"""
sensors/lidar/noise_filter.py

2nd step of preprocessing — adaptive marine noise filter.

Algorithm: Adaptive Temporal Persistence Grid

The ocean surface produces hundreds of false LiDAR returns per scan:
wave crests, spray, sunlight reflections, multipath off the hull.
These are transient — they appear for 1–2 scans then vanish.
Real obstacles appear consistently scan after scan.

Core mechanism:

Divide the 2D space into a grid of cells. Each incoming point votes
for its cell. A cell is trusted only when its hit_count >= min_hits.
Cells that haven't been hit within TTL scans are evicted.

noise_filter.py maintains a persistence grid and keeps only points that repeatedly appear 
in the same spatial cells, with an automatically adjusted threshold based 
on the current clutter level.

Adaptive threshold:

The minimum hit count adapts to the current sea state automatically.
Each scan we compute clutter_density = fraction of new points that
fall into cells that were NOT active in the previous scan. High
clutter = rough sea → raise threshold. Low clutter = calm → lower it.

This self-tunes without operator intervention across sea states.

    min_hits = clip(
        base + floor(clutter_density × (max - base)),
        base, max
    )

Ego-motion note:

This filter is in sensor frame (vessel-relative). For a stationary
vessel this is fine. For a moving vessel, a stationary buoy drifts
across grid cells and its hit count resets. Ego-motion compensation
(preprocessing/ego_motion.py) must be applied before this filter
when the vessel is underway. 
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any

from .preprocessor import CartesianPoint

log = logging.getLogger(__name__)


# Noise filter Config (stores all parameters that control filter, loaded from YAML)


@dataclass(frozen=True)
class NoiseFilterConfig:
    cell_size_m      : float
    min_hits_base    : int
    min_hits_max     : int
    ttl_scans        : int
    grid_radius_m    : float
    clutter_adapt_rate: float

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> "NoiseFilterConfig":
        nf = cfg.get("noise_filter", {})
        return cls(
            cell_size_m       = float(nf.get("cell_size_m", 0.25)),
            min_hits_base     = int(nf.get("min_hits_base", 3)),
            min_hits_max      = int(nf.get("min_hits_max", 8)),
            ttl_scans         = int(nf.get("ttl_scans", 15)),
            grid_radius_m     = float(nf.get("grid_radius_m", 35.0)),
            clutter_adapt_rate= float(nf.get("clutter_adapt_rate", 0.1)),
        )



# Internal cell structure for the persistence grid. Each cell tracks its hit count 
# and the last scan it was hit in. 

@dataclass(slots=True)
class _Cell:
    hit_count : int = 0
    last_scan : int = 0


# Filter


class AdaptiveNoiseFilter:
    """
    Stateful adaptive marine noise filter (it remembers past scans to build hit counts).
    One instance per LiDAR sensor, lives for the session.
    Not thread-safe — must run on a single thread.
    """

    def __init__(self, config: NoiseFilterConfig) -> None:
        self._cfg          = config
        self._grid         : dict[tuple[int, int], _Cell] = {}
        self._current_hits : int   = config.min_hits_base
        self._scan_id      : int   = 0
        self._prev_active  : set[tuple[int, int]] = set()

    def update(
        self,
        points  : list[CartesianPoint],
        scan_id : int,
    ) -> list[CartesianPoint]:
        """
        Feed points into persistence grid.
        Returns only points whose cell has sufficient hit history.
        """
        self._scan_id = scan_id    #saves scan_id for cell eviction and clutter calculation

        # compute clutter density for adaptive threshold 
           #convert points to grid cells

        new_cells = {self._cell_key(p) for p in points if self._cell_key(p) is not None}
        if new_cells and self._prev_active:
            new_arrivals    = new_cells - self._prev_active    # compute clutter density
            clutter_density = len(new_arrivals) / max(len(new_cells), 1)
        else:
            clutter_density = 0.0

        #  adapt threshold (computing target threshold)
        target = self._cfg.min_hits_base + int(
            clutter_density * (self._cfg.min_hits_max - self._cfg.min_hits_base)
        )
        # exponential moving average on the threshold itself to avoid jitter
        rate = self._cfg.clutter_adapt_rate

        #Smooth threshold
        self._current_hits = int(
            self._current_hits * (1 - rate) + target * rate
        )

        #clamp threshold to valid range
        self._current_hits = max(
            self._cfg.min_hits_base,
            min(self._cfg.min_hits_max, self._current_hits),
        )

        # vote for cells 
        #For every point:Compute cell key, Create cell if it doesn’t exist, Increment hit count and Update last_scan.
        for pt in points:
            key = self._cell_key(pt)
            if key is None:
                continue
            cell = self._grid.get(key)
            if cell is None:
                cell = _Cell()
                self._grid[key] = cell
            cell.hit_count += 1
            cell.last_scan  = scan_id

        # evict stale cells that haven't been hit in TTL scans
        stale = [
            k for k, c in self._grid.items()
            if (scan_id - c.last_scan) > self._cfg.ttl_scans
        ]
        for k in stale:
            del self._grid[k]

        # filter: only keep points in cells with enough hits 
        surviving = [
            pt for pt in points
            if self._passes(pt)
        ]

        self._prev_active = new_cells  # save active cells for next clutter calculation

        log.debug(
            "noise_filter scan_id=%d: %d/%d survived, "
            "threshold=%d, clutter=%.2f, grid=%d cells",
            scan_id, len(surviving), len(points),
            self._current_hits, clutter_density, len(self._grid),
        )
        return surviving


    # reset filter state
    def reset(self) -> None:
        self._grid.clear()
        self._prev_active.clear()
        self._current_hits = self._cfg.min_hits_base
        self._scan_id      = 0
        log.info("AdaptiveNoiseFilter reset")

    @property
    def current_threshold(self) -> int:
        return self._current_hits



    def _cell_key(self, pt: CartesianPoint) -> tuple[int, int] | None:

        """Convert CartesianPoint to grid cell key (row, col).
            Returns None if out of bounds."""

        r = self._cfg.grid_radius_m
        if abs(pt.x) > r or abs(pt.y) > r:
            return None
        col = int(pt.x / self._cfg.cell_size_m)
        row = int(pt.y / self._cfg.cell_size_m)
        return (row, col)

    def _passes(self, pt: CartesianPoint) -> bool:

        """Check if point's cell has hit_count >= current threshold. 
            Returns False if cell is inactive or below threshold."""
        
        key = self._cell_key(pt)
        if key is None:
            return False
        cell = self._grid.get(key)
        return cell is not None and cell.hit_count >= self._current_hits