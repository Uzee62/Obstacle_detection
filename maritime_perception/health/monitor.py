"""
health/monitor.py
=================
Collects per-cycle pipeline statistics for monitoring and logging.
Maintains rolling window metrics: scan rate, latency, track counts,
noise rejection rate.

This is observability — it tells you HOW the system is performing.
watchdog.py handles fault detection — it tells you IF something is wrong.
"""

from __future__ import annotations

import collections
import logging
import time
from dataclasses import dataclass, field

log = logging.getLogger(__name__)


@dataclass
class CycleStats:
    """Stats from one pipeline cycle."""
    timestamp_s      : float = 0.0
    scan_id          : int   = 0
    raw_points       : int   = 0
    after_preprocess : int   = 0
    after_noise      : int   = 0
    observations     : int   = 0
    confirmed_tracks : int   = 0
    world_objects    : int   = 0
    latency_ms       : float = 0.0
    noise_threshold  : int   = 0


class HealthMonitor:
    """
    Rolling-window health statistics.
    Call record() each cycle. Query properties for current metrics.
    """

    def __init__(self, window_scans: int = 100) -> None:
        self._window = window_scans
        self._history: collections.deque[CycleStats] = collections.deque(
            maxlen=window_scans
        )
        self._total_scans = 0
        self._start_time  = time.monotonic()

    def record(self, stats: CycleStats) -> None:
        self._history.append(stats)
        self._total_scans += 1

        # warn on high latency
        if stats.latency_ms > 80.0:
            log.warning(
                "health: high latency %.1fms on scan %d",
                stats.latency_ms, stats.scan_id,
            )

    # ------------------------------------------------------------------
    # Rolling metrics
    # ------------------------------------------------------------------

    @property
    def scan_rate_hz(self) -> float:
        """Estimated scan rate from recent history."""
        if len(self._history) < 2:
            return 0.0
        elapsed = (
            self._history[-1].timestamp_s
            - self._history[0].timestamp_s
        )
        if elapsed <= 0:
            return 0.0
        return (len(self._history) - 1) / elapsed

    @property
    def mean_latency_ms(self) -> float:
        if not self._history:
            return 0.0
        return sum(s.latency_ms for s in self._history) / len(self._history)

    @property
    def max_latency_ms(self) -> float:
        if not self._history:
            return 0.0
        return max(s.latency_ms for s in self._history)

    @property
    def mean_world_objects(self) -> float:
        if not self._history:
            return 0.0
        return sum(s.world_objects for s in self._history) / len(self._history)

    @property
    def mean_noise_rejection_pct(self) -> float:
        totals = [(s.after_preprocess, s.after_noise) for s in self._history
                  if s.after_preprocess > 0]
        if not totals:
            return 0.0
        pre   = sum(t[0] for t in totals)
        post  = sum(t[1] for t in totals)
        return 100.0 * (1.0 - post / pre) if pre > 0 else 0.0

    @property
    def uptime_s(self) -> float:
        return time.monotonic() - self._start_time

    @property
    def total_scans(self) -> int:
        return self._total_scans

    def log_summary(self) -> None:
        log.info(
            "health | uptime=%.0fs scans=%d rate=%.1fHz "
            "latency=%.1f/%.1fms objects=%.1f noise_rej=%.0f%%",
            self.uptime_s,
            self._total_scans,
            self.scan_rate_hz,
            self.mean_latency_ms,
            self.max_latency_ms,
            self.mean_world_objects,
            self.mean_noise_rejection_pct,
        )