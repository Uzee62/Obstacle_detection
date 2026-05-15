"""
health/watchdog.py
==================
Sensor fault detection and DEGRADED mode signalling.

The watchdog polls sensor health every cycle and raises a system-level
DEGRADED flag when a critical sensor fails. The safety layer must
consume this flag and respond conservatively — typically by stopping
autonomous manoeuvres and alerting the operator.

Degraded conditions
-------------------
- LiDAR scan gap > max_scan_gap_s     (sensor offline or disconnected)
- LiDAR thread dead                   (unrecoverable driver failure)
- Any sensor error_count spike        (intermittent hardware fault)

The watchdog does NOT attempt recovery — that is the driver's job.
It observes and reports.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto

log = logging.getLogger(__name__)


class SystemHealth(Enum):
    NOMINAL  = auto()   # all sensors healthy
    DEGRADED = auto()   # one or more sensors unhealthy — reduce autonomy
    CRITICAL = auto()   # primary sensor offline — stop autonomous ops


@dataclass
class WatchdogStatus:
    system_health    : SystemHealth = SystemHealth.NOMINAL
    lidar_healthy    : bool         = True
    last_check_s     : float        = field(default_factory=time.monotonic)
    degraded_since_s : float | None = None
    fault_messages   : list[str]    = field(default_factory=list)


class SensorWatchdog:
    """
    Polls sensor health each cycle and maintains system health state.

    Usage
    -----
        watchdog = SensorWatchdog(max_scan_gap_s=1.0)
        # each cycle:
        status = watchdog.check(lidar_thread)
        if status.system_health != SystemHealth.NOMINAL:
            safety_layer.set_degraded()
    """

    def __init__(self, max_scan_gap_s: float = 1.0) -> None:
        self._max_gap    = max_scan_gap_s
        self._status     = WatchdogStatus()
        self._prev_errors: dict[str, int] = {}

    def check(self, lidar_thread) -> WatchdogStatus:
        """
        Check all sensor threads and update system health.
        Call once per fusion cycle.
        """
        now    = time.monotonic()
        faults = []

        # --- LiDAR health ---
        lidar_ok = lidar_thread.is_healthy()
        if not lidar_ok:
            gap_ms = lidar_thread.scan_gap_ms
            faults.append(
                f"LiDAR unhealthy — scan gap={gap_ms:.0f}ms "
                f"errors={lidar_thread.error_count}"
            )

        # --- error spike detection ---
        cur_err = lidar_thread.error_count
        prev_err = self._prev_errors.get("lidar", 0)
        if cur_err - prev_err > 3:
            faults.append(
                f"LiDAR error spike: +{cur_err - prev_err} errors this window"
            )
        self._prev_errors["lidar"] = cur_err

        # --- determine system health ---
        if not lidar_ok:
            health = SystemHealth.CRITICAL
        elif faults:
            health = SystemHealth.DEGRADED
        else:
            health = SystemHealth.NOMINAL

        # --- track degraded_since ---
        if health != SystemHealth.NOMINAL and self._status.degraded_since_s is None:
            self._status.degraded_since_s = now
            log.warning(
                "watchdog: system entering %s — %s",
                health.name, "; ".join(faults),
            )
        elif health == SystemHealth.NOMINAL and self._status.degraded_since_s is not None:
            duration = now - self._status.degraded_since_s
            log.info(
                "watchdog: system recovered to NOMINAL "
                "(was degraded for %.1fs)", duration,
            )
            self._status.degraded_since_s = None

        self._status.system_health  = health
        self._status.lidar_healthy  = lidar_ok
        self._status.last_check_s   = now
        self._status.fault_messages = faults

        if faults:
            for msg in faults:
                log.warning("watchdog: %s", msg)

        return self._status

    @property
    def status(self) -> WatchdogStatus:
        return self._status