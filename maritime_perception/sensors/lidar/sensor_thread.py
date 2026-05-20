"""
sensors/lidar/sensor_thread.py

Thread wrapper for the LiDAR perception pipeline.
This file wraps the entire LiDAR pipeline in a dedicated background thread 
so the rest of your system can fetch the latest detections instantly.

Implements AbstractSensorPipeline so the fusion engine
treats this identically to any other sensor.

Threading model

One background thread runs continuously:
  while running:
      scan = driver.read_scan()       ← blocks until scan arrives
      obs  = pipeline.process(scan)   ← runs full perception pipeline
      buffer.put(obs)                 ← drops result in queue

Main loop calls latest() — instant, never blocks.
  obs = lidar_thread.latest()
  if obs is not None:
      fusion_engine.feed(obs)

Buffer policy: maxsize=1

Only the most recent result is kept. If the main loop is slightly
slow and two scans are processed before it reads, the older one
is discarded. In real-time maritime perception you always want the
freshest picture of the world. A stale scan is worse than no scan.

Health monitoring

is_healthy() returns False if:
  - Thread has died unexpectedly
  - No scan received within max_scan_gap_s
  - Driver has exceeded max reconnect attempts

Auto-reconnect is handled inside driver.read_scan().
The thread itself never dies on a transient driver error.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Any

from maritime_perception.models.common import now_ns, ns_to_ms
from maritime_perception.models.observation import DetectionObservation
from maritime_perception.sensors.base import AbstractSensorPipeline

from .driver import RPLidarDriver
from .pipeline import LidarPerceptionPipeline

log = logging.getLogger(__name__)


class LidarSensorThread(AbstractSensorPipeline):
    """
    Wraps LidarPerceptionPipeline in a background thread.
    Implements AbstractSensorPipeline for the fusion engine.
    """

    def __init__(
        self,
        driver          : RPLidarDriver,
        pipeline        : LidarPerceptionPipeline,
        max_scan_gap_s  : float = 1.0,
        startup_grace_s : float = 20.0,
    ) -> None:
        self._driver         = driver
        self._pipeline       = pipeline
        self._max_gap_s      = max_scan_gap_s
        self._startup_grace_s = startup_grace_s

        # single-slot buffer — keeps only the latest result
        self._buffer         : queue.Queue[list[DetectionObservation]] = queue.Queue(maxsize=1)

        self._thread         = threading.Thread(
            target = self._run,
            name   = "lidar-sensor-thread",
            daemon = True,
        )
        self._running        = False
        self._started_at_ns  = 0
        self._last_scan_ns   = 0
        self._error_count    = 0
        self._scan_count     = 0

    
    # AbstractSensorPipeline interface

    def start(self) -> None:
        """Connect driver and start background thread."""
        log.info("LidarSensorThread: connecting driver ...")
        self._driver.connect()
        self._running       = True
        self._started_at_ns = now_ns()
        self._thread.start()
        log.info("LidarSensorThread: started")

    def stop(self) -> None:
        """Gracefully stop the thread and disconnect the driver."""
        self._running = False
        self._driver.disconnect()
        self._thread.join(timeout=3.0)
        if self._thread.is_alive():
            log.warning("LidarSensorThread: thread did not stop cleanly")
        log.info(
            "LidarSensorThread: stopped. scans=%d errors=%d",
            self._scan_count, self._error_count,
        )

    def latest(self) -> list[DetectionObservation]:
        """
        Return the most recent DetectionObservations.
        Never blocks. Returns empty list if no data yet.
        """
        try:
            return self._buffer.get_nowait()
        except queue.Empty:
            return []

    def is_healthy(self) -> bool:
        """
        True if thread is alive and scans are arriving on schedule.
        During the startup grace window (before the first scan arrives)
        the thread is considered healthy — the publisher needs several
        seconds to warm up the motor and may retry startScan.
        """
        if not self._thread.is_alive():
            return False
        if self._last_scan_ns == 0:
            # No scan yet — give the publisher time to warm up.
            grace_ms = self._startup_grace_s * 1000.0
            since_start_ms = ns_to_ms(now_ns() - self._started_at_ns)
            return since_start_ms < grace_ms
        gap_s = ns_to_ms(now_ns() - self._last_scan_ns) / 1000.0
        return gap_s < self._max_gap_s

    # Convenience accessors

    @property
    def scan_count(self) -> int:
        return self._scan_count

    @property
    def error_count(self) -> int:
        return self._error_count

    @property
    def scan_gap_ms(self) -> float:
        """Milliseconds since last scan received (or since start if none yet)."""
        if self._last_scan_ns == 0:
            if self._started_at_ns == 0:
                return 0.0
            return ns_to_ms(now_ns() - self._started_at_ns)
        return ns_to_ms(now_ns() - self._last_scan_ns)

    # Background thread

    def _run(self) -> None:
        """
        Background thread body.
        Reads scans, processes them, drops latest into buffer.
        Handles all errors internally — never propagates exceptions.
        """
        log.info("LidarSensorThread: background thread running")

        while self._running:
            try:
                scan = self._driver.read_scan()
                self._last_scan_ns = now_ns()
                self._scan_count  += 1

                observations = self._pipeline.process(scan)
                self._drain_and_put(observations)

            except RuntimeError as exc:
                # driver gave up reconnecting — this is fatal
                self._error_count += 1
                log.error(
                    "LidarSensorThread: fatal driver error: %s. "
                    "Stopping thread.", exc
                )
                self._running = False
                break

            except Exception as exc:
                # unexpected error — log and continue
                self._error_count += 1
                log.warning(
                    "LidarSensorThread: unexpected error (count=%d): %s",
                    self._error_count, exc,
                )
                time.sleep(0.1)   # brief pause before retry

        log.info("LidarSensorThread: background thread exiting")

    def _drain_and_put(
        self,
        observations: list[DetectionObservation],
    ) -> None:
        """Replace buffer content with latest result."""
        try:
            self._buffer.get_nowait()   # discard old result
        except queue.Empty:
            pass
        self._buffer.put_nowait(observations)