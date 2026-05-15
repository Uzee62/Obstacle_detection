"""
sensors/lidar/driver.py

RPLidar S2 hardware driver with auto-reconnect.

Responsibilities

- Open serial connection to RPLidar
- Read one complete 360° scan
- Timestamp at acquisition (monotonic_ns)
- Convert raw driver units (mm) to metres
- Auto-reconnect on disconnect or error
- Expose health status

This module does ZERO perception. It is hardware IO only.
The output is a LidarScan — a timestamped list of (angle, distance) pairs.
All interpretation happens downstream.

RPLidar library notes
The rplidar-roboticia library returns scans as a list of tuples:
    (quality, angle_deg, distance_mm)
quality=0 means the reading is invalid — we drop those here.
distance_mm=0 means no return — we drop those here.
The library blocks until a full scan is received.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

from maritime_perception.models.common import Header, SensorSource, now_ns

log = logging.getLogger(__name__)


# Internal scan model (driver-level only)


# define RAW SCAN POINTS as a dataclass for clarity
@dataclass(slots=True)
class RawScanPoint:
    angle_deg  : float
    distance_m : float
    quality    : int

#define lidar SCAN POINTS
# represents a full 360° scan as acquired from the driver, before preprocessing
@dataclass
class LidarScan:
    """Raw scan as produced by the driver. Not yet preprocessed."""
    header : Header
    points : list[RawScanPoint]
    scan_id: int = 0

    def __len__(self) -> int:
        return len(self.points)


# RPLidar Driver
# responsible for talking to the physical LiDAR.

class RPLidarDriver:
    """
    Hardware driver for Slamtec RPLidar S2.

    Usage
    
        driver = RPLidarDriver(port="/dev/ttyUSB0")
        driver.connect()
        scan = driver.read_scan()
        driver.disconnect()

    Auto-reconnect:

    read_scan() handles reconnection internally.
    If the serial connection drops, it waits reconnect_delay_s and retries
    up to max_reconnect_attempts times before raising RuntimeError.
    """

    SENSOR_ID = "rplidar_s2"
    FRAME_ID  = "lidar"

    def __init__(
        self,
        port                   : str   = "/dev/ttyUSB0",
        timeout_s              : float = 5.0,
        reconnect_delay_s      : float = 2.0,
        max_reconnect_attempts : int   = 10,
    ) -> None:
        
        self._port            = port
        self._timeout_s       = timeout_s

        self._reconnect_delay = reconnect_delay_s
        self._max_attempts    = max_reconnect_attempts

        self._lidar           = None
        self._scan_id         = 0
        self._connected       = False

        log.info("RPLidarDriver initialised on port %s", port)


    # Connection management

    def connect(self) -> None:
        """Open connection to the RPLidar. Call before read_scan()."""
        self._connect_once()

    def disconnect(self) -> None:
        """Gracefully stop and disconnect."""
        self._safe_disconnect()

    def is_connected(self) -> bool:
        return self._connected

    # Main read method
    # handles auto-reconnect on error, returns one complete scan with timestamp.

    def read_scan(self) -> LidarScan:
        """
        Read one complete 360° scan from the RPLidar.
        Blocks until a scan is received.
        Auto-reconnects on error.

        Returns
        LidarScan with timestamp set at moment of acquisition.

        Raises
        
        RuntimeError if unable to reconnect after max_reconnect_attempts.
        """
        attempts = 0

        while attempts <= self._max_attempts:
            try:
                return self._read_one_scan()
            except Exception as exc:
                attempts += 1
                log.warning(
                    "RPLidar read error (attempt %d/%d): %s",
                    attempts, self._max_attempts, exc,
                )
                self._safe_disconnect()

                if attempts > self._max_attempts:
                    raise RuntimeError(
                        f"RPLidar failed after {self._max_attempts} "
                        f"reconnect attempts on port {self._port}"
                    ) from exc

                log.info(
                    "Reconnecting in %.1fs ...", self._reconnect_delay
                )
                time.sleep(self._reconnect_delay)
                self._connect_once()

        # unreachable but satisfies type checker
        raise RuntimeError("RPLidar read_scan failed")

    # Private
    # Connect once without retry logic, raises on failure.

    def _connect_once(self) -> None:
        try:
            from rplidar import RPLidar
            self._lidar     = RPLidar(
                self._port,
                timeout=self._timeout_s,
            )
            self._connected = True
            info = self._lidar.get_info()
            health = self._lidar.get_health()
            log.info(
                "RPLidar connected: model=%s firmware=%s.%s health=%s",
                info.get("model"),
                info.get("firmware", ("?", "?"))[0],
                info.get("firmware", ("?", "?"))[1],
                health[0],
            )
        except Exception as exc:
            self._connected = False
            raise RuntimeError(
                f"Failed to connect to RPLidar on {self._port}: {exc}"
            ) from exc

    def _read_one_scan(self) -> LidarScan:
        """Read one scan from the live iterator."""
        if not self._connected or self._lidar is None:
            raise RuntimeError("RPLidar not connected")

        for raw_scan in self._lidar.iter_scans(max_buf_meas=5000):
            # timestamp at the moment the scan buffer is handed to us
            ts_ns = now_ns()
            self._scan_id += 1

            points = [
                RawScanPoint(
                    angle_deg  = float(angle),
                    distance_m = float(distance) / 1000.0,  # mm to metre
                    quality    = int(quality),
                )
                for quality, angle, distance in raw_scan
                if quality > 0 and distance > 0   # drop invalid readings
            ]

            return LidarScan(
                header=Header(
                    timestamp_ns = ts_ns,
                    sensor_id    = self.SENSOR_ID,
                    frame_id     = self.FRAME_ID,
                    source       = SensorSource.LIDAR,
                ),
                points  = points,
                scan_id = self._scan_id,
            )

        raise RuntimeError("iter_scans ended without yielding a scan")

    def _safe_disconnect(self) -> None:
        """Disconnect without raising."""
        try:
            if self._lidar is not None:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
        except Exception:
            pass
        finally:
            self._lidar     = None
            self._connected = False