"""sensors/lidar/driver.py

RPLidar driver — thin subprocess client over the Slamtec C++ SDK.

The on-wire protocol (serial port, descriptors, scan packets, reconnect)
is owned by `tools/lidar_publisher`, a small C++ binary linked against
the official Slamtec SDK. This Python class spawns it and reads scans
from its stdout in a simple line-based format:

    SCAN <monotonic_ns>
    <angle_deg> <distance_m> <quality>
    ...
    END

The subprocess boundary buys us:
  - Crash isolation — a segfault in the SDK doesn't take down perception.
  - Trivial debugging — the publisher binary runs standalone.
  - No FFI / ctypes / shared-memory complexity in Python.

The public surface (`RPLidarDriver`, `LidarScan`, `RawScanPoint`,
`SENSOR_ID`, `FRAME_ID`) is identical to the previous direct-serial
implementation, so the rest of the pipeline (sensor_thread, pipeline,
preprocessor, main) needs no edits.
"""

from __future__ import annotations

import logging
import os
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Optional

from maritime_perception.models.common import Header, SensorSource

log = logging.getLogger(__name__)


# Default location of the publisher binary, relative to the repo root.
# Override via the `publisher_path=` constructor argument or the
# `RPLIDAR_PUBLISHER` env var if your build lives somewhere else.
_REPO_ROOT      = Path(__file__).resolve().parents[3]
_DEFAULT_PUB    = _REPO_ROOT / "tools" / "lidar_publisher" / "lidar_publisher"


# ---------------------------------------------------------------------------
# Public data model — unchanged from the previous driver, downstream
# modules (preprocessor.py, pipeline.py, test_pipeline.py) import these
# names directly so they must stay stable.
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class RawScanPoint:
    angle_deg  : float
    distance_m : float
    quality    : int


@dataclass
class LidarScan:
    """Raw scan as produced by the driver. Not yet preprocessed."""
    header  : Header
    points  : list[RawScanPoint]
    scan_id : int = 0

    def __len__(self) -> int:
        return len(self.points)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

class RPLidarDriver:
    """Spawn the SDK-backed publisher and read scans from its stdout.

    Usage:
        driver = RPLidarDriver(port="/dev/ttyUSB0")
        driver.connect()
        scan = driver.read_scan()       # blocks for one revolution
        driver.disconnect()

    read_scan() auto-reconnects on transient errors up to
    `max_reconnect_attempts` before giving up with RuntimeError.
    """

    SENSOR_ID = "rplidar_s2"
    FRAME_ID  = "lidar"

    def __init__(
        self,
        port                   : str   = "/dev/ttyUSB0",
        baudrate               : int   = 1_000_000,
        timeout_s              : float = 3.0,
        reconnect_delay_s      : float = 2.0,
        max_reconnect_attempts : int   = 10,
        publisher_path         : Optional[str] = None,
    ) -> None:
        self._port            = port
        self._baudrate        = baudrate
        self._timeout_s       = timeout_s
        self._reconnect_delay = reconnect_delay_s
        self._max_attempts    = max_reconnect_attempts

        env_path = os.environ.get("RPLIDAR_PUBLISHER")
        chosen   = publisher_path or env_path or str(_DEFAULT_PUB)
        self._publisher_path  = Path(chosen)

        self._proc      : Optional[subprocess.Popen] = None
        self._stderr_thr: Optional[threading.Thread] = None
        self._scan_id   = 0
        self._connected = False
        log.info(
            "RPLidarDriver initialised — port=%s baud=%d publisher=%s",
            port, baudrate, self._publisher_path,
        )

    # ----- Public API -----------------------------------------------------

    def connect(self) -> None:
        """Spawn the publisher subprocess and verify it started."""
        self._connect_once()

    def disconnect(self) -> None:
        """Signal the publisher to exit and reap the subprocess."""
        self._safe_disconnect()

    def is_connected(self) -> bool:
        return (
            self._connected
            and self._proc is not None
            and self._proc.poll() is None
        )

    def read_scan(self) -> LidarScan:
        """Read one complete 360° scan. Blocks until a full revolution
        arrives. Auto-reconnects on transient error."""
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
                        f"RPLidar failed after {self._max_attempts} attempts"
                    ) from exc
                log.info("Reconnecting in %.1fs ...", self._reconnect_delay)
                time.sleep(self._reconnect_delay)
                self._connect_once()
        raise RuntimeError("RPLidar read_scan failed")

    # ----- Subprocess lifecycle -------------------------------------------

    def _connect_once(self) -> None:
        if not self._publisher_path.exists():
            raise RuntimeError(
                f"Publisher binary not found at {self._publisher_path}. "
                f"Build it first:  make -C tools/lidar_publisher  "
                f"(requires the Slamtec SDK built at ~/rplidar_sdk)."
            )
        if not os.access(self._publisher_path, os.X_OK):
            raise RuntimeError(
                f"Publisher binary at {self._publisher_path} is not executable."
            )

        log.info(
            "Spawning publisher: %s %s %d",
            self._publisher_path, self._port, self._baudrate,
        )
        self._proc = subprocess.Popen(
            [str(self._publisher_path), self._port, str(self._baudrate)],
            stdin    = subprocess.PIPE,
            stdout   = subprocess.PIPE,
            stderr   = subprocess.PIPE,
            bufsize  = 1,            # line-buffered text stream
            text     = True,
            close_fds= True,
        )

        # Forward publisher's stderr into the Python logger asynchronously
        # so we see device-info / health / scan-start messages.
        self._stderr_thr = threading.Thread(
            target = self._forward_stderr,
            args   = (self._proc.stderr,),
            daemon = True,
            name   = "lidar-pub-stderr",
        )
        self._stderr_thr.start()

        self._connected = True

    def _forward_stderr(self, stream: IO[str]) -> None:
        try:
            for line in stream:
                line = line.rstrip()
                if line:
                    log.info("[publisher] %s", line)
        except Exception:
            pass

    def _safe_disconnect(self) -> None:
        proc = self._proc
        self._connected = False
        self._proc      = None
        if proc is None:
            return
        try:
            # Closing stdin asks the publisher to exit cleanly.
            if proc.stdin is not None:
                try:
                    proc.stdin.close()
                except Exception:
                    pass
            if proc.poll() is None:
                try:
                    proc.wait(timeout=self._timeout_s)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=1.0)
        except Exception as exc:
            log.warning("Disconnect cleanup error: %s", exc)

    # ----- Scan parsing ---------------------------------------------------

    def _read_one_scan(self) -> LidarScan:
        if (not self._connected
                or self._proc is None
                or self._proc.stdout is None):
            raise RuntimeError("Not connected")
        if self._proc.poll() is not None:
            raise RuntimeError(
                f"Publisher exited (code={self._proc.returncode})"
            )

        header = self._proc.stdout.readline()
        if not header:
            raise RuntimeError("Publisher stdout closed")
        header = header.strip()
        if not header.startswith("SCAN "):
            raise RuntimeError(f"Expected SCAN header, got: {header!r}")
        try:
            ts_ns = int(header.split()[1])
        except (IndexError, ValueError) as exc:
            raise RuntimeError(f"Malformed SCAN header: {header!r}") from exc

        points: list[RawScanPoint] = []
        while True:
            line = self._proc.stdout.readline()
            if not line:
                raise RuntimeError("Publisher stdout closed mid-scan")
            line = line.rstrip()
            if line == "END":
                break
            try:
                angle_s, dist_s, qual_s = line.split()
                points.append(RawScanPoint(
                    angle_deg  = float(angle_s),
                    distance_m = float(dist_s),
                    quality    = int(qual_s),
                ))
            except ValueError as exc:
                raise RuntimeError(
                    f"Malformed scan point: {line!r}"
                ) from exc

        self._scan_id += 1
        return LidarScan(
            header  = Header(
                timestamp_ns = ts_ns,
                sensor_id    = self.SENSOR_ID,
                frame_id     = self.FRAME_ID,
                source       = SensorSource.LIDAR,
            ),
            points  = points,
            scan_id = self._scan_id,
        )
