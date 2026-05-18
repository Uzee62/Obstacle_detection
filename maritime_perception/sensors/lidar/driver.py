"""sensors/lidar/driver.py

Slamtec RPLidar S-series hardware driver.

Owns the serial port lifecycle and walks the device through
    open → STOP/drain handshake → SCAN → one revolution → STOP.
Auto-reconnects read errors up to `max_reconnect_attempts`.

The on-wire protocol (descriptor parsing, command encoding,
scan-packet decoding) lives in `protocol.py`; this module is
intentionally I/O and state-machine only.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Final, Optional

import serial

from maritime_perception.models.common import Header, SensorSource, now_ns
from maritime_perception.sensors.lidar import protocol as proto

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Tuning constants
#
# These are wire-level device timings — they live with the driver, not in
# vessel YAML. If you tweak one, make sure the comment still explains why.
# ---------------------------------------------------------------------------

class _Timing:
    # Time the device gets to settle after we open the port, in case it
    # was just power-cycled and is still emitting its ASCII boot banner.
    SETTLE_AFTER_OPEN_S : Final[float] = 1.0

    # CMD_STOP can be ignored when it lands inside the lidar's RX state
    # for a scan packet. Send a few in a row to defeat that race.
    STOP_REPEAT         : Final[int]   = 3
    STOP_GAP_S          : Final[float] = 0.05
    STOP_DRAIN_S        : Final[float] = 0.02   # short post-STOP pause

    # After STOP, the device may keep emitting bytes already queued in
    # its TX FIFO. Drain until the wire stays silent for this long. The
    # quiet window must exceed the worst-case gap between scan-packet
    # bursts, otherwise we false-trigger and exit mid-stream.
    DRAIN_MAX_S         : Final[float] = 2.0
    DRAIN_QUIET_S       : Final[float] = 0.4

    # Cap on info/health payload reads. Some S2 firmwares advertise more
    # bytes in the descriptor than they actually send; take what arrives
    # rather than wait out the port-level timeout.
    PAYLOAD_READ_S      : Final[float] = 0.5

    # Poll cadence for non-blocking I/O loops.
    POLL_INTERVAL_S     : Final[float] = 0.005


# ---------------------------------------------------------------------------
# Public data model (preprocessor.py, pipeline.py, test_pipeline.py import
# these names directly — keep them stable)
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
    """Slamtec RPLidar (S1/S2) driver speaking the binary protocol
    directly over PySerial. No third-party RPLidar Python library
    required.

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
    ) -> None:
        self._port            = port
        self._baudrate        = baudrate
        self._timeout_s       = timeout_s
        self._reconnect_delay = reconnect_delay_s
        self._max_attempts    = max_reconnect_attempts
        self._serial          : Optional[serial.Serial] = None
        self._scan_id         = 0
        self._connected       = False
        log.info("RPLidarDriver initialised on %s at %d baud", port, baudrate)

    # ----- Public API -----------------------------------------------------

    def connect(self) -> None:
        """Open serial connection and verify sensor health."""
        self._connect_once()

    def disconnect(self) -> None:
        """Stop scanning and close serial port."""
        self._safe_disconnect()

    def is_connected(self) -> bool:
        return self._connected

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

    # ----- Connect / handshake --------------------------------------------

    def _connect_once(self) -> None:
        try:
            self._serial = self._open_port()
            self._quiesce_device()
            info   = self._exchange_info()
            health = self._exchange_health()
            self._connected = True
            self._log_connected(info, health)
            if health == proto.Health.ERROR:
                raise RuntimeError(
                    "RPLidar reports ERROR health status. "
                    "Power cycle the sensor and try again."
                )
        except serial.SerialException as exc:
            self._connected = False
            raise RuntimeError(f"Cannot open {self._port}: {exc}") from exc

    def _open_port(self) -> serial.Serial:
        s = serial.Serial(
            port      = self._port,
            baudrate  = self._baudrate,
            timeout   = self._timeout_s,
            bytesize  = serial.EIGHTBITS,
            parity    = serial.PARITY_NONE,
            stopbits  = serial.STOPBITS_ONE,
        )
        # Some FTDI/CP210x adapters tie DTR to the lidar's RESET line;
        # pyrplidar clears DTR for the same reason. Harmless if unwired.
        s.dtr = False
        s.rts = False
        time.sleep(_Timing.SETTLE_AFTER_OPEN_S)
        return s

    def _quiesce_device(self) -> None:
        """Stop any leftover scan stream and wait for the wire to go silent."""
        for _ in range(_Timing.STOP_REPEAT):
            self._write(proto.encode_command(proto.Command.STOP))
            time.sleep(_Timing.STOP_GAP_S)
        assert self._serial is not None
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        drained = self._drain_until_quiet(
            _Timing.DRAIN_MAX_S, _Timing.DRAIN_QUIET_S,
        )
        if drained:
            log.debug("Drained %d stale bytes before handshake", drained)

    def _exchange_info(self) -> Optional[proto.DeviceInfo]:
        self._write(proto.encode_command(proto.Command.GET_INFO))
        desc = proto.read_descriptor(self._read_byte)
        log.debug(
            "Info descriptor: form=%s len=%d type=0x%02X",
            desc.form, desc.data_len, desc.data_type,
        )
        raw = self._read_payload(desc.data_len, _Timing.PAYLOAD_READ_S)
        if len(raw) < desc.data_len:
            log.debug(
                "Info payload short: %d/%d bytes (raw=%s)",
                len(raw), desc.data_len, raw.hex(),
            )
        return proto.parse_info_payload(raw)

    def _exchange_health(self) -> proto.Health:
        self._write(proto.encode_command(proto.Command.GET_HEALTH))
        desc = proto.read_descriptor(self._read_byte)
        log.debug(
            "Health descriptor: form=%s len=%d type=0x%02X",
            desc.form, desc.data_len, desc.data_type,
        )
        raw = self._read_payload(desc.data_len, _Timing.PAYLOAD_READ_S)
        return proto.parse_health_payload(raw)

    def _log_connected(
        self,
        info  : Optional[proto.DeviceInfo],
        health: proto.Health,
    ) -> None:
        if info is None:
            log.info(
                "RPLidar connected — info unavailable, health=%s",
                health.name,
            )
        else:
            log.info(
                "RPLidar connected — model=%d firmware=%d.%d hardware=%d health=%s",
                info.model, info.fw_major, info.fw_minor, info.hardware,
                health.name,
            )

    def _safe_disconnect(self) -> None:
        try:
            if self._serial and self._serial.is_open:
                self._write(proto.encode_command(proto.Command.STOP))
                time.sleep(_Timing.STOP_DRAIN_S)
                self._serial.close()
        except Exception:
            pass
        finally:
            self._serial    = None
            self._connected = False

    # ----- Scan reading ---------------------------------------------------

    def _read_one_scan(self) -> LidarScan:
        if not self._connected or self._serial is None:
            raise RuntimeError("Not connected")

        self._write(proto.encode_command(proto.Command.SCAN))
        desc = proto.read_descriptor(self._read_byte)
        if desc.data_type != proto.DataType.SCAN:
            raise RuntimeError(
                f"Unexpected scan response type: 0x{desc.data_type:02X}"
            )

        points  : list[RawScanPoint] = []
        started : bool = False
        ts_ns   : int  = 0

        while True:
            raw = self._serial.read(proto.SCAN_PACKET_LEN)
            if len(raw) < proto.SCAN_PACKET_LEN:
                raise RuntimeError(
                    f"Short scan read: got {len(raw)} bytes, "
                    f"expected {proto.SCAN_PACKET_LEN}"
                )

            try:
                m = proto.parse_scan_packet(raw)
            except proto.ProtocolError:
                self._serial.reset_input_buffer()
                raise

            if m.new_revolution:
                if started and points:
                    self._write(proto.encode_command(proto.Command.STOP))
                    time.sleep(_Timing.STOP_DRAIN_S)
                    self._serial.reset_input_buffer()
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
                started = True
                ts_ns   = now_ns()
                points  = []

            if started and m.quality > 0 and m.distance_m > 0:
                points.append(RawScanPoint(
                    angle_deg  = m.angle_deg,
                    distance_m = m.distance_m,
                    quality    = m.quality,
                ))

    # ----- Low-level I/O helpers ------------------------------------------

    def _write(self, payload: bytes) -> None:
        assert self._serial is not None
        self._serial.write(payload)
        self._serial.flush()

    def _read_byte(self) -> Optional[int]:
        """Read one byte. Returns the byte value or None if no byte
        arrived within the port-level timeout. Used as the input source
        for `protocol.read_descriptor`."""
        assert self._serial is not None
        b = self._serial.read(1)
        return b[0] if b else None

    def _read_payload(self, expected: int, max_seconds: float) -> bytes:
        """Read up to `expected` bytes within `max_seconds` wall-clock.

        Polls `in_waiting` rather than blocking on `serial.read(N)`:
        `read(N)` would otherwise block the full port-level timeout
        (3 s) when fewer bytes arrive than requested, voiding the
        wall-clock budget.
        """
        assert self._serial is not None
        deadline = time.monotonic() + max_seconds
        buf      = bytearray()
        while len(buf) < expected and time.monotonic() < deadline:
            n = self._serial.in_waiting
            if n:
                buf.extend(self._serial.read(min(n, expected - len(buf))))
            else:
                time.sleep(_Timing.POLL_INTERVAL_S)
        return bytes(buf)

    def _drain_until_quiet(
        self,
        max_seconds  : float,
        quiet_window : float,
    ) -> int:
        """Drain incoming bytes until the line stays silent for
        `quiet_window` seconds (or `max_seconds` total). Returns
        bytes-drained — non-zero means the device was still streaming."""
        assert self._serial is not None
        start     = time.monotonic()
        deadline  = start + max_seconds
        last_byte = start
        drained   = 0
        while time.monotonic() < deadline:
            n = self._serial.in_waiting
            if n:
                self._serial.read(n)
                drained  += n
                last_byte = time.monotonic()
            elif time.monotonic() - last_byte >= quiet_window:
                return drained
            else:
                time.sleep(_Timing.POLL_INTERVAL_S)
        log.warning(
            "Lidar line never went silent for %.2fs (drained %d bytes in %.2fs)",
            quiet_window, drained, max_seconds,
        )
        return drained
