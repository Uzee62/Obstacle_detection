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
    # was just power-cycled. These values come from `test_lidar_dump.py`
    # — the exact sequence we empirically observed the S2 responding to.
    SETTLE_AFTER_OPEN_S : Final[float] = 2.0

    # CMD_STOP can be ignored when it lands inside the lidar's RX state
    # for a scan packet. Send a couple in a row with a real drain window
    # between them, rather than a fast STOP burst.
    STOP_REPEAT         : Final[int]   = 2
    STOP_DRAIN_S        : Final[float] = 0.3    # gap between STOPs / post-flush

    # Cap on info/health payload reads. Some S2 firmwares advertise more
    # bytes in the descriptor than they actually send; take what arrives
    # rather than wait out the port-level timeout.
    PAYLOAD_READ_S      : Final[float] = 1.0

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

    def _log_connected(
        self,
        info  : Optional[proto.DeviceInfo],
        health: Optional[proto.Health],
    ) -> None:
        health_str = health.name if health is not None else "unknown"
        if info is None:
            log.info(
                "RPLidar connected — info unavailable, health=%s",
                health_str,
            )
        else:
            log.info(
                "RPLidar connected — model=%d firmware=%d.%d hardware=%d health=%s",
                info.model, info.fw_major, info.fw_minor, info.hardware,
                health_str,
            )

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
        """Bring the device to a known-quiescent state.

        Sequence matches `test_lidar_dump.py` (the probe that confirmed
        the device responds to GET_INFO): drain any pre-existing stream,
        then STOP / drain N times, then flush, then one more drain.
        Each drain is fixed-duration rather than quiet-window-based,
        because we don't actually need a quiet-line guarantee — we just
        need to consume anything sitting in the buffer.
        """
        assert self._serial is not None

        pre = self._drain_for(_Timing.STOP_DRAIN_S)
        for i in range(_Timing.STOP_REPEAT):
            self._write(proto.encode_command(proto.Command.STOP))
            extra = self._drain_for(_Timing.STOP_DRAIN_S)
            if extra:
                log.debug("Drained %d bytes after STOP #%d", extra, i + 1)

        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        post = self._drain_for(_Timing.STOP_DRAIN_S)

        if pre + post:
            log.debug(
                "Drained %d stale bytes before handshake (pre=%d post=%d)",
                pre + post, pre, post,
            )

    def _drain_for(self, seconds: float) -> int:
        """Read and discard incoming bytes for `seconds` wall-clock,
        regardless of whether the wire is quiet. Returns bytes drained."""
        assert self._serial is not None
        end     = time.monotonic() + seconds
        drained = 0
        while time.monotonic() < end:
            n = self._serial.in_waiting
            if n:
                self._serial.read(n)
                drained += n
            else:
                time.sleep(_Timing.POLL_INTERVAL_S)
        return drained

    # Sanity bound on advertised payload size. The largest legitimate
    # response (GET_INFO) is 20 bytes; anything larger means we read
    # garbage as a descriptor and should bail instead of asking the
    # payload reader to wait for hundreds of MB.
    _MAX_REASONABLE_PAYLOAD = 256

    def _exchange_info(self) -> Optional[proto.DeviceInfo]:
        self._write(proto.encode_command(proto.Command.GET_INFO))
        desc = proto.read_descriptor(self._read_byte)
        log.debug(
            "Info descriptor: form=%s len=%d type=0x%02X",
            desc.form, desc.data_len, desc.data_type,
        )
        if (desc.data_type != proto.DataType.INFO
                or desc.data_len > self._MAX_REASONABLE_PAYLOAD):
            log.warning(
                "GET_INFO returned bogus descriptor (form=%s len=%d "
                "type=0x%02X); proceeding without device info",
                desc.form, desc.data_len, desc.data_type,
            )
            return None
        raw = self._read_payload(desc.data_len, _Timing.PAYLOAD_READ_S)
        if len(raw) < desc.data_len:
            log.debug(
                "Info payload short: %d/%d bytes (raw=%s)",
                len(raw), desc.data_len, raw.hex(),
            )
        return proto.parse_info_payload(raw)

    def _exchange_health(self) -> Optional[proto.Health]:
        """Returns None if the device fails to respond meaningfully —
        some S2 firmwares ignore GET_HEALTH while still being usable
        for scans. Real ERROR status is still returned as Health.ERROR."""
        self._write(proto.encode_command(proto.Command.GET_HEALTH))
        try:
            desc = proto.read_descriptor(self._read_byte)
        except proto.ProtocolError as exc:
            log.warning("Health descriptor read failed: %s", exc)
            return None
        log.debug(
            "Health descriptor: form=%s len=%d type=0x%02X",
            desc.form, desc.data_len, desc.data_type,
        )
        if (desc.data_type != proto.DataType.HEALTH
                or desc.data_len > self._MAX_REASONABLE_PAYLOAD):
            log.warning(
                "GET_HEALTH returned bogus descriptor (form=%s len=%d "
                "type=0x%02X); treating health as unknown",
                desc.form, desc.data_len, desc.data_type,
            )
            return None
        raw = self._read_payload(desc.data_len, _Timing.PAYLOAD_READ_S)
        return proto.parse_health_payload(raw)

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

