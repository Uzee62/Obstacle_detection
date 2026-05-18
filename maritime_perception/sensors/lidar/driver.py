# """
# sensors/lidar/driver.py

# RPLidar S2 hardware driver with auto-reconnect.

# Responsibilities

# - Open serial connection to RPLidar
# - Read one complete 360° scan
# - Timestamp at acquisition (monotonic_ns)
# - Convert raw driver units (mm) to metres
# - Auto-reconnect on disconnect or error
# - Expose health status

# This module does ZERO perception. It is hardware IO only.
# The output is a LidarScan — a timestamped list of (angle, distance) pairs.
# All interpretation happens downstream.

# RPLidar library notes
# Uses pyrplidar (not rplidar-roboticia) for S2 compatibility.
# rplidar-roboticia fails with "descriptor length mismatch" on S2 because it
# uses 115200 baud and doesn't handle S2's extended response descriptor format.
# pyrplidar uses 1000000 baud and handles the S2 protocol correctly.
# Each measurement exposes .quality (int), .angle (float deg), .distance (float mm).
# """

# from __future__ import annotations

# import logging
# import time
# from dataclasses import dataclass, field

# from maritime_perception.models.common import Header, SensorSource, now_ns

# log = logging.getLogger(__name__)


# # Internal scan model (driver-level only)


# # define RAW SCAN POINTS as a dataclass for clarity
# @dataclass(slots=True)
# class RawScanPoint:
#     angle_deg  : float
#     distance_m : float
#     quality    : int

# #define lidar SCAN POINTS
# # represents a full 360° scan as acquired from the driver, before preprocessing
# @dataclass
# class LidarScan:
#     """Raw scan as produced by the driver. Not yet preprocessed."""
#     header : Header
#     points : list[RawScanPoint]
#     scan_id: int = 0

#     def __len__(self) -> int:
#         return len(self.points)


# # RPLidar Driver
# # responsible for talking to the physical LiDAR.

# class RPLidarDriver:
#     """
#     Hardware driver for Slamtec RPLidar S2.

#     Usage
    
#         driver = RPLidarDriver(port="/dev/ttyUSB0")
#         driver.connect()
#         scan = driver.read_scan()
#         driver.disconnect()

#     Auto-reconnect:

#     read_scan() handles reconnection internally.
#     If the serial connection drops, it waits reconnect_delay_s and retries
#     up to max_reconnect_attempts times before raising RuntimeError.
#     """

#     SENSOR_ID = "rplidar_s2"
#     FRAME_ID  = "lidar"

#     def __init__(
#         self,
#         port                   : str   = "/dev/ttyUSB0",
#         timeout_s              : float = 5.0,
#         reconnect_delay_s      : float = 2.0,
#         max_reconnect_attempts : int   = 10,
#     ) -> None:
        
#         self._port            = port
#         self._timeout_s       = timeout_s

#         self._reconnect_delay = reconnect_delay_s
#         self._max_attempts    = max_reconnect_attempts

#         self._lidar           = None
#         self._scan_id         = 0
#         self._connected       = False

#         log.info("RPLidarDriver initialised on port %s", port)


#     # Connection management

#     def connect(self) -> None:
#         """Open connection to the RPLidar. Call before read_scan()."""
#         self._connect_once()

#     def disconnect(self) -> None:
#         """Gracefully stop and disconnect."""
#         self._safe_disconnect()

#     def is_connected(self) -> bool:
#         return self._connected

#     # Main read method
#     # handles auto-reconnect on error, returns one complete scan with timestamp.

#     def read_scan(self) -> LidarScan:
#         """
#         Read one complete 360° scan from the RPLidar.
#         Blocks until a scan is received.
#         Auto-reconnects on error.

#         Returns
#         LidarScan with timestamp set at moment of acquisition.

#         Raises
        
#         RuntimeError if unable to reconnect after max_reconnect_attempts.
#         """
#         attempts = 0

#         while attempts <= self._max_attempts:
#             try:
#                 return self._read_one_scan()
#             except Exception as exc:
#                 attempts += 1
#                 log.warning(
#                     "RPLidar read error (attempt %d/%d): %s",
#                     attempts, self._max_attempts, exc,
#                 )
#                 self._safe_disconnect()

#                 if attempts > self._max_attempts:
#                     raise RuntimeError(
#                         f"RPLidar failed after {self._max_attempts} "
#                         f"reconnect attempts on port {self._port}"
#                     ) from exc

#                 log.info(
#                     "Reconnecting in %.1fs ...", self._reconnect_delay
#                 )
#                 time.sleep(self._reconnect_delay)
#                 self._connect_once()

#         # unreachable but satisfies type checker
#         raise RuntimeError("RPLidar read_scan failed")

#     # Private
#     # Connect once without retry logic, raises on failure.

#     def _connect_once(self) -> None:
#         try:
#             from pyrplidar import PyRPlidar
#             self._lidar = PyRPlidar()
#             self._lidar.connect(
#                 port     = self._port,
#                 baudrate = 1000000,
#                 timeout  = int(self._timeout_s),
#             )
#             self._connected = True
#             info   = self._lidar.get_info()
#             health = self._lidar.get_health()
#             fw = info.get("firmware", ("?", "?"))
#             log.info(
#                 "RPLidar connected: model=%s firmware=%s.%s health=%s",
#                 info.get("model"),
#                 fw[0], fw[1],
#                 health[0],
#             )
#         except Exception as exc:
#             self._connected = False
#             raise RuntimeError(
#                 f"Failed to connect to RPLidar on {self._port}: {exc}"
#             ) from exc

#     def _read_one_scan(self) -> LidarScan:
#         """Read one scan from the live iterator."""
#         if not self._connected or self._lidar is None:
#             raise RuntimeError("RPLidar not connected")

#         scan_generator = self._lidar.start_scan()
#         for raw_scan in scan_generator():
#             ts_ns = now_ns()
#             self._scan_id += 1

#             points = [
#                 RawScanPoint(
#                     angle_deg  = float(m.angle),
#                     distance_m = float(m.distance) / 1000.0,  # mm to metre
#                     quality    = int(m.quality),
#                 )
#                 for m in raw_scan
#                 if m.quality > 0 and m.distance > 0
#             ]

#             return LidarScan(
#                 header=Header(
#                     timestamp_ns = ts_ns,
#                     sensor_id    = self.SENSOR_ID,
#                     frame_id     = self.FRAME_ID,
#                     source       = SensorSource.LIDAR,
#                 ),
#                 points  = points,
#                 scan_id = self._scan_id,
#             )

#         raise RuntimeError("start_scan ended without yielding a scan")

#     def _safe_disconnect(self) -> None:
#         """Disconnect without raising."""
#         try:
#             if self._lidar is not None:
#                 self._lidar.stop()
#                 self._lidar.disconnect()
#         except Exception:
#             pass
#         finally:
#             self._lidar     = None
#             self._connected = False


"""
sensors/lidar/driver.py
=======================
RPLidar S2 driver using direct serial communication.
Uses PySerial and the RPLidar binary protocol directly.
No third-party RPLidar Python libraries required.

Protocol reference: Slamtec RPLidar Communication Protocol v2.4
Baud rate: 1,000,000 for RPLidar S2
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass, field

import serial

from maritime_perception.models.common import Header, SensorSource, now_ns

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

SYNC_BYTE       = 0xA5
SYNC_BYTE2      = 0x5A

CMD_STOP        = 0x25
CMD_SCAN        = 0x20
CMD_GET_INFO    = 0x50
CMD_GET_HEALTH  = 0x52
CMD_RESET       = 0x40

# Response descriptor length
DESCRIPTOR_LEN  = 7

# Scan response: 5 bytes per measurement
SCAN_RESP_LEN   = 5

# Health status codes
HEALTH_GOOD     = 0
HEALTH_WARNING  = 1
HEALTH_ERROR    = 2


# ---------------------------------------------------------------------------
# Internal scan model 
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
    """
    RPLidar S2 driver using direct PySerial communication.
    Speaks the Slamtec binary protocol directly.
    No third-party RPLidar library required.
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
        self._serial          : serial.Serial | None = None
        self._scan_id         = 0
        self._connected       = False
        log.info("RPLidarDriver initialised on %s at %d baud", port, baudrate)

    # ------------------------------------------------------------------
    # Public API 
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open serial connection and verify sensor health."""
        self._connect_once()

    def disconnect(self) -> None:
        """Stop scanning and close serial port."""
        self._safe_disconnect()

    def is_connected(self) -> bool:
        return self._connected

    def read_scan(self) -> LidarScan:
        """
        Read one complete 360° scan.
        Blocks until a full scan is received.
        Auto-reconnects on error.
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
                        f"RPLidar failed after {self._max_attempts} attempts"
                    ) from exc
                log.info("Reconnecting in %.1fs ...", self._reconnect_delay)
                time.sleep(self._reconnect_delay)
                self._connect_once()

        raise RuntimeError("RPLidar read_scan failed")

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def _connect_once(self) -> None:
        """Open port, reset sensor, verify health."""
        try:
            self._serial = serial.Serial(
                port      = self._port,
                baudrate  = self._baudrate,
                timeout   = self._timeout_s,
                bytesize  = serial.EIGHTBITS,
                parity    = serial.PARITY_NONE,
                stopbits  = serial.STOPBITS_ONE,
            )

            # Silence any scan stream left running by a previous session,
            # then drain everything sitting in the OS / FTDI buffer.
            self._send_command(CMD_STOP)
            time.sleep(0.05)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

            # Reset cleanly. The S2 then emits an ASCII boot banner
            # ("RP LIDAR System...") that takes ~0.7–1.5 s — read past it
            # until the line is quiet, otherwise those bytes get mistaken
            # for the next response descriptor (0xA5 0x5A).
            self._send_command(CMD_RESET)
            self._drain_until_quiet(max_seconds=2.0, quiet_window=0.1)

            # verify it is alive
            info   = self._get_info()
            health = self._get_health()

            self._connected = True
            health_label = (
                ["Good", "Warning", "Error"][health]
                if isinstance(health, int) and 0 <= health <= 2
                else str(health)
            )
            log.info(
                "RPLidar connected — model=%s firmware=%s.%s health=%s",
                info.get("model", "?"),
                info.get("fw_major", "?"),
                info.get("fw_minor", "?"),
                health_label,
            )

            if health == HEALTH_ERROR:
                raise RuntimeError(
                    "RPLidar reports ERROR health status. "
                    "Power cycle the sensor and try again."
                )

        except serial.SerialException as exc:
            self._connected = False
            raise RuntimeError(
                f"Cannot open {self._port}: {exc}"
            ) from exc

    def _drain_until_quiet(
        self,
        max_seconds  : float = 2.0,
        quiet_window : float = 0.1,
    ) -> None:
        """Discard incoming bytes until the line stays silent for
        `quiet_window` seconds, or `max_seconds` total has elapsed."""
        deadline  = time.monotonic() + max_seconds
        last_byte = time.monotonic()
        while time.monotonic() < deadline:
            n = self._serial.in_waiting
            if n:
                self._serial.read(n)
                last_byte = time.monotonic()
            elif time.monotonic() - last_byte >= quiet_window:
                break
            else:
                time.sleep(0.01)
        self._serial.reset_input_buffer()

    def _safe_disconnect(self) -> None:
        """Stop motor and close port without raising."""
        try:
            if self._serial and self._serial.is_open:
                self._send_command(CMD_STOP)
                time.sleep(0.02)
                self._serial.close()
        except Exception:
            pass
        finally:
            self._serial    = None
            self._connected = False

    # ------------------------------------------------------------------
    # Scan reading
    # ------------------------------------------------------------------

    def _read_one_scan(self) -> LidarScan:
        """
        Start scan, collect one full 360° rotation, stop.

        The S2 sends scan packets continuously once CMD_SCAN is sent.
        Each packet is 5 bytes. We collect packets until we detect
        the start of a new rotation (start_bit flips to 1 on the
        first packet of each new revolution).
        """
        if not self._connected or not self._serial:
            raise RuntimeError("Not connected")

        # send scan command and read the response descriptor
        self._send_command(CMD_SCAN)
        descriptor = self._read_descriptor()

        if descriptor["type"] != 0x81:
            raise RuntimeError(
                f"Unexpected response type: 0x{descriptor['type']:02X}"
            )

        # collect one full rotation
        points    : list[RawScanPoint] = []
        started   : bool = False
        ts_ns     : int  = 0

        while True:
            raw = self._serial.read(SCAN_RESP_LEN)
            if len(raw) < SCAN_RESP_LEN:
                raise RuntimeError(
                    f"Short read: got {len(raw)} bytes, expected {SCAN_RESP_LEN}"
                )

            quality, angle_lo, angle_hi, dist_lo, dist_hi = raw

            # start_bit is bit 0 of the first byte
            # new_scan_bit is bit 1 of the first byte
            start_bit    = quality & 0x01
            new_scan_bit = (quality >> 1) & 0x01
            quality_val  = quality >> 2

            # check_bit must be 1 — sanity check
            check_bit = angle_lo & 0x01
            if check_bit != 1:
                # lost sync — flush and try again
                self._serial.reset_input_buffer()
                raise RuntimeError("Lost sync — check_bit not set")

            angle_q6 = ((angle_hi << 7) | (angle_lo >> 1))
            angle_deg = angle_q6 / 64.0

            dist_q2   = (dist_hi << 8) | dist_lo
            dist_m    = (dist_q2 / 4.0) / 1000.0   # mm → m

            if start_bit and new_scan_bit:
                # beginning of a new revolution
                if started and points:
                    # we have a complete scan — stop and return it
                    self._send_command(CMD_STOP)
                    time.sleep(0.02)
                    self._serial.reset_input_buffer()
                    self._scan_id += 1
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
                # start collecting
                started = True
                ts_ns   = now_ns()   # timestamp at start of this revolution
                points  = []

            if started and quality_val > 0 and dist_m > 0:
                points.append(RawScanPoint(
                    angle_deg  = angle_deg,
                    distance_m = dist_m,
                    quality    = quality_val,
                ))

    # ------------------------------------------------------------------
    # Low-level protocol helpers
    # ------------------------------------------------------------------

    def _send_command(self, cmd: int) -> None:
        """Send a no-payload command to the sensor."""
        self._serial.write(bytes([SYNC_BYTE, cmd]))
        self._serial.flush()

    def _read_descriptor(self) -> dict:
        """
        Read a 7-byte response descriptor.
        Format: 0xA5 0x5A | len(4 bytes) | send_mode | data_type
        """
        raw = self._serial.read(DESCRIPTOR_LEN)
        if len(raw) < DESCRIPTOR_LEN:
            raise RuntimeError(
                f"Descriptor read failed: got {len(raw)} bytes"
            )

        if raw[0] != SYNC_BYTE or raw[1] != SYNC_BYTE2:
            raise RuntimeError(
                f"Bad descriptor sync: 0x{raw[0]:02X} 0x{raw[1]:02X}"
            )

        data_len  = struct.unpack_from("<I", raw, 2)[0] & 0x3FFFFFFF
        send_mode = (struct.unpack_from("<I", raw, 2)[0] >> 30) & 0x03
        data_type = raw[6]

        return {
            "len"      : data_len,
            "send_mode": send_mode,
            "type"     : data_type,
        }

    def _get_info(self) -> dict:
        """Get device info — model, firmware version, serial number."""
        self._send_command(CMD_GET_INFO)
        descriptor = self._read_descriptor()
        raw = self._serial.read(descriptor["len"])

        if len(raw) < 20:
            return {}

        return {
            "model"    : raw[0],
            "fw_minor" : raw[1],
            "fw_major" : raw[2],
            "hardware" : raw[3],
            "serial"   : raw[4:].hex(),
        }

    def _get_health(self) -> int:
        """Get health status. Returns 0=Good, 1=Warning, 2=Error."""
        self._send_command(CMD_GET_HEALTH)
        descriptor = self._read_descriptor()
        raw = self._serial.read(descriptor["len"])

        if len(raw) < 3:
            return HEALTH_ERROR

        return raw[0]   # 0=Good, 1=Warning, 2=Error