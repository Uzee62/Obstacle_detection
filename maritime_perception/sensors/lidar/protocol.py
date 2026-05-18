"""sensors/lidar/protocol.py

Wire-format protocol for Slamtec RPLidar S-series sensors.

Pure parsing / encoding only — no I/O, no logging, no driver state.
Designed so the driver's transport can be swapped (mock, file replay,
unit tests) without touching anything in this module.

Why this module exists separately:
    The older `rplidar-roboticia` library fails with "descriptor length
    mismatch" on the S2 because it only knows the legacy 7-byte
    descriptor form (0xA5 0x5A | size30 + mode | type) and refuses the
    compact 4-byte form (0xA5 | size_lo | size_hi | type) that the S2
    firmware actually emits for GET_INFO and GET_HEALTH responses. We
    accept both forms by sniffing the byte after 0xA5.

    The S2 also runs at 1 Mbaud rather than the A-series 115200.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional


# Wire-format constants from Slamtec Communication Protocol v2.4.
SYNC1 = 0xA5
SYNC2 = 0x5A                # second byte of legacy descriptor

# Per-measurement byte count in the standard (non-Express) scan response.
SCAN_PACKET_LEN = 5


class Command(IntEnum):
    """Single-byte opcodes for no-payload commands."""
    STOP       = 0x25
    SCAN       = 0x20
    GET_INFO   = 0x50
    GET_HEALTH = 0x52
    RESET      = 0x40


class DataType(IntEnum):
    """`data_type` values seen in response descriptors."""
    INFO   = 0x04
    HEALTH = 0x06
    SCAN   = 0x81


class Health(IntEnum):
    GOOD    = 0
    WARNING = 1
    ERROR   = 2


class ProtocolError(RuntimeError):
    """Raised on any wire-format inconsistency."""


@dataclass(frozen=True, slots=True)
class Descriptor:
    form      : str     # "legacy" (7 bytes) or "s2-compact" (4 bytes)
    data_len  : int
    send_mode : int
    data_type : int


@dataclass(frozen=True, slots=True)
class DeviceInfo:
    model    : int
    fw_minor : int
    fw_major : int
    hardware : int
    serial   : str


@dataclass(frozen=True, slots=True)
class ScanMeasurement:
    angle_deg      : float
    distance_m     : float
    quality        : int
    new_revolution : bool    # True on the first packet of a new 360° sweep


# Function the parser uses to pull one byte from the transport. Returns
# the byte value (0..255), or None if the read timed out.
ByteReader = Callable[[], Optional[int]]


def encode_command(cmd: Command) -> bytes:
    """Encode a no-payload command as wire bytes."""
    return bytes([SYNC1, int(cmd)])


def read_descriptor(read_byte: ByteReader) -> Descriptor:
    """Read a response descriptor from a byte-at-a-time source.

    Two on-the-wire forms are supported, dispatched on the first byte:

      1. Legacy 7-byte (first byte == 0xA5):
           0xA5 0x5A | size30 + mode2 | type
         Documented Slamtec protocol; used by A1/A2/S1.

      2. Naked 5-byte (first byte != 0xA5):
           size30 + mode2 | type
         What the S2 firmware actually emits for GET_INFO / GET_HEALTH
         responses — there is NO sync prefix. This is the format that
         makes rplidar-roboticia fail with "descriptor length mismatch":
         it expects 0xA5 as byte 0 and bails when it sees the size
         field's low byte (e.g. 0x14 = 20 for info) instead.

    The reader does not scan past the first byte. If the line had stray
    pre-response bytes, the caller's drain should have handled them
    before this is called; the descriptor's `data_type` field is the
    caller's sanity check against parsing garbage.
    """
    b0 = read_byte()
    if b0 is None:
        raise ProtocolError("Descriptor timeout — no bytes received")

    if b0 == SYNC1:
        b1 = read_byte()
        if b1 is None:
            raise ProtocolError("Descriptor truncated after 0xA5")
        if b1 == SYNC2:
            rest = _read_n(read_byte, 5)
            if rest is None:
                raise ProtocolError("Legacy descriptor truncated")
            packed = struct.unpack("<I", bytes(rest[:4]))[0]
            return Descriptor(
                form      = "legacy",
                data_len  = packed & 0x3FFFFFFF,
                send_mode = (packed >> 30) & 0x03,
                data_type = rest[4],
            )
        # 0xA5 was not followed by 0x5A. Treat the 0xA5 as a stray
        # pre-sync byte (common when residual bytes from a previous
        # operation sit ahead of the real response) and continue as
        # if b1 were the first byte of a naked descriptor.
        b0 = b1

    # Naked 5-byte form: b0 is the low byte of a 4-byte little-endian
    # (size30 + mode2) field, followed by a 1-byte data_type.
    rest = _read_n(read_byte, 4)
    if rest is None:
        raise ProtocolError("Naked descriptor truncated")
    packed = struct.unpack("<I", bytes([b0, rest[0], rest[1], rest[2]]))[0]
    return Descriptor(
        form      = "naked",
        data_len  = packed & 0x3FFFFFFF,
        send_mode = (packed >> 30) & 0x03,
        data_type = rest[3],
    )


def parse_info_payload(raw: bytes) -> Optional[DeviceInfo]:
    """Parse a GET_INFO payload. Returns None if too short to be useful.

    Some S2 firmware revisions advertise more bytes in the descriptor
    than they actually send; we accept any length ≥ 4 (model + fw_minor +
    fw_major + hardware) and treat the rest as serial-number bytes.
    """
    if len(raw) < 4:
        return None
    return DeviceInfo(
        model    = raw[0],
        fw_minor = raw[1],
        fw_major = raw[2],
        hardware = raw[3],
        serial   = raw[4:].hex(),
    )


def parse_health_payload(raw: bytes) -> Optional[Health]:
    """Parse a GET_HEALTH payload.

    Returns None if `raw` is empty — i.e. the device didn't actually
    respond. The caller should treat that as "health unknown", not
    "health ERROR", since some S2 firmwares fail to answer the health
    query at all even when the device itself is otherwise usable.

    Returns Health.ERROR if the status byte is out of range (0..2).
    """
    if not raw:
        return None
    if raw[0] > int(Health.ERROR):
        return Health.ERROR
    return Health(raw[0])


def parse_scan_packet(raw: bytes) -> ScanMeasurement:
    """Decode one 5-byte scan packet.

    Raises ProtocolError if the packet fails its sync invariant
    (the `check_bit` in byte 1 must be 1). The caller should drain
    and resync the stream.
    """
    if len(raw) != SCAN_PACKET_LEN:
        raise ProtocolError(f"Scan packet wrong length: {len(raw)}")

    quality, angle_lo, angle_hi, dist_lo, dist_hi = raw

    if angle_lo & 0x01 != 1:
        raise ProtocolError("Lost scan sync — check_bit not set")

    start_bit    = bool(quality & 0x01)
    new_scan_bit = bool((quality >> 1) & 0x01)
    quality_val  = quality >> 2

    angle_q6  = (angle_hi << 7) | (angle_lo >> 1)
    angle_deg = angle_q6 / 64.0
    dist_q2   = (dist_hi << 8) | dist_lo
    dist_m    = (dist_q2 / 4.0) / 1000.0

    return ScanMeasurement(
        angle_deg      = angle_deg,
        distance_m     = dist_m,
        quality        = quality_val,
        new_revolution = start_bit and new_scan_bit,
    )


def _read_n(read_byte: ByteReader, n: int) -> Optional[list[int]]:
    out: list[int] = []
    for _ in range(n):
        b = read_byte()
        if b is None:
            return None
        out.append(b)
    return out


def _hex(bs: list[int]) -> list[str]:
    return [f"0x{b:02X}" for b in bs]
