"""Raw LiDAR probe — bypasses the driver and dumps exactly what the
device emits for each command. Use this to distinguish a driver-side
parsing problem from a device-side silence.

Run with the LiDAR freshly power-cycled (unplug USB + motor power for
30 s, plug back in, wait ~5 s for boot to settle, THEN run this)."""

import time
import serial

PORT = "/dev/ttyUSB0"
BAUD = 1_000_000

SYNC1, STOP, GET_INFO, GET_HEALTH = 0xA5, 0x25, 0x50, 0x52


def hexdump(buf: bytes) -> str:
    return " ".join(f"{b:02X}" for b in buf) if buf else "(empty)"


def drain_for(s: serial.Serial, seconds: float, label: str) -> bytes:
    end = time.monotonic() + seconds
    buf = bytearray()
    while time.monotonic() < end:
        n = s.in_waiting
        if n:
            buf.extend(s.read(n))
        else:
            time.sleep(0.005)
    print(f"[{label}] {len(buf)} bytes: {hexdump(bytes(buf))}")
    return bytes(buf)


def send(s: serial.Serial, opcode: int, name: str) -> None:
    print(f"--- sending {name} (0xA5 0x{opcode:02X})")
    s.write(bytes([SYNC1, opcode]))
    s.flush()


def main() -> None:
    print(f"Opening {PORT} at {BAUD}")
    s = serial.Serial(
        PORT, BAUD, timeout=1.0,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    s.dtr = False
    s.rts = False

    print("Sleeping 2.0 s for any boot banner to complete...")
    time.sleep(2.0)
    drain_for(s, 0.5, "post-settle")

    send(s, STOP, "STOP")
    drain_for(s, 0.3, "after STOP #1")
    send(s, STOP, "STOP")
    drain_for(s, 0.3, "after STOP #2")

    s.reset_input_buffer()
    print("(buffer flushed)")
    drain_for(s, 0.3, "post-flush idle")

    send(s, GET_INFO, "GET_INFO")
    drain_for(s, 1.0, "after GET_INFO")

    send(s, GET_HEALTH, "GET_HEALTH")
    drain_for(s, 1.0, "after GET_HEALTH")

    s.write(bytes([SYNC1, STOP]))
    s.close()
    print("done.")


if __name__ == "__main__":
    main()
