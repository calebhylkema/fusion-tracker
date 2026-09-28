"""Pi -> STM32 camera-detection message.

6-byte frame:  A5 5A | bearing_lo bearing_hi | flags | checksum
  - bearing : int16 little-endian, centi-degrees (deg * 100), + = right
  - flags   : bit0 = person present
  - checksum: (sum of the 3 payload bytes) & 0xFF
Matches the framed-parser style of the radar/IMU drivers on the STM32 side.
"""
import struct

HDR0, HDR1 = 0xA5, 0x5A


def pack_camera(bearing_deg, present):
    b = int(round(bearing_deg * 100.0))
    b = max(-32768, min(32767, b))
    payload = struct.pack("<hB", b, 1 if present else 0)   # int16 LE + uint8
    checksum = sum(payload) & 0xFF
    return bytes([HDR0, HDR1]) + payload + bytes([checksum])
