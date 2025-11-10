#!/usr/bin/env python3
"""Minimal host-side smoke tester for the rrclite protocol."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import List, Tuple


try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - optional helper
    serial = None


START0 = 0xAA
START1 = 0x55


def crc8(data: bytes) -> int:
    table = [
        0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
        157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
        35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
        190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
        70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
        219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
        101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
        248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
        140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
        17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
        175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
        50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
        202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
        87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
        233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
        116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
    ]
    value = 0
    for byte in data:
        value = table[value ^ byte]
    return value


def build_frame(function: int, payload: bytes) -> bytes:
    length = len(payload)
    header = bytes([START0, START1, function & 0xFF, length & 0xFF])
    checksum = crc8(header[2:] + payload)
    return header + payload + bytes([checksum])


def read_frame(port: "serial.Serial", timeout: float = 0.5) -> Tuple[int, bytes]:
    deadline = time.monotonic() + timeout
    state = 0
    buf = bytearray()
    while True:
        if time.monotonic() > deadline:
            raise TimeoutError("Timed out waiting for frame")
        data = port.read(1)
        if not data:
            continue
        byte = data[0]
        if state == 0:
            if byte == START0:
                state = 1
        elif state == 1:
            if byte == START1:
                state = 2
            else:
                state = 0
        elif state == 2:
            func = byte
            state = 3
        elif state == 3:
            length = byte
            payload = port.read(length + 1)
            if len(payload) != length + 1:
                raise TimeoutError("Short frame")
            body = bytes([func, length]) + payload[:-1]
            if crc8(body) != payload[-1]:
                raise ValueError("CRC mismatch")
            return func, payload[:-1]


def transact(port: "serial.Serial", func: int, sub: int, payload: bytes = b"") -> bytes:
    frame = build_frame(func, bytes([sub]) + payload)
    port.write(frame)
    while True:
        rx_func, data = read_frame(port)
        if rx_func == func:
            return data


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="rrclite protocol smoke tester")
    parser.add_argument("--port", required=True, help="Serial port to use")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Baud rate")
    parser.add_argument("--timeout", type=float, default=0.5, help="I/O timeout")
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    if serial is None:
        print("pyserial not installed; install it to use this tool", file=sys.stderr)
        return 2

    args = parse_args(argv)

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as port:
        version = transact(port, 0x00, 0xF0)
        caps = transact(port, 0x00, 0xF1)
        selftest = transact(port, 0x00, 0xF5)
        stats = transact(port, 0x00, 0xF3)
        health = transact(port, 0x00, 0xF4)

    v_major, v_minor, v_patch = struct.unpack_from("<BBH", version)
    caps_major = caps[0]
    caps_minor = caps[1]
    caps_flags = struct.unpack_from("<I", caps, 4)[0]
    max_baud, max_imu_hz, max_enc_hz = struct.unpack_from("<IHH", caps, 8)
    self_bits, = struct.unpack_from("<H", selftest)

    print(f"Version: {v_major}.{v_minor}.{v_patch}")
    print(f"Caps proto: {caps_major}.{caps_minor} flags=0x{caps_flags:08X} max_baud={max_baud} imu_hz={max_imu_hz} enc_hz={max_enc_hz}")
    print(f"Self-test bits: 0x{self_bits:04X}")
    print(f"Stats payload ({len(stats)} bytes): {stats.hex()}")
    print(f"Health payload ({len(health)} bytes): {health.hex()}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main(sys.argv[1:]))
