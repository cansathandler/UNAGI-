#!/usr/bin/env python3
"""
Mock UNAGI head for testing the laptop side without hardware.

    python mock_head.py                 # listens on 127.0.0.1:4210
    python unagi_control.py --open-loop --head 127.0.0.1

Decodes laptop UDP packets exactly like head.ino, builds the RS-485 frame the
real head would put on the bus (printed in hex with --verbose), and replies with
fake telemetry so the laptop's logging path is exercised too.
"""
import argparse
import socket
import struct
import time

RX_HDR = struct.Struct("<2sBBHI")
TX = struct.Struct("<2sHIffffB")
CMD, STOP, ARM, DISARM, PING = 1, 2, 3, 4, 5


def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def bus_frame(typ: int, seq: int, duties) -> bytes:
    body = bytes([typ, seq & 0xFF, 2 * len(duties)]) + struct.pack(f"<{len(duties)}h", *duties)
    return b"\xA5\x5A" + body + struct.pack("<H", crc16(body))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--dead", type=int, nargs="*", default=[], help="motor numbers that draw no current")
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", args.port))
    armed, bus_seq, n_cmd, energy = False, 0, 0, 0.0
    duties = [0] * 16
    t0 = time.time()
    print(f"mock head on 127.0.0.1:{args.port}")
    while True:
        data, addr = s.recvfrom(256)
        if len(data) < RX_HDR.size or data[:2] != b"UN":
            continue
        _, typ, n, seq, t_ms = RX_HDR.unpack_from(data)
        if typ == CMD and armed and len(data) == RX_HDR.size + 2 * n:
            duties = [max(-1000, min(1000, d)) for d in struct.unpack_from(f"<{n}h", data, RX_HDR.size)]
            frame = bus_frame(1, bus_seq, duties)
            bus_seq += 1
            n_cmd += 1
            if args.verbose or n_cmd % 100 == 0:
                print(f"cmd {n_cmd:6d} seq={seq:5d} duty={duties}  bus={frame.hex(' ')}")
        elif typ == ARM:
            armed = True; print("ARM")
        elif typ == DISARM:
            armed = False; print("DISARM")
        elif typ == STOP:
            duties = [0] * 16
        if not armed:
            duties = [0] * 16
        # fake supply current: 0.15 A idle electronics + 1.2 A per full-duty coil (dead coils draw nothing)
        current = 0.15 + sum(abs(d) / 1000 * 1.2 for k, d in enumerate(duties) if (k + 1) not in args.dead)
        energy += 12.0 * current * 0.01
        flags = (1 if armed else 0) | (2 if typ == CMD and armed else 0)
        s.sendto(TX.pack(b"UT", seq, int((time.time() - t0) * 1000), 12.0, current, 12.0 * current,
                         energy, flags), addr)


if __name__ == "__main__":
    main()
