#!/usr/bin/env python3
"""Passive RS485 Modbus RTU sniffer with Vision BMS / COM-Box decode.

Captures the battery RS485 bus (read-only tap, 9600 8N1 by default),
verifies CRC16 on every frame, decodes requests and responses of both
masters: the Deye inverter (FC03 0x00 qty=28 + 0x0050 qty=2) and the
COM-Box (FC03 0x00 qty=0x27). Output format matches
deye/rs485_sniff_inverter_battery_2026-10-09.log.

Usage:
  python3 tools/rs485_sniff.py --list
  python3 tools/rs485_sniff.py --port /dev/tty.usbserial-0001
  python3 tools/rs485_sniff.py --port /dev/tty.usbserial-0001 --baud 9600
  python3 tools/rs485_sniff.py --port /dev/tty.usbserial-0001 --duration 600
  python3 tools/rs485_sniff.py --selftest

The log file is written only on a clean exit (Ctrl-C, SIGTERM, or
--duration elapsed); background jobs should use --duration or `kill -INT`.
"""

import argparse
import signal
import sys
import time
from datetime import datetime

SILENCE_S = 0.008  # frame gap at 9600 baud (byte = 1.04 ms)


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def candidates(buf: bytes) -> list:
    """Possible frame lengths for the buffer head (request + response)."""
    if len(buf) < 2:
        return []
    fc = buf[1]
    if fc & 0x80:
        return [5]
    if fc in (0x01, 0x02, 0x03, 0x04):
        out = [8]  # request
        if len(buf) >= 3:
            out.append(5 + buf[2])  # response: bytecount
        return out
    if fc in (0x05, 0x06):
        return [8]
    if fc in (0x0F, 0x10):
        out = [8]  # normal response: addr fc start qty crc
        if len(buf) >= 7:
            out.append(9 + buf[6])  # request with payload
        return out
    return []


def crc_ok(buf: bytes, n: int) -> bool:
    return modbus_crc16(buf[: n - 2]) == buf[n - 2] | (buf[n - 1] << 8)


def extract_frames(buf: bytes):
    """Split buffer into (frame, ...) list; returns (frames, dropped, rest)."""
    frames = []
    dropped = 0
    while True:
        if len(buf) < 2:
            break
        cands = [c for c in dict.fromkeys(candidates(buf)) if c >= 4]
        if not cands:
            dropped += 1
            buf = buf[1:]
            continue
        valid = [c for c in cands if len(buf) >= c and crc_ok(buf, c)]
        if valid:
            n = min(valid)
            frames.append(buf[:n])
            buf = buf[n:]
            continue
        if any(len(buf) < c for c in cands):
            break  # incomplete frame, wait for more bytes
        dropped += 1
        buf = buf[1:]  # all candidates complete but bad CRC -> resync
    return frames, dropped, buf


def u16(b: bytes, i: int) -> int:
    return (b[i] << 8) | b[i + 1]


def s16(v: int) -> int:
    return v - 0x10000 if v & 0x8000 else v


def regs_of(frame: bytes) -> list:
    n = frame[2] // 2
    return [u16(frame, 3 + 2 * i) for i in range(n)]


def fmt_r(v: int) -> str:
    return f"0x{v:X}" if v >= 0x1000 else str(v)


def decode_vision(regs: list) -> str:
    n = len(regs)
    if n < 0x17:
        return " ".join(f"r{i:02X}={fmt_r(v)}" for i, v in enumerate(regs))
    out = []
    volt = regs[0] / 100.0
    cur = s16(regs[1]) / 100.0
    cells = [regs[i] for i in range(2, min(0x12, n)) if regs[i] > 0]
    cell_s = f"cells={min(cells)}-{max(cells)}mV" if cells else "cells=-"
    temps = "/".join(str(regs[i]) for i in (0x12, 0x13, 0x14) if i < n)
    out.append(f"V={volt:.2f} I={cur:g} {cell_s} t={temps} "
               f"SOC={regs[0x15]} SOH={regs[0x16]}")
    tail = [f"r{i:02X}={fmt_r(regs[i])}" for i in range(0x17, n)]
    if tail:
        out.append(" ".join(tail))
    return " ".join(out)


def decode_regs_ascii(regs: list) -> str:
    s = "".join(chr(r >> 8) + chr(r & 0xFF) for r in regs)
    printable = sum(1 for c in s if 32 <= ord(c) < 127)
    return f"ASCII='{s}'" if s and printable >= len(s) * 0.7 else None


def decode_response(start, qty, regs):
    bc = len(regs) * 2
    if start is None:
        start = 0x50 if bc == 4 else 0x00
    if start == 0x00 and (qty is None or qty <= 0x27):
        return decode_vision(regs)
    if start == 0x50:
        return f"@0x50={regs} ({', '.join(f'0x{v:04X}' for v in regs)})"
    ascii_s = decode_regs_ascii(regs)
    if ascii_s:
        return f"@0x{start:04X} {ascii_s}"
    return f"@0x{start:04X} {regs}"


class Sniffer:
    def __init__(self, out_lines):
        self.last_req = {}  # addr -> (start, qty)
        self.stats = {}
        self.writes = 0
        self.out = out_lines

    def emit(self, t, kind, text, raw: bytes):
        line = f"t={t:9.3f}  {kind:<5} {text} | {' '.join(f'{b:02x}' for b in raw)}"
        print(line, flush=True)
        self.out.append(line)

    def handle(self, t, frame: bytes):
        addr, fc = frame[0], frame[1]
        key = f"addr={addr:02X}"
        st = self.stats.setdefault(key, {"req": 0, "resp": 0})
        if fc & 0x80:
            st["resp"] += 1
            self.emit(t, "ERR", f"addr=0x{addr:02X} FC{fc & 0x7F:02X} exception={frame[2]:02X}", frame)
            return
        if fc in (0x01, 0x02, 0x03, 0x04) and len(frame) == 8:
            start, qty = u16(frame, 2), u16(frame, 4)
            self.last_req[addr] = (start, qty)
            st["req"] += 1
            self.emit(t, "REQ", f"addr=0x{addr:02X} FC{fc:02X} start=0x{start:04X} qty={qty}", frame)
        elif fc in (0x05, 0x06) or fc in (0x0F, 0x10):
            self.writes += 1
            st["req"] += 1
            self.emit(t, "WRITE", f"addr=0x{addr:02X} FC{fc:02X} !! unexpected write", frame)
        elif fc in (0x01, 0x02, 0x03, 0x04) and len(frame) >= 5:
            st["resp"] += 1
            start, qty = self.last_req.get(addr, (None, None))
            if frame[2] % 2 == 0 and frame[1] in (0x03, 0x04):
                regs = regs_of(frame)
                text = f"addr=0x{addr:02X} {decode_response(start, qty, regs)}"
            else:
                text = f"addr=0x{addr:02X} bitcount={frame[2]}"
            self.emit(t, "RESP", text, frame)
        else:
            self.emit(t, "RAW", f"addr=0x{addr:02X} FC{fc:02X}", frame)

    def summary(self, span):
        lines = [f"# span={span:.1f}s frames={sum(v['req'] + v['resp'] for v in self.stats.values())} writes={self.writes}",
                 "# per-addr: " + ", ".join(f"{k}:{v['req']}/{v['resp']}" for k, v in sorted(self.stats.items()))]
        for line in lines:
            print(line, flush=True)
            self.out.append(line)


def run_selftest() -> int:
    def mk_req(addr, start, qty):
        p = bytes([addr, 0x03]) + start.to_bytes(2, "big") + qty.to_bytes(2, "big")
        return p + modbus_crc16(p).to_bytes(2, "little")

    def mk_resp(addr, regs):
        body = b"".join(v.to_bytes(2, "big") for v in regs)
        p = bytes([addr, 0x03, len(body)]) + body
        return p + modbus_crc16(p).to_bytes(2, "little")

    regs = ([4945, 0] + [3297] * 15 + [0] + [24, 23, 23] + [50, 100, 100, 49]
            + [0] * 5 + [1] + [0] * 5 + [15, 1000, 0])
    assert len(regs) == 39, len(regs)
    req = mk_req(1, 0x0000, 0x27)
    resp = mk_resp(1, regs)
    f_req, drop, rest = extract_frames(req + b"\x00" + resp)
    assert len(f_req) == 2 and drop >= 1 and rest == b"", (len(f_req), drop, rest)
    out = []
    sn = Sniffer(out)
    sn.handle(0.0, f_req[0])
    sn.handle(0.1, f_req[1])
    text = "\n".join(out)
    assert "start=0x0000 qty=39" in text, text
    assert "V=49.45" in text and "SOC=50" in text and "SOH=100" in text, text
    assert "cells=3297-3297mV" in text and "r18=49" in text and "r24=15" in text, text
    # inverter-style qty=28 read
    resp28 = mk_resp(2, regs[:28])
    out2 = []
    sn2 = Sniffer(out2)
    sn2.handle(0.0, mk_req(2, 0x0000, 28))
    f2, d2, r2 = extract_frames(resp28)
    assert len(f2) == 1 and not r2
    sn2.handle(0.1, f2[0])
    t2 = "\n".join(out2)
    assert "SOC=50" in t2 and "r1B=" in t2 and "r24=" not in t2, t2
    # last_req must switch to 0x50 so the pair decodes as @0x50
    out3 = []
    sn3 = Sniffer(out3)
    sn3.handle(0.0, mk_req(2, 0x0050, 2))
    f3, _, _ = extract_frames(mk_resp(2, [10200, 10200]))
    sn3.handle(0.1, f3[0])
    assert "@0x50=[10200, 10200]" in "\n".join(out3), out3
    print("selftest OK")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. /dev/tty.usbserial-*")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--out", help="log file (default deye/rs485_sniff_<timestamp>.log)")
    ap.add_argument("--duration", type=float,
                    help="stop cleanly after N seconds (writes the log on exit)")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return run_selftest()

    import serial  # deferred so --selftest/--list --help work without pyserial
    from serial.tools import list_ports

    if args.list or not args.port:
        for p in list_ports.comports():
            print(f"{p.device}  {p.description}")
        if args.list and not args.port:
            return 0
        if not args.port:
            print("pass --port <device>", file=sys.stderr)
            return 1
        return 0

    out_path = args.out or f"deye/rs485_sniff_{datetime.now():%Y-%m-%d_%H-%M-%S}.log"
    out_lines = []
    header = [
        f"# RS485 sniff (passive)  port={args.port} baud={args.baud} 8N1",
        f"# Date: {datetime.now():%Y-%m-%d %H:%M:%S}",
        f"# Format: t=sec  TYPE addr=XX ... | raw hex",
    ]
    for h in header:
        print(h)

    ser = serial.Serial(args.port, args.baud, timeout=0.02)
    sn = Sniffer(out_lines)
    buf = b""

    def _stop(signum, frame):
        raise KeyboardInterrupt

    # Re-install both handlers explicitly: POSIX sets SIGINT to SIG_IGN for
    # background jobs in non-interactive shells, and SIGTERM has no default
    # Python clean exit — either would lose the in-memory log.
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    last_rx = time.monotonic()
    t0 = time.monotonic()
    deadline = t0 + args.duration if args.duration else None
    dropped = 0
    dropped_bursts = 0
    dropped_dups = 0

    def handle_batch(frames, batch_t, elapsed):
        # Some RS485 USB adapters replay a stale RX FIFO burst when the port
        # is opened: thousands of byte-identical frames arriving far faster
        # than the line baud allows. Drop whole batches over the physical
        # frame rate, and consecutive byte-identical frames (which the two
        # masters can never emit back-to-back on a healthy bus).
        nonlocal dropped_bursts, dropped_dups
        phys_cap = int(elapsed * (args.baud / 10) / 9) + 64
        if len(frames) > phys_cap:
            dropped_bursts += 1
            msg = (f"# dropped burst: {len(frames)} frames at t={elapsed:.3f}s "
                   f"(physical cap {phys_cap}) — stale adapter buffer replay")
            print(msg, flush=True)
            out_lines.append(msg)
            return
        prev = None
        for f in frames:
            if f == prev:
                dropped_dups += 1
                continue
            prev = f
            sn.handle(batch_t, f)

    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            data = ser.read(256)
            now = time.monotonic()
            if buf and now - last_rx >= SILENCE_S:
                frames, drop, buf = extract_frames(buf)
                dropped += drop
                handle_batch(frames, now - t0, now - t0)
            if data:
                buf += data
                last_rx = now
    except KeyboardInterrupt:
        pass
    finally:
        if buf:
            frames, drop, _ = extract_frames(buf)
            dropped += drop
            te = time.monotonic() - t0
            handle_batch(frames, te, te)
        sn.summary(time.monotonic() - t0)
        if dropped:
            print(f"# dropped/corrupt bytes: {dropped}")
        if dropped_bursts:
            print(f"# dropped bursts: {dropped_bursts}")
        if dropped_dups:
            print(f"# dropped duplicate frames: {dropped_dups}")
        with open(out_path, "w") as fh:
            fh.write("\n".join(header + out_lines) + "\n")
        print(f"# written: {out_path} ({len(out_lines)} frames)")
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
