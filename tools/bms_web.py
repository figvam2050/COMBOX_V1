#!/usr/bin/env python3
"""Local web monitor for Vision BMS (macOS analogue of BMS_TOOLS, v1.2: read-only).

Polls one battery pack over the RS485 tap (FC03 qty=39) and serves a
single-page dashboard on http://127.0.0.1:8080. Optional CSV logging.

Usage:
  python3 tools/bms_web.py --list
  python3 tools/bms_web.py --port /dev/cu.usbserial-A5069RR4
  python3 tools/bms_web.py --port /dev/cu.usbserial-A5069RR4 --addr 0x02
  python3 tools/bms_web.py --port /dev/cu.usbserial-A5069RR4 --passive
  python3 tools/bms_web.py --mock
  python3 tools/bms_web.py --selftest

Notes:
  - Read-only: no writes to the BMS (G5RC trap on the bus, see docs).
  - Active mode conflicts with the inverter/COM-Box polling (two masters).
    Use --passive while the system is live: it only listens and decodes the
    inverter's own traffic (qty=28 window, regs 0-27; cycles/FullCap etc.
    are outside the window and show as "—").
"""

import argparse
import csv
import json
import os
import random
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial  # pySerial (same dependency as tools/rs485_sniff.py)
except ImportError:
    serial = None

DEFAULT_PORT = "/dev/cu.usbserial-A5069RR4"
DEFAULT_BAUD = 9600
DEFAULT_ADDR = 0x01
DEFAULT_HTTP = 8080
POLL_S = 0.35
RESP_TIMEOUT_S = 1.0

# 0025 Status: enum in the low byte (0/1/2/4/8) + heating flag 0x8000,
# per MODBUS V01.01 (not a free bitfield).
STATUS_STATES = [
    (0x01, "Charging"),
    (0x02, "Discharging"),
    (0x04, "Protect"),
    (0x08, "Charging Lmt"),
]
WARN_BITS = [
    (0x0001, "Pack OV"), (0x0002, "Cell OV"), (0x0004, "Pack UV"),
    (0x0008, "Cell UV"), (0x0010, "Charge OC"), (0x0020, "Discharge OC"),
    (0x0040, "Temp anomaly"), (0x0080, "MOS OT"), (0x0100, "Charge OT"),
    (0x0200, "Discharge OT"), (0x0400, "Charge UT"), (0x0800, "Discharge UT"),
    (0x1000, "Low capacity"), (0x2000, "Float stopped"),
]
# 0027 Protection: same bits 0x0001..0x1000, but 0x2000 = Discharge SC and
# 0x4000 = Reverse (the extra LED from BMS_TOOLS; no Float stop here).
PROT_BITS = WARN_BITS[:13] + [(0x2000, "Discharge SC"), (0x4000, "Reverse")]
ERROR_BITS = [
    (0x0001, "Voltage error"), (0x0002, "Temperature error"),
    (0x0004, "Current sensor"), (0x0010, "Cell unbalance"),
]


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def crc_bytes(data: bytes) -> bytes:
    crc = modbus_crc16(data)
    return bytes([crc & 0xFF, crc >> 8])


def parse_hex_addr(s: str) -> int:
    return int(s, 16) if s.lower().startswith("0x") else int(s)


def s16(u: int) -> int:
    return u - 0x10000 if u & 0x8000 else u


def b8(u: int) -> int:
    return u - 0x100 if u & 0x80 else u


def bits_of(value: int, table) -> list:
    return [{"name": name, "on": bool(value & mask)} for mask, name in table]


def decode_ascii_regs(regs_slice: list) -> str:
    chars = []
    for reg in regs_slice:
        chars.append(chr((reg >> 8) & 0xFF))
        chars.append(chr(reg & 0xFF))
    return "".join(c for c in chars if 32 <= ord(c) <= 126).strip()


def decode_fw_version(reg: int) -> str:
    hi = (reg >> 8) & 0xFF
    lo = reg & 0xFF
    if hi > 0 or lo > 0:
        return f"V{hi:02d}R{lo:02d}"
    return f"0x{reg:04X}"


def decode_dev_info_block(regs: list) -> dict:
    model = decode_ascii_regs(regs[0:15]) if len(regs) >= 15 else None
    fw = decode_fw_version(regs[15]) if len(regs) >= 16 else None
    serial_no = decode_ascii_regs(regs[16:23]) if len(regs) >= 23 else None
    return {"model": model, "fw_version": fw, "serial": serial_no}


def decode_regs(regs: list) -> dict:
    n = len(regs)

    def r(i, default=None):
        return regs[i] if i < n else default

    cell_num = r(36) or 15
    cells = [regs[2 + i] for i in range(16) if 2 + i < n]
    cells = [c for c in cells[:cell_num] if c > 0] or [c for c in cells[:15] if c]
    alive = [c for c in cells if c > 0] or cells
    status = r(25, 0)
    low = status & 0x00FF
    state = next((name for v, name in STATUS_STATES if low == v),
                 "Standby" if low == 0 else f"Unknown({low:#04x})")
    info = decode_dev_info_block(regs[105:128]) if n >= 128 else {}
    return {
        "voltage": regs[0] / 100.0,
        "current": s16(regs[1]) / 100.0,
        "cells": cells,
        "cell_num": cell_num,
        "cell_min": min(alive),
        "cell_max": max(alive),
        "cell_diff": max(alive) - min(alive),
        "temp_pcb": s16(r(18, 0)),
        "temp_avg": s16(r(19, 0)),
        "temp_max": s16(r(20, 0)),
        "cap_remaining": r(21),
        "max_charge_a": r(22),
        "soh": r(23),
        "soc": r(24),
        "status": status,
        "status_name": state + (" + Heating" if status & 0x8000 else ""),
        "status_bits": ([{"name": name, "on": low == v} for v, name in STATUS_STATES]
                        + [{"name": "Heating ON", "on": bool(status & 0x8000)}]),
        "warning": r(26, 0),
        "warning_bits": bits_of(r(26, 0), WARN_BITS),
        "protection": r(27, 0),
        "protection_bits": bits_of(r(27, 0), PROT_BITS),
        "error_code": r(28),
        "error_bits": bits_of(r(28, 0) or 0, ERROR_BITS),
        "cycles": ((r(29, 0) << 16) | r(30, 0)) if n > 30 else None,
        "full_cap_ah": (((r(31, 0) << 16) | r(32, 0)) / 3.6e6) if n > 32 else None,
        "temp_sensors": ([b8((regs[33 + i // 2] >> (8 if i % 2 == 0 else 0)) & 0xFF)
                          for i in range(6)] if n > 35 else None),
        "balance": r(38),
        "balance_bits": ([{"n": i + 1, "on": bool(r(38, 0) & (1 << i))}
                          for i in range(cell_num)] if n > 38 else None),
        "designed_cap_ah": (regs[37] / 10.0) if n > 37 else None,
        "model": info.get("model"),
        "fw_version": info.get("fw_version"),
        "serial": info.get("serial"),
    }


def parse_bus(buf: bytes):
    """Extract Modbus FC03 frames (inverter requests/responses) from a raw
    stream chunk. Returns (frames, rest); a frame is (kind, addr, payload):
    kind 'req' (start, qty) or 'resp' (regs list). CRC-validated, resync
    by one byte on garbage."""
    frames = []
    i = 0
    n = len(buf)
    while i < n:
        if i + 3 > n:
            break
        addr, fc, b2 = buf[i], buf[i + 1], buf[i + 2]
        if fc != 0x03 or addr == 0:
            i += 1
            continue
        if b2 == 0x00:                       # request: start_hi=0, len 8
            ln = 8
            kind = "req"
        else:                                # response: bytecount + crc
            ln = b2 + 5
            kind = "resp"
        if i + ln > n:
            break                            # incomplete, wait for more
        frame = buf[i:i + ln]
        if crc_bytes(frame[:-2]) != frame[-2:]:
            i += 1                           # garbage, resync
            continue
        if kind == "req":
            start = (frame[3] << 8) | frame[4]
            qty = (frame[5] << 8) | frame[6]
            frames.append(("req", addr, (start, qty)))
        else:
            data = frame[3:-2]
            regs = [data[j] << 8 | data[j + 1] for j in range(0, len(data) - 1, 2)]
            frames.append(("resp", addr, regs))
        i += ln
    return frames, buf[i:]


class BmsBus:
    PASSIVE_STALE_S = 15.0   # inverter sweeps 0x01..0x10 in ~10.5 s
    PASSIVE_GAP_S = 0.15     # read timeout in passive mode (> REQ→RESP gap)

    def __init__(self, port: str, baud: int, addr: int, mock: bool,
                 passive: bool = False):
        self.port = port
        self.baud = baud
        self.addr = addr
        self.mock = mock
        self.passive = passive
        self.ser = None
        self.lock = threading.Lock()
        self.state = {"ok": False, "err": "starting", "ts": None, "raw": [],
                      "mode": "passive" if passive else "active", "seen": []}
        self.log_on = False
        self.log_path = None
        self.log_rows = 0
        self._mock_t = 0.0
        self._mock_v = 3452
        self._stop = False
        self.dev_info = {}

    def open(self) -> bool:
        if self.mock:
            return True
        if serial is None:
            self.state["err"] = "pySerial not installed (pip3 install pyserial)"
            return False
        try:
            to = self.PASSIVE_GAP_S if self.passive else RESP_TIMEOUT_S
            self.ser = serial.Serial(self.port, self.baud, timeout=to)
            return True
        except Exception as e:
            self.state["err"] = f"open {self.port}: {e}"
            return False

    def read_regs(self) -> list:
        req = bytes([self.addr, 0x03, 0x00, 0x00, 0x00, 0x27]) + crc_bytes(
            bytes([self.addr, 0x03, 0x00, 0x00, 0x00, 0x27]))
        self.ser.reset_input_buffer()
        self.ser.write(req)
        hdr = self.ser.read(3)
        if len(hdr) < 3 or hdr[0] != self.addr or hdr[1] != 0x03:
            raise IOError(f"bad response header {hdr.hex()}")
        bytecount = hdr[2]
        rest = self.ser.read(bytecount + 2)
        if len(rest) < bytecount + 2:
            raise IOError("short response")
        frame = hdr + rest
        if crc_bytes(frame[:-2]) != frame[-2:]:
            raise IOError("CRC mismatch")
        data = frame[3:3 + bytecount]
        return [data[i] << 8 | data[i + 1] for i in range(0, bytecount - 1, 2)]

    def read_dev_info(self) -> dict:
        try:
            req = bytes([self.addr, 0x03, 0x00, 0x69, 0x00, 0x17]) + crc_bytes(
                bytes([self.addr, 0x03, 0x00, 0x69, 0x00, 0x17]))
            self.ser.reset_input_buffer()
            self.ser.write(req)
            hdr = self.ser.read(3)
            if len(hdr) < 3 or hdr[0] != self.addr or hdr[1] != 0x03:
                return {}
            bytecount = hdr[2]
            rest = self.ser.read(bytecount + 2)
            if len(rest) < bytecount + 2:
                return {}
            frame = hdr + rest
            if crc_bytes(frame[:-2]) != frame[-2:]:
                return {}
            data = frame[3:3 + bytecount]
            regs = [data[i] << 8 | data[i + 1] for i in range(0, bytecount - 1, 2)]
            return decode_dev_info_block(regs)
        except Exception:
            return {}

    def mock_regs(self) -> list:
        if not self.dev_info:
            self.dev_info = {
                "model": "V-LFP48100-15S",
                "fw_version": "V10R18",
                "serial": "SN-20261022001",
            }
        self._mock_t += 1
        rnd = random.Random(self._mock_t // 10)
        self._mock_v += rnd.choice([-1, 0, 0, 1])
        base = max(3300, min(3600, self._mock_v))
        cells = [base + rnd.randint(-8, 8) for _ in range(15)] + [0]
        voltage = sum(cells[:15]) // 10
        current = 655 if self._mock_t % 400 < 320 else 0
        regs = [0] * 39
        regs[0] = voltage
        regs[1] = current
        for i, c in enumerate(cells):
            regs[2 + i] = c
        regs[18], regs[19], regs[20] = 23, 22, 23
        regs[21] = min(100, (base - 3300) // 3)
        regs[22] = 100
        regs[23] = 100
        regs[24] = regs[21]
        regs[25] = 1 if current else 0
        regs[26] = 0
        regs[27] = 0
        regs[28] = 0
        regs[29], regs[30] = 0, 1   # cycles hi-first: (r29<<16)|r30 = 1
        regs[31], regs[32] = 0x1575, 0x2A00   # 360,000,000 mAs = 100.0 Ah
        regs[33], regs[34], regs[35] = 0x1616, 0x1616, 0   # Temp01-04 = 22
        regs[36] = 15
        regs[37] = 1000
        regs[38] = 1 if (base > 3400 and max(cells[:15]) - min(cells[:15]) > 40) else 0
        return regs

    def log_row(self, regs: list, addr: int = None) -> None:
        if not self.log_on or not self.log_path:
            return
        new_file = not os.path.exists(self.log_path)
        with open(self.log_path, "a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(["ts", "addr"] + [f"r{i:02d}" for i in range(len(regs))])
            w.writerow([datetime.now().isoformat(timespec="seconds"),
                        f"0x{(self.addr if addr is None else addr):02X}"] + regs)
            self.log_rows += 1

    def loop(self) -> None:
        if self.passive:
            self.passive_loop()
        else:
            self.active_loop()

    def active_loop(self) -> None:
        while not self._stop:
            if not self.mock and (self.ser is None or not self.ser.is_open):
                if not self.open():
                    time.sleep(3.0)
                    continue
            try:
                if not self.mock and not self.dev_info:
                    self.dev_info = self.read_dev_info()
                regs = self.mock_regs() if self.mock else self.read_regs()
                with self.lock:
                    self.state = {
                        "ok": True, "err": None,
                        "ts": datetime.now().isoformat(timespec="seconds"),
                        "addr": f"0x{self.addr:02X}", "raw": regs,
                        "parsed": decode_regs(regs),
                        "mode": "active", "seen": [f"0x{self.addr:02X}"],
                        "log_on": self.log_on, "log_path": self.log_path,
                        "log_rows": self.log_rows,
                    }
                self.log_row(regs)
            except Exception as e:
                with self.lock:
                    self.state = dict(self.state, ok=False, err=str(e),
                                      ts=datetime.now().isoformat(timespec="seconds"))
                if not self.mock:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
            time.sleep(POLL_S)

    def passive_loop(self) -> None:
        """Listen-only: parse the inverter's own polling traffic (FC03
        qty=28 + 0x50 qty=2). No transmission, no bus collisions."""
        buf = b""
        last_seen: dict = {}
        last_regs: dict = {}
        while not self._stop:
            if self.ser is None or not self.ser.is_open:
                if not self.open():
                    time.sleep(3.0)
                    continue
            try:
                chunk = self.ser.read(512)
                if chunk:
                    buf += chunk
                    frames, buf = parse_bus(buf)
                else:
                    buf = b""      # silence ⇒ drop incomplete garbage
                    frames = []
                now = time.time()
                for kind, addr, payload in frames:
                    if kind == "resp" and len(payload) >= 28:
                        last_seen[addr] = now
                        last_regs[addr] = payload
                        self.log_row(payload, addr)   # every pack seen, not only --addr
                fresh = [a for a in sorted(last_seen)
                         if now - last_seen[a] <= self.PASSIVE_STALE_S]
                seen_hex = [f"0x{a:02X}" for a in fresh]
                mine = [p for k, a, p in frames
                        if k == "resp" and a == self.addr and len(p) >= 28]
                iso = datetime.now().isoformat(timespec="seconds")
                if mine:
                    regs = mine[-1]
                    with self.lock:
                        self.state = {
                            "ok": True, "err": None, "ts": iso,
                            "addr": f"0x{self.addr:02X}", "raw": regs,
                            "parsed": decode_regs(regs),
                            "mode": "passive", "seen": seen_hex,
                            "log_on": self.log_on, "log_path": self.log_path,
                            "log_rows": self.log_rows,
                        }
                else:
                    stale = (self.addr not in fresh)
                    err = (f"немає кадрів 0x{self.addr:02X} >{int(self.PASSIVE_STALE_S)} с "
                           f"(інвертор мовчить?)") if stale else None
                    with self.lock:
                        self.state = dict(
                            self.state,
                            ok=(self.state.get("ok") and not stale),
                            err=(err if stale else self.state.get("err")),
                            seen=seen_hex, ts=iso)
            except Exception as e:
                with self.lock:
                    self.state = dict(self.state, ok=False, err=str(e),
                                      ts=datetime.now().isoformat(timespec="seconds"))
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def snapshot(self) -> dict:
        with self.lock:
            st = dict(self.state)
            if st.get("parsed") and self.dev_info:
                p = dict(st["parsed"])
                for k, v in self.dev_info.items():
                    if not p.get(k):
                        p[k] = v
                st["parsed"] = p
            return st


HTML = """<!doctype html>
<html lang="uk"><head><meta charset="utf-8">
<title>BMS Web Monitor</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#111417;--card:#1a1f24;--fg:#d8dee4;--dim:#7d8894;--ok:#3fb950;
--bad:#f85149;--warn:#d29922;--acc:#58a6ff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 -apple-system,Segoe UI,sans-serif;padding:16px}
h1{font-size:18px;margin:0 0 4px}
.sub{color:var(--dim);margin-bottom:16px}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.card{background:var(--card);border-radius:10px;padding:14px}
.card h2{font-size:13px;text-transform:uppercase;letter-spacing:.06em;color:var(--dim);margin:0 0 10px}
.big{display:flex;gap:24px;flex-wrap:wrap}
.big div span{display:block}
.big .v{font-size:28px;font-weight:600}
.big .l{font-size:11px;color:var(--dim);text-transform:uppercase}
.pill{display:inline-block;padding:2px 10px;border-radius:20px;font-weight:600;background:#30363d}
.pill.ok{background:#1a3d24;color:var(--ok)}
.pill.bad{background:#4a1a18;color:var(--bad)}
.cells{display:grid;grid-template-columns:repeat(auto-fill,minmax(88px,1fr));gap:6px}
.cell{background:#232a31;border-radius:6px;padding:6px;text-align:center}
.cell .n{font-size:10px;color:var(--dim)}
.cell .mv{font-weight:600;font-variant-numeric:tabular-nums}
.cell.min{outline:1px solid var(--bad)}
.cell.max{outline:1px solid var(--ok)}
.cell.bal-active{box-shadow:0 0 8px var(--warn);border:1px solid var(--warn);background:#2d2415}
.b-tag{background:var(--warn);color:#111417;padding:1px 4px;border-radius:3px;font-size:9px;font-weight:700;margin-left:4px}
.leds{display:grid;grid-template-columns:1fr 1fr;gap:4px 12px}
.led{display:flex;align-items:center;gap:8px;color:var(--dim)}
.led i{width:12px;height:12px;border-radius:50%;background:#30363d;display:inline-block;flex:none}
.led.on{color:var(--fg)}
.led.on i{background:var(--bad);box-shadow:0 0 6px var(--bad)}
.led.onw i{background:var(--warn);box-shadow:0 0 6px var(--warn)}
.led.onok i{background:var(--ok);box-shadow:0 0 6px var(--ok)}
.bal{display:flex;gap:4px;flex-wrap:wrap}
.bal span{width:30px;height:30px;border-radius:6px;background:#232a31;display:grid;place-items:center;font-size:11px;color:var(--dim)}
.bal span.on{background:#3b2d12;color:var(--warn);font-weight:700}
#conn{float:right}
button{background:var(--acc);border:0;color:#0b0f14;padding:7px 16px;border-radius:7px;font-weight:600;cursor:pointer}
button.off{background:#30363d;color:var(--dim)}
.err{color:var(--bad);font-weight:600;margin-top:8px}
.foot{color:var(--dim);font-size:12px;margin-top:14px}
table.kv td{padding:2px 10px 2px 0;color:var(--dim)}
table.kv td:last-child{color:var(--fg);font-variant-numeric:tabular-nums;text-align:right}
</style></head><body>
<h1>Vision BMS Web Monitor <span id="conn" class="pill">…</span></h1>
<div class="sub">RS485 · <span id="mode">—</span> · <span id="seen" style="color:var(--dim)"></span> · <span id="ts">—</span></div>
<div class="grid">
<div class="card"><h2>Живлення</h2><div class="big">
<div><span class="l">Напруга</span><span class="v" id="v">—</span></div>
<div><span class="l">Струм</span><span class="v" id="i">—</span></div>
<div><span class="l">Стан</span><span class="v"><span class="pill" id="st">—</span></span></div>
</div><div class="err" id="err"></div>
<table class="kv" style="margin-top:10px">
<tr><td>Модель</td><td id="bmsmodel">—</td></tr>
<tr><td>Прошивка BMS</td><td id="bmsfw">—</td></tr>
<tr><td>Серійний номер</td><td id="bmssn">—</td></tr>
<tr><td>SOC / Cap Remaining</td><td id="soc">—</td></tr>
<tr><td>SOH / Max charge</td><td id="soh">—</td></tr>
<tr><td>Цикли</td><td id="cyc">—</td></tr>
<tr><td>Номінал</td><td id="cap">—</td></tr>
<tr><td>Full Capacity (0031)</td><td id="fcap">—</td></tr>
</table></div>
<div class="card"><h2>Комірки <span id="difft" style="color:var(--dim)"></span></h2>
<div class="cells" id="cells"></div></div>
<div class="card"><h2>Температури</h2><table class="kv">
<tr><td>PCB</td><td id="t0">—</td></tr><tr><td>Avg</td><td id="t1">—</td></tr>
<tr><td>Max</td><td id="t2">—</td></tr>
<tr><td>Temp01</td><td id="t3">—</td></tr><tr><td>Temp02</td><td id="t4">—</td></tr>
<tr><td>Temp03</td><td id="t5">—</td></tr><tr><td>Temp04</td><td id="t6">—</td></tr></table></div>
<div class="card"><h2>Status (reg 0025) <span id="statusraw" style="color:var(--dim);font-weight:400"></span></h2><div class="leds" id="stbits"></div></div>
<div class="card"><h2>Warning (0026) <span id="warnraw" style="color:var(--dim);font-weight:400"></span></h2><div class="leds" id="warn"></div></div>
<div class="card"><h2>Protection (0027) <span id="protraw" style="color:var(--dim);font-weight:400"></span></h2><div class="leds" id="prot"></div></div>
<div class="card"><h2>Error (0028) <span id="errraw" style="color:var(--dim);font-weight:400"></span></h2><div class="leds" id="errbits"></div></div>
<div class="card"><h2>Balance (0038)</h2><div class="bal" id="bal"></div></div>
<div class="card"><h2>CSV-лог</h2>
<p>Файл: <b id="logpath">—</b> · рядків: <span id="logrows">0</span></p>
<button id="logbtn" onclick="tglLog()">Старт</button></div>
</div>
<div class="foot">Тільки читання. Активний: python3 tools/bms_web.py --port /dev/cu.usbserial-A5069RR4 (лише коли шина вільна від інвертора/коробки). Пасивний (паралельно з інвертором, без TX): додати --passive.</div>
<script>
let logOn=false;
function leds(el,arr,cls){el.innerHTML=arr.map(b=>
`<div class="led ${b.on?cls:''}"><i></i>${b.name}</div>`).join('')}
async function tick(){
 try{
  const r=await fetch('/api/data'); const d=await r.json();
  const c=document.getElementById('conn');
  c.textContent=d.ok?'ONLINE '+(d.addr||''):'OFFLINE';
  c.className='pill '+(d.ok?'ok':'bad');
  const pas=d.mode==='passive';
  document.getElementById('mode').textContent=pas?'пасивний (трафік інвертора)':'активний опит FC03 qty=39';
  document.getElementById('seen').textContent=d.seen&&d.seen.length?'бачить: '+d.seen.join(' '):'';
  document.getElementById('ts').textContent=d.ts||'—';
  document.getElementById('err').textContent=d.err||'';
  if(!d.ok||!d.parsed){return}
  const p=d.parsed;
  document.getElementById('v').textContent=p.voltage.toFixed(2)+' В';
  document.getElementById('i').textContent=(p.current==null?'—':p.current.toFixed(1)+' А');
  const st=document.getElementById('st');
  st.textContent=p.status_name;
  st.className='pill '+(p.status_bits[2].on?'bad':'ok');
  document.getElementById('bmsmodel').textContent=p.model||'—';
  document.getElementById('bmsfw').textContent=p.fw_version||'—';
  document.getElementById('bmssn').textContent=p.serial||'—';
  document.getElementById('soc').textContent=(p.soc==null?'—':p.soc+' %')+' / '+(p.cap_remaining==null?'—':p.cap_remaining+' А·ч');
  document.getElementById('soh').textContent=(p.soh==null?'—':p.soh+' %')+' / '+(p.max_charge_a==null?'—':p.max_charge_a+' А');
  document.getElementById('cyc').textContent=p.cycles==null?'—':p.cycles;
  document.getElementById('cap').textContent=(p.designed_cap_ah==null?'—':p.designed_cap_ah+' А·ч')+' · '+p.cell_num+'S';
  document.getElementById('fcap').textContent=p.full_cap_ah==null?'—':p.full_cap_ah.toFixed(1)+' А·ч';
  document.getElementById('difft').textContent='Δ='+p.cell_diff+' мВ';
  const ci=document.getElementById('cells');
  ci.innerHTML=p.cells.map((mv,i)=>{
   let cls='cell';
   if(mv===p.cell_min)cls+=' min'; if(mv===p.cell_max)cls+=' max';
   const balOn = p.balance_bits && p.balance_bits[i] && p.balance_bits[i].on;
   if(balOn) cls += ' bal-active';
   return `<div class="${cls}"><div class="n">#${i+1}${balOn?' <span class="b-tag">BAL</span>':''}</div><div class="mv">${mv}</div></div>`}).join('');
  document.getElementById('t0').textContent=(p.temp_pcb==null?'—':p.temp_pcb+' °C');
  document.getElementById('t1').textContent=(p.temp_avg==null?'—':p.temp_avg+' °C');
  document.getElementById('t2').textContent=(p.temp_max==null?'—':p.temp_max+' °C');
  if(p.temp_sensors) p.temp_sensors.slice(0,4).forEach((tv,i)=>{
   document.getElementById('t'+(i+3)).textContent=tv+' °C'});
  const hx=v=>'0x'+(v==null?'????':v.toString(16).toUpperCase().padStart(4,'0'));
  document.getElementById('statusraw').textContent=hx(p.status);
  document.getElementById('warnraw').textContent=hx(p.warning);
  document.getElementById('protraw').textContent=hx(p.protection);
  document.getElementById('errraw').textContent=hx(p.error_code);
  leds(document.getElementById('stbits'),p.status_bits,'onok');
  leds(document.getElementById('warn'),p.warning_bits,'on');
  leds(document.getElementById('prot'),p.protection_bits,'on');
  leds(document.getElementById('errbits'),p.error_bits,'onw');
  document.getElementById('bal').innerHTML=p.balance_bits?p.balance_bits.map(b=>
   `<span class="${b.on?'on':''}">${b.n}</span>`).join(''):'—';
  logOn=d.log_on; document.getElementById('logpath').textContent=d.log_path||'—';
  document.getElementById('logrows').textContent=d.log_rows||0;
  const b=document.getElementById('logbtn');
  b.textContent=logOn?'Стоп':'Старт'; b.className=logOn?'off':'';
 }catch(e){const c=document.getElementById('conn');c.textContent='NO SERVER';c.className='pill bad'}
}
async function tglLog(){
 await fetch('/api/log',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({on:!logOn})}); tick();
}
tick(); setInterval(tick,1000);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    bus: BmsBus = None

    def log_message(self, *args):
        pass

    def _send(self, code, body, ctype):
        data = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, HTML, "text/html; charset=utf-8")
        elif self.path == "/api/data":
            self._send(200, json.dumps(self.bus.snapshot()),
                       "application/json")
        else:
            self._send(404, "not found", "text/plain")

    def do_POST(self):
        if self.path == "/api/log":
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except Exception:
                req = {}
            on = bool(req.get("on"))
            bus = self.bus
            if on and not bus.log_on:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                bus.log_path = req.get("file") or f"bms_monitor_{ts}.csv"
                bus.log_rows = 0
            bus.log_on = on
            self._send(200, json.dumps({"ok": True, "on": on}),
                       "application/json")
        else:
            self._send(404, "not found", "text/plain")


def selftest() -> int:
    ok = True
    req = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x27])
    assert crc_bytes(req) == bytes([0x05, 0xD0]), "CRC vector failed"
    req16 = bytes([0x10, 0x03, 0x00, 0x00, 0x00, 0x27])
    assert crc_bytes(req16) == bytes([0x06, 0x91]), "CRC vector (live BMS Tools) failed"
    regs = [0] * 39
    regs[0], regs[1] = 5231, 65535
    regs[2] = 3456
    regs[36] = 15
    p = decode_regs(regs)
    assert abs(p["voltage"] - 52.31) < 1e-6, "voltage decode"
    assert abs(p["current"] + 0.01) < 1e-6, "current decode"
    assert p["cell_min"] == 3456, "cell decode"
    regs[25] = 0x0004
    assert decode_regs(regs)["status_bits"][2]["on"], "status bit"
    # --- offline checks vs MODBUS V01.01 + BMS Tools (22.10.2026) ---
    regs[25] = 0x8001
    d = decode_regs(regs)
    assert d["status_name"] == "Charging + Heating", f"status enum+heating: {d['status_name']}"
    assert d["status_bits"][4]["on"], "heating bit"
    regs[25] = 0
    assert decode_regs(regs)["status_name"] == "Standby", "status standby"
    regs[25] = 0x0003
    assert decode_regs(regs)["status_name"] == "Unknown(0x03)", "status unknown enum"
    regs[29], regs[30] = 0, 1
    assert decode_regs(regs)["cycles"] == 1, "cycles hi-first (BMS Tools: CyclesNum=1)"
    regs[31], regs[32] = 0x1575, 0x2A00
    assert abs(decode_regs(regs)["full_cap_ah"] - 100.0) < 1e-6, "full cap 360e6 mAs = 100.0 Ah"
    regs[33], regs[34], regs[35] = 0x1414, 0x1514, 0
    ts = decode_regs(regs)["temp_sensors"]
    assert ts == [20, 20, 21, 20, 0, 0], f"temp sensors: {ts}"
    regs[27] = 0x2000
    on = [b["name"] for b in decode_regs(regs)["protection_bits"] if b["on"]]
    assert on == ["Discharge SC"], f"prot 0x2000: {on}"
    regs[27] = 0x4000
    on = [b["name"] for b in decode_regs(regs)["protection_bits"] if b["on"]]
    assert on == ["Reverse"], f"prot 0x4000: {on}"
    regs[27] = 0
    regs[26] = 0x2000
    on = [b["name"] for b in decode_regs(regs)["warning_bits"] if b["on"]]
    assert on == ["Float stopped"], f"warn 0x2000: {on}"
    assert len(decode_regs(regs)["warning_bits"]) == 14, "warn = 14 bits"
    assert len(decode_regs(regs)["protection_bits"]) == 15, "prot = 15 bits"
    # --- passive mode: frame parser + partial (qty=28) decode ---
    req_body = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x1C])
    req = req_body + crc_bytes(req_body)
    regs28 = [5306, 0] + [3500] * 15 + [0, 21, 20, 21,
                                        100, 100, 100, 100, 0, 2, 0x1000]
    assert len(regs28) == 28, "qty28 register count"
    resp_body = bytes([0x01, 0x03, 56]) + b"".join(
        v.to_bytes(2, "big") for v in regs28)
    resp = resp_body + crc_bytes(resp_body)
    stream = b"\x00\xFFgarbage" + req + b"\xFF" + resp + b"\x00\x00"
    frames, rest = parse_bus(stream)
    kinds = [(k, a) for k, a, _ in frames]
    assert ("req", 1) in kinds, f"parse request: {kinds}"
    assert ("resp", 1) in kinds, f"parse response: {kinds}"
    r28 = [p for k, a, p in frames if k == "resp"][0]
    d28 = decode_regs(r28)
    assert abs(d28["voltage"] - 53.06) < 1e-6, "partial voltage"
    assert d28["warning"] == 2 and d28["protection"] == 0x1000, "partial r1A/r1B"
    assert d28["soc"] == 100 and d28["cap_remaining"] == 100, "partial soc/cap"
    assert d28["cycles"] is None and d28["full_cap_ah"] is None, "partial no tail"
    assert d28["designed_cap_ah"] is None and d28["balance_bits"] is None
    assert d28["error_code"] is None and len(d28["cells"]) == 15, "partial cells"
    assert all(not b["on"] for b in d28["error_bits"]), "partial error bits off"
    # --- device info block & active balance test ---
    info_block = [
        0x562D, 0x4C46, 0x5034, 0x3831, 0x3030, 0x2D31, 0x3553, 0x2020,
        0x2020, 0x2020, 0x2020, 0x2020, 0x2020, 0x2020, 0x2020,  # V-LFP48100-15S
        0x0A12,  # V10R18
        0x534E, 0x2D30, 0x3031, 0x2020, 0x2020, 0x2020, 0x2020,  # SN-001
    ]
    parsed_info = decode_dev_info_block(info_block)
    assert parsed_info["model"] == "V-LFP48100-15S", f"model: {parsed_info['model']}"
    assert parsed_info["fw_version"] == "V10R18", f"fw: {parsed_info['fw_version']}"
    assert parsed_info["serial"] == "SN-001", f"sn: {parsed_info['serial']}"
    regs[38] = 0x0005
    d_bal = decode_regs(regs)
    assert d_bal["balance_bits"][0]["on"] and not d_bal["balance_bits"][1]["on"] and d_bal["balance_bits"][2]["on"]
    print("selftest OK")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--addr", type=parse_hex_addr, default=DEFAULT_ADDR,
                    help="slave address, decimal or 0xNN")
    ap.add_argument("--http", type=int, default=DEFAULT_HTTP)
    ap.add_argument("--mock", action="store_true", help="no serial, fake data")
    ap.add_argument("--passive", action="store_true",
                    help="listen only: parse inverter traffic (no TX, no collisions)")
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if args.list:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(p.device, p.description)
        return 0
    if args.passive and args.mock:
        print("--passive несумісний з --mock")
        return 2

    bus = BmsBus(args.port, args.baud, args.addr, args.mock, passive=args.passive)
    threading.Thread(target=bus.loop, daemon=True).start()
    Handler.bus = bus
    srv = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    mode = ("MOCK" if args.mock else
            f"{args.port} @ {args.baud}" + (" PASSIVE" if args.passive else ""))
    print(f"BMS web monitor: http://127.0.0.1:{args.http}  [{mode}]  "
          f"addr=0x{args.addr:02X}  (Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bus._stop = True
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
