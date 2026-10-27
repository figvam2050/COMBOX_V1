#!/usr/bin/env python3
"""Local multi-pack web monitor for Vision BMS (macOS analogue of BMS_TOOLS, v2.0: read-only).

Monitors multiple battery packs over the RS485 tap (Modbus RTU addresses 0x01..0x10)
and serves a tabbed responsive dashboard on http://127.0.0.1:8080.

Features:
  - Multi-pack auto-discovery: sweeps 0x01..0x10, discovers active packs, and fast-polls
    only available packs.
  - Hot-plug / unplug tracking: detects when packs are added or disconnected.
  - Mode switching via UI or CLI:
      * Active Polling (Master): queries packs with FC03 (only when bus is free of COM-Box/inverter).
      * Passive Scanning (Sniffer): zero TX transmissions, safely decodes live bus traffic
        without any collisions.
  - UI Layout:
      * Main "Загальний огляд" (Summary) tab with bank-level totals (voltage, current, power,
        average SOC, bank-wide cell delta, global min/max cells, summary table).
      * Dedicated individual tabs for each battery pack with complete cell voltages,
        active balancing tags [BAL], temperatures, LED status/warning/protection flags.
  - Live mode switcher and "🔄 Пересканувати всі 16" buttons directly in the web UI.
  - CSV logging of all active packs.

Usage:
  python3 tools/bms_web.py --list
  python3 tools/bms_web.py --port /dev/cu.usbserial-A5069RR4
  python3 tools/bms_web.py --port /dev/cu.usbserial-A5069RR4 --passive
  python3 tools/bms_web.py --mock
  python3 tools/bms_web.py --selftest
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
    import serial  # pySerial
except ImportError:
    serial = None

DEFAULT_PORT = "/dev/cu.usbserial-A5069RR4"
DEFAULT_BAUD = 9600
DEFAULT_ADDR = 0x01
DEFAULT_HTTP = 8080
POLL_S = 0.35
RESP_TIMEOUT_S = 0.4
PROBE_TIMEOUT_S = 0.12

# 0025 Status: enum in the low byte (0/1/2/4/8) + heating flag 0x8000, per MODBUS V01.01.
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
# 0027 Protection: same bits 0x0001..0x1000, plus 0x2000 = Discharge SC and 0x4000 = Reverse
PROT_BITS = WARN_BITS[:13] + [(0x2000, "Discharge SC"), (0x4000, "Reverse")]
ERROR_BITS = [
    (0x0001, "Voltage error"), (0x0002, "Temperature error"),
    (0x0004, "Current sensor"), (0x0010, "Cell unbalance"),
]


def find_usb_rs485_port() -> tuple:
    """Auto-detects USB-to-RS485/UART converter port across Windows, macOS, and Linux.
    Returns (port_device, description) or (None, reason)."""
    if serial is None:
        return None, "pySerial not installed"
    try:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
    except Exception as e:
        return None, str(e)

    if not ports:
        return None, "жодного COM-порту не знайдено (перевірте USB-перехідник)"

    # Known USB-UART chip VIDs (FTDI, WCH CH340, SiLabs CP210x, Prolific)
    KNOWN_VIDS = {
        0x0403: "FTDI",
        0x1A86: "CH340/CH341",
        0x10C4: "CP210x",
        0x067B: "PL2303",
    }

    candidates = []
    for p in ports:
        desc = (p.description or "").lower()
        mfg = (getattr(p, "manufacturer", "") or "").lower()
        vid = getattr(p, "vid", None)
        hwid = (getattr(p, "hwid", "") or "").lower()
        dev = p.device

        # Skip virtual/built-in debug/bluetooth ports on macOS
        if any(ign in dev.lower() for ign in ("bluetooth", "debug-console", "wlan-debug")):
            continue

        is_usb = (
            vid in KNOWN_VIDS
            or any(k in desc for k in ("ftdi", "ft232", "ch340", "ch341", "cp210", "prolific", "usb-serial", "usb serial", "uart", "rs485", "rs-485"))
            or any(k in mfg for k in ("ftdi", "wch", "silicon labs", "prolific"))
            or "usb" in hwid
            or "usbserial" in dev.lower()
            or "ttyusb" in dev.lower()
            or "ttyacm" in dev.lower()
        )
        if is_usb:
            chip = KNOWN_VIDS.get(vid) or p.description
            candidates.append((dev, f"{p.description} [{chip}]" if chip != p.description else p.description))

    if len(candidates) == 1:
        return candidates[0]
    elif len(candidates) > 1:
        # Prefer FTDI if present among multiple adapters
        for dev, desc in candidates:
            if "ftdi" in desc.lower() or "ft232" in desc.lower():
                return dev, f"{desc} (обрано FTDI)"
        return candidates[0][0], f"{candidates[0][1]} (перший з {len(candidates)} знайдених)"

    # Windows fallback: if there is a single COM port that isn't COM1 (COM1 is usually a motherboard header)
    win_coms = [p for p in ports if not p.device.startswith("/dev/") and p.device.upper() != "COM1"]
    if len(win_coms) == 1:
        return win_coms[0].device, win_coms[0].description

    # Single non-debug port fallback
    non_dbg = [p for p in ports if not any(ign in p.device.lower() for ign in ("bluetooth", "debug-console", "wlan-debug"))]
    if len(non_dbg) == 1:
        return non_dbg[0].device, non_dbg[0].description

    return None, f"доступні порти: {', '.join(p.device for p in ports)}"


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
    alive = [c for c in cells if c > 0] or cells or [0]
    status = r(25, 0)
    low = status & 0x00FF
    state = next((name for v, name in STATUS_STATES if low == v),
                 "Standby" if low == 0 else f"Unknown({low:#04x})")
    info = decode_dev_info_block(regs[105:128]) if n >= 128 else {}
    return {
        "voltage": regs[0] / 100.0 if n > 0 else 0.0,
        "current": s16(regs[1]) / 100.0 if n > 1 else 0.0,
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
    """Extract Modbus FC03 frames from stream chunk.
    Returns (frames, rest); a frame is (kind, addr, payload):
    kind 'req' (start, qty) or 'resp' (regs list)."""
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
            break                            # incomplete
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


def get_bank_summary(packs: dict) -> dict:
    """Calculates bank-wide aggregated telemetry across all online packs."""
    online_packs = [p for p in packs.values() if p.get("online") and p.get("parsed")]
    if not online_packs:
        return {
            "total_packs": len(packs),
            "online_packs": 0,
            "avg_voltage": 0.0,
            "total_current": 0.0,
            "total_power_kw": 0.0,
            "avg_soc": 0,
            "total_cap_remaining": 0.0,
            "min_cell_mv": None,
            "min_cell_pack": None,
            "min_cell_idx": None,
            "max_cell_mv": None,
            "max_cell_pack": None,
            "max_cell_idx": None,
            "cell_delta_mv": 0,
            "any_balancing": False,
            "alarm_count": 0,
        }

    voltages = [p["parsed"]["voltage"] for p in online_packs if p["parsed"].get("voltage") is not None]
    avg_volt = sum(voltages) / len(voltages) if voltages else 0.0

    currents = [p["parsed"]["current"] for p in online_packs if p["parsed"].get("current") is not None]
    total_curr = sum(currents)

    socs = [p["parsed"]["soc"] for p in online_packs if p["parsed"].get("soc") is not None]
    avg_soc = round(sum(socs) / len(socs)) if socs else 0

    cap_rem = sum(p["parsed"]["cap_remaining"] for p in online_packs if p["parsed"].get("cap_remaining") is not None)

    all_cells = []
    any_bal = False
    alarm_cnt = 0

    for p in online_packs:
        parsed = p["parsed"]
        p_addr = p.get("addr_hex", f"0x{p['addr']:02X}")
        for idx, c in enumerate(parsed.get("cells", [])):
            if c > 0:
                all_cells.append((c, p_addr, idx + 1))
        if parsed.get("balance_bits") and any(b["on"] for b in parsed["balance_bits"]):
            any_bal = True
        if parsed.get("warning", 0) != 0 or parsed.get("protection", 0) != 0 or parsed.get("error_code", 0) != 0:
            alarm_cnt += 1

    if all_cells:
        min_c = min(all_cells, key=lambda x: x[0])
        max_c = max(all_cells, key=lambda x: x[0])
        min_cell_mv, min_cell_pack, min_cell_idx = min_c
        max_cell_mv, max_cell_pack, max_cell_idx = max_c
        cell_delta_mv = max_cell_mv - min_cell_mv
    else:
        min_cell_mv = min_cell_pack = min_cell_idx = None
        max_cell_mv = max_cell_pack = max_cell_idx = None
        cell_delta_mv = 0

    return {
        "total_packs": len(packs),
        "online_packs": len(online_packs),
        "avg_voltage": round(avg_volt, 2),
        "total_current": round(total_curr, 2),
        "total_power_kw": round((avg_volt * total_curr) / 1000.0, 3),
        "avg_soc": avg_soc,
        "total_cap_remaining": round(cap_rem, 1),
        "min_cell_mv": min_cell_mv,
        "min_cell_pack": min_cell_pack,
        "min_cell_idx": min_cell_idx,
        "max_cell_mv": max_cell_mv,
        "max_cell_pack": max_cell_pack,
        "max_cell_idx": max_cell_idx,
        "cell_delta_mv": cell_delta_mv,
        "any_balancing": any_bal,
        "alarm_count": alarm_cnt,
    }


class BmsBus:
    PASSIVE_STALE_S = 15.0   # If pack quiet >15s in passive mode, mark offline
    PASSIVE_GAP_S = 0.15

    def __init__(self, port: str, baud: int, initial_addr: int = DEFAULT_ADDR,
                 mock: bool = False, passive: bool = False):
        self.port = port
        self.baud = baud
        self.focus_addr = initial_addr
        self.mock = mock
        self.mode = "passive" if passive else "active"
        self.ser = None
        self.lock = threading.Lock()

        # Multi-pack storage: dict of addr_int -> pack dict
        self.packs = {}
        self.active_addrs = []
        self.is_scanning = False
        self.scan_msg = ""
        self.last_err = None

        self._rescan_requested = True   # Trigger full 1..16 discovery on start
        self._bg_probe_idx = 0
        self._stop = False
        self._mock_t = 0.0

        # CSV logging
        self.log_on = False
        self.log_path = None
        self.log_rows = 0

    def open(self) -> bool:
        if self.mock:
            return True
        if serial is None:
            self.last_err = "pySerial not installed (pip3 install pyserial)"
            return False
        try:
            to = self.PASSIVE_GAP_S if self.mode == "passive" else RESP_TIMEOUT_S
            self.ser = serial.Serial(self.port, self.baud, timeout=to)
            return True
        except Exception as e:
            self.last_err = f"open {self.port}: {e}"
            return False

    def set_mode(self, new_mode: str) -> None:
        """Switch between 'active' and 'passive' on the fly."""
        with self.lock:
            if new_mode not in ("active", "passive"):
                return
            if self.mode == new_mode:
                return
            self.mode = new_mode
            if not self.mock and self.ser and self.ser.is_open:
                try:
                    self.ser.timeout = self.PASSIVE_GAP_S if self.mode == "passive" else RESP_TIMEOUT_S
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
            if self.mode == "active":
                self._rescan_requested = True

    def request_rescan(self) -> None:
        """Trigger an immediate 1..16 discovery sweep."""
        with self.lock:
            self._rescan_requested = True

    def read_regs(self, addr: int, timeout: float = RESP_TIMEOUT_S) -> list:
        req = bytes([addr, 0x03, 0x00, 0x00, 0x00, 0x27])
        req += crc_bytes(req)
        self.ser.timeout = timeout
        self.ser.reset_input_buffer()
        self.ser.write(req)
        hdr = self.ser.read(3)
        if len(hdr) < 3 or hdr[0] != addr or hdr[1] != 0x03:
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

    def read_dev_info(self, addr: int, timeout: float = 0.2) -> dict:
        try:
            req = bytes([addr, 0x03, 0x00, 0x69, 0x00, 0x17])
            req += crc_bytes(req)
            self.ser.timeout = timeout
            self.ser.reset_input_buffer()
            self.ser.write(req)
            hdr = self.ser.read(3)
            if len(hdr) < 3 or hdr[0] != addr or hdr[1] != 0x03:
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

    def mock_regs(self, addr: int) -> list:
        self._mock_t += 1
        rnd = random.Random((self._mock_t // 6) + addr * 17)
        # Distinct characteristics for pack 1 vs pack 2
        base_v = 3465 if addr == 1 else 3460
        delta = rnd.randint(-4, 4)
        base = max(3300, min(3600, base_v + delta))

        cells = [base + rnd.randint(-6, 6) for _ in range(15)]
        if addr == 2:
            # Let cell 15 be higher and balancing in pack 2
            cells[14] = base + 28
            bal_reg = 0x4000  # Cell 15 balancing active
        else:
            bal_reg = 0

        cells += [0]
        voltage = sum(cells[:15]) // 10
        current = 1500 if addr == 1 else 1450  # ~15A charge
        regs = [0] * 39
        regs[0] = voltage
        regs[1] = current
        for i, c in enumerate(cells):
            regs[2 + i] = c
        regs[18], regs[19], regs[20] = 23, 22, 23
        regs[21] = 85 if addr == 1 else 84  # Cap remaining
        regs[22] = 100
        regs[23] = 100
        regs[24] = 85 if addr == 1 else 84  # SOC %
        regs[25] = 1 if current > 0 else 0
        regs[26] = 0
        regs[27] = 0
        regs[28] = 0
        regs[29], regs[30] = 0, 1 + addr
        regs[31], regs[32] = 0x1575, 0x2A00
        regs[33], regs[34], regs[35] = 0x1616, 0x1616, 0
        regs[36] = 15
        regs[37] = 1000
        regs[38] = bal_reg
        return regs

    def log_row(self, regs: list, addr: int) -> None:
        if not self.log_on or not self.log_path:
            return
        new_file = not os.path.exists(self.log_path)
        try:
            with open(self.log_path, "a", newline="") as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow(["ts", "addr"] + [f"r{i:02d}" for i in range(len(regs))])
                w.writerow([datetime.now().isoformat(timespec="seconds"),
                            f"0x{addr:02X}"] + regs)
                self.log_rows += 1
        except Exception:
            pass

    def loop(self) -> None:
        buf = b""
        while not self._stop:
            if self.mode == "active":
                self.active_step()
            else:
                buf = self.passive_step(buf)

    def active_step(self) -> None:
        if not self.mock and (self.ser is None or not self.ser.is_open):
            if not self.open():
                time.sleep(2.5)
                return

        # 1. Full auto-discovery sweep if requested or no active packs known
        if self._rescan_requested or not self.active_addrs:
            self._rescan_requested = False
            self.is_scanning = True
            found = []

            for a in range(1, 17):
                if self._stop or self.mode != "active":
                    break
                self.scan_msg = f"Сканування 0x{a:02X} (1..16)..."
                try:
                    if self.mock:
                        if a in (1, 2):
                            regs = self.mock_regs(a)
                        else:
                            raise IOError("not present")
                    else:
                        regs = self.read_regs(a, timeout=PROBE_TIMEOUT_S)

                    found.append(a)
                    dev_info = {}
                    if not self.mock:
                        dev_info = self.read_dev_info(a)
                    else:
                        dev_info = {
                            "model": "V-LFP48100-15S",
                            "fw_version": "V10R18",
                            "serial": f"SN-2026102200{a}",
                        }

                    with self.lock:
                        self.packs[a] = {
                            "addr": a,
                            "addr_hex": f"0x{a:02X}",
                            "online": True,
                            "last_seen": time.time(),
                            "consecutive_fails": 0,
                            "raw": regs,
                            "parsed": decode_regs(regs),
                            "dev_info": dev_info,
                            "err": None,
                        }
                    self.log_row(regs, a)
                except Exception:
                    with self.lock:
                        if a in self.packs:
                            self.packs[a]["online"] = False

            with self.lock:
                self.active_addrs = sorted(found)
                self.is_scanning = False
                self.scan_msg = f"Знайдено АКБ: {len(self.active_addrs)}"
                self.last_err = None if self.active_addrs else "Жодного АКБ не знайдено на адресах 0x01..0x10"
            return

        # 2. Fast cyclic poll of active packs
        current_active = list(self.active_addrs)
        for a in current_active:
            if self._stop or self.mode != "active" or self._rescan_requested:
                break
            try:
                regs = self.mock_regs(a) if self.mock else self.read_regs(a, timeout=RESP_TIMEOUT_S)
                with self.lock:
                    p = self.packs.get(a, {})
                    dev_info = p.get("dev_info")
                    if not dev_info and not self.mock:
                        dev_info = self.read_dev_info(a)
                    elif not dev_info and self.mock:
                        dev_info = {"model": "V-LFP48100-15S", "fw_version": "V10R18", "serial": f"SN-2026102200{a}"}

                    self.packs[a] = {
                        "addr": a,
                        "addr_hex": f"0x{a:02X}",
                        "online": True,
                        "last_seen": time.time(),
                        "consecutive_fails": 0,
                        "raw": regs,
                        "parsed": decode_regs(regs),
                        "dev_info": dev_info,
                        "err": None,
                    }
                self.log_row(regs, a)
            except Exception as e:
                with self.lock:
                    if a in self.packs:
                        self.packs[a]["consecutive_fails"] = self.packs[a].get("consecutive_fails", 0) + 1
                        self.packs[a]["err"] = str(e)
                        # Hot-unplug: 3 consecutive timeouts marks pack offline
                        if self.packs[a]["consecutive_fails"] >= 3:
                            self.packs[a]["online"] = False
                            if a in self.active_addrs:
                                self.active_addrs.remove(a)
            time.sleep(POLL_S)

        # 3. Background hot-plug discovery of 1 inactive address per cycle
        inactive = [a for a in range(1, 17) if a not in self.active_addrs]
        if inactive and not self._stop and self.mode == "active" and not self._rescan_requested:
            probe_a = inactive[self._bg_probe_idx % len(inactive)]
            self._bg_probe_idx += 1
            try:
                regs = self.mock_regs(probe_a) if (self.mock and probe_a in (1, 2)) else (
                    self.read_regs(probe_a, timeout=PROBE_TIMEOUT_S) if not self.mock else None
                )
                if regs:
                    # New battery plugged in!
                    dev_info = self.read_dev_info(probe_a) if not self.mock else {
                        "model": "V-LFP48100-15S", "fw_version": "V10R18", "serial": f"SN-2026102200{probe_a}"}
                    with self.lock:
                        self.packs[probe_a] = {
                            "addr": probe_a,
                            "addr_hex": f"0x{probe_a:02X}",
                            "online": True,
                            "last_seen": time.time(),
                            "consecutive_fails": 0,
                            "raw": regs,
                            "parsed": decode_regs(regs),
                            "dev_info": dev_info,
                            "err": None,
                        }
                        if probe_a not in self.active_addrs:
                            self.active_addrs.append(probe_a)
                            self.active_addrs.sort()
                    self.log_row(regs, probe_a)
            except Exception:
                pass

    def passive_step(self, buf: bytes) -> bytes:
        if not self.mock and (self.ser is None or not self.ser.is_open):
            if not self.open():
                time.sleep(2.5)
                return b""

        if self.mock:
            time.sleep(0.8)
            now = time.time()
            for a in (1, 2):
                regs = self.mock_regs(a)
                with self.lock:
                    self.packs[a] = {
                        "addr": a,
                        "addr_hex": f"0x{a:02X}",
                        "online": True,
                        "last_seen": now,
                        "consecutive_fails": 0,
                        "raw": regs,
                        "parsed": decode_regs(regs),
                        "dev_info": {"model": "V-LFP48100-15S", "fw_version": "V10R18", "serial": f"SN-2026102200{a}"},
                        "err": None,
                    }
                self.log_row(regs, a)
            with self.lock:
                self.active_addrs = [1, 2]
            return b""

        try:
            chunk = self.ser.read(512)
            if chunk:
                buf += chunk
                frames, buf = parse_bus(buf)
            else:
                buf = b""
                frames = []

            now = time.time()
            for kind, addr, payload in frames:
                if kind == "resp" and len(payload) >= 28 and (1 <= addr <= 16):
                    with self.lock:
                        p = self.packs.get(addr, {})
                        self.packs[addr] = {
                            "addr": addr,
                            "addr_hex": f"0x{addr:02X}",
                            "online": True,
                            "last_seen": now,
                            "consecutive_fails": 0,
                            "raw": payload,
                            "parsed": decode_regs(payload),
                            "dev_info": p.get("dev_info") or {},
                            "err": None,
                        }
                        if addr not in self.active_addrs:
                            self.active_addrs.append(addr)
                            self.active_addrs.sort()
                    self.log_row(payload, addr)

            # Check staleness: if no packet from pack > 15s, mark offline
            with self.lock:
                for a, p in self.packs.items():
                    if p.get("online") and (now - p.get("last_seen", 0) > self.PASSIVE_STALE_S):
                        p["online"] = False
                        if a in self.active_addrs:
                            self.active_addrs.remove(a)
        except Exception as e:
            self.last_err = str(e)
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            time.sleep(1.0)
        return buf

    def snapshot(self) -> dict:
        with self.lock:
            # Serialize packs
            packs_dict = {}
            for a, p in self.packs.items():
                parsed = p.get("parsed")
                if parsed and p.get("dev_info"):
                    merged = dict(parsed)
                    for k, v in p["dev_info"].items():
                        if not merged.get(k):
                            merged[k] = v
                    p_copy = dict(p, parsed=merged)
                else:
                    p_copy = dict(p)
                packs_dict[f"0x{a:02X}"] = p_copy

            summary = get_bank_summary(self.packs)
            return {
                "mode": self.mode,
                "is_scanning": self.is_scanning,
                "scan_msg": self.scan_msg,
                "active_addrs": [f"0x{a:02X}" for a in sorted(self.active_addrs)],
                "summary": summary,
                "packs": packs_dict,
                "focus_addr": f"0x{self.focus_addr:02X}",
                "port": self.port,
                "baud": self.baud,
                "log_on": self.log_on,
                "log_path": self.log_path,
                "log_rows": self.log_rows,
                "last_err": self.last_err,
                "ts": datetime.now().isoformat(timespec="seconds"),
            }


HTML = """<!doctype html>
<html lang="uk"><head><meta charset="utf-8">
<title>Vision BMS Multi-Pack Monitor</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#111417;--card:#1a1f24;--card-hi:#232a31;--fg:#d8dee4;--dim:#7d8894;--ok:#3fb950;
--bad:#f85149;--warn:#d29922;--acc:#58a6ff;--border:#30363d}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 -apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif;padding:16px}
header{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:12px;margin-bottom:12px}
h1{font-size:20px;margin:0;display:flex;align-items:center;gap:10px}
.sub{color:var(--dim);font-size:13px;margin-bottom:16px}
.controls{display:flex;align-items:center;gap:8px;flex-wrap:wrap}

/* Buttons & Pills */
.pill{display:inline-block;padding:3px 10px;border-radius:20px;font-weight:600;font-size:12px;background:#30363d}
.pill.ok{background:#1a3d24;color:var(--ok)}
.pill.bad{background:#4a1a18;color:var(--bad)}
.pill.warn{background:#423213;color:var(--warn)}
button{background:var(--acc);border:0;color:#0b0f14;padding:7px 14px;border-radius:7px;font-weight:600;cursor:pointer;font-size:13px;transition:opacity .15s}
button:hover{opacity:.9}
button.off{background:#30363d;color:var(--dim)}
.btn-mode{background:#238636;color:#fff;border:1px solid #2ea043}
.btn-mode.passive{background:#1f6feb;border-color:#388bfd}
.btn-rescan{background:#21262d;border:1px solid var(--border);color:var(--fg)}
.btn-rescan:hover{background:#30363d}

/* Tab navigation */
.tabs{display:flex;gap:6px;margin-bottom:16px;border-bottom:1px solid var(--border);padding-bottom:8px;flex-wrap:wrap}
.tab-btn{background:#161b22;border:1px solid var(--border);color:var(--dim);padding:8px 14px;border-radius:8px;font-weight:600;cursor:pointer;display:flex;align-items:center;gap:6px}
.tab-btn:hover{background:#21262d;color:var(--fg)}
.tab-btn.active{background:var(--acc);color:#0b0f14;border-color:var(--acc)}
.tab-badge{background:rgba(0,0,0,0.25);padding:1px 6px;border-radius:8px;font-size:11px}
.b-tag{background:var(--warn);color:#111417;padding:1px 5px;border-radius:3px;font-size:10px;font-weight:700}
.al-dot{width:8px;height:8px;border-radius:50%;background:var(--bad);display:inline-block}

/* Summary Grid */
.bank-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px;margin-bottom:16px}
.bank-card{background:var(--card);border-radius:10px;padding:14px;border:1px solid #252c34}
.bank-card .l{font-size:11px;color:var(--dim);text-transform:uppercase;letter-spacing:.05em;margin-bottom:4px}
.bank-card .v{font-size:24px;font-weight:700;font-variant-numeric:tabular-nums}
.bank-card .subtext{font-size:12px;color:var(--dim);margin-top:4px}

/* Grid & Cards */
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(310px,1fr))}
.card{background:var(--card);border-radius:10px;padding:14px;border:1px solid #252c34}
.card h2{font-size:13px;text-transform:uppercase;letter-spacing:.06em;color:var(--dim);margin:0 0 10px;display:flex;justify-content:space-between}
.big{display:flex;gap:20px;flex-wrap:wrap}
.big div span{display:block}
.big .v{font-size:26px;font-weight:600;font-variant-numeric:tabular-nums}
.big .l{font-size:11px;color:var(--dim);text-transform:uppercase}

/* Cell display */
.cells{display:grid;grid-template-columns:repeat(auto-fill,minmax(84px,1fr));gap:6px}
.cell{background:#232a31;border-radius:6px;padding:6px;text-align:center;border:1px solid transparent}
.cell .n{font-size:10px;color:var(--dim)}
.cell .mv{font-weight:600;font-variant-numeric:tabular-nums;font-size:14px}
.cell.min{border-color:var(--bad);background:#2f1d1f}
.cell.max{border-color:var(--ok);background:#1a2e22}
.cell.bal-active{box-shadow:0 0 8px var(--warn);border-color:var(--warn);background:#2d2415}

/* Table */
table.tbl{width:100%;border-collapse:collapse;font-size:13px}
table.tbl th{padding:8px 10px;border-bottom:1px solid var(--border);color:var(--dim);text-align:left;font-size:11px;text-transform:uppercase}
table.tbl td{padding:9px 10px;border-bottom:1px solid #21262d;font-variant-numeric:tabular-nums}
table.tbl tr:hover{background:#1f262e}
table.kv td{padding:3px 10px 3px 0;color:var(--dim)}
table.kv td:last-child{color:var(--fg);font-variant-numeric:tabular-nums;text-align:right}

/* LEDs */
.leds{display:grid;grid-template-columns:1fr 1fr;gap:4px 12px}
.led{display:flex;align-items:center;gap:8px;color:var(--dim);font-size:13px}
.led i{width:11px;height:11px;border-radius:50%;background:#30363d;display:inline-block;flex:none}
.led.on{color:var(--fg)}
.led.on i{background:var(--bad);box-shadow:0 0 6px var(--bad)}
.led.onw i{background:var(--warn);box-shadow:0 0 6px var(--warn)}
.led.onok i{background:var(--ok);box-shadow:0 0 6px var(--ok)}
.bal{display:flex;gap:4px;flex-wrap:wrap}
.bal span{width:28px;height:28px;border-radius:6px;background:#232a31;display:grid;place-items:center;font-size:11px;color:var(--dim)}
.bal span.on{background:#3b2d12;color:var(--warn);font-weight:700;border:1px solid var(--warn)}

.err-msg{color:var(--bad);font-weight:600;margin-top:6px}
.foot{color:var(--dim);font-size:12px;margin-top:20px;border-top:1px solid var(--border);padding-top:10px}
</style></head><body>

<header>
  <h1>Vision BMS Multi-Pack Monitor <span id="conn" class="pill">…</span></h1>
  <div class="controls">
    <button id="modebtn" class="btn-mode" onclick="toggleMode()">⚡ Режим: Активний</button>
    <button class="btn-rescan" onclick="triggerRescan()">🔄 Пересканувати всі 16</button>
  </div>
</header>

<div class="sub">
  RS485 · <span id="portinfo">—</span> · <span id="modedesc">—</span> · <span id="seeninfo">—</span> · Оновлено: <span id="ts">—</span>
</div>

<div class="tabs" id="tabbar">
  <!-- Dynamic tabs populated by JS -->
</div>

<!-- VIEW 1: SUMMARY / GENERAL OVERVIEW -->
<div id="view_summary">
  <div class="bank-grid">
    <div class="bank-card">
      <div class="l">Середня напруга банку</div>
      <div class="v" id="b_v">—</div>
      <div class="subtext">Сер. значення по підключеним АКБ</div>
    </div>
    <div class="bank-card">
      <div class="l">Сумарний струм та потужність</div>
      <div class="v" id="b_i">—</div>
      <div class="subtext" id="b_p">— кВт</div>
    </div>
    <div class="bank-card">
      <div class="l">Середній SOC банку</div>
      <div class="v" id="b_soc">—</div>
      <div class="subtext" id="b_cap">Залишок: — А·ч</div>
    </div>
    <div class="bank-card">
      <div class="l">Розбаланс у всьому банку</div>
      <div class="v" id="b_diff">—</div>
      <div class="subtext" id="b_extremes">Min: — | Max: —</div>
    </div>
    <div class="bank-card">
      <div class="l">Статус системи</div>
      <div class="v" id="b_status" style="font-size:18px">—</div>
      <div class="subtext" id="b_alarm_status">Тривоги: —</div>
    </div>
  </div>

  <div class="card" style="margin-bottom:16px">
    <h2>Зведена таблиця всіх виявлених акумуляторів</h2>
    <div style="overflow-x:auto">
      <table class="tbl">
        <thead>
          <tr>
            <th>АКБ</th>
            <th>Стан</th>
            <th>Напруга</th>
            <th>Струм</th>
            <th>SOC</th>
            <th>Min ком.</th>
            <th>Max ком.</th>
            <th>Δ банку</th>
            <th>Баланс</th>
            <th>Темп. (PCB/Avg/Max)</th>
            <th>Тривоги</th>
            <th>Дія</th>
          </tr>
        </thead>
        <tbody id="summary_rows">
          <tr><td colspan="12" style="text-align:center;color:var(--dim);padding:20px">Очікування сканування шини...</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</div>

<!-- VIEW 2: PACK DETAIL -->
<div id="view_pack" style="display:none">
  <div class="grid">
    <div class="card">
      <h2>Параметри живлення</h2>
      <div class="big">
        <div><span class="l">Напруга</span><span class="v" id="p_v">—</span></div>
        <div><span class="l">Струм</span><span class="v" id="p_i">—</span></div>
        <div><span class="l">Стан</span><span class="v"><span class="pill" id="p_st">—</span></span></div>
      </div>
      <div class="err-msg" id="p_err"></div>
      <table class="kv" style="margin-top:12px">
        <tr><td>Адреса RS485</td><td id="p_addr">—</td></tr>
        <tr><td>Модель АКБ</td><td id="p_model">—</td></tr>
        <tr><td>Прошивка BMS</td><td id="p_fw">—</td></tr>
        <tr><td>Серійний номер</td><td id="p_sn">—</td></tr>
        <tr><td>SOC / Cap Remaining</td><td id="p_soc">—</td></tr>
        <tr><td>SOH / Max charge</td><td id="p_soh">—</td></tr>
        <tr><td>Кількість циклів</td><td id="p_cyc">—</td></tr>
        <tr><td>Номінал</td><td id="p_cap">—</td></tr>
        <tr><td>Full Capacity (0031)</td><td id="p_fcap">—</td></tr>
      </table>
    </div>

    <div class="card">
      <h2>Комірки 15S <span id="p_difft" style="color:var(--dim);font-weight:400"></span></h2>
      <div class="cells" id="p_cells"></div>
    </div>

    <div class="card">
      <h2>Температури</h2>
      <table class="kv">
        <tr><td>PCB</td><td id="p_t0">—</td></tr>
        <tr><td>Avg (сер.)</td><td id="p_t1">—</td></tr>
        <tr><td>Max</td><td id="p_t2">—</td></tr>
        <tr><td>Temp01 (сенсор 1)</td><td id="p_t3">—</td></tr>
        <tr><td>Temp02 (сенсор 2)</td><td id="p_t4">—</td></tr>
        <tr><td>Temp03 (сенсор 3)</td><td id="p_t5">—</td></tr>
        <tr><td>Temp04 (сенсор 4)</td><td id="p_t6">—</td></tr>
      </table>
    </div>

    <div class="card">
      <h2>Status (0025) <span id="p_straw" style="color:var(--dim);font-weight:400"></span></h2>
      <div class="leds" id="p_stbits"></div>
    </div>

    <div class="card">
      <h2>Warning (0026) <span id="p_warnraw" style="color:var(--dim);font-weight:400"></span></h2>
      <div class="leds" id="p_warn"></div>
    </div>

    <div class="card">
      <h2>Protection (0027) <span id="p_protraw" style="color:var(--dim);font-weight:400"></span></h2>
      <div class="leds" id="p_prot"></div>
    </div>

    <div class="card">
      <h2>Error (0028) <span id="p_errraw" style="color:var(--dim);font-weight:400"></span></h2>
      <div class="leds" id="p_errbits"></div>
    </div>

    <div class="card">
      <h2>Активне балансування (0038)</h2>
      <div class="bal" id="p_bal"></div>
    </div>
  </div>
</div>

<!-- COMMON: CSV LOGGING CARD -->
<div class="card" style="margin-top:16px">
  <h2>Запис CSV-лога</h2>
  <div style="display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:12px">
    <div>Файл: <b id="logpath">—</b> · Записано рядків: <span id="logrows">0</span></div>
    <button id="logbtn" onclick="toggleLog()">Старт запису</button>
  </div>
</div>

<div class="foot">
  <b>Режими роботи:</b><br>
  • <b>⚡ Активний (Master):</b> скрипт самостійно сканує всі 16 адрес (0x01..0x10), формує пул живих АКБ і швидко опитує лише їх. Якщо АКБ додано чи відключено — система автоматично оновить пул. Використовувати коли шина вільна від COM-Box/інвертора.<br>
  • <b>👂 Пасивний (Sniffer):</b> скрипт не передає жодного байту у шину RS485 (нуль колізій). Слухає трафік між COM-Box / інвертором та батареями, на льоту декодує дані всіх бачених АКБ.<br>
  <b>Безпека:</b> 100% Read-Only (жодного запису в регістри конфігурації BMS).
</div>

<script>
let currentTab = 'summary';
let logOn = false;
let latestData = null;

function leds(el, arr, cls) {
  if (!el || !arr) return;
  el.innerHTML = arr.map(b => `<div class="led ${b.on ? cls : ''}"><i></i>${b.name}</div>`).join('');
}

function selectTab(tabId) {
  currentTab = tabId;
  renderTabs();
  renderView();
}

function renderTabs() {
  if (!latestData) return;
  const tb = document.getElementById('tabbar');
  const packs = latestData.packs || {};
  const addrs = Object.keys(packs).sort();

  let html = `<button class="tab-btn ${currentTab==='summary'?'active':''}" onclick="selectTab('summary')">
    📊 Загальний огляд банку <span class="tab-badge">${latestData.summary.online_packs} АКБ</span>
  </button>`;

  addrs.forEach(addr => {
    const p = packs[addr];
    const online = p.online;
    const parsed = p.parsed;
    let badge = '';
    let balTag = '';
    let warnTag = '';

    if (parsed) {
      if (parsed.soc != null) badge = `<span class="tab-badge">${parsed.soc}%</span>`;
      if (parsed.balance_bits && parsed.balance_bits.some(b => b.on)) {
        balTag = `<span class="b-tag">BAL</span>`;
      }
      if (parsed.warning || parsed.protection || parsed.error_code) {
        warnTag = `<span class="al-dot" title="Активна тривога"></span>`;
      }
    }

    const title = `${online ? '🔋' : '⚪'} АКБ ${addr}`;
    const cls = `tab-btn ${currentTab===addr?'active':''} ${online?'':'off'}`;
    html += `<button class="${cls}" onclick="selectTab('${addr}')">
      ${title} ${badge} ${balTag} ${warnTag}
    </button>`;
  });

  tb.innerHTML = html;
}

function renderView() {
  if (!latestData) return;
  const isSummary = (currentTab === 'summary');
  document.getElementById('view_summary').style.display = isSummary ? 'block' : 'none';
  document.getElementById('view_pack').style.display = isSummary ? 'none' : 'block';

  if (isSummary) {
    renderSummary();
  } else {
    renderPackDetail(currentTab);
  }
}

function renderSummary() {
  const sum = latestData.summary;
  const packs = latestData.packs || {};

  document.getElementById('b_v').textContent = sum.avg_voltage ? (sum.avg_voltage.toFixed(2) + ' В') : '—';
  
  const currSign = sum.total_current > 0 ? '+' : '';
  document.getElementById('b_i').textContent = (sum.total_current != null ? (currSign + sum.total_current.toFixed(1) + ' А') : '—');
  document.getElementById('b_p').textContent = (sum.total_power_kw != null ? (currSign + sum.total_power_kw.toFixed(2) + ' кВт') : '—');
  
  document.getElementById('b_soc').textContent = sum.avg_soc ? (sum.avg_soc + ' %') : '—';
  document.getElementById('b_cap').textContent = 'Залишок: ' + (sum.total_cap_remaining ? sum.total_cap_remaining.toFixed(1) + ' А·ч' : '—');

  document.getElementById('b_diff').textContent = (sum.cell_delta_mv != null ? ('Δ = ' + sum.cell_delta_mv + ' мВ') : '—');
  if (sum.min_cell_mv && sum.max_cell_mv) {
    document.getElementById('b_extremes').textContent = `Min: ${sum.min_cell_mv} мВ (${sum.min_cell_pack} #${sum.min_cell_idx}) | Max: ${sum.max_cell_mv} мВ (${sum.max_cell_pack} #${sum.max_cell_idx})`;
  } else {
    document.getElementById('b_extremes').textContent = 'Min: — | Max: —';
  }

  const balEl = document.getElementById('b_status');
  balEl.textContent = sum.any_balancing ? 'АКТИВНЕ БАЛАНСУВАННЯ' : (sum.online_packs > 0 ? 'Працює в нормі' : 'АКБ відсутні');
  balEl.style.color = sum.any_balancing ? 'var(--warn)' : (sum.online_packs > 0 ? 'var(--ok)' : 'var(--bad)');

  const alEl = document.getElementById('b_alarm_status');
  alEl.textContent = sum.alarm_count > 0 ? `⚠️ Активних тривог: ${sum.alarm_count}` : 'Усі захисти та попередження в нормі';
  alEl.style.color = sum.alarm_count > 0 ? 'var(--bad)' : 'var(--dim)';

  // Table rows
  const tbody = document.getElementById('summary_rows');
  const addrs = Object.keys(packs).sort();
  if (addrs.length === 0) {
    tbody.innerHTML = `<tr><td colspan="12" style="text-align:center;color:var(--dim);padding:20px">Жодного АКБ не знайдено. Натисніть «Пересканувати всі 16» або перевірте зв'язок.</td></tr>`;
    return;
  }

  tbody.innerHTML = addrs.map(addr => {
    const p = packs[addr];
    const on = p.online;
    const d = p.parsed;
    if (!d) {
      return `<tr>
        <td><b>${addr}</b></td>
        <td colspan="10" style="color:var(--bad)">Офлайн (${p.err || 'немає відповіді'})</td>
        <td><button onclick="selectTab('${addr}')">Відкрити</button></td>
      </tr>`;
    }

    const stClass = on ? (d.status_bits && d.status_bits[2] && d.status_bits[2].on ? 'bad' : 'ok') : 'bad';
    const balActive = d.balance_bits && d.balance_bits.some(b => b.on);
    const hasAlarm = (d.warning || d.protection || d.error_code);
    const currStr = (d.current >= 0 ? '+' : '') + d.current.toFixed(1) + ' А';

    return `<tr>
      <td><b>${addr}</b> ${on ? '<span style="color:var(--ok)">●</span>' : '<span style="color:var(--bad)">○</span>'}</td>
      <td><span class="pill ${stClass}">${d.status_name}</span></td>
      <td><b>${d.voltage.toFixed(2)} В</b></td>
      <td>${currStr}</td>
      <td>${d.soc != null ? d.soc + ' %' : '—'}</td>
      <td style="color:var(--bad)">${d.cell_min} мВ</td>
      <td style="color:var(--ok)">${d.cell_max} мВ</td>
      <td><b>${d.cell_diff} мВ</b></td>
      <td>${balActive ? '<span class="b-tag">BAL</span>' : '—'}</td>
      <td>${d.temp_pcb}/${d.temp_avg}/${d.temp_max} °C</td>
      <td>${hasAlarm ? '<span class="pill bad">Увага</span>' : '<span class="pill ok">Норма</span>'}</td>
      <td><button onclick="selectTab('${addr}')" style="padding:4px 10px;font-size:12px">Детальніше →</button></td>
    </tr>`;
  }).join('');
}

function renderPackDetail(addr) {
  const packs = latestData.packs || {};
  const p = packs[addr];
  if (!p) return;

  const d = p.parsed;
  document.getElementById('p_addr').textContent = addr + (p.online ? ' (Онлайн)' : ' (Офлайн)');
  document.getElementById('p_err').textContent = p.err || '';

  if (!d) {
    document.getElementById('p_v').textContent = '—';
    document.getElementById('p_i').textContent = '—';
    document.getElementById('p_st').textContent = 'Офлайн';
    document.getElementById('p_cells').innerHTML = '';
    return;
  }

  document.getElementById('p_v').textContent = d.voltage.toFixed(2) + ' В';
  document.getElementById('p_i').textContent = (d.current == null ? '—' : d.current.toFixed(1) + ' А');
  
  const st = document.getElementById('p_st');
  st.textContent = d.status_name;
  st.className = 'pill ' + (d.status_bits && d.status_bits[2] && d.status_bits[2].on ? 'bad' : 'ok');

  document.getElementById('p_model').textContent = d.model || '—';
  document.getElementById('p_fw').textContent = d.fw_version || '—';
  document.getElementById('p_sn').textContent = d.serial || '—';
  document.getElementById('p_soc').textContent = (d.soc == null ? '—' : d.soc + ' %') + ' / ' + (d.cap_remaining == null ? '—' : d.cap_remaining + ' А·ч');
  document.getElementById('p_soh').textContent = (d.soh == null ? '—' : d.soh + ' %') + ' / ' + (d.max_charge_a == null ? '—' : d.max_charge_a + ' А');
  document.getElementById('p_cyc').textContent = d.cycles == null ? '—' : d.cycles;
  document.getElementById('p_cap').textContent = (d.designed_cap_ah == null ? '—' : d.designed_cap_ah + ' А·ч') + ' · ' + d.cell_num + 'S';
  document.getElementById('p_fcap').textContent = d.full_cap_ah == null ? '—' : d.full_cap_ah.toFixed(1) + ' А·ч';
  document.getElementById('p_difft').textContent = 'Δ = ' + d.cell_diff + ' мВ';

  // Cells
  const ci = document.getElementById('p_cells');
  ci.innerHTML = d.cells.map((mv, i) => {
    let cls = 'cell';
    if (mv === d.cell_min) cls += ' min';
    if (mv === d.cell_max) cls += ' max';
    const balOn = d.balance_bits && d.balance_bits[i] && d.balance_bits[i].on;
    if (balOn) cls += ' bal-active';
    return `<div class="${cls}">
      <div class="n">#${i + 1}${balOn ? ' <span class="b-tag">BAL</span>' : ''}</div>
      <div class="mv">${mv}</div>
    </div>`;
  }).join('');

  // Temperatures
  document.getElementById('p_t0').textContent = (d.temp_pcb == null ? '—' : d.temp_pcb + ' °C');
  document.getElementById('p_t1').textContent = (d.temp_avg == null ? '—' : d.temp_avg + ' °C');
  document.getElementById('p_t2').textContent = (d.temp_max == null ? '—' : d.temp_max + ' °C');
  if (d.temp_sensors) {
    d.temp_sensors.slice(0, 4).forEach((tv, i) => {
      const el = document.getElementById('p_t' + (i + 3));
      if (el) el.textContent = tv + ' °C';
    });
  }

  const hx = v => '0x' + (v == null ? '????' : v.toString(16).toUpperCase().padStart(4, '0'));
  document.getElementById('p_straw').textContent = hx(d.status);
  document.getElementById('p_warnraw').textContent = hx(d.warning);
  document.getElementById('p_protraw').textContent = hx(d.protection);
  document.getElementById('p_errraw').textContent = hx(d.error_code);

  leds(document.getElementById('p_stbits'), d.status_bits, 'onok');
  leds(document.getElementById('p_warn'), d.warning_bits, 'on');
  leds(document.getElementById('p_prot'), d.protection_bits, 'on');
  leds(document.getElementById('p_errbits'), d.error_bits, 'onw');

  document.getElementById('p_bal').innerHTML = d.balance_bits ? d.balance_bits.map(b =>
    `<span class="${b.on ? 'on' : ''}">${b.n}</span>`).join('') : '—';
}

async function tick() {
  try {
    const r = await fetch('/api/data');
    latestData = await r.json();

    const d = latestData;
    const c = document.getElementById('conn');
    const onlineCnt = d.summary ? d.summary.online_packs : 0;

    if (d.is_scanning) {
      c.textContent = d.scan_msg || 'СКАНУВАННЯ...';
      c.className = 'pill warn';
    } else if (onlineCnt > 0) {
      c.textContent = `${onlineCnt} АКБ ОНЛАЙН`;
      c.className = 'pill ok';
    } else {
      c.textContent = 'НЕМАЄ ЗВ’ЯЗКУ';
      c.className = 'pill bad';
    }

    const isPassive = (d.mode === 'passive');
    const mbtn = document.getElementById('modebtn');
    mbtn.textContent = isPassive ? '👂 Режим: Пасивний (Sniffer)' : '⚡ Режим: Активний (Master)';
    mbtn.className = 'btn-mode' + (isPassive ? ' passive' : '');

    document.getElementById('portinfo').textContent = d.port + ' @ ' + d.baud;
    document.getElementById('modedesc').textContent = isPassive ? 'Пасивний (слухає шину без передачі)' : 'Активний (автопошук та опитування FC03)';
    document.getElementById('seeninfo').textContent = 'Онлайн: ' + (d.active_addrs && d.active_addrs.length ? d.active_addrs.join(', ') : 'жодного');
    document.getElementById('ts').textContent = d.ts || '—';

    logOn = d.log_on;
    document.getElementById('logpath').textContent = d.log_path || '—';
    document.getElementById('logrows').textContent = d.log_rows || 0;
    const lb = document.getElementById('logbtn');
    lb.textContent = logOn ? 'Зупинити запис' : 'Старт запису';
    lb.className = logOn ? 'off' : '';

    renderTabs();
    renderView();
  } catch (e) {
    const c = document.getElementById('conn');
    c.textContent = 'NO SERVER';
    c.className = 'pill bad';
  }
}

async function toggleMode() {
  if (!latestData) return;
  const targetMode = (latestData.mode === 'active') ? 'passive' : 'active';
  await fetch('/api/mode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode: targetMode })
  });
  tick();
}

async function triggerRescan() {
  await fetch('/api/rescan', { method: 'POST' });
  tick();
}

async function toggleLog() {
  await fetch('/api/log', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ on: !logOn })
  });
  tick();
}

tick();
setInterval(tick, 1000);
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
            self._send(200, json.dumps(self.bus.snapshot()), "application/json")
        else:
            self._send(404, "not found", "text/plain")

    def do_POST(self):
        if self.path == "/api/mode":
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except Exception:
                req = {}
            new_mode = req.get("mode")
            if new_mode in ("active", "passive"):
                self.bus.set_mode(new_mode)
            self._send(200, json.dumps({"ok": True, "mode": self.bus.mode}), "application/json")
        elif self.path == "/api/rescan":
            self.bus.request_rescan()
            self._send(200, json.dumps({"ok": True}), "application/json")
        elif self.path == "/api/log":
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
            self._send(200, json.dumps({"ok": True, "on": on}), "application/json")
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

    # --- Multi-pack bank summary test ---
    pack1_regs = list(regs)
    pack1_regs[0] = 5280   # 52.80 V
    pack1_regs[1] = 1500   # +15.00 A
    pack1_regs[24] = 85    # SOC 85%
    pack1_regs[2] = 3450   # min cell
    pack1_regs[16] = 3475  # max cell

    pack2_regs = list(regs)
    pack2_regs[0] = 5270   # 52.70 V
    pack2_regs[1] = 1400   # +14.00 A
    pack2_regs[24] = 84    # SOC 84%
    pack2_regs[2] = 3455   # min cell
    pack2_regs[16] = 3495  # max cell
    pack2_regs[38] = 0x4000  # Cell 15 balancing active

    packs_test = {
        1: {"addr": 1, "addr_hex": "0x01", "online": True, "parsed": decode_regs(pack1_regs)},
        2: {"addr": 2, "addr_hex": "0x02", "online": True, "parsed": decode_regs(pack2_regs)},
        3: {"addr": 3, "addr_hex": "0x03", "online": False, "parsed": None},
    }
    summary = get_bank_summary(packs_test)
    assert summary["total_packs"] == 3, f"total packs: {summary['total_packs']}"
    assert summary["online_packs"] == 2, f"online packs: {summary['online_packs']}"
    assert abs(summary["avg_voltage"] - 52.75) < 0.01, f"avg voltage: {summary['avg_voltage']}"
    assert abs(summary["total_current"] - 29.0) < 0.01, f"total current: {summary['total_current']}"
    assert summary["avg_soc"] == 85 or summary["avg_soc"] == 84, f"avg soc: {summary['avg_soc']}"
    assert summary["min_cell_mv"] == 3450 and summary["min_cell_pack"] == "0x01", f"min cell: {summary}"
    assert summary["max_cell_mv"] == 3495 and summary["max_cell_pack"] == "0x02", f"max cell: {summary}"
    assert summary["cell_delta_mv"] == (3495 - 3450), f"bank delta: {summary['cell_delta_mv']}"
    assert summary["any_balancing"] is True, "any balancing must be true"

    # --- Cross-platform port auto-detection tests ---
    from collections import namedtuple
    from unittest.mock import patch
    FakePort = namedtuple('FakePort', ['device', 'description', 'manufacturer', 'vid', 'pid', 'hwid'])

    # Test Windows FTDI COM port detection
    with patch('serial.tools.list_ports.comports', return_value=[FakePort('COM3', 'USB Serial Port (COM3)', 'FTDI', 0x0403, 0x6001, 'USB VID:PID=0403:6001')]):
        p_dev, p_desc = find_usb_rs485_port()
        assert p_dev == 'COM3' and 'FTDI' in p_desc, f"Windows FTDI auto-detect: {p_dev}, {p_desc}"

    # Test Windows CH340 COM port detection
    with patch('serial.tools.list_ports.comports', return_value=[FakePort('COM5', 'USB-SERIAL CH340 (COM5)', 'wch.cn', 0x1A86, 0x7523, 'USB VID:PID=1A86:7523')]):
        p_dev, p_desc = find_usb_rs485_port()
        assert p_dev == 'COM5' and 'CH340' in p_desc, f"Windows CH340 auto-detect: {p_dev}, {p_desc}"

    # Test macOS filtering virtual ports & detecting FTDI
    with patch('serial.tools.list_ports.comports', return_value=[
        FakePort('/dev/cu.debug-console', 'n/a', None, None, None, 'n/a'),
        FakePort('/dev/cu.Bluetooth-Incoming-Port', 'n/a', None, None, None, 'n/a'),
        FakePort('/dev/cu.usbserial-A5069RR4', 'FT232R USB UART', 'FTDI', 0x0403, 0x6001, 'USB VID:PID=0403:6001')
    ]):
        p_dev, p_desc = find_usb_rs485_port()
        assert p_dev == '/dev/cu.usbserial-A5069RR4', f"macOS FTDI auto-detect: {p_dev}"

    print("selftest OK")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None,
                    help="serial port (e.g. COM3 on Windows, /dev/cu.usbserial-... on macOS). If omitted, auto-detected.")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--addr", type=parse_hex_addr, default=DEFAULT_ADDR,
                    help="initial battery tab address (e.g. 0x01)")
    ap.add_argument("--http", type=int, default=DEFAULT_HTTP)
    ap.add_argument("--mock", action="store_true", help="no serial, mock multi-pack data")
    ap.add_argument("--passive", action="store_true",
                    help="listen only: parse live bus traffic (no TX, no collisions)")
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

    port = args.port
    if not args.mock and not port:
        auto_port, desc = find_usb_rs485_port()
        if auto_port:
            port = auto_port
            print(f"[Автовизначення] Знайдено адаптер RS485: {port} ({desc})")
        elif os.path.exists(DEFAULT_PORT):
            port = DEFAULT_PORT
            print(f"[Порт] Використовуємо дефолтний: {port}")
        else:
            print("Помилка: не вдалося автоматично визначити COM-порт адаптера RS485.")
            print(f"Статус пошуку: {desc}")
            print("\nБудь ласка, вкажіть порт явно або перевірте USB-підключення:")
            print("  Windows:  python tools/bms_web.py --port COM3")
            print("  macOS:    python3 tools/bms_web.py --port /dev/cu.usbserial-XXXX")
            print("  Список доступних портів: python3 tools/bms_web.py --list")
            return 1
    elif not port:
        port = "MOCK"

    bus = BmsBus(port, args.baud, initial_addr=args.addr, mock=args.mock, passive=args.passive)
    threading.Thread(target=bus.loop, daemon=True).start()
    Handler.bus = bus
    srv = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)

    mode_label = "MOCK MULTI-PACK" if args.mock else (
        f"{port} @ {args.baud} " + ("PASSIVE SNIFFER" if args.passive else "ACTIVE MASTER"))
    print(f"BMS Web Monitor: http://127.0.0.1:{args.http}  [{mode_label}]")
    print("Multi-pack scanning: 0x01..0x10 enabled. Press Ctrl-C to stop.")
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
