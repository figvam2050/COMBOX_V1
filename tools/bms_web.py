#!/usr/bin/env python3
"""Local multi-pack web monitor for Vision BMS (macOS analogue of BMS_TOOLS, v2.2: read-only).

Monitors multiple battery packs over the RS485 tap (Modbus RTU addresses 0x01..0x10)
and serves an interactive tabbed dashboard on http://127.0.0.1:8080.
Version-3
Features (v2.2):
  - Multi-pack auto-discovery (0x01..0x10) with Plug & Play hot-plug / unplug detection.
  - Active Polling (Master) vs Passive Scanning (Sniffer) switchable in 1 click.
  - Cross-platform auto-detection of USB-RS485 adapters (Windows COMx, macOS, Linux).
  - Autonomy & Energy Calculator (kWh stored, ETA charge time, ETA discharge runtime).
  - Cell Health & Weak Cell Tracker: detects chronically lagging cells and estimates
    internal resistance (R_int in mО©) and balancing duration per cell.
  - Live HTML5 Canvas Charts:
      * Rolling time chart (voltage, current, power, bank delta).
      * 15S Cell bar chart with min/max, active balancing glow, and R_int per cell.
  - Diagnostic Passport Report (PDF / Print export) with complete battery metrics.
  - CSV logging (v2.3): one file per pack, readable header row (register names),
    registers 31..35 decoded (Full Capacity, temp sensors), cell voltages in volts,
    full date/time column. Offline analyzer reads both new and legacy logs.
  - Dual logging profiles (v2.4), selectable independently in the log card:
      * Alarm - logs only alarm events from registers 0025..0028
        (status / warning / protection / error) and temperature thresholds.
        Nothing is written while the register state is unchanged; a row is
        written on activation, on any change and on recovery.
      * Work  - periodic cell-analysis / test samples, interval set in the
        log card (default 5 s).
    Both profiles may run at once, sharing one CSV file per pack. Writes are
    serialized and Alarm rows always take priority over the periodic Work row
    (the Work sample is postponed, never duplicated). Column "Log" marks every
    row as Alarm or Work.
  - Zero external dependencies beyond standard Python and pySerial.

Usage:
  python3 tools/bms_web.py           # Запуск за замовчуванням у пасивному режимі (Sniffer, безпечно для шини)
  python3 tools/bms_web.py --active  # Примусовий активний режим (Master, опитування батареї)
  python3 tools/bms_web.py --mock    # Демонстраційний режим
  python3 tools/bms_web.py --port COM3
  python3 tools/bms_web.py --selftest
"""

import argparse
import collections
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
    (0x00, "Standby"),
    (0x01, "Charging"),
    (0x02, "Discharging"),
    (0x04, "Protect"),
    (0x08, "Charging Lmt"),
]

# Базовые биты (с 0 по 12) одинаковы для Warn и Protect
BASE_BITS = [
    (0x0001, "Pack OV"), (0x0002, "Cell OV"), (0x0004, "Pack UV"),
    (0x0008, "Cell UV"), (0x0010, "Charge OC"), (0x0020, "Discharge OC"),
    (0x0040, "Temp anomaly"), (0x0080, "MOS OT"), (0x0100, "Charge OT"),
    (0x0200, "Discharge OT"), (0x0400, "Charge UT"), (0x0800, "Discharge UT"),
]

# В регистре 0026 (Warn)
WARN_BITS = BASE_BITS + [
     (0x1000, "Low Capacity"), (0x2000, "Float stop"),
]

# В регистре 0027 (Protect) 
PROT_BITS = BASE_BITS + [
    (0x1000, "Low Capacity"), (0x2000, "Short Circuit"), (0x4000, "Reverse")
]

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
        for dev, desc in candidates:
            if "ftdi" in desc.lower() or "ft232" in desc.lower():
                return dev, f"{desc} (обрано FTDI)"
        return candidates[0][0], f"{candidates[0][1]} (перший з {len(candidates)} знайдених)"

    win_coms = [p for p in ports if not p.device.startswith("/dev/") and p.device.upper() != "COM1"]
    if len(win_coms) == 1:
        return win_coms[0].device, win_coms[0].description

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
    model = decode_ascii_regs(regs[0:12]) if len(regs) >= 12 else None
    fw = decode_ascii_regs(regs[12:15]) if len(regs) >= 15 else None
    if not fw and len(regs) >= 13:
        fw = decode_fw_version(regs[12])
    serial_no = decode_ascii_regs(regs[15:23]) if len(regs) >= 23 else None
    return {"model": model, "fw_version": fw, "serial": serial_no}


# 15S LiFePO4 OCV-таблиця калібрована під Vision V-LFP48100 (ізоморфно до src/main.cpp)
# ===== 3-режимний адаптивний розрахунок SOC (Заряд / Розряд / Спокій) =====
LFP_CHARGE_TABLE = [
    (3550, 100.0),
    (3480, 95.0),
    (3420, 88.0),
    (3385, 78.0),
    (3365, 65.0),
    (3345, 50.0),
    (3320, 35.0),
    (3280, 20.0),
    (3150, 10.0),
    (2900, 0.0),
]

LFP_DISCHARGE_TABLE = [
    (3360, 100.0),
    (3300, 90.0),
    (3280, 80.0),
    (3250, 60.0),
    (3220, 40.0),
    (3180, 25.0),
    (3100, 15.0),
    (3000, 5.0),
    (2850, 0.0),
]

LFP_REST_TABLE = [
    (3450, 100.0),
    (3380, 99.0),
    (3350, 90.0),
    (3330, 80.0),
    (3300, 65.0),
    (3280, 50.0),
    (3250, 35.0),
    (3220, 20.0),
    (3150, 10.0),
    (2900, 0.0),
]


def interpolate_table(table: list, mv: int) -> float:
    if mv >= table[0][0]:
        return table[0][1]
    if mv <= table[-1][0]:
        return table[-1][1]
    for i in range(1, len(table)):
        if mv >= table[i][0]:
            mv_hi, soc_hi = table[i - 1]
            mv_lo, soc_lo = table[i]
            frac = (mv - mv_lo) / (mv_hi - mv_lo)
            return round(soc_lo + frac * (soc_hi - soc_lo), 1)
    return 0.0


def calc_adaptive_soc(avg_cell_mv: int, min_cell_mv: int, max_cell_mv: int, current_a: float, pack_v: float) -> float:
    # У Modbus (Vision): current > 0.5 A — заряд, current < -0.5 A — розряд
    if current_a > 0.5:
        soc = interpolate_table(LFP_CHARGE_TABLE, avg_cell_mv)
        if pack_v >= 53.5 and max_cell_mv >= 3550 and current_a < 1.0:
            soc = 100.0
        elif soc >= 100.0:
            soc = 99.0
    elif current_a < -0.5:
        soc = interpolate_table(LFP_DISCHARGE_TABLE, avg_cell_mv)
        if min_cell_mv <= 2750:
            soc = 0.0
    else:
        soc = interpolate_table(LFP_REST_TABLE, avg_cell_mv)
    return round(soc, 1)


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
    min_c = min(alive)
    max_c = max(alive)
    avg_c = int(round(sum(alive) / len(alive))) if alive else min_c
    volts = regs[0] / 100.0 if n > 0 else 0.0
    curr = s16(regs[1]) / 100.0 if n > 1 else 0.0
    real_soc = calc_adaptive_soc(avg_c, min_c, max_c, curr, volts) if min_c > 0 else None
    return {
        "voltage": volts,
        "current": curr,
        "cells": cells,
        "cell_num": cell_num,
        "cell_min": min_c,
        "cell_max": max_c,
        "cell_diff": max_c - min_c,
        "temp_pcb": s16(r(18, 0)),
        "temp_avg": s16(r(19, 0)),
        "temp_max": s16(r(20, 0)),
        "cap_remaining": r(21),
        "max_charge_a": r(22),
        "soh": r(23),
        "soc": r(24),
        "real_soc": real_soc,
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
    frames = []
    i = 0
    n = len(buf)
    while i < n:
        if i + 3 > n:
            break
        addr, fc, b2 = buf[i], buf[i + 1], buf[i + 2]
        if fc != 0x03 or addr < 1 or addr > 16:
            i += 1
            continue
        if b2 == 0x00:                       # request: start_hi=0, len 8
            ln = 8
            kind = "req"
        elif 10 <= b2 <= 100 and (b2 % 2 == 0):  # valid response byte count (46, 56, 78)
            ln = b2 + 5
            kind = "resp"
        else:
            i += 1
            continue
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


# --------------------------------------------------------------------------
# CSV log format: readable header row + volts + date (ported from bms_webX)
# --------------------------------------------------------------------------
LOG_CELL_SLOTS = 16
LOG_TEMP_SLOTS = 6

# v2.4: two independent logging profiles (may run together; Alarm has priority).
LOG_KIND_ALARM = "Alarm"
LOG_KIND_WORK = "Work"
DEFAULT_WORK_INTERVAL_S = 5.0
MIN_WORK_INTERVAL_S = 1.0
MAX_WORK_INTERVAL_S = 3600.0
# Temperature events for the Alarm profile (degrees C).
ALARM_TEMP_HIGH_C = 45.0
ALARM_TEMP_LOW_C = 0.0


def _log_slots(parsed: dict) -> tuple:
    """Return (cell_columns, temp_columns) for a log file, fixed on creation."""
    cells = parsed.get("cells") or []
    try:
        n_cells = int(parsed.get("cell_num") or len(cells) or 15)
    except (TypeError, ValueError):
        n_cells = 15
    n_cells = max(15, min(LOG_CELL_SLOTS, n_cells))
    temps = parsed.get("temp_sensors") or []
    n_temps = min(LOG_TEMP_SLOTS, len(temps))
    return n_cells, n_temps


def build_log_header(cell_slots: int = 15, temp_slots: int = 6) -> list:
    """First CSV row: readable register names instead of raw r00..r38."""
    header = [
        "Date", "Log", "addr", "Voltage_V", "Current_A", "Power_W",
        "SOC_pct", "Cap_Remaining_Ah", "SOH_pct", "Max_Charge_A",
        "Cycles", "Full_Capacity_Ah", "Designed_Cap_Ah", "Cell_Num",
        "Cell_Min_V", "Cell_Max_V", "Cell_Diff_V",
        "Temp_PCB_C", "Temp_Avg_C", "Temp_Max_C",
    ]
    header += [f"Cell_{i}_V" for i in range(1, cell_slots + 1)]
    header += [f"Temp_{i}_C" for i in range(1, temp_slots + 1)]
    header += ["Status_Hex", "Warning_Hex", "Protection_Hex", "Error_Hex", "Balance_Hex"]
    return header


def _mv_to_v(mv):
    return round(mv / 1000.0, 3) if mv is not None else None


def build_log_values(parsed: dict, addr: int, cell_slots: int = 15,
                     temp_slots: int = 6, ts: str = None,
                     log_kind: str = LOG_KIND_WORK) -> list:
    """One CSV data row: volts for cells, decoded regs 31..35, ISO-like date.

    log_kind is written into the "Log" column: Alarm or Work (see logging profiles).
    """
    cells = parsed.get("cells") or []
    temps = parsed.get("temp_sensors") or []
    volts = parsed.get("voltage")
    curr = parsed.get("current")
    power = round(volts * curr, 1) if (volts is not None and curr is not None) else None
    full_cap = parsed.get("full_cap_ah")

    row = [
        ts or datetime.now().strftime("%Y-%m-%d | %H:%M:%S"),
        log_kind,
        f"0x{addr:02X}",
        volts, curr, power,
        parsed.get("soc"), parsed.get("cap_remaining"), parsed.get("soh"),
        parsed.get("max_charge_a"), parsed.get("cycles"),
        round(full_cap, 2) if full_cap is not None else None,
        parsed.get("designed_cap_ah"), parsed.get("cell_num"),
        _mv_to_v(parsed.get("cell_min")),
        _mv_to_v(parsed.get("cell_max")),
        _mv_to_v(parsed.get("cell_diff")),
        parsed.get("temp_pcb"), parsed.get("temp_avg"), parsed.get("temp_max"),
    ]
    for i in range(cell_slots):
        row.append(_mv_to_v(cells[i]) if i < len(cells) else None)
    for i in range(temp_slots):
        row.append(temps[i] if i < len(temps) else None)

    def hx(v):
        return f"0x{v:04X}" if v is not None else None

    row += [
        hx(parsed.get("status")), hx(parsed.get("warning")),
        hx(parsed.get("protection")), hx(parsed.get("error_code")),
        hx(parsed.get("balance")),
    ]
    return row


def calculate_autonomy(avg_voltage: float, total_current: float, cap_remaining: float, full_cap: float) -> dict:
    """Calculates stored energy in kWh and ETA charge/discharge time."""
    energy_kwh = (avg_voltage * cap_remaining) / 1000.0 if (avg_voltage and cap_remaining) else 0.0
    energy_full_kwh = (avg_voltage * full_cap) / 1000.0 if (avg_voltage and full_cap) else 0.0
    power_kw = (avg_voltage * total_current) / 1000.0 if avg_voltage else 0.0

    if total_current > 0.5:
        needed_ah = max(0.0, full_cap - cap_remaining)
        eta_h = needed_ah / total_current
        h = int(eta_h)
        m = int((eta_h - h) * 60)
        eta_str = f"до 100%: ~{h} год {m:02d} хв"
        state_label = "Зарядка"
    elif total_current < -0.5:
        eta_h = cap_remaining / abs(total_current)
        h = int(eta_h)
        m = int((eta_h - h) * 60)
        eta_str = f"автономність: ~{h} год {m:02d} хв"
        state_label = "Розрядка"
    else:
        eta_str = "стендбай / буфер"
        state_label = "Очікування"

    return {
        "energy_kwh": round(energy_kwh, 2),
        "energy_full_kwh": round(energy_full_kwh, 2),
        "power_kw": round(power_kw, 3),
        "eta_str": eta_str,
        "state_label": state_label,
    }


def get_bank_summary(packs: dict) -> dict:
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
            "autonomy": {
                "energy_kwh": 0.0,
                "energy_full_kwh": 0.0,
                "power_kw": 0.0,
                "eta_str": "—",
                "state_label": "Офлайн"
            }
        }

    voltages = [p["parsed"]["voltage"] for p in online_packs if p["parsed"].get("voltage") is not None]
    avg_volt = sum(voltages) / len(voltages) if voltages else 0.0

    currents = [p["parsed"]["current"] for p in online_packs if p["parsed"].get("current") is not None]
    total_curr = sum(currents)

    socs = [p["parsed"]["soc"] for p in online_packs if p["parsed"].get("soc") is not None]
    avg_soc = round(sum(socs) / len(socs)) if socs else 0

    real_socs = [p["parsed"]["real_soc"] for p in online_packs if p["parsed"].get("real_soc") is not None]
    avg_real_soc = round(sum(real_socs) / len(real_socs), 1) if real_socs else None

    cap_rem = sum(p["parsed"]["cap_remaining"] for p in online_packs if p["parsed"].get("cap_remaining") is not None)
    full_cap = sum(p["parsed"].get("full_cap_ah") or p["parsed"].get("designed_cap_ah") or 100.0 for p in online_packs)

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

    autonomy = calculate_autonomy(avg_volt, total_curr, cap_rem, full_cap)

    return {
        "total_packs": len(packs),
        "online_packs": len(online_packs),
        "avg_voltage": round(avg_volt, 2),
        "total_current": round(total_curr, 2),
        "total_power_kw": autonomy["power_kw"],
        "avg_soc": avg_soc,
        "avg_real_soc": avg_real_soc,
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
        "autonomy": autonomy,
    }


class BmsBus:
    PASSIVE_STALE_S = 35.0
    PASSIVE_GAP_S = 0.15

    def __init__(self, port: str, baud: int, initial_addr: int = DEFAULT_ADDR,
                 mock: bool = False, passive: bool = True):
        self.port = port
        self.baud = baud
        self.focus_addr = initial_addr
        self.mock = mock
        self.mode = "passive" if passive else "active"
        self.ser = None
        self.lock = threading.Lock()

        self.packs = {}
        self.active_addrs = []
        self.is_scanning = False
        self.scan_msg = ""
        self.last_err = None

        self._rescan_requested = True
        self._passive_scan_until = 0.0
        self._last_rx_t = 0.0
        self._bg_probe_idx = 0
        self._stop = False
        self._mock_t = 0.0

        # Diagnostics & History
        self.history = collections.deque(maxlen=300)
        self.cell_stats = {}    # (addr, cell_idx) -> {"min_cnt": int, "max_cnt": int, "samples": int}
        self.prev_i = {}        # addr -> float
        self.prev_cells = {}    # addr -> [int]
        self.rint = {}          # addr -> [15 floats in mΩ]
        self.bal_seconds = {}   # addr -> [15 floats in sec]
        self.last_history_t = 0.0

        # CSV logging (per pack: own file, readable header, volts, date)
        # addr -> {"on", "path", "rows", "last_ts", "cell_slots", "temp_slots",
        #          "alarm_sig", "work_next_t", "alarm_rows", "work_rows"}
        self.logs = {}
        # v2.4: independent logging profiles; one shared file per pack.
        # alarm=True -> write on r25..r28/temperature state change only
        # work=True  -> periodic samples every work_interval seconds
        self.log_cfg = {
            "alarm": True,
            "work": True,
            "work_interval": DEFAULT_WORK_INTERVAL_S,
        }
        # True while at least one profile is selected: new packs start logging
        # automatically; the Stop button clears it.
        self.log_auto = False
        self._log_write_lock = threading.Lock()

    def open(self) -> bool:
        if self.mock:
            return True
        if serial is None:
            self.last_err = "pySerial not installed (pip3 install pyserial)"
            return False
        try:
            to = self.PASSIVE_GAP_S if self.mode == "passive" else RESP_TIMEOUT_S
            self.ser = serial.Serial(
                self.port, self.baud, timeout=to,
                dsrdtr=False, rtscts=False, xonxoff=False,
            )
            # Сброс линий DTR/RTS для правильной инициализации донгла на Windows
            try:
                self.ser.dtr = False
                self.ser.rts = False
                time.sleep(0.05)
                self.ser.dtr = True
                self.ser.rts = True
                time.sleep(0.05)
            except Exception:
                pass
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            return True
        except Exception as e:
            self.last_err = f"open {self.port}: {e}"
            return False

    def set_mode(self, new_mode: str) -> None:
        with self.lock:
            if new_mode not in ("active", "passive") or self.mode == new_mode:
                return
            self.mode = new_mode
            if not self.mock and self.ser and self.ser.is_open:
                try:
                    self.ser.timeout = self.PASSIVE_GAP_S if self.mode == "passive" else RESP_TIMEOUT_S
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
            self._rescan_requested = True
            self.is_scanning = True
            self.scan_msg = "Запуск сканування 0x01..0x10..." if self.mode == "active" else "Пасивне сканування: слухаю шину RS485..."
            if self.mode == "passive":
                self._passive_scan_until = time.time() + 4.0

    def request_rescan(self) -> None:
        with self.lock:
            self._rescan_requested = True
            self.is_scanning = True
            self.scan_msg = "Запуск сканування 0x01..0x10..." if self.mode == "active" else "Пасивне сканування: слухаю шину RS485..."
            if self.mode == "passive":
                self._passive_scan_until = time.time() + 4.0

    def _log_state(self, addr: int) -> dict:
        st = self.logs.get(addr)
        if st is None:
            st = {"on": False, "path": None, "rows": 0, "last_ts": None,
                  "cell_slots": 15, "temp_slots": 6,
                  "alarm_sig": None, "work_next_t": None,
                  "alarm_rows": 0, "work_rows": 0}
            self.logs[addr] = st
        return st

    def _start_log_state(self, st: dict, addr: int) -> None:
        """Create a fresh CSV log for a pack (caller holds self.lock)."""
        st["path"] = f"bms_log_{addr:02X}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        st["on"] = True
        st["rows"] = 0
        st["last_ts"] = None
        st["alarm_sig"] = None
        st["work_next_t"] = None
        st["alarm_rows"] = 0
        st["work_rows"] = 0

    def set_log(self, on: bool, addrs=None) -> list:
        """Start/stop CSV logging. addrs=None -> all active (or known) packs."""
        with self.lock:
            self.log_auto = on
            if addrs is None:
                targets = list(self.active_addrs) or sorted(self.packs)
            else:
                targets = [a for a in addrs if a]
            for a in targets:
                st = self._log_state(a)
                if on and not st["on"]:
                    self._start_log_state(st, a)
                st["on"] = on
            return targets

    def set_log_cfg(self, alarm=None, work=None, work_interval=None) -> dict:
        """Update the logging profiles. Both may be active at the same time."""
        with self.lock:
            if alarm is not None:
                self.log_cfg["alarm"] = bool(alarm)
            if work is not None:
                self.log_cfg["work"] = bool(work)
            if work_interval is not None:
                try:
                    iv = float(work_interval)
                except (TypeError, ValueError):
                    iv = DEFAULT_WORK_INTERVAL_S
                self.log_cfg["work_interval"] = max(
                    MIN_WORK_INTERVAL_S, min(MAX_WORK_INTERVAL_S, iv))
            if self.log_cfg["alarm"] or self.log_cfg["work"]:
                self.log_auto = True
            return dict(self.log_cfg)

    def read_regs(self, addr: int, timeout: float = RESP_TIMEOUT_S) -> list:
        req = bytes([addr, 0x03, 0x00, 0x00, 0x00, 0x27])
        req += crc_bytes(req)
        self.ser.timeout = timeout
        self.ser.reset_input_buffer()
        self.ser.write(req)
        self.ser.flush() # Ждем, пока байты физически уйдут в порт
        
        # Читаем заголовок. Если с первого раза пусто (задержка Windows), пробуем еще раз
        hdr = self.ser.read(3)
        if not hdr:
            time.sleep(0.05)
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
        rnd = random.Random((self._mock_t // 8) + addr * 17)
        # Alternate current steps periodically to showcase R_int and Weak Cell tracking
        cycle_phase = int(self._mock_t) % 60
        if cycle_phase < 25:
            current = 1500 if addr == 1 else 1450   # +15A charging
        elif cycle_phase < 35:
            current = 200 if addr == 1 else 150     # +2A trickle
        else:
            current = -800 if addr == 1 else -750   # -8A discharging

        base_v = 3465 if addr == 1 else 3460
        delta = rnd.randint(-3, 3)
        base = max(3300, min(3600, base_v + delta))

        cells = [base + rnd.randint(-4, 4) for _ in range(15)]
        if addr == 1:
            # Simulate cell #8 having a slightly higher resistance & lagging under load
            if current < 0:
                cells[7] = base - 28
            elif current > 1000:
                cells[7] = base + 12
        if addr == 2:
            # Cell 15 balancing active in pack 2
            cells[14] = base + 26
            bal_reg = 0x4000
        else:
            bal_reg = 0

        cells += [0]
        voltage = sum(cells[:15]) // 10
        regs = [0] * 39
        regs[0] = voltage
        regs[1] = current & 0xFFFF
        for i, c in enumerate(cells):
            regs[2 + i] = c
        regs[18], regs[19], regs[20] = 23, 22, 24
        regs[21] = 85 if addr == 1 else 84
        regs[22] = 100
        regs[23] = 100
        regs[24] = 85 if addr == 1 else 84
        regs[25] = 1 if current > 0 else (2 if current < 0 else 0)
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

    def _update_pack_telemetry(self, a: int, regs: list, dev_info: dict = None) -> None:
        """Internal updater for decoded telemetry, diagnostics, and resistance tracking."""
        parsed = decode_regs(regs)
        curr = parsed["current"]
        cells = parsed["cells"]
        now = time.time()

        # 1. Balancing duration tracking
        if parsed.get("balance_bits"):
            for b in parsed["balance_bits"]:
                if b["on"]:
                    idx = b["n"] - 1
                    b_list = self.bal_seconds.setdefault(a, [0.0] * 15)
                    b_list[idx] += POLL_S

        # 2. Weak cell tracking (identify chronically lagging cells)
        alive = [c for c in cells if c > 0]
        if len(alive) >= 15:
            min_val = min(alive)
            max_val = max(alive)
            for i, c in enumerate(cells[:15]):
                st = self.cell_stats.setdefault((a, i), {"min_cnt": 0, "max_cnt": 0, "samples": 0})
                st["samples"] += 1
                if c == min_val and curr <= 0.5:
                    st["min_cnt"] += 1
                if c == max_val and curr > 0.5:
                    st["max_cnt"] += 1

        # 3. Dynamic internal resistance (R_int in mΩ) estimation on current steps
        old_i = self.prev_i.get(a)
        old_cells = self.prev_cells.get(a)
        if old_i is not None and old_cells and len(old_cells) >= 15 and len(cells) >= 15:
            di = abs(curr - old_i)
            if di >= 2.0:
                r_list = self.rint.setdefault(a, [0.0] * 15)
                for i in range(15):
                    du = abs(cells[i] - old_cells[i])
                    r_cand = du / di  # mV / A = mΩ
                    if 0.1 <= r_cand <= 15.0:
                        if r_list[i] == 0.0:
                            r_list[i] = round(r_cand, 2)
                        else:
                            r_list[i] = round(0.8 * r_list[i] + 0.2 * r_cand, 2)
        self.prev_i[a] = curr
        self.prev_cells[a] = list(cells[:15])

        # Store pack state
        p = self.packs.get(a, {})
        dinfo = dev_info or p.get("dev_info") or {}
        self.packs[a] = {
            "addr": a,
            "addr_hex": f"0x{a:02X}",
            "online": True,
            "last_seen": now,
            "consecutive_fails": 0,
            "raw": regs,
            "parsed": parsed,
            "dev_info": dinfo,
            "rint": self.rint.get(a, [0.0] * 15),
            "bal_seconds": [int(s) for s in self.bal_seconds.get(a, [0.0] * 15)],
            "err": None,
        }

    def _sample_history(self) -> None:
        """Records rolling history point (~1 sec interval)."""
        now = time.time()
        if now - self.last_history_t < 1.0:
            return
        self.last_history_t = now
        sum_data = get_bank_summary(self.packs)
        if sum_data["online_packs"] > 0:
            self.history.append({
                "t": int(now),
                "v": sum_data["avg_voltage"],
                "i": sum_data["total_current"],
                "p": sum_data["total_power_kw"],
                "diff": sum_data["cell_delta_mv"],
            })

    def get_diagnostics_summary(self) -> dict:
        """Finds chronically lagging cells and summarizes internal resistance."""
        weak_cell = None
        highest_rate = 0.0
        for (a, idx), st in self.cell_stats.items():
            if st["samples"] >= 15:
                rate = st["min_cnt"] / st["samples"]
                if rate > 0.45 and rate > highest_rate:
                    highest_rate = rate
                    weak_cell = {
                        "addr_hex": f"0x{a:02X}",
                        "cell_idx": idx + 1,
                        "rate_pct": int(rate * 100),
                        "text": f"АКБ 0x{a:02X}, ком. #{idx + 1} відстає у {int(rate * 100)}% вимірювань під навантаженням"
                    }

        all_r = []
        for a, r_list in self.rint.items():
            for idx, r in enumerate(r_list):
                if r > 0.05:
                    all_r.append((r, a, idx + 1))

        if all_r:
            avg_r = sum(r for r, _, _ in all_r) / len(all_r)
            max_r = max(all_r, key=lambda x: x[0])
            r_summary = f"Сер. опір комірок: {avg_r:.2f} мОм (макс: {max_r[0]:.2f} мОм, АКБ 0x{max_r[1]:02X} #{max_r[2]})"
        else:
            r_summary = "Очікування стрибка струму для розрахунку опору комірок..."

        return {
            "weak_cell": weak_cell["text"] if weak_cell else "Розбаланс у нормі, стійких відстаючих комірок не виявлено",
            "has_weak_cell": bool(weak_cell),
            "rint_summary": r_summary,
        }

    def _alarm_signature(self, parsed: dict) -> tuple:
        """Alarm-profile state: registers 0025..0028 + temperature thresholds.

        Rows are written only when this signature changes, so a stable state
        produces no repeated records.
        """
        status = parsed.get("status") or 0
        warning = parsed.get("warning") or 0
        protection = parsed.get("protection") or 0
        error = parsed.get("error_code") or 0
        t_max = parsed.get("temp_max")
        temp_high = t_max is not None and t_max >= ALARM_TEMP_HIGH_C
        temp_low = t_max is not None and t_max <= ALARM_TEMP_LOW_C
        return (status, warning, protection, error, bool(temp_high), bool(temp_low))

    @staticmethod
    def _is_alarm(sig: tuple) -> bool:
        status, warning, protection, error, temp_high, temp_low = sig
        return bool(warning or protection or error or (status & 0x04)
                    or temp_high or temp_low)

    def log_row(self, regs: list, addr: int) -> None:
        """Append one sample to this pack's CSV file using the active profiles.

        Alarm profile: only on alarm-state change (activation/change/recovery).
        Work profile: periodic sample every work_interval seconds.
        Alarm has priority; when both are due, the Work sample is postponed.
        """
        parsed = decode_regs(regs)
        now = time.monotonic()

        with self.lock:
            st = self._log_state(addr)
            if not st["on"] and self.log_auto:
                self._start_log_state(st, addr)
            if not st["on"] or not st["path"]:
                return
            cfg = dict(self.log_cfg)
            kind = None
            sig = self._alarm_signature(parsed)
            prev_sig = st["alarm_sig"]

            if cfg["alarm"] and sig != prev_sig:
                # Activation, any change while active, or recovery.
                if self._is_alarm(sig) or (prev_sig is not None and self._is_alarm(prev_sig)):
                    kind = LOG_KIND_ALARM
                st["alarm_sig"] = sig

            if kind is None and cfg["work"]:
                if st["work_next_t"] is None or now >= st["work_next_t"]:
                    kind = LOG_KIND_WORK

            if kind is None:
                return

            # Alarm supersedes the periodic sample: no two rows for one cycle.
            if cfg["work"]:
                st["work_next_t"] = now + cfg["work_interval"]

            path = st["path"]
            cell_slots, temp_slots = st["cell_slots"], st["temp_slots"]

        try:
            with self._log_write_lock:
                new_file = not os.path.exists(path)
                if new_file:
                    cell_slots, temp_slots = _log_slots(parsed)
                    with self.lock:
                        st["cell_slots"], st["temp_slots"] = cell_slots, temp_slots
                with open(path, "a", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    if new_file:
                        w.writerow(build_log_header(cell_slots, temp_slots))
                    w.writerow(build_log_values(parsed, addr, cell_slots, temp_slots,
                                                log_kind=kind))
            with self.lock:
                st["rows"] += 1
                if kind == LOG_KIND_ALARM:
                    st["alarm_rows"] += 1
                else:
                    st["work_rows"] += 1
                st["last_ts"] = datetime.now().strftime("%Y-%m-%d | %H:%M:%S")
        except Exception as e:
            self.last_err = f"log {path}: {e}"
            if kind == LOG_KIND_ALARM:
                with self.lock:
                    st["alarm_sig"] = prev_sig

    def loop(self) -> None:
        buf = b""
        while not self._stop:
            if self.mode == "active":
                self.active_step()
            else:
                buf = self.passive_step(buf)
            self._sample_history()

    def active_step(self) -> None:
        if not self.mock and (self.ser is None or not self.ser.is_open):
            if not self.open():
                time.sleep(2.5)
                return

        # 1. Full auto-discovery sweep
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
                        self._update_pack_telemetry(a, regs, dev_info)
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
                    self._update_pack_telemetry(a, regs)
                self.log_row(regs, a)
            except Exception as e:
                with self.lock:
                    if a in self.packs:
                        self.packs[a]["consecutive_fails"] = self.packs[a].get("consecutive_fails", 0) + 1
                        self.packs[a]["err"] = str(e)
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
                    dev_info = self.read_dev_info(probe_a) if not self.mock else {
                        "model": "V-LFP48100-15S", "fw_version": "V10R18", "serial": f"SN-2026102200{probe_a}"}
                    with self.lock:
                        self._update_pack_telemetry(probe_a, regs, dev_info)
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

        now = time.time()
        with self.lock:
            if self._rescan_requested:
                self._rescan_requested = False
                self.is_scanning = True
                self.scan_msg = "Пасивне сканування: слухаю шину RS485 (1..16)..."
                self._passive_scan_until = now + 4.0
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.reset_input_buffer()
                    except Exception:
                        pass
                buf = b""

            if getattr(self, "_passive_scan_until", 0) > 0:
                if now < self._passive_scan_until:
                    self.is_scanning = True
                    self.scan_msg = "Пасивне сканування: слухаю шину RS485 (1..16)..."
                else:
                    self.is_scanning = False
                    self._passive_scan_until = 0
                    if not self.active_addrs:
                        self.scan_msg = "Шина мовчить"
                        self.last_err = "Шина RS485 мовчить. Перемкніть у ⚡ Активний режим для прямого опитування."

        if self.mock:
            time.sleep(0.8)
            for a in (1, 2):
                regs = self.mock_regs(a)
                with self.lock:
                    self._update_pack_telemetry(a, regs, {"model": "V-LFP48100-15S", "fw_version": "V10R18", "serial": f"SN-2026102200{a}"})
                self.log_row(regs, a)
            with self.lock:
                self.active_addrs = [1, 2]
                self.is_scanning = False
                self._passive_scan_until = 0
            return b""

        try:
            avail = getattr(self.ser, "in_waiting", 0)
            chunk = self.ser.read(avail if avail > 0 else 1)
            if chunk:
                buf += chunk
                self._last_rx_t = now
                frames, buf = parse_bus(buf)
            else:
                frames = []
                if (now - getattr(self, "_last_rx_t", now)) > 1.5:
                    buf = b""

            for kind, addr, payload in frames:
                if kind == "resp" and (1 <= addr <= 16):
                    if len(payload) >= 28:
                        with self.lock:
                            self._update_pack_telemetry(addr, payload)
                            if addr not in self.active_addrs:
                                self.active_addrs.append(addr)
                                self.active_addrs.sort()
                            self.is_scanning = False
                            self._passive_scan_until = 0
                        self.log_row(payload, addr)
                    elif len(payload) == 23:
                        dev_info = decode_dev_info_block(payload)
                        with self.lock:
                            p = self.packs.setdefault(addr, {})
                            p["dev_info"] = dev_info
                            if addr in self.packs:
                                self.packs[addr]["dev_info"] = dev_info

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
                lg = self.logs.get(a)
                p_copy.update({
                    "log_on": bool(lg and lg["on"]),
                    "log_path": lg["path"] if lg else None,
                    "log_rows": lg["rows"] if lg else 0,
                    "log_alarm_rows": lg["alarm_rows"] if lg else 0,
                    "log_work_rows": lg["work_rows"] if lg else 0,
                    "log_last_ts": lg["last_ts"] if lg else None,
                })
                packs_dict[f"0x{a:02X}"] = p_copy

            summary = get_bank_summary(self.packs)
            diag = self.get_diagnostics_summary()

            log_files = {
                f"0x{a:02X}": {
                    "on": st["on"], "path": st["path"], "rows": st["rows"],
                    "alarm_rows": st["alarm_rows"], "work_rows": st["work_rows"],
                    "last_ts": st["last_ts"],
                }
                for a, st in self.logs.items()
            }
            active_logs = [v for v in log_files.values() if v["on"]]
            if len(active_logs) == 1:
                log_path = active_logs[0]["path"]
            elif len(active_logs) > 1:
                log_path = f"{len(active_logs)} файли"
            else:
                log_path = None

            return {
                "mode": self.mode,
                "is_scanning": self.is_scanning,
                "scan_msg": self.scan_msg,
                "active_addrs": [f"0x{a:02X}" for a in sorted(self.active_addrs)],
                "summary": summary,
                "diagnostics": diag,
                "history": list(self.history),
                "packs": packs_dict,
                "focus_addr": f"0x{self.focus_addr:02X}",
                "port": self.port,
                "baud": self.baud,
                "log_on": bool(active_logs),
                "log_path": log_path,
                "log_rows": sum(st["rows"] for st in self.logs.values()),
                "log_files": log_files,
                "log_cfg": dict(self.log_cfg),
                "last_err": self.last_err,
                "ts": datetime.now().strftime("%Y-%m-%d | %H:%M:%S"),
            }


HTML = """<!doctype html>
<html lang="uk"><head><meta charset="utf-8">
<title>Vision BMS Multi-Pack Monitor & Diagnostics</title>
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
button:disabled{opacity:.6;cursor:wait}
.btn-mode{background:#238636;color:#fff;border:1px solid #2ea043}
.btn-mode.passive{background:#1f6feb;border-color:#388bfd}
.btn-rescan{background:#21262d;border:1px solid var(--border);color:var(--fg)}
.btn-rescan:hover{background:#30363d}
.btn-report{background:#8957e5;color:#fff;border:1px solid #a371f7}

/* Logging profiles (v2.4) */
.log-profiles{margin-top:12px;padding:10px 12px;border:1px solid var(--border);border-radius:8px;background:#151a20}
.log-profiles-title{font-weight:600;margin-bottom:8px}
.log-profile-row{display:flex;align-items:flex-start;gap:8px;margin:6px 0;font-size:13px;line-height:1.4;cursor:pointer}
.log-profile-row input[type=checkbox]{margin-top:2px;accent-color:var(--acc)}
.log-profile-row input[type=number]{width:66px;background:#0d1117;color:var(--fg);border:1px solid var(--border);border-radius:5px;padding:2px 6px;font-size:13px}
.log-profiles-note{color:var(--dim);font-size:12px;margin-top:8px;border-top:1px solid var(--border);padding-top:8px}
.log-profiles-note b{color:var(--fg)}

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
.card h2{font-size:13px;text-transform:uppercase;letter-spacing:.06em;color:var(--dim);margin:0 0 10px;display:flex;justify-content:space-between;align-items:center}
.big{display:flex;gap:20px;flex-wrap:wrap}
.big div span{display:block}
.big .v{font-size:26px;font-weight:600;font-variant-numeric:tabular-nums}
.big .l{font-size:11px;color:var(--dim);text-transform:uppercase}

/* Cell display */
.cells{display:grid;grid-template-columns:repeat(auto-fill,minmax(84px,1fr));gap:6px}
.cell{background:#232a31;border-radius:6px;padding:6px;text-align:center;border:1px solid transparent}
.cell .n{font-size:10px;color:var(--dim)}
.cell .mv{font-weight:600;font-variant-numeric:tabular-nums;font-size:14px}
.cell .rint{font-size:10px;color:var(--acc);margin-top:2px}
.cell.min{border-color:var(--bad);background:#2f1d1f}
.cell.max{border-color:var(--ok);background:#1a2e22}
.cell.bal-active{box-shadow:0 0 8px var(--warn);border-color:var(--warn);background:#2d2415}
.cell-legend{display:flex;gap:15px;flex-wrap:wrap;margin-top:12px;font-size:13px;font-weight:600;color:var(--dim);align-items:center}
.cell-legend .box{display:inline-block;width:12px;height:12px;border-radius:3px;margin-right:5px;vertical-align:middle}
.cell-legend .box.min{border:1px solid var(--bad);background:#2f1d1f}
.cell-legend .box.max{border:1px solid var(--ok);background:#1a2e22}
.cell-legend b{color:var(--fg);font-variant-numeric:tabular-nums}

/* Table */
table.tbl{width:100%;border-collapse:collapse;font-size:13px}
table.tbl th{padding:8px 10px;border-bottom:1px solid var(--border);color:var(--dim);text-align:left;font-size:11px;text-transform:uppercase}
table.tbl td{padding:9px 10px;border-bottom:1px solid #21262d;font-variant-numeric:tabular-nums}
table.tbl tr:hover{background:#1f262e}
table.kv td{padding:4px 8px 4px 0;color:var(--dim);vertical-align:top;white-space:nowrap}
table.kv td:last-child{color:var(--fg);font-variant-numeric:tabular-nums;text-align:right;white-space:normal;word-break:break-all}

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

/* Canvas styling */
.chart-box{background:#161b22;border:1px solid var(--border);border-radius:8px;padding:10px;margin-bottom:16px}
canvas{width:100%;height:auto;display:block}

/* Report Modal */
#report_modal{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.85);z-index:999;overflow-y:auto;padding:24px}
.report-paper{background:#fff;color:#111;max-width:960px;margin:0 auto;border-radius:8px;padding:28px 32px;box-shadow:0 8px 30px rgba(0,0,0,0.5)}
.report-paper h2{margin-top:0;color:#000;border-bottom:2px solid #333;padding-bottom:8px;font-size:20px}
.report-paper table{width:100%;border-collapse:collapse;margin:12px 0;font-size:12px}
.report-paper th,.report-paper td{border:1px solid #ccc;padding:6px 8px;text-align:left}
.report-paper th{background:#f0f0f0;font-weight:600}
.report-paper .rep-sec{font-size:13px;font-weight:700;margin:16px 0 6px;text-transform:uppercase;color:#333;letter-spacing:0.5px}
.report-actions{display:flex;justify-content:flex-end;gap:10px;margin-bottom:16px}

.report-paper .rep-cells-table{width:100%;table-layout:fixed;border-collapse:collapse;margin:8px 0;font-size:11px}
.report-paper .rep-cells-table th, .report-paper .rep-cells-table td{border:1px solid #ccc;padding:4px 1px;text-align:center;word-break:keep-all}
.report-paper .rep-cells-table th:first-child, .report-paper .rep-cells-table td:first-child{width:46px;font-weight:700}
.report-paper .rep-cells-table th:last-child, .report-paper .rep-cells-table td:last-child{width:56px;font-weight:700}

/* Modals (Inverter, Alarm Help) */
.modal-overlay{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.85);z-index:999;overflow-y:auto;padding:24px}
.modal-box{background:#161b22;border:1px solid var(--border);color:var(--fg);max-width:800px;margin:30px auto;border-radius:12px;padding:24px 28px;box-shadow:0 12px 40px rgba(0,0,0,0.7)}
.modal-close-btn{background:#30363d;color:#fff;border:1px solid var(--border);padding:6px 14px;border-radius:6px;cursor:pointer;font-weight:600}
.modal-close-btn:hover{background:#3c444d}
.btn-inverter{background:#1f6feb;color:#fff;border:none;padding:6px 14px;border-radius:6px;font-weight:600;cursor:pointer}
.btn-inverter:hover{background:#388bfd}

/* Offline Log Viewer styles */
.drop-zone{border:2px dashed var(--border);border-radius:10px;padding:36px;text-align:center;cursor:pointer;background:#0d1117;transition:all 0.2s}
.drop-zone:hover, .drop-zone.dragover{border-color:var(--acc);background:#161b22}

@media print{
  @page{size:A4 portrait;margin:8mm 8mm 8mm 8mm}
  html, body{background:#fff !important;color:#000 !important;height:auto !important;min-height:0 !important;margin:0 !important;padding:0 !important}
  body > *{display:none !important}
  body > #report_modal{display:block !important;position:static !important;width:100% !important;height:auto !important;min-height:0 !important;background:#fff !important;padding:0 !important;margin:0 !important;overflow:visible !important}
  .report-actions{display:none !important}
  .report-paper{box-shadow:none !important;border-radius:0 !important;padding:0 !important;margin:0 !important;max-width:100% !important;width:100% !important}
  .report-paper h2{font-size:16px !important;margin:0 0 2px 0 !important;padding-bottom:4px !important}
  .report-paper .rep-sec{font-size:11px !important;margin:8px 0 3px !important}
  .report-paper table{font-size:10px !important;margin:4px 0 !important;width:100% !important}
  .report-paper th, .report-paper td{padding:3px 4px !important}
  .report-paper .rep-cells-table{font-size:8.5px !important;margin:4px 0 !important}
  .report-paper .rep-cells-table th, .report-paper .rep-cells-table td{padding:2px 1px !important}
  .report-paper .rep-cells-table th:first-child, .report-paper .rep-cells-table td:first-child{width:38px !important}
  .report-paper .rep-cells-table th:last-child, .report-paper .rep-cells-table td:last-child{width:48px !important}
}
</style></head><body>

<header>
  <h1>Vision BMS Multi-Pack Monitor <span id="conn" class="pill">…</span></h1>
  <div class="controls">
    <button id="modebtn" class="btn-mode passive" onclick="toggleMode()">👂 Режим: Пасивний (Sniffer)</button>
    <button class="btn-rescan" onclick="triggerRescan()">🔄 Пересканувати всі 16</button>
    <button class="btn-inverter" onclick="openInverterModal()">⚙️ Налаштування інвертора</button>
    <button class="btn-report" onclick="openReport()">📄 Діагностичний звіт</button>
  </div>
</header>

<div class="sub">
  RS485 · <span id="portinfo">—</span> · <span id="modedesc">—</span> · <span id="seeninfo">—</span> · Оновлено: <span id="ts">—</span>
</div>

<div class="tabs" id="tabbar">
  <!-- Dynamic tabs -->
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

<!-- VIEW 2: CHARTS & DEEP DIAGNOSTICS TAB -->
<div id="view_charts" style="display:none">
  <!-- AUTONOMY BANNER -->
  <div class="bank-grid" style="margin-bottom:16px">
    <div class="bank-card" style="border-left:4px solid var(--ok)">
      <div class="l">Запас енергії в банку</div>
      <div class="v" id="c_energy" style="color:var(--ok)">— кВт·год</div>
      <div class="subtext">Розраховано з урахуванням поточної ємності</div>
    </div>
    <div class="bank-card" style="border-left:4px solid var(--acc)">
      <div class="l">Прогноз часу (ETA)</div>
      <div class="v" id="c_eta" style="font-size:20px;color:var(--fg)">—</div>
      <div class="subtext" id="c_state">Режим: —</div>
    </div>
  </div>

  <!-- LIVE ROLLING CANVAS CHARTS -->
  <div class="card" style="margin-bottom:16px">
    <h2>📈 Напруга та струм банку (в реальному часі)</h2>
    <div class="chart-box">
      <canvas id="historyChart" width="840" height="190"></canvas>
    </div>
  </div>

  <div class="card" style="margin-bottom:16px">
    <h2>⚖️ Динаміка розбалансу комірок (Δ банку, mV)</h2>
    <div class="chart-box">
      <canvas id="deltaChart" width="840" height="150"></canvas>
    </div>
  </div>

  <!-- DIAGNOSTICS & WEAK CELL BANNER -->
  <div class="card" style="margin-bottom:16px;border-left:4px solid var(--acc)">
    <h2>🔬 Діагностика здоров'я та аналітика комірок</h2>
    <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px">
      <div>
        <div style="font-size:11px;color:var(--dim);text-transform:uppercase">Слабка / Відстаюча комірка</div>
        <div id="diag_weak" style="font-size:14px;font-weight:600;margin-top:4px">—</div>
      </div>
      <div>
        <div style="font-size:11px;color:var(--dim);text-transform:uppercase">Внутрішній опір (R_int)</div>
        <div id="diag_rint" style="font-size:14px;font-weight:600;margin-top:4px;color:var(--acc)">—</div>
      </div>
    </div>
  </div>

  <!-- DIAGNOSTIC RESISTANCE & BALANCING TABLE -->
  <div class="card" style="margin-bottom:16px">
    <h2>Зведена аналітика опору та часу балансування по кожному АКБ</h2>
    <div style="overflow-x:auto">
      <table class="tbl">
        <thead>
          <tr>
            <th>АКБ</th>
            <th>Сер. опір (мОм)</th>
            <th>Макс. опір (мОм)</th>
            <th>Статус балансування</th>
            <th>Сумарний час балансування</th>
          </tr>
        </thead>
        <tbody id="diag_rows">
          <tr><td colspan="5" style="text-align:center;color:var(--dim);padding:14px">Очікування даних...</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</div>

<!-- VIEW 2: PACK DETAIL -->
<div id="view_pack" style="display:none">
  <!-- CELL BAR CHART -->
  <div class="card" style="margin-bottom:16px">
    <h2>📊 Гістограма 15 комірок вибраного АКБ (з активним балансом та опором)</h2>
    <div class="chart-box">
      <canvas id="cellBarChart" width="840" height="170"></canvas>
    </div>
  </div>

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
        <tr><td>Серійний №</td><td id="p_sn">—</td></tr>
        <tr><td>SOC (OCV)</td><td id="p_real_soc">—</td></tr>
        <tr><td>SOC (BMS)</td><td id="p_soc">-</td></tr>
        <tr><td>SOH</td><td id="p_soh">—</td></tr>
        <tr><td>Кількість циклів</td><td id="p_cyc">—</td></tr>
        <tr><td>Номінал</td><td id="p_cap">—</td></tr>
        <tr><td>Full Capacity</td><td id="p_fcap">—</td></tr>
      </table>
    </div>

    <div class="card">
      <h2>Комірки 15S</h2>
      <div class="cells" id="p_cells"></div>
      <div class="cell-legend">
        <span><span class="box min"></span>V_min = <b id="p_vmin">—</b></span>
        <span><span class="box max"></span>V_max = <b id="p_vmax">—</b></span>
        <span id="p_difft" style="color:var(--fg);font-weight:700">—</span>
      </div>
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
      <h2>Error (0028) <span id="p_errraw" style="color:var(--dim);font-weight:400"></span></h2>
      <div class="leds" id="p_errbits"></div>
    </div>

    <div class="card">
      <h2>Активне балансування (0038)</h2>
      <div class="bal" id="p_bal"></div>
    </div>

    <div class="card">
      <h2>📝 CSV-лог цієї АКБ</h2>
      <table class="kv" style="width:100%">
        <tr><td style="width:110px">Файл</td><td id="p_logpath" style="font-family:ui-monospace,SFMono-Regular,Consolas,monospace;font-size:11px;color:var(--acc);word-break:break-all">—</td></tr>
        <tr><td>Записано рядків</td><td id="p_logrows">0</td></tr>
        <tr><td>Alarm / Work</td><td><span id="p_log_alarm">0</span> / <span id="p_log_work">0</span></td></tr>
        <tr><td>Останній запис</td><td id="p_logts" style="font-size:12px">—</td></tr>
      </table>
      <div style="margin-top:10px">
        <button id="p_logbtn" onclick="togglePackLog()">Старт запису</button>
      </div>
    </div>
  </div>
</div>

<!-- VIEW 4: OFFLINE CSV LOG VIEWER -->
<div id="view_logviewer" style="display:none">
  <div class="card">
    <h2>📂 Офлайн-аналізатор записаних CSV-логів</h2>
    <div style="color:var(--dim);font-size:13px;margin-bottom:14px">
      Завантажте збережений файл <code>bms_log_*.csv</code> для детального аналізу внутрішнього опору R_int, динаміки напруг та балансування.
    </div>

    <div id="drop_zone" class="drop-zone" onclick="document.getElementById('csv_file_input').click()">
      <div style="font-size:32px;margin-bottom:8px">📥</div>
      <div style="font-size:15px;font-weight:600;margin-bottom:4px">Перетягніть сюди файл .csv або натисніть для вибору</div>
      <div style="font-size:12px;color:var(--dim)">Працює повністю у браузері, конфіденційно, без відправки на сервер</div>
      <input type="file" id="csv_file_input" accept=".csv" style="display:none" onchange="handleCsvFile(this.files[0])">
    </div>

    <div id="log_results" style="display:none;margin-top:20px">
      <div style="display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px;margin-bottom:14px">
        <h3 id="log_file_title" style="margin:0;font-size:16px;color:var(--acc)"></h3>
        <button onclick="resetLogViewer()" style="background:#30363d;color:#fff;border:1px solid var(--border);padding:4px 12px;border-radius:6px;cursor:pointer">✕ Закрити аналіз</button>
      </div>

      <!-- Stats Grid -->
      <div class="bank-grid" id="log_stats_grid" style="margin-bottom:16px"></div>

      <!-- Log Chart -->
      <div class="chart-box">
        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">
          <b>📈 Графік напруги та струму за весь сеанс</b>
          <div style="font-size:12px;color:var(--dim)">Напруга банку (зелена) та струм (блакитна)</div>
        </div>
        <canvas id="logChart" width="900" height="260"></canvas>
      </div>

      <!-- Cell Analysis Table -->
      <div class="card" style="background:#161b22;padding:0;overflow-x:auto">
        <div style="padding:14px 16px;border-bottom:1px solid var(--border);font-weight:700">
          🔋 Аналіз стану кожної комірки (15S) за весь період логу
        </div>
        <table class="tbl" id="log_cells_table">
          <thead>
            <tr>
              <th>АКБ</th>
              <th>Комірка</th>
              <th>Min (V)</th>
              <th>Max (V)</th>
              <th>Δ розмах (mV)</th>
              <th>Опір R_int (mOm)</th>
              <th>Балансувань (BAL)</th>
              <th>Оцінка здоров'я</th>
            </tr>
          </thead>
          <tbody id="log_cells_tbody"></tbody>
        </table>
      </div>
    </div>
  </div>
</div>

<!-- COMMON: CSV LOGGING CARD -->
<div class="card" style="margin-top:16px">
  <h2>📝 Запис CSV-лога (окремий файл для кожної АКБ)</h2>
  <div style="display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:12px">
    <div>Файлів у записі: <b id="logpath">—</b> · Записано рядків всього: <span id="logrows">0</span></div>
    <button id="logbtn" onclick="toggleLog()">Старт запису (усі АКБ)</button>
  </div>

  <div class="log-profiles">
    <div class="log-profiles-title">Профілі запису <span style="font-weight:400;color:var(--dim)">(можна увімкнути обидва одночасно)</span></div>
    <label class="log-profile-row">
      <input type="checkbox" id="cfg_alarm" onchange="saveLogCfg()">
      <span><b>🚨 Лише аварійні події</b> — регістри 0025–0028 (статус, попередження, захист, помилка) + температура (поріг 0…45&nbsp;°C).
        Запис виконується лише при зміні стану: поява, зміна або зняття аварії. Без змін — файл не пишеться.</span>
    </label>
    <label class="log-profile-row">
      <input type="checkbox" id="cfg_work" onchange="saveLogCfg()">
      <span><b>🔬 Аналіз / тест комірок</b> — періодичний запис повної телеметрії з інтервалом
        <input type="number" id="cfg_interval" min="1" max="3600" step="1" value="5" onchange="saveLogCfg()"> с.</span>
    </label>
    <div class="log-profiles-note">
      Колонка <b>Log</b> у файлі позначає тип рядка: <b>Alarm</b> або <b>Work</b>.
      Аварійні записи мають пріоритет: якщо в циклі є аварія, плановий запис Work переноситься (без дублювання рядків).
      Обидва профілі пишуть в один файл кожної АКБ, тому колізій немає.
    </div>
  </div>

  <table class="kv" id="logfiles" style="margin-top:10px"></table>
</div>

<div class="foot">
  <b>Режими роботи:</b><br>
  • <b>⚡ Активний (Master):</b> скрипт опитує знайдені АКБ, розраховує автономність, внутрішній опір комірок та веде історію.<br>
  • <b>👂 Пасивний (Sniffer):</b> скрипт не передає жодного байту у шину RS485 (нуль колізій). Безпечно працює при живій системі.<br>
  <b>Безпека:</b> 100% Read-Only (жодного запису в конфігураційні регістри BMS).
</div>

<!-- INVERTER SETTINGS MODAL -->
<div id="inverter_modal" class="modal-overlay">
  <div class="modal-box" id="inverter_modal_content"></div>
</div>

<!-- ALARM & PROTECTION HELPER MODAL -->
<div id="alarm_modal" class="modal-overlay">
  <div class="modal-box" id="alarm_modal_content"></div>
</div>

<!-- DIAGNOSTIC REPORT MODAL -->
<div id="report_modal">
  <div class="report-actions">
    <button onclick="window.print()" style="background:#238636;color:#fff">🖨️ Друк / Зберегти як PDF</button>
    <button onclick="closeReport()" style="background:#555;color:#fff">✕ Закрити</button>
  </div>
  <div class="report-paper" id="report_content">
    <!-- Populated by JS -->
  </div>
</div>

<script>
let currentTab = 'summary';
let currentPackAddr = null;
let logOn = false;
let latestData = null;
let logAnalysisData = null;

const ALARM_HELP_DB = {
  // Statuses
  "Charging": {
    cat: "Status (0025)",
    title: "Йде заряд (Charging)",
    level: "ok",
    levelName: "Штатний режим",
    desc: "Акумулятор отримує енергію від інвертора або зарядного пристрою (струм > 0 А). Силовий зарядний ключ відкритий.",
    action: "Штатний режим. Контролюйте напругу комірок наприкінці заряду."
  },
  "Discharging": {
    cat: "Status (0025)",
    title: "Йде розряд (Discharging)",
    level: "ok",
    levelName: "Штатний режим",
    desc: "Акумулятор віддає енергію в навантаження (струм < 0 А). Силовий розрядний ключ відкритий.",
    action: "Штатний режим. Забезпечте підзарядку при досягненні 10-15% SOC."
  },
  "Protect": {
    cat: "Status (0025)",
    title: "Спрацював захист BMS (Protect)",
    level: "bad",
    levelName: "Аварія / Захист",
    desc: "BMS заблокувала силові ключі через вихід одного або кількох параметрів (напруга, струм або температура) за критичні межі.",
    action: "Перевірте блоки Warning та Protection нижче. Усуньте першопричину (перенапругу або перевантаження по струму)."
  },
  "Charging Lmt": {
    cat: "Status (0025)",
    title: "Обмеження струму заряду (Charging Limit)",
    level: "warn",
    levelName: "Попередження",
    desc: "BMS обмежує допустимий струм заряду для захисту комірок від перегріву або передчасного перезаряду.",
    action: "Інвертор повинен автоматично знизити струм заряду згідно з обмеженнями BMS."
  },
  "Standby": {
    cat: "Status (0025)",
    title: "Режим очікування (Standby)",
    level: "ok",
    levelName: "Очікування",
    desc: "Струм навантаження близький до 0 А. Батарея повністю заряджена або очікує підключення навантаження.",
    action: "Штатний стан у буферному режимі. При появі струму розряду автоматично перейде в Discharging."
  },

  // Warnings (0026)
  "Pack OV": {
    cat: "Warning (0026)",
    title: "Загальна перенапруга пака (Pack Over Voltage)",
    level: "warn",
    levelName: "Попередження",
    desc: "Загальна напруга 15S батареї перевищила заводський поріг попередження (> 54.0 V).",
    action: "Знизьте напругу Bulk/Absorption на інверторі до рекомендованих 52.5 V (3.50 V / ком)."
  },
  "Cell OV": {
    cat: "Warning (0026)",
    title: "Перенапруга комірки (Cell Over Voltage Warning)",
    level: "warn",
    levelName: "Попередження",
    desc: "Напруга хоча б однієї комірки перевищила заводський поріг 3570 mV. Найчастіше трапляється наприкінці швидкого заряду через розбаланс.",
    action: "Знизьте напругу Bulk на інверторі до 52.5 V або увімкніть балансування (DIP 4 на COM-Box). Скидається автоматично, коли напруга опускається нижче 3550 mV."
  },
  "Pack UV": {
    cat: "Warning (0026)",
    title: "Знижена напруга пака (Pack Under Voltage)",
    level: "warn",
    levelName: "Попередження",
    desc: "Загальна напруга впала нижче 43.5 V (глибокий розряд).",
    action: "Негайно розпочніть заряджання акумулятора, щоб уникнути деградації хімії LiFePO4."
  },
  "Cell UV": {
    cat: "Warning (0026)",
    title: "Знижена напруга комірки (Cell Under Voltage)",
    level: "warn",
    levelName: "Попередження",
    desc: "Напруга найслабшої комірки впала нижче безпечного рівня (< 2850 mV).",
    action: "Зупиніть розряд та почніть заряд помірним струмом (0.2C)."
  },
  "Charge OC": {
    cat: "Warning (0026)",
    title: "Перевантаження по струму заряду (Charge Over Current)",
    level: "warn",
    levelName: "Попередження",
    desc: "Струм заряду перевищив налаштоване обмеження (зазвичай > 100 А на один пак).",
    action: "Обмежте максимальний струм заряду на інверторі або розподіліть струм між паралельними батареями."
  },
  "Discharge OC": {
    cat: "Warning (0026)",
    title: "Перевантаження по струму розряду (Discharge Over Current)",
    level: "warn",
    levelName: "Попередження",
    desc: "Споживання навантаження перевищує номінальний струм віддачі акумулятора.",
    action: "Знизьте потужність підключених споживачів."
  },
  "Temp anomaly": {
    cat: "Warning (0026)",
    title: "Аномальний градієнт температур (Temp Anomaly)",
    level: "warn",
    levelName: "Попередження",
    desc: "Різниця між датчиками температури всередині пака перевищила допустиму дельту (> 10°C).",
    action: "Перевірте рівномірність вентиляції та відсутність локального нагріву контактів шин."
  },
  "MOS OT": {
    cat: "Warning (0026)",
    title: "Перегрів силових ключів (MOS Over Temperature)",
    level: "bad",
    levelName: "Аварія / Перегрів",
    desc: "Температура силових MOSFET-ключів BMS перевищила 85°C під високим навантаженням.",
    action: "Знизьте струм навантаження та забезпечте циркуляцію повітря навколо батарейного блоку."
  },
  "Charge OT": {
    cat: "Warning (0026)",
    title: "Перегрів при заряді (Charge Over Temperature)",
    level: "bad",
    levelName: "Аварія / Перегрів",
    desc: "Температура комірок під час заряду перевищила 55°C.",
    action: "Заряд припиняється автоматично до охолодження батареї до безпечної температури (< 45°C)."
  },
  "Discharge OT": {
    cat: "Warning (0026)",
    title: "Перегрів при розряді (Discharge Over Temperature)",
    level: "bad",
    levelName: "Аварія / Перегрів",
    desc: "Температура комірок під час розряду перевищила 65°C.",
    action: "Знизьте навантаження і дайте блоку охолонути."
  },
  "Charge UT": {
    cat: "Warning (0026)",
    title: "Заряд при низькій температурі (Charge Under Temp)",
    level: "bad",
    levelName: "Критична заборона",
    desc: "Температура батареї нижче 0°C. Заряджати LiFePO4 при мінусовій температурі суворо заборонено через ризик випадіння металевого літію.",
    action: "Увімкніть підігрів або прогрійте приміщення до плюсової температури (+10°C) перед початком заряду."
  },
  "Discharge UT": {
    cat: "Warning (0026)",
    title: "Розряд при низькій температурі (Discharge Under Temp)",
    level: "warn",
    levelName: "Попередження",
    desc: "Температура комірок нижче -20°C.",
    action: "Експлуатація при сильному морозі знижує корисну ємність і збільшує внутрішній опір."
  },
  "Low capacity": {
    cat: "Protection (0027) / Warning (0026)",
    title: "Низька ємність / Standby Latch (Low capacity)",
    level: "ok",
    levelName: "Інформаційний стан",
    desc: "Заводський прапорець Vision BMS (біт 0x1000). Встановлюється в парі з переходом у Standby, коли струм зарядки падає до нуля і батарея виходить на верхню полицю напруги, або при вичерпанні залишку ємності.",
    action: "Це нормальний штатний стан після закінчення заряду. Прапорець знімається автоматично під час першого ж розряду струмом від 1-2 А."
  },
  "Float stopped": {
    cat: "Warning (0026)",
    title: "Зупинка буферного режиму (Float Stopped)",
    level: "ok",
    levelName: "Інформаційний стан",
    desc: "Буферний режим підзарядки завершено або тимчасово призупинено за таймером BMS.",
    action: "Штатний процес."
  },

  // Protections & Errors (0027 / 0028)
  "Discharge SC": {
    cat: "Protection (0027)",
    title: "Коротке замикання на виході (Discharge Short Circuit)",
    level: "bad",
    levelName: "Аварія / КЗ",
    desc: "BMS миттєво відсікла розрядний ключ через виявлення короткого замикання або надструму.",
    action: "Вимкніть навантаження, перевірте кабелі підключення до інвертора на предмет КЗ. Скидання: вимкнути живлення або підключити зарядний пристрій."
  },
  "Reverse": {
    cat: "Protection (0027)",
    title: "Захист від переполюсовки (Reverse Polarity)",
    level: "bad",
    levelName: "Критична аварія",
    desc: "Виявлено неправильну полярність підключення силових клем (+/-).",
    action: "Негайно перевірте полярність підключення кабелів!"
  },
  "Voltage error": {
    cat: "Error (0028)",
    title: "Апаратна помилка вимірювання напруги (Voltage ADC Error)",
    level: "bad",
    levelName: "Апаратна несправність",
    desc: "Контролер BMS не може зчитати покази одного з вимірювальних каналів напруги комірок.",
    action: "Перевірте надійність шлейфа балансувальних проводів від комірок до роз'єму плати BMS."
  },
  "Temperature error": {
    cat: "Error (0028)",
    title: "Помилка датчика температури (NTC Error)",
    level: "bad",
    levelName: "Апаратна несправність",
    desc: "Обрив або коротке замикання одного з температурних сенсорів (T1, T2, MOS).",
    action: "Перевірте роз'єми датчиків NTC на платі BMS."
  },
  "Current sensor": {
    cat: "Error (0028)",
    title: "Помилка датчика струму (Current Sensor Error)",
    level: "bad",
    levelName: "Апаратна несправність",
    desc: "Збій вимірювального шунта або підсилювача струму BMS.",
    action: "Перевірте силові з'єднання біля струмового шунта на платі BMS."
  },
  "Cell unbalance": {
    cat: "Error (0028)",
    title: "Критичний розбаланс комірок (Severe Cell Unbalance)",
    level: "bad",
    levelName: "Критичний розбаланс",
    desc: "Різниця між найвищою та найнижчою коміркою всередині батареї перевищила аварійний поріг (> 300 mV).",
    action: "Необхідно провести вирівнювання: увімкніть балансування в COM-Box (DIP 4) та встановіть невеликий струм буферного заряду."
  }
};

function showAlarmHelp(name) {
  const item = ALARM_HELP_DB[name];
  if (!item) return;
  const badgeCls = item.level === 'ok' ? 'ok' : (item.level === 'warn' ? 'warn' : 'bad');
  let html = `
    <div style="display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:14px">
      <div>
        <span class="pill ${badgeCls}">${item.levelName}</span>
        <span style="color:var(--dim);font-size:12px;margin-left:8px">${item.cat}</span>
        <h2 style="margin:8px 0 0;font-size:18px">${item.title}</h2>
      </div>
      <button class="modal-close-btn" onclick="closeAlarmModal()">✕ Закрити</button>
    </div>
    <div style="background:#0d1117;border:1px solid var(--border);border-radius:8px;padding:14px;margin-bottom:12px;font-size:13px;line-height:1.6">
      <b style="color:var(--text)">Що це означає:</b><br>${item.desc}
    </div>
    <div style="background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;font-size:13px;line-height:1.6">
      <b style="color:var(--acc)">💡 Що робити / Як скинути:</b><br>${item.action}
    </div>
  `;
  document.getElementById('alarm_modal_content').innerHTML = html;
  document.getElementById('alarm_modal').style.display = 'block';
}

function closeAlarmModal() {
  document.getElementById('alarm_modal').style.display = 'none';
}

function leds(el, arr, cls) {
  if (!el || !arr) return;
  el.innerHTML = arr.map(b => {
    const isClickable = !!ALARM_HELP_DB[b.name];
    const clickAttr = isClickable ? `onclick="showAlarmHelp('${b.name}')" title="Клікніть для детального опису та інструкції" style="cursor:pointer"` : '';
    return `<div class="led ${b.on ? cls : ''}" ${clickAttr}><i></i>${b.name}${isClickable ? ' <span style="opacity:0.5;font-size:9px">ℹ️</span>' : ''}</div>`;
  }).join('');
}

function statusBitOn(d, name) {
  if (!d || !Array.isArray(d.status_bits)) return false;
  const b = d.status_bits.find(x => x.name === name);
  return !!(b && b.on);
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
    📊 Загальний огляд <span class="tab-badge">${latestData.summary.online_packs} АКБ</span>
  </button>
  <button class="tab-btn ${currentTab==='charts'?'active':''}" onclick="selectTab('charts')">
    📈 Графіки та Діагностика
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

  html += `<button class="tab-btn ${currentTab==='logviewer'?'active':''}" onclick="selectTab('logviewer')">
    📂 Аналіз логу
  </button>`;

  tb.innerHTML = html;
}

function renderView() {
  if (!latestData) return;
  document.getElementById('view_summary').style.display = (currentTab === 'summary') ? 'block' : 'none';
  document.getElementById('view_charts').style.display = (currentTab === 'charts') ? 'block' : 'none';
  document.getElementById('view_logviewer').style.display = (currentTab === 'logviewer') ? 'block' : 'none';
  document.getElementById('view_pack').style.display = (currentTab !== 'summary' && currentTab !== 'charts' && currentTab !== 'logviewer') ? 'block' : 'none';

  if (currentTab === 'summary') {
    renderSummary();
  } else if (currentTab === 'charts') {
    renderChartsView();
    drawHistoryChart();
    drawDeltaChart();
  } else if (currentTab === 'logviewer') {
    if (logAnalysisData) {
      drawLogChart();
    }
  } else {
    renderPackDetail(currentTab);
    drawCellBarChart(currentTab);
  }
}

function renderSummary() {
  const sum = latestData.summary;
  const packs = latestData.packs || {};

  document.getElementById('b_v').textContent = sum.avg_voltage ? (sum.avg_voltage.toFixed(2) + ' V') : '—';
  
  const currSign = sum.total_current > 0 ? '+' : '';
  document.getElementById('b_i').textContent = (sum.total_current != null ? (currSign + sum.total_current.toFixed(1) + ' А') : '—');
  document.getElementById('b_p').textContent = (sum.total_power_kw != null ? (currSign + sum.total_power_kw.toFixed(2) + ' кВт') : '—');
  
  const realSocStr = sum.avg_real_soc != null ? (sum.avg_real_soc.toFixed(1) + ' %') : (sum.avg_soc ? (sum.avg_soc + ' %') : '—');
  document.getElementById('b_soc').textContent = realSocStr;
  const bmsNote = (sum.avg_soc != null && sum.avg_real_soc != null) ? ` · BMS: ${sum.avg_soc}%` : '';
  document.getElementById('b_cap').textContent = 'Залишок: ' + (sum.total_cap_remaining ? sum.total_cap_remaining.toFixed(1) + ' А·ч' : '—') + bmsNote;

  document.getElementById('b_diff').textContent = (sum.cell_delta_mv != null ? ('Δ = ' + sum.cell_delta_mv + ' mV') : '—');
  if (sum.min_cell_mv && sum.max_cell_mv) {
    document.getElementById('b_extremes').textContent = `Min: ${(sum.min_cell_mv / 1000).toFixed(3)} V (${sum.min_cell_pack} #${sum.min_cell_idx}) | Max: ${(sum.max_cell_mv / 1000).toFixed(3)} V (${sum.max_cell_pack} #${sum.max_cell_idx})`;
  } else {
    document.getElementById('b_extremes').textContent = 'Min: — | Max: —';
  }

  const balEl = document.getElementById('b_status');
  balEl.textContent = sum.any_balancing ? 'АКТИВНЕ БАЛАНСУВАННЯ' : (sum.online_packs > 0 ? 'Працює в нормі' : 'АКБ відсутні');
  balEl.style.color = sum.any_balancing ? 'var(--warn)' : (sum.online_packs > 0 ? 'var(--ok)' : 'var(--bad)');

  const alEl = document.getElementById('b_alarm_status');
  alEl.textContent = sum.alarm_count > 0 ? `⚠️ Активних тривог: ${sum.alarm_count}` : 'Усі захисти та попередження в нормі';
  alEl.style.color = sum.alarm_count > 0 ? 'var(--bad)' : 'var(--dim)';

  // Summary Table
  const tbody = document.getElementById('summary_rows');
  const addrs = Object.keys(packs).sort();
  if (addrs.length === 0) {
    tbody.innerHTML = `<tr><td colspan="12" style="text-align:center;color:var(--dim);padding:20px">Жодного АКБ не знайдено.</td></tr>`;
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

    const stClass = on ? (statusBitOn(d, 'Protect') ? 'bad' : 'ok') : 'bad';
    const balActive = d.balance_bits && d.balance_bits.some(b => b.on);
    const hasAlarm = (d.warning || d.protection || d.error_code);
    const currStr = (d.current >= 0 ? '+' : '') + d.current.toFixed(1) + ' А';

    return `<tr>
      <td><b>${addr}</b> ${on ? '<span style="color:var(--ok)">●</span>' : '<span style="color:var(--bad)">○</span>'}</td>
      <td><span class="pill ${stClass}">${d.status_name}</span></td>
      <td><b>${d.voltage.toFixed(2)} V</b></td>
      <td>${currStr}</td>
      <td><b>${d.real_soc != null ? d.real_soc.toFixed(1) + '%' : (d.soc != null ? d.soc + '%' : '—')}</b>${d.soc != null && d.real_soc != null ? ` <span style="color:var(--dim);font-size:11px" title="Заводський BMS SOC">(${d.soc}%)</span>` : ''}</td>
      <td style="color:var(--bad)">${(d.cell_min / 1000).toFixed(3)} V</td>
      <td style="color:var(--ok)">${(d.cell_max / 1000).toFixed(3)} V</td>
      <td><b>${d.cell_diff} mV</b></td>
      <td>${balActive ? '<span class="b-tag">BAL</span>' : '—'}</td>
      <td>${d.temp_pcb}/${d.temp_avg}/${d.temp_max} °C</td>
      <td>${hasAlarm ? '<span class="pill bad">Увага</span>' : '<span class="pill ok">Норма</span>'}</td>
      <td><button onclick="selectTab('${addr}')" style="padding:4px 10px;font-size:12px">Детальніше →</button></td>
    </tr>`;
  }).join('');
}

function renderChartsView() {
  const sum = latestData.summary || {};
  const aut = sum.autonomy || {};
  const diag = latestData.diagnostics || {};
  const packs = latestData.packs || {};

  document.getElementById('c_energy').textContent = aut.energy_kwh != null ? (aut.energy_kwh.toFixed(2) + ' кВт·год (' + sum.avg_soc + '%)') : '—';
  document.getElementById('c_eta').textContent = aut.eta_str || '—';
  const cSign = sum.total_current >= 0 ? '+' : '';
  const pSign = sum.total_power_kw >= 0 ? '+' : '';
  document.getElementById('c_state').textContent = 'Режим: ' + (aut.state_label || '—') + (sum.total_current != null ? (' (' + cSign + sum.total_current.toFixed(1) + ' А / ' + pSign + sum.total_power_kw.toFixed(2) + ' кВт)') : '');

  document.getElementById('diag_weak').textContent = diag.weak_cell || '—';
  document.getElementById('diag_rint').textContent = diag.rint_summary || '—';

  const tbody = document.getElementById('diag_rows');
  if (!tbody) return;
  const addrs = Object.keys(packs).sort();
  if (addrs.length === 0) {
    tbody.innerHTML = `<tr><td colspan="5" style="text-align:center;color:var(--dim);padding:14px">Немає підключених АКБ</td></tr>`;
    return;
  }

  tbody.innerHTML = addrs.map(addr => {
    const p = packs[addr];
    const rints = p.rint || [];
    const balSecs = p.bal_seconds || [];
    const onR = rints.filter(r => r > 0.05);
    const avgR = onR.length ? (onR.reduce((a, b) => a + b, 0) / onR.length).toFixed(2) + ' мОм' : '—';
    const maxR = onR.length ? Math.max(...onR).toFixed(2) + ' мОм' : '—';
    const totalBalS = balSecs.reduce((a, b) => a + b, 0);
    const balStr = totalBalS > 0 ? (totalBalS > 60 ? Math.floor(totalBalS / 60) + ' хв ' + (totalBalS % 60) + ' с' : totalBalS + ' с') : '0 с';
    const isBalNow = p.parsed && p.parsed.balance_bits && p.parsed.balance_bits.some(b => b.on);

    return `<tr>
      <td><b>${addr}</b></td>
      <td style="color:var(--acc)">${avgR}</td>
      <td style="color:var(--acc)">${maxR}</td>
      <td>${isBalNow ? '<span class="b-tag">АКТИВНЕ ЗАРАЗ</span>' : '<span style="color:var(--dim)">Неактивне</span>'}</td>
      <td>${balStr}</td>
    </tr>`;
  }).join('');
}

function renderPackDetail(addr) {
  const packs = latestData.packs || {};
  const p = packs[addr];
  if (!p) return;
  currentPackAddr = addr;

  const d = p.parsed;
  document.getElementById('p_addr').textContent = addr + (p.online ? ' (Онлайн)' : ' (Офлайн)');
  document.getElementById('p_err').textContent = p.err || '';

  // Per-pack CSV logging card (works even when the pack is offline)
  const lp = document.getElementById('p_logpath');
  const lr = document.getElementById('p_logrows');
  const lt = document.getElementById('p_logts');
  const lb = document.getElementById('p_logbtn');
  if (lp) lp.textContent = p.log_path || '—';
  if (lr) lr.textContent = p.log_rows || 0;
  const la = document.getElementById('p_log_alarm');
  const lw = document.getElementById('p_log_work');
  if (la) la.textContent = p.log_alarm_rows || 0;
  if (lw) lw.textContent = p.log_work_rows || 0;
  if (lt) lt.textContent = p.log_last_ts || '—';
  if (lb) {
    lb.textContent = p.log_on ? 'Зупинити запис' : 'Старт запису';
    lb.className = p.log_on ? 'off' : '';
  }

  if (!d) return;

  document.getElementById('p_v').textContent = d.voltage.toFixed(2) + ' V';
  document.getElementById('p_i').textContent = (d.current == null ? '—' : d.current.toFixed(1) + ' А');
  
  const st = document.getElementById('p_st');
  st.textContent = d.status_name;
  st.className = 'pill ' + (statusBitOn(d, 'Protect') ? 'bad' : 'ok');

  document.getElementById('p_model').textContent = d.model || '—';
  document.getElementById('p_fw').textContent = d.fw_version || '—';
  document.getElementById('p_sn').textContent = d.serial || '—';
  const rSoc = d.real_soc != null ? `${d.real_soc.toFixed(1)} % <span style="font-size:11px;color:var(--dim)">(по мін. ком. ${(d.cell_min / 1000).toFixed(3)} V)</span>` : '—';
  document.getElementById('p_real_soc').innerHTML = `<b>${rSoc}</b>`;
  document.getElementById('p_soc').textContent = (d.soc == null ? '—' : d.soc + ' %') + ' / ' + (d.cap_remaining == null ? '—' : d.cap_remaining + ' А·ч');
  document.getElementById('p_soh').textContent = (d.soh == null ? '—' : d.soh + ' %') + ' / ' + (d.max_charge_a == null ? '—' : d.max_charge_a + ' А');
  document.getElementById('p_cyc').textContent = d.cycles == null ? '—' : d.cycles;
  document.getElementById('p_cap').textContent = (d.designed_cap_ah == null ? '—' : d.designed_cap_ah + ' А·ч') + ' · ' + d.cell_num + 'S';
  document.getElementById('p_fcap').textContent = d.full_cap_ah == null ? '—' : d.full_cap_ah.toFixed(1) + ' А·ч';
  document.getElementById('p_difft').textContent = 'Δ = ' + d.cell_diff + ' mV';
  if (d.cell_min != null) document.getElementById('p_vmin').textContent = (d.cell_min / 1000).toFixed(3);
  if (d.cell_max != null) document.getElementById('p_vmax').textContent = (d.cell_max / 1000).toFixed(3);

  // Cells grid
  const rints = p.rint || [];
  const ci = document.getElementById('p_cells');
  ci.innerHTML = d.cells.map((mv, i) => {
    let cls = 'cell';
    if (mv === d.cell_min) cls += ' min';
    if (mv === d.cell_max) cls += ' max';
    const balOn = d.balance_bits && d.balance_bits[i] && d.balance_bits[i].on;
    if (balOn) cls += ' bal-active';
    const rVal = rints[i] ? rints[i] + ' mΩ' : '';
    return `<div class="${cls}">
      <div class="n">#${i + 1}${balOn ? ' <span class="b-tag">BAL</span>' : ''}</div>
      <div class="mv" title="${mv} mV">${(mv / 1000).toFixed(3)} V</div>
      ${rVal ? `<div class="rint">${rVal}</div>` : ''}
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

/* --- CANVAS CHART 1: ROLLING HISTORY (VOLTAGE & CURRENT) --- */
function drawHistoryChart() {
  const canvas = document.getElementById('historyChart');
  if (!canvas || !latestData || !latestData.history) return;
  const ctx = canvas.getContext('2d');
  const hist = latestData.history;
  const w = canvas.width, h = canvas.height;

  ctx.clearRect(0, 0, w, h);
  if (hist.length < 2) {
    ctx.fillStyle = '#7d8894';
    ctx.font = '13px sans-serif';
    ctx.fillText('Накопичення точок історії напруги та струму...', 20, 30);
    return;
  }

  const padL = 40, padR = 40, padT = 30, padB = 25;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  // Find min/max for voltage & current
  const vArr = hist.map(p => p.v);
  const iArr = hist.map(p => p.i);

  let minV = Math.min(...vArr) - 0.5, maxV = Math.max(...vArr) + 0.5;
  if (maxV - minV < 2.0) { minV -= 1.0; maxV += 1.0; }

  let maxAbsI = Math.max(...iArr.map(Math.abs), 5.0);
  let minI = -maxAbsI, maxI = maxAbsI;

  // Draw legend with live metrics
  const lastPt = hist[hist.length - 1] || {};
  const lastV = (lastPt.v != null ? Number(lastPt.v).toFixed(2) : '—');
  const lastI = (lastPt.i != null ? ((lastPt.i >= 0 ? '+' : '') + Number(lastPt.i).toFixed(1)) : '—');

  ctx.font = '11px sans-serif';
  ctx.fillStyle = '#58a6ff'; ctx.fillText('■ Напруга: ' + lastV + ' V', 20, 18);
  ctx.fillStyle = '#3fb950'; ctx.fillText('■ Струм: ' + lastI + ' А', 160, 18);

  // Grid
  ctx.strokeStyle = '#252c34';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let s = 0; s <= 4; s++) {
    const y = padT + (plotH / 4) * s;
    ctx.moveTo(padL, y); ctx.lineTo(w - padR, y);
    // V label left
    const vVal = (maxV - ((maxV - minV) / 4) * s).toFixed(1);
    ctx.fillStyle = '#7d8894';
    ctx.fillText(vVal + 'V', 5, y + 4);
    // I label right
    const iVal = (maxI - ((maxI - minI) / 4) * s).toFixed(0);
    ctx.fillText(iVal + 'A', w - padR + 5, y + 4);
  }
  ctx.stroke();

  // Zero Current Line
  const yZeroI = padT + plotH * (maxI / (maxI - minI));
  ctx.strokeStyle = '#30363d';
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(padL, yZeroI); ctx.lineTo(w - padR, yZeroI);
  ctx.stroke();
  ctx.setLineDash([]);

  // Plot Voltage Line (blue #58a6ff)
  ctx.strokeStyle = '#58a6ff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  hist.forEach((pt, idx) => {
    const x = padL + (idx / (hist.length - 1)) * plotW;
    const y = padT + plotH - ((pt.v - minV) / (maxV - minV)) * plotH;
    if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Plot Current Line (green #3fb950)
  ctx.strokeStyle = '#3fb950';
  ctx.lineWidth = 1.8;
  ctx.beginPath();
  hist.forEach((pt, idx) => {
    const x = padL + (idx / (hist.length - 1)) * plotW;
    const y = padT + plotH - ((pt.i - minI) / (maxI - minI)) * plotH;
    if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

/* --- CANVAS CHART 2: CELL DELTA HISTORY --- */
function drawDeltaChart() {
  const canvas = document.getElementById('deltaChart');
  if (!canvas || !latestData || !latestData.history) return;
  const ctx = canvas.getContext('2d');
  const hist = latestData.history;
  const w = canvas.width, h = canvas.height;

  ctx.clearRect(0, 0, w, h);
  if (hist.length < 2) {
    ctx.fillStyle = '#7d8894';
    ctx.font = '13px sans-serif';
    ctx.fillText('Накопичення точок історії розбалансу...', 20, 30);
    return;
  }

  const padL = 45, padR = 40, padT = 30, padB = 25;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  const dArr = hist.map(p => (p.diff != null ? p.diff : 0));
  let maxD = Math.max(...dArr, 25);
  maxD = Math.max(60, Math.ceil(maxD / 30) * 30);

  const lastPt = hist[hist.length - 1] || {};
  const lastD = (lastPt.diff != null ? Math.round(lastPt.diff) : 0);

  let statusCol = '#3fb950', statusTxt = '🟢 Відмінно (< 30 mV)';
  if (lastD > 50) {
    statusCol = '#f85149';
    statusTxt = '🔴 Рекомендовано балансування (> 50 mV)';
  } else if (lastD >= 30) {
    statusCol = '#d29922';
    statusTxt = '🟡 Робоча норма (30..50 mV)';
  }

  // Draw legend
  ctx.font = '11px sans-serif';
  ctx.fillStyle = '#d29922'; ctx.fillText('■ Δ розбалансу банку: ' + lastD + ' mV', 20, 18);
  ctx.fillStyle = statusCol; ctx.fillText('Стан: ' + statusTxt, 210, 18);

  // Grid (3 intervals)
  ctx.strokeStyle = '#252c34';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let s = 0; s <= 3; s++) {
    const y = padT + (plotH / 3) * s;
    ctx.moveTo(padL, y); ctx.lineTo(w - padR, y);
    const dVal = Math.round(maxD - (maxD / 3) * s);
    ctx.fillStyle = '#7d8894';
    ctx.fillText(dVal + ' mV', 5, y + 4);
  }
  ctx.stroke();

  // Balance threshold line (50 mV)
  const thresh = 50;
  if (thresh < maxD) {
    const yThresh = padT + plotH - (thresh / maxD) * plotH;
    ctx.strokeStyle = 'rgba(248, 81, 73, 0.45)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(padL, yThresh); ctx.lineTo(w - padR, yThresh);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.font = '10px sans-serif';
    ctx.fillStyle = '#f85149';
    ctx.fillText('Поріг балансування (50 mV)', w - padR - 150, yThresh - 4);
  }

  // Plot Delta Line & Area Fill (golden gradient)
  const grad = ctx.createLinearGradient(0, padT, 0, padT + plotH);
  grad.addColorStop(0, 'rgba(210, 153, 34, 0.28)');
  grad.addColorStop(1, 'rgba(210, 153, 34, 0.02)');
  ctx.fillStyle = grad;
  ctx.beginPath();
  hist.forEach((pt, idx) => {
    const x = padL + (idx / (hist.length - 1)) * plotW;
    const dVal = Math.min(maxD, Math.max(0, pt.diff != null ? pt.diff : 0));
    const y = padT + plotH - (dVal / maxD) * plotH;
    if (idx === 0) {
      ctx.moveTo(x, padT + plotH);
      ctx.lineTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
  });
  ctx.lineTo(padL + plotW, padT + plotH);
  ctx.closePath();
  ctx.fill();

  ctx.strokeStyle = '#d29922';
  ctx.lineWidth = 2;
  ctx.beginPath();
  hist.forEach((pt, idx) => {
    const x = padL + (idx / (hist.length - 1)) * plotW;
    const dVal = Math.min(maxD, Math.max(0, pt.diff != null ? pt.diff : 0));
    const y = padT + plotH - (dVal / maxD) * plotH;
    if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Highlight latest point with dot
  if (hist.length > 0) {
    const x = padL + plotW;
    const dVal = Math.min(maxD, Math.max(0, lastPt.diff != null ? lastPt.diff : 0));
    const y = padT + plotH - (dVal / maxD) * plotH;
    ctx.fillStyle = '#d29922';
    ctx.beginPath();
    ctx.arc(x, y, 3.5, 0, 2 * Math.PI);
    ctx.fill();
  }
}

/* --- CANVAS CHART 2: CELL BAR CHART --- */
function drawCellBarChart(addr) {
  const canvas = document.getElementById('cellBarChart');
  if (!canvas || !latestData) return;
  const p = (latestData.packs || {})[addr];
  if (!p || !p.parsed) return;
  const ctx = canvas.getContext('2d');
  const cells = p.parsed.cells || [];
  const rints = p.rint || [];
  const balBits = p.parsed.balance_bits || [];
  const w = canvas.width, h = canvas.height;

  ctx.clearRect(0, 0, w, h);
  if (cells.length === 0) return;

  const padL = 40, padR = 20, padT = 25, padB = 35;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  const minVal = p.parsed.cell_min;
  const maxVal = p.parsed.cell_max;
  const avgVal = cells.reduce((a, b) => a + b, 0) / cells.length;
  let chartMin = Math.min(...cells) - 20;
  let chartMax = Math.max(...cells) + 20;
  if (chartMax - chartMin < 60) { chartMin -= 20; chartMax += 20; }

  const barW = Math.floor(plotW / cells.length) - 8;

  // Grid
  ctx.strokeStyle = '#252c34';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let s = 0; s <= 3; s++) {
    const y = padT + (plotH / 3) * s;
    ctx.moveTo(padL, y); ctx.lineTo(w - padR, y);
    const val = Math.round(chartMax - ((chartMax - chartMin) / 3) * s);
    ctx.fillStyle = '#7d8894';
    ctx.font = '10px sans-serif';
    ctx.fillText((val / 1000).toFixed(2) + ' V', 5, y + 3);
  }
  ctx.stroke();

  // Average line
  const yAvg = padT + plotH - ((avgVal - chartMin) / (chartMax - chartMin)) * plotH;
  ctx.strokeStyle = '#7d8894';
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.moveTo(padL, yAvg); ctx.lineTo(w - padR, yAvg);
  ctx.stroke();
  ctx.setLineDash([]);

  // Bars
  cells.forEach((mv, i) => {
    const x = padL + i * (plotW / cells.length) + 4;
    const barH = ((mv - chartMin) / (chartMax - chartMin)) * plotH;
    const y = padT + plotH - barH;

    const isMin = (mv === minVal);
    const isMax = (mv === maxVal);
    const isBal = balBits[i] && balBits[i].on;

    if (isBal) {
      ctx.fillStyle = '#d29922'; // golden amber
    } else if (isMin) {
      ctx.fillStyle = '#f85149'; // red min
    } else if (isMax) {
      ctx.fillStyle = '#3fb950'; // green max
    } else {
      ctx.fillStyle = '#238636';
    }

    ctx.fillRect(x, y, barW, barH);

    // Cell label
    ctx.fillStyle = '#d8dee4';
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('#' + (i + 1), x + barW / 2, h - 20);

    // R_int label under bar
    if (rints[i] > 0) {
      ctx.fillStyle = '#58a6ff';
      ctx.fillText(rints[i] + 'mΩ', x + barW / 2, h - 8);
    }

    // BAL badge text on top
    if (isBal) {
      ctx.fillStyle = '#d29922';
      ctx.font = 'bold 9px sans-serif';
      ctx.fillText('BAL', x + barW / 2, y - 5);
    }
  });
  ctx.textAlign = 'left';
}

/* --- DIAGNOSTIC PASSPORT REPORT --- */
function openReport() {
  if (!latestData) return;
  const d = latestData;
  const sum = d.summary;
  const aut = sum.autonomy || {};
  const diag = d.diagnostics || {};
  const packs = d.packs || {};
  const addrs = Object.keys(packs).sort();

  let html = `<h2>Паспорт-звіт діагностики стану акумуляторного банку Vision</h2>
  <div style="color:#666;font-size:12px;margin-bottom:14px">Дата формування: ${d.ts || new Date().toLocaleString()} · Порт: ${d.port}</div>

  <div class="rep-sec">1. Загальний енергетичний статус банку</div>
  <table>
    <tr><th>Підключено АКБ</th><th>Загальна напруга</th><th>Сумарний струм</th><th>Потужність</th><th>Середній SOC (OCV)</th><th>Залишок енергії</th><th>Макс. розбаланс банку</th></tr>
    <tr>
      <td><b>${sum.online_packs} з ${sum.total_packs}</b></td>
      <td>${sum.avg_voltage.toFixed(2)} V</td>
      <td>${sum.total_current >= 0 ? '+' : ''}${sum.total_current.toFixed(1)} А</td>
      <td>${sum.total_power_kw.toFixed(2)} кВт</td>
      <td><b>${sum.avg_real_soc != null ? sum.avg_real_soc.toFixed(1) + ' %' : (sum.avg_soc + ' %')}</b>${sum.avg_soc != null && sum.avg_real_soc != null ? ` <span style="font-size:11px;color:#666">(${sum.avg_soc}% BMS)</span>` : ''}</td>
      <td><b>${aut.energy_kwh} кВт·год</b> (${aut.eta_str})</td>
      <td><b style="color:${sum.cell_delta_mv > 50 ? '#c00' : '#000'}">Δ = ${sum.cell_delta_mv} mV</b></td>
    </tr>
  </table>

  <div class="rep-sec">2. Паспортні дані акумуляторів (BMS Info)</div>
  <table>
    <tr><th>Адреса</th><th>Модель</th><th>Прошивка BMS</th><th>Серійний номер</th><th>SOC (OCV / BMS)</th><th>SOH</th><th>Цикли</th><th>Номінал</th><th>Стан</th></tr>`;

  addrs.forEach(a => {
    const p = packs[a];
    const parsed = p.parsed || {};
    html += `<tr>
      <td><b>${a}</b></td>
      <td>${parsed.model || '—'}</td>
      <td>${parsed.fw_version || '—'}</td>
      <td>${parsed.serial || '—'}</td>
      <td><b>${parsed.real_soc != null ? parsed.real_soc.toFixed(1) + '%' : '—'}</b> / <span style="color:#666">${parsed.soc != null ? parsed.soc + '%' : '—'}</span></td>
      <td>${parsed.soh != null ? parsed.soh + '%' : '—'}</td>
      <td>${parsed.cycles != null ? parsed.cycles : '—'}</td>
      <td>${parsed.designed_cap_ah != null ? parsed.designed_cap_ah + ' А·ч' : '—'}</td>
      <td>${p.online ? parsed.status_name : 'Офлайн'}</td>
    </tr>`;
  });
  html += `</table>
  <div class="rep-sec">3. Матриця напруг 15S комірок (V), опір та балансування</div>
  <div style="overflow-x:auto"><table class="rep-cells-table">
    <tr><th>АКБ</th><th>#1</th><th>#2</th><th>#3</th><th>#4</th><th>#5</th><th>#6</th><th>#7</th><th>#8</th><th>#9</th><th>#10</th><th>#11</th><th>#12</th><th>#13</th><th>#14</th><th>#15</th><th>Δ пака</th></tr>`;

  addrs.forEach(a => {
    const p = packs[a];
    const parsed = p.parsed;
    if (!parsed) return;
    const cells = parsed.cells || [];
    html += `<tr><td><b>${a}</b></td>`;
    cells.forEach((mv, idx) => {
      let style = '';
      if (mv === parsed.cell_min) style = 'background:#ffebee;font-weight:700;color:#c00;';
      if (mv === parsed.cell_max) style = 'background:#e8f5e9;font-weight:700;color:#2e7d32;';
      html += `<td style="${style}">${(mv / 1000).toFixed(3)}</td>`;
    });
    html += `<td><b>${parsed.cell_diff} mV</b></td></tr>`;
  });
  html += `</table></div>`;

  html += `<div class="rep-sec">4. Висновок діагностики та рекомендації</div>
  <div style="background:#f9f9f9;padding:12px;border:1px solid #ddd;border-radius:6px;font-size:12px;line-height:1.6">
    • <b>Стан розбалансу:</b> ${diag.weak_cell}<br>
    • <b>Оцінка внутрішнього опору:</b> ${diag.rint_summary}<br>
    • <b>Рекомендації:</b> ${sum.cell_delta_mv > 50 ? 'Рекомендується увімкнути режим балансування (DIP 4 на COM-Box) для вирівнювання відстаючих комірок під час буферного заряду.' : 'Банк збалансований, відхилення напруг у межах норми.'}
  </div>`;

  document.getElementById('report_content').innerHTML = html;
  document.getElementById('report_modal').style.display = 'block';
}

function closeReport() {
  document.getElementById('report_modal').style.display = 'none';
}

/* --- INVERTER SETTINGS MODAL --- */
function openInverterModal() {
  const d = latestData;
  const packs = (d && d.packs) ? d.packs : {};
  const onlineAddrs = Object.keys(packs).filter(a => packs[a].online);
  const packCount = onlineAddrs.length || 3;
  
  let cellCount = 15;
  for (const a of onlineAddrs) {
    if (packs[a].parsed && packs[a].parsed.cell_num) {
      cellCount = packs[a].parsed.cell_num;
      break;
    }
  }

  const vBulkSafe = (cellCount * 3.50).toFixed(2);
  const vBulkMax = (cellCount * 3.55).toFixed(2);
  const vFloat = (cellCount * 3.37).toFixed(2);
  const vCutoff = (cellCount * 2.95).toFixed(2);
  const vRestart = (cellCount * 3.30).toFixed(2);

  const iOpt = packCount * 30;
  const iMax = packCount * 50;
  const iDis = packCount * 100;
  const capAh = packCount * 100;
  const kwh = (packCount * (cellCount * 3.2 * 100) / 1000).toFixed(1);

  let html = `
    <div style="display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:14px">
      <div>
        <h2 style="margin:0;font-size:20px;color:var(--fg)">⚙️ Рекомендовані налаштування для інвертора</h2>
        <div style="color:var(--dim);font-size:12px;margin-top:4px">
          Deye · Victron · Luxpower · Growatt · Voltronic для <b>${packCount} АКБ (${cellCount}S LiFePO4, ${capAh} А·ч / ~${kwh} кВт·год)</b>
        </div>
      </div>
      <button class="modal-close-btn" onclick="closeInverterModal()">✕ Закрити</button>
    </div>

    <div style="background:#0d1117;border:1px solid var(--border);border-radius:8px;padding:12px 16px;margin-bottom:16px;font-size:12px;line-height:1.5">
      💡 <b>Чому це важливо:</b> Більшість тривог <i>Warning 0x0002 (Cell OV)</i> виникають, коли на інверторі виставлено завищену напругу заряду (54.0 В або 54.8 V). Для 15S LiFePO4 безпечна та оптимальна напруга заряду становить <b>52.5 V</b> (3.50 V/ком), що забезпечує 98-99% заряду без перенапруги та розбалансу!
    </div>

    <table class="tbl" style="margin-bottom:16px">
      <tr><th>Параметр інвертора</th><th>Значення</th><th>На комірку</th><th>Призначення та коментар</th></tr>
      <tr>
        <td><b>Bulk / Absorption (Заряд)</b></td>
        <td><b style="color:var(--ok);font-size:15px">${vBulkSafe} V</b></td>
        <td>3.50 V</td>
        <td><b>Рекомендований оптимум.</b> Запобігає спрацьовуванню Cell OV 0x0002 та продовжує ресурс комірок.</td>
      </tr>
      <tr>
        <td>Bulk (Макс. ємність 100%)</td>
        <td><b>${vBulkMax} V</b></td>
        <td>3.55 V</td>
        <td>Повний заряд. Використовувати тільки якщо банк ідеально відбалансований (Δ &lt; 30 mV).</td>
      </tr>
      <tr>
        <td><b>Float (Буферний підзаряд)</b></td>
        <td><b style="color:var(--acc);font-size:15px">${vFloat} V</b></td>
        <td>3.37 V</td>
        <td>Тривале перебування під 100% готовністю без нагріву та деградації комірок.</td>
      </tr>
      <tr>
        <td><b>Cut-off (Відключення розряду)</b></td>
        <td><b style="color:var(--bad);font-size:15px">${vCutoff} V</b></td>
        <td>2.95 V</td>
        <td>Захист найслабшої комірки від глибокого перерозряду (залишає ~5% безпечного буфера).</td>
      </tr>
      <tr>
        <td><b>Re-start (Старт нового заряду)</b></td>
        <td><b>${vRestart} V</b></td>
        <td>3.30 V</td>
        <td>Початок нового циклу заряду після переходу на роботу від мережі/сонця.</td>
      </tr>
      <tr>
        <td><b>Рекомендований струм заряду</b></td>
        <td><b style="color:var(--ok)">${iOpt} А</b> (0.3C)</td>
        <td>—</td>
        <td>Оптимальний струм для щоденної тривалої експлуатації (${packCount} × 30 А).</td>
      </tr>
      <tr>
        <td><b>Максимальний струм заряду</b></td>
        <td><b>${iMax} А</b> (0.5C)</td>
        <td>—</td>
        <td>Максимум для швидкого заряду від сонця або генератора (${packCount} × 50 А).</td>
      </tr>
      <tr>
        <td><b>Максимальний струм розряду</b></td>
        <td><b>${iDis} А</b> (1.0C)</td>
        <td>—</td>
        <td>Пікове навантаження всієї паралелі (${packCount} × 100 А).</td>
      </tr>
    </table>

    <div style="display:flex;justify-content:flex-end;gap:10px">
      <button onclick="copyInverterSettings(${packCount}, ${cellCount}, '${vBulkSafe}', '${vFloat}', '${vCutoff}', ${iOpt}, ${iMax}, ${iDis})" style="background:#238636;color:#fff;border:none;padding:8px 16px;border-radius:6px;cursor:pointer;font-weight:600">
        📋 Скопіювати у буфер
      </button>
      <button class="modal-close-btn" onclick="closeInverterModal()">Закрити</button>
    </div>
  `;
  document.getElementById('inverter_modal_content').innerHTML = html;
  document.getElementById('inverter_modal').style.display = 'block';
}

function closeInverterModal() {
  document.getElementById('inverter_modal').style.display = 'none';
}

function copyInverterSettings(packs, cells, bulk, floatV, cutoff, iOpt, iMax, iDis) {
  const text = `=== Налаштування інвертора для Vision BMS (${packs}x АКБ, ${cells}S LiFePO4) ===\n` +
    `Bulk / Absorption: ${bulk} V\n` +
    `Float: ${floatV} V\n` +
    `Low Battery Cut-off: ${cutoff} V\n` +
    `Charge Current (Opt): ${iOpt} A (0.3C)\n` +
    `Charge Current (Max): ${iMax} A (0.5C)\n` +
    `Discharge Current (Max): ${iDis} A (1.0C)\n` +
    `Battery Chemistry: LiFePO4 (User-defined / Lithium)`;
  navigator.clipboard.writeText(text).then(() => {
    alert('Параметри скопійовано в буфер обміну!');
  }).catch(() => {
    alert(text);
  });
}

/* --- OFFLINE CSV LOG VIEWER --- */
function initLogViewer() {
  const dz = document.getElementById('drop_zone');
  if (!dz) return;
  dz.ondragover = e => { e.preventDefault(); dz.classList.add('dragover'); };
  dz.ondragleave = () => dz.classList.remove('dragover');
  dz.ondrop = e => {
    e.preventDefault();
    dz.classList.remove('dragover');
    if (e.dataTransfer.files && e.dataTransfer.files.length) {
      handleCsvFile(e.dataTransfer.files[0]);
    }
  };
}

function handleCsvFile(file) {
  if (!file) return;
  const reader = new FileReader();
  reader.onload = function(e) {
    parseAndAnalyzeCSV(file.name, e.target.result);
  };
  reader.readAsText(file);
}

function resetLogViewer() {
  document.getElementById('log_results').style.display = 'none';
  document.getElementById('drop_zone').style.display = 'block';
  document.getElementById('csv_file_input').value = '';
  logAnalysisData = null;
}

function parseAndAnalyzeCSV(filename, text) {
  const lines = text.split(String.fromCharCode(10)).map(l => l.replace(String.fromCharCode(13), '')).filter(l => l.trim().length > 0);
  if (lines.length < 2) {
    alert('Файл CSV порожній або містить замало рядків');
    return;
  }
  const delim = lines[0].includes(';') ? ';' : ',';
  const header = lines[0].split(delim).map(s => s.trim());
  const newFormat = header.indexOf('Voltage_V') !== -1;

  let addrIdx, tsIdx, cellIndices = [];
  let voltIdx = -1, currIdx = -1, balHexIdx = -1;
  let r00Idx = -1, r01Idx = -1, r38Idx = -1;

  if (newFormat) {
    // New format (ported from bms_webX): Date,Log,addr,Voltage_V,...,Cell_1_V..Cell_16_V,...
    tsIdx = header.indexOf('Date');
    if (tsIdx === -1) tsIdx = 0;
    addrIdx = header.indexOf('addr');
    voltIdx = header.indexOf('Voltage_V');
    currIdx = header.indexOf('Current_A');
    balHexIdx = header.indexOf('Balance_Hex');
    for (let c = 1; c <= 16; c++) {
      const idx = header.indexOf('Cell_' + c + '_V');
      if (idx !== -1) cellIndices.push(idx);
    }
    if (addrIdx === -1 || voltIdx === -1) {
      alert('Не знайдено обов’язкових колонок (addr, Voltage_V) у файлі');
      return;
    }
  } else {
    // Legacy format: ts,addr,r00..r38
    addrIdx = header.indexOf('addr');
    tsIdx = header.indexOf('ts');
    r00Idx = header.indexOf('r00');
    r01Idx = header.indexOf('r01');
    r38Idx = header.indexOf('r38');
    if (addrIdx === -1 || r00Idx === -1) {
      alert('Не знайдено обов’язкових колонок телеметрії (addr, r00) у файлі');
      return;
    }
    for (let c = 2; c <= 16; c++) {
      const idx = header.indexOf('r' + (c < 10 ? '0' + c : c));
      if (idx !== -1) cellIndices.push(idx);
    }
  }

  const rows = [];
  const packMap = {};

  for (let i = 1; i < lines.length; i++) {
    const parts = lines[i].split(delim).map(s => s.trim());
    if (parts.length < header.length) continue;

    const addr = parts[addrIdx] || '';
    let ts, v, curr, cells, balMask;

    if (newFormat) {
      // "2026-10-28 | 13:35:05" -> ISO, чтобы new Date() понимал во всех браузерах
      ts = (parts[tsIdx] || '').replace(' | ', 'T');
      v = parseFloat(parts[voltIdx]);
      if (isNaN(v)) v = 0;
      curr = parseFloat(parts[currIdx]);
      if (isNaN(curr)) curr = 0;
      // В CSV ячейки записаны в Вольтах -> возвращаем к милливольтам
      cells = cellIndices.map(idx => {
        const val = parseFloat(parts[idx]);
        return isNaN(val) ? 0 : Math.round(val * 1000);
      });
      const bh = (parts[balHexIdx] || '').trim();
      balMask = (bh && bh.toLowerCase() !== 'none') ? (parseInt(bh, 16) || 0) : 0;
    } else {
      ts = parts[tsIdx] || '';
      v = parseInt(parts[r00Idx], 10) / 100.0;
      const rawI = parseInt(parts[r01Idx], 10);
      curr = rawI > 32767 ? (rawI - 65536) / 100.0 : rawI / 100.0;
      balMask = r38Idx !== -1 ? parseInt(parts[r38Idx], 10) : 0;
      cells = cellIndices.map(idx => parseInt(parts[idx], 10));
    }

    const item = { ts, addr, v, curr, cells, balMask };
    rows.push(item);

    if (!packMap[addr]) packMap[addr] = [];
    packMap[addr].push(item);
  }

  if (rows.length === 0) {
    alert('Не знайдено валідних рядків даних у файлі');
    return;
  }

  const addrs = Object.keys(packMap).sort();
  const startTime = rows[0].ts;
  const endTime = rows[rows.length - 1].ts;
  const minV = Math.min(...rows.map(r => r.v));
  const maxV = Math.max(...rows.map(r => r.v));
  const maxCharge = Math.max(...rows.map(r => r.curr));
  const maxDischarge = Math.min(...rows.map(r => r.curr));

  const cellStats = [];

  addrs.forEach(addr => {
    const pRows = packMap[addr];
    const nCells = pRows[0].cells.length;

    const cellMins = new Array(nCells).fill(99999);
    const cellMaxs = new Array(nCells).fill(0);
    const balCounts = new Array(nCells).fill(0);
    const rintSamples = Array.from({ length: nCells }, () => []);

    for (let i = 0; i < pRows.length; i++) {
      const r = pRows[i];
      for (let c = 0; c < nCells; c++) {
        const mv = r.cells[c];
        if (mv > 1000 && mv < 5000) {
          if (mv < cellMins[c]) cellMins[c] = mv;
          if (mv > cellMaxs[c]) cellMaxs[c] = mv;
        }
        if (r.balMask & (1 << c)) {
          balCounts[c]++;
        }
      }

      if (i > 0) {
        const prev = pRows[i - 1];
        const dI = Math.abs(r.curr - prev.curr);
        if (dI >= 1.5 && dI <= 50.0) {
          for (let c = 0; c < nCells; c++) {
            const dV = Math.abs(r.cells[c] - prev.cells[c]);
            const rint = (dV / dI);
            if (rint >= 0.5 && rint <= 15.0) {
              rintSamples[c].push(rint);
            }
          }
        }
      }
    }

    for (let c = 0; c < nCells; c++) {
      const samples = rintSamples[c];
      const avgR = samples.length ? (samples.reduce((a, b) => a + b, 0) / samples.length).toFixed(2) : '—';
      const diff = (cellMaxs[c] > cellMins[c] && cellMins[c] < 99999) ? (cellMaxs[c] - cellMins[c]) : 0;
      cellStats.push({
        addr,
        cellNum: c + 1,
        min: cellMins[c] < 99999 ? cellMins[c] : '—',
        max: cellMaxs[c] > 0 ? cellMaxs[c] : '—',
        diff,
        rint: avgR,
        balCount: balCounts[c]
      });
    }
  });

  let maxBankDelta = 0;
  rows.forEach(r => {
    if (r.cells && r.cells.length) {
      const valid = r.cells.filter(c => c > 1000);
      if (valid.length) {
        const d = Math.max(...valid) - Math.min(...valid);
        if (d > maxBankDelta) maxBankDelta = d;
      }
    }
  });

  logAnalysisData = {
    filename,
    rows,
    packMap,
    addrs,
    startTime,
    endTime,
    minV,
    maxV,
    maxCharge,
    maxDischarge,
    maxBankDelta,
    cellStats
  };

  renderLogAnalysisResults();
}

function renderLogAnalysisResults() {
  if (!logAnalysisData) return;
  const d = logAnalysisData;
  document.getElementById('drop_zone').style.display = 'none';
  document.getElementById('log_results').style.display = 'block';
  document.getElementById('log_file_title').textContent = `📁 ${d.filename} (${d.rows.length} рядків)`;

  const durationSec = Math.round((new Date(d.endTime) - new Date(d.startTime)) / 1000) || 0;
  const durMin = Math.floor(durationSec / 60);
  const durSec = durationSec % 60;
  const durStr = durMin > 0 ? `${durMin} хв ${durSec} сек` : `${durSec} сек`;

  document.getElementById('log_stats_grid').innerHTML = `
    <div class="bank-card">
      <div class="l">Тривалість сесії</div>
      <div class="v" style="font-size:20px">${durStr}</div>
      <div class="subtext">Знайдено АКБ: <b>${d.addrs.join(', ')}</b></div>
    </div>
    <div class="bank-card">
      <div class="l">Діапазон напруги банку</div>
      <div class="v" style="font-size:20px">${d.minV.toFixed(2)} — ${d.maxV.toFixed(2)} V</div>
      <div class="subtext">ΔV сеансу: ${(d.maxV - d.minV).toFixed(2)} V</div>
    </div>
    <div class="bank-card">
      <div class="l">Пікові струми</div>
      <div class="v" style="font-size:20px"><span style="color:var(--ok)">+${d.maxCharge.toFixed(1)}А</span> / <span style="color:var(--bad)">${d.maxDischarge.toFixed(1)}А</span></div>
      <div class="subtext">Заряд / Розряд</div>
    </div>
    <div class="bank-card">
      <div class="l">Макс. розбаланс банку</div>
      <div class="v" style="font-size:20px;color:${d.maxBankDelta > 50 ? 'var(--bad)' : 'var(--ok)'}">Δ = ${d.maxBankDelta} mV</div>
      <div class="subtext">Пікове розходження комірок</div>
    </div>
  `;

  const tbody = document.getElementById('log_cells_tbody');
  tbody.innerHTML = d.cellStats.map(s => {
    const isBadR = (s.rint !== '—' && parseFloat(s.rint) > 2.8);
    const isHighDiff = (s.diff > 250);
    const statusPill = isBadR ? '<span class="pill bad">Підвищений опір</span>' :
                       (isHighDiff ? '<span class="pill warn">Розбалансована</span>' : '<span class="pill ok">Норма</span>');
    const balPill = s.balCount > 0 ? `<span class="pill warn">${s.balCount} раз</span>` : '<span style="color:var(--dim)">0</span>';
    const minV = (s.min === '—') ? '—' : (s.min / 1000).toFixed(3);
    const maxV = (s.max === '—') ? '—' : (s.max / 1000).toFixed(3);
    return `
      <tr>
        <td><b>${s.addr}</b></td>
        <td>#${s.cellNum}</td>
        <td>${minV}</td>
        <td>${maxV}</td>
        <td><b>${s.diff} mV</b></td>
        <td><b style="color:var(--acc)">${s.rint !== '—' ? s.rint + ' мОм' : '—'}</b></td>
        <td>${balPill}</td>
        <td>${statusPill}</td>
      </tr>
    `;
  }).join('');

  drawLogChart();
}

function drawLogChart() {
  const canvas = document.getElementById('logChart');
  if (!canvas || !logAnalysisData) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  const rows = logAnalysisData.rows;
  if (!rows || rows.length < 2) return;

  ctx.clearRect(0, 0, w, h);

  const padL = 50, padR = 40, padT = 20, padB = 30;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  let minV = logAnalysisData.minV - 0.5;
  let maxV = logAnalysisData.maxV + 0.5;
  if (maxV <= minV) maxV = minV + 1.0;

  let minI = logAnalysisData.maxDischarge - 1.0;
  let maxI = logAnalysisData.maxCharge + 1.0;
  if (maxI <= minI) maxI = minI + 2.0;

  ctx.strokeStyle = '#252c34';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const y = padT + (plotH / 4) * i;
    ctx.moveTo(padL, y); ctx.lineTo(w - padR, y);

    const vVal = (maxV - ((maxV - minV) / 4) * i).toFixed(1);
    ctx.fillStyle = '#3fb950';
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'right';
    ctx.fillText(vVal + 'V', padL - 6, y + 3);

    const iVal = (maxI - ((maxI - minI) / 4) * i).toFixed(1);
    ctx.fillStyle = '#58a6ff';
    ctx.textAlign = 'left';
    ctx.fillText(iVal + 'A', w - padR + 6, y + 3);
  }
  ctx.stroke();

  if (minI < 0 && maxI > 0) {
    const y0 = padT + plotH - ((0 - minI) / (maxI - minI)) * plotH;
    ctx.strokeStyle = '#7d8894';
    ctx.setLineDash([2, 4]);
    ctx.beginPath();
    ctx.moveTo(padL, y0); ctx.lineTo(w - padR, y0);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Voltage
  ctx.strokeStyle = '#3fb950';
  ctx.lineWidth = 1.8;
  ctx.beginPath();
  rows.forEach((r, idx) => {
    const x = padL + (idx / (rows.length - 1)) * plotW;
    const y = padT + plotH - ((r.v - minV) / (maxV - minV)) * plotH;
    if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Current
  ctx.strokeStyle = '#58a6ff';
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  rows.forEach((r, idx) => {
    const x = padL + (idx / (rows.length - 1)) * plotW;
    const y = padT + plotH - ((r.curr - minI) / (maxI - minI)) * plotH;
    if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  ctx.fillStyle = '#7d8894';
  ctx.font = '10px sans-serif';
  ctx.textAlign = 'left';
  const tStart = (rows[0].ts || '').split('T')[1] || rows[0].ts;
  const tEnd = (rows[rows.length - 1].ts || '').split('T')[1] || rows[rows.length - 1].ts;
  ctx.fillText(tStart, padL, h - 10);
  ctx.textAlign = 'right';
  ctx.fillText(tEnd, w - padR, h - 10);
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
    lb.textContent = logOn ? 'Зупинити запис (усі)' : 'Старт запису (усі АКБ)';
    lb.className = logOn ? 'off' : '';

    // Logging profiles (v2.4): both checkboxes reflect server-side state
    const cfg = d.log_cfg || {};
    const ca = document.getElementById('cfg_alarm');
    const cw = document.getElementById('cfg_work');
    const ci = document.getElementById('cfg_interval');
    if (ca) ca.checked = !!cfg.alarm;
    if (cw) cw.checked = !!cfg.work;
    if (ci && document.activeElement !== ci && cfg.work_interval != null) {
      ci.value = cfg.work_interval;
    }

    const lf = document.getElementById('logfiles');
    if (lf) {
      const files = d.log_files || {};
      const keys = Object.keys(files).sort();
      lf.innerHTML = keys.length ? keys.map(k => {
        const f = files[k];
        const state = f.on ? '<span class="pill ok">запис</span>' : '<span class="pill">стоп</span>';
        return `<tr><td>${k}</td><td>${f.path || '—'}</td><td>${f.rows || 0} · ${f.last_ts || '—'}</td><td>${state}</td><td style="color:var(--dim);font-size:12px">A ${f.alarm_rows || 0} / W ${f.work_rows || 0}</td></tr>`;
      }).join('') : '<tr><td>—</td><td>лог ще не створювався</td><td>—</td><td>—</td><td>—</td></tr>';
    }

    renderTabs();
    renderView();
  } catch (e) {
    const c = document.getElementById('conn');
    c.textContent = 'NO SERVER';
    c.className = 'pill bad';
  }
}

async function toggleMode() {
  const mbtn = document.getElementById('modebtn');
  if (mbtn) {
    mbtn.disabled = true;
    mbtn.textContent = '⏳ Перемикання...';
  }
  const curPassive = latestData ? (latestData.mode === 'passive') : (mbtn && mbtn.classList.contains('passive'));
  const targetMode = curPassive ? 'active' : 'passive';
  try {
    await fetch('/api/mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode: targetMode })
    });
    await tick();
  } catch (e) {
    console.error(e);
  } finally {
    if (mbtn) mbtn.disabled = false;
  }
}

async function triggerRescan() {
  const btn = document.querySelector('.btn-rescan');
  if (btn) {
    btn.disabled = true;
    btn.textContent = '🔄 Сканування...';
  }

  if (latestData && latestData.mode === 'passive') {
    const doActive = confirm(
      "Утиліта зараз у ПАСИВНОМУ режимі (Sniffer) — передачу в шину вимкнено, щоб не створювати колізій із COMBOX.\\n\\n" +
      "Бажаєте перемкнутися в ⚡ АКТИВНИЙ режим (Master), щоб утиліта сама опитала адреси 0x01..0x10 прямо зараз?"
    );
    if (doActive) {
      await fetch('/api/mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: 'active' })
      });
      await fetch('/api/rescan', { method: 'POST' });
      setTimeout(tick, 100);
      setTimeout(() => { if (btn) { btn.disabled = false; btn.textContent = '🔄 Пересканувати всі 16'; } }, 3000);
      return;
    }
  }

  await fetch('/api/rescan', { method: 'POST' });
  setTimeout(tick, 100);
  setTimeout(() => { if (btn) { btn.disabled = false; btn.textContent = '🔄 Пересканувати всі 16'; } }, 2500);
}

async function toggleLog() {
  await fetch('/api/log', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ on: !logOn })
  });
  tick();
}

async function togglePackLog() {
  if (!currentPackAddr) return;
  const p = (latestData && latestData.packs) ? (latestData.packs[currentPackAddr] || {}) : {};
  await fetch('/api/log', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ on: !p.log_on, addr: currentPackAddr })
  });
  tick();
}

async function saveLogCfg() {
  const alarm = !!(document.getElementById('cfg_alarm') || {}).checked;
  const work = !!(document.getElementById('cfg_work') || {}).checked;
  const interval = parseFloat((document.getElementById('cfg_interval') || {}).value) || 5;
  await fetch('/api/logcfg', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ alarm: alarm, work: work, work_interval: interval })
  });
  tick();
}

tick();
setInterval(tick, 1000);
initLogViewer();
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
        elif self.path == "/api/logcfg":
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except Exception:
                req = {}
            cfg = self.bus.set_log_cfg(
                req.get("alarm"), req.get("work"), req.get("work_interval"))
            self._send(200, json.dumps({"ok": True, "log_cfg": cfg}),
                       "application/json")
        elif self.path == "/api/log":
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except Exception:
                req = {}
            on = bool(req.get("on"))
            bus = self.bus
            addr_str = req.get("addr")
            addrs = None
            if addr_str is not None:
                try:
                    addrs = [int(addr_str, 16) if isinstance(addr_str, str) else int(addr_str)]
                except (TypeError, ValueError):
                    addrs = []
            started = bus.set_log(on, addrs)
            self._send(200, json.dumps({"ok": True, "on": on,
                                        "addrs": [f"0x{a:02X}" for a in started]}),
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
    bits = {b["name"]: b["on"] for b in decode_regs(regs)["status_bits"]}
    assert bits.get("Protect"), "status bit"

    regs[25] = 0x8001
    d = decode_regs(regs)
    assert d["status_name"] == "Charging + Heating", f"status enum+heating: {d['status_name']}"
    assert {b["name"]: b["on"] for b in d["status_bits"]}.get("Heating ON"), "heating bit"
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
    regs[27] = 0x1000
    on = [b["name"] for b in decode_regs(regs)["protection_bits"] if b["on"]]
    assert on == ["Low Capacity"], f"prot 0x1000: {on}"
    regs[27] = 0x2000
    on = [b["name"] for b in decode_regs(regs)["protection_bits"] if b["on"]]
    assert on == ["Short Circuit"], f"prot 0x2000: {on}"
    regs[27] = 0x4000
    on = [b["name"] for b in decode_regs(regs)["protection_bits"] if b["on"]]
    assert on == ["Reverse"], f"prot 0x4000: {on}"
    regs[27] = 0
    regs[26] = 0x2000
    on = [b["name"] for b in decode_regs(regs)["warning_bits"] if b["on"]]
    assert on == ["Float stop"], f"warn 0x2000: {on}"
    assert len(decode_regs(regs)["warning_bits"]) == 14, "warn = 14 bits"
    assert len(decode_regs(regs)["protection_bits"]) == 15, "prot = 15 bits"

    # --- CSV log format: readable header, volts, date, regs 31..35 ---
    lregs = list(regs)
    lregs[25], lregs[26], lregs[27], lregs[28] = 0, 0, 0, 0
    lregs[33], lregs[34], lregs[35] = 0x1414, 0x1514, 0
    lp = decode_regs(lregs)
    lcells, ltemps = _log_slots(lp)
    assert (lcells, ltemps) == (15, 6), f"log slots: {lcells}, {ltemps}"
    lhdr = build_log_header(lcells, ltemps)
    lrow = build_log_values(lp, 1, lcells, ltemps, ts="2026-10-28 | 13:35:05")
    assert len(lhdr) == len(lrow), f"log width: {len(lhdr)} != {len(lrow)}"
    assert lrow[0] == "2026-10-28 | 13:35:05", "log date"
    assert lrow[lhdr.index("Log")] == "Work", "log kind default"
    assert lrow[lhdr.index("addr")] == "0x01", "log addr"
    assert abs(lrow[lhdr.index("Cell_1_V")] - 3.456) < 1e-9, "cell volts"
    assert abs(lrow[lhdr.index("Cell_Min_V")] - 3.456) < 1e-9, "cell min volts"
    assert abs(lrow[lhdr.index("Full_Capacity_Ah")] - 100.0) < 0.01, "reg31/32 full cap"
    assert lrow[lhdr.index("Temp_1_C")] == 20, "reg33 temp sensor 1"
    assert lrow[lhdr.index("Temp_3_C")] == 21, "reg34 temp sensor 3"
    assert lrow[lhdr.index("Status_Hex")] == "0x0000", "status hex"
    lalarm = build_log_values(lp, 1, lcells, ltemps, ts="2026-10-28 | 13:35:05",
                              log_kind=LOG_KIND_ALARM)
    assert len(lalarm) == len(lhdr), "alarm log width"
    assert lalarm[lhdr.index("Log")] == "Alarm", "log kind alarm"

    # --- passive mode frame parser ---
    req_body = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x1C])
    req = req_body + crc_bytes(req_body)
    regs28 = [5306, 0] + [3500] * 15 + [0, 21, 20, 21,
                                        100, 100, 100, 100, 0, 2, 0x1000]
    resp_body = bytes([0x01, 0x03, 56]) + b"".join(
        v.to_bytes(2, "big") for v in regs28)
    resp = resp_body + crc_bytes(resp_body)
    stream = b"\x00\xFFgarbage" + req + b"\xFF" + resp + b"\x00\x00"
    frames, rest = parse_bus(stream)
    kinds = [(k, a) for k, a, _ in frames]
    assert ("req", 1) in kinds, f"parse request: {kinds}"
    assert ("resp", 1) in kinds, f"parse response: {kinds}"

    # --- Multi-pack bank summary & autonomy test ---
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
    assert summary["cell_delta_mv"] == (3495 - 3450), f"bank delta: {summary['cell_delta_mv']}"
    assert summary["any_balancing"] is True, "any balancing must be true"

    # --- Autonomy calculation test ---
    aut_chg = calculate_autonomy(52.5, 25.0, 100.0, 200.0)
    assert aut_chg["energy_kwh"] == 5.25, f"energy kwh: {aut_chg}"
    assert "до 100%: ~4 год" in aut_chg["eta_str"], f"eta charge: {aut_chg}"

    aut_dis = calculate_autonomy(51.0, -10.0, 100.0, 200.0)
    assert "автономність: ~10 год" in aut_dis["eta_str"], f"eta discharge: {aut_dis}"

    # --- Port auto-detection test ---
    from collections import namedtuple
    from unittest.mock import patch
    FakePort = namedtuple('FakePort', ['device', 'description', 'manufacturer', 'vid', 'pid', 'hwid'])

    with patch('serial.tools.list_ports.comports', return_value=[FakePort('COM3', 'USB Serial Port (COM3)', 'FTDI', 0x0403, 0x6001, 'USB VID:PID=0403:6001')]):
        p_dev, p_desc = find_usb_rs485_port()
        assert p_dev == 'COM3' and 'FTDI' in p_desc, f"Windows FTDI auto-detect: {p_dev}, {p_desc}"

    with patch('serial.tools.list_ports.comports', return_value=[FakePort('COM5', 'USB-SERIAL CH340 (COM5)', 'wch.cn', 0x1A86, 0x7523, 'USB VID:PID=1A86:7523')]):
        p_dev, p_desc = find_usb_rs485_port()
        assert p_dev == 'COM5' and 'CH340' in p_desc, f"Windows CH340 auto-detect: {p_dev}, {p_desc}"

    # --- Dev Info block decoder test (Vision V-LFP48100 0x0069 registers) ---
    raw_info_bytes = (
        b"LFP-15SP10G3-100Ah-100A "  # 24 B (12 regs): Model
        b"V10R18"                    # 6 B (3 regs): Firmware ASCII
        b"2025040900109522"          # 16 B (8 regs): Serial Number
    )
    info_regs = [
        (raw_info_bytes[i] << 8) | raw_info_bytes[i + 1]
        for i in range(0, len(raw_info_bytes), 2)
    ]
    dev_info = decode_dev_info_block(info_regs)
    assert dev_info["model"] == "LFP-15SP10G3-100Ah-100A", f"model: {dev_info['model']}"
    assert dev_info["fw_version"] == "V10R18", f"fw_version: {dev_info['fw_version']}"
    assert dev_info["serial"] == "2025040900109522", f"serial: {dev_info['serial']}"

    # --- Logging Profile v2.4 (Alarm signature & log row header) test ---
    bus_mock = BmsBus("MOCK", 9600, mock=True)
    norm_parsed = {"status": 0x0001, "warning": 0x0000, "protection": 0x0000, "error_code": 0x0000, "temp_max": 25.0}
    norm_sig = bus_mock._alarm_signature(norm_parsed)
    assert not bus_mock._is_alarm(norm_sig), f"norm_sig must not be alarm: {norm_sig}"

    alarm_parsed = {"status": 0x0004, "warning": 0x0000, "protection": 0x2000, "error_code": 0x0000, "temp_max": 25.0}
    alarm_sig = bus_mock._alarm_signature(alarm_parsed)
    assert bus_mock._is_alarm(alarm_sig), f"alarm_sig must be alarm: {alarm_sig}"
    assert alarm_sig != norm_sig, "alarm signature must differ from normal"

    temp_hot = {"status": 0x0001, "warning": 0, "protection": 0, "error_code": 0, "temp_max": 48.0}
    assert bus_mock._is_alarm(bus_mock._alarm_signature(temp_hot)), "temp_hot must trigger alarm"

    hdr = build_log_header()
    assert hdr[0] == "Date" and hdr[1] == "Log" and hdr[2] == "addr", f"header structure: {hdr[:3]}"
    row_alarm = build_log_values(alarm_parsed, 1, log_kind=LOG_KIND_ALARM)
    assert row_alarm[1] == "Alarm" and row_alarm[2] == "0x01", f"alarm row: {row_alarm[:3]}"
    row_work = build_log_values(norm_parsed, 1, log_kind=LOG_KIND_WORK)
    assert row_work[1] == "Work" and row_work[2] == "0x01", f"work row: {row_work[:3]}"

    # --- Telemetry bitmask definitions test ---
    prot_names = [name for mask, name in PROT_BITS if (0x1000 & mask)]
    assert "Low Capacity" in prot_names, f"0x1000 Low Capacity in PROT_BITS: {prot_names}"
    warn_names = [name for mask, name in WARN_BITS if (0x1000 & mask)]
    assert "Low Capacity" in warn_names, f"0x1000 Low Capacity in WARN_BITS: {warn_names}"

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
    ap.add_argument("--active", action="store_true",
                    help="start in active master polling mode (default is passive sniffer)")
    ap.add_argument("--passive", action="store_true",
                    help="start in passive sniffer mode (default, listen only, no TX)")
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

    is_passive = not args.active
    bus = BmsBus(port, args.baud, initial_addr=args.addr, mock=args.mock, passive=is_passive)
    threading.Thread(target=bus.loop, daemon=True).start()
    Handler.bus = bus
    srv = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)

    mode_label = "MOCK MULTI-PACK" if args.mock else (
        f"{port} @ {args.baud} " + ("PASSIVE SNIFFER" if is_passive else "ACTIVE MASTER"))
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
