#!/usr/bin/env python3
"""tools/can_sniff.py — Пасивний/Активний CAN-сніфер та декодер протоколу Deye/Pylontech.

Підтримує USB-адаптери з протоколом SLCAN (Lawicel ASCII):
  * WeAct USB to CAN Module V2 (STM32G431, USB2CANV2 ComPort)
  * Будь-які адаптери на базі SLCAN / CANable / candlelight у режимі VCP.

Можливості:
  1. Автоматичне виявлення WeAct USB2CAN V2 за USB VID:PID (0x0483:0x5740).
  2. Ініціалізація шини на 500 кбіт/с (S6):
     - Режим Normal (з ACK, команда 'O'): для тестового стенда без інвертора.
     - Режим Listen-Only (без ACK, команда 'L'): безпечний пасивний моніторинг шини з інвертором.
  3. Декодування стандартних кадрів COM-Box та Deye:
     - 0x351: Ліміти заряд/розряд (CVL, CCL, DCL, DVL)
     - 0x355: SOC (%) та SOH (%)
     - 0x356: Напруга (В), знаковий струм (А, заряд/розряд), температура (°C)
     - 0x359: Біти захистів (Protect), аварій (Alarms), кількість паків, маркер 'PN'
     - 0x35C: Прапорці дозволів і запитів (Charge/Discharge EN, Request Full Charge / Баланс)
     - 0x35E: Бренд акумулятора (ASCII 'PYLON   ')
     - 0x305: Серцебиття / системний час інвертора Deye (PCS Time)
  4. Селфтест (--selftest) для перевірки декодування без підключеного заліза.

Використання:
  python3 tools/can_sniff.py --selftest
  python3 tools/can_sniff.py --list
  python3 tools/can_sniff.py
  python3 tools/can_sniff.py --port /dev/cu.usbmodemB24D205D38631
  python3 tools/can_sniff.py --listen-only
  python3 tools/can_sniff.py --duration 30 --out deye/can_test.log
"""

import argparse
import os
import signal
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


@dataclass
class CanFrame:
    can_id: int
    is_extended: bool
    is_rtr: bool
    dlc: int
    data: bytes
    timestamp: float = 0.0
    raw: str = ""


# ---------------------------------------------------------------------------
# Декодери бітових полів Deye / Pylontech
# ---------------------------------------------------------------------------

PROT_B0_MAP = [
    (0x01, "Cell_OV"),
    (0x02, "Pack_OV"),
    (0x04, "Cell_UV"),
    (0x08, "Pack_UV"),
    (0x10, "Charge_OT"),
    (0x20, "Charge_UT"),
    (0x40, "Discharge_OT"),
    (0x80, "Discharge_OC"),
]

PROT_B1_MAP = [
    (0x01, "Charge_OC"),
    (0x08, "System_Error"),
]

FLAGS_MAP = [
    (0x80, "Charge_EN"),
    (0x40, "Discharge_EN"),
    (0x10, "Req_Force_Charge"),
    (0x08, "Req_Full_Charge(BAL)"),
]


def decode_flags(val: int, mapping: list) -> str:
    matches = [name for mask, name in mapping if (val & mask) != 0]
    return ", ".join(matches) if matches else "None"


def u16le(b: bytes) -> int:
    return struct.unpack("<H", b)[0]


def i16le(b: bytes) -> int:
    return struct.unpack("<h", b)[0]


def parse_slcan_line(line: str) -> Optional[CanFrame]:
    """Розбирає рядок протоколу SLCAN (t, T, r, R)."""
    line = line.strip()
    if not line:
        return None

    cmd = line[0]
    try:
        if cmd == 't':  # Standard data frame
            can_id = int(line[1:4], 16)
            dlc = int(line[4])
            data = bytes.fromhex(line[5:5 + dlc * 2])
            return CanFrame(can_id=can_id, is_extended=False, is_rtr=False, dlc=dlc, data=data, raw=line)
        elif cmd == 'T':  # Extended data frame
            can_id = int(line[1:9], 16)
            dlc = int(line[9])
            data = bytes.fromhex(line[10:10 + dlc * 2])
            return CanFrame(can_id=can_id, is_extended=True, is_rtr=False, dlc=dlc, data=data, raw=line)
        elif cmd == 'r':  # Standard RTR frame
            can_id = int(line[1:4], 16)
            dlc = int(line[4])
            return CanFrame(can_id=can_id, is_extended=False, is_rtr=True, dlc=dlc, data=b"", raw=line)
        elif cmd == 'R':  # Extended RTR frame
            can_id = int(line[1:9], 16)
            dlc = int(line[9])
            return CanFrame(can_id=can_id, is_extended=True, is_rtr=True, dlc=dlc, data=b"", raw=line)
    except (ValueError, IndexError):
        return None
    return None


def decode_can_frame(f: CanFrame) -> str:
    """Декодує CAN-кадр протоколу Deye/Pylontech у зрозумілий людині текст."""
    cid = f.can_id
    d = f.data

    if cid == 0x351 and len(d) >= 8:
        cvl = u16le(d[0:2]) / 10.0
        ccl = u16le(d[2:4]) / 10.0
        dcl = u16le(d[4:6]) / 10.0
        dvl = u16le(d[6:8]) / 10.0
        return f"LIMITS: CVL={cvl:.1f}V CCL={ccl:.1f}A DCL={dcl:.1f}A DVL={dvl:.1f}V"

    elif cid == 0x355 and len(d) >= 4:
        soc = u16le(d[0:2])
        soh = u16le(d[2:4])
        return f"SOC/SOH: SOC={soc}% SOH={soh}%"

    elif cid == 0x356 and len(d) >= 6:
        v = u16le(d[0:2]) / 100.0
        i = i16le(d[2:4]) / 10.0
        t = i16le(d[4:6]) / 10.0
        mode = "CHG" if i < -0.5 else ("DISCHG" if i > 0.5 else "IDLE")
        return f"BATTERY: V={v:.2f}V I={i:+.1f}A ({mode}) T={t:+.1f}°C"

    elif cid == 0x359 and len(d) >= 7:
        b0, b1, b2, b3, packs = d[0], d[1], d[2], d[3], d[4]
        marker = chr(d[5]) + chr(d[6]) if len(d) >= 7 else ""
        p_str = []
        if b0:
            p_str.append(decode_flags(b0, PROT_B0_MAP))
        if b1:
            p_str.append(decode_flags(b1, PROT_B1_MAP))
        prot_desc = "; ".join(p_str) if p_str else "None"

        a_str = []
        if b2:
            a_str.append(decode_flags(b2, PROT_B0_MAP))
        if b3:
            a_str.append(decode_flags(b3, PROT_B1_MAP))
        alarm_desc = "; ".join(a_str) if a_str else "None"

        return f"ALARMS: Packs={packs} Mark='{marker}' Prot=[{prot_desc}] Warn=[{alarm_desc}]"

    elif cid == 0x35C and len(d) >= 1:
        req = decode_flags(d[0], FLAGS_MAP)
        return f"REQUESTS: [0x{d[0]:02X}] {req}"

    elif cid == 0x35E:
        brand = d.decode("ascii", errors="replace").rstrip("\x00").strip()
        return f"BRAND: '{brand}'"

    elif cid == 0x305 and len(d) >= 6:
        # PCS Heartbeat / Time broadcast: ss mm hh DD MM YY
        sec, minute, hour, day, month, year = d[0], d[1], d[2], d[3], d[4], 2000 + d[5]
        return f"PCS_TIME: {year:04d}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{sec:02d}"

    # Загальний вивід для невідомих ID
    hex_data = " ".join(f"{b:02X}" for b in d)
    return f"DATA [{hex_data}]"


# ---------------------------------------------------------------------------
# Автовиявлення адаптера
# ---------------------------------------------------------------------------

def detect_slcan_device() -> Tuple[Optional[str], Optional[str]]:
    """Шукає WeAct USB2CAN V2 або сумісні SLCAN-пристрої."""
    if serial is None:
        return None, None

    ports = list(serial.tools.list_ports.comports())
    candidates = []

    for p in ports:
        desc = p.description or ""
        mfr = p.manufacturer or ""
        prod = p.product or ""
        dev = p.device

        # Прямий збіг по WeAct USB2CAN V2 (VID=0x0483, PID=0x5740)
        if p.vid == 0x0483 and p.pid == 0x5740 and ("USB2CAN" in desc or "USB2CAN" in prod or "WeAct" in mfr):
            return dev, f"WeAct Studio USB2CAN V2 ({dev})"

        # Загальні критерії SLCAN / CANable
        if any(keyword in s.lower() for keyword in ["usb2can", "slcan", "canable", "cangaroo"] for s in [desc, prod]):
            candidates.append((dev, f"{desc} ({dev})"))

    if candidates:
        return candidates[0]
    return None, None


# ---------------------------------------------------------------------------
# Клас роботи з SLCAN
# ---------------------------------------------------------------------------

class SlcanBus:
    def __init__(self, port: str, baudrate: int = 115200, speed_cmd: str = "S6", listen_only: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.speed_cmd = speed_cmd  # S6 = 500k
        self.listen_only = listen_only
        self.ser = None

    def open(self):
        if serial is None:
            raise RuntimeError("Модуль 'pyserial' не встановлено. Виконайте: pip install pyserial")

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
        # 1. Закрити попередню сесію
        self.ser.write(b"C\r")
        time.sleep(0.05)
        self.ser.read_all()

        # 2. Встановити швидкість (S6 = 500 kbit/s)
        self.ser.write(f"{self.speed_cmd}\r".encode("ascii"))
        time.sleep(0.05)
        res = self.ser.read_all()
        if b"\x07" in res:
            print(f"[УВАГА] Адаптер повернув помилку на команду {self.speed_cmd}", file=sys.stderr)

        # 3. Відкрити канал: 'L' для Listen-Only, 'O' для Normal (з ACK)
        cmd = b"L\r" if self.listen_only else b"O\r"
        self.ser.write(cmd)
        time.sleep(0.05)
        res_open = self.ser.read_all()
        if b"\x07" in res_open:
            print(f"[УВАГА] Адаптер повернув помилку при відкритті каналу ({cmd})", file=sys.stderr)

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"C\r")
                time.sleep(0.02)
                self.ser.close()
            except Exception:
                pass

    def read_frame(self, rx_buf: bytes) -> Tuple[Optional[CanFrame], bytes]:
        """Читає та повертає наступний валідний кадр із вхідного буфера."""
        if not self.ser or not self.ser.is_open:
            return None, rx_buf

        chunk = self.ser.read(256)
        if chunk:
            rx_buf += chunk

        if b"\r" in rx_buf:
            line_bytes, rx_buf = rx_buf.split(b"\r", 1)
            line = line_bytes.decode("ascii", errors="replace").strip()
            if line:
                frame = parse_slcan_line(line)
                if frame:
                    frame.timestamp = time.monotonic()
                    return frame, rx_buf
        return None, rx_buf


# ---------------------------------------------------------------------------
# Selftest (Автоматична верифікація логіки без заліза)
# ---------------------------------------------------------------------------

def run_selftest() -> int:
    print("=" * 65)
    print("CAN-SNIFF SELFTEST: Перевірка парсингу та декодування Deye/Pylontech")
    print("=" * 65)

    test_vectors = [
        # 0x351: CVL=54.0V (0x021C), CCL=100.0A (0x03E8), DCL=100.0A (0x03E8), DVL=45.0V (0x01C2)
        ("t35181C02E803E803C201", 0x351, 8, "CVL=54.0V CCL=100.0A DCL=100.0A DVL=45.0V"),
        # 0x355: SOC=98% (0x0062), SOH=100% (0x0064)
        ("t355462006400", 0x355, 4, "SOC=98% SOH=100%"),
        # 0x356: V=51.20V (0x1400 = 5120), I=-15.0A (0xFF6A = -150), T=23.5°C (0x00EB = 235)
        ("t356600146AFFEB00", 0x356, 6, "V=51.20V I=-15.0A (CHG) T=+23.5°C"),
        # 0x359: Cell_OV (b0=0x01), System_Error (b1=0x08), 2 packs, 'PN'
        ("t35980108000002504E00", 0x359, 8, "Packs=2 Mark='PN' Prot=[Cell_OV; System_Error] Warn=[None]"),
        # 0x35C: Charge_EN (0x80) + Discharge_EN (0x40) + Req_Full_Charge (0x08) = 0xC8
        ("t35C2C800", 0x35C, 2, "REQUESTS: [0xC8] Charge_EN, Discharge_EN, Req_Full_Charge(BAL)"),
        # 0x35E: 'PYLON   ' (50 59 4C 4F 4E 00 00 00)
        ("t35E850594C4F4E000000", 0x35E, 8, "BRAND: 'PYLON'"),
        # 0x305: PCS Heartbeat 2026-11-03 10:25:30 (ss=30, mm=25, hh=10, DD=03, MM=11, YY=26)
        ("t30561E190A030B1A", 0x305, 6, "PCS_TIME: 2026-11-03 10:25:30"),
    ]

    passed = 0
    for raw, exp_id, exp_dlc, exp_substr in test_vectors:
        frame = parse_slcan_line(raw)
        assert frame is not None, f"Помилка розбору SLCAN: {raw}"
        assert frame.can_id == exp_id, f"ID розійшовся: 0x{frame.can_id:X} != 0x{exp_id:X}"
        assert frame.dlc == exp_dlc, f"DLC розійшовся: {frame.dlc} != {exp_dlc}"

        desc = decode_can_frame(frame)
        assert exp_substr in desc, f"Декод не містить '{exp_substr}':\n  Отримано: {desc}"
        print(f"[OK] 0x{frame.can_id:03X} -> {desc}")
        passed += 1

    print("=" * 65)
    print(f"Всі {passed} векторів успішно перевірені (100% OK).")
    return 0


# ---------------------------------------------------------------------------
# Точка входу CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="Сніфер та декодер шини CAN для COM-Box та Deye (SLCAN / WeAct)")
    parser.add_argument("--selftest", action="store_true", help="Запуск тестів перевірки парсера кадрів")
    parser.add_argument("--list", action="store_true", help="Список доступних COM-портів")
    parser.add_argument("-p", "--port", type=str, default=None, help="COM-порт адаптера (наприклад /dev/cu.usbmodem...)")
    parser.add_argument("-l", "--listen-only", action="store_true", help="Режим Listen-Only (без генерації ACK)")
    parser.add_argument("--speed", type=str, default="500k", choices=["125k", "250k", "500k", "1000k"], help="Швидкість CAN (за замовчуванням 500k)")
    parser.add_argument("--raw", action="store_true", help="Виводити сирі SLCAN рядки поруч із декодом")
    parser.add_argument("--duration", type=float, default=None, help="Тривалість запису в секундах")
    parser.add_argument("-o", "--out", type=str, default=None, help="Шлях до файлу логу")
    args = parser.parse_args()

    if args.selftest:
        return run_selftest()

    if serial is None:
        print("[ПОМИЛКА] Бібліотеку 'pyserial' не знайдено. Встановіть через 'pip install pyserial'.", file=sys.stderr)
        return 1

    if args.list:
        ports = list(serial.tools.list_ports.comports())
        print(f"Знайдено портів: {len(ports)}")
        for p in ports:
            print(f"  {p.device} | {p.description} | VID:PID={hex(p.vid or 0)}:{hex(p.pid or 0)} | Mfr={p.manufacturer}")
        return 0

    target_port = args.port
    if not target_port:
        detected, desc = detect_slcan_device()
        if detected:
            print(f"[АВТОВИЯВЛЕННЯ] Знайдено адаптер: {desc}")
            target_port = detected
        else:
            print("[ПОМИЛКА] Адаптер USB2CAN не знайдено автоматично. Вкажіть порт через --port <порт>.", file=sys.stderr)
            print("Використайте 'python3 tools/can_sniff.py --list' для списку портів.", file=sys.stderr)
            return 1

    speed_map = {"125k": "S4", "250k": "S5", "500k": "S6", "1000k": "S8"}
    speed_cmd = speed_map.get(args.speed, "S6")

    mode_name = "LISTEN-ONLY (Пасивний, без ACK)" if args.listen_only else "NORMAL (Активний з ACK, стенд)"
    print(f"Підключення до {target_port}...")
    print(f"Швидкість CAN: {args.speed} ({speed_cmd}), Режим: {mode_name}")

    bus = SlcanBus(target_port, baudrate=115200, speed_cmd=speed_cmd, listen_only=args.listen_only)
    try:
        bus.open()
    except Exception as e:
        print(f"[ПОМИЛКА ВІДКРИТТЯ ПОРТУ] {e}", file=sys.stderr)
        return 1

    out_lines: List[str] = []
    out_path = args.out or f"deye/can_sniff_{datetime.now():%Y-%m-%d_%H-%M-%S}.log"
    header = [
        f"# CAN sniff (SLCAN)  port={target_port} speed={args.speed} mode={mode_name}",
        f"# Date: {datetime.now():%Y-%m-%d %H:%M:%S}",
        f"# Format: t=sec  ID [DLC]  DECODE  |  RAW",
    ]
    for h in header:
        print(h)

    def _stop(sig, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    t0 = time.monotonic()
    deadline = t0 + args.duration if args.duration else None
    rx_buf = b""
    frames_count = 0

    print("-" * 75)
    print("Слухаю CAN-шину... Натисніть Ctrl-C для зупинки.")
    print("-" * 75)

    try:
        while True:
            now = time.monotonic()
            if deadline and now >= deadline:
                break

            frame, rx_buf = bus.read_frame(rx_buf)
            if frame:
                frames_count += 1
                elapsed = frame.timestamp - t0
                decode = decode_can_frame(frame)
                raw_info = f" | {frame.raw}" if args.raw else ""
                line_str = f"[{elapsed:8.3f}s]  0x{frame.can_id:03X} [{frame.dlc}]  {decode}{raw_info}"
                print(line_str, flush=True)
                out_lines.append(line_str)
            else:
                time.sleep(0.002)

    except KeyboardInterrupt:
        print("\nЗупинка за запитом користувача.")
    finally:
        bus.close()
        elapsed_total = time.monotonic() - t0
        print("-" * 75)
        print(f"Завершено. Захоплено кадрів: {frames_count} за {elapsed_total:.2f} с.")

        if out_lines:
            os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
            with open(out_path, "w", encoding="utf-8") as f:
                f.write("\n".join(header + out_lines) + "\n")
            print(f"Лог збережено у: {out_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
