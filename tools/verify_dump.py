#!/usr/bin/env python3
"""verify_dump.py — регресійна перевірка кадрів BMS (Modbus RTU) для COM-Box.

Що робить:
  1. Перевіряє CRC16 Modbus кадрів відповіді BMS (FC03, 83 байти).
  2. Декодує поля ТОЧНО за розкладом прошивки src/main.cpp :: bms_parse_response
     (зсуви байтів, масштаби, & 0xFF для SOC/SOH, знаковий струм).
  3. Перевіряє CRC усіх 16 запитів BMS_QUERY_TABLE (таблиця витягується з
     src/main.cpp, тож синхронізація з джерелом автоматична).
  4. --selftest: будує синтетичний кадр зі значень, задокументованих у
     ANALYSIS.md (дамп 08.10.2026), і проганяє його через той самий парсер.

Використання:
  python3 tools/verify_dump.py --selftest
  python3 tools/verify_dump.py --queries
  python3 tools/verify_dump.py tools/dumps/bms_2026-10-08.hex
  python3 tools/verify_dump.py - "01 03 4E ... 7A CC"

Файл дампу: один кадр на рядок, hex байти через пробіл/кому/без розділювача;
рядки, що починаються з # — коментарі.

Код виходить з ненульового exit-коду, якщо хоч одна перевірка впала.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC_MAIN = ROOT / "src" / "main.cpp"

# Розклад відповіді BMS — мусить збігатися з bms_parse_response() у src/main.cpp:
# [0]=адреса [1]=0x03 [2]=0x4E(78) [3..80]=39 регістрів BE [81..82]=CRC (lo,hi)
RESPONSE_LEN = 83
BYTECOUNT = 0x4E
REG_VOLTAGE = 0x0000   # байти [3,4]   /100 → V
REG_CURRENT = 0x0001   # байти [5,6]   int16 BE, /100 → A (знаковий!)
REG_CELLS_FIRST = 0x0002  # байти [7,8] … [36,37] — 15 комірок, мВ
CELL_COUNT = 15
REG_TEMP_A = 0x0012    # байти [39,40]
REG_TEMP = 0x0013      # байти [41,42] — парсер бере саме цей (цілі °C)
REG_TEMP_B = 0x0014    # байти [43,44]
REG_SOC = 0x0015       # байти [45,46] — формула 0x00XX → & 0xFF
REG_SOH = 0x0016       # байти [47,48]
REG_CELL_COUNT = 0x0024  # байти [75,76]
REG_PACK_100AH = 0x0025  # байти [77,78] (у дампі = 1000)

# Очікувані значення з дампа 08.10.2026 (ANALYSIS.md, Q3) — для --selftest
GOLDEN = {
    "voltage_v": 49.45,
    "current_a": 0.0,
    "temp_c": 23.0,
    "soc_pct": 50,
    "soh_pct": 100,
    "cells_mv": [3297] * 15,
    "cell16_mv": 0,
    "cell_count_reg": 15,
}


def modbus_crc16(data: bytes) -> int:
    """CRC16 Modbus (poly 0xA001, init 0xFFFF) — той самий, що modbus_crc16() у прошивці."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def parse_hex_line(line: str) -> bytes:
    """Hex-рядок (пробіли/коми/0x…/по одному символу) → bytes."""
    s = re.sub(r"0x", " ", line)
    s = re.sub(r"[,\t]", " ", s)
    parts = s.split()
    if len(parts) == 1 and len(parts[0]) > 2 and len(parts[0]) % 2 == 0:
        return bytes.fromhex(parts[0])
    return bytes(int(p, 16) for p in parts)


def reg(frame: bytes, index: int) -> int:
    """Реєстр index (0-based) як uint16 BE із даних відповіді."""
    off = 3 + index * 2
    return (frame[off] << 8) | frame[off + 1]


def decode_response(frame: bytes) -> dict:
    """Декод відповіді за розкладом прошивки. Ключі — очікувані значення."""
    cells = [reg(frame, REG_CELLS_FIRST + i) for i in range(CELL_COUNT)]
    return {
        "addr": frame[0],
        "func": frame[1],
        "bytecount": frame[2],
        "voltage_raw": reg(frame, REG_VOLTAGE),
        "voltage_v": reg(frame, REG_VOLTAGE) / 100.0,
        "current_raw": (reg(frame, REG_CURRENT) ^ 0x8000) - 0x8000,  # int16 BE
        "current_a": ((reg(frame, REG_CURRENT) ^ 0x8000) - 0x8000) / 100.0,
        "cells_mv": cells,
        "cell16_mv": reg(frame, 0x11),            # рег 0x11: 0 → комірки 16 нема
        "temp_a": reg(frame, REG_TEMP_A),
        "temp_c": float(reg(frame, REG_TEMP)),
        "temp_b": reg(frame, REG_TEMP_B),
        "soc_pct": reg(frame, REG_SOC) & 0xFF,
        "soh_pct": reg(frame, REG_SOH) & 0xFF,
        "cell_count_reg": reg(frame, REG_CELL_COUNT),
        "pack_100ah_reg": reg(frame, REG_PACK_100AH),
    }


def check_frame(frame: bytes, expect_len: int | None = None) -> list[str]:
    """Структурна перевірка FC03-кадру; порожній список = валідний.

    expect_len — якщо задано, додатково вимагає цю довжину (83 = дані BMS).
    """
    errs = []
    if len(frame) < 5:
        errs.append(f"довжина {len(frame)} < 5")
        return errs
    if frame[1] != 0x03:
        errs.append(f"функція 0x{frame[1]:02X} ≠ 0x03")
    if len(frame) != frame[2] + 5:
        errs.append(f"довжина {len(frame)} ≠ bytecount 0x{frame[2]:02X} + 5 = {frame[2] + 5}")
    rx_crc = (frame[-1] << 8) | frame[-2]          # як у прошивці: (last<<8)|last-1
    calc = modbus_crc16(frame[:-2])
    if rx_crc != calc:
        errs.append(f"CRC приймано 0x{rx_crc:04X} ≠ обчислено 0x{calc:04X}")
    if expect_len is not None and len(frame) != expect_len:
        errs.append(f"довжина {len(frame)} ≠ {expect_len}")
    return errs


def is_data_frame(frame: bytes) -> bool:
    """83-байтний кадр даних (відповідь на qty 0x0027)."""
    return len(frame) == RESPONSE_LEN and frame[1] == 0x03 and frame[2] == BYTECOUNT


def is_model_frame(frame: bytes) -> bool:
    """51-байтний кадр моделі (відповідь на qty 0x0017, ASCII регістри 0x69)."""
    return len(frame) == 51 and frame[1] == 0x03 and frame[2] == 0x2E


def print_frame(frame: bytes) -> None:
    if is_model_frame(frame):
        ascii_bytes = bytes(frame[3:-2])
        text = ascii_bytes.decode("latin-1")
        print(f"  адреса           0x{frame[0]:02X}")
        print(f"  модель (ASCII)   {text}")
        return
    if not is_data_frame(frame):
        print(f"  (не 83-байтний кадр даних: довжина {len(frame)}, bytecount 0x{frame[2]:02X})")
        return
    d = decode_response(frame)
    print(f"  адреса           0x{d['addr']:02X}")
    print(f"  напруга          {d['voltage_v']:.2f} V (raw {d['voltage_raw']})")
    print(f"  струм            {d['current_a']:+.2f} A (raw {d['current_raw']}, знаковий)")
    print(f"  комірки (15)     min {min(d['cells_mv'])} / max {max(d['cells_mv'])} мВ  {d['cells_mv']}")
    print(f"  реєстр 0x11      {reg(frame, 0x11)} мВ (0 → 15 комірок, не 16)")
    print(f"  температури      0x12={d['temp_a']}  0x13={d['temp_c']:.0f}°C (парсер)  0x14={d['temp_b']}")
    print(f"  SOC / SOH        {d['soc_pct']}% / {d['soh_pct']}%  (& 0xFF)")
    print(f"  реєстр 0x24/0x25 {d['cell_count_reg']} / {d['pack_100ah_reg']}")


def verify_queries(src: str) -> int:
    """Витягує BMS_QUERY_TABLE із src/main.cpp і перевіряє CRC кожного запиту."""
    m = re.search(r"BMS_QUERY_TABLE\[16\]\[8\]\s*=\s*\{(.*?)\};", src, re.S)
    if not m:
        print("FAIL: BMS_QUERY_TABLE не знайдено в src/main.cpp")
        return 1
    rows = re.findall(r"\{([^}]+)\}", m.group(1))
    if len(rows) != 16:
        print(f"FAIL: у таблиці {len(rows)} рядків, очікувалося 16")
        return 1
    fails = 0
    for row in rows:
        vals = [int(x, 16) for x in row.split(",")]
        frame = bytes(vals)
        rx = (frame[-1] << 8) | frame[-2]
        calc = modbus_crc16(frame[:-2])
        ok = rx == calc
        fails += 0 if ok else 1
        print(f"  {'OK ' if ok else 'FAIL'} addr 0x{frame[0]:02X}  {' '.join(f'{b:02X}' for b in frame)}  CRC {rx:04X}")
    print(f"BMS_QUERY_TABLE: {16 - fails}/16 валідні")
    return 1 if fails else 0


def build_synthetic_frame(current_raw: int = 0) -> bytes:
    """Синтетичний кадр зі значень дампа 08.10.2026 (ANALYSIS.md, Q3).

    Невідомі з публікації реєстри = 0. Це ФІКСТУРА для регресії парсера,
    не literalний дамп; реальний дамп зберігається поруч у tools/dumps/.
    """
    data = bytearray(BYTECOUNT)
    def put(idx: int, val: int) -> None:
        off = idx * 2
        data[off] = (val >> 8) & 0xFF
        data[off + 1] = val & 0xFF

    put(REG_VOLTAGE, 4945)                 # 49.45 V
    put(REG_CURRENT, current_raw & 0xFFFF) # int16 BE
    for i in range(CELL_COUNT):
        put(REG_CELLS_FIRST + i, 3297)      # 15 × 3297 мВ
    put(0x11, 0)                            # 16-ї комірки нема
    put(REG_TEMP_A, 24)
    put(REG_TEMP, 23)
    put(REG_TEMP_B, 23)
    put(REG_SOC, 50)
    put(REG_SOH, 100)
    put(REG_CELL_COUNT, 15)
    put(REG_PACK_100AH, 1000)

    frame = bytearray([0x01, 0x03, BYTECOUNT]) + data
    crc = modbus_crc16(frame)
    frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])  # lo, hi — порядок Modbus
    return bytes(frame)


def selftest() -> int:
    fails = []

    def expect(cond: bool, msg: str) -> None:
        if not cond:
            fails.append(msg)

    # 1) Синтетичний кадр із значеннями дампа проходить перевірку
    f = build_synthetic_frame()
    errs = check_frame(f)
    expect(not errs, f"синтетичний кадр не пройшов: {errs}")
    d = decode_response(f)
    expect(d["voltage_v"] == GOLDEN["voltage_v"], f"напруга {d['voltage_v']}")
    expect(d["current_a"] == GOLDEN["current_a"], f"струм {d['current_a']}")
    expect(d["temp_c"] == GOLDEN["temp_c"], f"t° {d['temp_c']}")
    expect(d["soc_pct"] == GOLDEN["soc_pct"], f"SOC {d['soc_pct']}")
    expect(d["soh_pct"] == GOLDEN["soh_pct"], f"SOH {d['soh_pct']}")
    expect(d["cells_mv"] == GOLDEN["cells_mv"], f"комірки {d['cells_mv'][:3]}…")
    expect(d["cell_count_reg"] == GOLDEN["cell_count_reg"], f"0x24 {d['cell_count_reg']}")
    expect(reg(f, 0x11) == GOLDEN["cell16_mv"], f"рег 0x11 {reg(f, 0x11)}")

    # 2) Знаковий струм: B1 (від'ємний струм у 0x356)
    f_neg = build_synthetic_frame(current_raw=((-12345) & 0xFFFF))
    d_neg = decode_response(f_neg)
    expect(d_neg["current_a"] == -123.45, f"від'ємний струм {d_neg['current_a']}")
    expect(not check_frame(f_neg), "кадр зі струмом -123.45 A не пройшов CRC")

    # 3) Псування байта ловиться CRC
    bad = bytearray(f)
    bad[10] ^= 0xFF
    expect(bool(check_frame(bytes(bad))), "псування байта НЕ виявлено")

    # 4) Неправильна довжина ловиться
    expect(bool(check_frame(f[:-1])), "обрізаний кадр НЕ виявлено")

    # 5) Золоті фікстури tools/dumps/*.hex (реальні дампи BMS Tools, 08.10.2026)
    dumps_dir = Path(__file__).resolve().parent / "dumps"
    golden_files = sorted(dumps_dir.glob("*.hex")) if dumps_dir.is_dir() else []
    if not golden_files:
        fails.append(f"немає золотих фікстур у {dumps_dir} (*.hex)")
    for path in golden_files:
        for ln, line in enumerate(path.read_text().splitlines(), 1):
            line = line.split("#")[0].strip()
            if not line:
                continue
            fr = parse_hex_line(line)
            loc = f"{path.name}:{ln}"
            errs = check_frame(fr)
            if errs:
                fails.append(f"{loc} -> {errs}")
                continue
            if is_data_frame(fr):
                dd = decode_response(fr)
                if dd["addr"] not in (0x01, 0x02):
                    fails.append(f"{loc} несподівана адреса 0x{dd['addr']:02X}")
                if dd["voltage_v"] != GOLDEN["voltage_v"]:
                    fails.append(f"{loc} напруга {dd['voltage_v']}")
                if dd["soc_pct"] != GOLDEN["soc_pct"] or dd["soh_pct"] != GOLDEN["soh_pct"]:
                    fails.append(f"{loc} SOC/SOH {dd['soc_pct']}/{dd['soh_pct']}")
                if dd["temp_c"] != GOLDEN["temp_c"]:
                    fails.append(f"{loc} t° {dd['temp_c']}")
                if dd["cell_count_reg"] != GOLDEN["cell_count_reg"]:
                    fails.append(f"{loc} рег 0x24 {dd['cell_count_reg']}")
                if not all(c in (3296, 3297) for c in dd["cells_mv"]):
                    fails.append(f"{loc} комірки {dd['cells_mv'][:4]}…")
            elif is_model_frame(fr):
                text = fr[3:-2].decode("latin-1")
                if not text.startswith("LFP-15SP10G3-100Ah-100A"):
                    fails.append(f"{loc} модель '{text[:30]}'")
            else:
                fails.append(f"{loc} невідомий тип кадру (довжина {len(fr)})")
    if not fails and golden_files:
        print(f"Золоті фікстури: {len(golden_files)} файл(и) у tools/dumps/ — усі кадри валідні "
              f"(49.45 V, SOC 50, SOH 100, 23°C, 15 комірок, модель LFP-15S…)")

    if fails:
        print("SELFTEST FAIL:")
        for x in fails:
            print(f"  - {x}")
        return 1
    print("SELFTEST OK: синтетичний дамп, знаковий струм (B1), CRC, довжина — усі перевірки пройдено")
    return 0


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    cmd = argv[1]

    if cmd == "--selftest":
        return selftest()

    if cmd == "--queries":
        return verify_queries(SRC_MAIN.read_text())

    frames: list[bytes] = []
    if cmd == "-":
        if len(argv) < 3:
            print("FAIL: після '-' подайте hex-рядок")
            return 2
        frames.append(parse_hex_line(argv[2]))
    else:
        path = Path(cmd)
        if not path.exists():
            print(f"FAIL: файл не знайдено: {path}")
            return 2
        for line in path.read_text().splitlines():
            line = line.split("#")[0].strip()
            if line:
                frames.append(parse_hex_line(line))

    rc = 0
    for i, frame in enumerate(frames, 1):
        print(f"Кадр {i}: {' '.join(f'{b:02X}' for b in frame)}")
        errs = check_frame(frame)
        if errs:
            rc = 1
            for e in errs:
                print(f"  FAIL: {e}")
        else:
            print(f"CRC OK ({frame[-2]:02X} {frame[-1]:02X})")  # порядок байтів у проводі, як у логах
            print_frame(frame)
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv))
