# CAN-сніфер COM-Box (ESP32: ACK/normal mode; Nano: пасивний)

Сніфер шини CAN (500 kbit/s) для логування кадрів COM-Box ↔ інвертор.
Два апаратні цілі в одному скетчі (`src/main.cpp`), вибір через env:

- **ESP32 (TWAI, з 13.10.2026)** — режим `TWAI_MODE_NORMAL`: контролер ставить
  ACK на кадрах інших вузлів. Потрібен для лабораторного стенда **без інвертора**: без
  вузла підтвердження шлюз COM-Box багаторазово повторює перший кадр (auto-retransmit, див.
  `../README.md` §8.4), а зі сніфером-ACK шина чиста (6 fps, усі 6 ID). На
  робочій шині з увімкненим інвертором підтвердження вже достатньо — за
  потреби безпечніший пасивний режим (див. нижче, «D на GND»).
- **Arduino Nano (MCP2515)** — пасивний, без передавання.

Див. також:
* [`../README.md`](../README.md) — основна прошивка COM-Box; §2 містить команди
  збірки/прошивки і паралельний RS485-сніфер батарейної шини;
* [`../DEYE_CAN_RS485_BASE.md`](../DEYE_CAN_RS485_BASE.md) — база знань CAN/RS485
  Deye (декод кадрів 0x351…0x35E/0x305 нижче відповідає її розділам по CAN);
* [`../PORT_MAPPING.md`](../PORT_MAPPING.md) — незмінний еталон фізичних пінів
  (MCU ↔ CAN/RS485: PB8/PB9 = PCS-CAN, `chmod 444`);
* [`../tools/rs485_sniff.py`](../tools/rs485_sniff.py) — пасивний RS485-сніфер
  шини батарей (Vision/Modbus), для повного логування обох шин.

## Апаратні цілі

| env | Плата | Перетворювач | Піни |
|---|---|---|---|
| `esp32dev` | ESP32 (ESP-WROOM-32, LuaNode32) | SN65HVD230 (3.3 В) | R→GPIO4 (TWAI RX) |
| `nanoatmega328new` | Arduino Nano 5 В | MCP2515 + TJA1050 | SPI D10–D13, INT не потрібен (poll) |

### Шина COM-Box (RJ45 PCS-CAN)

- пін 8 = CANH, пін 7 = CANL, пін 1 = GND — до CANH/CANL/GND перетворювача.
  GND обов'язковий: спільна земля з шиною.

### Схема підключення SN65HVD230 (ESP32, ACK-режим з 13.10.2026)

| SN65HVD230 | ESP32 / живлення | Примітка |
|---|---|---|
| 3V3 (VCC) | 3V3 | 2.3–3.6 В |
| GND | GND | + спільна земля з шиною (RJ45 пін 1) |
| R (RX) | GPIO4 | `twai_rx` |
| **D (TX)** | **GPIO5** через ~1k (пряме з'єднання — допустимо для стенда) + 10k pull-down на D | `twai_tx`; рекомендовано послідовний 1k із 10k pull-down на D через strapping GPIO5 |
| Rs | GND (або через 10k) | швидкий режим, потрібен для 500 kbit/s |
| RE | GND (якщо виведений) | приймач постійно увімкнений |

Для **пасивного** режиму (робоча шина з інвертором): D від'єднати від GPIO5 і
під'єднати до GND — постійний recessive, жодних домінантних імпульсів на шині
під час reset; прошивку при цьому бажано зібрати з `TWAI_MODE_LISTEN_ONLY`
(стандартна збірка 13.10.2026 — normal mode).

## Збірка і прошивка (ESP32)

```bash
cd can_sniffer
~/.platformio/penv/bin/pio run -e esp32dev                 # збірка
~/.platformio/penv/bin/pio run -e esp32dev -t erase        # стерти flash (за бажанням)
~/.platformio/penv/bin/pio run -e esp32dev -t upload       # прошивка (порт auto-detect)
~/.platformio/penv/bin/pio device monitor -p /dev/cu.usbserial-0001 -b 115200
```

- CP2102 на платі має **авто-reset** (DTR/RTS) — кнопку BOOT натискати не треба.
- Якщо `upload` не вдається з'єднатись (`failed to connect / timeout`): затиснути **BOOT**
  (IO0), коротко натиснути **RST/EN**, відпустити BOOT і повторити upload.
- Кабель має бути data (не зарядний), драйвер CP2102 в macOS вбудований.

## Поточний стан плати (13.10.2026)

- Прошита збіркою `esp32dev` цього проєкту в нормальному режимі (upload,
  13.10.2026); банер у моніторі: `ESP32 TWAI CAN sniffer: 500 kbps, ACK (normal mode)`.
  D-дріт переставлено з GND на GPIO5. Перевірено наживо: шина без інвертора —
  6 fps, усі 6 ID, `missed=0`, TEC=REC=0.
- Раніше (11.10.2026) була заводська прошивка ESP-AT (AT+WiFi/BLE, Espressif
  v2.3.0.0, збірка 20.05.2022), знята перед стиранням і лежить у `backups/`:
  - `backups/esp32_flash_2026-10-11_mac8c94df905998.bin` (4 MB,
    sha256 `d8faf3abdc37904b76413d642559a691e64ea0b320431756ff83204c06ba1634`,
    chip ESP32-D0WD-V3, MAC `8c:94:df:90:59:98`);
  - відкат: `esptool.py write_flash 0x0 <dump>.bin` (або повне стирання + dump).
- `backups/` у `.gitignore` — у git не потрапляє.

## Вивід (Serial 115200)

Кожен кадр: `[час ms] ID STD/EXT len hex...` + декод відомих:

| ID | Зміст |
|---|---|
| `0x351` | CVL / CCL / DCL / DVL |
| `0x355` | SOC / SOH |
| `0x356` | V / I / T |
| `0x359` | протокол, кількість акумуляторних модулів, мітка PN |
| `0x35C` | flags |
| `0x35E` | brand (ASCII) |
| `0x305` | heartbeat PCS: час (С=0, Хв, Год, Д, М, Р+2000; все 0 = годинник не виставлений) |

Кожні 5 с — рядок статистики: fps, total, TEC/REC, missed, bus state.
RTR-кадри пропускаються; приймальна черга TWAI = 64.

## План: MQTT → Home Assistant

Розширити прошивку: Wi-Fi STA + MQTT-клієнт, ті самі декодовані значення
(`0x351/355/356/359/35C/35E/305`) як сенсори Home Assistant (autodiscovery).
Заводську ESP-AT для цього повертати не треба — дамп у `backups/` на випадок
повного відкату. Реалізація — окремим кроком, поки не зроблено.
