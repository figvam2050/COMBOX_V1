# CAN-сніффер COM-Box (пасивний, listen-only)

Пасивний снифер шини CAN (500 kbit/s) для логування кадрів COM-Box ↔ інвертор.
Режим **listen-only**: ніколи не передає, не ставить ACK — безпечний підключ до
робочої шини. Два апаратні цілі в одному скетчі (`src/main.cpp`), вибір через env.

Див. також:
* [`../README.md`](../README.md) — основна прошивка COM-Box; §2 містить команди
  збірки/прошивки і паралельний RS485-снифер батарейної шини;
* [`../DEYE_CAN_RS485_BASE.md`](../DEYE_CAN_RS485_BASE.md) — база знань CAN/RS485
  Deye (декод кадрів 0x351…0x35E/0x305 нижче відповідає її розділам по CAN);
* [`../tools/rs485_sniff.py`](../tools/rs485_sniff.py) — пасивний RS485-снифер
  шини батарей (Vision/Modbus), для повного логування обох шин.

## Апаратні цілі

| env | Плата | Перетворювач | Піни |
|---|---|---|---|
| `esp32dev` | ESP32 (ESP-WROOM-32, LuaNode32) | SN65HVD230 (3.3 В) | R→GPIO4 (TWAI RX) |
| `nanoatmega328new` | Arduino Nano 5 В | MCP2515 + TJA1050 | SPI D10–D13, INT не потрібен (poll) |

### Шина COM-Box (RJ45 PCS-CAN)

- пін 8 = CANH, пін 7 = CANL, пін 1 = GND — до CANH/CANL/GND перетворювача.
  GND обов'язковий: спільна земля з шиною.

### Схема підключення SN65HVD230 (остаточна, 11.10.2026)

| SN65HVD230 | ESP32 / живлення | Примітка |
|---|---|---|
| 3V3 (VCC) | 3V3 | 2.3–3.6 В |
| GND | GND | + спільна земля з шиною (RJ45 пін 1) |
| R (RX) | GPIO4 | `twai_rx` |
| **D (TX) — не підключати до ESP32, а посадити на GND** | GND | постійний recessive; GPIO4/5 не дергаються, жодних домінантних імпульсів на шині під час reset (GPIO5 — strapping-пін) |
| Rs | GND (або через 10k) | швидкий режим, потрібен для 500 kbit/s |
| RE | GND (якщо виведений) | приймач постійно увімкнений |

Піни GPIO21/22/19/23 теж вільні, але GPIO4 у коді вже задіяний. Якщо в майбутньому
потрібна передача (напр. віртуальна батарея для інвертора) — D підключається до
GPIO5 (код на це вже налаштований), але тоді на період скидання можливий короткий
домінантний імпульс через strapping GPIO5; пом'якшується резистором ~1k + 10k
pull-down на D.

## Збірка і прошивка (ESP32)

```bash
cd can_sniffer
~/.platformio/penv/bin/pio run -e esp32dev                 # збірка
~/.platformio/penv/bin/pio run -e esp32dev -t erase        # стерти flash (за бажанням)
~/.platformio/penv/bin/pio run -e esp32dev -t upload       # прошивка (порт auto-detect)
~/.platformio/penv/bin/pio device monitor -p /dev/cu.usbserial-0001 -b 115200
```

- CP2102 на платі має **авто-reset** (DTR/RTS) — кнопку BOOT натискати не треба.
- Якщо `upload` не коннектиться (`failed to connect / timeout`): затиснути **BOOT**
  (IO0), коротко ткнути **RST/EN**, відпустити BOOT і повторити upload.
- Кабель має бути data (не зарядковий), драйвер CP2102 в macOS вбудований.

## Поточний стан плати (11.10.2026)

- Прошита збіркою `esp32dev` цього проєкту (erase + upload, 11.10.2026);
  банер у моніторі: `ESP32 TWAI CAN sniffer: 500 kbps, listen-only`.
- Заводська прошка ESP-AT (AT+WiFi/BLE, Espressif v2.3.0.0, збірка 20.05.2022)
  знята перед стиранням і лежить у `backups/`:
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
| `0x359` | протокол, к-сть паків, мітка |
| `0x35C` | flags |
| `0x35E` | brand (ASCII) |
| `0x305` | heartbeat PCS: SOC, V |

Кожні 5 с — рядок статистики: fps, total, TEC/REC, missed, bus state.
RTR-кадри пропускаються; приймальна черга TWAI = 64.

## План: MQTT → Home Assistant

Розширити прошивку: Wi-Fi STA + MQTT-клієнт, ті самі декодовані значення
(`0x351/355/356/359/35C/35E/305`) як сенсори Home Assistant (autodiscovery).
Стокову ESP-AT для цього повертати не треба — дамп у `backups/` на випадок
повного відкату. Реалізація — окремим кроком, поки не зроблено.
