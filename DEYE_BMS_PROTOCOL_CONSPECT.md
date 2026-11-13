# Конспект протоколів обміну Deye (CAN-шина, RS485 BMS та Modbus RTU/TCP інвертора)

> Документ узагальнює вимоги та специфікації інверторів **Deye** на основі 5 джерел:
> 1. **SolarMark Modbus Reference:** `https://solarmark.us/modbus/deye/`
> 2. **RichardAntalik Deye Modbus Reference:** `https://github.com/RichardAntalik/deye-modbus-reference` (`04`, `06`, `07`, `08`, `09`, `10`)
> 3. **Psynosaur ESPHome Deye BMS + PCS CAN V3.3:** `https://github.com/Psynosaur/esphome-deye-bms` (`deye-can-bus.yaml` та `PCS CAN-Bus-protocol-DY-low-voltage V3.3.zh-CN.en.pdf`)
> 4. **BMS RS485-Modbus REV1.30:** `/Users/vlad/Downloads/document.pdf` (*BMS Protocol for Lithium Battery Pack RS485-Modbus REV1.30*)
> 5. **Deye Modbus Protocol V118:** `https://github.com/Hypfer/deye-microinverter-cloud-free-assets/blob/master/Deye_Modbus.-.-.V118.pdf`

---

## 1. Загальна архітектура інтерфейсів Deye

В екосистемі Deye існує **чотири різні комунікаційні ролі**, які важливо не змішувати:

| Інтерфейс | Роль інвертора | Швидкість / Параметри | Призначення |
| :--- | :--- | :--- | :--- |
| **BMS CAN (PCS CAN)** | Приймає телеметрію BMS + шле heartbeat | `500 kbps`, Standard 11-bit ID, цикл ~1 с | Основний зв'язок інвертор ↔ Master BMS (обмеження струмів/напруг, SOC, аварії). |
| **InterCAN (між батареями)** | Не бере участі (шина між паками) | `250 kbps`, Extended 29-bit ID | Обмін між модулями батарей Deye (RW-M6.1 / SE-G5.1Pro) для формування зведеного CAN на PCS. |
| **BMS RS485 (Порт BMS)** | **Modbus Master** (опитує BMS) | `9600 bps`, `8N1` (REV1.30) або ін. залежно від reg `325` | Альтернативний канал зв'язку інвертор ↔ BMS через RS485. |
| **Inverter Modbus (RS485/TCP)** | **Modbus Slave** (відповідає логеру / HA) | Modbus RTU (`9600/115200`) або Modbus TCP (`502`, Unit ID `1`) | Зовнішній моніторинг і керування інвертором (зокрема дзеркало даних BMS у регістрах `312–325`). |

---

## 2. Що інвертор Deye приймає та надсилає по CAN-шині (PCS CAN 500 kbps)

Джерело: `PCS CAN-Bus-protocol-DY-low-voltage V3.3.zh-CN.en.pdf` + `esphome-deye-bms`.

### 2.1. Запити / кадри від інвертора (Downlink `0x300..0x30F`)
Інвертор сам надсилає службові кадри на шину PCS CAN:
- **`0x305` (Heartbeat інвертора, кожну 1 с):**
  - `Byte 0`: Секунди (`0..59`)
  - `Byte 1`: Хвилини (`0..59`)
  - `Byte 2`: Години (`0..23`)
  - `Byte 3`: День (`1..31`)
  - `Byte 4`: Місяць (`1..12`)
  - `Byte 5`: Рік (зміщення `+2000`, напр. `26` → `2026`)
  - `Byte 6..7`: Зарезервовано
  - *Важливо:* Наявність регулярних кадрів `0x305` підтверджує BMS, що інвертор живий і слухає CAN.
- **`0x306`:** Керування/взаємодія з USB-модулем моніторингу.

### 2.2. Основні зведені кадри від BMS до інвертора (цикл 1 с, Little-Endian)

| CAN ID | Призначення | Байти та масштаб | Примітки |
| :---: | :--- | :--- | :--- |
| **`0x351`** | **Ліміти заряду/розряду** | `B0..1`: **Charge Voltage Limit** (`uint16`, `0.1 В`)<br>`B2..3`: **Charge Current Limit (off-grid/заг.)** (`int16`, `0.1 А`)<br>`B4..5`: **Discharge Current Limit (off-grid/заг.)** (`int16`, `0.1 А`)<br>`B6..7`: **Discharge Voltage Limit** (`uint16`, `0.1 В`) | Обов'язковий кадр. Інвертор жорстко обмежує струм і напругу за цими лімітами. |
| **`0x355`** | **SOC та SOH** | `B0..1`: **SOC** (`uint16`, `1 %`)<br>`B2..3`: **SOH** (`uint16`, `1 %`) | Обов'язковий кадр. |
| **`0x356`** | **Фактичні V, I, T системи** | `B0..1`: **Voltage** (`int16`, `0.01 В`)<br>`B2..3`: **Current** (`int16`, `0.1 А`)<br>`B4..5`: **Avg Cell Temp** (`int16`, `0.1 °C`) | **Критично (V2.5+ / V3.3):** у `0x356` **розряд = `+` (додатній), заряд = `-` (від'ємний)**! |
| **`0x358`** | **USB-плата (опційно)** | `B0..1`: USB Power (`0.1 Вт`)<br>`B2..5`: USB Energy (`0.1 Вт·год`)<br>`B6`: Switch (`0=Disable, 1=Enable`) | Спеціальний кадр для батарей з USB-портом. |
| **`0x359`** | **Захисти, аларми та системні помилки** | `B0..3`: Protection Tables 1–4<br>`B4..5`: Alarm Tables 5–6<br>`B6`: System Error Table 7 (`B7` не використовується) | Кожен біт `1` = активна подія, `0` = норма. Детальна розшифровка нижче. |
| **`0x35C`** | **Прапори запитів (Request Flags)** | `B0`: бітова маска керування зарядом/розрядом/підігрівом | Обов'язковий кадр для керування логікою заряду інвертора. |
| **`0x35E`** | **Ідентифікація та ємність** | `B0..3`: Manufacturer ASCII (`"DEYE"` або ім'я BMS)<br>`B4`: Кількість паків (`Battery Pack Number`)<br>`B5`: Виробник комірок (`1=GOTION, 2=CATL, 3=EVE`)<br>`B6..7`: Номінальна ємність системи (`uint16`, `0.1 А·год`) | Бажаний кадр для коректної ідентифікації та розрахунку ємності. |
| **`0x361`** | **Екстремуми комірок** | `B0..1`: Max Cell V (`uint16`, `0.001 В` / мВ)<br>`B2..3`: Min Cell V (`uint16`, `0.001 В` / мВ)<br>`B4..5`: Max Cell Temp (`int16`, `0.1 °C`)<br>`B6..7`: Min Cell Temp (`int16`, `0.1 °C`) | Використовується для розширеної діагностики. |
| **`0x363`** | **Версії ПЗ / заліза** | `B0..1`: Software version<br>`B2..3`: Hardware version | Береться з Master-модуля. |
| **`0x364`** | **Статус паралельних модулів** | `B0`: К-сть нормальних модулів<br>`B1`: Заборона заряду<br>`B2`: Заборона розряду<br>`B3`: Втрата зв'язку<br>`B4`: К-сть паралельних модулів | Агрегований статус стеку. |
| **`0x371`** | **Мережеві ліміти струму (On-Grid)** | `B0..1`: On-Grid Charge Current Limit (`int16`, `0.1 А`)<br>`B2..3`: On-Grid Discharge Current Limit (`int16`, `0.1 А`) | Доповнює `0x351` для мережевого режиму. |

### 2.3. Детальні бітові маски `0x359` (Захисти, Аларми, Помилки)
- **`Byte 0` (Protection Table 1):**
  - `Bit 0`: Cell Over Voltage (`Err 1`)
  - `Bit 1`: Cell Under Voltage (`Err 2`)
  - `Bit 2`: Module Over Voltage (`Err 3`)
  - `Bit 3`: Module Under Voltage (`Err 4`)
  - `Bit 4`: Charge Over Current (`Err 5`)
  - `Bit 5`: Discharge Over Current (`Err 6`)
  - `Bit 6`: Cell Over Temp (Charge) (`Err 7`)
  - `Bit 7`: Cell Under Temp (Charge) (`Err 8`)
- **`Byte 1` (Protection Table 2):**
  - `Bit 0`: Cell Voltage Over Difference (`Err 11`)
  - `Bit 1`: Cell Temp Over Difference (`Err 12`)
  - `Bit 2`: MOS Over Temp (`Err 13`)
  - `Bit 3`: AFE-OCD1 (`Err 15`) / AFE-OCD2 (`Err 16`)
  - `Bit 4`: Cell Over Temp (Discharge) (`Err 9`)
  - `Bit 5`: Cell Under Temp (Discharge) (`Err 10`)
  - `Bit 6`: Heating Film Over Temp (`Err 14`)
  - `Bit 7`: *(резерв / Heating Film)*
- **`Byte 2` (Protection Table 3 — AFE апаратні):**
  - `Bit 0`: AFE-UV (`Err 17`)
  - `Bit 1`: AFE-OV (`Err 18`)
  - `Bit 2`: AFE-OCC (`Err 20`)
  - `Bit 3`: AFE-SCD (`Err 21`)
  - `Bit 4`: AFE-UT (`Err 22`)
  - `Bit 5`: AFE-OT (`Err 23`)
  - `Bit 6`: AFE-SCDL (`Err 24`)
  - `Bit 7`: AFE-OCDL/OCD1/OCD2 (`Err 19`)
- **`Byte 3` (System Error Table 4):**
  - `Bit 0`: Cell voltage sampling fail (`Err 26`)
  - `Bit 1`: MOSFET short circuit (`Err 28`)
  - `Bit 2`: EEPROM error (`Err 29`)
  - `Bit 3`: AFE communication fail (`Err 25`)
  - `Bit 4`: Temperature sampling fail (`Err 27`)
  - `Bit 5`: Internal communication fail (`Err 30`)
  - `Bit 6`: PCS communication fail
  - `Bit 7`: Master address repeat (`Err 31`)
- **`Byte 4` (Alarm Table 5 — попередження):**
  - `Bit 0..7`: ті ж події, що й у Table 1 (Cell HV/LV, Module HV/LV, Charge/Discharge HC, Charge HT/LT), але на рівні **попередження (Alarm)**.
- **`Byte 5` (Alarm Table 6 — попередження):**
  - `Bit 0`: Cell voltage high difference (`Err 11`)
  - `Bit 1`: Cell temp high difference (`Err 12`)
  - `Bit 2`: MOS high temp (`Err 13`)
  - `Bit 3`: HeatMosAdhesion (`Err 1`)
  - `Bit 4`: HeatError (`Err 2`)
  - `Bit 5`: Cell High Temp (Discharge) (`Err 9`)
  - `Bit 6`: Cell Low Temp (Discharge) (`Err 10`)
  - `Bit 7`: Heating film high temp (`Err 14`)
- **`Byte 6` (System Error Table 7):**
  - `Bit 0`: OverConnectTemp
  - `Bit 1`: OverTerminalTemp
  - `Bit 2`: Pre Charge Failed (`Err 3`)
  - `Bit 3`: Charge Inversed (`Err 4`)
  - `Bit 4`: FUSE Blown (`Err 5`)
  - `Bit 5`: VOLT_OPEN_WIRE_FAIL (`Err 6`)
  - `Bit 6`: CHG_VOLT_LOW (`Err 8`)
  - `Bit 7`: TEMP_OPEN_WIRE_FAIL (`Err 7`)

### 2.4. Детальна розшифровка `0x35C` (Byte 0 — Request Flags)
- `Bit 7`: **Charge enable** (`1` = дозвіл заряду)
- `Bit 6`: **Discharge enable** (`1` = дозвіл розряду)
- `Bit 5`: **Request force charge I** (інвертор може вимкнути/пробудити батарею для примусового заряду при `SOC < 20%`, відновлення `> 23%`)
- `Bit 4`: **Request force charge II** (рекомендований: примусовий заряд без вимкнення батареї; інвертор має тримати гістерезис до розряду)
- `Bit 3`: **Request full charge** (якщо `SOC` не піднімався `≥ 97%` протягом 30 днів → `1`, щоб відкалібрувати SOC від мережі; скидається в `0` при `SOC ≥ 97%`)
- `Bit 2`: **Request heating** (запит підігріву)
- `Bit 1`: **Request heat (Banned)**
- `Bit 0`: **Full charge mark**

### 2.5. Багатомодульна телеметрія по CAN (`BmsAddr = 1..15`)
Для кожного пака з адресою `BmsAddr` на шину також передаються:
- `0x110 + (BmsAddr - 1)`: Fault (Bytes 0..6, Tables 1–7) + `Byte 7` стан MOS (`Bit 0` Parallel finish, `Bit 4` Charge MOS, `Bit 5` Discharge MOS, `Bit 6` Precharge MOS, `Bit 7` Heat MOS).
- `0x150 + (BmsAddr - 1)` .. `0x750 + (BmsAddr - 1)`: напруга, струм (розряд `+`, заряд `-`), SOC, SOH, ліміти, напруги комірок по кожному модулю, температури та серійний номер.

### 2.6. InterCAN (250 kbps, Extended 29-bit — `esphome-deye-bms`)
Шину InterCAN використовують модулі Deye між собою:
- **`0x02098001`..`0x0209800F`:** напруга пака (`0.1 В`), струм (`0.1 А`, офсет `30000`), температури (`0.1 °C`, офсет `400`).
- **`0x04008001`..`0x04078001`:** покоміркові напруги (`uint16`, `мВ`), по 4 комірки на кадр.

---

## 3. Що інвертор запитує та приймає по RS485 BMS (Protocol REV1.30 — `document.pdf`)

Коли в інверторі обрано режим RS485 BMS (див. регістр `325`), інвертор виступає **Modbus RTU Master**, а BMS — **Slave**.
У документі `/Users/vlad/Downloads/document.pdf` (*BMS Protocol for Lithium Battery Pack RS485-Modbus REV1.30*) описано базовий стандартний RS485-протокол:

- **Фізичний рівень:** `9600 bps`, `8 data bits`, `No parity`, `1 stop bit` (`8N1`).
- **Адреса Slave:** `0x00..0x0F` (0..15 через DIP-перемикач).
- **Запит, який надсилає хост (інвертор):**
  - **FC `0x04` (Read Collected / Input Registers):**
    `<Addr> 0x04 0x10 0x00 0x00 0x17 <CRC_L> <CRC_H>` — опитування одразу 23 регістрів (`0x1000..0x1016`) для адрес `0x00..0x0F`.
  - **FC `0x11` (Read Product Info):**
    `<Addr> 0x11 <CRC_L> <CRC_H>` — повертає рядок формату `Model*SW*HW*SN*`.

### 3.1. Карта регістрів `0x1000..0x1016` (Modbus addr `34097..34119`)

| Reg (Hex) | Параметр | Тип / Масштаб | Примітка |
| :---: | :--- | :--- | :--- |
| `0x1000` | **Pack Voltage** | `uint16`, `10 мВ` (`0.01 В`) | Загальна напруга батареї |
| `0x1001` | **Pack Current** | `int16`, `10 мА` (`0.01 А`) | Зі знаком (заряд/розряд) |
| `0x1002` | **Remaining Capacity** | `uint16`, `10 мА·год` | Залишкова ємність |
| `0x1003` | **Avg Cell Temperature** | `int16`, `0.1 °C` | Середня температура комірок |
| `0x1004` | **Env / Ambient Temp** | `int16`, `0.1 °C` | Температура середовища |
| `0x1005` | **Warning Flag** | `uint16` (бітова маска) | Попередження (Cell/Pack OV/UV, Temp, OC, Low SOC) |
| `0x1006` | **Protection Flag** | `uint16` (бітова маска) | Спрацьовані захисти (OVP, UVP, OCC, OCD, OTP, UTP, SCP) |
| `0x1007` | **Fault / Status Flag** | `uint16` (бітова маска) | Стан FET (Charge/Discharge/Limit), балансир, апаратні помилки (NTC, AFE, Fuse, MOS) |
| `0x1008` | **SOC** | `uint16`, `0.1 %` | Напр. `850` = `85.0 %` |
| `0x1009` | **SOH** | `uint16`, `0.1 %` | Напр. `1000` = `100.0 %` |
| `0x100A` | **Full Charged Capacity** | `uint16`, `10 мА·год` | Повна ємність |
| `0x100B` | **Cycle Count** | `uint16`, `1 цикл` | Лічильник циклів |
| `0x100C` | **Max Charge Current** | `uint16`, `10 мА` | Ліміт струму заряду |
| `0x100D` | **Max Cell Voltage** | `uint16`, `1 мВ` | Найвища комірка |
| `0x100E` | **Min Cell Voltage** | `uint16`, `1 мВ` | Найнижча комірка |
| `0x100F` | **Max Cell Temp** | `int16`, `0.1 °C` | Максимальна температура комірки |
| `0x1010` | **Min Cell Temp** | `int16`, `0.1 °C` | Мінімальна температура комірки |
| `0x1011` | **FET Temperature** | `int16`, `0.1 °C` | Температура силових ключів |
| `0x1012..0x1014` | Резерв / спец. дані | — | Залежить від ревізії |
| `0x1015` | **Nominal Float Voltage** | `uint16`, `10 мВ` | Номінальна буферна напруга |
| `0x1016` | **Design Capacity** | `uint16`, `10 мА·год` | Проєктна ємність |

---

## 4. Регістри Modbus самого інвертора Deye (Inverter = Slave)

Джерела: `Deye_Modbus.-.-.V118.pdf`, `RichardAntalik/deye-modbus-reference`, `solarmark.us/modbus/deye/`.

- **Протокол:** Modbus RTU або Modbus TCP (через Wi-Fi/LAN-логер, зазвичай порт `502`, Unit ID `1`).
- **Функції:**
  - `0x03` Read Holding Registers (до `125` регістрів за запит).
  - `0x10` Write Multiple Registers (до `123` регістрів за запит).

### 4.1. Дзеркало BMS в інверторі (`312–325`)
Ці регістри показують, **що саме інвертор реально отримав від BMS** (по CAN або RS485) і використовує у своїй логіці:

| Reg | Назва | Масштаб / Примітка |
| :---: | :--- | :--- |
| `312` | **BMS Charging Voltage** | `0.01 В` (з BMS `0x351`) |
| `313` | **BMS Discharging Voltage** | `0.01 В` (з BMS `0x351`) |
| `314` | **BMS Charge Current Limit (CCL)** | `1 А` (поточний ліміт заряду) |
| `315` | **BMS Discharge Current Limit (DCL)** | `1 А` (поточний ліміт розряду) |
| `316` | **BMS Real-time SOC** | `1 %` |
| `317` | **BMS Real-time Voltage** | `0.01 В` |
| `318` | **BMS Real-time Current** | `1 А` |
| `319` | **BMS Real-time Temp** | `0.1 °C` зі зміщенням `+1000` (`1250` = `25.0 °C`) |
| `320` | **BMS Max Charge Current** | `1 А` |
| `321` | **BMS Max Discharge Current** | `1 А` |
| `322` | **Lithium Alarm Flags** | Біти попереджень від BMS |
| `323` | **Lithium Fault Flags** | Біти аварій від BMS |
| `324` | **Lithium Symbol 2** | `Bit 1`: **Force Charge Request** (запит примусового заряду від BMS) |
| `325` | **Lithium Battery Type** | `0x0000` = **PYLON / SOLAX CAN**;<br>`0x0001..0x0009` = різні RS485-протоколи BMS;<br>`0x000A` = **Shenggao CAN** |

### 4.2. Інші критичні регістри батареї на інверторі

| Діапазон | Зміст |
| :---: | :--- |
| **`059`** | Стан та коди аварій інвертора (зокрема `F58` BMS Comm Fault тощо). |
| **`098–099`** | Бітові маски попереджень інвертора (Alarms). |
| **`106`** | Test/Comm прапори (`Bit 8`: Li-ion RS485 зв'язок активний, `Bit 9`: Li-ion CAN зв'язок активний). |
| **`182–191`** | Фактична телеметрія батареї, яку бачить інвертор: `182` Temp (`+1000`), `183` Voltage (`0.01 В`), `184` SOC (`%`), `190` Power (`Вт`), `191` Current (`0.01 А`). |
| **`200–240`** | Налаштування батареї в інверторі: `200` Control Mode (Lead/Lithium/NoBatt), `202` Capacity (`А·год`), `203–205` Equal/Absorb/Float V (`0.01 В`), `210–214` Max Charge/Discharge A, Low/Restart/Shutdown V/SOC, `240` Lithium Wake-Up. |
| **`500–589`** | Серійні номери (SN) підключених батарейних паків `Pack 1..15`. |
| **`600–809`** | Попакові дані `Pack 1..15` (по 14 регістрів на пак у V118: Voltage `0.01 В`, Current `0.1 А`, Avg Temp `1250=25°C`, SOC `0.1%`, Rem/Total Cap `0.1 А·год`, ChgV, ChgI, DsgI, Max/Min CellV `0.01 В`, Cycles, Warning, Fault). |
| **`10000+`** | Окрема Read-Only зона для фірмових батарей Deye (`Device Type`, `Protocol Version`, `Packnum`, `Force Charge Flag 10017`, `Check SOC Flag 10018`, `Battery Fault 1/2`, `Battery Alarm 1/2`, SN і покоміркові/пакові дані `10040+`). |

---

## 5. Аварії та попередження Deye, пов'язані з BMS та батареєю

Джерела: `09-alarm-codes.md`, `10-fault-codes.md`, `Deye_Modbus_V118.pdf`.

| Код | Назва | Значення та причина |
| :---: | :--- | :--- |
| **`F58`** | **BMS Communication Fault** | Втрата зв'язку з BMS по CAN або RS485 (інвертор не бачить валідних кадрів `0x351/0x355/0x356/0x35C` або відповідей RS485, або неправильний тип у reg `325`). |
| **`F56`** | **DC Busbar Voltage Too Low** | Напруга шини/батареї занадто низька (або BMS відключила розряд/ключі при розряженій батареї). |
| **`F46 / F49`** | **Backup Battery Fault** | Проблема з батареєю або її контактом/струмом під навантаженням у резервному режимі. |
| **`F40`** | **Batt_Over_Current** | Перевищення струму батареї. |
| **`F29`** | **Parallel CANBus Fault** | Помилка CAN-шини паралельної роботи інверторів. |
| **`F26`** | **DC Busbar Unbalanced** | Розбаланс шини постійного струму. |
| **`F20`** | **DC Over Current Fault** | Апаратне перевантаження по DC (PV або батарея). |
| **`F17 / F25`** | **Active_Battery_Hold / AC_Active_Batt_Fault** | Помилки активації/утримання батареї. |
| **`F07`** | **DC/DC Softstart Fault** | Помилка плавного старту DC/DC (перевірити запобіжник батареї). |
| **`W01..W04`** | **Warnings** | `W02` FAN_WARN, `W03` Grid phase wrong, `W04` meter_Comm_Fail (деталі в регістрах `098–099`). |

---

## 6. Практичний підсумок: що обов'язково потрібно інвертору Deye від BMS (по CAN)

Щоб Deye у режимі **Lithium (CAN, reg `325 = 0x0000`)** працював стабільно без `F58`:
1. **Швидкість та таймінг:** `500 kbps`, стандартні 11-бітні фрейми, інтервал відправки **`≤ 1 с`**, Little-Endian.
2. **Обов'язковий мінімум кадрів:**
   - **`0x351`**: Charge V (`0.1 В`), CCL (`0.1 А`), DCL (`0.1 А`), Discharge V (`0.1 В`).
   - **`0x355`**: SOC (`1 %`), SOH (`1 %`).
   - **`0x356`**: Voltage (`0.01 В`), Current (`0.1 А`, **розряд `+`, заряд `-`**), Temp (`0.1 °C`).
   - **`0x359`**: Біти захистів/алармів/системних помилок (`0` у нормі).
   - **`0x35C`**: Прапори дозволу заряду (`Bit 7`), розряду (`Bit 6`), примусового дозаряду (`Bit 4/5`) та калібрування (`Bit 3`).
   - **`0x35E`**: Рядок виробника (напр. `"DEYE"` або `"PYLO"`), кількість паків та номінальна ємність (`0.1 А·год`).
3. **Бажаний кадр для діагностики:** `0x361` (Max/Min Cell V в `мВ` і Max/Min Cell T в `0.1 °C`) та `0x371` (On-Grid CCL/DCL).
4. **Контроль зв'язку:** Відстежувати вхідний `0x305` від інвертора (раз на 1 с) для підтвердження живого каналу.
