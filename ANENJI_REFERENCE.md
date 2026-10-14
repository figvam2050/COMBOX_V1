# ANENJI ANJ-6200W-48V WIFI — зведений документ (OEM-платформа SRNE)

**Статус:** довідник. Рівень підтвердження джерел зазначено по секціях.
**Модель:** Anenji ANJ-6200W-48V WIFI (6.2 кВт, 48 В, гібрид on/off-grid, 230 В,
WiFi). Апарат у користувача **відсутній** (13.10.2026) — документ зібраний
із первинних джерел на випадок майбутнього апарату/тесту.
**OEM-платформа:** SRNE (ShenZhen ShuoRi New Energy), лінійки HF/HES; той самий
OEM у PowMr, Y&H, Anern. Гіпотеза «Voltronic PI30/PI18» **відкинута**
(14.10.2026): моніторинг — двійковий Modbus RTU SRNE, не текстовий QPIGS.

## 0. Джерела

| # | Джерело | Що дає | Статус звірки |
|---|---|---|---|
| S1 | Офіційний мануал ANJ-6200W-48V-WIFI (2024, 32 с., `cdn-files.myshopline.com/.../6-2kw.pdf`) | батарейний канал Pylon, програми меню, LCD-логіка, пінаути | звірено з PDF 13.10.2026 |
| S2 | `SRNE PV Inverter RS485 MODBUS Communication Protocol V1.00` (midnitesolar.com, 2017, 8 с.) | фреймінг: 9600 8N1, FC, CRC, помилки 0x01–0x0B | звірено з PDF 14.10.2026 |
| S3 | `Modbus register address of integrated inverter controller V1.3` (github jblance/mpp-solar, 6 с.) | повна мапа регістрів V1.3 | звірено з PDF 14.10.2026 |
| S4 | SRNE Modbus protocol **V1.7** (scribd 593797158) | пласка мапа `0x0100` (стан/Vbat/Ibat/SOC/PV/AC/temperature) | scribd закритий JS-челенджем; цифри зі зведення користувача — **потребують звірки на апараті** |
| S5 | `SRNE HF Series User Manual` (github krimsonkla/srne_ble_modbus) | LCD-коди аварій 01–29, захисти, §5.2 RS485/CAN-порт | звірено з MD 14.10.2026 |
| S6 | Попередні KB-розділи `DEYE_CAN_RS485_BASE.md` (13.10/14.10.2026), `ERRORS_REFERENCE.md` §5 | контекст проєкту, застереження про бренд-специфічність | — |

## 1. Батарейний канал (RS485 BMS) — Pylon, головне для COM-Box

- **Протокол:** Pylon RS485, діалект «PYLON US2000 Protocol 3.5» — меню 05 →
  тип батареї **Li2** (S1). Каналу CAN для батареї мануал не згадує
  (CAN на SRNE-платформі — опція, у стандарті відсутня, S5 §5.2).
- **Режими меню 05 (Battery type):** AGM / Flooded / User-Defined (свинець);
  **PYLON US2000 3.5 (Li2)**; «Standard communication Protocol from inverter
  supplier» (власний протокол батарей Anenji); **«Lithium battery without
  communication» (LIB)** — тупий літій: фіксовані 12/13/29 (аналог нашої
  Q5-дискусії «батарея без даних»).
- **Пінаут батарейного RJ45:** PIN1 = RS485-B, PIN2 = RS485-A, PIN7 =
  RS485-A, PIN8 = RS485-B (S1) — стандартний Pylon-кабель.
- **LCD за живої комунікації (S1):** іконка зв'язку, max напруга заряду, max
  струм заряду, SOC, «заборонено розряджати» (блимає 1 раз/с), «заборонено
  заряджати» (2 рази/с). Програми **43/44/45** (SOC-пороги) при живій
  комунікації **заміняють 12/13/29** — інвертор бере ліміти від батареї,
  як і Deye поважає CCL/CVL.
- **Wake-up:** програми 41 (авто-активація літія) / 42 (ручна активація).
- **DIP на батареї US2000 (за меню-інструкцією S1):** SW1 = baud 9600,
  SW2–SW4 = групова адреса; master-пак (перший) несе адресу групи.
- **Висновок для проєкту:** канал релевантний для **Q23** (коробка як
  Pylon-батарея/система для RS485-only інвертора, варіант «proxy»;
  `DEYE_CAN_RS485_BASE.md`, розділ від 13.10.2026). Anenji + Vision напряму
  теж має працювати (Pylon з обох боків) — коробка для базового зв'язку не
  обов'язкова, її цінність = агрегація лімітів, fail-closed, захист мережі.

## 2. Дата-порт (телеметрія) — SRNE Modbus RTU

### 2.1. Фізичний рівень і фреймінг (V1.00, S2 — звірено)

- **9600, 8N1**, Modbus RTU, стандартний **CRC16** (init `0xFFFF`, поліном
  `0xA001`, low byte першим).
- **Адреси:** `0x00` = broadcast; до `0xF7`; **універсальна `0xF7`** — slave
  відповідає без звірки зі своєю адресою (зручно для пошуку; не для
  multi-master). ≤32 слейви на шину.
- **Функції:** `0x03` читання, `0x10` запис («other: invalid»). Підтримка
  `0x06` (зі зведення V1.7) — **перевірити наживо**.
- **Ліміт:** N ≤ 20 регістрів (40 Б) на запит у специфікації кадру, але
  помилка `0AH` каже «>12» — на практиці **триматися ≤12 регістрів/запит**.
- **Slave ID:** за замовчуванням 1; у меню інвертора «ID Modbus» (програма
  25, `nod 25 001`); у V1.3 адреса/бод також налаштовуються через `0xE200`/
  `0xE201`.

### 2.2. ⚠️ CRC-застереження (перевірено 14.10.2026)

Поширений приклад `01 03 01 00 00 0A C5 F5` (PowMr-форуми / V1.7) має
**хибний CRC**. Правильний Modbus CRC16 для `01 03 01 00 00 0A` — **`C4 31`**
(перевірено: класичний вектор `01 03 00 00 00 01 → 84 0A` + C-алгоритм із
S2). Рахуйте CRC самостійно. Якщо пристрій відповідає лише на `C5 F5` — сам
факт нетипової CRC-перевірки фіксуйте як діагностику прошивки.

### 2.3. Дві ревізії мапи — НЕ ЗМІШУВАТИ

#### V1.3 (S3, звірено повністю) — «integrated inverter controller»

| Блок | Діапазон | Призначення |
|---|---|---|
| Product info | `0x000A`–`0x0020` | системна напруга/струм, тип виробу (`00` ctrl-home, `01` street, `03` inverter, **`04` integrated inverter controller**, `05` mains-freq off-grid), модель (8 слів), версії CPU/board, SN, RS485-адреса, версія протоколу, дата виробництва, сайт |
| Controller data | `0x0100`–`0x0121` | див. таблицю нижче |
| Inverter data | `0x0200`–`0x023B` | див. таблицю нижче |
| Control | `0xDF00`–`0xDF0F` | керування (⚠️ небезпечні записи, див. §4) |
| Battery params | `0xE000`–`0xE024` | параметри батареї (у таблиці V1.3 масштаб під 12 В — для 48 В системи перераховувати/перевіряти!) |
| User settings | `0xE200`–`0xE215` | RS485-адреса/бод, пароль, пріоритети, ліміти |
| Statistics | `0xF000`–`0xF3FF` | 7-денна історія PV/заряд/розряд (AH, kWh), добові підсумки, накопичувачі, лічильники |
| Fault history | `0xF800`–`0xFFFF` | 16 записів аварій × 16 слів + 16 записів станів × 16 слів |

**Controller data (P01):**

| Адреса | Назва | Формат |
|---|---|---|
| `0x100` | Battery SOC | % |
| `0x101` | Battery voltage | 0.1 В |
| `0x102` | Charge current (у батарею) | 0.1 А |
| `0x103` | Температури: high = контролер, low = батарея | 1 °C |
| `0x104–106` | DC-навантаження V / I (0.01 А) / P (Вт) | — |
| `0x107–109` | PV напруга / струм / потужність | 0.1 В / 0.1 А / Вт |
| `0x10A` | DC load on/off command | W |
| `0x10B` | Load/charge status | low: 00 off / 01 start / 02 mppt / 03 equalize / 04 boost / 05 float / 06 current limit; high (ctrl): b7 load, b0–b6 brightness |
| `0x10C–10D` | Контролер: біти аварій (B16 over-discharge … B30 MOS short) | 32 біти |

**Inverter data (P02):**

| Адреса | Назва | Формат |
|---|---|---|
| `0x200–203` | 64-бітова маска поточних аварій | hex |
| `0x204–207` | До 4 кодів аварій одночасно (0 = немає) | %d |
| `0x20C–20E` | Поточний час (RW): рік/міс, день/год, хв/сек (по 8 біт) | — |
| `0x210` | **Стан машини:** 0 power-up delay, 1 waiting, 2 init, 3 soft start, 4 mains, 5 inverter, 6 inverter→mains, 7 mains→inverter, 10 shutdown, 11 fault | %d |
| `0x211` | Password state (0 none / 1 user / 4 manufacturer) | — |
| `0x212` | Bus voltage | 0.1 В |
| `0x213–215` | Grid V (0.1 В) / I (0.1 А) / f (0.01 Гц) | — |
| `0x216–218` | Inverter output V / I / f | 0.1 В / 0.1 А / 0.01 Гц |
| `0x219–21C` | Load current / PF (знаковий 0.01) / active power / apparent power | — |
| `0x21D` | Inverter DC component | мВ, знаковий |
| `0x21E` | Mains charge current (струм батареї при мережевому заряді) | 0.1 А |
| `0x21F` | Load ratio | % |
| `0x220–222` | Радіатори A (DC-DC) / B (DC-AC) / C | 0.1 °C, знаковий |
| `0x223` | Ambient temperature | 0.1 °C, знаковий |
| `0x224–225` | Buck currents 1 / 2 (2-ге покоління) | 0.1 А |

**Battery params (P05) — ключове:** `E001` PV charge current limit (A);
`E002` nominal capacity (Ah); `E003` system voltage (12/24/36/48/FF auto);
**`E004` battery type (0–10)**; `E005` over voltage (0.1 В!); `E006` limited
charge voltage; `E007` equalize; `E008` boost / lithium overcharge; `E009`
float / lithium overcharge-return; `E00A` boost return; `E00B` over-discharge
return; `E00C` undervoltage warning; `E00D` over-discharge; `E00E` limited
discharge; `E00F` charge/discharge cut-off SOC (high/low byte); `E010`
over-discharge delay; `E011–013` equalize time/interval; `E014` temp
compensation; `E015–018` charge/discharge temp limits; `E019–01A` heating;
`E01B` mains switching voltage; **`E01C` stop charging current (тейпер
літію)**; `E020` series cell count; `E024` lithium activation current.

**User settings (P07):** **`E200` RS485 address (1–246)**; **`E201` baud
(48=4800 … 96=9600, 384=38400)**; `E202/E203` user password set/input;
`E204` output priority (0 solar / 1 line / 2 sbu); `E205` mains charge
current limit; `E206` equalize enable; `E207` eco threshold; `E208/E209`
output V/f; **`E20A` max charge current (0.1 А)**; `E20B` AC input range;
`E20C` eco mode; `E20D/E20E` overload/over-temp auto restart; `E20F` charge
priority (0 PV / 1 mains / 2 hybrid / 3 PV only); `E210/E211` alarm;
`E212` overload bypass; `E213` record fault code; `E214` split-phase.

**Fault history (P09):** запис = 16 слів: `+0x00` код аварії (0 = запис
недійсний), `+0x01–03` час, `+0x04–0F` 12 слів знімка даних у момент
аварії. Аналогічно status records від `0xF900`.

#### V1.7 (S4 — статус «потребує звірки на апараті»)

Пласка мапа, все від `0x0100`:

| Адреса | Назва | Формат |
|---|---|---|
| `0x100` | Machine state: 0 init, 1 standby, 2 inverter (АКБ), 3 mains/bypass, 4 fault | %d |
| `0x101` | Battery voltage | 0.1 В |
| `0x102` | Battery current (знаковий: + заряд, − розряд) | 0.1 А |
| `0x103` | Battery SOC | % |
| `0x104–106` | PV V / charge current / power | 0.1 В / 0.1 А / Вт |
| `0x107–108` | AC input V / I | 0.1 В / 0.1 А |
| `0x10B–10C` | AC output V / I | 0.1 В / 0.1 А |
| `0x10D–10E` | Output active / apparent power | Вт / ВА |
| `0x10F` | Load percentage | % |
| `0x113–114` | Температура радіатора / DCDC-MPPT | 1 °C, знаковий |
| `0xE000+` | Блок налаштувань батареї: тип `0xE004`, max charge current `0xE005`, low cut-off `0xE009`, float/bulk `0xE00B/0xE00C` (за зведенням — **зсунутий відносно V1.3, не довіряти V1.3-таблиці!**) | — |

### 2.4. Дискримінатор ревізій (один запит на живому апараті)

Прочитати **4 регістри з `0x0100`**: `адреса 03 01 00 00 04` + CRC.

- **V1.3** → `[SOC%, Vbat, Ichg, температурні байти]`;
- **V1.7** → `[режим 0–4, Vbat, Ibat, SOC%]`.

Якщо регістр 0 малий (0–11) і регістр 3 виглядає як відсоток → V1.7; якщо
регістр 0 = SOC (часто 20–100), а регістр 3 = температурні байти → V1.3.
Додатково: `01 03 02 10 00 01` (стан @`0x210`) — V1.7-пристрій швидше за все
відповість винятком `02H` (illegal address).

## 3. Система аварій (SRNE, S5 + S3)

### 3.1. LCD-коди (HF manual §6.2, повний перелік)

| Код | Назва | Вплив на вихід |
|---|---|---|
| 01 | BatVoltLow (undervoltage alarm) | так |
| 02 | BatOverCurrSw (розряд, софт) | так |
| 03 | BatOpen (батарея не підключена) | так |
| 04 | BatLowEod (undervoltage stop discharge) | так |
| 05 | BatOverCurrHw | так |
| 06 | BatOverVolt | так |
| 07 / 08 | BusOverVolt Hw / Sw | так |
| 09 | PvVoltHigh | ні |
| 10 / 11 | PvBuckOCSw / PvBuckOCHw | ні |
| 12 | bLineLoss (немає мережі) | ні |
| 13 / 14 | OverloadBypass / OverloadInverter | так |
| 15 | AcOverCurrHw | так |
| 17 | InvShort | так |
| 19 / 20 | OverTemperMppt / OverTemperInv | ні / так |
| 21 | FanFail | так |
| 22 | EEPROM | так |
| 23 | ModelNumErr | так |
| 26 | RlyShort (AC output backfill to bypass) | так |
| 29 | BusShort (boost circuit failure) | так |

### 3.2. Modbus-бік аварій (V1.3)

64-бітова маска `0x200–203` (кожен біт = одна аварія; призначений для
внутрішнього дебаг-тула) + до 4 паралельних кодів `0x204–207` + історія
`0xF800` зі 12-слівним знімком даних у момент аварії — діагностично
багатше за дейовський memory log (1000–1499).

### 3.3. Помилки протоколу (V1.00, S2)

`01H` illegal command; `02H` illegal data address; `03H` illegal data value;
`04H` operation failed; `05H` password error (V1.00: реєстр `0x1305`; V1.3:
`0xE202/0xE203`); `06H` data frame error (CRC/довжина); `07H` read-only;
`08H` не можна змінити в роботі; `09H` password protection (лок без
введення пароля); `0AH` length error; `0BH` permission denied.

## 4. ⚠️ Небезпечні записи (V1.3, ніколи не писати вліпую)

| Реєстр | Ефект |
|---|---|
| `0xDF00` = 0 | вимикає машину |
| `0xDF01` = 1 | reset |
| `0xDF02` = `0xAA` | **фабричне скидання** + перезавантаження |
| `0xDF03–05` | clear alarm / statistics / history |
| `0xDF08` = `0x5A5A` | sleep (прокидання: `0xA5A5`) |
| `0xE000`-блок | напруги заряду/захистів батареї |
| `0xE200/0xE201` | адреса/бод RS485 (зміна = втрата зв'язку з логером) |

## 5. Топологія портів і пінаути

| Канал | Протокол | RJ45 (мануал виробника) |
|---|---|---|
| Anenji дата-порт (RS485) | SRNE Modbus slave (хост-софт / Bluetooth / WiFi-стик) | **pin1 = 5V, pin2 = GND, pin7 = A, pin8 = B** (S5 §5.2) |
| Anenji CAN (опція, не в стандарті) | — | pin5 = CANH, pin6 = CANL |
| Anenji батарейний RS485 | Pylon master | **pin1 = B, pin2 = A, pin7 = A, pin8 = B** (S1) |
| COM-Box PCS-485 | UART4, слейв (майбутній Q23) | **pin5 = 485B, pin6 = 485A** (порт-мапинг коробки) |
| COM-Box PCS-CAN | CAN 500k | pin1 = GND, pin7 = CANL, pin8 = CANH |

**Увага:** пінаути дата-порту SRNE (7/8) і PCS-485 коробки (5/6) **різні** —
при майбутньому з'єднанні «коробка ↔ Anenji» потрібен перехідник за пінами.

## 6. Релевантність для проєкту COM-Box

1. **Q23 (другий RS485):** Anenji — перший реальний кандидат-інвертор, для
   якого варіант Б («відповіді зі стану» через PCS-порт) стає практичним:
   батарейний канал Pylon-RS485, дата-порт окремий. Рішення відкладено
   (KB 13.10.2026), поки апарату немає.
2. **Структурна аналогія з Deye:** інвертор = Modbus slave для логера +
   Pylon master для батареї; різниця лише в бренд-карті моніторингу
   (SRNE V1.3/V1.7 замість дейовської V104.1).
3. **`ERRORS_REFERENCE.md`** описує Deye-специфіку; SRNE-коди/реєстри — у
   цьому документі (§2–§3); застереження про бренд-специфічність — у
   `ERRORS_REFERENCE.md` §5.
4. **Інструменти:** `tools/rs485_sniff.py` працює як є (пасивний sniff
   трафіку WiFi-стика); активний тест — poll `адреса 03 01 00 00 04` з
   правильно рахованим CRC (§2.2, §2.4).

## 7. Відкриті питання / що перевірити на апараті

1. Яка саме ревізія мапи в ANJ-6200W: V1.3 чи V1.7 (дискримінатор §2.4).
2. Чи підтримується FC `0x06` (V1.00 каже «invalid», зведення V1.7 — так).
3. Реальний ліміт регістрів на запит: 12 чи 20.
4. Чому CRC-приклад `C5 F5` поширений — чи приймає пристрій його (нетипова
   прошивка) чи це суто документальна помилка.
5. Поведінка multi-pack на Pylon-каналі (1 master-пак + слейви, DIP-адреси).
6. Семантика E-блоку на 48 В (у V1.3-таблиці значення 9.0–15.5 В — масштаб
   12 В; для 48 В ×4 — перевірити перед будь-яким записом).
7. Підтвердження OEM-зв'язку Anenji↔SRNE на рівні прошивки (серійний,
   версія протоколу через `0x001C`/product info).

## 8. Історія документа

- **14.10.2026** — створено: зведення S1–S6, CRC-виправлення, дискримінатор
  ревізій, топологія портів, релевантність для Q23; гіпотеза Voltronic
  відкинута. Підтверджено з PDF: S1, S2, S3, S5; S4 — користувацьке
  зведення, очікує апарату.
