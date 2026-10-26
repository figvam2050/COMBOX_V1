/**
 * BMS 485 to CAN Converter for Deye Inverter (Lithium Mode 00 - Deye generic
 * CAN / Pylontech). Hardware: GD32F305 + ISO1050 (CAN) + CA-IS3080WX (RS485)
 *
 * Frame set and layout follow "PCS CAN-Bus-protocol-DY-low-voltage V3.3" and
 * "PYLON low voltage Protocol CAN Bus v2.0.6": standard frames, 500 kbit/s,
 * cycle 1 s, BMS transmits 0x351/0x355/0x356/0x359/0x35C/0x35E.
 * IDs 0x300..0x30F are reserved for the PCS downlink (0x305 is the inverter
 * heartbeat) and must never be transmitted by the battery side.
 *
 * CAN bus speed: 500 kbit/s, measured manually on real hardware (BTR = 0x030A0000).
 * The 250 kbit/s value was never confirmed and is not a valid option.
 *
 * SPDX-License-Identifier: CC-BY-NC-SA-4.0
 * Copyright (C) 2026 FigVam
 */

#include "gd32f30x.h"
#include <stdbool.h>
#include <string.h>

#define BMS_BAUDRATE 9600
#define BMS_TIMEOUT_MS 400         // 400 ms: matches Deye RS485 turnaround window for slow BMS
#define BMS_RESPONSE_LEN 83
#define BMS_POLL_PERIOD_MS 400     // 400 ms per query: matches Deye inverter polling cadence
#define BMS_POLL_GAP_MS 20         // 20 ms minimum bus quiet gap between RS485 transactions
#define BMS_DEFAULT_TIMEOUT_MS 8000 // 8 s fail-closed: covers full 16-addr sweep (16 * 400 ms = 6.4 s)

const uint8_t BMS_QUERY_TABLE[16][8] = {
    {0x01, 0x03, 0x00, 0x00, 0x00, 0x27, 0x05, 0xD0},
    {0x02, 0x03, 0x00, 0x00, 0x00, 0x27, 0x05, 0xE3},
    {0x03, 0x03, 0x00, 0x00, 0x00, 0x27, 0x04, 0x32},
    {0x04, 0x03, 0x00, 0x00, 0x00, 0x27, 0x05, 0x85},
    {0x05, 0x03, 0x00, 0x00, 0x00, 0x27, 0x04, 0x54},
    {0x06, 0x03, 0x00, 0x00, 0x00, 0x27, 0x04, 0x67},
    {0x07, 0x03, 0x00, 0x00, 0x00, 0x27, 0x05, 0xB6},
    {0x08, 0x03, 0x00, 0x00, 0x00, 0x27, 0x05, 0x49},
    {0x09, 0x03, 0x00, 0x00, 0x00, 0x27, 0x04, 0x98},
    {0x0A, 0x03, 0x00, 0x00, 0x00, 0x27, 0x04, 0xAB},
    {0x0B, 0x03, 0x00, 0x00, 0x00, 0x27, 0x05, 0x7A},
    {0x0C, 0x03, 0x00, 0x00, 0x00, 0x27, 0x04, 0xCD},
    {0x0D, 0x03, 0x00, 0x00, 0x00, 0x27, 0x05, 0x1C},
    {0x0E, 0x03, 0x00, 0x00, 0x00, 0x27, 0x05, 0x2F},
    {0x0F, 0x03, 0x00, 0x00, 0x00, 0x27, 0x04, 0xFE},
    {0x10, 0x03, 0x00, 0x00, 0x00, 0x27, 0x06, 0x91}};

static uint16_t modbus_crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// No-data presentation (Q5, рішення 13.10.2026): «порожня інертна батарея».
// SOC=0 — не запрошує інвертор до розряду; T=25 — нейтральна (45 °C могла
// тригерити derating/тривоги); SOH=100 — без фейкового «старіння»; V=52.2
// залишається номіналом (V=0 лякає інвертори; при лімітах 0 А не впливає).
// Разом із CCL/DCL=0 і 0x35C=0x00 (без прапорців) — чесний стан «нема даних».
#define BATTERY_SOC_DEFAULT 0
#define BATTERY_SOH_DEFAULT 100
#define BATTERY_VOLTAGE_DEFAULT 52.2f
#define BATTERY_CURRENT_DEFAULT 0.0f
#define BATTERY_TEMP_DEFAULT 25.0f
#define CHARGE_CURRENT_LIMIT_DEFAULT 0
#define DISCHARGE_CURRENT_LIMIT_DEFAULT 0

#define CHARGE_VOLTAGE_LIMIT_STATIC 540
#define CHARGE_VOLTAGE_LIMIT_DIP90 518
#define CHARGE_VOLTAGE_LIMIT_FLOAT_HOLD 516 // 51.6 V (3.44 V/cell * 15): hold voltage during balancing

// Per-pack charge current limit, selected by DIP5 (see DIP_CHARGE_CURRENT_50A):
// OFF = 40 A (soft margin), ON = 50 A (battery manufacturer maximum).
#define CHARGE_CURRENT_LIMIT_PER_PACK_DIP5_OFF_X10 400
#define CHARGE_CURRENT_LIMIT_PER_PACK_X10 500
#define DISCHARGE_CURRENT_LIMIT_PER_PACK_X10 1000
#define DISCHARGE_VOLTAGE_LIMIT_X10 450

// Max-cell charge taper: 50 A/pack to 0 A between 3.450 V and 3.600 V.
// Re-enabling below 3.550 V prevents charge-limit oscillation at the cutoff.
#define CELL_TAPER_START_MV 3450
#define CELL_CHARGE_STOP_MV 3600
#define CELL_CHARGE_REENABLE_MV 3550
#define CELL_TAPER_START_MV_DIP90 3400
#define CELL_CHARGE_STOP_MV_DIP90 3455
#define CELL_CHARGE_REENABLE_MV_DIP90 3405
// DIP1 OFF (default) = standard 54.0 V mode (100% profile, full thresholds);
// DIP1 ON = 51.8 V option (90% profile, economy thresholds, auto balance
// sessions disabled, trickle cap from 3400 mV, LED1 solid indicator).
// Bit moved from SW4 on 17.10.2026 (convenient end switch, silkscreen "SW1"
// near the PCS-485/CAN jacks); stock SW1 protocol meaning (PYLON) unused.
#define DIP_PROFILE_CODE_90 1
// DIP5 (SW5, weight 16, PC6) selects the per-pack charge current limit:
// OFF = 40 A, ON = 50 A. Orthogonal to the SW1 voltage profile.
#define DIP_CHARGE_CURRENT_50A 0x10
// DIP3 (SW3, weight 4, PB14): fast polling mode (active packs only)
// OFF = full scan across all 16 addresses, ON = fast polling latched active packs.
#define DIP_FAST_POLL_MASK 0x04
#define BG_PROBE_INTERVAL_MS 45000UL // background probe every 45 s for auto-discovering new packs

// Physical cell over-voltage threshold for BMS hardware protection confirmation (Q17).
// Independent of DIP90 taper target so that software 90% profile never generates false alarms:
#define CELL_PHYSICAL_OV_CONFIRM_MV 3600
#define CELL_PHYSICAL_OV_WARN_CONFIRM_MV 3550

// Vision V-LFP48100 cell protection thresholds with proactive inverter cut-off:
// BMS hardware thresholds (BMS_TOOLS): Warn 2800 mV, Protect 2500 mV, Release 3000 mV.
// COM-Box operates proactively ahead of BMS to prevent hard contactor cuts under load:
#define CELL_UV_WARN_MV 2900            // Soft Warning: 50% DCL + 0x359 Warning Cell UV (proactive)
#define CELL_DISCHARGE_STOP_MV 2750     // Soft Cut-off: DCL = 0 + 0x359 Protect Cell UV (250 mV before BMS 2500 mV)
#define CELL_DISCHARGE_REENABLE_MV 3000 // Release: restore discharge when recharged to 3000 mV (matches BMS)
#define CELL_MAX_DELTA_MV 400

// ===== SOC з напруги комірок (P1.1, 22.10.2026) =====
// Заводський SOC Vision (рег 24) мертвий — зависає на 100% навіть за
// розряду до 3.43 В/комірку (польові логи 22.10: SOC=100 при мін 3425 мВ).
// Тому 0x355 SOC рахується з мінімальної (найгіршої) комірки за LFP
// OCV-таблицею (25 °C) з EMA-згладжуванням (α=1/8 ≈ 8 sweep-ів). Мінімальна
// комірка — консервативний вибір: інвертор ніколи не бачить більше, ніж є.
// Обмеження: плоска ділянка LFP 3.33–3.38 В дає похибку ±10%; просідання
// напруги під навантаженням тимчасово занижує SOC — EMA вирівнює.
// Валідність лише для 15S LFP (Vision); для інших хімій/серий — своя таблиця.
// Відкалібровано за реальними фактами акумуляторів Vision V-LFP48100:
// 3285 мВ = 31% (скріншот BMS_TOOLS), 3300 мВ = 50% (дамп 3297 мВ = 49-50%).
static const struct {
  uint16_t mv;
  uint8_t soc;
} lfp_ocv_table[] = {
    {3600, 100}, // 100% — верхня точка повного заряду (Float hold 3.56..3.60 В)
    {3450, 95},
    {3400, 90},  // 90% — початок балансування (балансир вмикається від 3.400 В)
    {3340, 75},
    {3320, 60},
    {3300, 50},  // 50% при 3.300 В (підтверджено дампом 08.10: 3297 мВ = 49%)
    {3285, 31},  // 31% при 3.285 В (підтверджено скріном BMS_TOOLS: 31% / 31 А·ч)
    {3250, 20},
    {3200, 10},  // початок стрімкого завалу напруги LFP
    {3000, 5},   // перехід до коліна розряду (Release поріг 3000 мВ)
    {2900, 0},   // 0% — Soft Warning поріг (2900 мВ)
};
static int16_t soc_ema_x8 = 0;   // SOC у 1/8 % для EMA без дробів
static bool soc_ema_valid = false;

static uint8_t soc_from_cell_mv(uint16_t cell_mv) {
  uint8_t n = sizeof(lfp_ocv_table) / sizeof(lfp_ocv_table[0]);
  if (cell_mv >= lfp_ocv_table[0].mv)
    return 100;
  if (cell_mv <= lfp_ocv_table[n - 1].mv)
    return 0;
  for (uint8_t i = 1; i < n; i++) {
    if (cell_mv >= lfp_ocv_table[i].mv) {
      uint16_t mv_hi = lfp_ocv_table[i - 1].mv;
      uint16_t mv_lo = lfp_ocv_table[i].mv;
      uint8_t soc_hi = lfp_ocv_table[i - 1].soc;
      uint8_t soc_lo = lfp_ocv_table[i].soc;
      return (uint8_t)(soc_lo +
                       (uint16_t)(soc_hi - soc_lo) * (cell_mv - mv_lo) /
                           (mv_hi - mv_lo));
    }
  }
  return 0;
}

// Temperature thresholds: LiFePO4 charging blocked < +2°C (lithium plating) and > 55°C.
// Charging re-enables with hysteresis (>= +3°C and <= 50°C) to prevent chattering.
#define TEMP_CHARGE_MIN_C 2.0f
#define TEMP_CHARGE_MIN_REENABLE_C 3.0f
#define TEMP_CHARGE_MAX_C 55.0f
#define TEMP_CHARGE_MAX_REENABLE_C 50.0f
#define TEMP_DISCHARGE_MIN_C -20.0f
#define TEMP_DISCHARGE_MIN_REENABLE_C -18.0f
#define TEMP_DISCHARGE_MAX_C 60.0f
#define TEMP_DISCHARGE_MAX_REENABLE_C 58.0f

// ===== Balance session engine (17.10.2026) =====
// DIP1 OFF (standard): CVL 54.0 V; auto balance windows (BAL_AUTO_ENABLED)
// open on cell-spread analysis: full thresholds + 2 A per pack cap.
// DIP1 ON (51.8 V option): auto logic disabled, profile holds 51.8 V with
// the same 2 A trickle from 3400 mV (~90%) so passive balancing has gentle
// CV time.
#define BAL_AUTO_ENABLED 1 // verdict (A) 18.10.2026: weak cell #7 confirmed
#define BAL_SPREAD_HIGH_MV 100     // anomaly bucket (>= 100 mV): retry in 24 h
#define BAL_SPREAD_MED_MV 30       // regular spread bucket (30..100 mV): retry in 72 h
#define BAL_SPREAD_EXIT_MV 25      // session success target (spread <= 25 mV)
#define BAL_TREND_DELTA_MV_12H 5   // 5 mV over 12 h == 10 mV/24 h -> HIGH
#define BAL_INTERVAL_HIGH_MS (24UL * 3600UL * 1000UL)
#define BAL_INTERVAL_MED_MS (72UL * 3600UL * 1000UL)
#define BAL_INTERVAL_OK_MS (30UL * 24UL * 3600UL * 1000UL)
#define BAL_COOLDOWN_TRY_MS (12UL * 3600UL * 1000UL)
#define BAL_COOLDOWN_OUTAGE_MS (1UL * 3600UL * 1000UL)
#define BAL_MIN_SOC_START 85
#define BAL_TRICKLE_CELL_MV 3400   // ~90% -> 2 A per pack phase
#define BAL_TRICKLE_A_X10 20       // 2.0 A per pack cap
#define BAL_HOLD_CELL_MV 3560      // Float hold trigger: max cell reaches 3.56 V
#define BAL_HOLD_RELEASE_MV 3530   // Float hold release: max cell relaxes below 3.53 V
#define BAL_HOLD_A_X10 10          // 1.0 A per pack holding cap during float hold
#define CELL_EMERGENCY_OV_MV 3620  // Hard emergency cutoff during balance session
#define BAL_NO_PROGRESS_MS (90UL * 60UL * 1000UL)
#define BAL_SESSION_MAX_MS (8UL * 3600UL * 1000UL)
#define BAL_START_I_MAX_A 1.0f     // start only when not discharging
#define BAL_OUTAGE_I_MIN_A 1.5f    // sustained discharge -> abort session
#define BAL_OUTAGE_AGG_N 19        // ~120 s in 6.4 s sweep (or ~15 s in Fast Polling 2-pack mode)
#define BAL_EXIT_STREAK 3          // consecutive aggregates with spread <= exit
#define BAL_TEMP_MIN_C 0.0f
#define BAL_TEMP_MAX_C 45.0f
#define BAL_HIST_HOURS 24          // hourly spread history for trend

// 0x359 Pylontech v2.0.6 Protection & Alarm bit definitions
#define CAN_PROT_CELL_OV       0x01
#define CAN_PROT_PACK_OV       0x02
#define CAN_PROT_CELL_UV       0x04
#define CAN_PROT_OT            0x08
#define CAN_PROT_UT            0x10
#define CAN_PROT_DISCHARGE_OC  0x80

#define CAN_PROT_CHARGE_OC     0x01
#define CAN_PROT_SYSTEM_ERROR  0x08

// RS485 register units -> physical units. Confirm against the battery model.
// reg1 current: 1 = BMS reports charge as positive (negated for Deye CAN),
//               0 = BMS reports discharge as positive (passed through).
#define BMS_CURRENT_CHARGE_IS_POSITIVE 1
// reg19 temperature raw divisor: 1 = reg 0x13 holds whole C (dump 08.10.2026).
#define BMS_TEMP_RAW_DIV 1.0f

#define CAN_ID_LIMITS 0x351
#define CAN_ID_SOC 0x355
#define CAN_ID_STATUS 0x356
#define CAN_ID_PROTECT 0x359
#define CAN_ID_FLAGS 0x35C
#define CAN_ID_BRAND 0x35E

#define UART_BMS USART0
#define UART_BMS_RCU RCU_USART0
#define UART_BMS_GPIO_PORT GPIOA
#define UART_BMS_GPIO_RCU RCU_GPIOA
#define UART_BMS_TX_PIN GPIO_PIN_9
#define UART_BMS_RX_PIN GPIO_PIN_10

// PCS-485 port (UART4, PC12/PD2 — PORT_MAPPING.md §3/§5; no UART5 on GD32F30x)
#define PCS_UART UART4
#define PCS_UART_RCU RCU_UART4
#define PCS_TX_PORT GPIOC
#define PCS_TX_RCU RCU_GPIOC
#define PCS_TX_PIN GPIO_PIN_12
#define PCS_RX_PORT GPIOD
#define PCS_RX_RCU RCU_GPIOD
#define PCS_RX_PIN GPIO_PIN_2

#define DEYE_CAN CAN0
#define DEYE_CAN_RCU RCU_CAN0
#define DEYE_CAN_GPIO_PORT GPIOB
#define DEYE_CAN_GPIO_RCU RCU_GPIOB
#define DEYE_CAN_RX_PIN GPIO_PIN_8
#define DEYE_CAN_TX_PIN GPIO_PIN_9

// LED assignment per PORT_MAPPING.md §4 (factory panel labels, 17.10.2026):
//   LED5 (PB0)  = system running (program alive) — steady ON after init;
//   LED6 (PB1)  = BMS RS485 exchange — blinks on TX/RX with battery;
//   LED4 (PC5)  = CAN to inverter — blinks on TX bursts;
//   LED3 (PC4)  = PCS-485 port (UART4, Q23): blinks on TX/RX — mirror mode
//                 (DIP2 OFF) or inverter bridge traffic (DIP2 ON);
//   LED1 (PC13) = free/debug — dark;
//   LED2 (VDDA) = power — wired to rail, not MCU-controlled.
// PC5 is NOT an RS485 direction pin (PC5=DE disproven, RS485 has
// auto-direction transceivers — ORIGINAL_FIRMWARE.md §14.3).
#define LED_RUNNING_PORT GPIOB
#define LED_RUNNING_PIN GPIO_PIN_0
#define LED_BMS485_PORT GPIOB
#define LED_BMS485_PIN GPIO_PIN_1
#define LED_PCSCAN_PORT GPIOC
#define LED_PCSCAN_PIN GPIO_PIN_5
#define LED_INV485_PORT GPIOC
#define LED_INV485_PIN GPIO_PIN_4
#define LED1_DEBUG_PORT GPIOC
#define LED1_DEBUG_PIN GPIO_PIN_13

// Stock boot state for PB10 (PP HIGH) and PB11 (OD HIGH) — likely transceiver
// enables (ORIGINAL_FIRMWARE.md §14.4); released HIGH like the factory firmware.
#define TRANSCEIVER_ENA_PORT GPIOB
#define TRANSCEIVER_ENA_PIN GPIO_PIN_10
#define TRANSCEIVER_ENB_PORT GPIOB
#define TRANSCEIVER_ENB_PIN GPIO_PIN_11

#define DIP_GPIO_PORT GPIOB
#define DIP_GPIO_RCU RCU_GPIOB
#define DIP_GPIO_PINS (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
#define DIP5_GPIO_PORT GPIOC
#define DIP5_GPIO_RCU RCU_GPIOC
#define DIP5_GPIO_PIN GPIO_PIN_6

static volatile uint32_t g_systick_ms = 0;
static volatile uint8_t uart_rx_buffer[256];
static volatile uint16_t uart_rx_index = 0;

static float bms_voltage = BATTERY_VOLTAGE_DEFAULT;
static float bms_current = BATTERY_CURRENT_DEFAULT;
static uint8_t bms_soc = BATTERY_SOC_DEFAULT;
static float bms_temp = BATTERY_TEMP_DEFAULT;
static uint8_t bms_soh = BATTERY_SOH_DEFAULT;
// Packs seen in the last full cycle; kept (not cleared) on comm loss so
// 0x359 byte 4 stays the fixed system size the inverter expects.
static uint8_t bms_pack_count = 0;

static float agg_voltage_sum = 0.0f;
static float agg_current_sum = 0.0f;
static uint32_t agg_soh_sum = 0;
static float agg_temp_max = -1000.0f;
static float agg_temp_min = 1000.0f;
static uint8_t agg_count = 0;
static uint16_t agg_max_cell_mv = 0;
static uint16_t agg_min_cell_mv = 0xFFFF;
static bool charge_cell_ov_inhibit = false;
static bool charge_temp_inhibit = false;
static bool discharge_cell_uv_inhibit = false;
static bool discharge_temp_inhibit = false;

// 0x359 CAN protection and alarm flags (active state and sweep accumulators)
static uint8_t can_protect_b0 = 0;
static uint8_t can_protect_b1 = 0;
static uint8_t can_alarm_b2 = 0;
static uint8_t can_alarm_b3 = 0;
static uint8_t sweep_protect_b0 = 0;
static uint8_t sweep_protect_b1 = 0;
static uint8_t sweep_alarm_b2 = 0;
static uint8_t sweep_alarm_b3 = 0;

// Multi-pack debounce (Q21): tolerate 10-20% RS485 packet loss without flapping limits
static uint8_t pack_miss_count[16] = {0};
static uint16_t pack_ever_seen_mask = 0;
static uint16_t sweep_seen_mask = 0;

static uint16_t charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_STATIC;
static uint16_t charge_current_limit = CHARGE_CURRENT_LIMIT_DEFAULT;
static uint16_t discharge_current_limit = DISCHARGE_CURRENT_LIMIT_DEFAULT;

static uint16_t profile_charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_STATIC;
static uint16_t profile_taper_start_mv = CELL_TAPER_START_MV;
static uint16_t profile_charge_stop_mv = CELL_CHARGE_STOP_MV;
static uint16_t profile_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
static uint16_t profile_charge_current_per_pack_x10 = CHARGE_CURRENT_LIMIT_PER_PACK_X10;

// Active thresholds seen by calculate_charge_limit(): a running balance
// session swaps the 90% profile taper for the full-charge triple, otherwise
// they mirror the current DIP profile.
static uint16_t calc_taper_start_mv = CELL_TAPER_START_MV;
static uint16_t calc_charge_stop_mv = CELL_CHARGE_STOP_MV;
static uint16_t calc_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;

// Balance session engine state
static bool bal_session_active = false;
static bool bal_trickle = false;
static bool bal_float_hold = false;
static uint16_t bms_balance_mask = 0;
static uint16_t sweep_balance_mask = 0;
static bool bal_manual_prev = false;
static uint32_t bal_last_success_ms = 0;
static uint32_t bal_cooldown_until_ms = 0;
static uint32_t bal_session_start_ms = 0;
static uint32_t bal_last_improve_ms = 0;
static uint16_t bal_best_spread_mv = 0xFFFF;
static uint8_t bal_exit_streak = 0;
static uint8_t bal_outage_agg = 0;
static uint8_t bal_prev_packs = 0;
static uint16_t bal_hist_mv[BAL_HIST_HOURS];
static uint8_t bal_hist_n = 0;
static uint32_t bal_last_hist_ms = 0;

// Fast polling state (DIP3 / SW3)
static bool fast_poll_enabled = false;
static uint16_t fast_poll_mask = 0;
static uint32_t last_bg_probe_ms = 0;
static uint8_t bg_probe_candidate = 0;
static bool bg_probe_in_flight = false;
static uint8_t current_addr_index = 0;

static uint8_t dip_applied_code = 0xFF;
static uint8_t dip_last_code = 0;
static uint8_t dip_stable_reads = 0;

static volatile uint32_t last_bms_response_ms = 0;
// True while BMS answers are fresh (< BMS_DEFAULT_TIMEOUT_MS): drives the
// 0x35C request flags (Q5). False at boot and on comm loss.
static bool bms_data_valid = false;
static uint32_t can_led_off_ms = 0;

extern "C" void osSystickHandler(void) { g_systick_ms++; }
static uint32_t millis(void) { return g_systick_ms; }

static void leds_init(void) {
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  // LED5 (PB0): system running — steady ON (program alive).
  gpio_init(LED_RUNNING_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_RUNNING_PIN);
  gpio_bit_set(LED_RUNNING_PORT, LED_RUNNING_PIN);
  // LED6 (PB1): BMS RS485 exchange — blinks on TX/RX (set at poll TX,
  // cleared when the response window closes).
  gpio_init(LED_BMS485_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_BMS485_PIN);
  gpio_bit_reset(LED_BMS485_PORT, LED_BMS485_PIN);
  // LED4 (PC5): CAN to inverter — blinks on TX bursts.
  gpio_init(LED_PCSCAN_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_PCSCAN_PIN);
  gpio_bit_reset(LED_PCSCAN_PORT, LED_PCSCAN_PIN);
  // LED3 (PC4): PCS-485 (UART4) activity — mirror/bridge, see pcs_poll_service().
  gpio_init(LED_INV485_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_INV485_PIN);
  gpio_bit_reset(LED_INV485_PORT, LED_INV485_PIN);
  // LED1 (PC13): free/debug — dark.
  gpio_init(LED1_DEBUG_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED1_DEBUG_PIN);
  gpio_bit_reset(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
  // Transceiver enables: match factory boot state.
  gpio_init(TRANSCEIVER_ENA_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            TRANSCEIVER_ENA_PIN);
  gpio_bit_set(TRANSCEIVER_ENA_PORT, TRANSCEIVER_ENA_PIN);
  gpio_init(TRANSCEIVER_ENB_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_2MHZ,
            TRANSCEIVER_ENB_PIN);
  gpio_bit_set(TRANSCEIVER_ENB_PORT, TRANSCEIVER_ENB_PIN);
}

static void rs485_init(void) {
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(UART_BMS_GPIO_RCU);
  rcu_periph_clock_enable(UART_BMS_RCU);
  gpio_init(UART_BMS_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
            UART_BMS_TX_PIN);
  gpio_init(UART_BMS_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,
            UART_BMS_RX_PIN);
  usart_deinit(UART_BMS);
  usart_baudrate_set(UART_BMS, BMS_BAUDRATE);
  usart_parity_config(UART_BMS, USART_PM_NONE);
  usart_word_length_set(UART_BMS, USART_WL_8BIT);
  usart_stop_bit_set(UART_BMS, USART_STB_1BIT);
  usart_transmit_config(UART_BMS, USART_TRANSMIT_ENABLE);
  usart_receive_config(UART_BMS, USART_RECEIVE_ENABLE);
  usart_enable(UART_BMS);
}

static void rs485_send_byte(uint8_t data) {
  usart_data_transmit(USART0, data);
  while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET)
    ;
}

static void rs485_clear_errors(void) {
  usart_flag_clear(USART0, USART_FLAG_ORERR);
  usart_flag_clear(USART0, USART_FLAG_NERR);
  usart_flag_clear(USART0, USART_FLAG_FERR);
  usart_flag_clear(USART0, USART_FLAG_PERR);
}

static void pcs_mirror_push(uint8_t b); // defined with the PCS-485 block below

static void rs485_flush_rx(void) {
  while (usart_flag_get(USART0, USART_FLAG_RBNE) != RESET)
    pcs_mirror_push((uint8_t)usart_data_receive(USART0)); // залишкові байти обірваного кадру
  rs485_clear_errors();
}

// ===== PCS-485 port (UART4, Q23) — two modes selected by DIP2 (18.10.2026) =====
// DIP2 OFF (default): MIRROR — the box works as always (own polls, CAN) and a
//   byte-level copy of its BMS exchange (both directions) shifts out of UART4,
//   so a USB-UART adapter on the PCS-485 jack hears everything the box says to
//   the battery and everything the battery answers, without tapping the cable.
// DIP2 ON: PURE BRIDGE — the box sends NO commands of its own to the battery;
//   inverter requests arriving on UART4 are forwarded to the BMS line and the
//   answer is relayed straight back. The same traffic is pushed through
//   bms_parse_response() so CAN keeps broadcasting (listen/observe) and the
//   balance engine keeps its aggregates. Safeguards: only fixed 8-byte
//   FC03/FC04 read requests with a good CRC are forwarded (writes and noise
//   are dropped, the inverter just times out).
//   PRODUCTION KEEPS DIP2 OFF — 13.10 analysis: the inverter would read raw
//   packs over RS485, bypassing our CCL stop-tap aggregation; this mode is
//   research/diagnostics (DEYE_CAN_RS485_BASE.md §Q23, TODO §5.1).
#define DIP2_BRIDGE_MODE 0x02
#define PCS_GAP_MS 5           // ~3.5 idle chars at 9600 (3.65 ms): Modbus frame gap
#define PCS_RESP_FIRST_MS 300  // BMS must start answering within this
#define PCS_RESP_MAX_MS 1000   // whole bridged response hard cap
#define PCS_RESP_STALL_MS 200  // give up on a 1..3 byte fragment after this
#define PCS_NOISE_MIN_LEN 4    // shorter bursts (floating bus) are noise
#define BR_SWEEP_MAX_MS 15000UL // inverter loop ≈13 s for 16 addrs @ ~400 ms

static void watchdog_feed(void);
static bool bms_parse_response(uint8_t expected_addr);
static void bms_aggregate_data(void);
static void agg_reset(void);

static void pcs_uart_init(void) {
  rcu_periph_clock_enable(RCU_GPIOD);
  rcu_periph_clock_enable(PCS_TX_RCU);
  rcu_periph_clock_enable(PCS_UART_RCU);
  gpio_init(PCS_TX_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PCS_TX_PIN);
  gpio_init(PCS_RX_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, PCS_RX_PIN);
  usart_deinit(PCS_UART);
  usart_baudrate_set(PCS_UART, BMS_BAUDRATE);
  usart_parity_config(PCS_UART, USART_PM_NONE);
  usart_word_length_set(PCS_UART, USART_WL_8BIT);
  usart_stop_bit_set(PCS_UART, USART_STB_1BIT);
  usart_transmit_config(PCS_UART, USART_TRANSMIT_ENABLE);
  usart_receive_config(PCS_UART, USART_RECEIVE_ENABLE);
  usart_enable(PCS_UART);
}

// UART4 error flags, same treatment as rs485_clear_errors() gives USART0:
// without this an overrun taken while the main loop is blocked (CAN burst)
// could stick on the PCS port and wedge its receive path.
static void pcs_clear_errors(void) {
  usart_flag_clear(PCS_UART, USART_FLAG_ORERR);
  usart_flag_clear(PCS_UART, USART_FLAG_NERR);
  usart_flag_clear(PCS_UART, USART_FLAG_FERR);
  usart_flag_clear(PCS_UART, USART_FLAG_PERR);
}

static uint32_t pcs_led_off_ms = 0;
static void pcs_led_activity(void) {
  gpio_bit_set(LED_INV485_PORT, LED_INV485_PIN);
  pcs_led_off_ms = millis() + 100;
}

// --- mirror ring buffer (DIP2 OFF); single-threaded polling, no volatile ---
static uint8_t pcs_mirror_q[256];
static uint16_t pcs_mirror_head = 0;
static uint16_t pcs_mirror_tail = 0;
static bool pcs_bridge_mode = false;

static void pcs_mirror_push(uint8_t b) {
  if (pcs_bridge_mode)
    return; // PCS port belongs to the inverter in bridge mode
  uint16_t next = (uint16_t)((pcs_mirror_head + 1) & 0xFF);
  if (next == pcs_mirror_tail)
    return; // listener too slow — drop, never block the BMS path
  pcs_mirror_q[pcs_mirror_head] = b;
  pcs_mirror_head = next;
  pcs_led_activity();
}

static void pcs_mirror_pump(void) {
  if (pcs_mirror_head != pcs_mirror_tail &&
      usart_flag_get(PCS_UART, USART_FLAG_TBE) != RESET) {
    usart_data_transmit(PCS_UART, pcs_mirror_q[pcs_mirror_tail]);
    pcs_mirror_tail = (uint16_t)((pcs_mirror_tail + 1) & 0xFF);
    pcs_led_activity(); // LED3 follows the line until the queue drains
  }
}

// --- bridge state (DIP2 ON) ---
enum { BR_IDLE = 0, BR_SEND, BR_WAIT_RESP, BR_FWD_RESP };
static uint8_t br_state = BR_IDLE;
static uint8_t inv_frame[256]; // one complete inverter request
static uint16_t inv_frame_n = 0;
static uint8_t inv_rx[256]; // request bytes still collecting
static uint16_t inv_rx_n = 0;
static uint32_t inv_last_rx_ms = 0;
static uint32_t br_sent_ms = 0;
static uint32_t br_first_ms = 0;
static uint32_t br_last_ms = 0;
static bool br_got = false;

// Modbus RTU request sanity: fixed 8-byte read (FC03/FC04), good CRC.
static bool pcs_request_ok(const uint8_t *f) {
  if (f[1] != 0x03 && f[1] != 0x04)
    return false;
  uint16_t crc = modbus_crc16(f, 6);
  return f[6] == (uint8_t)(crc & 0xFF) && f[7] == (uint8_t)(crc >> 8);
}

static void pcs_uart_rx_service(uint32_t now) {
  if (usart_flag_get(PCS_UART, USART_FLAG_RBNE) == RESET)
    return;
  uint8_t b = (uint8_t)usart_data_receive(PCS_UART);
  pcs_led_activity();
  if (inv_rx_n < sizeof(inv_rx))
    inv_rx[inv_rx_n++] = b;
  inv_last_rx_ms = now;
}

// Called every main-loop pass and inside bms_receive_response(): mirror mode
// drains the ring; bridge mode reads UART4 and closes a request on silence.
static void pcs_poll_service(uint32_t now) {
  if (!pcs_bridge_mode) {
    pcs_mirror_pump();
    return;
  }
  pcs_uart_rx_service(now);
  if (inv_frame_n == 0 && inv_rx_n != 0 && (now - inv_last_rx_ms) >= PCS_GAP_MS) {
    if (inv_rx_n >= PCS_NOISE_MIN_LEN && pcs_request_ok(inv_rx)) {
      memcpy(inv_frame, inv_rx, inv_rx_n);
      inv_frame_n = inv_rx_n;
    }
    inv_rx_n = 0;
  }
}

// Bridge mode drives aggregation itself (the box sends no own polls): each
// parsed bridged response feeds the sums once per sweep (dedupe in the
// parser), and the sweep closes when every known pack has been seen or after
// BR_SWEEP_MAX_MS — the inverter's own loop takes ~13 s for 16 addresses.
// A timeout with agg_count == 0 still runs bms_aggregate_data(): it only
// does the presence bookkeeping and clears sweep_seen_mask (after a
// fail-closed agg_reset mid-sweep the mask could otherwise stick full and
// dedupe would starve every later response).
static uint32_t br_sweep_start_ms = 0;

static void bridge_sweep_tick(uint32_t now) {
  if (br_sweep_start_ms == 0) {
    br_sweep_start_ms = now;
    return;
  }
  bool complete = pack_ever_seen_mask != 0 && agg_count != 0 &&
                  ((sweep_seen_mask & pack_ever_seen_mask) == pack_ever_seen_mask);
  if (complete || (now - br_sweep_start_ms) >= BR_SWEEP_MAX_MS) {
    bms_aggregate_data();
    br_sweep_start_ms = 0;
  }
}

static bool pcs_bridge_finish(void) {
  inv_frame_n = 0;
  uart_rx_index = 0;
  gpio_bit_reset(LED_BMS485_PORT, LED_BMS485_PIN);
  br_state = BR_IDLE;
  return true; // informational only — own polling never resumes in bridge mode
}

// Bridge step: runs once per main-loop pass. Returns true when a bridged
// transaction just finished (ignored by the caller — while DIP2 is ON the
// box sends no polls of its own).
static bool pcs_bridge_step(uint32_t now) {
  switch (br_state) {
  case BR_IDLE: {
    if (inv_frame_n == 0)
      return false;
    rs485_flush_rx();
    pcs_clear_errors(); // drop UART4 overrun left by a blocked loop pass
    uart_rx_index = 0;
    gpio_bit_set(LED_BMS485_PORT, LED_BMS485_PIN);
    for (uint16_t i = 0; i < inv_frame_n; i++) {
      rs485_send_byte(inv_frame[i]);
      pcs_poll_service(millis()); // keep UART4 RX from overrunning
    }
    br_sent_ms = millis();
    br_got = false;
    br_state = BR_WAIT_RESP;
    return false;
  }

  case BR_WAIT_RESP: {
    pcs_poll_service(millis());
    if (usart_flag_get(USART0, USART_FLAG_RBNE) != RESET) {
      uint8_t b = (uint8_t)usart_data_receive(USART0);
      if (!br_got) {
        br_got = true;
        br_first_ms = now;
      }
      br_last_ms = now;
      if (uart_rx_index < sizeof(uart_rx_buffer))
        uart_rx_buffer[uart_rx_index++] = b;
    }
    if (br_got && uart_rx_index >= PCS_NOISE_MIN_LEN &&
        (now - br_last_ms) >= PCS_GAP_MS) {
      // Feed CAN/balance from the bridged exchange: in bridge mode the box
      // sends no own polls, so this is the ONLY data source. Keeps
      // last_bms_response_ms fresh -> fail-closed still trips when the
      // inverter stops polling.
      if (bms_parse_response(uart_rx_buffer[0]))
        last_bms_response_ms = millis();
      br_state = BR_FWD_RESP; // complete answer — relay it as-is
      return false;
    }
    if (br_got && (now - br_first_ms) >= PCS_RESP_MAX_MS)
      return pcs_bridge_finish(); // too slow — let the inverter time out
    if (br_got && (now - br_last_ms) >= PCS_RESP_STALL_MS)
      return pcs_bridge_finish(); // truncated fragment (1..3 bytes)
    if (!br_got && (now - br_sent_ms) >= PCS_RESP_FIRST_MS)
      return pcs_bridge_finish(); // BMS stayed silent
    return false;
  }

  case BR_FWD_RESP: {
    for (uint16_t i = 0; i < uart_rx_index; i++) {
      usart_data_transmit(PCS_UART, uart_rx_buffer[i]);
      while (usart_flag_get(PCS_UART, USART_FLAG_TBE) == RESET)
        ;
      pcs_led_activity();
      pcs_poll_service(millis());
    }
    return pcs_bridge_finish();
  }

  default:
    return pcs_bridge_finish();
  }
}

// DIP2 mode switch; deferred while a bridge transaction is in flight
// (retried from the 250 ms DIP poll until the bridge goes idle).
static void pcs_mode_apply(uint8_t code) {
  bool bridge = (code & DIP2_BRIDGE_MODE) != 0;
  if (bridge == pcs_bridge_mode)
    return;
  if (br_state != BR_IDLE || inv_frame_n != 0)
    return; // busy — keep the old mode for now
  pcs_bridge_mode = bridge;
  pcs_mirror_head = pcs_mirror_tail = 0;
  inv_rx_n = 0;
  // Drop a half-built sweep from the other mode so dedupe/sums start clean.
  br_sweep_start_ms = 0;
  sweep_seen_mask = 0;
  if (agg_count != 0)
    agg_reset();
}

static void bms_send_query(uint8_t addr_index) {
  rs485_flush_rx();
  uart_rx_index = 0;
  gpio_bit_set(LED_BMS485_PORT, LED_BMS485_PIN); // poll in flight
  // No DE pin on the transceiver (auto-direction, ORIGINAL_FIRMWARE.md §14.3):
  // just shift the frame out; direction switches in hardware.
  for (uint8_t i = 0; i < 8; i++) {
    rs485_send_byte(BMS_QUERY_TABLE[addr_index][i]);
    pcs_mirror_push(BMS_QUERY_TABLE[addr_index][i]); // no-op in bridge mode
  }
}

static uint16_t bms_receive_response(void) {
  uart_rx_index = 0;
  uint32_t start_ms = millis();
  uint32_t wait_start = start_ms;
  while (millis() - wait_start < BMS_TIMEOUT_MS) {
    pcs_poll_service(millis()); // mirror drain / inverter RX while we wait
    if (millis() - start_ms >= 700)
      break; // жорсткий дедлайн від першого байта (запас під повільні BMS)
    if (usart_flag_get(USART0, USART_FLAG_RBNE) != RESET) {
      uint8_t byte = usart_data_receive(USART0);
      pcs_mirror_push(byte); // no-op in bridge mode
      if (uart_rx_index < sizeof(uart_rx_buffer)) {
        uart_rx_buffer[uart_rx_index++] = byte;
      } else {
        uart_rx_index = 0; // overflow
      }
      if (uart_rx_index >= BMS_RESPONSE_LEN)
        break; // кадр повний
      wait_start = millis();
    }
  }
  rs485_clear_errors();
  gpio_bit_reset(LED_BMS485_PORT, LED_BMS485_PIN); // poll finished
  return uart_rx_index;
}

static bool bms_parse_response(uint8_t expected_addr) {
  if (expected_addr < 1 || expected_addr > 16)
    return false; // not one of our pack addresses (bridge forwards anything)
  if (uart_rx_index < 5)
    return false;
  if (uart_rx_buffer[0] != expected_addr || uart_rx_buffer[1] != 0x03)
    return false;
  // Byte count: our own window is 0x4E (39 regs, 83 B); a bridged inverter
  // frame (Deye) asks 0x38 (28 regs, 61 B). Other windows (e.g. reg 0x50)
  // are not ours — they stay transparent and are never aggregated.
  uint16_t data_bytes = uart_rx_buffer[2];
  if (data_bytes != 0x4E && data_bytes != 0x38)
    return false;
  uint16_t frame_len = (uint16_t)(5 + data_bytes);
  if (uart_rx_index < frame_len)
    return false;

  uint16_t received_crc = (uart_rx_buffer[frame_len - 1] << 8) |
                          uart_rx_buffer[frame_len - 2];
  uint16_t calculated_crc =
      modbus_crc16((uint8_t *)uart_rx_buffer, frame_len - 2);
  if (received_crc != calculated_crc)
    return false;

  uint16_t raw_voltage = (uart_rx_buffer[3] << 8) | uart_rx_buffer[4];
  int16_t raw_current = (int16_t)((uart_rx_buffer[5] << 8) | uart_rx_buffer[6]);
  // P1.1 (22.10.2026): офсети за PDF V01.01 — SOC = рег 0024, SOH = рег 0023.
  // Кулонометр Vision (рег 24) ненадійний/зависає; CAN-SOC (0x355) та UV-захист (0x359)
  // базуються виключно на фізичних вимірах напруги комірок.
  uint16_t raw_soh = (uart_rx_buffer[49] << 8) | uart_rx_buffer[50];

  for (uint8_t cell = 0; cell < 15; cell++) {
    uint16_t cell_mv = ((uint16_t)uart_rx_buffer[7 + cell * 2] << 8) |
                       uart_rx_buffer[8 + cell * 2];
    if (cell_mv > agg_max_cell_mv)
      agg_max_cell_mv = cell_mv;
    if (cell_mv > 0 && cell_mv < agg_min_cell_mv)
      agg_min_cell_mv = cell_mv;
  }

  // Multi-sensor temperature tracking: regs 18, 19, 20 (bytes 39..44)
  // Sanity check all sensor channels (-40..120 °C) to reject disconnected (0x7FFF) or corrupted probes:
  int16_t t_a = (int16_t)((uart_rx_buffer[39] << 8) | uart_rx_buffer[40]);
  int16_t t_main = (int16_t)((uart_rx_buffer[41] << 8) | uart_rx_buffer[42]);
  int16_t t_b = (int16_t)((uart_rx_buffer[43] << 8) | uart_rx_buffer[44]);

  float pack_t_max = -1000.0f;
  float pack_t_min = 1000.0f;
  if (t_main >= -40 && t_main <= 120) {
    pack_t_max = (float)t_main;
    pack_t_min = (float)t_main;
  }
  if (t_a >= -40 && t_a <= 120) {
    if (pack_t_max < -500.0f || (float)t_a > pack_t_max) pack_t_max = (float)t_a;
    if (pack_t_min > 500.0f || (float)t_a < pack_t_min) pack_t_min = (float)t_a;
  }
  if (t_b >= -40 && t_b <= 120) {
    if (pack_t_max < -500.0f || (float)t_b > pack_t_max) pack_t_max = (float)t_b;
    if (pack_t_min > 500.0f || (float)t_b < pack_t_min) pack_t_min = (float)t_b;
  }
  if (pack_t_max < -500.0f) {
    pack_t_max = BATTERY_TEMP_DEFAULT;
    pack_t_min = BATTERY_TEMP_DEFAULT;
  }
  if (pack_t_max > agg_temp_max)
    agg_temp_max = pack_t_max;
  if (pack_t_min < agg_temp_min)
    agg_temp_min = pack_t_min;

  // Status, Warning, Protection (regs 25..27, bytes 53..58): present in both
  // accepted windows (data_bytes >= 56). Error (reg 28, bytes 59..60) exists
  // only in the full 39-reg window — the 28-reg inverter window ends at reg 27.
  uint16_t reg_status  = ((uint16_t)uart_rx_buffer[53] << 8) | uart_rx_buffer[54];
  uint16_t reg_warning = ((uint16_t)uart_rx_buffer[55] << 8) | uart_rx_buffer[56];
  uint16_t reg_protect = ((uint16_t)uart_rx_buffer[57] << 8) | uart_rx_buffer[58];
  uint16_t reg_error   = 0;
  if (data_bytes >= 58)
    reg_error = ((uint16_t)uart_rx_buffer[59] << 8) | uart_rx_buffer[60];
  if (data_bytes >= 78) {
    uint16_t reg_balance = ((uint16_t)uart_rx_buffer[79] << 8) | uart_rx_buffer[80];
    sweep_balance_mask |= reg_balance;
  }

  // Cross-validate with physical measurements to reject transient Boot Latch flags (Q17):
  // Confirmation uses physical threshold CELL_PHYSICAL_OV_CONFIRM_MV (3600 mV), NOT calc_charge_stop_mv,
  // so that activating the 90% software taper profile never generates false emergency cell OV alarms:
  if ((reg_protect & 0x0002) && agg_max_cell_mv >= CELL_PHYSICAL_OV_CONFIRM_MV)
    sweep_protect_b0 |= (CAN_PROT_CELL_OV | CAN_PROT_PACK_OV);
  // Cell UV Protect (0x359 b0): triggers proactively at <= 2750 mV (ahead of BMS 2500 mV)
  // or if confirmed by BMS hardware alarm:
  if (agg_min_cell_mv <= CELL_DISCHARGE_STOP_MV ||
      ((reg_protect & 0x0008) && agg_min_cell_mv <= CELL_UV_WARN_MV))
    sweep_protect_b0 |= CAN_PROT_CELL_UV;
  if ((reg_protect & 0x0001) && raw_voltage >= 5450)
    sweep_protect_b0 |= CAN_PROT_PACK_OV;
  if ((reg_protect & 0x0004) && raw_voltage <= 4320)
    sweep_protect_b0 |= CAN_PROT_CELL_UV;
  if ((reg_protect & 0x0300) || pack_t_max >= TEMP_DISCHARGE_MAX_C)
    sweep_protect_b0 |= CAN_PROT_OT;
  // System-level UT protection stops both charge and discharge in Deye; trigger only if battery
  // is under the physical discharge limit (-20°C) or BMS asserts protection (charge limit handled via CCL=0):
  if ((reg_protect & 0x0C00) || pack_t_min <= TEMP_DISCHARGE_MIN_C)
    sweep_protect_b0 |= CAN_PROT_UT;
  if (reg_protect & 0x0020)
    sweep_protect_b0 |= CAN_PROT_DISCHARGE_OC;

  if (reg_protect & 0x0010)
    sweep_protect_b1 |= CAN_PROT_CHARGE_OC;
  if ((reg_protect & 0x2000) || (reg_status & 0x0004) || reg_error != 0)
    sweep_protect_b1 |= CAN_PROT_SYSTEM_ERROR;

  // Mirror confirmed warnings into CAN alarm bytes 2 and 3:
  if ((reg_warning & 0x0002) && agg_max_cell_mv >= CELL_PHYSICAL_OV_WARN_CONFIRM_MV)
    sweep_alarm_b2 |= (CAN_PROT_CELL_OV | CAN_PROT_PACK_OV);
  // Cell UV Warning (0x359 b2): proactive warning <= 2900 mV (ahead of BMS 2800 mV)
  // or if confirmed by BMS warning:
  if (agg_min_cell_mv <= CELL_UV_WARN_MV ||
      ((reg_warning & 0x0008) && agg_min_cell_mv <= CELL_UV_WARN_MV))
    sweep_alarm_b2 |= CAN_PROT_CELL_UV;
  if ((reg_warning & 0x0001) && raw_voltage >= 5400)
    sweep_alarm_b2 |= CAN_PROT_PACK_OV;
  if ((reg_warning & 0x0004) && raw_voltage <= 4400)
    sweep_alarm_b2 |= CAN_PROT_CELL_UV;
  if ((reg_warning & 0x0300) || pack_t_max >= 50.0f)
    sweep_alarm_b2 |= CAN_PROT_OT;
  if ((reg_warning & 0x0C00) || pack_t_min <= (TEMP_DISCHARGE_MIN_C + 5.0f))
    sweep_alarm_b2 |= CAN_PROT_UT;
  if (reg_warning & 0x0020)
    sweep_alarm_b2 |= CAN_PROT_DISCHARGE_OC;
  if (reg_warning & 0x0010)
    sweep_alarm_b3 |= CAN_PROT_CHARGE_OC;

  bool dup = (sweep_seen_mask & (1 << (expected_addr - 1))) != 0;
  sweep_seen_mask |= (1 << (expected_addr - 1));

  // Feed the sums once per address per sweep: in bridge mode the inverter may
  // repeat an address and would bias the averages toward it. Cells/temps and
  // the |= latches above stay idempotent, so only the sums are gated.
  if (!dup) {
    agg_voltage_sum += raw_voltage / 100.0f;
    agg_current_sum += raw_current / 100.0f;
    agg_soh_sum += (uint8_t)(raw_soh & 0xFF);
    agg_count++;
  }

  return true;
}

static uint16_t calculate_charge_limit(uint8_t pack_count) {
  uint32_t per_pack_x10 = profile_charge_current_per_pack_x10;
  // Balance/manual top-off phase: cap at 2.0 A per pack (or 1.0 A during float hold)
  // so passive balancing gets hours of gentle CV time instead of a rushed stop at the threshold.
  if (bal_session_active) {
    if (bal_float_hold && per_pack_x10 > BAL_HOLD_A_X10)
      per_pack_x10 = BAL_HOLD_A_X10;
    else if (bal_trickle && per_pack_x10 > BAL_TRICKLE_A_X10)
      per_pack_x10 = BAL_TRICKLE_A_X10;
  } else if (bal_trickle && per_pack_x10 > BAL_TRICKLE_A_X10) {
    per_pack_x10 = BAL_TRICKLE_A_X10;
  }
  uint32_t max_current_x10 = per_pack_x10 * pack_count;

  // Temperature cut-offs: LiFePO4 plating occurs below +2°C; over-temp at 55°C.
  // Hysteresis release: re-enable only when temp is within [+3°C .. 50°C] to prevent chattering.
  if (agg_temp_min < TEMP_CHARGE_MIN_C || agg_temp_max > TEMP_CHARGE_MAX_C) {
    charge_temp_inhibit = true;
  } else if (charge_temp_inhibit) {
    if (agg_temp_min >= TEMP_CHARGE_MIN_REENABLE_C && agg_temp_max <= TEMP_CHARGE_MAX_REENABLE_C) {
      charge_temp_inhibit = false;
    }
  }

  // System error or over-current protection from 0x359
  if (can_protect_b1 & (CAN_PROT_CHARGE_OC | CAN_PROT_SYSTEM_ERROR)) {
    return 0;
  }

  // Cell voltage cut-off:
  // In a balance session, hard cutoff is pushed to CELL_EMERGENCY_OV_MV (3620 mV)
  // because float-hold voltage regulation (51.6 V) prevents cells from staying too high,
  // avoiding premature 0 A cutoffs that force Deye into Standby.
  uint16_t stop_mv = bal_session_active ? CELL_EMERGENCY_OV_MV : calc_charge_stop_mv;
  uint16_t reenable_mv = bal_session_active ? (CELL_EMERGENCY_OV_MV - 50) : calc_charge_reenable_mv;
  if (agg_max_cell_mv >= stop_mv) {
    charge_cell_ov_inhibit = true;
  } else if (charge_cell_ov_inhibit) {
    if (agg_max_cell_mv < reenable_mv) {
      charge_cell_ov_inhibit = false;
    }
  }

  if (charge_temp_inhibit || charge_cell_ov_inhibit) {
    return 0;
  }

  if (bal_session_active && bal_float_hold) {
    return (uint16_t)max_current_x10;
  }

  if (agg_max_cell_mv <= calc_taper_start_mv)
    return (uint16_t)max_current_x10;

  return (uint16_t)(max_current_x10 *
                    (calc_charge_stop_mv - agg_max_cell_mv) /
                    (calc_charge_stop_mv - calc_taper_start_mv));
}

static uint16_t calculate_discharge_limit(uint8_t pack_count) {
  uint32_t max_current_x10 =
      (uint32_t)DISCHARGE_CURRENT_LIMIT_PER_PACK_X10 * pack_count;

  if (agg_min_cell_mv == 0xFFFF)
    return 0;

  // Temperature cut-offs for discharge (-20°C .. +60°C) with hysteresis
  if (agg_temp_min < TEMP_DISCHARGE_MIN_C || agg_temp_max > TEMP_DISCHARGE_MAX_C) {
    discharge_temp_inhibit = true;
  } else if (discharge_temp_inhibit) {
    if (agg_temp_min >= TEMP_DISCHARGE_MIN_REENABLE_C && agg_temp_max <= TEMP_DISCHARGE_MAX_REENABLE_C) {
      discharge_temp_inhibit = false;
    }
  }

  // System error protection
  if (can_protect_b1 & CAN_PROT_SYSTEM_ERROR) {
    return 0;
  }

  // Cell voltage cut-off: soft cut <= 2750 mV (ahead of BMS 2500 mV), re-enable >= 3000 mV (Vision HMI Release)
  if (agg_min_cell_mv <= CELL_DISCHARGE_STOP_MV) {
    discharge_cell_uv_inhibit = true;
  } else if (discharge_cell_uv_inhibit) {
    if (agg_min_cell_mv >= CELL_DISCHARGE_REENABLE_MV) {
      discharge_cell_uv_inhibit = false;
    }
  }

  if (discharge_temp_inhibit || discharge_cell_uv_inhibit) {
    return 0;
  }

  // Soft cut / Cell UV warning (<= 2900 mV): derate discharge current to 50%
  if (agg_min_cell_mv <= CELL_UV_WARN_MV) {
    max_current_x10 /= 2;
  }

  // Severe cell imbalance check (> 400 mV delta): derate to 50%
  if (agg_max_cell_mv > agg_min_cell_mv && (agg_max_cell_mv - agg_min_cell_mv) > CELL_MAX_DELTA_MV) {
    max_current_x10 /= 2;
  }

  return (uint16_t)max_current_x10;
}

// ===== Balance session engine (17.10.2026) =====
// DIP1 OFF (standard 54.0 V): auto sessions (BAL_AUTO_ENABLED) driven by
// cell-spread analysis open balance windows (CVL stays 54 V; thresholds stay
// full; current capped at 2 A per pack once min cell >= 3400 mV).
// DIP1 ON (51.8 V option): auto logic disabled; the same 2 A trickle from
// 3400 mV applies. Sessions exit on spread convergence (<= 25 mV), BMS loss,
// sustained discharge (grid outage), no progress or an 8 h timeout.

static void balance_session_end(bool success, uint32_t cooldown_until) {
  bal_session_active = false;
  bal_trickle = false;
  bal_float_hold = false;
  bal_exit_streak = 0;
  bal_outage_agg = 0;
  if (success)
    bal_last_success_ms = millis();
  bal_cooldown_until_ms = cooldown_until;
}

static void balance_on_bms_lost(void) {
  if (bal_session_active)
    balance_session_end(false, millis() + BAL_COOLDOWN_OUTAGE_MS);
}

static void balance_hist_push(uint16_t spread, uint32_t now) {
  if (bal_last_hist_ms != 0 && (now - bal_last_hist_ms) < 3600000UL)
    return;
  bal_last_hist_ms = now;
  if (bal_hist_n < BAL_HIST_HOURS) {
    bal_hist_mv[bal_hist_n++] = spread;
  } else {
    memmove(&bal_hist_mv[0], &bal_hist_mv[1],
            (BAL_HIST_HOURS - 1) * sizeof(uint16_t));
    bal_hist_mv[BAL_HIST_HOURS - 1] = spread;
  }
}

// >=5 mV growth over 12 h == 10 mV/day -> anomaly regardless of the bucket.
static bool balance_trend_high(void) {
  if (bal_hist_n < 13)
    return false;
  return ((int32_t)bal_hist_mv[bal_hist_n - 1] -
          (int32_t)bal_hist_mv[bal_hist_n - 12]) > BAL_TREND_DELTA_MV_12H;
}

static void balance_update(uint8_t packs) {
  bool manual = (dip_applied_code & DIP_PROFILE_CODE_90) != 0;
  uint32_t now = millis();
  bool have_cells = (agg_max_cell_mv != 0) && (agg_min_cell_mv != 0xFFFF) &&
                    (agg_max_cell_mv >= agg_min_cell_mv);
  uint16_t spread =
      have_cells ? (uint16_t)(agg_max_cell_mv - agg_min_cell_mv) : 0;

  if (have_cells)
    balance_hist_push(spread, now);

  // DIP1 transitions: switching the 51.8 V option ON aborts an active
  // session; leaving the option returns to standard 54 V behind a cooldown
  // so an auto session cannot re-trigger instantly.
  if (manual && !bal_manual_prev) {
    bal_session_active = false;
    bal_manual_prev = true;
  } else if (!manual && bal_manual_prev) {
    bal_session_active = false;
    bal_manual_prev = false;
    bal_cooldown_until_ms = now + BAL_COOLDOWN_TRY_MS;
  }

  if (bal_session_active) {
    // Sustained discharge (~60 s) = grid outage or heavy load -> abort.
    if (bms_current > BAL_OUTAGE_I_MIN_A) {
      if (bal_outage_agg < 255)
        bal_outage_agg++;
      if (bal_outage_agg >= BAL_OUTAGE_AGG_N) {
        balance_session_end(false, now + BAL_COOLDOWN_OUTAGE_MS);
        return;
      }
    } else {
      bal_outage_agg = 0;
    }

    // Top-off phase: min cell reached ~90% -> 2 A per pack cap
    // (applied by calculate_charge_limit() through bal_trickle).
    bal_trickle = (agg_min_cell_mv != 0xFFFF) &&
                  (agg_min_cell_mv >= BAL_TRICKLE_CELL_MV);

    // Dynamic Float Hold: when max cell reaches 3.56 V, clamp requested voltage
    // to 51.6 V to prevent emergency cell OV while maintaining gentle balance current.
    if (agg_max_cell_mv != 0 && agg_max_cell_mv >= BAL_HOLD_CELL_MV) {
      bal_float_hold = true;
    } else if (bal_float_hold && agg_max_cell_mv < BAL_HOLD_RELEASE_MV) {
      bal_float_hold = false;
    }

    if (bal_trickle) {
      if (bal_best_spread_mv == 0xFFFF) {
        bal_best_spread_mv = spread;
        bal_last_improve_ms = now;
      } else if (spread < bal_best_spread_mv) {
        bal_best_spread_mv = spread;
        bal_last_improve_ms = now;
      } else if ((now - bal_last_improve_ms) >= BAL_NO_PROGRESS_MS) {
        // Balancing not helping (possible weak cell) -> back off.
        balance_session_end(false, now + BAL_COOLDOWN_TRY_MS);
        return;
      }
    }

    if (spread <= BAL_SPREAD_EXIT_MV) {
      if (++bal_exit_streak >= BAL_EXIT_STREAK) {
        balance_session_end(true, now);
        return;
      }
    } else {
      bal_exit_streak = 0;
    }

    if ((now - bal_session_start_ms) >= BAL_SESSION_MAX_MS) {
      balance_session_end(false, now + BAL_COOLDOWN_TRY_MS);
      return;
    }

    // In-session thresholds: full-charge triple so the 54 V session is not
    // cut by the 90% profile stop; the existing 3600 mV OV inhibit stays
    // as the hard guard.
    calc_taper_start_mv = CELL_TAPER_START_MV;
    calc_charge_stop_mv = CELL_CHARGE_STOP_MV;
    calc_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
    return;
  }

  // Not in a session: active thresholds mirror the DIP profile.
  calc_taper_start_mv = profile_taper_start_mv;
  calc_charge_stop_mv = profile_charge_stop_mv;
  calc_charge_reenable_mv = profile_charge_reenable_mv;
  bal_trickle = manual && have_cells &&
                (agg_min_cell_mv >= BAL_TRICKLE_CELL_MV);

  if (manual || !BAL_AUTO_ENABLED)
    return;
  if (!bms_data_valid)
    return;

  // Pack composition change (a pack was switched on/off) -> HIGH session due.
  if (bal_prev_packs != 0 && packs != bal_prev_packs)
    bal_last_success_ms = now - BAL_INTERVAL_HIGH_MS;
  bal_prev_packs = packs;

  if (bms_soc < BAL_MIN_SOC_START)
    return;
  if (bms_current >= BAL_START_I_MAX_A)
    return;
  if (charge_cell_ov_inhibit || charge_temp_inhibit)
    return;
  if (agg_temp_min < BAL_TEMP_MIN_C || agg_temp_max > BAL_TEMP_MAX_C)
    return;
  if ((int32_t)(now - bal_cooldown_until_ms) < 0)
    return;

  // Severity: spread bucket + 12 h trend. Outliers >= 100 mV are covered by
  // the HIGH bucket (interval: 24 h).
  uint32_t interval;
  if ((spread >= BAL_SPREAD_HIGH_MV) || balance_trend_high())
    interval = BAL_INTERVAL_HIGH_MS;
  else if (spread >= BAL_SPREAD_MED_MV)
    interval = BAL_INTERVAL_MED_MS;
  else
    interval = BAL_INTERVAL_OK_MS;

  if ((now - bal_last_success_ms) < interval)
    return;

  // Already well balanced: renew the interval instead of burning a session.
  if (spread <= BAL_SPREAD_EXIT_MV) {
    bal_last_success_ms = now;
    return;
  }

  // Start a session.
  bal_session_active = true;
  bal_session_start_ms = now;
  bal_exit_streak = 0;
  bal_outage_agg = 0;
  bal_best_spread_mv = 0xFFFF;
  bal_last_improve_ms = now;
  bal_trickle = false;
  bal_float_hold = false;
  calc_taper_start_mv = CELL_TAPER_START_MV;
  calc_charge_stop_mv = CELL_CHARGE_STOP_MV;
  calc_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
}

// LED1 (PC13): solid = 51.8 V option (DIP1 ON), slow blink = auto session,
// off = standard 54 V without session. Called from the main loop.
static void balance_led_tick(uint32_t now) {
  static bool on = false;
  static uint32_t last_toggle = 0;
  if (dip_applied_code & DIP_PROFILE_CODE_90) {
    gpio_bit_set(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
    on = true;
    return;
  }
  if (bal_session_active) {
    if ((now - last_toggle) >= 500) {
      last_toggle = now;
      on = !on;
      if (on)
        gpio_bit_set(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
      else
        gpio_bit_reset(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
    }
  } else if (on) {
    on = false;
    gpio_bit_reset(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
  }
}

static void agg_reset(void) {
  agg_voltage_sum = 0.0f;
  agg_current_sum = 0.0f;
  agg_soh_sum = 0;
  agg_temp_max = -1000.0f;
  agg_temp_min = 1000.0f;
  agg_count = 0;
  agg_max_cell_mv = 0;
  agg_min_cell_mv = 0xFFFF;
}

static void bms_aggregate_data(void) {
  // Update pack presence with 3-miss debounce (Q21)
  uint8_t active_pack_count = 0;
  uint16_t expected_mask = (fast_poll_enabled && fast_poll_mask != 0) ? fast_poll_mask : pack_ever_seen_mask;
  for (uint8_t i = 0; i < 16; i++) {
    if (sweep_seen_mask & (1 << i)) {
      pack_ever_seen_mask |= (1 << i);
      pack_miss_count[i] = 0;
      active_pack_count++;
    } else if (expected_mask & (1 << i)) {
      if (pack_miss_count[i] < 255)
        pack_miss_count[i]++;
      if (pack_miss_count[i] <= 2) {
        active_pack_count++;
      }
    }
  }
  sweep_seen_mask = 0;

  if (agg_count == 0)
    return;

  bms_voltage = agg_voltage_sum / (float)agg_count;
  // Deye PCS CAN V3.3: 0x356 current is discharge positive, charge negative.
  bms_current = BMS_CURRENT_CHARGE_IS_POSITIVE ? -agg_current_sum
                                               : agg_current_sum;
  // P1.1: SOC з мінімальної комірки (мертвий рег 24 BMS не використовується);
  // перший sweep — snap без фільтра, далі EMA α=1/8.
  if (agg_min_cell_mv != 0xFFFF) {
    uint8_t soc_now = soc_from_cell_mv(agg_min_cell_mv);
    if (!soc_ema_valid) {
      soc_ema_x8 = (int16_t)soc_now * 8;
      soc_ema_valid = true;
    } else {
      soc_ema_x8 += ((int16_t)soc_now * 8 - soc_ema_x8) / 8;
    }
    bms_soc = (uint8_t)((soc_ema_x8 + 4) / 8);
  }
  bms_soh = (uint8_t)(agg_soh_sum / agg_count);
  bms_temp = agg_temp_max / BMS_TEMP_RAW_DIV;

  // Latch sweep alarms and protection into active CAN frame registers
  can_protect_b0 = sweep_protect_b0;
  can_protect_b1 = sweep_protect_b1;
  can_alarm_b2 = sweep_alarm_b2;
  can_alarm_b3 = sweep_alarm_b3;
  sweep_protect_b0 = 0;
  sweep_protect_b1 = 0;
  sweep_alarm_b2 = 0;
  sweep_alarm_b3 = 0;
  bms_balance_mask = sweep_balance_mask;
  sweep_balance_mask = 0;

  uint8_t effective_packs = (active_pack_count > 0) ? active_pack_count : agg_count;
  bms_pack_count = effective_packs;

  balance_update(effective_packs);
  if (bal_session_active) {
    charge_voltage_limit = bal_float_hold ? CHARGE_VOLTAGE_LIMIT_FLOAT_HOLD
                                          : CHARGE_VOLTAGE_LIMIT_STATIC;
  } else {
    charge_voltage_limit = profile_charge_voltage_limit;
  }
  charge_current_limit = calculate_charge_limit(effective_packs);
  discharge_current_limit = calculate_discharge_limit(effective_packs);

  // Set data valid now that fresh aggregated values are computed
  bms_data_valid = true;

  agg_reset();
}

static void can_init_500k(void) {
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(DEYE_CAN_GPIO_RCU);
  rcu_periph_clock_enable(DEYE_CAN_RCU);
  gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);
  gpio_init(DEYE_CAN_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ,
            DEYE_CAN_RX_PIN);
  gpio_init(DEYE_CAN_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
            DEYE_CAN_TX_PIN);

  can_deinit(DEYE_CAN);
  can_parameter_struct params;
  can_struct_para_init(CAN_INIT_STRUCT, &params);
  params.auto_bus_off_recovery = ENABLE;
  params.auto_retrans = ENABLE;
  params.working_mode = CAN_NORMAL_MODE;
  params.resync_jump_width = CAN_BT_SJW_1TQ;
  params.time_segment_1 = CAN_BT_BS1_11TQ;
  params.time_segment_2 = CAN_BT_BS2_4TQ;
  params.prescaler = 1;
  can_init(DEYE_CAN, &params);

  can_filter_parameter_struct filter;
  can_struct_para_init(CAN_FILTER_STRUCT, &filter);
  filter.filter_number = 0;
  filter.filter_mode = CAN_FILTERMODE_MASK;
  filter.filter_bits = CAN_FILTERBITS_32BIT;
  filter.filter_list_high = 0x0000;
  filter.filter_list_low = 0x0000;
  filter.filter_mask_high = 0x0000;
  filter.filter_mask_low = 0x0000;
  filter.filter_fifo_number = CAN_FIFO0;
  filter.filter_enable = ENABLE;
  can_filter_init(&filter);
}

static bool can_mailbox_free(void) {
  return (can_flag_get(DEYE_CAN, CAN_FLAG_TME0) == SET) ||
         (can_flag_get(DEYE_CAN, CAN_FLAG_TME1) == SET) ||
         (can_flag_get(DEYE_CAN, CAN_FLAG_TME2) == SET);
}

static void send_can_std(uint32_t id, const uint8_t *data, uint8_t len) {
  uint32_t wait_start = millis();
  while (!can_mailbox_free()) {
    if (millis() - wait_start >= 5)
      return; // скриньки зайняті (немає ACK) — кидати кадр
  }

  can_transmit_message_struct tx_msg;
  can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx_msg);
  tx_msg.tx_sfid = id;
  tx_msg.tx_ff = CAN_FF_STANDARD;
  tx_msg.tx_ft = CAN_FT_DATA;
  tx_msg.tx_dlen = len;
  for (uint8_t i = 0; i < len; i++)
    tx_msg.tx_data[i] = data[i];

  uint8_t mbox = can_message_transmit(DEYE_CAN, &tx_msg);
  if (mbox == CAN_NOMAILBOX)
    return;

  uint32_t tx_start = millis();
  while (can_transmit_states(DEYE_CAN, mbox) == CAN_TRANSMIT_PENDING) {
    if (millis() - tx_start >= 3)
      return;
  }
}

static int16_t round_to_i16(float v) {
  return (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static void put_u16le(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)(value & 0xFF);
  data[1] = (uint8_t)((value >> 8) & 0xFF);
}

static void send_all_frames(void) {
  gpio_bit_set(LED_PCSCAN_PORT, LED_PCSCAN_PIN); // LED4: CAN activity
  uint8_t data[8] = {0};
  put_u16le(&data[0], charge_voltage_limit);
  put_u16le(&data[2], charge_current_limit);
  put_u16le(&data[4], discharge_current_limit);
  put_u16le(&data[6], DISCHARGE_VOLTAGE_LIMIT_X10);
  send_can_std(CAN_ID_LIMITS, data, 8);

  memset(data, 0, 8);
  // Deye off-grid workaround (Battery-Emulator BYD-CAN.cpp): оф-грід Deye
  // ігнорує CCL/DCL і розряджає до SOC=0. Дзеркалити ліміти в SOC:
  // DCL=0 (розряд заборонено / порожній) → SOC=0; CCL=0 (заряд заборонено /
  // повний) → SOC=100. Обидва 0 (нема даних) → SOC=0 (чесний «не запрошуй»).
  uint8_t can_soc = bms_soc;
  if (discharge_current_limit == 0) {
    can_soc = 0;
  } else if (bal_session_active) {
    // During active balance session: protect inverter from jumping to Standby on SOC=100.
    // Retain cell-based OCV calculation, capped at 98% until session completes successfully.
    if (can_soc > 98)
      can_soc = 98;
  } else if (charge_current_limit == 0) {
    can_soc = 100;
  }
  put_u16le(&data[0], can_soc);
  put_u16le(&data[2], bms_soh);
  send_can_std(CAN_ID_SOC, data, 4);

  memset(data, 0, 8);
  put_u16le(&data[0], (uint16_t)round_to_i16(bms_voltage * 100.0f));
  put_u16le(&data[2], (uint16_t)round_to_i16(bms_current * 10.0f));
  put_u16le(&data[4], (uint16_t)round_to_i16(bms_temp * 10.0f));
  send_can_std(CAN_ID_STATUS, data, 6);

  // 0x359: Bytes 0..1 Protection, Bytes 2..3 Alarm, Byte 4 Module count, Bytes 5..6 'P' 'N', Byte 7 0
  memset(data, 0, 8);
  data[0] = can_protect_b0;
  data[1] = can_protect_b1;
  data[2] = can_alarm_b2;
  data[3] = can_alarm_b3;
  data[4] = bms_pack_count;
  data[5] = 'P';
  data[6] = 'N';
  data[7] = 0;
  send_can_std(CAN_ID_PROTECT, data, 8);

  // 0x35C: Request / Enable flags
  // Bit 7 (0x80) = Charge enable: active only if data valid, not inhibited, CCL > 0
  // Bit 6 (0x40) = Discharge enable: active only if data valid, not inhibited, DCL > 0
  // Bit 3 (0x08) = Request Full Charge (Pylontech v1.2+ for balancing/equalization)
  memset(data, 0, 8);
  uint8_t req_flags = 0x00;
  if (bms_data_valid) {
    if (bal_session_active) {
      if (!charge_temp_inhibit && charge_current_limit > 0)
        req_flags |= (0x80 | 0x08);
    } else {
      if (!charge_cell_ov_inhibit && !charge_temp_inhibit && charge_current_limit > 0)
        req_flags |= 0x80;
    }
    if (!discharge_cell_uv_inhibit && !discharge_temp_inhibit && discharge_current_limit > 0)
      req_flags |= 0x40;
  }
  data[0] = req_flags;
  send_can_std(CAN_ID_FLAGS, data, 2);

  static const uint8_t brand[8] = {'P', 'Y', 'L', 'O', 'N', 0, 0, 0};
  send_can_std(CAN_ID_BRAND, brand, 8);

  can_led_off_ms = millis() + 100; // keep the LED on briefly after TX
}

/* FWDGT — незалежний watchdog (LSI). SPL-джерела gd32f30x_fwdgt.c у проєкті нема,
 * тому регістри пишуться напряму. FWDGT_PSC_DIV256 + RLD 400 →
 * ≈3.13 с при LSI 32.768 кГц або ≈2.56 с при 40 кГц — запас над найгіршим
 * проходом циклу (прийом ≤500 мс + CAN ≤40 мс). Після старту не вимикається. */
static void watchdog_init(void) {
  uint32_t spin;
  FWDGT_CTL = FWDGT_WRITEACCESS_ENABLE;
  FWDGT_PSC = FWDGT_PSC_DIV256;
  for (spin = FWDGT_PSC_TIMEOUT; spin && (FWDGT_STAT & FWDGT_STAT_PUD); spin--)
    ;
  FWDGT_RLD = 400U;
  for (spin = FWDGT_RLD_TIMEOUT; spin && (FWDGT_STAT & FWDGT_STAT_RUD); spin--)
    ;
  FWDGT_CTL = FWDGT_KEY_RELOAD;
  FWDGT_CTL = FWDGT_KEY_ENABLE;
}

static void watchdog_feed(void) { FWDGT_CTL = FWDGT_KEY_RELOAD; }

static void fast_poll_mode_apply(uint8_t code) {
  bool enabled = (code & DIP_FAST_POLL_MASK) != 0;
  if (enabled == fast_poll_enabled)
    return;
  fast_poll_enabled = enabled;
  if (enabled) {
    uint16_t mask = pack_ever_seen_mask;
    if (mask == 0)
      mask = sweep_seen_mask;
    fast_poll_mask = mask;
    last_bg_probe_ms = millis();
    bg_probe_candidate = 0;
    bg_probe_in_flight = false;
    if (fast_poll_mask != 0) {
      for (uint8_t i = 0; i < 16; i++) {
        if (fast_poll_mask & (1 << i)) {
          current_addr_index = i;
          break;
        }
      }
    }
  } else {
    fast_poll_mask = 0;
    bg_probe_in_flight = false;
  }
}

static void bms_advance_poll_address(uint32_t now) {
  if (!fast_poll_enabled || fast_poll_mask == 0) {
    current_addr_index++;
    if (current_addr_index >= 16) {
      current_addr_index = 0;
      bms_aggregate_data();
      if (fast_poll_enabled && pack_ever_seen_mask != 0) {
        fast_poll_mask = pack_ever_seen_mask;
        for (uint8_t i = 0; i < 16; i++) {
          if (fast_poll_mask & (1 << i)) {
            current_addr_index = i;
            break;
          }
        }
      }
    }
    return;
  }

  if (bg_probe_in_flight) {
    bg_probe_in_flight = false;
    for (uint8_t i = 0; i < 16; i++) {
      if (fast_poll_mask & (1 << i)) {
        current_addr_index = i;
        break;
      }
    }
    return;
  }

  uint8_t next_idx = 0xFF;
  for (uint8_t a = current_addr_index + 1; a < 16; a++) {
    if (fast_poll_mask & (1 << a)) {
      next_idx = a;
      break;
    }
  }

  if (next_idx != 0xFF) {
    current_addr_index = next_idx;
    return;
  }

  // Fast sweep complete for all latched packs
  bms_aggregate_data();

  // Background discovery probe check
  if ((fast_poll_mask != 0xFFFF) && (now - last_bg_probe_ms >= BG_PROBE_INTERVAL_MS)) {
    uint8_t cand = 0xFF;
    for (uint8_t s = 0; s < 16; s++) {
      uint8_t c = (bg_probe_candidate + s) & 0x0F;
      if (!(fast_poll_mask & (1 << c))) {
        cand = c;
        break;
      }
    }
    if (cand != 0xFF) {
      bg_probe_candidate = (cand + 1) & 0x0F;
      last_bg_probe_ms = now;
      bg_probe_in_flight = true;
      current_addr_index = cand;
      return;
    }
  }

  for (uint8_t i = 0; i < 16; i++) {
    if (fast_poll_mask & (1 << i)) {
      current_addr_index = i;
      break;
    }
  }
}

static uint8_t dip_read_code(void) {
  uint8_t code = (uint8_t)((~GPIO_ISTAT(DIP_GPIO_PORT) >> 12) & 0x0FU);
  if (!(GPIO_ISTAT(DIP5_GPIO_PORT) & DIP5_GPIO_PIN)) code |= DIP_CHARGE_CURRENT_50A;
  return code;
}

static void dip_charge_profile_apply(uint8_t code) {
  if (code & DIP_PROFILE_CODE_90) {
    profile_charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_DIP90;
    profile_taper_start_mv = CELL_TAPER_START_MV_DIP90;
    profile_charge_stop_mv = CELL_CHARGE_STOP_MV_DIP90;
    profile_charge_reenable_mv = CELL_CHARGE_REENABLE_MV_DIP90;
  } else {
    profile_charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_STATIC;
    profile_taper_start_mv = CELL_TAPER_START_MV;
    profile_charge_stop_mv = CELL_CHARGE_STOP_MV;
    profile_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
  }
  profile_charge_current_per_pack_x10 =
      (code & DIP_CHARGE_CURRENT_50A) ? CHARGE_CURRENT_LIMIT_PER_PACK_X10
                                      : CHARGE_CURRENT_LIMIT_PER_PACK_DIP5_OFF_X10;
  charge_voltage_limit = profile_charge_voltage_limit;
  dip_applied_code = code;
  pcs_mode_apply(code);       // DIP2: mirror (OFF) vs inverter bridge (ON)
  fast_poll_mode_apply(code); // DIP3: full scan (OFF) vs fast polling (ON)
}

static void dip_charge_profile_init(void) {
  rcu_periph_clock_enable(DIP_GPIO_RCU);
  gpio_init(DIP_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, DIP_GPIO_PINS);
  rcu_periph_clock_enable(DIP5_GPIO_RCU);
  gpio_init(DIP5_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, DIP5_GPIO_PIN);
  dip_last_code = dip_read_code();
  dip_stable_reads = 2;
  dip_charge_profile_apply(dip_last_code);
}

static void dip_charge_profile_poll(void) {
  uint8_t code = dip_read_code();
  if (code != dip_last_code) {
    dip_last_code = code;
    dip_stable_reads = 0;
    return;
  }
  if (dip_stable_reads < 2)
    dip_stable_reads++;
  if (dip_stable_reads >= 2 && code != dip_applied_code)
    dip_charge_profile_apply(code);
}

int main(void) {
  SystemInit();
  if (SysTick_Config(SystemCoreClock / 1000U))
    while (1)
      ;
  dip_charge_profile_init();
  leds_init();
  rs485_init();
  pcs_uart_init(); // PCS-485 port (mirror/bridge, mode from DIP2)
  can_init_500k();
  watchdog_init();

  uint32_t last_bms_start = 0, last_bms_finish = 0, last_can = 0, last_dip_poll = 0;

  while (1) {
    watchdog_feed();
    uint32_t now = millis();
    pcs_poll_service(now); // mirror drain / bridge RX every pass
    if (now - last_dip_poll >= 250) {
      last_dip_poll = now;
      dip_charge_profile_poll();
      pcs_mode_apply(dip_applied_code); // retry a DIP2 switch deferred by a busy bridge
      balance_led_tick(now);
    }
    // Own BMS polls only in mirror mode — DIP2 ON is a pure bridge (no own
    // commands on the battery line at all).
    // Polling cadence: 400 ms per query (matches Deye inverter) with min 20 ms bus silence.
    if (now - last_bms_start >= BMS_POLL_PERIOD_MS && now - last_bms_finish >= BMS_POLL_GAP_MS && !pcs_bridge_mode) {
      last_bms_start = now;
      bms_send_query(current_addr_index);
      bms_receive_response();
      if (bms_parse_response(current_addr_index + 1)) {
        last_bms_response_ms = millis();
        if (fast_poll_enabled && bg_probe_in_flight) {
          fast_poll_mask |= (1 << current_addr_index);
          pack_ever_seen_mask |= (1 << current_addr_index);
        }
      }
      bms_advance_poll_address(millis());
      last_bms_finish = millis();
    }

    // Pure bridge (DIP2 ON): relay inverter <-> BMS and keep CAN fed from the
    // same traffic (parse happens inside the step; aggregates close per sweep).
    if (pcs_bridge_mode) {
      pcs_bridge_step(now);
      bridge_sweep_tick(now);
    }

    if (millis() - last_bms_response_ms > BMS_DEFAULT_TIMEOUT_MS) {
      balance_on_bms_lost(); // an auto session cannot run blind
      agg_reset(); // не змішувати старі накопичення з новими після відновлення
      bms_data_valid = false;
      bms_voltage = BATTERY_VOLTAGE_DEFAULT;
      bms_current = BATTERY_CURRENT_DEFAULT;
      bms_soc = BATTERY_SOC_DEFAULT;
      bms_soh = BATTERY_SOH_DEFAULT;
      bms_temp = BATTERY_TEMP_DEFAULT;
      soc_ema_valid = false; // наступний sweep snap без фільтра
      charge_voltage_limit = profile_charge_voltage_limit;
      charge_current_limit = CHARGE_CURRENT_LIMIT_DEFAULT;
      discharge_current_limit = DISCHARGE_CURRENT_LIMIT_DEFAULT;
      charge_cell_ov_inhibit = false;
      charge_temp_inhibit = false;
      discharge_cell_uv_inhibit = false;
      discharge_temp_inhibit = false;
      can_protect_b0 = 0;
      can_protect_b1 = 0;
      can_alarm_b2 = 0;
      can_alarm_b3 = 0;
      sweep_protect_b0 = 0;
      sweep_protect_b1 = 0;
      sweep_alarm_b2 = 0;
      sweep_alarm_b3 = 0;
    }

    if (now - last_can >= 1000) {
      last_can = now;
      send_all_frames();
    }

    // LED4 (PC5, CAN): release ~100 ms after the last CAN burst.
    if (can_led_off_ms && (int32_t)(millis() - can_led_off_ms) >= 0) {
      gpio_bit_reset(LED_PCSCAN_PORT, LED_PCSCAN_PIN);
      can_led_off_ms = 0;
    }

    // LED3 (PC4, PCS-485): release ~100 ms after the last mirror/bridge byte.
    if (pcs_led_off_ms && (int32_t)(millis() - pcs_led_off_ms) >= 0) {
      gpio_bit_reset(LED_INV485_PORT, LED_INV485_PIN);
      pcs_led_off_ms = 0;
    }
  }
}
