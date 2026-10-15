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
 */

#include "gd32f30x.h"
#include <stdbool.h>
#include <string.h>

#define BMS_BAUDRATE 9600
#define BMS_TIMEOUT_MS 200
#define BMS_RESPONSE_LEN 83
#define BMS_POLL_PERIOD_MS 200
#define BMS_DEFAULT_TIMEOUT_MS 4000

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
// Per-pack charge current limit, selected by DIP1 (see DIP_CHARGE_CURRENT_50A):
// OFF = 40 A (soft margin), ON = 50 A (battery manufacturer maximum).
#define CHARGE_CURRENT_LIMIT_PER_PACK_DIP1_OFF_X10 400
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
// 90% profile = LAST DIP switch ON (SW4, weight 8 => code 0b1000); stock SW1
// (PYLON, code 1) stays on the default 100% voltage profile.
#define DIP_PROFILE_CODE_90 8
// DIP1 (SW1, weight 1) selects the per-pack charge current limit:
// OFF = 40 A, ON = 50 A. Orthogonal to the SW4 voltage profile.
#define DIP_CHARGE_CURRENT_50A 0x01

// Physical cell over-voltage threshold for BMS hardware protection confirmation (Q17).
// Independent of DIP90 taper target so that software 90% profile never generates false alarms:
#define CELL_PHYSICAL_OV_CONFIRM_MV 3600
#define CELL_PHYSICAL_OV_WARN_CONFIRM_MV 3550

// Vision V-LFP48100 cell protection thresholds (HMI ground truth: Warn 2800 mV, Protect 2500 mV, Release 3000 mV):
#define CELL_UV_WARN_MV 2800
#define CELL_DISCHARGE_STOP_MV 2650
#define CELL_DISCHARGE_REENABLE_MV 3000
#define CELL_MAX_DELTA_MV 400

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

#define DEYE_CAN CAN0
#define DEYE_CAN_RCU RCU_CAN0
#define DEYE_CAN_GPIO_PORT GPIOB
#define DEYE_CAN_GPIO_RCU RCU_GPIOB
#define DEYE_CAN_RX_PIN GPIO_PIN_8
#define DEYE_CAN_TX_PIN GPIO_PIN_9

// RS485 transceivers have no DE/RE pin (auto-direction, ORIGINAL_FIRMWARE.md
// §14.3): PC5 is NOT a direction pin — stock uses it as the PCS-CAN activity
// LED (§14.7), so PC5 must not be driven during RS485 TX.
// LED mapping follows the factory panel (ORIGINAL_FIRMWARE.md §14.7):
//   PC4 = Running (static HIGH), PB1 = BMS-485 activity,
//   PC5 = PCS-CAN activity, PC13+PB0 = PCS-485 pair (dual-gpio drive).
#define LED_RUNNING_PORT GPIOC
#define LED_RUNNING_PIN GPIO_PIN_4
#define LED_BMS485_PORT GPIOB
#define LED_BMS485_PIN GPIO_PIN_1
#define LED_PCSCAN_PORT GPIOC
#define LED_PCSCAN_PIN GPIO_PIN_5
#define LED_PCS485_A_PORT GPIOC
#define LED_PCS485_A_PIN GPIO_PIN_13
#define LED_PCS485_B_PORT GPIOB
#define LED_PCS485_B_PIN GPIO_PIN_0

// Stock boot state for PB10 (PP HIGH) and PB11 (OD HIGH) — likely transceiver
// enables (ORIGINAL_FIRMWARE.md §14.4); released HIGH like the factory firmware.
#define TRANSCEIVER_ENA_PORT GPIOB
#define TRANSCEIVER_ENA_PIN GPIO_PIN_10
#define TRANSCEIVER_ENB_PORT GPIOB
#define TRANSCEIVER_ENB_PIN GPIO_PIN_11

#define DIP_GPIO_PORT GPIOB
#define DIP_GPIO_RCU RCU_GPIOB
#define DIP_GPIO_PINS (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

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
static uint32_t agg_soc_sum = 0;
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
  // Running LED: steady ON (factory semantics).
  gpio_init(LED_RUNNING_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_RUNNING_PIN);
  gpio_bit_set(LED_RUNNING_PORT, LED_RUNNING_PIN);
  // BMS-485 LED: ON while a poll is in flight.
  gpio_init(LED_BMS485_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_BMS485_PIN);
  gpio_bit_reset(LED_BMS485_PORT, LED_BMS485_PIN);
  // PCS-CAN LED: ON around each CAN burst, cleared ~100 ms after TX.
  gpio_init(LED_PCSCAN_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_PCSCAN_PIN);
  gpio_bit_reset(LED_PCSCAN_PORT, LED_PCSCAN_PIN);
  // PCS-485 LED pair: dual-gpio drive, complementary; OFF = A LOW / B HIGH.
  gpio_init(LED_PCS485_A_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_PCS485_A_PIN);
  gpio_init(LED_PCS485_B_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_PCS485_B_PIN);
  gpio_bit_reset(LED_PCS485_A_PORT, LED_PCS485_A_PIN);
  gpio_bit_set(LED_PCS485_B_PORT, LED_PCS485_B_PIN);
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

static void rs485_flush_rx(void) {
  while (usart_flag_get(USART0, USART_FLAG_RBNE) != RESET)
    (void)usart_data_receive(USART0); // залишкові байти обірваного кадру
  rs485_clear_errors();
}

static void bms_send_query(uint8_t addr_index) {
  rs485_flush_rx();
  uart_rx_index = 0;
  gpio_bit_set(LED_BMS485_PORT, LED_BMS485_PIN); // poll in flight
  // No DE pin on the transceiver (auto-direction, ORIGINAL_FIRMWARE.md §14.3):
  // just shift the frame out; direction switches in hardware.
  for (uint8_t i = 0; i < 8; i++)
    rs485_send_byte(BMS_QUERY_TABLE[addr_index][i]);
}

static uint16_t bms_receive_response(void) {
  uart_rx_index = 0;
  uint32_t start_ms = millis();
  uint32_t wait_start = start_ms;
  while (millis() - wait_start < BMS_TIMEOUT_MS) {
    if (millis() - start_ms >= 500)
      break; // жорсткий дедлайн від першого байта
    if (usart_flag_get(USART0, USART_FLAG_RBNE) != RESET) {
      uint8_t byte = usart_data_receive(USART0);
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
  if (uart_rx_index < BMS_RESPONSE_LEN)
    return false;
  if (uart_rx_buffer[0] != expected_addr || uart_rx_buffer[1] != 0x03 ||
      uart_rx_buffer[2] != 0x4E)
    return false;

  uint16_t received_crc = (uart_rx_buffer[BMS_RESPONSE_LEN - 1] << 8) |
                          uart_rx_buffer[BMS_RESPONSE_LEN - 2];
  uint16_t calculated_crc =
      modbus_crc16((uint8_t *)uart_rx_buffer, BMS_RESPONSE_LEN - 2);
  if (received_crc != calculated_crc)
    return false;

  uint16_t raw_voltage = (uart_rx_buffer[3] << 8) | uart_rx_buffer[4];
  int16_t raw_current = (int16_t)((uart_rx_buffer[5] << 8) | uart_rx_buffer[6]);
  uint16_t raw_soc = (uart_rx_buffer[45] << 8) | uart_rx_buffer[46];
  uint16_t raw_soh = (uart_rx_buffer[47] << 8) | uart_rx_buffer[48];

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

  // Status, Warning, Protection, Error registers (regs 25..28, bytes 53..60)
  uint16_t reg_status  = ((uint16_t)uart_rx_buffer[53] << 8) | uart_rx_buffer[54];
  uint16_t reg_warning = ((uint16_t)uart_rx_buffer[55] << 8) | uart_rx_buffer[56];
  uint16_t reg_protect = ((uint16_t)uart_rx_buffer[57] << 8) | uart_rx_buffer[58];
  uint16_t reg_error   = ((uint16_t)uart_rx_buffer[59] << 8) | uart_rx_buffer[60];

  // Cross-validate with physical measurements to reject transient Boot Latch flags (Q17):
  // Confirmation uses physical threshold CELL_PHYSICAL_OV_CONFIRM_MV (3600 mV), NOT profile_charge_stop_mv,
  // so that activating the 90% software taper profile never generates false emergency cell OV alarms:
  if ((reg_protect & 0x0002) && agg_max_cell_mv >= CELL_PHYSICAL_OV_CONFIRM_MV)
    sweep_protect_b0 |= (CAN_PROT_CELL_OV | CAN_PROT_PACK_OV);
  if ((reg_protect & 0x0008) && (agg_min_cell_mv <= CELL_UV_WARN_MV || (raw_soc & 0xFF) <= 5))
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
  if ((reg_warning & 0x0008) && (agg_min_cell_mv <= CELL_UV_WARN_MV || (raw_soc & 0xFF) <= 10))
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

  if (expected_addr >= 1 && expected_addr <= 16)
    sweep_seen_mask |= (1 << (expected_addr - 1));

  agg_voltage_sum += raw_voltage / 100.0f;
  agg_current_sum += raw_current / 100.0f;
  agg_soc_sum += (uint8_t)(raw_soc & 0xFF);
  agg_soh_sum += (uint8_t)(raw_soh & 0xFF);
  agg_count++;

  return true;
}

static uint16_t calculate_charge_limit(uint8_t pack_count) {
  uint32_t max_current_x10 =
      (uint32_t)profile_charge_current_per_pack_x10 * pack_count;

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

  // Cell voltage cut-off: stop at profile stop, re-enable below re-enable threshold
  if (agg_max_cell_mv >= profile_charge_stop_mv) {
    charge_cell_ov_inhibit = true;
  } else if (charge_cell_ov_inhibit) {
    if (agg_max_cell_mv < profile_charge_reenable_mv) {
      charge_cell_ov_inhibit = false;
    }
  }

  if (charge_temp_inhibit || charge_cell_ov_inhibit) {
    return 0;
  }

  if (agg_max_cell_mv <= profile_taper_start_mv)
    return (uint16_t)max_current_x10;

  return (uint16_t)(max_current_x10 *
                    (profile_charge_stop_mv - agg_max_cell_mv) /
                    (profile_charge_stop_mv - profile_taper_start_mv));
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

  // Cell voltage cut-off: hard cut <= 2650 mV, re-enable >= 3000 mV (Vision HMI Release)
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

  // Soft cut / Cell UV warning (<= 2800 mV): derate discharge current to 50%
  if (agg_min_cell_mv <= CELL_UV_WARN_MV) {
    max_current_x10 /= 2;
  }

  // Severe cell imbalance check (> 400 mV delta): derate to 50%
  if (agg_max_cell_mv > agg_min_cell_mv && (agg_max_cell_mv - agg_min_cell_mv) > CELL_MAX_DELTA_MV) {
    max_current_x10 /= 2;
  }

  return (uint16_t)max_current_x10;
}

static void agg_reset(void) {
  agg_voltage_sum = 0.0f;
  agg_current_sum = 0.0f;
  agg_soc_sum = 0;
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
  for (uint8_t i = 0; i < 16; i++) {
    if (sweep_seen_mask & (1 << i)) {
      pack_ever_seen_mask |= (1 << i);
      pack_miss_count[i] = 0;
      active_pack_count++;
    } else if (pack_ever_seen_mask & (1 << i)) {
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
  bms_soc = (uint8_t)(agg_soc_sum / agg_count);
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

  uint8_t effective_packs = (active_pack_count > 0) ? active_pack_count : agg_count;
  bms_pack_count = effective_packs;

  charge_voltage_limit = profile_charge_voltage_limit;
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
  gpio_bit_set(LED_PCSCAN_PORT, LED_PCSCAN_PIN); // PCS-CAN activity
  uint8_t data[8] = {0};
  put_u16le(&data[0], charge_voltage_limit);
  put_u16le(&data[2], charge_current_limit);
  put_u16le(&data[4], discharge_current_limit);
  put_u16le(&data[6], DISCHARGE_VOLTAGE_LIMIT_X10);
  send_can_std(CAN_ID_LIMITS, data, 8);

  memset(data, 0, 8);
  put_u16le(&data[0], bms_soc);
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
  memset(data, 0, 8);
  uint8_t req_flags = 0x00;
  if (bms_data_valid) {
    if (!charge_cell_ov_inhibit && !charge_temp_inhibit && charge_current_limit > 0)
      req_flags |= 0x80;
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

static uint8_t dip_applied_code = 0xFF;
static uint8_t dip_last_code = 0;
static uint8_t dip_stable_reads = 0;

static uint8_t dip_read_code(void) {
  return (uint8_t)((~GPIO_ISTAT(DIP_GPIO_PORT) >> 12) & 0x0FU);
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
                                      : CHARGE_CURRENT_LIMIT_PER_PACK_DIP1_OFF_X10;
  charge_voltage_limit = profile_charge_voltage_limit;
  dip_applied_code = code;
}

static void dip_charge_profile_init(void) {
  rcu_periph_clock_enable(DIP_GPIO_RCU);
  gpio_init(DIP_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, DIP_GPIO_PINS);
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
  can_init_500k();
  watchdog_init();

  uint32_t last_bms = 0, last_can = 0, led_pcs485_timer = 0, last_dip_poll = 0;
  uint8_t current_addr_index = 0, led_pcs485_state = 0;

  while (1) {
    watchdog_feed();
    uint32_t now = millis();
    if (now - last_dip_poll >= 250) {
      last_dip_poll = now;
      dip_charge_profile_poll();
    }
    if (now - last_bms >= BMS_POLL_PERIOD_MS) {
      last_bms = now;
      bms_send_query(current_addr_index);
      bms_receive_response();
      if (bms_parse_response(current_addr_index + 1)) {
        last_bms_response_ms = millis();
      }
      if (++current_addr_index >= 16) {
        current_addr_index = 0;
        bms_aggregate_data();
      }
    }

    if (millis() - last_bms_response_ms > BMS_DEFAULT_TIMEOUT_MS) {
      agg_reset(); // не змішувати старі накопичення з новими після відновлення
      bms_data_valid = false;
      bms_voltage = BATTERY_VOLTAGE_DEFAULT;
      bms_current = BATTERY_CURRENT_DEFAULT;
      bms_soc = BATTERY_SOC_DEFAULT;
      bms_soh = BATTERY_SOH_DEFAULT;
      bms_temp = BATTERY_TEMP_DEFAULT;
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

    // PCS-485 LED pair: free-run blink until the UART4 slave exists.
    if (now - led_pcs485_timer >= 500) {
      led_pcs485_timer = now;
      led_pcs485_state = !led_pcs485_state;
      if (led_pcs485_state) {
        gpio_bit_set(LED_PCS485_A_PORT, LED_PCS485_A_PIN);
        gpio_bit_reset(LED_PCS485_B_PORT, LED_PCS485_B_PIN);
      } else {
        gpio_bit_reset(LED_PCS485_A_PORT, LED_PCS485_A_PIN);
        gpio_bit_set(LED_PCS485_B_PORT, LED_PCS485_B_PIN);
      }
    }

    // PCS-CAN LED: release ~100 ms after the last CAN burst.
    if (can_led_off_ms && (int32_t)(millis() - can_led_off_ms) >= 0) {
      gpio_bit_reset(LED_PCSCAN_PORT, LED_PCSCAN_PIN);
      can_led_off_ms = 0;
    }
  }
}
