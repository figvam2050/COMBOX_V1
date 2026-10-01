/**
 * BMS 485 to CAN Converter for Deye Inverter (Li Mode 00 - Vision/Megarevo)
 * Hardware: GD32F305 + ISO1050 (CAN) + CA-IS3080WX (RS485)
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

#define BATTERY_SOC_DEFAULT 77
#define BATTERY_SOH_DEFAULT 88
#define BATTERY_VOLTAGE_DEFAULT 52.2f
#define BATTERY_CURRENT_DEFAULT 0.0f
#define BATTERY_TEMP_DEFAULT 45.0f
#define CHARGE_VOLTAGE_LIMIT_DEFAULT 533
#define CHARGE_CURRENT_LIMIT_DEFAULT 0
#define DISCHARGE_CURRENT_LIMIT_DEFAULT 0

#define CHARGE_VOLTAGE_LIMIT_STATIC 533
#define CHARGE_CURRENT_LIMIT_PER_PACK_X10 900
#define DISCHARGE_CURRENT_LIMIT_PER_PACK_X10 1000
#define DISCHARGE_VOLTAGE_LIMIT_X10 450

// Max-cell charge taper: 90 A/pack to 0 A between 3.400 V and 3.550 V.
// Re-enabling below 3.500 V prevents charge-limit oscillation at the cutoff.
#define CELL_TAPER_START_MV 3400
#define CELL_CHARGE_STOP_MV 3550
#define CELL_CHARGE_REENABLE_MV 3500

#define CAN_ID_HEARTBEAT 0x305
#define CAN_ID_LIMITS 0x351
#define CAN_ID_SOC 0x355
#define CAN_ID_STATUS 0x356
#define CAN_ID_FLAGS 0x35C

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

#define LED1_PORT GPIOC
#define LED1_PIN GPIO_PIN_13
#define LED3_PORT GPIOC
#define LED3_PIN GPIO_PIN_4
#define LED4_PORT GPIOC
#define LED4_PIN GPIO_PIN_5
#define LED5_PORT GPIOB
#define LED5_PIN GPIO_PIN_0
#define LED6_PORT GPIOB
#define LED6_PIN GPIO_PIN_1

static volatile uint32_t g_systick_ms = 0;
static volatile uint8_t uart_rx_buffer[256];
static volatile uint16_t uart_rx_index = 0;

static float bms_voltage = BATTERY_VOLTAGE_DEFAULT;
static float bms_current = BATTERY_CURRENT_DEFAULT;
static uint8_t bms_soc = BATTERY_SOC_DEFAULT;
static float bms_temp = BATTERY_TEMP_DEFAULT;
static uint8_t bms_soh = BATTERY_SOH_DEFAULT;

static float agg_voltage_sum = 0.0f;
static float agg_current_sum = 0.0f;
static uint32_t agg_soc_sum = 0;
static uint32_t agg_soh_sum = 0;
static float agg_temp_max = 0.0f;
static uint8_t agg_count = 0;
static uint16_t agg_max_cell_mv = 0;
static bool charge_inhibit = false;

static uint16_t charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_DEFAULT;
static uint16_t charge_current_limit = CHARGE_CURRENT_LIMIT_DEFAULT;
static uint16_t discharge_current_limit = DISCHARGE_CURRENT_LIMIT_DEFAULT;

static volatile uint32_t last_bms_response_ms = 0;
static volatile uint32_t last_can_ack_ms = 0;
static volatile uint32_t led6_timer = 0;
static volatile uint8_t led6_state = 0;

extern "C" void osSystickHandler(void) { g_systick_ms++; }
static uint32_t millis(void) { return g_systick_ms; }

static void leds_init(void) {
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  gpio_init(LED1_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED1_PIN);
  gpio_init(LED3_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED3_PIN);
  gpio_init(LED4_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED4_PIN);
  gpio_init(LED5_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED5_PIN);
  gpio_init(LED6_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED6_PIN);
  gpio_bit_reset(LED1_PORT, LED1_PIN);
  gpio_bit_reset(LED3_PORT, LED3_PIN);
  gpio_bit_reset(LED6_PORT, LED6_PIN);
  gpio_bit_set(LED4_PORT, LED4_PIN);
  gpio_bit_set(LED5_PORT, LED5_PIN);
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

static void bms_send_query(uint8_t addr_index) {
  uart_rx_index = 0;
  for (uint8_t i = 0; i < 8; i++)
    rs485_send_byte(BMS_QUERY_TABLE[addr_index][i]);
  if (millis() - led6_timer >= 500) {
    led6_timer = millis();
    led6_state = !led6_state;
    if (led6_state)
      gpio_bit_set(LED6_PORT, LED6_PIN);
    else
      gpio_bit_reset(LED6_PORT, LED6_PIN);
  }
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
  usart_flag_clear(USART0, USART_FLAG_ORERR);
  usart_flag_clear(USART0, USART_FLAG_NERR);
  usart_flag_clear(USART0, USART_FLAG_FERR);
  usart_flag_clear(USART0, USART_FLAG_PERR);
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
  uint16_t raw_temp = (uart_rx_buffer[41] << 8) | uart_rx_buffer[42];
  uint16_t raw_soc = (uart_rx_buffer[45] << 8) | uart_rx_buffer[46];
  uint16_t raw_soh = (uart_rx_buffer[47] << 8) | uart_rx_buffer[48];

  for (uint8_t cell = 0; cell < 15; cell++) {
    uint16_t cell_mv = ((uint16_t)uart_rx_buffer[7 + cell * 2] << 8) |
                       uart_rx_buffer[8 + cell * 2];
    if (cell_mv > agg_max_cell_mv)
      agg_max_cell_mv = cell_mv;
  }

  agg_voltage_sum += raw_voltage / 100.0f;
  agg_current_sum += raw_current / 100.0f;
  agg_soc_sum += (uint8_t)(raw_soc & 0xFF);
  agg_soh_sum += (uint8_t)(raw_soh & 0xFF);
  if ((float)raw_temp > agg_temp_max)
    agg_temp_max = (float)raw_temp;
  agg_count++;

  return true;
}

static uint16_t calculate_charge_limit(uint8_t pack_count) {
  uint32_t max_current_x10 =
      (uint32_t)CHARGE_CURRENT_LIMIT_PER_PACK_X10 * pack_count;

  if (charge_inhibit) {
    if (agg_max_cell_mv >= CELL_CHARGE_REENABLE_MV)
      return 0;
    charge_inhibit = false;
  }

  if (agg_max_cell_mv >= CELL_CHARGE_STOP_MV) {
    charge_inhibit = true;
    return 0;
  }

  if (agg_max_cell_mv <= CELL_TAPER_START_MV)
    return (uint16_t)max_current_x10;

  return (uint16_t)(max_current_x10 *
                    (CELL_CHARGE_STOP_MV - agg_max_cell_mv) /
                    (CELL_CHARGE_STOP_MV - CELL_TAPER_START_MV));
}

static void bms_aggregate_data(void) {
  if (agg_count == 0)
    return;

  bms_voltage = agg_voltage_sum / (float)agg_count;
  bms_current = agg_current_sum;
  bms_soc = (uint8_t)(agg_soc_sum / agg_count);
  bms_soh = (uint8_t)(agg_soh_sum / agg_count);
  bms_temp = agg_temp_max;

  charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_STATIC;
  charge_current_limit = calculate_charge_limit(agg_count);
  discharge_current_limit =
      DISCHARGE_CURRENT_LIMIT_PER_PACK_X10 * agg_count;

  agg_voltage_sum = 0.0f;
  agg_current_sum = 0.0f;
  agg_soc_sum = 0;
  agg_soh_sum = 0;
  agg_temp_max = 0.0f;
  agg_count = 0;
  agg_max_cell_mv = 0;
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
  if (can_transmit_states(DEYE_CAN, mbox) == CAN_TRANSMIT_OK) {
    last_can_ack_ms = millis();
  }
}

static void put_u16le(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)(value & 0xFF);
  data[1] = (uint8_t)((value >> 8) & 0xFF);
}

static void send_all_frames(void) {
  uint8_t data[8] = {0};
  send_can_std(CAN_ID_HEARTBEAT, data, 8);
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
  put_u16le(&data[0], (uint16_t)(bms_voltage * 100));
  put_u16le(&data[2], (uint16_t)(bms_current * 10));
  put_u16le(&data[4], (uint16_t)(bms_temp * 10));
  send_can_std(CAN_ID_STATUS, data, 6);
  memset(data, 0, 8);
  data[0] = 0xC0;
  send_can_std(CAN_ID_FLAGS, data, 2);
}

int main(void) {
  SystemInit();
  if (SysTick_Config(SystemCoreClock / 1000U))
    while (1)
      ;
  leds_init();
  rs485_init();
  can_init_500k();

  uint32_t last_bms = 0, last_can = 0, led5_timer = 0, led4_timer = 0,
           led3_timer = 0;
  uint8_t current_addr_index = 0, led5_state = 0, led4_state = 0;

  while (1) {
    uint32_t now = millis();
    if (now - last_bms >= BMS_POLL_PERIOD_MS) {
      last_bms = now;
      bms_send_query(current_addr_index);
      bms_receive_response();
      if (bms_parse_response(current_addr_index + 1)) {
        last_bms_response_ms = millis();
        gpio_bit_set(LED3_PORT, LED3_PIN);
        led3_timer = millis();
      }
      if (++current_addr_index >= 16) {
        current_addr_index = 0;
        bms_aggregate_data();
      }
    }

    if (millis() - last_bms_response_ms > BMS_DEFAULT_TIMEOUT_MS) {
      bms_voltage = BATTERY_VOLTAGE_DEFAULT;
      bms_current = BATTERY_CURRENT_DEFAULT;
      bms_soc = BATTERY_SOC_DEFAULT;
      bms_soh = BATTERY_SOH_DEFAULT;
      bms_temp = BATTERY_TEMP_DEFAULT;
      charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_DEFAULT;
      charge_current_limit = CHARGE_CURRENT_LIMIT_DEFAULT;
      discharge_current_limit = DISCHARGE_CURRENT_LIMIT_DEFAULT;
    }

    if (now - last_can >= 1000) {
      last_can = now;
      send_all_frames();
    }
    if (led3_timer && (millis() - led3_timer >= 50)) {
      gpio_bit_reset(LED3_PORT, LED3_PIN);
      led3_timer = 0;
    }

    if ((millis() - last_can_ack_ms) < 2000) {
      if (millis() - led4_timer >= 250) {
        led4_timer = millis();
        led4_state = !led4_state;
        if (led4_state)
          gpio_bit_set(LED4_PORT, LED4_PIN);
        else
          gpio_bit_reset(LED4_PORT, LED4_PIN);
      }
    } else
      gpio_bit_set(LED4_PORT, LED4_PIN);

    if ((millis() - last_bms_response_ms) < 2000) {
      if (millis() - led5_timer >= 500) {
        led5_timer = millis();
        led5_state = !led5_state;
        if (led5_state)
          gpio_bit_set(LED5_PORT, LED5_PIN);
        else
          gpio_bit_reset(LED5_PORT, LED5_PIN);
      }
    } else
      gpio_bit_set(LED5_PORT, LED5_PIN);
  }
}
