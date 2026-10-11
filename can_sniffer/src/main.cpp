/**
 * CAN bus sniffer — passive listen-only, 500 kbit/s, serial 115200.
 *
 * Two hardware targets, selected by the PlatformIO env:
 *
 *  A) Arduino Nano + MCP2515 + TJA1050 (5 V)
 *     VCC->5V, GND->GND(+bus GND), SCK->D13, MISO->D12, MOSI->D11,
 *     CS->D10, INT->D2 (unused, poll mode).
 *     MCP2515 crystal: cheap boards ship 8 MHz; some are 16 MHz (CAN_CLK).
 *
 *  B) ESP32 (LuaNode32 / ESP-WROOM-32) + SN65HVD230 (3.3 V, built-in TWAI)
 *     3V3->VCC, GND->GND(+bus GND), TX->GPIO5, RX->GPIO4 (CAN_*_GPIO).
 *
 * Bus side (COM-Box PCS-CAN RJ45): pin 8 = CANH, pin 7 = CANL, pin 1 = GND.
 * Listen-only: the sniffer never ACKs and never transmits.
 * Output: raw hex dump + decode of 0x351/0x355/0x356/0x359/0x35C/0x35E/0x305.
 */

#include <Arduino.h>

#if defined(ESP32)

#include <driver/twai.h>

#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

static bool can_ok = false;

static void can_begin(void) {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_GPIO,
                                   (gpio_num_t)CAN_RX_GPIO,
                                   TWAI_MODE_LISTEN_ONLY);
  g_config.rx_queue_len = 64;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  while (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
         twai_start() != ESP_OK) {
    Serial.println(F("TWAI init fail, retrying..."));
    delay(500);
  }
  can_ok = true;
}

static bool can_recv(uint32_t *id, bool *ext, uint8_t *len, uint8_t *buf) {
  twai_message_t msg;
  if (!can_ok || twai_receive(&msg, 0) != ESP_OK)
    return false;
  if (msg.rtr)
    return false; // remote frames carry no data
  *id = msg.identifier;
  *ext = (msg.flags & TWAI_MSG_FLAG_EXTD) != 0;
  *len = msg.data_length_code;
  memcpy(buf, msg.data, *len > 8 ? 8 : *len);
  return true;
}

static void can_stats_line(uint32_t fps, uint32_t total) {
  twai_status_info_t st;
  Serial.print(F("--- "));
  Serial.print(millis());
  Serial.print(F(" ms | "));
  Serial.print(fps);
  Serial.print(F(" fps | total "));
  Serial.print(total);
  if (twai_get_status_info(&st) == ESP_OK) {
    Serial.print(F(" | TEC="));
    Serial.print(st.tx_error_counter);
    Serial.print(F(" REC="));
    Serial.print(st.rx_error_counter);
    Serial.print(F(" missed="));
    Serial.print(st.rx_missed_count);
    Serial.print(F(" state="));
    Serial.print(st.state);
  }
  Serial.println(F(" ---"));
}

#else // Arduino Nano + MCP2515 + TJA1050

#include <SPI.h>
#include <mcp_can.h>

#define SPI_CS_PIN 10
#define CAN_CLK MCP_8MHZ  // module crystal: 8 MHz (MCP_16MHZ for 16 MHz)
#define CAN_BPS CAN_500KBPS

MCP_CAN CAN(SPI_CS_PIN);
static bool can_ok = false;

static void can_begin(void) {
  SPI.begin();
  while (CAN.begin(MCP_ANY, CAN_BPS, CAN_CLK) != CAN_OK) {
    Serial.println(F("MCP2515 init fail, check wiring/crystal, retrying..."));
    delay(500);
  }
  CAN.setMode(MCP_LISTENONLY);
  can_ok = true;
}

static bool can_recv(uint32_t *id, bool *ext, uint8_t *len, uint8_t *buf) {
  if (!can_ok || CAN_MSGAVAIL != CAN.checkReceive())
    return false;
  uint8_t e = 0;
  if (CAN.readMsgBuf(id, &e, len, buf) != CAN_OK)
    return false;
  *ext = e != 0;
  return true;
}

static void can_stats_line(uint32_t fps, uint32_t total) {
  Serial.print(F("--- "));
  Serial.print(millis());
  Serial.print(F(" ms | "));
  Serial.print(fps);
  Serial.print(F(" fps | total "));
  Serial.print(total);
  Serial.print(F(" | TEC="));
  Serial.print(CAN.errorCountTX(), DEC);
  Serial.print(F(" REC="));
  Serial.print(CAN.errorCountRX(), DEC);
  Serial.println(F(" ---"));
}

#endif // ESP32 / AVR

// ---------------------------------------------------------------------------

static const uint32_t STATS_PERIOD_MS = 5000;
static uint32_t frames_total = 0;
static uint32_t frames_window = 0;
static uint32_t stats_ms = 0;
static uint32_t boot_ms = 0;

static uint16_t u16le(const uint8_t *b) {
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static int16_t i16le(const uint8_t *b) { return (int16_t)u16le(b); }

static void decode_known(uint32_t id, const uint8_t *buf, uint8_t len) {
  switch (id) {
  case 0x351:
    if (len >= 8) {
      Serial.print(F(" | CVL="));
      Serial.print(u16le(&buf[0]) / 10.0f, 1);
      Serial.print(F("V CCL="));
      Serial.print(u16le(&buf[2]) / 10.0f, 1);
      Serial.print(F("A DCL="));
      Serial.print(u16le(&buf[4]) / 10.0f, 1);
      Serial.print(F("A DVL="));
      Serial.print(u16le(&buf[6]) / 10.0f, 1);
      Serial.print(F("V"));
    }
    break;
  case 0x355:
    if (len >= 4) {
      Serial.print(F(" | SOC="));
      Serial.print(u16le(&buf[0]));
      Serial.print(F("% SOH="));
      Serial.print(u16le(&buf[2]));
      Serial.print(F("%"));
    }
    break;
  case 0x356:
    if (len >= 6) {
      Serial.print(F(" | V="));
      Serial.print(u16le(&buf[0]) / 100.0f, 2);
      Serial.print(F(" I="));
      Serial.print(i16le(&buf[2]) / 10.0f, 1);
      Serial.print(F(" T="));
      Serial.print(i16le(&buf[4]) / 10.0f, 1);
      Serial.print(F("C"));
    }
    break;
  case 0x359:
    if (len >= 7) {
      Serial.print(F(" | prot="));
      Serial.print(buf[0], HEX);
      Serial.print(buf[1], HEX);
      Serial.print(buf[2], HEX);
      Serial.print(buf[3], HEX);
      Serial.print(F(" packs="));
      Serial.print(buf[4]);
      Serial.print(F(" mark="));
      Serial.write(buf[5]);
      Serial.write(buf[6]);
    }
    break;
  case 0x35C:
    if (len >= 2) {
      Serial.print(F(" | flags="));
      Serial.print(buf[0], HEX);
      Serial.print(buf[1], HEX);
    }
    break;
  case 0x35E:
    if (len >= 5) {
      Serial.print(F(" | brand=\""));
      for (uint8_t i = 0; i < len && i < 8; i++) {
        if (buf[i] >= 32 && buf[i] < 127)
          Serial.write(buf[i]);
      }
      Serial.print('"');
    }
    break;
  case 0x305:
    if (len >= 4) {
      Serial.print(F(" | PCS-heartbeat SOC="));
      Serial.print(buf[0]);
      Serial.print(F("% V="));
      Serial.print(u16le(&buf[1]) / 10.0f, 1);
      Serial.print(F("V"));
    }
    break;
  default:
    break;
  }
}

static void print_frame(uint32_t now, uint32_t id, bool ext, uint8_t len,
                        const uint8_t *buf) {
  Serial.print('[');
  if (now < 100000)
    Serial.print(' ');
  if (now < 10000)
    Serial.print(' ');
  if (now < 1000)
    Serial.print(' ');
  Serial.print(now);
  Serial.print(F("] "));
  if (id < 0x1000)
    Serial.print(' ');
  if (id < 0x100)
    Serial.print(' ');
  if (id < 0x10)
    Serial.print(' ');
  Serial.print(id, HEX);
  Serial.print(ext ? F(" EXT ") : F(" STD "));
  Serial.print(len, DEC);
  for (uint8_t i = 0; i < len && i < 8; i++) {
    Serial.print(' ');
    if (buf[i] < 0x10)
      Serial.print('0');
    Serial.print(buf[i], HEX);
  }
  decode_known(id, buf, len);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ;

#if defined(ESP32)
  Serial.println(F("ESP32 TWAI CAN sniffer: 500 kbps, listen-only"));
#else
  Serial.println(F("MCP2515 CAN sniffer: 500 kbps, listen-only"));
#endif
  can_begin();
  Serial.println(F("up. Raw hex + decode of 0x351/355/356/359/35C/35E/305"));

  boot_ms = millis();
  stats_ms = millis();
}

void loop() {
  uint32_t id = 0;
  bool ext = false;
  uint8_t len = 0, buf[8] = {0};
  if (can_recv(&id, &ext, &len, buf)) {
    print_frame(millis(), id, ext, len, buf);
    frames_total++;
    frames_window++;
  }

  uint32_t now = millis();
  if (now - stats_ms >= STATS_PERIOD_MS) {
    can_stats_line(frames_window * 1000UL / STATS_PERIOD_MS, frames_total);
    stats_ms = now;
    frames_window = 0;
  }
}
