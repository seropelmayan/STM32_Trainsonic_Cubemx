/**
 ******************************************************************************
 * @file    ts_link.c
 * @brief   ESP-IDF UART link to the STM32 motor controller. Event-queue RX task
 *          + SLIP/CRC16 framing + a thin sequence/ACK/retransmit layer for
 *          reliable messages + shared telemetry snapshot. Matches STM32 esp_link.c.
 *
 *          Frame: [id][seq][len][payload][crc16-LE]. seq==0 => fire-and-forget;
 *          seq!=0 => reliable (receiver ACKs, sender retransmits until ACKed).
 *
 * NOTE: builds in the ESP32 (ESP-IDF) project, not the STM32 toolchain here.
 ******************************************************************************
 */
#include "ts_link.h"
#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "ts_link";

#define SLIP_END 0xC0u
#define SLIP_ESC 0xDBu
#define SLIP_ESC_END 0xDCu
#define SLIP_ESC_ESC 0xDDu
#define TS_MAX_PAYLOAD 96u
#define TS_RX_BUF (TS_MAX_PAYLOAD + 5u)          /* id+seq+len+payload+crc16 */

#define TS_UART_RXBUF 2048
#define TS_UART_TXBUF 1024                        /* non-zero => uart_write_bytes non-blocking */
#define TS_UART_QUEUE 20
#define TS_RX_FULL_THRESH 60
#define TS_TICK_US (20 * 1000)                    /* reliable retransmit tick */

static int s_uart = -1;
static QueueHandle_t s_uart_q;

/* telemetry snapshot state */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static tsl_status_t s_status;
static bool     s_have_status = false;
static int64_t  s_last_rx_us  = 0;
static uint32_t s_rx_frames   = 0;
static uint32_t s_crc_errors  = 0;
static uint16_t s_peer_ver    = 0;
static uint32_t s_hb_seq      = 0;

/* reliable-layer state (guarded by s_rmux) */
static portMUX_TYPE s_rmux = portMUX_INITIALIZER_UNLOCKED;
#define TS_RQ 4u
#define TS_CMD_MAX 16u
typedef struct { uint8_t id, len, data[TS_CMD_MAX]; } ts_rmsg_t;
static ts_rmsg_t s_rq[TS_RQ];
static uint8_t s_rq_head = 0, s_rq_tail = 0;
static uint8_t s_tx_seq = 0;
static ts_rmsg_t s_out;
static uint8_t  s_out_seq = 0, s_out_retries = 0;
static int64_t  s_out_time_us = 0;
static uint8_t  s_peer_seq = 0;
static uint32_t s_rel_errs = 0;

static uint16_t ts_crc16(const uint8_t *d, uint32_t n)
{
  uint16_t c = 0xFFFFu;
  for (uint32_t i = 0; i < n; i++) {
    c ^= (uint16_t)((uint16_t)d[i] << 8);
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
  }
  return c;
}

/* ---- TX (uart_write_bytes is thread-safe in ESP-IDF) ---------------------- */
static void slip_enc(uint8_t *out, uint16_t *n, uint8_t b)
{
  if (b == SLIP_END)      { out[(*n)++] = SLIP_ESC; out[(*n)++] = SLIP_ESC_END; }
  else if (b == SLIP_ESC) { out[(*n)++] = SLIP_ESC; out[(*n)++] = SLIP_ESC_ESC; }
  else                    { out[(*n)++] = b; }
}

static void ts_frame_send(uint8_t id, uint8_t seq, const uint8_t *p, uint8_t len)
{
  if (s_uart < 0 || len > TS_MAX_PAYLOAD) return;
  uint8_t frame[3u + TS_MAX_PAYLOAD];
  frame[0] = id; frame[1] = seq; frame[2] = len;
  for (uint8_t i = 0; i < len; i++) frame[3u + i] = p[i];
  uint16_t crc = ts_crc16(frame, (uint32_t)(3u + len));

  uint8_t out[2u * (TS_MAX_PAYLOAD + 7u)]; uint16_t n = 0;
  out[n++] = SLIP_END;
  for (uint8_t i = 0; i < (uint8_t)(3u + len); i++) slip_enc(out, &n, frame[i]);
  slip_enc(out, &n, (uint8_t)(crc & 0xFFu));
  slip_enc(out, &n, (uint8_t)(crc >> 8));
  out[n++] = SLIP_END;
  uart_write_bytes(s_uart, (const char *)out, n);
}
static inline void ts_send(uint8_t id, const void *p, uint8_t len) { ts_frame_send(id, 0u, (const uint8_t *)p, len); }

static bool ts_send_reliable(uint8_t id, const void *p, uint8_t len)
{
  if (len > TS_CMD_MAX) return false;
  bool ok;
  portENTER_CRITICAL(&s_rmux);
  uint8_t next = (uint8_t)((s_rq_head + 1u) % TS_RQ);
  ok = (next != s_rq_tail);
  if (ok) { s_rq[s_rq_head].id = id; s_rq[s_rq_head].len = len; memcpy(s_rq[s_rq_head].data, p, len); s_rq_head = next; }
  portEXIT_CRITICAL(&s_rmux);
  return ok;
}

/* periodic (esp_timer): retransmit outstanding / launch next queued reliable */
static void ts_reliable_tick(void *arg)
{
  (void)arg;
  uint8_t id = 0, seq = 0, len = 0, data[TS_CMD_MAX]; bool do_send = false;
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&s_rmux);
  if (s_out_seq != 0) {
    if ((now - s_out_time_us) >= (int64_t)TSL_RELIABLE_TIMEOUT_MS * 1000) {
      if (s_out_retries >= TSL_RELIABLE_RETRIES) { s_rel_errs++; s_out_seq = 0; }
      else { s_out_retries++; s_out_time_us = now; id = s_out.id; seq = s_out_seq; len = s_out.len; memcpy(data, s_out.data, len); do_send = true; }
    }
  } else if (s_rq_tail != s_rq_head) {
    s_out = s_rq[s_rq_tail]; s_rq_tail = (uint8_t)((s_rq_tail + 1u) % TS_RQ);
    s_tx_seq++; if (s_tx_seq == 0) s_tx_seq = 1;
    s_out_seq = s_tx_seq; s_out_retries = 0; s_out_time_us = now;
    id = s_out.id; seq = s_out_seq; len = s_out.len; memcpy(data, s_out.data, len); do_send = true;
  }
  portEXIT_CRITICAL(&s_rmux);
  if (do_send) ts_frame_send(id, seq, data, len);          /* TX outside the critical section */
}

/* ---- public command senders ----------------------------------------------- */
void ts_link_send_hello(void)   { tsl_hello_t h = { .protocol_version = TSL_PROTOCOL_VERSION, .fw_version = 0 }; ts_send_reliable(TSL_MSG_HELLO, &h, sizeof(h)); }
void ts_link_set_mode(uint8_t mode) { tsl_set_mode_t m = { .mode = mode };   ts_send_reliable(TSL_MSG_SET_MODE, &m, sizeof(m)); }
void ts_link_enable(bool en)        { tsl_enable_t e = { .enable = en?1:0 };  ts_send_reliable(TSL_MSG_ENABLE, &e, sizeof(e)); }
void ts_link_estop(void)            { ts_send_reliable(TSL_MSG_ESTOP, NULL, 0); }
void ts_link_ack_fault(void)        { ts_send_reliable(TSL_MSG_ACK_FAULT, NULL, 0); }
void ts_link_calibrate(uint8_t action) { tsl_calibrate_t c = { .action = action }; ts_send_reliable(TSL_MSG_CALIBRATE, &c, sizeof(c)); }

void ts_link_send_heartbeat(int16_t torque_mA, int16_t speed_rpm)   /* stream */
{ tsl_heartbeat_t hb = { .seq = ++s_hb_seq, .torque_mA = torque_mA, .speed_rpm = speed_rpm }; ts_send(TSL_MSG_HEARTBEAT, &hb, sizeof(hb)); }
void ts_link_set_setpoint(int16_t torque_mA, int16_t speed_rpm)     /* stream */
{ tsl_setpoint_t sp = { .torque_mA = torque_mA, .speed_rpm = speed_rpm }; ts_send(TSL_MSG_SETPOINT, &sp, sizeof(sp)); }

/* ---- RX ------------------------------------------------------------------- */
static void ts_on_packet(const uint8_t *buf, uint16_t n)
{
  if (n < 5u) return;
  uint8_t id = buf[0], seq = buf[1], plen = buf[2];
  if ((uint16_t)(3u + plen + 2u) != n) return;
  uint16_t rx_crc = (uint16_t)buf[3u + plen] | (uint16_t)((uint16_t)buf[4u + plen] << 8);
  if (ts_crc16(buf, (uint32_t)(3u + plen)) != rx_crc) { s_crc_errors++; return; }
  const uint8_t *pl = &buf[3];

  portENTER_CRITICAL(&s_mux); s_rx_frames++; s_last_rx_us = esp_timer_get_time(); portEXIT_CRITICAL(&s_mux);

  if (id == TSL_MSG_ACK) {
    if (plen >= 1) { portENTER_CRITICAL(&s_rmux); if (s_out_seq != 0 && pl[0] == s_out_seq) s_out_seq = 0; portEXIT_CRITICAL(&s_rmux); }
    return;
  }

  bool dup = false;
  if (seq != 0u) {
    tsl_ack_t a = { .acked_seq = seq };
    ts_frame_send(TSL_MSG_ACK, 0u, (const uint8_t *)&a, sizeof(a));   /* ACK now (non-blocking TX) */
    portENTER_CRITICAL(&s_rmux); if (seq == s_peer_seq) dup = true; else s_peer_seq = seq; portEXIT_CRITICAL(&s_rmux);
  }
  if (dup) return;

  portENTER_CRITICAL(&s_mux);
  switch (id) {
    case TSL_MSG_STATUS: if (plen >= sizeof(tsl_status_t)) { memcpy(&s_status, pl, sizeof(s_status)); s_have_status = true; } break;
    case TSL_MSG_HELLO:  if (plen >= sizeof(tsl_hello_t))  { tsl_hello_t h; memcpy(&h, pl, sizeof(h)); s_peer_ver = h.protocol_version; } break;
    case TSL_MSG_FAULT_EVENT: /* TODO: surface fault events to the app */ break;
    default: break;
  }
  portEXIT_CRITICAL(&s_mux);
}

static void ts_feed(uint8_t b)
{
  static uint8_t buf[TS_RX_BUF]; static uint16_t len = 0; static uint8_t esc = 0;
  if (b == SLIP_END) { if (len) ts_on_packet(buf, len); len = 0; esc = 0; return; }
  if (esc) { esc = 0; if (b == SLIP_ESC_END) b = SLIP_END; else if (b == SLIP_ESC_ESC) b = SLIP_ESC; }
  else if (b == SLIP_ESC) { esc = 1; return; }
  if (len < TS_RX_BUF) buf[len++] = b; else len = 0;
}

static void ts_rx_task(void *arg)
{
  (void)arg;
  uart_event_t ev;
  uint8_t *rd = malloc(TS_UART_RXBUF);
  for (;;) {
    if (xQueueReceive(s_uart_q, &ev, portMAX_DELAY) != pdTRUE) continue;
    switch (ev.type) {
      case UART_DATA: {
        int n = uart_read_bytes(s_uart, rd, ev.size, portMAX_DELAY);
        for (int i = 0; i < n; i++) ts_feed(rd[i]);
        break;
      }
      case UART_FIFO_OVF:
      case UART_BUFFER_FULL:
        ESP_LOGW(TAG, "rx overflow -> flush+resync");
        uart_flush_input(s_uart); xQueueReset(s_uart_q);
        break;
      default: break;
    }
  }
}

/* ---- init + getters ------------------------------------------------------- */
esp_err_t ts_link_init(int uart_num, int tx_gpio, int rx_gpio, int baud)
{
  s_uart = uart_num;
  uart_config_t cfg = {
    .baud_rate = baud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_param_config(uart_num, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(uart_num, TS_UART_RXBUF, TS_UART_TXBUF, TS_UART_QUEUE, &s_uart_q, 0));
  uart_set_rx_full_threshold(uart_num, TS_RX_FULL_THRESH);
  xTaskCreate(ts_rx_task, "ts_rx", 4096, NULL, 12, NULL);

  const esp_timer_create_args_t targs = { .callback = ts_reliable_tick, .name = "ts_tick" };
  esp_timer_handle_t th;
  ESP_ERROR_CHECK(esp_timer_create(&targs, &th));
  ESP_ERROR_CHECK(esp_timer_start_periodic(th, TS_TICK_US));

  ESP_LOGI(TAG, "ts_link up: UART%d tx=%d rx=%d @ %d", uart_num, tx_gpio, rx_gpio, baud);
  return ESP_OK;
}

bool ts_link_get_status(tsl_status_t *out)
{
  bool ok; portENTER_CRITICAL(&s_mux); ok = s_have_status; if (ok && out) *out = s_status; portEXIT_CRITICAL(&s_mux); return ok;
}
uint32_t ts_link_age_ms(void)
{
  portENTER_CRITICAL(&s_mux); int64_t last = s_last_rx_us; portEXIT_CRITICAL(&s_mux);
  if (last == 0) return 0xFFFFFFFFu;
  return (uint32_t)((esp_timer_get_time() - last) / 1000);
}
bool     ts_link_up(void)           { return ts_link_age_ms() <= TSL_LINK_TIMEOUT_MS; }
uint32_t ts_link_rx_frames(void)    { return s_rx_frames; }
uint32_t ts_link_crc_errors(void)   { return s_crc_errors; }
uint16_t ts_link_peer_version(void) { return s_peer_ver; }
uint32_t ts_link_reliable_errors(void) { return s_rel_errs; }
