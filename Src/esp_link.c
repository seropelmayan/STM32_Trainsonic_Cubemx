/**
 ******************************************************************************
 * @file    esp_link.c
 * @brief   STM32 side of the ESP32 UART link -- SLIP framing + CRC16, RX parser,
 *          command dispatch, telemetry TX, a thin sequence/ACK/retransmit layer
 *          for reliable messages, and the link watchdog. See esp_link.h.
 *
 *          Frame: [id][seq][len][payload 0..len][crc16-LE], CRC over id..payload.
 *          seq==0 => fire-and-forget (stream). seq!=0 => reliable: the receiver
 *          replies TSL_MSG_ACK{seq}; the sender retransmits until ACKed.
 *
 *          Threading: esp_link_feed() runs in the USART RX ISR and does NO TX --
 *          it only parses, clears an outstanding ACK, flags a pending ACK, and
 *          dispatches (setting volatile flags). All TX (ACK, retransmit, stream)
 *          happens in esp_link_reliable_tick()/esp_link_send*(), which the app
 *          calls from ONE thread context -> the TX byte-writer is a lock-free
 *          single-producer ring (see main.c). This keeps a slow/stalled TX from
 *          ever wedging the receiver.
 ******************************************************************************
 */
#include "esp_link.h"
#include <string.h>

/* Existing motor setter flags (mc_tasks_foc.c), shared with the USB-CDC console. */
extern volatile int32_t g_torque_set_ma;   extern volatile uint8_t g_torque_set_req;
extern volatile int32_t g_speed_set_rpm;   extern volatile uint8_t g_speed_set_req;
extern volatile uint8_t g_restart_req;
extern volatile uint8_t g_cogg_cal_req;     /* anti-cogging: 'y' start cal   */
extern volatile uint8_t g_cogg_cal_abort;   /* 'Y' finish/abort cal          */
extern volatile uint8_t g_cogg_save_req;    /* 'X' save map to flash (reboots) */
extern volatile uint8_t g_cogg_erase_req;   /* 'n' erase saved map           */
extern volatile float   g_spdcap_rpm;       /* torque-mode velocity cap (rpm), same as CDC 'V' */
extern volatile float   g_spdcap_hard_rpm;  /* firmware ceiling: ESP requests are clamped to this */
extern volatile int32_t g_hb_brake_ma;      /* speed-window brake-side limit from the heartbeat (mA); <0 = console default */
extern volatile uint8_t g_ctrl_scheme;      /* 1 = speed window, 0 = legacy torque governor; selected by the heartbeat below */
extern volatile uint8_t g_ctrl_scheme_lock; /* 1 = CDC '#' pinned it: leave the scheme alone */

/* Clamp an ESP-requested speed cap to the firmware hard ceiling. 0 ("unlimited")
   also becomes the hard ceiling -- the STM32 has the last word on top speed. */
static float esp_cap_clamp(float rpm)
{
  if ((rpm <= 0.0f) || (rpm > g_spdcap_hard_rpm)) { rpm = g_spdcap_hard_rpm; }
  return rpm;
}

/* Flags owned here, serviced by the app in STEP 3. */
volatile uint8_t g_esp_estop_req  = 0U;
volatile uint8_t g_esp_enable_req = 0U;
volatile uint8_t g_esp_enable_val = 0U;
volatile uint8_t g_esp_mode_req   = 0U;
volatile uint8_t g_esp_mode_val   = 0U;

/* ---- SLIP (RFC 1055) ------------------------------------------------------ */
#define SLIP_END      0xC0u
#define SLIP_ESC      0xDBu
#define SLIP_ESC_END  0xDCu
#define SLIP_ESC_ESC  0xDDu

#define ESP_MAX_PAYLOAD 96u
#define ESP_RX_BUF      (ESP_MAX_PAYLOAD + 5u)   /* id+seq+len+payload+crc16 */

static esp_link_write_fn s_write = 0;

static uint8_t  s_rx[ESP_RX_BUF];
static uint16_t s_rx_len = 0;
static uint8_t  s_rx_esc = 0;

static volatile uint8_t  s_got_frame = 0;        /* watchdog: valid frame seen */
static uint32_t s_last_ok_ms = 0;
static volatile uint32_t s_rx_frames = 0;
static volatile uint32_t s_crc_errs  = 0;

/* ---- reliable (thin ARQ) -------------------------------------------------- */
#define ESP_RELIABLE_Q  4u
#define ESP_CMD_MAXLEN  16u
typedef struct { uint8_t id; uint8_t len; uint8_t data[ESP_CMD_MAXLEN]; } esp_rmsg_t;

static esp_rmsg_t s_rq[ESP_RELIABLE_Q];          /* pending reliable send queue */
static uint8_t s_rq_head = 0, s_rq_tail = 0;
static uint8_t s_tx_seq  = 0;                    /* last seq we assigned */
static esp_rmsg_t s_out_msg;                     /* current outstanding (for retx) */
static volatile uint8_t s_out_seq = 0;           /* outstanding seq (0=none); cleared by ACK in ISR */
static uint8_t  s_out_retries = 0;
static uint32_t s_out_time    = 0;
static volatile uint8_t  s_ack_pending = 0;      /* seq to ACK (set in ISR, sent in tick) */
static uint8_t  s_peer_seq  = 0;                 /* last processed reliable seq (dedup) */
static volatile uint32_t s_rel_errs = 0;         /* reliable give-ups */
static volatile uint8_t  s_hello_pending = 0;    /* RX ISR asks for a HELLO reply; tick sends it */

static uint16_t esp_crc16(const uint8_t *d, uint32_t n)
{
  uint16_t c = 0xFFFFu;
  for (uint32_t i = 0u; i < n; i++)
  {
    c ^= (uint16_t)((uint16_t)d[i] << 8);
    for (uint8_t b = 0u; b < 8u; b++)
    {
      c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
    }
  }
  return c;
}

/* ---- TX ------------------------------------------------------------------- */
static void slip_put(uint8_t b)
{
  if (s_write == 0) { return; }
  if (b == SLIP_END)      { s_write(SLIP_ESC); s_write(SLIP_ESC_END); }
  else if (b == SLIP_ESC) { s_write(SLIP_ESC); s_write(SLIP_ESC_ESC); }
  else                    { s_write(b); }
}

static void esp_frame_send(uint8_t id, uint8_t seq, const uint8_t *p, uint8_t len)
{
  if ((s_write == 0) || (len > ESP_MAX_PAYLOAD)) { return; }
  uint8_t frame[3u + ESP_MAX_PAYLOAD];
  frame[0] = id; frame[1] = seq; frame[2] = len;
  for (uint8_t i = 0u; i < len; i++) { frame[3u + i] = p[i]; }
  uint16_t crc = esp_crc16(frame, (uint32_t)(3u + len));

  s_write(SLIP_END);
  for (uint8_t i = 0u; i < (uint8_t)(3u + len); i++) { slip_put(frame[i]); }
  slip_put((uint8_t)(crc & 0xFFu));
  slip_put((uint8_t)(crc >> 8));
  s_write(SLIP_END);
}

void esp_link_send(uint8_t msg_id, const void *payload, uint8_t len)
{
  esp_frame_send(msg_id, 0u, (const uint8_t *)payload, len);   /* stream: seq 0 */
}

void esp_link_send_status(const tsl_status_t *st)
{
  esp_link_send(TSL_MSG_STATUS, st, (uint8_t)sizeof(*st));
}

/* Queue a reliable message (ACKed + retransmitted). Returns 1 if queued, 0 if full. */
uint8_t esp_link_send_reliable(uint8_t msg_id, const void *payload, uint8_t len)
{
  if (len > ESP_CMD_MAXLEN) { return 0u; }
  uint8_t next = (uint8_t)((s_rq_head + 1u) % ESP_RELIABLE_Q);
  if (next == s_rq_tail) { return 0u; }                        /* queue full */
  s_rq[s_rq_head].id  = msg_id;
  s_rq[s_rq_head].len = len;
  for (uint8_t i = 0u; i < len; i++) { s_rq[s_rq_head].data[i] = ((const uint8_t *)payload)[i]; }
  s_rq_head = next;
  return 1u;
}

/* Thread-context service: send any pending ACK, and drive the reliable queue
   (retransmit on timeout / start the next message). Call from ONE context (app). */
void esp_link_reliable_tick(uint32_t now_ms)
{
  /* 0) queue a HELLO reply if the peer said HELLO (all reliable enqueues happen here,
        in one thread context, so the reliable queue has a single writer) */
  if (s_hello_pending != 0u)
  {
    s_hello_pending = 0u;
    tsl_hello_t h; h.protocol_version = TSL_PROTOCOL_VERSION; h.fw_version = 0u;
    (void)esp_link_send_reliable(TSL_MSG_HELLO, &h, (uint8_t)sizeof(h));
  }

  /* 1) deferred ACK for a reliable frame we received (RX ISR flagged it) */
  uint8_t ack = s_ack_pending;
  if (ack != 0u)
  {
    s_ack_pending = 0u;
    tsl_ack_t a; a.acked_seq = ack;
    esp_frame_send(TSL_MSG_ACK, 0u, (const uint8_t *)&a, (uint8_t)sizeof(a));
  }

  /* 2) outstanding reliable frame: retransmit on timeout, or give up */
  if (s_out_seq != 0u)
  {
    if ((now_ms - s_out_time) >= (uint32_t)TSL_RELIABLE_TIMEOUT_MS)
    {
      if (s_out_retries >= (uint8_t)TSL_RELIABLE_RETRIES)
      {
        s_rel_errs++;
        s_out_seq = 0u;                                        /* give up */
      }
      else
      {
        s_out_retries++;
        s_out_time = now_ms;
        esp_frame_send(s_out_msg.id, s_out_seq, s_out_msg.data, s_out_msg.len);
      }
    }
    return;                                                    /* stop-and-wait */
  }

  /* 3) no outstanding: launch the next queued reliable message */
  if (s_rq_tail != s_rq_head)
  {
    s_out_msg = s_rq[s_rq_tail];
    s_rq_tail = (uint8_t)((s_rq_tail + 1u) % ESP_RELIABLE_Q);
    s_tx_seq++; if (s_tx_seq == 0u) { s_tx_seq = 1u; }         /* seq 1..255 */
    s_out_seq     = s_tx_seq;
    s_out_retries = 0u;
    s_out_time    = now_ms;
    esp_frame_send(s_out_msg.id, s_out_seq, s_out_msg.data, s_out_msg.len);
  }
}

/* ---- dispatch (RX ISR context; flags only, no TX) ------------------------- */
static void esp_dispatch(uint8_t id, const uint8_t *pl, uint8_t len)
{
  switch (id)
  {
    case TSL_MSG_HELLO:
      s_hello_pending = 1u;   /* reply is queued from reliable_tick (thread ctx), not this ISR */
      break;
    case TSL_MSG_HEARTBEAT:
      if (len >= 8u)                                   /* v3 (8 B) or v4 (10 B, + brake_mA) */
      {
        uint8_t v4 = (len >= (uint8_t)sizeof(tsl_heartbeat_t)) ? 1U : 0U;
        tsl_heartbeat_t hb;
        memset(&hb, 0, sizeof(hb));
        memcpy(&hb, pl, v4 ? sizeof(hb) : 8u);
        /* THE SENDER PICKS THE CONTROL SCHEME. A v4 heartbeat carries brake_mA, so its
           author knows torque_mA is a LIMIT and speed_rpm a REFERENCE -> speed window.
           A v3 heartbeat means an ESP that still speaks plain torque -> legacy governor,
           i.e. flashing this image in front of an unmodified ESP changes nothing.
           CDC '#' pins the scheme for bench A/B and switches this line off. */
        if (g_ctrl_scheme_lock == 0U) { g_ctrl_scheme = v4; }
        g_hb_brake_ma   = v4 ? (int32_t)hb.brake_mA : -1;     /* -1 = console default */
        g_torque_set_ma = (int32_t)hb.torque_mA; g_torque_set_req = 1U;
        g_spdcap_rpm    = esp_cap_clamp((float)hb.speed_rpm); /* speed reference / cap, hard-limited by STM32 */
      }
      break;
    case TSL_MSG_SETPOINT:
      if (len >= (uint8_t)sizeof(tsl_setpoint_t))
      {
        tsl_setpoint_t sp; memcpy(&sp, pl, sizeof(sp));
        if (g_ctrl_scheme_lock == 0U) { g_ctrl_scheme = 0U; } /* plain torque setpoint: legacy scheme */
        g_hb_brake_ma   = -1;                                 /* no brake field: console default */
        g_torque_set_ma = (int32_t)sp.torque_mA; g_torque_set_req = 1U;
        g_spdcap_rpm    = esp_cap_clamp((float)sp.speed_rpm); /* velocity cap, hard-limited by STM32 */
      }
      break;
    case TSL_MSG_SET_MODE:
      if (len >= (uint8_t)sizeof(tsl_set_mode_t)) { g_esp_mode_val = pl[0]; g_esp_mode_req = 1U; }
      break;
    case TSL_MSG_ENABLE:
      if (len >= (uint8_t)sizeof(tsl_enable_t)) { g_esp_enable_val = pl[0]; g_esp_enable_req = 1U; }
      break;
    case TSL_MSG_ESTOP:
      g_esp_estop_req = 1U;
      g_torque_set_ma = 0; g_torque_set_req = 1U;
      break;
    case TSL_MSG_ACK_FAULT:
      g_restart_req = 1U;
      break;
    case TSL_MSG_CALIBRATE:
      if (len >= (uint8_t)sizeof(tsl_calibrate_t))
      {
        switch (pl[0])                          /* mirrors CDC y / Y / X / n */
        {
          case TSL_CAL_START: g_cogg_cal_req   = 1U; break;  /* needs motor in RUN */
          case TSL_CAL_ABORT: g_cogg_cal_abort = 1U; break;
          case TSL_CAL_SAVE:  g_cogg_save_req  = 1U; break;  /* saves + reboots */
          case TSL_CAL_ERASE: g_cogg_erase_req = 1U; break;
          default: break;
        }
      }
      break;
    default:
      break;
  }
}

/* ---- RX (ISR context) ----------------------------------------------------- */
static void esp_on_packet(const uint8_t *buf, uint16_t n)
{
  if (n < 5u) { return; }                                     /* id+seq+len+crc2 */
  uint8_t id   = buf[0];
  uint8_t seq  = buf[1];
  uint8_t plen = buf[2];
  if ((uint16_t)(3u + plen + 2u) != n) { return; }
  uint16_t rx_crc = (uint16_t)buf[3u + plen] | (uint16_t)((uint16_t)buf[4u + plen] << 8);
  if (esp_crc16(buf, (uint32_t)(3u + plen)) != rx_crc) { s_crc_errs++; return; }

  s_rx_frames++;
  s_got_frame = 1u;
  const uint8_t *pl = &buf[3];

  if (id == TSL_MSG_ACK)
  {
    if ((plen >= 1u) && (s_out_seq != 0u) && (pl[0] == s_out_seq)) { s_out_seq = 0u; }
    return;                                                   /* ACKs aren't ACKed/dispatched */
  }

  if (seq != 0u)                                              /* reliable frame */
  {
    s_ack_pending = seq;                                      /* defer the ACK to the tick */
    if (seq == s_peer_seq) { return; }                        /* duplicate -> re-ACK, don't reprocess */
    s_peer_seq = seq;
  }
  esp_dispatch(id, pl, plen);
}

void esp_link_feed(uint8_t b)
{
  if (b == SLIP_END)
  {
    if (s_rx_len > 0u) { esp_on_packet(s_rx, s_rx_len); }
    s_rx_len = 0u; s_rx_esc = 0u;
    return;
  }
  if (s_rx_esc)
  {
    s_rx_esc = 0u;
    if (b == SLIP_ESC_END) { b = SLIP_END; }
    else if (b == SLIP_ESC_ESC) { b = SLIP_ESC; }
  }
  else if (b == SLIP_ESC)
  {
    s_rx_esc = 1u;
    return;
  }
  if (s_rx_len < (uint16_t)ESP_RX_BUF) { s_rx[s_rx_len++] = b; }
  else { s_rx_len = 0u; }
}

/* ---- watchdog / init / stats ---------------------------------------------- */
uint8_t esp_link_ok(uint32_t now_ms)
{
  if (s_got_frame) { s_got_frame = 0u; s_last_ok_ms = now_ms; }
  return (uint8_t)((now_ms - s_last_ok_ms) <= (uint32_t)TSL_LINK_TIMEOUT_MS);
}

void esp_link_init(esp_link_write_fn writer)
{
  s_write = writer;
  s_rx_len = 0u; s_rx_esc = 0u; s_got_frame = 0u; s_last_ok_ms = 0u;
  s_rx_frames = 0u; s_crc_errs = 0u;
  s_rq_head = 0u; s_rq_tail = 0u; s_tx_seq = 0u;
  s_out_seq = 0u; s_out_retries = 0u; s_ack_pending = 0u; s_peer_seq = 0u; s_rel_errs = 0u;
  s_hello_pending = 0u;
}

uint32_t esp_link_rx_frames(void)       { return s_rx_frames; }
uint32_t esp_link_crc_errors(void)      { return s_crc_errs; }
uint32_t esp_link_reliable_errors(void) { return s_rel_errs; }
