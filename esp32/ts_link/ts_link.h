/**
 ******************************************************************************
 * @file    ts_link.h
 * @brief   ESP32 (ESP-IDF) side of the ESP32 <-> STM32 UART link.
 *
 *          Mirrors the STM32 esp_link.c wire format EXACTLY (SLIP framing +
 *          CRC16-CCITT, frame = [id][len][payload][crc16-LE]) so the two talk
 *          without either side changing. Shares protocol/trainsonic_link.h.
 *
 *          Usage:
 *            ts_link_init(UART_NUM_1, TX_GPIO, RX_GPIO, 115200);
 *            ts_link_send_hello();                       // handshake
 *            // 50 Hz control loop:
 *            ts_link_send_heartbeat(torque_mA, speed_rpm);
 *            // read latest telemetry any time:
 *            tsl_status_t s; if (ts_link_get_status(&s)) { ... }
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "trainsonic_link.h"        /* the shared comm table (see README) */

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the link on a UART (NOT UART_NUM_0 = console). Starts the RX task. */
esp_err_t ts_link_init(int uart_num, int tx_gpio, int rx_gpio, int baud);

/* --- commands: ESP32 -> STM32 ------------------------------------------------ */
void ts_link_send_hello(void);
void ts_link_send_heartbeat(int16_t torque_mA, int16_t speed_rpm); /* call ~50 Hz keep-alive */
void ts_link_set_setpoint(int16_t torque_mA, int16_t speed_rpm);   /* fire-and-forget stream  */
void ts_link_set_mode(uint8_t mode);        /* tsl_mode_t */
void ts_link_enable(bool en);
void ts_link_estop(void);
void ts_link_ack_fault(void);
void ts_link_calibrate(uint8_t action);     /* tsl_cal_action_t: START/ABORT/SAVE/ERASE */

/* --- telemetry: STM32 -> ESP32 (thread-safe snapshots) ---------------------- */
bool     ts_link_get_status(tsl_status_t *out);  /* latest STATUS; false if none yet   */
bool     ts_link_up(void);                       /* true if a valid frame within timeout */
uint32_t ts_link_age_ms(void);                   /* ms since last valid frame (link health) */
uint32_t ts_link_rx_frames(void);
uint32_t ts_link_crc_errors(void);
uint32_t ts_link_reliable_errors(void);          /* reliable give-ups (retries exhausted) */
uint16_t ts_link_peer_version(void);             /* protocol version from STM32 HELLO (0=unknown) */

/* Command reliability is automatic (queued + ACKed + retransmitted by an internal
   20 ms tick). The set_mode/enable/estop/ack_fault/hello calls above go reliable;
   heartbeat/setpoint go fire-and-forget. */

#ifdef __cplusplus
}
#endif
