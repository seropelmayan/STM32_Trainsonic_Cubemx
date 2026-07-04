/**
 ******************************************************************************
 * @file    esp_link.h
 * @brief   STM32 side of the ESP32 <-> STM32 UART link (see protocol/trainsonic_link.h).
 *
 *          This layer is UART-agnostic: it does framing (SLIP) + CRC16, parses
 *          incoming frames, dispatches commands onto the existing motor setter
 *          flags, sends telemetry, and runs a link watchdog. The physical USART
 *          is bound by the app: feed RX bytes with esp_link_feed() and provide a
 *          byte writer to esp_link_init().
 *
 *          STEP 1 (this file): framing + dispatch + watchdog, self-contained.
 *          STEP 2: bind a real USART (DMA RX + idle IRQ) -- needs the pin choice.
 *          STEP 3: reliable ACK/retransmit for R-messages + mode/enable/estop
 *                  servicing in the app thread.
 ******************************************************************************
 */
#ifndef ESP_LINK_H
#define ESP_LINK_H

#include <stdint.h>
#include "../protocol/trainsonic_link.h"  /* shared comm table (submodule master lives in protocol/) */

#ifdef __cplusplus
extern "C" {
#endif

/* One-byte writer into the ESP32 USART TX path (bound by the app). */
typedef void (*esp_link_write_fn)(uint8_t byte);

/* Initialise the link and bind the TX byte writer. */
void esp_link_init(esp_link_write_fn writer);

/* Feed one received byte. Call from the USART RX ISR / DMA-idle handler.
   Cheap and self-contained; parses+dispatches a frame on the closing delimiter. */
void esp_link_feed(uint8_t byte);

/* Send a STREAM (fire-and-forget) message: framed + CRC, seq=0, never ACKed. */
void esp_link_send(uint8_t msg_id, const void *payload, uint8_t len);

/* Convenience: send the fast periodic telemetry status (stream). */
void esp_link_send_status(const tsl_status_t *st);

/* Queue a RELIABLE message (sequenced, ACKed, retransmitted). Returns 1 if
   queued, 0 if the queue is full. Payload <= 16 bytes. */
uint8_t esp_link_send_reliable(uint8_t msg_id, const void *payload, uint8_t len);

/* Drive the reliable layer: sends any pending ACK and retransmits/advances the
   reliable queue. Call periodically from ONE thread context (e.g. the app loop);
   this is also the only place stream/reliable TX originates, so the TX writer is
   a lock-free single-producer path. */
void esp_link_reliable_tick(uint32_t now_ms);

uint32_t esp_link_reliable_errors(void);   /* reliable give-ups (retries exhausted) */

/* Link watchdog. Call periodically with the current uptime (HAL_GetTick()).
   Returns 1 while a valid ESP frame was seen within TSL_LINK_TIMEOUT_MS, else 0
   (caller MUST enter safe state: ramp torque to 0 / stop). */
uint8_t esp_link_ok(uint32_t now_ms);

/* Count of valid frames received (telemetry / bring-up sanity). */
uint32_t esp_link_rx_frames(void);
uint32_t esp_link_crc_errors(void);

/* --- Request flags OWNED by esp_link, SET by incoming commands, SERVICED by the
   app thread in STEP 3 (mirrors the existing g_*_req pattern). torque/speed reuse
   the existing g_torque_set_* / g_speed_set_* globals in mc_tasks_foc.c. --- */
extern volatile uint8_t g_esp_estop_req;   /* TSL_MSG_ESTOP    -> MC_StopMotor1 + 0 A      */
extern volatile uint8_t g_esp_enable_req;  /* TSL_MSG_ENABLE   -> enable/disable the drive */
extern volatile uint8_t g_esp_enable_val;  /* 0/1                                          */
extern volatile uint8_t g_esp_mode_req;    /* TSL_MSG_SET_MODE -> set control mode         */
extern volatile uint8_t g_esp_mode_val;    /* tsl_mode_t                                   */

#ifdef __cplusplus
}
#endif

#endif /* ESP_LINK_H */
