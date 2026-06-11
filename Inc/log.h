/**
  ******************************************************************************
  * @file    log.h
  * @brief   Non-blocking text logger over the USB CDC virtual COM port.
  *
  * LOG_Printf() formats into a RAM ring buffer and returns immediately.
  * LOG_Process(), called from the main loop, drains the buffer to the host
  * only when the CDC endpoint is idle and the device is enumerated. Nothing
  * here ever blocks, so it is safe to call alongside the FOC interrupts.
  *
  * Call from main (thread) context only -- NOT from an ISR or the control loop.
  ******************************************************************************
  */

#ifndef LOG_H
#define LOG_H

#include <stdint.h>

/** @brief Reset the ring buffer. Call once at startup. */
void LOG_Init(void);

/** @brief printf-style log into the ring buffer (non-blocking, drops on full). */
void LOG_Printf(const char *fmt, ...);

/** @brief Drain pending bytes to the USB CDC port. Call repeatedly from main(). */
void LOG_Process(void);

#endif /* LOG_H */
