/**
  ******************************************************************************
  * @file    as5047.h
  * @brief   Boot-time configuration driver for the AS5047P magnetic encoder.
  *
  * The AS5047P provides the A/B quadrature (ABI) signals consumed by TIM3 for
  * MCSDK FOC position/speed feedback. This driver talks to the encoder's SPI
  * configuration port (SPI1, software CS on PA4) only at boot, to force the ABI
  * output into the resolution / direction the firmware expects. FOC itself
  * never uses this SPI link.
  *
  * Settings registers are volatile (reloaded from OTP at each power-up), so this
  * runs on every boot and never burns OTP.
  ******************************************************************************
  */

#ifndef AS5047_H
#define AS5047_H

#include "main.h"   /* HAL types + generated pin macros */

/* ------------------------------------------------------------------------- */
/* User configuration                                                        */
/* ------------------------------------------------------------------------- */

/* Software chip-select. If you relabel PA4 to "ENC_CS" in CubeMX, switch
   these to ENC_CS_GPIO_Port / ENC_CS_Pin. */
#define AS5047_CS_GPIO_Port   GPIOA
#define AS5047_CS_Pin         GPIO_PIN_4

/* SPI transfer timeout (ms). */
#define AS5047_SPI_TIMEOUT    10U

/* ABI count direction. The AS5047 DIR bit must combine with the MCSDK encoder
   "inverted" setting to give a positive speed sign for positive torque. If the
   SDK reports inverted speed or encoder alignment fails during bring-up, flip
   this (0 <-> 1) and rebuild. */
#define AS5047_INVERT_DIR     0

/* ------------------------------------------------------------------------- */
/* Status                                                                    */
/* ------------------------------------------------------------------------- */
typedef enum
{
  AS5047_OK = 0,
  AS5047_ERR_COMM,      /* parity/framing error on the SPI link            */
  AS5047_ERR_MAGNET,    /* magnet out of range or CORDIC overflow (DIAAGC) */
  AS5047_ERR_VERIFY     /* read-back of SETTINGS did not match             */
} AS5047_Status;

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

/**
 * @brief  Configure the AS5047P ABI output at boot (resolution, mode,
 *         direction) and verify it. Call once after MX_SPI1_Init(), before
 *         the motor is started.
 * @param  hspi  initialised SPI handle wired to the encoder (e.g. &hspi1)
 * @retval AS5047_Status
 */
AS5047_Status AS5047_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief  Read the compensated absolute angle (ANGLECOM), 0..16383 (14-bit).
 *         Useful for logging/diagnostics; not used by FOC.
 * @param  st  optional out: comm status (may be NULL)
 */
uint16_t AS5047_ReadAngle(SPI_HandleTypeDef *hspi, AS5047_Status *st);

/* Low-level register access, exposed for diagnostics. */
uint16_t AS5047_ReadRegister(SPI_HandleTypeDef *hspi, uint16_t addr, AS5047_Status *st);

/** @brief Dump error/diagnostic/settings registers and angle via the USB logger. */
void AS5047_LogStatus(SPI_HandleTypeDef *hspi);

#endif /* AS5047_H */
