/**
  ******************************************************************************
  * @file    drv8353.h
  * @brief   Software (bit-banged) SPI driver and boot configuration for the
  *          TI DRV8353RS three-phase smart gate driver.
  *
  * SPI is bit-banged on GPIO (no hardware SPI peripheral):
  *      CS   = PA15   (output, idle high)
  *      SCLK = PB9    (output, idle low)
  *      MOSI = PB11   (output)         [DRV SDI]
  *      MISO = PB10   (input)          [DRV SDO]
  *
  * Frame: 16 bits, bit15 = R/W (1 = read), bits[14:11] = address,
  *        bits[10:0] = data. Clock idles low; DRV captures SDI on the falling
  *        edge and drives SDO on the rising edge.
  *
  * The driver is configured once at boot, before the motor is started:
  *   - 6x PWM mode (STM32 supplies 6 signals with its own dead time)
  *   - 1 A peak source / 2 A peak sink gate drive current
  *   - CSA gain 20 V/V (firmware AMPLIFICATION_GAIN = 20 V/V * 5 mOhm shunt
  *     = 0.1 V/A; do NOT put the bare V/V number in the MC config)
  *   - VDS overcurrent protection (threshold MUST be reviewed for the MOSFETs)
  ******************************************************************************
  */

#ifndef DRV8353_H
#define DRV8353_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* Pin mapping                                                               */
/* ------------------------------------------------------------------------- */
#define DRV8353_CS_PORT     GPIOA
#define DRV8353_CS_PIN      GPIO_PIN_15
#define DRV8353_SCLK_PORT   GPIOB
#define DRV8353_SCLK_PIN    GPIO_PIN_9
#define DRV8353_MOSI_PORT   GPIOB
#define DRV8353_MOSI_PIN    GPIO_PIN_11
#define DRV8353_MISO_PORT   GPIOB
#define DRV8353_MISO_PIN    GPIO_PIN_10

/* ------------------------------------------------------------------------- */
/* User-tunable configuration (4-bit IDRIVE codes, see datasheet tables)     */
/* ------------------------------------------------------------------------- */
/* Peak gate-drive currents. Code 0b1111 = 1000 mA source / 2000 mA sink.
   (Tried lowering to 0x08 to slow dv/dt -> no change in the noise, so switching
   ringing/EMI is NOT the cause; reverted to the original max.) */
#define DRV8353_IDRIVEP_CODE   0x0FU   /* source: 1111 = 1000 mA (1 A)        */
#define DRV8353_IDRIVEN_CODE   0x0FU   /* sink:   1111 = 2000 mA (2 A)        */

/* Current-sense-amplifier gain. 00=5, 01=10, 10=20, 11=40 V/V.
   Firmware AMPLIFICATION_GAIN (power_stage_parameters.h) MUST match this. Change
   both together. 20 V/V -> 0.1 V/A, full scale +/-16.5 A.
   (Tried 40 V/V for 2x resolution -> measured Iq ripple dropped but motor got
   AUDIBLY LOUDER: higher counts/amp = higher effective current-loop gain = more
   voltage thrash. Confirms the loop is too aggressive; reverted.) */
#define DRV8353_CSA_GAIN_CODE  0x2U

/* !!! REVIEW BEFORE POWERING THE STAGE !!!
   VDS overcurrent threshold, 4-bit code (OCP Control register VDS_LVL).
   Trip voltage = code value (see datasheet OCP Control table); trip current
   ~= V_DS_threshold / RDS(on) of the power MOSFETs. The value below is a
   conservative placeholder, NOT computed for your FETs. Set it for your
   hardware or OCP will nuisance-trip or fail to protect. */
#define DRV8353_VDS_LVL_CODE   0x09U

/* ------------------------------------------------------------------------- */
/* Register addresses                                                        */
/* ------------------------------------------------------------------------- */
#define DRV8353_REG_FAULT_STATUS1   0x00U
#define DRV8353_REG_VGS_STATUS2     0x01U
#define DRV8353_REG_DRIVER_CTRL     0x02U
#define DRV8353_REG_GATE_DRIVE_HS   0x03U
#define DRV8353_REG_GATE_DRIVE_LS   0x04U
#define DRV8353_REG_OCP_CTRL        0x05U
#define DRV8353_REG_CSA_CTRL        0x06U

/* Fault Status 1 (0x00): bit10 = FAULT (logical OR of all fault conditions). */
#define DRV8353_FAULT1_FAULT        (1U << 10)

/* ------------------------------------------------------------------------- */
typedef enum
{
  DRV8353_OK = 0,
  DRV8353_ERR_VERIFY,   /* register read-back did not match what was written */
  DRV8353_ERR_FAULT     /* a fault is latched in the status registers        */
} DRV8353_Status;

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

/**
 * @brief  Initialise GPIO, wake the driver, write all configuration registers,
 *         verify them, and check fault status. Call once after MX_GPIO_Init()
 *         and before the motor is started.
 */
DRV8353_Status DRV8353_Init(void);

/** @brief Read an 11-bit register value. */
uint16_t DRV8353_ReadRegister(uint8_t addr);

/** @brief Write an 11-bit register value. */
void     DRV8353_WriteRegister(uint8_t addr, uint16_t data);

/** @brief Read both fault status registers (0x00, 0x01). */
void     DRV8353_GetFaultStatus(uint16_t *status1, uint16_t *status2);

/** @brief Dump fault status and configuration registers via the USB logger. */
void     DRV8353_LogStatus(void);

#endif /* DRV8353_H */
