/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v3.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
/* speed-loop gain setters live in mc_tasks_foc.c (keeps the MC header chain out
   of this USB file); declared extern in the RX handler below. */
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  /* Live tuning over the USB serial terminal. Single-letter commands fire
   * IMMEDIATELY (no Enter needed); numeric commands accumulate digits and apply
   * on the next non-digit (Enter, space, or the next command letter).
   *   e / d        enable / disable INL correction
   *   + / -        INL sign +1 / -1
   *   c            re-arm INL capture (fires once speed is steady)
   *   v / b        speed feedback: v = SPI velocity, b = TIM3 quadrature
   *   g            trigger the d-axis current-loop step-response capture (+dump)
   *   p<n> / i<n>  speed-loop Kp / Ki        (e.g.  p8000  i150 )
   *   P<n> / I<n>  torque(Iq+Id)-loop Kp / Ki
   *   D<n>         dead-time compensation magnitude (s16 V); 0 = off
   *   f<n>         current LPF cutoff in Hz on measured Iq/Id; 0 = off
   *   F<n>         velocity-smoothing cutoff Hz on SPI speed estimate; 0 = raw
   *   t<n>         TORQUE-mode current setpoint in mA (commands Iq; clamped 0..8000)
   *   s<n>         SPEED-mode setpoint in rpm (commands speed ramp; clamped 0..600)
   *   r            toggle live raw-encoder monitor (motor stopped; check it's sane)
   *   a            AGC-vs-position sweep (spin shaft slowly): magnet-field diagnostic
   *   y / Y        anti-cogging calibration: y = arm capture, Y = stop + dump map
   *   K / k        anti-cogging feed-forward ON / OFF
   *   C<n>         anti-cogging FF clamp (max |Iq FF|, s16 current units)
   * Gains clamp to [0..32767] (int16). Runs in USB IRQ context: only sets
   * volatile globals / PID fields -- never logs (log ring is single-producer). */
  extern volatile uint8_t  g_inl_enable;
  extern volatile float    g_inl_sign;
  extern volatile uint8_t  g_cal_state;
  extern volatile uint16_t g_cal_idx;
  extern volatile uint8_t  g_use_spi_speed;
  extern volatile uint8_t  g_step_request;
  extern void Ropetow_SetSpeedKp(int32_t kp);  /* mc_tasks_foc.c */
  extern void Ropetow_SetSpeedKi(int32_t ki);  /* mc_tasks_foc.c */
  extern void Ropetow_SetTorqueKp(int32_t kp); /* mc_tasks_foc.c */
  extern void Ropetow_SetTorqueKi(int32_t ki); /* mc_tasks_foc.c */
  extern volatile int16_t g_dt_comp;           /* dead-time comp magnitude (mc_tasks_foc.c) */
  extern void Ropetow_SetCurrentLpf(int32_t fc_hz); /* current LPF cutoff (mc_tasks_foc.c) */
  extern void Ropetow_SetEncVelLp(int32_t fc_hz);   /* velocity-smoothing cutoff (mc_tasks_foc.c) */
  extern volatile int32_t g_torque_set_ma;          /* torque setpoint mA (mc_tasks_foc.c) */
  extern volatile uint8_t g_torque_set_req;
  extern volatile int32_t g_speed_set_rpm;          /* speed setpoint rpm (mc_tasks_foc.c) */
  extern volatile uint8_t g_speed_set_req;
  extern volatile uint8_t g_agc_log_req;            /* AGC sweep trigger (mc_tasks_foc.c) */
  extern volatile uint8_t g_enc_mon;                /* live raw-encoder monitor toggle (mc_tasks_foc.c) */
  extern volatile uint8_t g_cogg_cal_req;           /* anti-cogging cal arm (mc_tasks_foc.c) */
  extern volatile uint8_t g_cogg_cal_state;         /* anti-cogging cal state (mc_tasks_foc.c) */
  extern volatile uint8_t g_cogg_enable;            /* anti-cogging FF on/off (mc_tasks_foc.c) */
  extern volatile int16_t g_cogg_clamp;             /* anti-cogging FF clamp (mc_tasks_foc.c) */

  static char    s_numcmd = 0;   /* pending numeric command letter (p/i/P/I), 0=none */
  static int32_t s_val    = 0;
  static uint8_t s_ndig   = 0U;

  for (uint32_t i = 0U; i < *Len; i++)
  {
    uint8_t ch = Buf[i];

    if ((ch >= '0') && (ch <= '9'))
    {
      if (s_numcmd != 0)
      {
        s_val = (s_val * 10) + (int32_t)(ch - '0');
        if (s_val > 32767) { s_val = 32767; }
        s_ndig++;
      }
      continue;   /* digit consumed */
    }

    /* any non-digit finalizes a pending numeric command first */
    if ((s_numcmd != 0) && (s_ndig > 0U))
    {
      switch (s_numcmd)
      {
        case 'p': Ropetow_SetSpeedKp(s_val);  break;
        case 'i': Ropetow_SetSpeedKi(s_val);  break;
        case 'P': Ropetow_SetTorqueKp(s_val); break;
        case 'I': Ropetow_SetTorqueKi(s_val); break;
        case 'D': g_dt_comp = (int16_t)s_val;  break;
        case 'f': Ropetow_SetCurrentLpf(s_val); break;
        case 'F': Ropetow_SetEncVelLp(s_val); break;
        case 't': g_torque_set_ma = s_val; g_torque_set_req = 1U; break;
        case 's': g_speed_set_rpm = s_val; g_speed_set_req = 1U; break;
        case 'C': g_cogg_clamp = (int16_t)s_val; break;
        default:  break;
      }
    }
    s_numcmd = 0; s_val = 0; s_ndig = 0U;

    /* then act on this character */
    switch (ch)
    {
      case 'e': g_inl_enable = 1U;     break;
      case 'd': g_inl_enable = 0U;     break;
      case '+': g_inl_sign   = 1.0f;   break;
      case '-': g_inl_sign   = -1.0f;  break;
      case 'c': g_cal_idx = 0U; g_cal_state = 0U; break;
      case 'v': g_use_spi_speed = 1U;  break;
      case 'b': g_use_spi_speed = 0U;  break;
      case 'g': g_step_request = 1U;   break;
      case 'r': g_enc_mon ^= 1U;       break;   /* toggle live encoder monitor  */
      case 'a': g_agc_log_req = 1U;    break;   /* AGC-vs-position sweep        */
      case 'y': g_cogg_cal_req = 1U;   break;   /* arm anti-cogging calibration */
      case 'Y': if (g_cogg_cal_state == 1U) { g_cogg_cal_state = 2U; } break; /* stop + dump */
      case 'K': g_cogg_enable = 1U;    break;   /* anti-cogging FF on           */
      case 'k': g_cogg_enable = 0U;    break;   /* anti-cogging FF off          */
      case 'p': case 'i': case 'P': case 'I': case 'D': case 'f': case 'F': case 't': case 's': case 'C':
        s_numcmd = (char)ch; s_val = 0; s_ndig = 0U; break;
      default:  break;   /* CR/LF/space/unknown: ignore */
    }
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
