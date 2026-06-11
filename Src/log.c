/**
  ******************************************************************************
  * @file    log.c
  * @brief   Non-blocking USB CDC logger (see log.h).
  ******************************************************************************
  */

#include "log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "usb_device.h"     /* hUsbDeviceFS, USBD_STATE_CONFIGURED */
#include "usbd_cdc.h"       /* USBD_CDC_HandleTypeDef (TxState)    */
#include "usbd_cdc_if.h"    /* CDC_Transmit_FS                     */

#define LOG_BUF_SIZE   2048U   /* ring buffer size (power-of-two not required) */
#define LOG_TX_CHUNK   64U     /* bytes pushed to the CDC endpoint per drain   */
#define LOG_TMP_SIZE   200U    /* max length of one LOG_Printf() expansion     */

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t           s_ring[LOG_BUF_SIZE];
static volatile uint16_t s_head;   /* producer (LOG_Printf)  */
static volatile uint16_t s_tail;   /* consumer (LOG_Process) */
static uint8_t           s_txbuf[LOG_TX_CHUNK];

void LOG_Init(void)
{
  s_head = 0;
  s_tail = 0;
}

void LOG_Printf(const char *fmt, ...)
{
  char tmp[LOG_TMP_SIZE];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  if (n <= 0)
  {
    return;
  }
  if (n > (int)sizeof(tmp))
  {
    n = (int)sizeof(tmp);   /* output was truncated by vsnprintf */
  }

  for (int i = 0; i < n; i++)
  {
    uint16_t next = (uint16_t)((s_head + 1U) % LOG_BUF_SIZE);
    if (next == s_tail)
    {
      break;                /* buffer full: drop the remainder */
    }
    s_ring[s_head] = (uint8_t)tmp[i];
    s_head = next;
  }
}

/* True only when the device is enumerated and the previous CDC IN transfer has
   completed -- i.e. it is safe to reuse s_txbuf and start a new transfer. */
static uint8_t cdc_tx_ready(void)
{
  USBD_CDC_HandleTypeDef *hcdc;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return 0;
  }
  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL)
  {
    return 0;
  }
  return (hcdc->TxState == 0U) ? 1U : 0U;
}

void LOG_Process(void)
{
  uint16_t len;

  if (!cdc_tx_ready())
  {
    return;
  }
  if (s_head == s_tail)
  {
    return;   /* nothing pending */
  }

  /* Copy one contiguous chunk (up to the ring wrap) into the TX bounce buffer.
     Copying first, while the endpoint is idle, keeps the data stable for the
     duration of the zero-copy CDC transfer. */
  len = (s_head > s_tail) ? (uint16_t)(s_head - s_tail)
                          : (uint16_t)(LOG_BUF_SIZE - s_tail);
  if (len > LOG_TX_CHUNK)
  {
    len = LOG_TX_CHUNK;
  }
  memcpy(s_txbuf, &s_ring[s_tail], len);

  if (CDC_Transmit_FS(s_txbuf, len) == USBD_OK)
  {
    s_tail = (uint16_t)((s_tail + len) % LOG_BUF_SIZE);
  }
  /* On USBD_BUSY/FAIL leave s_tail; retry on the next call. */
}
