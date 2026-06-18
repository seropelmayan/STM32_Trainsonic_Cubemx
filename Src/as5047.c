/**
  ******************************************************************************
  * @file    as5047.c
  * @brief   Boot-time configuration driver for the AS5047P magnetic encoder.
  *          See as5047.h for the role of this driver.
  ******************************************************************************
  */

#include "as5047.h"
#include "log.h"

/* ------------------------------------------------------------------------- */
/* AS5047P register map (verified against ams datasheet DS000324)            */
/* ------------------------------------------------------------------------- */
#define AS5047P_NOP            0x0000U
#define AS5047P_ERRFL          0x0001U
#define AS5047P_DIAAGC         0x3FFCU
#define AS5047P_ANGLECOM       0x3FFFU
#define AS5047P_SETTINGS1      0x0018U
#define AS5047P_SETTINGS2      0x0019U

/* 16-bit SPI frame fields */
#define AS5047P_FRAME_PARITY   (1U << 15)   /* even parity over bits [14:0]   */
#define AS5047P_FRAME_RW_READ  (1U << 14)   /* command: 1 = read, 0 = write   */
#define AS5047P_FRAME_EF       (1U << 14)   /* response: 1 = error flag set   */
#define AS5047P_FRAME_DATA     0x3FFFU      /* data/address payload bits      */

/* SETTINGS1 (0x18) bit fields */
#define AS5047P_S1_DIR         (1U << 2)    /* rotation direction             */
#define AS5047P_S1_UVW_ABI     (1U << 3)    /* 0 = ABI active, 1 = UVW active  */
#define AS5047P_S1_ABIBIN      (1U << 5)    /* 0 = decimal, 1 = binary ABIRES  */

/* SETTINGS2 (0x19) bit fields */
#define AS5047P_S2_ABIRES_Msk  (0x7U << 5)  /* ABIRES[2:0] @ bits [7:5]        */
#define AS5047P_S2_ABIRES_MAX  (0x0U << 5)  /* 000 = 4096 steps/rev = 1024 PPR */

/* DIAAGC (0x3FFC) diagnostic flags */
#define AS5047P_DIAG_LF        (1U << 8)    /* offset compensation finished    */
#define AS5047P_DIAG_COF       (1U << 9)    /* CORDIC overflow (angle invalid) */
#define AS5047P_DIAG_MAGH      (1U << 10)   /* field too strong                */
#define AS5047P_DIAG_MAGL      (1U << 11)   /* field too weak                  */

/* ------------------------------------------------------------------------- */
/* Low-level helpers                                                         */
/* ------------------------------------------------------------------------- */

#define AS5047_CS_LOW()   HAL_GPIO_WritePin(AS5047_CS_GPIO_Port, AS5047_CS_Pin, GPIO_PIN_RESET)
#define AS5047_CS_HIGH()  HAL_GPIO_WritePin(AS5047_CS_GPIO_Port, AS5047_CS_Pin, GPIO_PIN_SET)

/* Short busy-wait (~1 us @170 MHz) to satisfy the AS5047P CS setup time and
   minimum CS-high time between frames (tCSn ~350 ns). */
static inline void as5047_delay(void)
{
  for (volatile uint32_t i = 0; i < 60U; i++)
  {
    __NOP();
  }
}

/* Even parity over all bits of v (bit15 should be 0 on input). */
static uint8_t as5047_parity(uint16_t v)
{
  v ^= (uint16_t)(v >> 8);
  v ^= (uint16_t)(v >> 4);
  v ^= (uint16_t)(v >> 2);
  v ^= (uint16_t)(v >> 1);
  return (uint8_t)(v & 1U);
}

/* Build a command frame (read or write) with the parity bit set. */
static uint16_t as5047_cmd(uint16_t addr, uint8_t read)
{
  uint16_t f = (uint16_t)(addr & AS5047P_FRAME_DATA);
  if (read)
  {
    f |= AS5047P_FRAME_RW_READ;
  }
  f |= (uint16_t)(as5047_parity(f) << 15);
  return f;
}

/* Build a write-data frame (no R/W bit) with the parity bit set. */
static uint16_t as5047_data(uint16_t data)
{
  uint16_t f = (uint16_t)(data & AS5047P_FRAME_DATA);
  f |= (uint16_t)(as5047_parity(f) << 15);
  return f;
}

/* One framed 16-bit transfer with software CS. */
static uint16_t as5047_xfer(SPI_HandleTypeDef *hspi, uint16_t tx)
{
  uint16_t rx = 0;

  AS5047_CS_LOW();
  as5047_delay();
  (void)HAL_SPI_TransmitReceive(hspi, (uint8_t *)&tx, (uint8_t *)&rx, 1, AS5047_SPI_TIMEOUT);
  AS5047_CS_HIGH();
  as5047_delay();

  return rx;
}

/* Pipelined register read: send read command, then clock the data out with a
   following NOP read. Validates response parity and error flag. */
uint16_t AS5047_ReadRegister(SPI_HandleTypeDef *hspi, uint16_t addr, AS5047_Status *st)
{
  (void)as5047_xfer(hspi, as5047_cmd(addr, 1U));            /* issue command   */
  uint16_t resp = as5047_xfer(hspi, as5047_cmd(AS5047P_NOP, 1U)); /* read data */

  if (st != NULL)
  {
    if ((resp & AS5047P_FRAME_EF) || (as5047_parity(resp) != 0U))
    {
      *st = AS5047_ERR_COMM;
    }
  }
  return (uint16_t)(resp & AS5047P_FRAME_DATA);
}

/* Register write: command frame followed by data frame. */
static void as5047_write_reg(SPI_HandleTypeDef *hspi, uint16_t addr, uint16_t data)
{
  (void)as5047_xfer(hspi, as5047_cmd(addr, 0U));
  (void)as5047_xfer(hspi, as5047_data(data));
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

uint16_t AS5047_ReadAngle(SPI_HandleTypeDef *hspi, AS5047_Status *st)
{
  return AS5047_ReadRegister(hspi, AS5047P_ANGLECOM, st);
}

AS5047_Status AS5047_Init(SPI_HandleTypeDef *hspi)
{
  AS5047_Status st = AS5047_OK;
  uint16_t s1, s2, v1, v2, diag;

  /* PA4 may power up low; make sure CS is deasserted before the first frame. */
  AS5047_CS_HIGH();
  as5047_delay();

  /* Clear any power-up framing error (first transaction often flags one). */
  (void)AS5047_ReadRegister(hspi, AS5047P_ERRFL, NULL);

  /* Confirm the link works now. */
  (void)AS5047_ReadRegister(hspi, AS5047P_SETTINGS1, &st);
  if (st != AS5047_OK)
  {
    return st;   /* no point configuring a dead/miswired link */
  }

  /* Magnet diagnostics: weak/strong field or CORDIC overflow => angle invalid. */
  diag = AS5047_ReadRegister(hspi, AS5047P_DIAAGC, &st);
  if (diag & (AS5047P_DIAG_MAGL | AS5047P_DIAG_MAGH | AS5047P_DIAG_COF))
  {
    st = AS5047_ERR_MAGNET;   /* report, but still program the registers below */
  }

  /* SETTINGS1: select ABI mode, binary resolution, set direction.
     Read-modify-write to preserve factory/reserved bits. */
  s1 = AS5047_ReadRegister(hspi, AS5047P_SETTINGS1, NULL);
  s1 &= (uint16_t)~AS5047P_S1_UVW_ABI;   /* 0 -> ABI active (not UVW)          */
  s1 |=  AS5047P_S1_ABIBIN;              /* 1 -> binary (2^N) resolution       */
#if (AS5047_INVERT_DIR != 0)
  s1 |=  AS5047P_S1_DIR;
#else
  s1 &= (uint16_t)~AS5047P_S1_DIR;
#endif
  as5047_write_reg(hspi, AS5047P_SETTINGS1, s1);

  /* SETTINGS2: ABIRES = 000 -> 4096 steps/rev (= 1024 PPR), preserve HYS/UVWPP. */
  s2 = AS5047_ReadRegister(hspi, AS5047P_SETTINGS2, NULL);
  s2 &= (uint16_t)~AS5047P_S2_ABIRES_Msk;
  s2 |=  AS5047P_S2_ABIRES_MAX;
  as5047_write_reg(hspi, AS5047P_SETTINGS2, s2);

  /* Verify the bits we care about actually took. */
  v1 = AS5047_ReadRegister(hspi, AS5047P_SETTINGS1, NULL);
  v2 = AS5047_ReadRegister(hspi, AS5047P_SETTINGS2, NULL);
  if (((v1 ^ s1) & (AS5047P_S1_UVW_ABI | AS5047P_S1_ABIBIN | AS5047P_S1_DIR)) ||
      ((v2 ^ s2) & AS5047P_S2_ABIRES_Msk))
  {
    st = AS5047_ERR_VERIFY;
  }

  return st;
}

void AS5047_LogStatus(SPI_HandleTypeDef *hspi)
{
  AS5047_Status st = AS5047_OK;
  uint16_t errfl = AS5047_ReadRegister(hspi, AS5047P_ERRFL, &st);
  uint16_t diag  = AS5047_ReadRegister(hspi, AS5047P_DIAAGC, NULL);
  uint16_t s1    = AS5047_ReadRegister(hspi, AS5047P_SETTINGS1, NULL);
  uint16_t s2    = AS5047_ReadRegister(hspi, AS5047P_SETTINGS2, NULL);
  uint16_t ang   = AS5047_ReadRegister(hspi, AS5047P_ANGLECOM, NULL);
  uint32_t cdeg  = ((uint32_t)ang * 36000U) / 16384U;   /* centi-degrees */

  LOG_Printf("[AS5047] ERRFL=0x%03X DIAAGC=0x%03X\r\n", errfl, diag);
  LOG_Printf("[AS5047]   MAGL=%u MAGH=%u COF=%u LF=%u AGC=%u\r\n",
             (unsigned)!!(diag & AS5047P_DIAG_MAGL),
             (unsigned)!!(diag & AS5047P_DIAG_MAGH),
             (unsigned)!!(diag & AS5047P_DIAG_COF),
             (unsigned)!!(diag & AS5047P_DIAG_LF),
             (unsigned)(diag & 0xFFU));
  LOG_Printf("[AS5047] SETTINGS1=0x%03X (ABIBIN=%u UVW_ABI=%u DIR=%u)\r\n", s1,
             (unsigned)!!(s1 & AS5047P_S1_ABIBIN),
             (unsigned)!!(s1 & AS5047P_S1_UVW_ABI),
             (unsigned)!!(s1 & AS5047P_S1_DIR));
  LOG_Printf("[AS5047] SETTINGS2=0x%03X (ABIRES=%u -> %s)\r\n", s2,
             (unsigned)((s2 & AS5047P_S2_ABIRES_Msk) >> 5),
             ((s2 & AS5047P_S2_ABIRES_Msk) == AS5047P_S2_ABIRES_MAX)
                 ? "4096 steps / 1024 PPR" : "reduced");
  LOG_Printf("[AS5047] angle=0x%04X (%lu.%02lu deg) comm=%s\r\n", ang,
             (unsigned long)(cdeg / 100U), (unsigned long)(cdeg % 100U),
             (st == AS5047_OK) ? "OK" : "ERR");
}

void AS5047_LogAngle(SPI_HandleTypeDef *hspi)
{
  AS5047_Status st = AS5047_OK;
  uint16_t raw  = AS5047_ReadAngle(hspi, &st);          /* ANGLECOM, 0..16383 */
  uint16_t diag = AS5047_ReadRegister(hspi, AS5047P_DIAAGC, NULL);
  uint16_t err  = AS5047_ReadRegister(hspi, AS5047P_ERRFL, NULL);
  uint32_t cdeg = ((uint32_t)raw * 36000U) / 16384U;    /* centi-degrees */

  LOG_Printf("[enc] raw=%5u %lu.%02lu deg AGC=%u MAGL=%u MAGH=%u COF=%u ERRFL=0x%03X %s\r\n",
             (unsigned)raw,
             (unsigned long)(cdeg / 100U), (unsigned long)(cdeg % 100U),
             (unsigned)(diag & 0xFFU),
             (unsigned)!!(diag & AS5047P_DIAG_MAGL),
             (unsigned)!!(diag & AS5047P_DIAG_MAGH),
             (unsigned)!!(diag & AS5047P_DIAG_COF),
             err, (st == AS5047_OK) ? "OK" : "COMM-ERR");
}
