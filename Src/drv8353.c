/**
  ******************************************************************************
  * @file    drv8353.c
  * @brief   Bit-banged SPI driver and boot configuration for the DRV8353RS.
  *          See drv8353.h for pin mapping and frame format.
  ******************************************************************************
  */

#include "drv8353.h"
#include "log.h"

/* ------------------------------------------------------------------------- */
/* Register bit fields (verified against TI DRV8353 datasheet SLVSDJ6)       */
/* ------------------------------------------------------------------------- */

/* Driver Control (0x02) */
#define DRV_CTRL_OCP_ACT     (1U << 10)  /* 0 = shut down only the faulted half-bridge */
#define DRV_CTRL_DIS_GDUV    (1U << 9)   /* 0 = gate-drive UVLO fault enabled           */
#define DRV_CTRL_DIS_GDF     (1U << 8)   /* 0 = gate-drive fault enabled                */
#define DRV_CTRL_OTW_REP     (1U << 7)   /* 1 = report over-temp warning on nFAULT      */
#define DRV_CTRL_PWM_MODE_6X (0U << 5)   /* bits[6:5] = 00 -> 6x PWM mode               */
#define DRV_CTRL_COAST       (1U << 2)
#define DRV_CTRL_BRAKE       (1U << 1)
#define DRV_CTRL_CLR_FLT     (1U << 0)   /* 1 = clear latched faults                    */

/* Gate Drive HS (0x03) */
#define DRV_GDHS_LOCK_UNLOCKED (0x3U << 8) /* bits[10:8] LOCK; reads 011 when unlocked  */

/* Gate Drive LS (0x04) */
#define DRV_GDLS_TDRIVE_500NS (0x1U << 8) /* bits[9:8] peak gate-current drive time     */

/* OCP Control (0x05) */
#define DRV_OCP_DEAD_TIME_100NS (0x1U << 8) /* bits[9:8]; only used in 1x/3x PWM modes  */
#define DRV_OCP_MODE_LATCH      (0x0U << 6) /* bits[7:6] = 00 -> overcurrent latches off */
#define DRV_OCP_DEG_4US         (0x2U << 4) /* bits[5:4] overcurrent deglitch time      */

/* CSA Control (0x06) */
#define DRV_CSA_VREF_DIV     (1U << 9)   /* 1 = VREF/2 reference -> bidirectional sense  */
#define DRV_CSA_SEN_LVL_1V   (0x3U << 0) /* bits[1:0] sense overcurrent level = 1.0 V    */

/* ------------------------------------------------------------------------- */
/* Bit-bang primitives                                                       */
/* ------------------------------------------------------------------------- */

#define DRV_CS_LOW()    HAL_GPIO_WritePin(DRV8353_CS_PORT,   DRV8353_CS_PIN,   GPIO_PIN_RESET)
#define DRV_CS_HIGH()   HAL_GPIO_WritePin(DRV8353_CS_PORT,   DRV8353_CS_PIN,   GPIO_PIN_SET)
#define DRV_SCLK_HIGH() HAL_GPIO_WritePin(DRV8353_SCLK_PORT, DRV8353_SCLK_PIN, GPIO_PIN_SET)
#define DRV_SCLK_LOW()  HAL_GPIO_WritePin(DRV8353_SCLK_PORT, DRV8353_SCLK_PIN, GPIO_PIN_RESET)

/* ~0.5 us half-bit -> SCLK ~1 MHz, well under the DRV8353 10 MHz limit. */
static inline void drv_delay(void)
{
  for (volatile uint32_t i = 0; i < 30U; i++)
  {
    __NOP();
  }
}

static void drv8353_gpio_init(void)
{
  GPIO_InitTypeDef io = {0};

  /* CS / SCLK / MOSI: push-pull outputs. Idle: CS high, SCLK low. */
  DRV_CS_HIGH();
  DRV_SCLK_LOW();
  HAL_GPIO_WritePin(DRV8353_MOSI_PORT, DRV8353_MOSI_PIN, GPIO_PIN_RESET);

  io.Mode  = GPIO_MODE_OUTPUT_PP;
  io.Pull  = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_HIGH;

  io.Pin = DRV8353_CS_PIN;
  HAL_GPIO_Init(DRV8353_CS_PORT, &io);
  io.Pin = DRV8353_SCLK_PIN;
  HAL_GPIO_Init(DRV8353_SCLK_PORT, &io);
  io.Pin = DRV8353_MOSI_PIN;
  HAL_GPIO_Init(DRV8353_MOSI_PORT, &io);

  /* MISO: input (CubeMX generates it as an output by default). */
  io.Mode = GPIO_MODE_INPUT;
  io.Pull = GPIO_NOPULL;
  io.Pin  = DRV8353_MISO_PIN;
  HAL_GPIO_Init(DRV8353_MISO_PORT, &io);
}

/* Exchange one 16-bit frame, MSB first. Clock idles low; MOSI is set while
   SCLK is low (DRV captures it on the falling edge), MISO is sampled while
   SCLK is high (DRV drives it on the rising edge). */
static uint16_t drv8353_xfer(uint16_t tx)
{
  uint16_t rx = 0;

  DRV_CS_LOW();
  drv_delay();

  for (int8_t i = 15; i >= 0; i--)
  {
    HAL_GPIO_WritePin(DRV8353_MOSI_PORT, DRV8353_MOSI_PIN,
                      ((tx >> i) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    drv_delay();

    DRV_SCLK_HIGH();                 /* rising edge: DRV shifts SDO out */
    drv_delay();

    if (HAL_GPIO_ReadPin(DRV8353_MISO_PORT, DRV8353_MISO_PIN) == GPIO_PIN_SET)
    {
      rx |= (uint16_t)(1U << i);
    }

    DRV_SCLK_LOW();                  /* falling edge: DRV captures SDI  */
    drv_delay();
  }

  DRV_CS_HIGH();
  drv_delay();
  return rx;
}

/* ------------------------------------------------------------------------- */
/* Register access                                                           */
/* ------------------------------------------------------------------------- */

uint16_t DRV8353_ReadRegister(uint8_t addr)
{
  uint16_t frame = (uint16_t)((1U << 15) | ((uint16_t)(addr & 0x0FU) << 11));
  return (uint16_t)(drv8353_xfer(frame) & 0x07FFU);
}

void DRV8353_WriteRegister(uint8_t addr, uint16_t data)
{
  uint16_t frame = (uint16_t)(((uint16_t)(addr & 0x0FU) << 11) | (data & 0x07FFU));
  (void)drv8353_xfer(frame);
}

void DRV8353_GetFaultStatus(uint16_t *status1, uint16_t *status2)
{
  if (status1 != NULL)
  {
    *status1 = DRV8353_ReadRegister(DRV8353_REG_FAULT_STATUS1);
  }
  if (status2 != NULL)
  {
    *status2 = DRV8353_ReadRegister(DRV8353_REG_VGS_STATUS2);
  }
}

/* ------------------------------------------------------------------------- */
/* Boot configuration                                                        */
/* ------------------------------------------------------------------------- */

DRV8353_Status DRV8353_Init(void)
{
  uint16_t driver_ctrl, gate_hs, gate_ls, ocp_ctrl, csa_ctrl;
  uint16_t rb;

  drv8353_gpio_init();
  HAL_Delay(2);   /* DRV8353 wake-up time after ENABLE is asserted (tWAKE) */

  /* Assemble register contents. */
  driver_ctrl = DRV_CTRL_OTW_REP | DRV_CTRL_PWM_MODE_6X;   /* 6x PWM, protections on */

  gate_hs = (uint16_t)(((uint16_t)DRV8353_IDRIVEP_CODE << 4) |
                        (uint16_t)DRV8353_IDRIVEN_CODE);

  gate_ls = (uint16_t)(DRV_GDLS_TDRIVE_500NS |
                       ((uint16_t)DRV8353_IDRIVEP_CODE << 4) |
                        (uint16_t)DRV8353_IDRIVEN_CODE);

  ocp_ctrl = (uint16_t)(DRV_OCP_DEAD_TIME_100NS | DRV_OCP_MODE_LATCH |
                        DRV_OCP_DEG_4US | (uint16_t)(DRV8353_VDS_LVL_CODE & 0x0FU));

  csa_ctrl = (uint16_t)(DRV_CSA_VREF_DIV |
                        ((uint16_t)DRV8353_CSA_GAIN_CODE << 6) |
                        DRV_CSA_SEN_LVL_1V);

  /* Clear any latched power-up faults, then write the real configuration. */
  DRV8353_WriteRegister(DRV8353_REG_DRIVER_CTRL, driver_ctrl | DRV_CTRL_CLR_FLT);
  DRV8353_WriteRegister(DRV8353_REG_DRIVER_CTRL, driver_ctrl);
  DRV8353_WriteRegister(DRV8353_REG_GATE_DRIVE_HS, gate_hs);
  DRV8353_WriteRegister(DRV8353_REG_GATE_DRIVE_LS, gate_ls);
  DRV8353_WriteRegister(DRV8353_REG_OCP_CTRL, ocp_ctrl);
  DRV8353_WriteRegister(DRV8353_REG_CSA_CTRL, csa_ctrl);

  /* Verify the configuration registers read back as written. The LOCK field
     (GATE_DRIVE_HS bits 10:8) only accepts the codes 110/011 and ignores any
     other write, so it reads back 011 (unlocked) rather than the 000 written
     above; expecting 011 also confirms the registers are not locked. */
  if ((DRV8353_ReadRegister(DRV8353_REG_DRIVER_CTRL)   != driver_ctrl) ||
      (DRV8353_ReadRegister(DRV8353_REG_GATE_DRIVE_HS) !=
                            (uint16_t)(DRV_GDHS_LOCK_UNLOCKED | gate_hs)) ||
      (DRV8353_ReadRegister(DRV8353_REG_GATE_DRIVE_LS) != gate_ls)     ||
      (DRV8353_ReadRegister(DRV8353_REG_OCP_CTRL)      != ocp_ctrl)    ||
      (DRV8353_ReadRegister(DRV8353_REG_CSA_CTRL)      != csa_ctrl))
  {
    return DRV8353_ERR_VERIFY;
  }

  /* A clean, enabled driver with no PWM activity should report no faults. */
  rb = DRV8353_ReadRegister(DRV8353_REG_FAULT_STATUS1);
  if (rb & DRV8353_FAULT1_FAULT)
  {
    return DRV8353_ERR_FAULT;
  }

  return DRV8353_OK;
}

void DRV8353_LogStatus(void)
{
  uint16_t f1 = DRV8353_ReadRegister(DRV8353_REG_FAULT_STATUS1);
  uint16_t f2 = DRV8353_ReadRegister(DRV8353_REG_VGS_STATUS2);

  LOG_Printf("[DRV8353] FAULT1=0x%03X VGS2=0x%03X\r\n", f1, f2);
  LOG_Printf("[DRV8353]   FAULT=%u VDS_OCP=%u GDF=%u UVLO=%u OTSD=%u\r\n",
             (unsigned)!!(f1 & (1U << 10)), (unsigned)!!(f1 & (1U << 9)),
             (unsigned)!!(f1 & (1U << 8)),  (unsigned)!!(f1 & (1U << 7)),
             (unsigned)!!(f1 & (1U << 6)));
  LOG_Printf("[DRV8353] DRV_CTRL=0x%03X GATE_HS=0x%03X GATE_LS=0x%03X\r\n",
             DRV8353_ReadRegister(DRV8353_REG_DRIVER_CTRL),
             DRV8353_ReadRegister(DRV8353_REG_GATE_DRIVE_HS),
             DRV8353_ReadRegister(DRV8353_REG_GATE_DRIVE_LS));
  LOG_Printf("[DRV8353] OCP_CTRL=0x%03X CSA_CTRL=0x%03X\r\n",
             DRV8353_ReadRegister(DRV8353_REG_OCP_CTRL),
             DRV8353_ReadRegister(DRV8353_REG_CSA_CTRL));
}
