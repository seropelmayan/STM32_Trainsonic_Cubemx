
/**
  ******************************************************************************
  * @file    mc_app_hooks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements default motor control app hooks.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup MCAppHooks
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_app_hooks.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup COMMON_MC
  * @{
  */

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks.
 *
 *
 * @{
 */

/**
 * @brief Hook function called right before the end of the MCboot function.
 *
 *
 *
 */
__weak void MC_APP_BootHook(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed at the end of MCboot().
   */

/* USER CODE BEGIN BootHook */
  /* Ropetow: load the compiled anti-cogging table into the runtime LUT. */
  {
    extern void Ropetow_CoggInit(void);
    Ropetow_CoggInit();
  }
/* USER CODE END BootHook */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
__weak void MC_APP_PostMediumFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

/* USER SECTION BEGIN PostMediumFrequencyHookM1 */
  /* Ropetow: update the SPI encoder estimate here, in the medium-frequency THREAD
     context where the HAL SPI read is safe (it hangs in the priority-0 FOC ISR).
     The HF FOC ISR only extrapolates the published angle. See mc_tasks_foc.c. */
  {
    extern void Ropetow_EncoderUpdate(void);
    Ropetow_EncoderUpdate();
  }
  /* Ropetow: position->torque PD servo. Must run AFTER the encoder update so it
     sees the freshly published mechanical angle / speed this same MF cycle. */
  {
    extern void Ropetow_PositionControl(void);
    Ropetow_PositionControl();
  }
/* USER SECTION END PostMediumFrequencyHookM1 */
}

/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
