
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
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
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"
#include "ics_g4xx_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

  const ICS_Params_t ICS_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio         = FREQ_RATIO,
  .IsHigherFreqTim   = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ADC1,
  .ADCx_2 = ADC2,
  .ADCConfig1        = (uint32_t)(MC_ADC_CHANNEL_15 << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
  .ADCConfig2        = (uint32_t)(MC_ADC_CHANNEL_12 << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .TIMx              = TIM1

};

ScaleParams_t scaleParams_M1 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV,
 .frequency = U_RPM/SPEED_UNIT
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/

