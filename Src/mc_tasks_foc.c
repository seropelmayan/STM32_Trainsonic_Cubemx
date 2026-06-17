
/**
  ******************************************************************************
  * @file    mc_tasks_foc.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
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
  * @ingroup MCTasksFOC
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mc_app_hooks.h"

/** @addtogroup MCSDK
  * @{
  */

 /** @defgroup ConvFOC Conventional FOC
  *
  * @brief
  *
  * @{
  */

  /** @defgroup MCCockpit MC Cockpit
  *
  * @brief
  *
  * @{
  */

/** @addtogroup ConvFOC
  * @{
  */

/** @addtogroup	MCCockpit
  * @{
  */

/** @addtogroup	MCTasksFOC
  * @{
  */

/** @defgroup MCTasksFOC Motor Control Tasks for FOC
  *
  * @brief FOC legacy Motor Control subsystem configuration and operation routines.
  *
  * @{
  */

/* USER CODE BEGIN Includes */
#include "as5047.h"                 /* AS5047P absolute-angle SPI read     */
#include <math.h>                   /* sinf/cosf for the INL correction    */
extern SPI_HandleTypeDef hspi1;     /* AS5047 SPI link (defined in main.c) */
/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/
/* --- Encoder: AS5047 SPI absolute angle (Ropetow) ----------------------------
 * Ropetow_EncoderUpdate() (MF hook, mc_app_hooks.c, THREAD context) reads the
 * AS5047 absolute angle over SPI, captures the constant SPI<->motor offset vs the
 * low-speed-reliable TIM3 angle, and publishes the raw angle + a finite-diff
 * velocity. The HF FOC ISR extrapolates that (~1 kHz hook -> loop rate) and
 * commutates on it instead of the EMI-glitchy ABI/TIM3 pulses. */
#define ENC_USE_SPI_ANGLE  1     /* 1 = commutate on SPI angle, 0 = TIM3/ABI        */
#define ENC_SPI_INVERT     1     /* flip if SPI angle counts opposite the motor     */
#define ENC_SPI_SLOW_CNT   256   /* per-hook delta below which the offset is captured */
#define ENC_VEL_LP         0.3f  /* velocity low-pass (HF interpolation only)        */

/* --- Encoder INL (eccentricity) correction, Ropetow -------------------------
 * Cancels the encoder's once-per-MECHANICAL-rev angle error (magnet eccentricity
 * / non-linearity) BEFORE the *PP electrical conversion, so it isn't multiplied
 * 20x into the electrical angle. Fitted as 2 mechanical harmonics from a 1000-
 * sample / ~10-rev closed-loop capture at ~63 rpm (host fit, see logs.md):
 *     corr(theta) = C1*cos(t) + S1*sin(t) + C2*cos(2t) + S2*sin(2t)   [counts]
 *     mech_corr   = wrap16384(raw14 - sign*corr)
 * H1+H2 explain ~80-85% of the repeatable (angle-averaged) error; H1 amplitude
 * ~76 cnt = 1.7 deg mech = 34 deg elec. Units: encoder counts (16384 / mech rev).
 * MUST be verified empirically to REDUCE ripple -- if it doubles it, flip
 * INL_SIGN_DEFAULT to -1.0f (a single sign flip turns the fix into 2x the error). */
#define INL_C1  (37.450338f)
#define INL_S1  (66.162436f)
#define INL_C2  (8.039487f)
#define INL_S2  (-26.622343f)
#define INL_2PI_OVER_16384  (3.14159265358979f * 2.0f / 16384.0f)
#define INL_ENABLE_DEFAULT  0U     /* OFF: closed-loop table was invalid (raw angle best); toggle 'e' live to test */
#define INL_SIGN_DEFAULT    1.0f   /* flip to -1.0f if enabling INCREASES ripple     */
/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

/* USER CODE BEGIN Private Variables */
/* Encoder estimate shared between Ropetow_EncoderUpdate() (MF hook, thread ctx)
   and the HF FOC ISR: the hook does the SPI read & publishes, the ISR
   extrapolates the angle between hook updates. */
volatile float    g_enc_speed_rpm      = 0.0f; /* SPI-derived mech speed, rpm (diag) */
volatile int16_t  g_dbg_spi_minus_tim3 = 0;    /* diag: SPI angle - TIM3 angle (s16) */
volatile uint32_t g_spi_err_count      = 0U;   /* diag: SPI parity/EF read failures  */
static volatile float    g_enc_theta0     = 0.0f; /* published el angle at last hook   */
static volatile float    g_enc_omega_tick = 0.0f; /* el speed, counts per HF tick      */
static volatile uint32_t g_enc_base_tick  = 0U;   /* HF tick count at last publish     */
static volatile uint32_t g_enc_seq        = 0U;   /* increments each hook publish      */
static volatile uint32_t g_hf_tick_count  = 0U;   /* free-running HF tick counter (RUN) */
volatile uint8_t  g_use_spi_commutation = 1U; /* 1=SPI extrapolation, 0=TIM3/ABI (extern: main.c) */
volatile uint8_t  g_use_spi_speed = 0U;       /* 1=speed loop fed from SPI velocity, 0=TIM3 deltas */
/* Open-loop forced commutation (INL capture, method A): rotate the electrical
   angle at a constant commanded speed, ignoring the encoder, so the rotor follows
   a perfectly constant field -- the encoder logged vs that ramp is speed-ripple
   free. Driven from main.c boot; g_ol_step is el-angle (s16) per HF tick. */
volatile uint8_t  g_ol_enable = 0U;
volatile float    g_ol_step   = 0.0f;
static   float    g_ol_acc    = 0.0f;   /* free-running el-angle accumulator (s16) */
static   uint8_t  g_ol_seeded = 0U;     /* seed acc from MCSDK angle on RUN entry  */
/* INL calibration capture: raw ANGLECOM samples at constant speed, dumped over
   USB for host-side deviation analysis (idx -> raw14, mechanical 0..16383). */
#define CAL_N      1000           /* samples captured                          */
#define CAL_DECIM  10             /* 1 kHz hook / 10 = ~100 Hz sampling        */
uint16_t          g_cal_raw[CAL_N];
volatile uint16_t g_cal_idx   = 0U;
volatile uint8_t  g_cal_state = 0U; /* 0 idle, 1 capturing, 2 ready, 3 done    */
static   uint16_t g_cal_dec   = 0U;
/* INL correction runtime controls (toggle live to A/B test; see Private define) */
volatile uint8_t  g_inl_enable = INL_ENABLE_DEFAULT;
volatile float    g_inl_sign   = INL_SIGN_DEFAULT;
/* Current(torque)-loop step-response capture: log Iqref and measured Iq at the
   full HF loop rate (25 kHz) around a commanded Iq step, for tuning the Iq PI.
   STEP_DECIM=1 -> every ISR; STEP_N=512 -> ~20 ms window (settles in <1 ms). */
#define STEP_N         512
#define STEP_DECIM     1     /* current-loop capture: every HF ISR (25 kHz)       */
#define SPEED_CAP_DECIM 4    /* speed-loop capture: every 4th MF tick -> 250 Hz   */
                             /* 512 samples / 250 Hz = ~2.0 s window               */
int16_t           g_step_ref[STEP_N];
int16_t           g_step_iq[STEP_N];
volatile uint16_t g_step_idx     = 0U;
volatile uint8_t  g_step_state   = 0U;  /* 0 idle, 1 capturing, 2 ready, 3 done */
volatile uint8_t  g_step_request = 0U;  /* set by CDC 'g'; appTask injects step */
static   uint16_t g_step_dec     = 0U;
static   uint16_t g_spd_dec      = 0U;
/* Current injection for the step test. appTask sets these; FOC_CalcCurrRef pins
   Iqdref accordingly (same ctx -> no race). g_step_axis selects which loop:
   - d-axis (0): Id makes ZERO torque (aligns with rotor flux) -> rotor HOLDS, no
     spin / no voltage wall. Same R/L/gains as q, so it characterises the loop.
   - q-axis (1): Iq makes torque -> rotor spins; valid only from near standstill
     captured fast (the <1 ms current transient finishes before speed builds). */
volatile uint8_t  g_inj_override = 0U;  /* 1 = force Iqdref (injected axis = g_inj_s16, other = 0) */
volatile int16_t  g_inj_s16      = 0;   /* commanded current on the injected axis, s16 units        */
volatile uint8_t  g_step_axis    = 0U;  /* 0 = d-axis (rotor holds), 1 = q-axis (torque)            */
volatile uint8_t  g_step_mode    = 0U;  /* 0 = current-loop step (HF capture), 1 = speed-loop step (MF/1kHz) */
#define ENC_CPT_TO_RPM (((float)ISR_FREQUENCY_HZ * 60.0f) / (65536.0f * (float)POLE_PAIR_NUM))
#define ENC_HF_PER_MF  (ISR_FREQUENCY_HZ / SPEED_LOOP_FREQUENCY_HZ) /* nominal HF ticks/hook */
/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);

MCI_Handle_t *GetMCI(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);
void TSK_SafetyTask_LSON(uint8_t motor);

/* USER CODE BEGIN Private Functions */
void Ropetow_EncoderUpdate(void);   /* called from the MF hook (mc_app_hooks.c) */
void Ropetow_SetSpeedKp(int32_t kp); /* live speed-loop tuning (CDC RX handler)  */
void Ropetow_SetSpeedKi(int32_t ki);
void Ropetow_SetTorqueKp(int32_t kp); /* live Iq+Id-loop tuning (CDC RX handler) */
void Ropetow_SetTorqueKi(int32_t ki);
/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  */
__weak void FOC_Init(void)
{

  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R3_2_Init(&PWM_Handle_M1);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    ENC_Init (&ENCODER_M1);

    /******************************************************/
    /*   Main encoder alignment component initialization  */
    /******************************************************/
    EAC_Init(&EncAlignCtrlM1,pSTC[M1],&VirtualSpeedSensorM1,&ENCODER_M1);
    pEAC[M1] = &EncAlignCtrlM1;

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &ENCODER_M1._Super);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    STC_Clear(pSTC[M1]);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;

    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /* First command to STC */

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
  R3_2_SwitchOffPWM(pwmcHandle[motor]);

  FOC_Clear(motor);
  STC_Clear(pSTC[motor]);

  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
  (void)ENC_CalcAvrgMecSpeedUnit(&ENCODER_M1, &wAux);
  PQD_CalcElMotorPower(pMPM[M1]);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {

        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              /* Calibration already done. Enables only TIM channels */
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);

              R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
              {
                FOC_Clear(M1);
                STC_Clear(pSTC[M1]);
                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                Mci[M1].State = IDLE;
              }
              else
              {
                R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                Mci[M1].State = CHARGE_BOOT_CAP;
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              R3_2_SwitchOffPWM(pwmcHandle[M1]);
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );

              ENC_Clear(&ENCODER_M1);

              FOC_Clear( M1 );

              if (EAC_IsAligned(&EncAlignCtrlM1) == false)
              {
                EAC_StartAlignment(&EncAlignCtrlM1);
                Mci[M1].State = ALIGNMENT;
              }
              else
              {
                STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
                STC_SetSpeedSensor(pSTC[M1], &ENCODER_M1._Super);
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef(M1);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M1].State = RUN;
              }
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

        case ALIGNMENT:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            bool isAligned = EAC_IsAligned(&EncAlignCtrlM1);
            bool EACDone = EAC_Exec(&EncAlignCtrlM1);
            if ((isAligned == false)  && (EACDone == false))
            {
              qd_t IqdRef;
              IqdRef.q = 0;
              IqdRef.d = STC_CalcTorqueReference(pSTC[M1]);
              FOCVars[M1].Iqdref = IqdRef;
            }
            else
            {
              R3_2_SwitchOffPWM( pwmcHandle[M1] );
              STC_Clear(pSTC[M1]);
              STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
              STC_SetSpeedSensor(pSTC[M1], &ENCODER_M1._Super);
              FOC_Clear(M1);
              R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
              Mci[M1].State = WAIT_STOP_MOTOR;
              /* USER CODE BEGIN MediumFrequencyTask M1 EndOfEncAlignment */

              /* USER CODE END MediumFrequencyTask M1 EndOfEncAlignment */
            }
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

            MCI_ExecBufferedCommands(&Mci[M1]);

              FOC_CalcCurrRef(M1);
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {

            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        case WAIT_STOP_MOTOR:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_StopPermanencyTimeHasElapsedM1())
            {
              ENC_Clear(&ENCODER_M1);
              R3_2_SwitchOnPWM(pwmcHandle[M1]);
              FOC_InitAdditionalMethods(M1);
              STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
              FOC_CalcCurrRef(M1);
              Mci[M1].State = RUN;
            }
            else
            {
              /* Nothing to do */
            }
          }
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */

  ab_t NULL_ab = {((int16_t)0), ((int16_t)0)};
  qd_t NULL_qd = {((int16_t)0), ((int16_t)0)};
  alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};

  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)0));
  PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)0));

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
      /* Nothing to do */
    }
    else
    {
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference latching */
  /* to avoid MF task writing them while HF task reading them */
  __disable_irq();
  IqdTmp = FOCVars[bMotor].Iqdref;

  /* Exit critical section */
  __enable_irq();

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */
  /* Optionally feed the speed loop from the SPI absolute-angle velocity instead
     of the TIM3 quadrature deltas (finer/less laggy at low speed). This runs
     AFTER ENC_CalcAvrgMecSpeedUnit (top of the MF task) and immediately BEFORE
     STC_CalcTorqueReference (next, which reads hAvrMecSpeedUnit), so it cleanly
     overrides the speed the PI sees. g_enc_speed_rpm is published by the MF hook.
     NOTE: sign must match TIM3's (forward = positive); verify empirically. */
  if ((bMotor == M1) && (g_use_spi_speed != 0U) && (Mci[M1].State == RUN))
  {
    int32_t mec_unit = (int32_t)(g_enc_speed_rpm * (float)SPEED_UNIT / (float)U_RPM);
    if (mec_unit >  32767) { mec_unit =  32767; }
    if (mec_unit < -32768) { mec_unit = -32768; }
    ENCODER_M1._Super.hAvrMecSpeedUnit = (int16_t)mec_unit;
  }
  /* USER CODE END FOC_CalcCurrRef 0 */
  if (INTERNAL == FOCVars[bMotor].bDriveInput)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    IqdTmp.q = FOCVars[bMotor].hTeref;

  }
  else
  {
    /* Nothing to do */
  }

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference restoring */
  __disable_irq();
  FOCVars[bMotor].Iqdref = IqdTmp;

  /* Exit critical section */
  __enable_irq();
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */
  /* Step test: pin Iqref to the injected axis (other axis = 0) so the chosen
     current loop is exercised. Runs in the same ctx that just latched Iqdref, so
     overriding here is race-free. */
  if ((bMotor == M1) && (g_inj_override != 0U))
  {
    __disable_irq();
    if (g_step_axis == 0U) { FOCVars[M1].Iqdref.d = g_inj_s16; FOCVars[M1].Iqdref.q = 0; }
    else                   { FOCVars[M1].Iqdref.q = g_inj_s16; FOCVars[M1].Iqdref.d = 0; }
    __enable_irq();
  }
  /* USER CODE END FOC_CalcCurrRef 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  * @param bMotorNbr Motor reference number defined
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t hFOCreturn;
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

  RCM_ReadOngoingConv();
  RCM_ExecNextConv();

  (void)ENC_CalcAngle(&ENCODER_M1);   /* If not sensorless then 2nd parameter is MC_NULL */

  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
  hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  if(hFOCreturn == MC_DURATION)
  {
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }
  else
  {
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
  }

  return (bMotorNbr);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError = MC_NO_FAULTS;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);

  /* --- Encoder angle: extrapolate the hook's SPI estimate (Ropetow) ----------
   * The SPI read runs in Ropetow_EncoderUpdate() (MF hook, thread ctx). Here in
   * the HF ISR we only EXTRAPOLATE the published angle by the published velocity
   * (pure float -- no SPI, cannot hang). Outside RUN the MCSDK angle is left
   * untouched so the forced alignment ramp is unaffected. */
  if (Mci[M1].State == RUN)
  {
    g_hf_tick_count++;
    if (g_ol_enable != 0U)
    {
      /* Open-loop forced commutation: drive the el angle as a constant-speed ramp,
         encoder entirely out of the loop. Seed from the post-alignment angle so
         there is no torque step on RUN entry, then free-run. */
      if (g_ol_seeded == 0U) { g_ol_acc = (float)hElAngle; g_ol_seeded = 1U; }
      g_ol_acc += g_ol_step;
      while (g_ol_acc >= 32768.0f) { g_ol_acc -= 65536.0f; }
      while (g_ol_acc < -32768.0f) { g_ol_acc += 65536.0f; }
      hElAngle = (int16_t)g_ol_acc;
    }
    else if (g_use_spi_commutation != 0U)
    {
      uint32_t dt = g_hf_tick_count - g_enc_base_tick;  /* HF ticks since last publish */
      float    a;
      if (dt > (uint32_t)(4U * ENC_HF_PER_MF)) { dt = (uint32_t)(4U * ENC_HF_PER_MF); }
      a = g_enc_theta0 + (g_enc_omega_tick * (float)dt);
      while (a >= 32768.0f) { a -= 65536.0f; }
      while (a < -32768.0f) { a += 65536.0f; }
      hElAngle = (int16_t)a;
    }
  }
  else
  {
    g_ol_seeded = 0U;   /* re-seed the forced ramp on the next RUN entry */
  }

  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);

  /* Current-loop step-response capture (HF rate): injected-axis ref vs measured
     (g_step_axis: d -> Idref/Id, q -> Iqref/Iq). Speed-loop capture (g_step_mode==1)
     is done in Ropetow_EncoderUpdate at the 1 kHz MF rate instead. */
  if ((g_step_state == 1U) && (g_step_mode == 0U))
  {
    if (++g_step_dec >= (uint16_t)STEP_DECIM)
    {
      g_step_dec = 0U;
      if (g_step_idx < (uint16_t)STEP_N)
      {
        if (g_step_axis == 0U)
        { g_step_ref[g_step_idx] = FOCVars[M1].Iqdref.d; g_step_iq[g_step_idx] = Iqd.d; }
        else
        { g_step_ref[g_step_idx] = FOCVars[M1].Iqdref.q; g_step_iq[g_step_idx] = Iqd.q; }
        g_step_idx++;
      }
      if (g_step_idx >= (uint16_t)STEP_N) { g_step_state = 2U; }
    }
  }

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
    Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  }
  else
  {
    Vqd.q = 0;
    Vqd.d = 0;
  }
  Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  }
  else
  {
    /* Nothing to do. No PWM setting to prevent possible ChargeBootCap conflict */

  }

  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

  return (hCodeError);
}

/* USER CODE BEGIN mc_task 0 */
/* Encoder update (Ropetow): runs in the MF hook (thread context, where HAL SPI
   is safe). Reads the AS5047 absolute angle, captures the SPI<->motor offset vs
   the low-speed-reliable TIM3 angle, computes velocity, and publishes (angle,
   per-HF-tick velocity, speed) for the HF FOC ISR to extrapolate. */
void Ropetow_EncoderUpdate(void)
{
  static float    enc_theta      = 0.0f;
  static float    enc_vel        = 0.0f;
  static int16_t  spi_offset     = 0;
  static int16_t  spi_el_prev    = 0;
  static uint32_t hook_last_tick = 0U;
  static uint8_t  locked         = 0U;

  int16_t tim3_el;
  int16_t enc_meas;

  if (Mci[M1].State != RUN)
  {
    locked = 0U;                         /* re-lock on next RUN entry */
    g_enc_speed_rpm = 0.0f;
    return;
  }

  tim3_el  = SPD_GetElAngle(STC_GetSpeedSensor(pSTC[M1])); /* TIM3/ABI angle */
  enc_meas = tim3_el;                                      /* default source */

#if (ENC_USE_SPI_ANGLE != 0)
  if (__get_IPSR() == 0U)                /* HAL SPI is thread-context only */
  {
    AS5047_Status sst   = AS5047_OK;
    uint16_t      raw14 = AS5047_ReadAngle(&hspi1, &sst);
    if (sst == AS5047_OK)
    {
      /* INL correction in the MECHANICAL domain, BEFORE the *PP electrical
         conversion (so the eccentricity isn't multiplied 20x into el angle).
         mech = wrap16384(raw14 - sign*corr(raw14)); see INL_* in Private define. */
      uint16_t mech = raw14;
      if (g_inl_enable != 0U)
      {
        float    ang  = (float)raw14 * INL_2PI_OVER_16384;
        float    corr = (INL_C1 * cosf(ang))        + (INL_S1 * sinf(ang))
                      + (INL_C2 * cosf(2.0f * ang))  + (INL_S2 * sinf(2.0f * ang));
        int32_t  m    = (int32_t)raw14 - (int32_t)(g_inl_sign * corr);
        m %= 16384;
        if (m < 0) { m += 16384; }
        mech = (uint16_t)m;
      }

      /* mech(14b) -> electrical s16: el = mech * PP * (65536/16384) = mech*PP*4 */
      int16_t spi_el = (int16_t)((uint16_t)((uint32_t)mech *
                                            (uint32_t)(POLE_PAIR_NUM * 4)));
      int16_t spi_delta;
  #if (ENC_SPI_INVERT != 0)
      spi_el = (int16_t)(-spi_el);
  #endif
      spi_delta   = (int16_t)(spi_el - spi_el_prev);
      spi_el_prev = spi_el;
      /* refresh the constant offset while nearly stationary (TIM3 reliable) */
      if ((spi_delta < ENC_SPI_SLOW_CNT) && (spi_delta > -ENC_SPI_SLOW_CNT))
      {
        spi_offset = (int16_t)(spi_el - tim3_el);
      }
      enc_meas             = (int16_t)(spi_el - spi_offset);
      g_dbg_spi_minus_tim3 = (int16_t)(enc_meas - tim3_el);

      /* INL cal: capture the mechanical angle ACTUALLY USED for commutation at
         ~100 Hz -- raw14 when disabled (baseline) or the corrected 'mech' when
         enabled (Phase-5 residual). Same routine validates before vs after. */
      if (g_cal_state == 1U)
      {
        if (++g_cal_dec >= (uint16_t)CAL_DECIM)
        {
          g_cal_dec = 0U;
          if (g_cal_idx < (uint16_t)CAL_N) { g_cal_raw[g_cal_idx++] = mech; }
          if (g_cal_idx >= (uint16_t)CAL_N) { g_cal_state = 2U; }
        }
      }
    }
    else
    {
      g_spi_err_count++;                 /* bad read (parity/EF): rejected */
    }
  }
#endif

  /* Commutate on the RAW SPI absolute angle. enc_vel is a low-passed
     finite-difference velocity used ONLY to interpolate up to the loop rate. */
  if (locked == 0U)
  {
    enc_theta = (float)enc_meas;
    enc_vel   = 0.0f;
    locked    = 1U;
  }
  else
  {
    float d = (float)enc_meas - enc_theta;
    if (d >= 32768.0f)      { d -= 65536.0f; }
    else if (d < -32768.0f) { d += 65536.0f; }
    enc_vel   += ENC_VEL_LP * (d - enc_vel);
    enc_theta  = (float)enc_meas;
  }

  /* Publish atomically w.r.t. the FOC ISR (counts-per-HF-tick from elapsed ticks). */
  {
    uint32_t now = g_hf_tick_count;
    uint32_t dt  = now - hook_last_tick;
    float    omega_tick;
    hook_last_tick = now;
    if (dt == 0U) { dt = 1U; }
    omega_tick = enc_vel / (float)dt;

    __disable_irq();
    g_enc_theta0     = enc_theta;
    g_enc_omega_tick = omega_tick;
    g_enc_base_tick  = now;
    g_enc_seq++;
    __enable_irq();

    g_enc_speed_rpm = omega_tick * ENC_CPT_TO_RPM;
  }

  /* Speed-loop step-response capture, decimated to ~250 Hz (SPEED_CAP_DECIM of the
     1 kHz MF rate) so 512 samples span ~2 s: speed reference vs measured, in rpm. */
  if ((g_step_state == 1U) && (g_step_mode == 1U))
  {
    if (++g_spd_dec >= (uint16_t)SPEED_CAP_DECIM)
    {
      g_spd_dec = 0U;
      if (g_step_idx < (uint16_t)STEP_N)
      {
        g_step_ref[g_step_idx] =
          (int16_t)(((int32_t)STC_GetMecSpeedRefUnit(pSTC[M1]) * U_RPM) / (int32_t)SPEED_UNIT);
        g_step_iq[g_step_idx] =
          (int16_t)(((int32_t)ENCODER_M1._Super.hAvrMecSpeedUnit * U_RPM) / (int32_t)SPEED_UNIT);
        g_step_idx++;
      }
      if (g_step_idx >= (uint16_t)STEP_N) { g_step_state = 2U; }
    }
  }
}

/* Live speed-loop gain setters, called from the USB CDC RX handler (p<n>/i<n>).
   PID gains are int16; the caller already clamps to [0..32767]. A plain field
   write is atomic on M4 -- the next MF speed-loop cycle picks it up. */
void Ropetow_SetSpeedKp(int32_t kp)
{
  PID_SetKP(&PIDSpeedHandle_M1, (int16_t)kp);
}

void Ropetow_SetSpeedKi(int32_t ki)
{
  PID_SetKI(&PIDSpeedHandle_M1, (int16_t)ki);
}

/* Torque loop = Iq and Id PIs (kept identical, as the autotune sets them). */
void Ropetow_SetTorqueKp(int32_t kp)
{
  PID_SetKP(&PIDIqHandle_M1, (int16_t)kp);
  PID_SetKP(&PIDIdHandle_M1, (int16_t)kp);
}

void Ropetow_SetTorqueKi(int32_t ki)
{
  PID_SetKI(&PIDIqHandle_M1, (int16_t)ki);
  PID_SetKI(&PIDIdHandle_M1, (int16_t)ki);
}
/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
