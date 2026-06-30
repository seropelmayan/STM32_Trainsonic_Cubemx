
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
#include "flux_weakening_ctrl.h"    /* MCSDK native voltage-feedback FW component */
#include "pid_regulator.h"          /* PID_HandleInit / PID_SetKI for the FW PID  */
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
/* Commutation latency feed-forward: advance the extrapolated electrical angle by
   this many EXTRA HF ticks of velocity, to cancel the SPI-read + half-PWM apply
   latency that otherwise leaves a SPEED-PROPORTIONAL angle error (angle sampled,
   then used ~T_ff later). omega-scaled => auto-corrects at every speed. ~4 ticks
   ~= cancels ~14 deg electrical at 700 rpm (the Id->Iq leakage / buzz). Tune live
   via CDC 'G<tenths>' while watching Iq track Iqref cleanly. */
volatile float    g_enc_ff_ticks = 4.0f;
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
volatile int32_t  g_torque_set_ma = 0;  /* CDC 't<mA>' -> appTask commands torque ref */
volatile uint8_t  g_torque_set_req = 0U;
volatile int32_t  g_speed_set_rpm = 0;  /* CDC 's<rpm>' -> appTask commands SPEED ramp  */
volatile uint8_t  g_speed_set_req = 0U;
volatile uint8_t  g_iadc_log_req  = 0U; /* CDC 'j<rpm>' -> appTask logs raw per-phase currents     */
volatile int32_t  g_iadc_speed_rpm = 0; /* j arg: 0 = standstill/zero cmd, >0 = sample at this speed */
volatile uint8_t  g_restart_req   = 0U; /* CDC 'R' -> appTask acks faults + restarts motor (0 A)    */
/* ---- Position control mode (Ropetow) ----------------------------------------
   Direct position -> torque PD servo. No inner speed loop: the loop closes
   mechanical position by commanding Iq directly, and a derivative term on the
   measured speed supplies the damping (Kd) the missing speed loop would have.
       Iq[A] = Kp*(target - pos)  -  Kd*speed_rpm     (then clamped)
   Multi-turn position is accumulated from g_enc_mech14 (14-bit, 16384 counts/rev)
   by unwrapping the wrap each 1 kHz MF tick -- safe up to ~30000 rpm, far above
   the 700 rpm ceiling. Runs in Ropetow_PositionControl() from the MF hook, and
   only actuates while the drive is in RUN. A velocity clamp (g_pos_max_rpm) and a
   torque clamp (g_pos_max_iq) bound the motion so a bad target can't run away.
   Engaged live over USB CDC: 'O' toggle, 'z' hold-here/zero, 'o<deg>' set target,
   'L'/'N' set Kp/Kd, 'M' max Iq (mA), 'Q' max rpm. All CDC writes are just
   volatile stores / request flags -- every STC_* call happens in the MF thread. */
#define POS_COUNTS_PER_REV   16384       /* g_enc_mech14 is 14-bit (0..16383)        */
#define POS_IQ_SIGN          (-1.0f)     /* maps +Iq (torque) to the +g_pos_counts direction.
                                            On THIS drive +Iq DECREASES the count (verified: a
                                            -83 deg error drove -max Iq and the count ran AWAY
                                            positive), so it is inverted. Flip if a future
                                            encoder/TIM3 sign change makes the servo run away. */
#define POS_KP_DEFAULT       0.0020f     /* A per count   (~0.9 A at 10 deg error)   */
#define POS_KD_DEFAULT       0.0015f     /* A per rpm     (damping)                  */
#define POS_MAX_IQ_DEFAULT   3.0f        /* |Iq| clamp, A                            */
#define POS_MAX_RPM_DEFAULT  150.0f      /* velocity clamp, rpm                      */
volatile uint8_t  g_pos_mode       = 0U; /* 1 = position servo active (actuating)   */
volatile int32_t  g_pos_counts     = 0;  /* measured multi-turn position, counts     */
volatile int32_t  g_pos_origin     = 0;  /* reference zero for 'o<deg>' targets      */
volatile int32_t  g_pos_target     = 0;  /* commanded position, counts               */
volatile float    g_pos_kp         = POS_KP_DEFAULT;
volatile float    g_pos_kd         = POS_KD_DEFAULT;
volatile float    g_pos_max_iq     = POS_MAX_IQ_DEFAULT;
volatile float    g_pos_max_rpm    = POS_MAX_RPM_DEFAULT;
volatile uint8_t  g_pos_toggle_req = 0U; /* CDC 'O': enable/disable (handled in MF)  */
volatile uint8_t  g_pos_zero_req   = 0U; /* CDC 'z': set target=origin=current pos   */
volatile int32_t  g_pos_set_deg    = 0;  /* CDC 'o<deg>': pending target, deg from origin */
volatile uint8_t  g_pos_set_req    = 0U;
static   uint16_t g_step_dec     = 0U;
static   uint16_t g_spd_dec      = 0U;
volatile uint16_t g_cap_decim    = STEP_DECIM; /* HF-capture decimation (1=25kHz step; larger for Iq-ripple over revs) */
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
/* Dead-time compensation: adds +sign(Iphase)*g_dt_comp (s16 V) per phase after the
   inverse-Park, to cancel the 6x-electrical voltage distortion dead-time causes
   (the ripple ODrive compensates and we didn't). 0 = off; set live via 'D<n>',
   sweep against a 'g' ripple capture. Flip the code sign if ripple INCREASES. */
volatile int16_t  g_dt_comp      = 45;  /* tuned: D45 minimized the 6x & 1x electrical lines */
/* First-order LPF on the MEASURED Iq/Id before the PI: cuts current-sense noise
   the loop would otherwise amplify into voltage thrash. alpha=1.0 -> off (raw).
   Set live via 'f<cutoff_Hz>'. y += alpha*(x - y), alpha = 2*pi*fc/ISR_FREQ. */
volatile float    g_iq_lpf_alpha = 1.0f;
/* Running average of Iq/Id over ~0.3 s (many electrical revs) -> the 50Hz AC
   averages to ~0, leaving the DC component. mean Id != 0 => commutation
   MISALIGNED (encoder offset); mean Id ~0 with 50Hz ripple => current offset. */
volatile int16_t  g_avg_iq = 0;
volatile int16_t  g_avg_id = 0;
/* Live velocity-smoothing coefficient on the SPI-derived speed estimate (used by
   the speed loop in 'v' mode and by the HF commutation interpolation). alpha in
   (0..1]: 1 = raw (no smoothing), smaller = heavier LPF (less noise, MORE lag).
   Set live via 'F<cutoff_Hz>' (filter runs at the 1 kHz MF rate). */
volatile float    g_enc_vel_lp = ENC_VEL_LP;
/* ---- Anti-cogging (Ropetow) -------------------------------------------------
   Position-indexed Iq feed-forward that cancels cogging / low-speed torque ripple.
   The table is captured by a both-direction slow sweep (host cogging.py combines
   fwd+rev), compiled in via cogg_table.h, and applied in FOC_CalcCurrRef (MF,
   guarded). Gated by g_cogg_enable (default OFF) and clamped by g_cogg_clamp so a
   bad table can never command large current. See the plan / cogg_table.h. */
#define COGG_DEFINE_TABLE
#include "cogg_table.h"
int16_t           g_cogg_lut[COGG_NBINS];        /* runtime FF table (copied from COGG_TABLE_INIT at boot) */
volatile uint8_t  g_cogg_enable = 0U;            /* 0 = off (safe default); CDC 'K'/'k'        */
volatile int16_t  g_cogg_clamp  = 800;           /* max |FF|, s16 current units (~0.4 A); CDC 'C<n>' */
volatile uint16_t g_enc_mech14  = 0U;            /* latest mechanical angle 0..16383 (published by MF hook) */
/* Calibration accumulators: filled in the MF hook, dumped in appTask (both thread
   context, so plain statics). 512*(4+2)=3 KB. */
static   int32_t  g_cogg_acc[COGG_NBINS];
static   uint16_t g_cogg_cnt[COGG_NBINS];
volatile uint8_t  g_cogg_cal_req   = 0U;         /* CDC 'y' -> appTask clears & arms           */
volatile uint8_t  g_cogg_cal_state = 0U;         /* 0 idle, 1 capturing, 2 ready-to-dump       */
/* AGC-vs-position sweep (Phase 0 magnet diagnostic): CDC 'a' -> appTask logs
   AS5047 AGC/MAGL/MAGH + angle across a slow revolution. */
volatile uint8_t  g_agc_log_req = 0U;
/* Live raw-encoder monitor: CDC 'r' toggles a compact per-iteration angle/AGC/flag
   readout in appTask (motor STOPPED -- turn the shaft by hand). For "is the encoder
   sane?" checks: watch raw track smoothly 0..16383 and flags stay clear. */
volatile uint8_t  g_enc_mon = 0U;
#define ENC_CPT_TO_RPM (((float)ISR_FREQUENCY_HZ * 60.0f) / (65536.0f * (float)POLE_PAIR_NUM))
#define ENC_HF_PER_MF  (ISR_FREQUENCY_HZ / SPEED_LOOP_FREQUENCY_HZ) /* nominal HF ticks/hook */
/* ---- Flux weakening (Ropetow) ----------------------------------------------
   Simple speed-gated d-axis (field-weakening) current injection to push past the
   ~600 rpm voltage wall (at 48 V the back-EMF meets the SVPWM voltage ceiling, so
   speed mode buzzes/clips). When |mech speed| exceeds g_fw_speed_thr_rpm, Id is
   slewed toward g_fw_id_target_a (negative = weakening, frees voltage headroom);
   below the threshold Id slews back to 0. This is an OPEN-LOOP fixed target, NOT
   ST's voltage-feedback FW PI -- start small (-1 A) and raise while watching Vqd
   headroom in the status log. Applied in FOC_CalcCurrRef (MF, guarded). Tune live
   over USB CDC: 'w' toggle, 'W<rpm>' threshold, 'J<mA>' weakening magnitude. */
volatile uint8_t  g_fw_enable        = 1U;      /* 1 = flux weakening active (CDC 'w')        */
volatile float    g_fw_speed_thr_rpm = 525.0f;  /* |mech rpm| above which FW engages (CDC 'W')*/
volatile float    g_fw_hyst_rpm      = 50.0f;   /* engage hysteresis: drop out below thr-hyst (CDC 'H') */
volatile float    g_fw_id_target_a   = -1.0f;   /* d-axis current target, A, negative (CDC 'J')*/
volatile float    g_fw_id_slew_a_s   = 5.0f;    /* Id slew rate, A/s -- limits the step       */
volatile float    g_fw_id_now_a      = 0.0f;    /* slewed Id ACTUALLY applied, A (0 = FW idle) */
static   uint8_t  g_fw_engaged       = 0U;      /* latch: 1 once over thr, 0 once below thr-hyst */
/* ---- Torque-mode speed cap / governor (Ropetow) -----------------------------
   Caps top speed by rolling Iqref toward 0 as |mech speed| approaches the cap, so
   the drive LEVELS OFF short of the MAX_APPLICATION_SPEED (700 rpm) over-speed
   fault instead of tripping it. Primarily for torque mode (no speed loop), but it
   runs in any mode as a backstop -- harmless in speed mode as long as the cap is
   above the commanded speed. Only the ACCELERATING torque is rolled off (q and
   speed same sign); braking torque always passes so you can decelerate. Uses the
   MCSDK average speed (hAvrMecSpeedUnit), whose sign matches Iqref.q. The
   over-speed fault is intentionally LEFT ENABLED as a hard backstop below this.
   0 = cap disabled. Set live via CDC 'V<rpm>'. */
volatile float    g_spdcap_rpm       = 500.0f; /* speed cap, rpm (0 = off); keep < 700 fault */
#define SPDCAP_BAND_RPM   40.0f                 /* roll-off band below the cap (rpm)          */
/* ---- MCSDK native flux weakening (voltage-feedback) -- hand-instantiated -----
   The project was generated without FW, so ST's FW_Handle (flux_weakening_ctrl.c)
   is wired up here by hand. It is the ADAPTIVE voltage-feedback FW: a PI on the
   error (FW_V_Ref%-of-MaxModule  -  |Vqd|) whose output drives Id negative once
   the voltage saturates. Safe in our LOW-torque torque mode: FW_CalcCurrRef only
   clamps Iq via the current circle sqrt(Inom^2 - Id^2), which never bites at
   ~0.5 A; and its pSpeedPID integral-limit writes are harmless on the (idle)
   speed PID. Hard floor = hDemagCurrent. Default OFF; enable with CDC 'x', tune
   FW_V_Ref with 'A<tenths-%>' and the FW PI Ki with 'B<n>'. When ON it REPLACES
   the custom g_fw_ block. */
#define FW_V_REF_DEFAULT   950U      /* target |Vqd| as tenths-of-% of MaxModule (950 = 95%) */
#define FW_DEMAG_A         8.0f      /* hard floor on FW |Id| (A); motor takes 29A, tune via 'S<mA>' */
#define FW_VQD_BWLOG       4U        /* Vqd 1st-order LPF: 2^4 = 16                           */
#define FW_PID_KP_DEFAULT  0         /* FW PI: start integral-only (Kp=0), tune live          */
#define FW_PID_KI_DEFAULT  200       /* FW PI Ki -- conservative start; raise via 'B<n>'      */
PID_Handle_t PIDFluxWeakeningHandle_M1 =
{
  .hDefKpGain          = (int16_t)FW_PID_KP_DEFAULT,
  .hDefKiGain          = (int16_t)FW_PID_KI_DEFAULT,
  .wUpperIntegralLimit = 0,                  /* FW only weakens: NO positive windup (anti-windup) */
  .wLowerIntegralLimit = -(int32_t)INT16_MAX * (int32_t)TF_KIDIV,
  .hUpperOutputLimit   = 0,                  /* output <= 0 => weaken-or-nothing; can't wind up at low speed */
  .hLowerOutputLimit   = -INT16_MAX,         /* hDemagCurrent is the real floor  */
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain = 0, .hKdDivisor = 0, .hKdDivisorPOW2 = 0,
};
FW_Handle_t FW_M1 =
{
  .hMaxModule             = MAX_MODULE,
  .hFW_V_Ref              = FW_V_REF_DEFAULT,
  .hDefaultFW_V_Ref       = FW_V_REF_DEFAULT,
  .hDemagCurrent          = -(int16_t)(FW_DEMAG_A * (float)CURRENT_CONV_FACTOR),
  .wNominalSqCurr         = (int32_t)NOMINAL_CURRENT * (int32_t)NOMINAL_CURRENT,
  .hVqdLowPassFilterBW    = (uint16_t)(1U << FW_VQD_BWLOG),
  .hVqdLowPassFilterBWLOG = (uint16_t)FW_VQD_BWLOG,
};
volatile uint8_t  g_mcfw_enable = 0U;   /* CDC 'x': 1 = use MCSDK native FW (replaces custom g_fw_) */
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
void Ropetow_SetEncVelLp(int32_t fc_hz); /* live velocity-smoothing cutoff (CDC 'F<n>') */
void Ropetow_CoggInit(void);          /* copy COGG_TABLE_INIT -> g_cogg_lut (MC_APP_BootHook) */
void Ropetow_CoggCalArm(void);        /* clear accumulators + arm capture (appTask, CDC 'y') */
void Ropetow_CoggCalGet(uint16_t bin, int16_t *mean, uint16_t *count); /* read a cal bin (dump) */
void Ropetow_GetOffsets(int32_t *a, int32_t *b, int32_t *c); /* calibrated 3-shunt offsets */
void Ropetow_SetCurrentLpf(int32_t fc_hz); /* live current LPF cutoff (CDC 'f<n>') */
void Ropetow_PositionControl(void);   /* position->torque PD servo (MF hook, after EncoderUpdate) */
void Ropetow_McFwInit(void);          /* init MCSDK native flux weakening (called from BootHook)   */
void Ropetow_SetMcFwVRef(int32_t v);  /* FW target voltage, tenths-of-% (CDC 'A<n>')               */
void Ropetow_SetMcFwKi(int32_t ki);   /* FW PI Ki (CDC 'B<n>')                                     */
void Ropetow_SetMcFwDemag(int32_t ma);/* FW demag |Id| clamp, mA (CDC 'S<n>')                      */
int16_t Ropetow_McFwAvVolt(void);     /* filtered |Vqd| the FW PI regulates (telemetry)            */
int16_t Ropetow_McFwVTarget(void);    /* FW target voltage module (telemetry)                      */
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

  /* Anti-cogging feed-forward: add the position-indexed Iq term to the freshly
     latched reference. Skipped during the step-test override. Gated by
     g_cogg_enable (default OFF) and clamped by g_cogg_clamp. Linear interpolation
     between bins. g_enc_mech14 is the latest mechanical angle from the MF hook
     (<=1 ms stale -> negligible at the low speeds where cogging is felt). */
  if ((bMotor == M1) && (g_inj_override == 0U) && (g_cogg_enable != 0U) &&
      (Mci[M1].State == RUN))
  {
    uint16_t m    = g_enc_mech14;
    uint16_t bin  = (uint16_t)(m >> COGG_SHIFT);
    uint16_t nb   = (uint16_t)((bin + 1U) & (uint16_t)(COGG_NBINS - 1U));
    int32_t  frac = (int32_t)(m & (uint16_t)((1U << COGG_SHIFT) - 1U));   /* 0..31 */
    int32_t  ff   = (int32_t)g_cogg_lut[bin] +
                    ((((int32_t)g_cogg_lut[nb] - (int32_t)g_cogg_lut[bin]) * frac) >> COGG_SHIFT);
    if (ff >  (int32_t)g_cogg_clamp) { ff =  (int32_t)g_cogg_clamp; }
    if (ff < -(int32_t)g_cogg_clamp) { ff = -(int32_t)g_cogg_clamp; }
    __disable_irq();
    FOCVars[M1].Iqdref.q = (int16_t)((int32_t)FOCVars[M1].Iqdref.q + ff);
    __enable_irq();
  }

  /* Flux weakening: speed-gated d-axis current injection with hysteresis. While
     RUNNING, FW ENGAGES once |mech speed| rises above g_fw_speed_thr_rpm and
     DISENGAGES only once it falls below (thr - g_fw_hyst_rpm). The latch stops
     the on/off chatter (and the resulting big Id spikes) when speed oscillates
     right around the threshold. When engaged, slew Id toward g_fw_id_target_a
     (negative weakens the field, freeing voltage headroom above the ~600 rpm
     wall); otherwise slew back to 0. The slew (g_fw_id_slew_a_s, A/s at the MF
     rate) keeps the transition from stepping the current loop. Runs in the same
     ctx that just latched Iqdref, so writing Iqdref.d is race-free. Acts on the
     d axis only -> independent of the anti-cogging q-axis FF above. Skipped
     during the step-test override (which owns both axes). */
  /* MCSDK native flux weakening (voltage-feedback). When enabled it REPLACES the
     custom g_fw_ block below: feed the last-cycle Vqd into the FW PI, then let
     FW_CalcCurrRef drive Iqdref.d negative (and circle-clamp Iqdref.q). Runs in
     the same ctx that just latched Iqdref -> the critical section makes it
     race-free. Skipped during the step-test override. */
  {
    static uint8_t s_mcfw_prev = 0U;   /* tracks the 0->1 enable / RUN-entry edge */
    if ((bMotor == M1) && (g_mcfw_enable != 0U) && (g_inj_override == 0U) &&
        (Mci[M1].State == RUN))
    {
      if (s_mcfw_prev == 0U) { FW_Clear(&FW_M1); }  /* reset FW integral + Vqd filter on (re)enable */
      s_mcfw_prev = 1U;
      FW_DataProcess(&FW_M1, FOCVars[M1].Vqd);
      __disable_irq();
      FOCVars[M1].Iqdref = FW_CalcCurrRef(&FW_M1, FOCVars[M1].Iqdref);
      __enable_irq();
    }
    else
    {
      s_mcfw_prev = 0U;   /* disabled / stopped / step-test -> re-arm the FW_Clear */
    }
  }

  if ((bMotor == M1) && (g_inj_override == 0U) && (g_mcfw_enable == 0U))
  {
    if (Mci[M1].State == RUN)
    {
      float spd = fabsf(g_enc_speed_rpm);
      if (g_fw_engaged == 0U) { if (spd > g_fw_speed_thr_rpm) { g_fw_engaged = 1U; } }
      else { if (spd < (g_fw_speed_thr_rpm - g_fw_hyst_rpm)) { g_fw_engaged = 0U; } }

      float target_a = ((g_fw_enable != 0U) && (g_fw_engaged != 0U))
                       ? g_fw_id_target_a : 0.0f;
      float step = g_fw_id_slew_a_s / (float)SPEED_LOOP_FREQUENCY_HZ;
      if      (g_fw_id_now_a < (target_a - step)) { g_fw_id_now_a += step; }
      else if (g_fw_id_now_a > (target_a + step)) { g_fw_id_now_a -= step; }
      else                                        { g_fw_id_now_a  = target_a; }
      __disable_irq();
      FOCVars[M1].Iqdref.d = (int16_t)(g_fw_id_now_a * (float)CURRENT_CONV_FACTOR);
      __enable_irq();
    }
    else
    {
      g_fw_id_now_a = 0.0f;   /* re-start from 0 on the next RUN */
      g_fw_engaged = 0U;      /* and re-arm the hysteresis latch */
    }
  }

  /* Torque-mode speed cap (governor): roll the q-axis reference toward 0 as
     |speed| approaches g_spdcap_rpm, so the drive plateaus BELOW the
     MAX_APPLICATION_SPEED over-speed fault rather than tripping it. Only the
     ACCELERATING component is limited (q and speed same sign) -- braking torque
     passes through so deceleration always works. Uses the MCSDK average speed
     (hAvrMecSpeedUnit) so its sign is consistent with Iqref.q. Runs LAST so it
     caps the final q reference (incl. anti-cogging FF). Skipped during the
     step-test override. */
  if ((bMotor == M1) && (g_inj_override == 0U) && (g_spdcap_rpm > 0.0f) &&
      (Mci[M1].State == RUN))
  {
    int32_t spd_rpm = ((int32_t)ENCODER_M1._Super.hAvrMecSpeedUnit * U_RPM) / SPEED_UNIT;
    int32_t aspd    = (spd_rpm < 0) ? -spd_rpm : spd_rpm;
    if ((float)aspd > (g_spdcap_rpm - SPDCAP_BAND_RPM))
    {
      float f = (g_spdcap_rpm - (float)aspd) / SPDCAP_BAND_RPM;  /* 1 -> 0 across band */
      if (f < 0.0f) { f = 0.0f; }
      if (f > 1.0f) { f = 1.0f; }
      int16_t q = FOCVars[M1].Iqdref.q;
      if (((int32_t)q * spd_rpm) > 0)        /* torque is accelerating |speed| */
      {
        __disable_irq();
        FOCVars[M1].Iqdref.q = (int16_t)((float)q * f);
        __enable_irq();
      }
    }
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
      a = g_enc_theta0 + (g_enc_omega_tick * ((float)dt + g_enc_ff_ticks)); /* +latency FF */
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

  /* Optional first-order LPF on measured Iq/Id before the loop sees them
     (g_iq_lpf_alpha < 1). Reduces sense noise -> less voltage thrash; costs a
     little phase lag (tolerable now that the current loop is detuned). */
  if (g_iq_lpf_alpha < 1.0f)
  {
    static float lpf_q = 0.0f, lpf_d = 0.0f;
    if (Mci[M1].State != RUN) { lpf_q = (float)Iqd.q; lpf_d = (float)Iqd.d; }
    lpf_q += g_iq_lpf_alpha * ((float)Iqd.q - lpf_q);
    lpf_d += g_iq_lpf_alpha * ((float)Iqd.d - lpf_d);
    Iqd.q = (int16_t)lpf_q;
    Iqd.d = (int16_t)lpf_d;
  }

  /* Alignment / offset diagnostic: average Iq,Id over ~0.33 s (8192 ticks ~= 16
     elec revs at 150 rpm) so the 50Hz AC cancels and the DC shows. */
  if (Mci[M1].State == RUN)
  {
    static int32_t  acc_q = 0, acc_d = 0;
    static uint16_t acc_n = 0U;
    acc_q += (int32_t)Iqd.q;
    acc_d += (int32_t)Iqd.d;
    if (++acc_n >= 8192U)
    {
      g_avg_iq = (int16_t)(acc_q / 8192);
      g_avg_id = (int16_t)(acc_d / 8192);
      acc_q = 0; acc_d = 0; acc_n = 0U;
    }
  }

  /* Current-loop step-response capture (HF rate): injected-axis ref vs measured
     (g_step_axis: d -> Idref/Id, q -> Iqref/Iq). Speed-loop capture (g_step_mode==1)
     is done in Ropetow_EncoderUpdate at the 1 kHz MF rate instead. */
  if ((g_step_state == 1U) && (g_step_mode == 0U))
  {
    if (++g_step_dec >= g_cap_decim)
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

  /* Dead-time compensation: add +sign(Iphase)*g_dt_comp per phase (Clarke'd into
     alpha/beta) to restore the volt-seconds dead-time eats. Cancels the 6x-elec
     ripple. Off when g_dt_comp==0. (2-input Clarke approx; magnitude tuned live.) */
  if (g_dt_comp != 0)
  {
    ab_t        dtab;
    alphabeta_t dtab2;
    dtab.a = (Iab.a >= 0) ? g_dt_comp : (int16_t)(-g_dt_comp);
    dtab.b = (Iab.b >= 0) ? g_dt_comp : (int16_t)(-g_dt_comp);
    dtab2  = MCM_Clarke(dtab);
    Valphabeta.alpha = (int16_t)(Valphabeta.alpha + dtab2.alpha);
    Valphabeta.beta  = (int16_t)(Valphabeta.beta  + dtab2.beta);
  }

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

      /* Anti-cogging: publish the mechanical angle (consumed by the FF lookup in
         FOC_CalcCurrRef) and, while calibrating, bin the measured Iq the loop
         fights by mechanical position. FOCVars[M1].Iqd.q is the latest measured Iq
         (stored by the HF loop each tick). Sweep both directions on the host. */
      g_enc_mech14 = mech;
      if (g_cogg_cal_state == 1U)
      {
        uint16_t cbin = (uint16_t)(mech >> COGG_SHIFT);
        if (cbin < (uint16_t)COGG_NBINS)
        {
          g_cogg_acc[cbin] += (int32_t)FOCVars[M1].Iqd.q;
          if (g_cogg_cnt[cbin] < 0xFFFFU) { g_cogg_cnt[cbin]++; }
        }
      }

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
    enc_vel   += g_enc_vel_lp * (d - enc_vel);
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

/* MCSDK native flux-weakening init -- called once from MC_APP_BootHook. Sets up
   the FW PI working gains from defaults and binds the FW component to the speed
   PID (used only for its integral-limit writes, harmless in torque mode). */
void Ropetow_McFwInit(void)
{
  PID_HandleInit(&PIDFluxWeakeningHandle_M1);
  FW_Init(&FW_M1, &PIDSpeedHandle_M1, &PIDFluxWeakeningHandle_M1);
  FW_Clear(&FW_M1);
}

void Ropetow_SetMcFwVRef(int32_t v)
{
  if (v < 0)    { v = 0; }
  if (v > 1000) { v = 1000; }            /* tenths-of-% : 0..100.0% */
  FW_M1.hFW_V_Ref = (uint16_t)v;
}

void Ropetow_SetMcFwKi(int32_t ki)
{
  if (ki < 0)     { ki = 0; }
  if (ki > 32767) { ki = 32767; }
  PID_SetKI(&PIDFluxWeakeningHandle_M1, (int16_t)ki);
}

/* FW demag clamp: max |Id| the flux-weakening loop may command, in mA. Bounds the
   weakening headroom (and is the demag safety floor). Keep < the motor/FET limit. */
void Ropetow_SetMcFwDemag(int32_t ma)
{
  if (ma < 0) { ma = 0; }
  /* Convert mA -> s16A and HARD-clamp to int16 range BEFORE negating, or the
     cast wraps and the demag "floor" flips POSITIVE (commands +Id = field
     strengthening / runaway current). Max |Id| in s16A = 32767 (~16.5 A at the
     ICS/3-shunt gain), which is also the current-sense saturation limit. */
  int32_t s16 = (int32_t)(((float)ma / 1000.0f) * (float)CURRENT_CONV_FACTOR);
  if (s16 > INT16_MAX) { s16 = INT16_MAX; }
  FW_M1.hDemagCurrent = (int16_t)(-s16);
}

/* FW telemetry: filtered |Vqd| the FW PI regulates, and its target module
   (hFW_V_Ref% of MaxModule). When FW engages, avV tracks the target. */
int16_t Ropetow_McFwAvVolt(void)  { return FW_M1.AvVoltAmpl; }
int16_t Ropetow_McFwVTarget(void) { return (int16_t)(((uint32_t)FW_M1.hFW_V_Ref * FW_M1.hMaxModule) / 1000U); }

/* Position->torque PD servo. Called every MF tick (1 kHz) from the MF hook,
   right AFTER Ropetow_EncoderUpdate() has published a fresh g_enc_mech14 and
   g_enc_speed_rpm. Runs in the medium-frequency THREAD context, so it is safe to
   call STC_* here (unlike the CDC IRQ, which only sets the request flags below).
   Position is always accumulated so a target captured on enable has no jump; the
   PD law only drives the motor while g_pos_mode is set and the drive is in RUN. */
void Ropetow_PositionControl(void)
{
  /* --- multi-turn position accumulation (unwrap the 14-bit angle) --- */
  static uint8_t  seeded   = 0U;
  static uint16_t prev_raw = 0U;
  static float    vel_rpm  = 0.0f;        /* sign-locked to g_pos_counts (see below) */
  uint16_t raw = g_enc_mech14;
  if (seeded == 0U) { prev_raw = raw; seeded = 1U; }
  int32_t d = (int32_t)raw - (int32_t)prev_raw;
  if (d >  (POS_COUNTS_PER_REV / 2)) { d -= POS_COUNTS_PER_REV; }   /* wrapped down->up */
  if (d < -(POS_COUNTS_PER_REV / 2)) { d += POS_COUNTS_PER_REV; }   /* wrapped up->down */
  g_pos_counts += d;
  prev_raw = raw;

  /* Velocity for damping + the clamp, derived from the SAME per-tick delta as the
     position (counts/tick @ 1 kHz -> rpm). Deriving it here -- rather than reading
     g_enc_speed_rpm -- guarantees the D term and the velocity clamp share the P
     term's sign convention, so the damping can never become positive feedback no
     matter how the encoder/TIM3 sign is set. Lightly LPF'd (alpha 0.2 ~ 35 Hz at
     1 kHz) to smooth the 1-count quantization at low speed. */
  {
    float vel_inst = (float)d * (60000.0f / (float)POS_COUNTS_PER_REV);
    vel_rpm += 0.2f * (vel_inst - vel_rpm);
  }

  /* --- service deferred CDC requests (thread ctx -> STC_* calls are safe) --- */
  if (g_pos_zero_req != 0U)
  {
    g_pos_zero_req = 0U;
    g_pos_origin = g_pos_counts;
    g_pos_target = g_pos_counts;            /* hold right here, no motion */
  }
  if (g_pos_set_req != 0U)
  {
    g_pos_set_req = 0U;
    g_pos_target = g_pos_origin +
      (int32_t)(((int64_t)g_pos_set_deg * POS_COUNTS_PER_REV) / 360);
  }
  if (g_pos_toggle_req != 0U)
  {
    g_pos_toggle_req = 0U;
    if (g_pos_mode == 0U)
    {
      /* engage only from RUN: capture current spot as the target so the servo
         starts at zero error (no lurch), then take over torque control. */
      if (Mci[M1].State == RUN)
      {
        g_pos_origin = g_pos_counts;
        g_pos_target = g_pos_counts;
        STC_SetControlMode(pSTC[M1], MCM_TORQUE_MODE);
        g_pos_mode = 1U;
      }
    }
    else
    {
      g_pos_mode = 0U;
      STC_ExecRamp(pSTC[M1], 0, 0U);        /* release torque; user resumes via s/t */
    }
  }

  if ((g_pos_mode == 0U) || (Mci[M1].State != RUN)) { return; }

  /* --- PD law, computed in POSITION-COUNT space (where +u means "increase the
     count"), then sign-mapped to torque at the very end. Doing the clamps here in
     count space keeps the velocity backstop correct regardless of the torque
     sign. --- */
  int32_t err = g_pos_target - g_pos_counts;
  float   u   = (g_pos_kp * (float)err) - (g_pos_kd * vel_rpm);

  /* velocity clamp: if already over-speed in one direction, don't add effort that
     pushes the count further that way (the D term brakes; this is the backstop). */
  if ((vel_rpm >  g_pos_max_rpm) && (u > 0.0f)) { u = 0.0f; }
  if ((vel_rpm < -g_pos_max_rpm) && (u < 0.0f)) { u = 0.0f; }

  /* magnitude clamp (|sign| = 1, so this bounds |Iq| too) */
  if (u >  g_pos_max_iq) { u =  g_pos_max_iq; }
  if (u < -g_pos_max_iq) { u = -g_pos_max_iq; }

  /* map count-space effort -> torque command */
  float iq = POS_IQ_SIGN * u;
  STC_ExecRamp(pSTC[M1], (int16_t)(iq * (float)CURRENT_CONV_FACTOR), 0U);
}

/* Calibrated three-shunt ADC offsets (set by MCSDK at boot). A phase whose offset
   differs from the others reads a DC current bias -> 1x-electrical Iq ripple. */
void Ropetow_GetOffsets(int32_t *a, int32_t *b, int32_t *c)
{
  PWMC_R3_2_Handle_t *h = (PWMC_R3_2_Handle_t *)pwmcHandle[M1];
  *a = (int32_t)h->PhaseAOffset;
  *b = (int32_t)h->PhaseBOffset;
  *c = (int32_t)h->PhaseCOffset;
}

/* Current LPF cutoff (Hz). fc<=0 -> off (alpha=1, raw). alpha = 2*pi*fc/ISR_FREQ. */
void Ropetow_SetCurrentLpf(int32_t fc_hz)
{
  if (fc_hz <= 0)
  {
    g_iq_lpf_alpha = 1.0f;
  }
  else
  {
    float a = (2.0f * 3.14159265f * (float)fc_hz) / (float)ISR_FREQUENCY_HZ;
    if (a > 1.0f)   { a = 1.0f; }
    if (a < 0.002f) { a = 0.002f; }
    g_iq_lpf_alpha = a;
  }
}

/* Velocity-smoothing cutoff (Hz) on the SPI speed estimate. fc<=0 -> raw (alpha=1).
   alpha = 2*pi*fc/SPEED_LOOP_FREQUENCY_HZ (the enc_vel filter runs at the MF rate).
   Heavier smoothing (lower fc) cuts speed noise but ADDS lag -> watch for worse
   hunting. */
void Ropetow_SetEncVelLp(int32_t fc_hz)
{
  if (fc_hz <= 0)
  {
    g_enc_vel_lp = 1.0f;
  }
  else
  {
    float a = (2.0f * 3.14159265f * (float)fc_hz) / (float)SPEED_LOOP_FREQUENCY_HZ;
    if (a > 1.0f)   { a = 1.0f; }
    if (a < 0.002f) { a = 0.002f; }
    g_enc_vel_lp = a;
  }
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

/* Copy the compiled anti-cogging table into the runtime LUT. Called from
   MC_APP_BootHook so a regenerated/zeroed table simply means "no compensation". */
void Ropetow_CoggInit(void)
{
  for (uint16_t i = 0U; i < (uint16_t)COGG_NBINS; i++) { g_cogg_lut[i] = COGG_TABLE_INIT[i]; }
}

/* Clear the calibration accumulators and arm a capture (thread context: 512-entry
   clear is too long for the USB IRQ, so CDC 'y' only sets g_cogg_cal_req and
   appTask calls this). */
void Ropetow_CoggCalArm(void)
{
  for (uint16_t i = 0U; i < (uint16_t)COGG_NBINS; i++) { g_cogg_acc[i] = 0; g_cogg_cnt[i] = 0U; }
  g_cogg_cal_state = 1U;
}

/* Read one calibration bin: mean measured-Iq (s16) and sample count. Returns 0
   mean for empty bins. Used by the appTask dump. */
void Ropetow_CoggCalGet(uint16_t bin, int16_t *mean, uint16_t *count)
{
  if (bin >= (uint16_t)COGG_NBINS) { *mean = 0; *count = 0U; return; }
  *count = g_cogg_cnt[bin];
  *mean  = (g_cogg_cnt[bin] != 0U) ? (int16_t)(g_cogg_acc[bin] / (int32_t)g_cogg_cnt[bin]) : (int16_t)0;
}
/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
