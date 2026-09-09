
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM           1200 /*!< rpm, mechanical. 700->1200 (hand-edit; REDO IN MC WORKBENCH on regen):
                                                 only bounds the speed REFERENCE a ramp may program (CHECK_BOUNDARY in
                                                 speed_torq_ctrl.c silently rejects anything above it). The speed-window
                                                 control programs references up to g_spdcap_hard_rpm; the over-speed
                                                 FAULT is the separate 1400 rpm line in mc_config_common.c. */
#define MIN_APPLICATION_SPEED_RPM           0 /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS     16 /*!< 3->16: tolerate occasional ABI/EMI speed-read glitches (enc=BAD) instead of faulting. Set in MC Workbench too to survive regen. */

/*** Encoder **********************/
#define ENC_AVERAGING_FIFO_DEPTH            16 /*!< MUST be <= ENC_SPEED_ARRAY_SIZE (16) -- the encoder's DeltaCapturesBuffer is a fixed 16-element array; 32 overran it -> HardFault boot loop. 16 is the max. */

/* USER CODE BEGIN angle reconstruction M1 */
#define PARK_ANGLE_COMPENSATION_FACTOR      0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR  0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                       25000
#define PWM_FREQ_SCALING                    1
#define LOW_SIDE_SIGNALS_ENABLING           LS_PWM_TIMER
#define SW_DEADTIME_NS                      100 /*!< Dead-time to be inserted by FW, only if low side signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE           1 /*!< FOC execution rate in number of PWM cycles */
#define ISR_FREQUENCY_HZ                    (PWM_FREQUENCY/REGULATION_EXECUTION_RATE) /*!< @brief FOC execution rate in Hz */

/* Gains values for torque and flux control loops */
/* Workbench autotune values (for AMPLIFICATION_GAIN=20 / LS=1.4 mH). Restored
   after the current-sense polarity fix in r3_2_g4xx_pwm_curr_fdbk.c -- the /8
   diagnostic detune is no longer needed now the loop has negative feedback. */
#define PID_TORQUE_KP_DEFAULT               1000 /* bench-tuned: P1000/I1000 smoothest (loop was too hot at 3688; calmer = less voltage thrash). Iq Kp */
#define PID_TORQUE_KI_DEFAULT               1000 /* Iq Ki */
#define PID_TORQUE_KD_DEFAULT               100
#define PID_FLUX_KP_DEFAULT                 1000 /* Id Kp -- 'P' sets BOTH Iq+Id, so match torque */
#define PID_FLUX_KI_DEFAULT                 1000 /* Id Ki -- 'I' sets BOTH Iq+Id */
#define PID_FLUX_KD_DEFAULT                 100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                            512
#define TF_KIDIV                            16384
#define TF_KDDIV                            8192
#define TF_KPDIV_LOG                        LOG2((512))
#define TF_KIDIV_LOG                        LOG2((16384))
#define TF_KDDIV_LOG                        LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING        DISABLE

#define PID_SPEED_KP_DEFAULT                312/(SPEED_UNIT/10) /* = the old 10000 @ SP_KPDIV 512, rescaled for SP_KPDIV 16 (same effective gain).
                                                 Only used when the speed-window band is 0 (manual 'p'); the band recomputes Kp per heartbeat. */
#define PID_SPEED_KI_DEFAULT                10000/(SPEED_UNIT/10) /* = raw 10000 (SPEED_UNIT=10); matches live p10000/i10000 */
#define PID_SPEED_KD_DEFAULT                0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ             (uint16_t)1000 /*!<Execution rate of speed regulation loop (Hz) */

/* Speed PID parameter dividers */
#define SP_KPDIV                            16 /* 8192->512 (June) ->16 (2026-09-08, hand-edit; REDO IN MC WORKBENCH on regen): the speed-window
                                                 control needs Kp up to ~0.2 A/rpm (a 120 rpm fade band at 80 kg); at 512 that overflows int16. */
#define SP_KIDIV                            16384
#define SP_KDDIV                            16
#define SP_KPDIV_LOG                        LOG2((16))
#define SP_KIDIV_LOG                        LOG2((16384))
#define SP_KDDIV_LOG                        LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV         0 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING      DISABLE
#define IQMAX_A                             29.0 /* raised from 8.2 (hand-edit; REDO IN MC WORKBENCH on regen) */

/* Default settings */
#define DEFAULT_CONTROL_MODE                MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM            252
#define DEFAULT_TARGET_SPEED_UNIT           (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A          0
#define DEFAULT_FLUX_COMPONENT_A            0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V              70.0 /* raised 60->70 for the 15S pack (hand-edit; REDO IN MC WORKBENCH on regen).
                                                 15S full = 63 V; bench scope shows ~+5 V IR rise at the pack during heavy
                                                 regen (58 V rest -> ~64 V), so 60 V nuisance-tripped. 70 V = 6 V above that
                                                 peak, 20 V below the weakest bus part (90 V buck converter), and 10 V below
                                                 the Vbus sense saturation (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR
                                                 = 80.3 V) so the reading stays linear through the threshold. */
#define UD_VOLTAGE_THRESHOLD_V              42.0 /* raised 20->42 for the 15S pack (hand-edit; REDO IN MC WORKBENCH on regen).
                                                 = 2.8 V/cell. 20.0 was a 13S-era placeholder that let the drive keep pulling
                                                 29 A far below the pack floor, leaving the BMS to disconnect under load --
                                                 the failure FW_OPTIMIZATION_AUDIT.md flags as the classic VESC field kill.
                                                 MUST sit ABOVE the BMS LVC setpoint so firmware stops FIRST -- VERIFY. */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                     TURN_ON_LOW_SIDES /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C          70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C         10 /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS           DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this
                                                         feature may be used to bypass HW over-current protection
                                                         (if supported by power stage) */
#define OVP_INVERTINGINPUT_MODE             INT_MODE
#define OVP_INVERTINGINPUT_MODE2            INT_MODE
#define OVP_SELECTION                       COMP_Selection_COMP1
#define OVP_SELECTION2                      COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/
/* Encoder alignment */
#define M1_ALIGNMENT_DURATION               700 /*!< milliseconds */
#define M1_ALIGNMENT_ANGLE_DEG              90 /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT_A                 8 /*!< 8A alignment (firm snap on the coggy 20pp rotor); just under IQMAX/NOMINAL=8.2A so no clamp. ~1.6A supply draw on 30V -> ensure supply can source ~2A (or use battery). */
/* With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment */
/* phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt) */
/* being Av the voltage gain between Rshunt and A/D input */

#define TRANSITION_DURATION                 25 /* Switch over duration, ms */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(47)

/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES                 (6 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
