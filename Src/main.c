/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv8353.h"             /* DRV8353RS gate driver (bit-banged SPI)     */
#include "log.h"                 /* USB-CDC logging                            */
#include "mc_api.h"              /* motor command API                          */
#include "parameters_conversion.h" /* CURRENT_CONV_FACTOR(_INV), SPEED_UNIT    */
#include "mc_config_common.h"    /* BusVoltageSensor_M1 for Vbus logging       */
#include "mc_config.h"           /* FOCVars / pwmcHandle for loop diagnostics  */
#include "usb_device.h"          /* MX_USB_Device_Init -- see appTask note      */
#include "as5047.h"              /* AS5047P absolute encoder (SPI1) -- now USED for the FOC angle */
#include "cogg_table.h"          /* COGG_NBINS -- anti-cogging map dump                       */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ENCODER-ONLY bring-up: when 1, the motor is NEVER started (no torque, no
   commutation, no PWM). appTask just streams the raw AS5047 reading continuously
   so you can verify the encoder/magnet in isolation by turning the shaft by hand.
   Set back to 0 for normal motor operation. */
#define ENC_ONLY                0
#define BOOT_TORQUE_REF_A       0.0f    /* torque-control boot current (A); 0 = rotor holds at rest for a clean d-axis step capture. Raise only if the shaft is mechanically loaded/held. */
#define BOOT_TORQUE_RAMP_MS     500     /* 0 -> target ramp duration (ms)       */
#define STEP_MODE               0       /* 'g' test: 0 = current-loop step, 1 = speed-loop step, 2 = Iq-ripple @ const speed
                                           STEP_MODE 1 BOOTS in SPEED control to SPEED_BASE_RPM (safe, won't run away). */
/* --- Iq-ripple capture (STEP_MODE 2): run steady, capture Iq for an FFT --- */
#define RIPPLE_SPEED_RPM        150     /* constant speed for the ripple capture (rpm) */
#define RIPPLE_DECIM            10      /* HF/this = 2.5 kHz; 512 samp = ~205 ms (Nyquist 1.25 kHz) */
/* --- current-loop step (STEP_MODE 0) --- */
#define STEP_AXIS_Q             0       /* axis: 1 = q (torque, rotor spins), 0 = d (rotor holds) */
#define STEP_FROM_A             0.2f    /* step start current (A) */
#define STEP_TO_A               1.0f    /* step end current (A) */
#define STEP_BASELINE_MS        3       /* current-step baseline before the step (ms) */
#define STEP_DECIM              1       /* must match STEP_DECIM in mc_tasks_foc.c (log-rate display only) */
/* --- speed-loop step (STEP_MODE 1) --- captured at 1 kHz over 512 ms --- */
#define SPEED_BASE_RPM          50      /* speed-step baseline (rpm); moving baseline avoids standstill stiction */
#define SPEED_STEP_RPM          150     /* speed-step target (rpm); keep < ~400 (30 V voltage wall) */
#define SPEED_BASELINE_MS       40      /* speed-step baseline captured before the step (ms) */
#define SPEED_CAP_DECIM         4       /* must match SPEED_CAP_DECIM in mc_tasks_foc.c: 1kHz/4 = 250 Hz -> 512 samp = ~2 s */
#define BOOT_SPEED_RPM          60      /* speed-mode boot target (rpm); low for clean INL capture */
#define BOOT_SPEED_RAMP_MS      2000    /* 0 -> target speed ramp duration (ms) */
#define APP_LOOP_PERIOD_MS      100U    /* app-task yield / health-tick interval (INL & spi: line count on this) */
#define MOTOR_LOG_PERIOD_MS     1000U   /* periodic [motor] status log interval (1 Hz; raise to log less)        */

/* Open-loop forced commutation for a CLEAN INL capture (method A). When 1, boot
   in torque mode with a fixed holding current and rotate the electrical field at
   a constant commanded speed -- the encoder is OUT of the loop, so the captured
   angle vs the constant ramp is free of speed-loop ripple. Set 0 for normal run. */
#define BOOT_OPENLOOP           0       /* 0 = normal closed-loop (encoder in loop)        */
#define BOOT_OL_SPEED_RPM       30      /* commanded mech speed (rpm); low for clean INL  */
#define BOOT_OL_IQ_A            4.0f    /* holding q-current (A); raise if the rotor slips */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CORDIC_HandleTypeDef hcordic;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

osThreadId mediumFrequencyHandle;
osThreadId safetyHandle;
/* USER CODE BEGIN PV */
osThreadId appTaskHandle;        /* boot diagnostics + auto-start + status log  */
static void StartAppTask(void const *argument);
static const char *mc_state_name(MCI_State_t state);
static void log_mc_faults(const char *label, uint16_t faults);
static void motor_status_log(void);
/* Encoder index (Z) on PB3 -> reset TIM3 count once per mech rev (cancels ABI
   miscount drift). Impl in Ropetow_IndexInit()/EXTI3_IRQHandler() (USER CODE 4). */
static void Ropetow_IndexInit(void);
static volatile uint16_t g_idx_cnt    = 0U;  /* TIM3 count captured at the index   */
static volatile uint8_t  g_idx_have   = 0U;  /* index reference captured this run?  */
static volatile uint32_t g_idx_events = 0U;  /* diag: index pulses handled          */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void startMediumFrequencyTask(void const * argument);
extern void StartSafetyTask(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_MotorControl_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mediumFrequency */
  osThreadDef(mediumFrequency, startMediumFrequencyTask, osPriorityNormal, 0, 128);
  mediumFrequencyHandle = osThreadCreate(osThread(mediumFrequency), NULL);

  /* definition and creation of safety */
  osThreadDef(safety, StartSafetyTask, osPriorityAboveNormal, 0, 128);
  safetyHandle = osThreadCreate(osThread(safety), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Application task: configures the DRV8353, dumps boot diagnostics, starts
     the motor, and streams periodic status over USB CDC. Low priority so it
     never disturbs the medium-frequency / safety MC tasks; larger stack for
     the printf-style logger. */
  osThreadDef(appTask, StartAppTask, osPriorityLow, 0, 512);
  appTaskHandle = osThreadCreate(osThread(appTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 3;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = M1_PULSE_NBR;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M1_EN_DRIVER_GPIO_Port, M1_EN_DRIVER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : M1_EN_DRIVER_Pin */
  GPIO_InitStruct.Pin = M1_EN_DRIVER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M1_EN_DRIVER_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* --- Encoder index (Z) on PB3 ---------------------------------------------
 * The AS5047 index pulses once per MECHANICAL revolution. EMI miscounts on the
 * ABI lines accumulate in TIM3->CNT and drift the commutation angle; this snaps
 * the count back to its index-aligned value every rev. EXTI3 rising edge, NVIC
 * priority below the FOC ISRs. The first index after each RUN captures the
 * reference count (consistent with the startup alignment); later indexes correct
 * only plausible drift (<~1/32 rev) so a spurious edge can't throw the angle. */
static void Ropetow_IndexInit(void)
{
  GPIO_InitTypeDef gi = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gi.Pin  = GPIO_PIN_3;
  gi.Mode = GPIO_MODE_IT_RISING;          /* index idles low, pulses high */
  gi.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &gi);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0); /* below TIM1/ADC/TIM3/BRK (prio 0..4) */
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void EXTI3_IRQHandler(void)
{
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != 0U)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    if (MC_GetSTMStateMotor1() == RUN)
    {
      uint16_t cnt = (uint16_t)TIM3->CNT;
      if (g_idx_have == 0U)
      {
        g_idx_cnt  = cnt;                 /* first index this run = reference */
        g_idx_have = 1U;
      }
      else
      {
        int32_t period = (int32_t)M1_PULSE_NBR + 1;       /* TIM3 wraps 0..M1_PULSE_NBR */
        int32_t d = (int32_t)cnt - (int32_t)g_idx_cnt;    /* shortest signed wrap diff  */
        if (d >  (period / 2)) { d -= period; }
        if (d < -(period / 2)) { d += period; }
        if ((d > -128) && (d < 128))      /* correct small drift; reject spurious edges */
        {
          TIM3->CNT = (uint32_t)g_idx_cnt;
        }
      }
      g_idx_events++;
    }
  }
}

static const char *mc_state_name(MCI_State_t state)
{
  switch (state)
  {
    case IDLE:            return "IDLE";
    case ICLWAIT:         return "ICLWAIT";
    case ALIGNMENT:       return "ALIGNMENT";
    case CHARGE_BOOT_CAP: return "CHARGE_BOOT_CAP";
    case OFFSET_CALIB:    return "OFFSET_CALIB";
    case START:           return "START";
    case SWITCH_OVER:     return "SWITCH_OVER";
    case RUN:             return "RUN";
    case STOP:            return "STOP";
    case FAULT_NOW:       return "FAULT_NOW";
    case FAULT_OVER:      return "FAULT_OVER";
    case WAIT_STOP_MOTOR: return "WAIT_STOP_MOTOR";
    default:              return "UNKNOWN";
  }
}

static void log_mc_faults(const char *label, uint16_t faults)
{
  LOG_Printf("[motor]   %s=0x%04X%s%s%s%s%s%s%s%s%s\r\n", label, (unsigned)faults,
             (faults & MC_DURATION)   ? " FOC_DURATION" : "",
             (faults & MC_OVER_VOLT)  ? " OVER_VOLT"    : "",
             (faults & MC_UNDER_VOLT) ? " UNDER_VOLT"   : "",
             (faults & MC_OVER_TEMP)  ? " OVER_TEMP"    : "",
             (faults & MC_START_UP)   ? " START_UP"     : "",
             (faults & MC_SPEED_FDBK) ? " SPEED_FDBK"   : "",
             (faults & MC_OVER_CURR)  ? " OVER_CURR"    : "",
             (faults & MC_SW_ERROR)   ? " SW_ERROR"     : "",
             (faults & MC_DP_FAULT)   ? " DP_FAULT"     : "");
}

/* Periodic motor status over USB CDC: one line per MOTOR_LOG_PERIOD_MS, plus
   an immediate line whenever the MC state or the fault flags change. Called
   from the low-priority app task; LOG_Printf only fills the ring buffer, so
   nothing here blocks or disturbs the FOC interrupts. */
static void motor_status_log(void)
{
  static uint32_t    last_tick;
  static MCI_State_t last_state  = STATE_ENUM_COUNT;   /* forces first print */
  static uint16_t    last_faults;

  MCI_State_t state   = MC_GetSTMStateMotor1();
  uint16_t    faults  = MC_GetCurrentFaultsMotor1();
  uint32_t    now     = HAL_GetTick();
  uint8_t     changed = (state != last_state) || (faults != last_faults);

  if (!changed && ((now - last_tick) < MOTOR_LOG_PERIOD_MS))
  {
    return;
  }
  last_tick   = now;
  last_state  = state;
  last_faults = faults;

  /* newlib-nano printf has no %f support, so format the current with integer
     math: round to tenths of an amp, then split into whole/fractional digit. */
  int32_t i_tenths = (int32_t)(((float)MC_GetPhaseCurrentAmplitudeMotor1() *
                                (float)CURRENT_CONV_FACTOR_INV) * 10.0f + 0.5f);

  /* Iqref: q-axis current reference (s16A). Iq/Id: MEASURED dq currents.
     Vq: commanded q-axis voltage. cnt: raw TIM3 encoder count. pwm: PWMC on. */
  LOG_Printf("[motor] %s spd=%ld/%ld rpm I=%ld.%ldA Vbus=%uV enc=%s "
             "Iqref=%d Iq=%d Id=%d Vq=%d cnt=%lu pwm=%u\r\n",
             mc_state_name(state),
             (int32_t)MC_GetMecSpeedAverageMotor1()   * U_RPM / SPEED_UNIT,
             (int32_t)MC_GetMecSpeedReferenceMotor1() * U_RPM / SPEED_UNIT,
             i_tenths / 10, i_tenths % 10,
             (unsigned)VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super),
             MC_GetSpeedSensorReliabilityMotor1() ? "ok" : "BAD",
             (int)FOCVars[M1].Iqdref.q,
             (int)FOCVars[M1].Iqd.q,
             (int)FOCVars[M1].Iqd.d,
             (int)FOCVars[M1].Vqd.q,
             (unsigned long)TIM3->CNT,
             (unsigned)PWMC_GetPWMState(pwmcHandle[M1]));

  if (changed && ((faults != 0U) || (MC_GetOccurredFaultsMotor1() != 0U)))
  {
    log_mc_faults("current", faults);
    log_mc_faults("occurred", MC_GetOccurredFaultsMotor1());
  }
}

/* Application thread. Runs after the scheduler starts (and after the medium-
   frequency task). The DRV8353 must be configured (6x PWM, CSA gain 20 V/V,
   OCP) BEFORE the motor starts, i.e. before any PWM. */
static void StartAppTask(void const *argument)
{
  (void)argument;

  /* Bring up USB CDC here. CubeMX places MX_USB_Device_Init() in the *weak*
     startMediumFrequencyTask in main.c, but the MCSDK provides a strong
     startMediumFrequencyTask in mc_tasks.c that OVERRIDES it -- so that init
     never runs and no CDC device enumerates. Call it from this task instead. */
  MX_USB_Device_Init();

  /* Let USB CDC enumerate and the supply settle before talking to anything. */
  osDelay(1000);
  LOG_Init();

  DRV8353_Status drv = DRV8353_Init();

  LOG_Printf("\r\n=== Trainsonic boot ===\r\n");
  LOG_Printf("DRV8353_Init=%d\r\n", (int)drv);
  DRV8353_LogStatus();
  {
    AS5047_Status enc = AS5047_Init(&hspi1);   /* FOC now commutates on the AS5047 SPI angle */
    LOG_Printf("AS5047_Init=%d\r\n", (int)enc);
    AS5047_LogStatus(&hspi1);                  /* dump AGC/MAGL/MAGH -> magnet health vs vibration */
  }
  LOG_Printf("=== end boot diagnostics ===\r\n");

  /* --- Automatic motor start --------------------------------------------
   * WARNING: spins the drive on every power-up with no operator interlock.
   * On a rope-tow that moves a load this is a hazard -- add an enable / e-stop
   * interlock before field use. Only start if the gate driver configured. */
  if ((drv == DRV8353_OK) && (ENC_ONLY == 0))
  {
    Ropetow_IndexInit();                     /* arm PB3 index -> TIM3 count reset */
    (void)MC_AcknowledgeFaultMotor1();       /* clear any latched fault        */
#if BOOT_OPENLOOP
    {
      /* Open-loop forced commutation (method A INL capture): encoder out of the
         loop, torque mode + fixed Iq, electrical field rotated at a constant
         commanded speed. After MCSDK alignment locks the rotor, the forced ramp
         carries it around at BOOT_OL_SPEED_RPM. */
      extern volatile uint8_t g_use_spi_commutation;
      extern volatile uint8_t g_ol_enable;
      extern volatile float   g_ol_step;
      g_use_spi_commutation = 0U;            /* don't extrapolate the encoder      */
      g_ol_step = (float)BOOT_OL_SPEED_RPM * (float)POLE_PAIR_NUM * 65536.0f
                  / (60.0f * (float)ISR_FREQUENCY_HZ);
      g_ol_enable = 1U;
      MC_ProgramTorqueRampMotor1_F(BOOT_OL_IQ_A, BOOT_TORQUE_RAMP_MS);
      if (MC_StartMotor1())
      {
        LOG_Printf("OPEN-LOOP start: forced %d rpm, Iq=%d mA (encoder out of loop)\r\n",
                   (int)BOOT_OL_SPEED_RPM, (int)(BOOT_OL_IQ_A * 1000.0f));
      }
      else
      {
        LOG_Printf("Motor start REJECTED: state=%d faults=0x%04X\r\n",
                   (int)MC_GetSTMStateMotor1(),
                   (unsigned)MC_GetCurrentFaultsMotor1());
      }
    }
#elif (STEP_MODE == 1) || (STEP_MODE == 2)
    /* Closed-loop SPEED mode. Mode 1: baseline for the speed-step test.
       Mode 2: hold a constant speed for the Iq-ripple capture. */
  #if STEP_MODE == 2
    #define BOOT_SPD_RPM RIPPLE_SPEED_RPM
  #else
    #define BOOT_SPD_RPM SPEED_BASE_RPM
  #endif
    MC_ProgramSpeedRampMotor1((int16_t)((int32_t)BOOT_SPD_RPM * SPEED_UNIT / U_RPM),
                              BOOT_SPEED_RAMP_MS);
    if (MC_StartMotor1())
    {
      LOG_Printf("Motor start commanded -> speed mode, %d rpm over %d ms\r\n",
                 (int)BOOT_SPD_RPM, BOOT_SPEED_RAMP_MS);
    }
    else
    {
      LOG_Printf("Motor start REJECTED: state=%d faults=0x%04X\r\n",
                 (int)MC_GetSTMStateMotor1(),
                 (unsigned)MC_GetCurrentFaultsMotor1());
    }
#else
    /* Closed-loop TORQUE mode: encoder commutation (g_use_spi_commutation=1),
       no speed regulation -- Iq ramps to BOOT_TORQUE_REF_A and the motor produces
       that torque (unloaded it accelerates until friction/back-EMF balance). */
    MC_ProgramTorqueRampMotor1_F(BOOT_TORQUE_REF_A, BOOT_TORQUE_RAMP_MS);
    if (MC_StartMotor1())
    {
      LOG_Printf("Motor start commanded -> torque mode, Iq=%d mA over %d ms\r\n",
                 (int)(BOOT_TORQUE_REF_A * 1000.0f), BOOT_TORQUE_RAMP_MS);
    }
    else
    {
      LOG_Printf("Motor start REJECTED: state=%d faults=0x%04X\r\n",
                 (int)MC_GetSTMStateMotor1(),
                 (unsigned)MC_GetCurrentFaultsMotor1());
    }
#endif
  }
  else if (ENC_ONLY != 0)
  {
    LOG_Printf("ENC_ONLY: motor NOT started -- streaming raw encoder. Turn shaft by hand.\r\n");
  }
  else
  {
    LOG_Printf("Motor start SKIPPED: DRV8353_Init=%d\r\n", (int)drv);
  }

  for (;;)
  {
    static uint32_t idx_log = 0U;
    static uint32_t run_ticks = 0U;
    extern uint16_t          g_cal_raw[];
    extern volatile uint16_t g_cal_idx;
    extern volatile uint8_t  g_cal_state;
    extern int16_t           g_step_ref[];
    extern int16_t           g_step_iq[];
    extern volatile uint16_t g_step_idx;
    extern volatile uint8_t  g_step_state;
    extern volatile uint8_t  g_step_request;
    extern volatile uint8_t  g_inj_override;
    extern volatile int16_t  g_inj_s16;
    extern volatile uint8_t  g_step_axis;
    extern volatile uint8_t  g_step_mode;
    extern volatile uint16_t g_cap_decim;

#if ENC_ONLY
    /* Pure encoder readout: the motor was never started, so the MF hook isn't
       touching the SPI and appTask owns it. Stream the raw AS5047 reading (~10 Hz)
       so the encoder/magnet can be validated in isolation. Watch raw sweep
       0..16383 smoothly per hand-turn, AGC mid-scale, all flags 0. */
    AS5047_LogAngle(&hspi1);
    for (uint16_t eguard = 0U; (LOG_Pending() > 0U) && (eguard < 200U); eguard++)
    { LOG_Process(); osDelay(2U); }
    osDelay(80U);
    continue;
#endif

    /* One-shot: log the calibrated 3-shunt offsets once the motor is running.
       A phase offset that differs from the others is a DC current bias -> the
       1x-electrical Iq ripple we saw. ODrive re-nulls these every startup. */
    {
      static uint8_t offsets_logged = 0U;
      extern void Ropetow_GetOffsets(int32_t *a, int32_t *b, int32_t *c);
      if ((offsets_logged == 0U) && (MC_GetSTMStateMotor1() == RUN))
      {
        int32_t oa = 0, ob = 0, oc = 0;
        Ropetow_GetOffsets(&oa, &ob, &oc);
        LOG_Printf("offsets(3-shunt): A=%ld B=%ld C=%ld  spread=%ld\r\n",
                   (long)oa, (long)ob, (long)oc,
                   (long)((oa>ob?oa:ob)>oc ? (oa>ob?oa:ob) : oc) -
                   (long)((oa<ob?oa:ob)<oc ? (oa<ob?oa:ob) : oc));
        offsets_logged = 1U;
      }
    }

    /* Live torque-current setpoint ('t<mA>', signed): command Iq in torque
       control, clamped to a safe +/-8 A. Negative = reverse torque (e.g. t-200).
       Switches the motor to torque mode at that current. */
    {
      extern volatile int32_t g_torque_set_ma;
      extern volatile uint8_t g_torque_set_req;
      if ((g_torque_set_req != 0U) && (MC_GetSTMStateMotor1() == RUN))
      {
        int32_t ma = g_torque_set_ma;
        g_torque_set_req = 0U;
        if (ma >  8000) { ma =  8000; }
        if (ma < -8000) { ma = -8000; }
        MC_ProgramTorqueRampMotor1_F((float)ma / 1000.0f, 200U);
        LOG_Printf("torque set -> %ld mA\r\n", (long)ma);
      }
    }

    /* Live SPEED setpoint ('s<rpm>'): command a speed ramp (switches to / stays in
       speed control). Clamped to a safe 0..600 rpm. Sweep this to find where the
       low-speed stick-slip starts/stops. */
    {
      extern volatile int32_t g_speed_set_rpm;
      extern volatile uint8_t g_speed_set_req;
      if ((g_speed_set_req != 0U) && (MC_GetSTMStateMotor1() == RUN))
      {
        int32_t rpm = g_speed_set_rpm;
        g_speed_set_req = 0U;
        if (rpm > 500) { rpm = 500; }
        if (rpm < 0)   { rpm = 0; }
        MC_ProgramSpeedRampMotor1((int16_t)((int32_t)rpm * SPEED_UNIT / U_RPM), 500U);
        LOG_Printf("speed set -> %ld rpm\r\n", (long)rpm);
      }
    }

    /* Motor restart ('R'): acknowledge any latched fault and re-start the drive in
       torque mode at 0 A (stationary, safe) so it lands in RUN -- e.g. after a
       speed-feedback fault, or to re-arm for the 'j' / position workflows without
       a board reset. Mirrors the boot start sequence; logs the same way. NOTE: if
       the underlying fault (e.g. enc=BAD speed reliability) persists, the drive
       will simply re-fault -- fix the root cause, this only clears the latch. */
    {
      extern volatile uint8_t g_restart_req;
      if (g_restart_req != 0U)
      {
        g_restart_req = 0U;
        (void)MC_AcknowledgeFaultMotor1();
        MC_ProgramTorqueRampMotor1_F(0.0f, 0U);
        if (MC_StartMotor1())
        {
          LOG_Printf("restart: motor commanded -> torque mode, 0 A\r\n");
        }
        else
        {
          LOG_Printf("restart REJECTED: state=%d faults=0x%04X\r\n",
                     (int)MC_GetSTMStateMotor1(),
                     (unsigned)MC_GetCurrentFaultsMotor1());
        }
      }
    }

    /* Raw per-phase current log ('j<rpm>'): burst the measured phase currents
       (Ia, Ib, Ic = -(Ia+Ib)) to characterise the 3-shunt sense chain.
         j0      -> standstill, zero current commanded: residual DC = offset error,
                    spread = noise (currents should sit at ~0).
         j<rpm>  -> spin at a constant speed first, then sample: currents are AC
                    (mean ~0, pkpk = current amplitude), then the motor is stopped.
       Values are MCSDK s16 current units (offset already removed; the calibrated
       offsets are printed too so raw ADC can be reconstructed). */
    {
      extern volatile uint8_t g_iadc_log_req;
      extern volatile uint8_t g_pos_mode;
      extern void Ropetow_GetOffsets(int32_t *a, int32_t *b, int32_t *c);
      if (g_iadc_log_req != 0U)
      {
        g_iadc_log_req = 0U;
        if (MC_GetSTMStateMotor1() != RUN)
        {
          LOG_Printf("iadc: SKIPPED -- drive not in RUN (state=%d)\r\n",
                     (int)MC_GetSTMStateMotor1());
        }
        else if (g_pos_mode != 0U)
        {
          LOG_Printf("iadc: SKIPPED -- position servo active; send 'O' to disable first\r\n");
        }
        else
        {
          extern volatile int32_t g_iadc_speed_rpm;
          int32_t rpm = g_iadc_speed_rpm;
          if (rpm < 0)   { rpm = 0; }
          if (rpm > 500) { rpm = 500; }              /* same clamp as 's<rpm>' */
          int32_t oa = 0, ob = 0, oc = 0;
          Ropetow_GetOffsets(&oa, &ob, &oc);
          if (rpm == 0)
          {
            MC_ProgramTorqueRampMotor1_F(0.0f, 0U);  /* zero current commanded */
            osDelay(50U);                            /* let residual current decay */
          }
          else
          {
            /* spin to a constant speed and WAIT until it actually stabilizes before
               sampling (at speed the phase currents are AC: mean ~0, pkpk = current
               amplitude). Poll the measured speed and require it within tolerance
               for a sustained window; bail after a timeout so a stuck ramp can't
               hang the logger. */
            int32_t  tol    = (rpm * 3) / 100;       /* +/-3% of target ... */
            uint32_t t0     = HAL_GetTick();
            uint16_t stable = 0U;
            if (tol < 3) { tol = 3; }                /* ... but at least +/-3 rpm */
            MC_ProgramSpeedRampMotor1((int16_t)((int32_t)rpm * SPEED_UNIT / U_RPM), 1000U);
            for (;;)
            {
              int32_t meas = (int32_t)MC_GetMecSpeedAverageMotor1() * U_RPM / SPEED_UNIT;
              int32_t e    = meas - rpm;
              if (e < 0) { e = -e; }
              stable = (e <= tol) ? (uint16_t)(stable + 1U) : 0U;
              if (stable >= 15U) { break; }          /* ~300 ms continuously in band */
              if ((HAL_GetTick() - t0) > 5000U)
              {
                LOG_Printf("iadc: speed did NOT stabilize (meas=%ld tgt=%ld); sampling anyway\r\n",
                           (long)meas, (long)rpm);
                break;
              }
              LOG_Process();
              osDelay(20U);
            }
          }
          LOG_Printf("iadc_start n=64 (k,Ia,Ib,Ic s16) speed=%ld offsets A=%ld B=%ld C=%ld\r\n",
                     (long)rpm, (long)oa, (long)ob, (long)oc);
          int32_t sa = 0, sb = 0, sc = 0;
          int32_t mnA = 32767, mxA = -32768, mnB = 32767, mxB = -32768, mnC = 32767, mxC = -32768;
          for (int k = 0; k < 64; k++)
          {
            int16_t ia = FOCVars[M1].Iab.a;
            int16_t ib = FOCVars[M1].Iab.b;
            int16_t ic = (int16_t)(-(ia + ib));
            LOG_Printf("%d,%d,%d,%d\r\n", k, (int)ia, (int)ib, (int)ic);
            sa += ia; sb += ib; sc += ic;
            if (ia < mnA) { mnA = ia; }  if (ia > mxA) { mxA = ia; }
            if (ib < mnB) { mnB = ib; }  if (ib > mxB) { mxB = ib; }
            if (ic < mnC) { mnC = ic; }  if (ic > mxC) { mxC = ic; }
            for (uint16_t g = 0U; (LOG_Pending() > 0U) && (g < 200U); g++)
            { LOG_Process(); osDelay(1U); }
            osDelay(2U);
          }
          LOG_Printf("iadc_end mean A=%ld B=%ld C=%ld pkpk A=%d B=%d C=%d\r\n",
                     (long)(sa / 64), (long)(sb / 64), (long)(sc / 64),
                     (int)(mxA - mnA), (int)(mxB - mnB), (int)(mxC - mnC));
          if (rpm != 0) { MC_ProgramSpeedRampMotor1(0, 500U); }  /* stop after sampling */
        }
      }
    }

    /* Position-servo telemetry: while the position mode is engaged, print target
       vs measured angle (and error in degrees) at ~5 Hz so the servo can be tuned
       over the serial link. The PD loop itself runs in the 1 kHz MF hook. */
    {
      extern volatile uint8_t g_pos_mode;
      extern volatile int32_t g_pos_counts, g_pos_target;
      extern volatile uint8_t g_cogg_cal_state;
      static uint32_t pos_log_tick = 0U;
      /* suppress the [pos] line during cogging cal (the servo drives the sweep) */
      if ((g_pos_mode != 0U) && (g_cogg_cal_state == 0U) && ((HAL_GetTick() - pos_log_tick) >= 200U))
      {
        pos_log_tick = HAL_GetTick();
        int32_t err   = g_pos_target - g_pos_counts;
        /* counts -> deci-degrees (16384 counts/rev): deg*10 = counts*3600/16384 */
        int32_t edd   = (int32_t)(((int64_t)err * 3600) / 16384);
        LOG_Printf("[pos] tgt=%ld cur=%ld err=%ld (%ld.%01ld deg)\r\n",
                   (long)g_pos_target, (long)g_pos_counts, (long)err,
                   (long)(edd / 10), (long)(edd < 0 ? -edd % 10 : edd % 10));
      }
    }

    /* Live raw-encoder monitor ('r'): compact angle/AGC/flags each iteration so you
       can watch the sensor while hand-turning. Motor must be STOPPED (the MF hook
       owns hspi1 in RUN). Toggle off with 'r' again. */
    {
      extern volatile uint8_t g_enc_mon;
      if (g_enc_mon != 0U)
      {
        if (MC_GetSTMStateMotor1() == RUN)
        {
          LOG_Printf("enc mon: STOP the motor first (SPI shared with the FOC loop)\r\n");
          g_enc_mon = 0U;
        }
        else
        {
          AS5047_LogAngle(&hspi1);
          for (uint16_t guard = 0U; (LOG_Pending() > 0U) && (guard < 200U); guard++)
          { LOG_Process(); osDelay(2U); }
          osDelay(80U);
          continue;
        }
      }
    }

    /* AGC-vs-position sweep ('a'): log AS5047 AGC/MAGL/MAGH + angle ~24x over ~3 s.
       Spin the shaft slowly during this. Flat AGC (~246 everywhere) => uniform
       under-field (magnet too weak / air-gap too large). AGC that swings with the
       logged angle => eccentric / off-axis mount. See plan Phase 0. */
    {
      extern volatile uint8_t g_agc_log_req;
      if (g_agc_log_req != 0U)
      {
        g_agc_log_req = 0U;
        /* Motor must be STOPPED: the MF hook reads the AS5047 on hspi1 while in RUN,
           and a second reader here would race the SPI. Turn the shaft BY HAND. */
        if (MC_GetSTMStateMotor1() == RUN)
        {
          LOG_Printf("agc sweep: STOP the motor first (turn shaft by hand)\r\n");
          continue;
        }
        LOG_Printf("agc sweep: turn shaft slowly by hand now (24 samples, ~3 s)...\r\n");
        for (uint8_t k = 0U; k < 24U; k++)
        {
          AS5047_LogStatus(&hspi1);
          for (uint16_t guard = 0U; (LOG_Pending() > 0U) && (guard < 200U); guard++)
          { LOG_Process(); osDelay(2U); }
          osDelay(120U);
        }
        LOG_Printf("agc sweep: done\r\n");
        continue;
      }
    }

    /* Anti-cogging calibration arm ('y'): clear accumulators and start binning the
       measured Iq by mechanical position (done in the MF hook). Run a slow constant
       speed and let it sweep several revolutions, then send 'Y' to stop + dump. */
    {
      extern volatile uint8_t g_cogg_cal_req;
      extern void Ropetow_CoggCalArm(void);
      if ((g_cogg_cal_req != 0U) && (MC_GetSTMStateMotor1() == RUN))
      {
        g_cogg_cal_req = 0U;
        Ropetow_CoggCalArm();
        LOG_Printf("cogg cal: auto-sweep + ILC + harmonic denoise (free shaft!) -- ~2.5min, dumps when done. 'Y'=abort\r\n");
      }
    }
    /* 'm' -> more ILC refine passes on the EXISTING map (keep refining until smooth). */
    {
      extern volatile uint8_t g_cogg_refine_req;
      extern volatile uint8_t g_cogg_cal_state;
      if ((g_cogg_refine_req != 0U) && (MC_GetSTMStateMotor1() == RUN))
      {
        g_cogg_refine_req = 0U;
        if (g_cogg_cal_state == 0U)   /* only when fully idle (after any dump finishes) */
        {
          g_cogg_cal_state = 4U;   /* PositionControl runs the refine passes */
          LOG_Printf("cogg refine: more ILC passes on current map (free shaft) -- ~80s\r\n");
        }
      }
    }

    if ((g_step_request != 0U) && (g_step_state == 0U) &&
        (MC_GetSTMStateMotor1() == RUN))
    {
      g_step_request = 0U;
#if STEP_MODE == 2
      /* Iq-ripple capture: motor already holds RIPPLE_SPEED_RPM. Just arm the HF
         capture (q-axis: Iqref vs measured Iq) at the decimated rate -- NO step,
         NO injection. The dump is FFT'd on the host to find the ripple frequency. */
      g_step_mode = 0U;            /* HF-rate capture path */
      g_step_axis = 1U;            /* log Iqref.q / Iq.q   */
      g_cap_decim = RIPPLE_DECIM;
      g_step_idx = 0U; g_step_state = 1U;
      LOG_Printf("ripple: capturing Iq @ %d rpm, %u Hz...\r\n",
                 (int)RIPPLE_SPEED_RPM, (unsigned)(ISR_FREQUENCY_HZ / RIPPLE_DECIM));
#elif STEP_MODE == 1
      /* Speed-loop step: motor already runs at SPEED_BASE_RPM; capture a baseline
         then step the speed reference to SPEED_STEP_RPM. Logged at 1 kHz (MF). */
      g_step_mode = 1U;
      g_step_idx = 0U; g_step_state = 1U;                /* arm (baseline @ SPEED_BASE) */
      osDelay(SPEED_BASELINE_MS);
      MC_ProgramSpeedRampMotor1((int16_t)((int32_t)SPEED_STEP_RPM * SPEED_UNIT / U_RPM), 0U);
      LOG_Printf("step: speed %d -> %d rpm, capturing @ 1 kHz...\r\n",
                 (int)SPEED_BASE_RPM, (int)SPEED_STEP_RPM);
#else
      /* Current-loop step: pin the chosen axis (q=torque or d=hold), short baseline
         at STEP_FROM_A, then step to STEP_TO_A. HF ISR logs ref vs measured @25 kHz. */
      g_step_mode = 0U;
      g_step_axis = STEP_AXIS_Q;
      g_inj_s16 = (int16_t)(STEP_FROM_A * (float)CURRENT_CONV_FACTOR); /* baseline */
      g_inj_override = 1U;
      osDelay(20U);                          /* settle baseline BEFORE arming */
      g_step_idx = 0U; g_step_state = 1U;                              /* arm capture */
      osDelay(STEP_BASELINE_MS);
      g_inj_s16 = (int16_t)(STEP_TO_A * (float)CURRENT_CONV_FACTOR);   /* inject step */
      LOG_Printf("step: %c-axis %d -> %d mA, capturing @ %u kHz...\r\n",
                 (g_step_axis ? 'q' : 'd'),
                 (int)(STEP_FROM_A * 1000.0f), (int)(STEP_TO_A * 1000.0f),
                 (unsigned)(ISR_FREQUENCY_HZ / 1000U / STEP_DECIM));
#endif
    }

    /* Step-response dump (drain-safe): idx,ref,meas (current=s16, speed=rpm). */
    if (g_step_state == 2U)
    {
      static uint16_t sd = 0U;
      uint16_t n = g_step_idx;
      uint16_t end = (uint16_t)(sd + 8U);
      char axc = (g_step_mode ? 's' : (g_step_axis ? 'q' : 'd'));
      unsigned rate_hz = (g_step_mode ? (SPEED_LOOP_FREQUENCY_HZ / SPEED_CAP_DECIM)
                                      : (ISR_FREQUENCY_HZ / g_cap_decim));
      if (end > n) { end = n; }
      if (sd == 0U)
      {
        /* capture done -> stop driving the step IMMEDIATELY: current -> release,
           speed -> ramp back to baseline. */
        if (g_step_mode == 0U) { g_inj_s16 = 0; }
        else { MC_ProgramSpeedRampMotor1((int16_t)((int32_t)SPEED_BASE_RPM * SPEED_UNIT / U_RPM), 0U); }
        LOG_Printf("step_start n=%u hz=%u axis=%c\r\n", (unsigned)n, rate_hz, axc);
      }
      for (; sd < end; sd++)
      { LOG_Printf("%u,%d,%d\r\n", (unsigned)sd, (int)g_step_ref[sd], (int)g_step_iq[sd]); }
      if (sd >= n)
      {
        LOG_Printf("step_end\r\n");
        sd = 0U; g_step_state = 0U;   /* re-arm: 'g' can trigger again (was stuck at 3 = once/boot) */
        if (g_step_mode == 0U) { g_inj_override = 0U; }
      }
      for (uint16_t guard = 0U; (LOG_Pending() > 0U) && (guard < 200U); guard++)
      {
        LOG_Process();
        osDelay(2U);
      }
      continue;
    }

    /* Anti-cogging map dump (drain-safe): per-bin mean of the measured Iq the loop
       fought, indexed by mechanical position. Host cogging.py parses this (run it
       once per direction, then combine). Format mirrors cal_start/step_start. */
    {
      extern volatile uint8_t g_cogg_cal_state;
      extern volatile uint8_t g_cogg_harm_enable;
      extern void Ropetow_CoggCalGet(uint16_t bin, int16_t *mean, uint16_t *count);
      extern void Ropetow_CoggHarmonicFit(void);
      if (g_cogg_cal_state == 2U)
      {
        static uint16_t gd = 0U;
        uint16_t end = (uint16_t)(gd + 8U);
        if (end > (uint16_t)COGG_NBINS) { end = (uint16_t)COGG_NBINS; }
        if (gd == 0U)
        {
          if (g_cogg_harm_enable != 0U) { Ropetow_CoggHarmonicFit(); }  /* denoise once before dumping */
          LOG_Printf("cogg_start nbins=%u harm=%u\r\n", (unsigned)COGG_NBINS, (unsigned)g_cogg_harm_enable);
        }
        for (; gd < end; gd++)
        {
          int16_t  mean = 0; uint16_t cnt = 0U;
          Ropetow_CoggCalGet(gd, &mean, &cnt);
          LOG_Printf("%u,%d,%u\r\n", (unsigned)gd, (int)mean, (unsigned)cnt);
        }
        if (gd >= (uint16_t)COGG_NBINS) { LOG_Printf("cogg_end\r\n"); gd = 0U; g_cogg_cal_state = 0U; }
        for (uint16_t guard = 0U; (LOG_Pending() > 0U) && (guard < 200U); guard++)
        {
          LOG_Process();
          osDelay(2U);
        }
        continue;
      }
    }

#if BOOT_OPENLOOP
    const int32_t cal_target_rpm = BOOT_OL_SPEED_RPM;
#else
    const int32_t cal_target_rpm = BOOT_SPEED_RPM;
#endif
    int32_t spd_now = (int32_t)MC_GetMecSpeedAverageMotor1() * U_RPM / SPEED_UNIT;
    int32_t spd_err = spd_now - cal_target_rpm;
    if (MC_GetSTMStateMotor1() != RUN) { g_idx_have = 0U; }

    /* INL capture: trigger only after the speed has been STEADY at target
       (within +/-15 rpm) for ~3 s, so the startup/overshoot transient is over. */
    if ((MC_GetSTMStateMotor1() == RUN) && (spd_err > -15) && (spd_err < 15)) { run_ticks++; }
    else { run_ticks = 0U; }
    if ((run_ticks >= 30U) && (g_cal_state == 0U))
    {
      g_cal_idx = 0U; g_cal_state = 1U;
      LOG_Printf("cal: speed steady, capturing angle @ ~100 Hz...\r\n");
    }

    /* INL dump: emit a small chunk, then DRAIN THE RING TO EMPTY before producing
       the next chunk, so the 2048-byte log buffer can never overflow (the previous
       version pushed ~144 B/iter but LOG_Process drains only 64 B/call -> the ring
       filled after ~400 lines and LOG_Printf dropped the rest -> corrupt CSV).
       The continue suppresses status logs while dumping so the CSV stays clean. */
    if (g_cal_state == 2U)
    {
      static uint16_t cd = 0U;
      uint16_t n = g_cal_idx;
      uint16_t end = (uint16_t)(cd + 8U);
      if (end > n) { end = n; }
      if (cd == 0U) { LOG_Printf("cal_start n=%u\r\n", (unsigned)n); }
      for (; cd < end; cd++) { LOG_Printf("%u,%u\r\n", (unsigned)cd, (unsigned)g_cal_raw[cd]); }
      if (cd >= n) { LOG_Printf("cal_end\r\n"); cd = 0U; g_cal_state = 3U; }
      /* drain-to-empty: CDC sends <=64 B per completed transfer (~1 ms at FS),
         so loop with a short yield until the ring clears (bounded for safety). */
      for (uint16_t guard = 0U; (LOG_Pending() > 0U) && (guard < 200U); guard++)
      {
        LOG_Process();
        osDelay(2U);
      }
      continue;
    }

    {
    extern volatile uint8_t g_cogg_cal_state;
    extern volatile uint8_t g_cogg_cal_pass;
    extern volatile uint8_t g_cogg_cal_target;
    if (g_cogg_cal_state == 1U)
    {
      /* CALIBRATING: show only the cal progress, suppress the normal telemetry flood. */
      static uint32_t cogg_log_tick = 0U;
      if ((HAL_GetTick() - cogg_log_tick) >= 500U)
      {
        extern volatile int16_t g_avg_iq;
        cogg_log_tick = HAL_GetTick();
        LOG_Printf("[cogg] pass %u/%u  Iq=%d  (sweeping)\r\n",
                   (unsigned)(g_cogg_cal_pass + 1U), (unsigned)g_cogg_cal_target, (int)g_avg_iq);
      }
    }
    else if (g_cogg_cal_state == 0U)
    {
    motor_status_log();
    if (++idx_log >= 10U)                                    /* ~1 s: index health */
    {
      extern volatile uint32_t g_spi_err_count;     /* SPI read failures (mc_tasks_foc.c) */
      extern volatile int16_t  g_dbg_spi_minus_tim3; /* SPI angle - TIM3 angle (s16)       */
      extern volatile uint8_t  g_use_spi_speed;      /* speed source: 1=SPI, 0=TIM3        */
      extern volatile float    g_enc_speed_rpm;      /* SPI-derived mech speed (rpm)       */
      extern volatile int16_t  g_dt_comp;            /* dead-time comp magnitude (s16 V)   */
      extern volatile int16_t  g_avg_iq;             /* avg Iq over ~0.3s (DC)             */
      extern volatile int16_t  g_avg_id;             /* avg Id over ~0.3s: !=0 => misaligned */
      extern volatile uint8_t  g_cogg_enable;        /* anti-cogging FF on/off             */
      extern volatile uint8_t  g_cogg_harm_enable;   /* harmonic denoise on/off ('h')      */
      extern volatile uint8_t  g_cogg_dir_en;        /* direction-split on/off ('n')       */
      extern volatile int16_t  g_cogg_clamp;         /* anti-cogging FF clamp (s16)        */
      extern volatile uint8_t  g_fw_enable;          /* flux weakening on/off              */
      extern volatile float    g_fw_speed_thr_rpm;   /* FW speed threshold (rpm)           */
      extern volatile float    g_fw_hyst_rpm;        /* FW engage hysteresis (rpm)         */
      extern volatile float    g_fw_id_target_a;     /* FW Id target (A, negative)         */
      extern volatile float    g_fw_id_now_a;        /* FW Id ACTUALLY applied (A, 0=idle) */
      extern volatile float    g_spdcap_rpm;         /* torque-mode speed cap (rpm, 0=off) */
      idx_log = 0U;
      LOG_Printf("spi: err=%lu spd=%d | PI Kp=%d Ki=%d dt=%d | avgIq=%d avgId=%d | cogg=%s harm=%s dir=%s clamp=%d | "
                 "fw=%s thr=%dr hys=%dr Id*=%dmA Idnow=%dmA | cap=%dr\r\n",
                 (unsigned long)g_spi_err_count, (int)spd_now,
                 (int)PID_GetKP(&PIDSpeedHandle_M1), (int)PID_GetKI(&PIDSpeedHandle_M1),
                 (int)g_dt_comp, (int)g_avg_iq, (int)g_avg_id,
                 (g_cogg_enable ? "ON" : "off"), (g_cogg_harm_enable ? "ON" : "off"), (g_cogg_dir_en ? "ON" : "off"), (int)g_cogg_clamp,
                 (g_fw_enable ? "ON" : "off"), (int)g_fw_speed_thr_rpm, (int)g_fw_hyst_rpm,
                 (int)(g_fw_id_target_a * 1000.0f), (int)(g_fw_id_now_a * 1000.0f),
                 (int)g_spdcap_rpm);
      {
        extern volatile uint8_t g_mcfw_enable;
        extern int16_t Ropetow_McFwAvVolt(void);
        extern int16_t Ropetow_McFwVTarget(void);
        extern volatile float   g_enc_ff_ticks;
        extern volatile uint8_t g_pll_enable;
        extern volatile float   g_pll_fn_hz;
        extern volatile float   g_pll_lock_err;
        LOG_Printf("mcfw=%s avV=%d/%d Idref=%d | ff=%d | pll=%s/%dHz e=%d\r\n",
                   (g_mcfw_enable ? "ON" : "off"),
                   (int)Ropetow_McFwAvVolt(), (int)Ropetow_McFwVTarget(),
                   (int)FOCVars[M1].Iqdref.d, (int)(g_enc_ff_ticks * 10.0f),
                   (g_pll_enable ? "ON" : "off"), (int)g_pll_fn_hz, (int)g_pll_lock_err);
      }
    }
    }  /* end else-if (cal idle) */
    }  /* end cal-state telemetry gate */
    LOG_Process();
    osDelay(APP_LOOP_PERIOD_MS);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startMediumFrequencyTask */
/**
  * @brief  Function implementing the mediumFrequency thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startMediumFrequencyTask */
__weak void startMediumFrequencyTask(void const * argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
