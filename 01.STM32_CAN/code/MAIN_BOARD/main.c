/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
//=========== CAN
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
volatile uint8_t gear = 0;
volatile uint16_t acc_pedal_pos = 0;
volatile uint32_t encoder = 0;
volatile uint8_t CAN_send_flag = 0;

//=========== Motor
// Mapping ADC with ARR
#define ARR_MAX				3599u 	// TIM2 ARR = 3599  (period = 3599)
#define ADC_ON				2250u 	// threshold to start moving
#define ADC_FULLSCALE		3600u 	// ADC where duty reaches ~100%

// Slew: PWM at 20 kHz, but update CCR every 1 kHz (each 20 PWM ticks)
#define SLEW_STEP_UP		80u 	// max +80 CCR counts per control update (~+2.2% duty)
#define SLEW_STEP_DOWN		160u	// max -160 CCR counts per control update (~-4.4% duty)

// Internal control state
static uint16_t s_ccr		= 0;	// current CCR actually applied
static uint16_t s_ccr_sp	= 0;	// CCR setpoint from acc_pedal_pos

// Direction guard:
// - s_dir_req: requested direction from 'gear' (1=Forward, 2=Reverse)
// - s_dir_now: direction already applied to pins IN1/IN2
static uint8_t s_dir_req 	= 1;
static uint8_t s_dir_now 	= 1;

//=========== Encoder
volatile uint32_t rpm 		= 0;
#define ENCODER_PPR     	20u
#define RPM_WIN_HZ     	 	50u     // TIM6 interrupt rate (Hz)
#define ACCUM_WINDOWS   	12u     // 12 * 20ms = 240ms

//=========== Current sensor
#define CS_ADC_BUF_LEN 		256
#define EMA_SHIFT        	2
volatile uint16_t cs_adc_buf[CS_ADC_BUF_LEN];
volatile int32_t acs712_C0 	= 0;     // offset @0A (counts)
volatile int32_t motor_I_mA = 0;
volatile int32_t adc 		= 0;
volatile int32_t i_mA		= 0;
volatile float i_A			= 0;
volatile uint8_t wait		= 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void ConfigCANFilter(void);
void Motor_Control_1kHz_Task(void);
void Encoder_Start(void);
void CS_Calibration_0A(void);
void CAN_SendRPMCurrent(uint32_t rpm, int32_t i_mA);
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // ============================================== CAN
  // Configure CAN filter
  ConfigCANFilter();
  // Start CAN
  HAL_CAN_Start(&hcan1);

  // Enable interrupt when FIFO0 receive data
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // ============================================== Motor Driver
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // ENA

  // ============================================== Control timer 1 kHz
  HAL_TIM_Base_Start_IT(&htim4);

  // ============================================== Encoder
  Encoder_Start();

  // ============================================== Current sensor
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)cs_adc_buf, CS_ADC_BUF_LEN);
  HAL_Delay(600);
  CS_Calibration_0A();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (CAN_send_flag)
	  {
		  CAN_SendRPMCurrent(rpm, i_mA);
		  CAN_send_flag = 0;
	  }
	  adc = cs_adc_buf[0];
	  HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_IC_Init(&htim5);

  sIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sIC.ICPrescaler = TIM_ICPSC_DIV1;
  sIC.ICFilter    = 2;
  HAL_TIM_IC_ConfigChannel(&htim5, &sIC, TIM_CHANNEL_1);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1439;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IN1_Pin */
  GPIO_InitStruct.Pin = IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN2_Pin */
  GPIO_InitStruct.Pin = IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ============================================== Motor control
// --- Hard-OFF
static inline void Motor_OutputDisable(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	s_ccr = 0;
	s_ccr_sp = 0;
}

// --- Mapping ADC --> CCR
static inline uint16_t CCR_Map_From_ADC(uint16_t adc)
{
	if (adc < ADC_ON)			return 0;
	if (adc >= ADC_FULLSCALE)	return ARR_MAX;
	return adc;
}

// --- Set motor direction
static inline void Motor_SetDIR(uint8_t dir)
{
	if (dir == 1) // Drive
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	}
	else if (dir == 2) // Reverse
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
	}
}

// --- 1 kHz control task (called by TIM4 update interrupt) ---
void Motor_Control_1kHz_Task(void)
{
	uint8_t  g   = gear;
	uint16_t acc = acc_pedal_pos;

	// 0. Hard-OFF
	if (g == 0)
	{
		Motor_OutputDisable();
		return;
	}

	// 1. Update direction
	if (g == 1)			s_dir_req = 1;
	else if (g == 2)	s_dir_req = 2;

	// 2. Build CCR setpoint
	s_ccr_sp = CCR_Map_From_ADC(acc);

	// 3. Safety guard before direction change
	if ((s_dir_req != s_dir_now) && (s_ccr > 0))
	{
		if (s_ccr > SLEW_STEP_DOWN)
			s_ccr -= SLEW_STEP_DOWN;
		else
			s_ccr = 0;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_ccr);
		return;
	}

	// 4. Direction change
	if (s_ccr == 0)
	{
		s_dir_now = s_dir_req;
		Motor_SetDIR(s_dir_now);
	}

	// 5. Slew-rate: update CCR @1 kHz
	int32_t delta = (int32_t)s_ccr_sp - (int32_t)s_ccr;
	if (delta > 0)
	{
		if (delta > (int32_t)SLEW_STEP_UP)		delta = (int32_t)SLEW_STEP_UP;
	}
	else if (delta < 0)
	{
		if (-delta > (int32_t)SLEW_STEP_DOWN)	delta = -(int32_t)SLEW_STEP_DOWN;
	}

	int32_t ccr_new = (int32_t)s_ccr + delta;
	if (ccr_new < 0)							ccr_new = 0;
	if (ccr_new > (int32_t)ARR_MAX)				ccr_new = (int32_t)ARR_MAX;
	s_ccr = (uint16_t)ccr_new;

	// 6. Apply PWM
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_ccr);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
    	static uint8_t div10 = 0;
    	if (++div10 >= 10)
    	{
    		div10 = 0;
    		CAN_send_flag = 1;
    	}
    	Motor_Control_1kHz_Task();
    	return;
    }

    if (htim->Instance == TIM6)
    {
// ============================================== Encoder -> RPM
    	static uint32_t prev = 0;
    	static uint8_t  first_run = 1;
    	static uint32_t accum = 0;
    	static uint32_t win_cnt = 0;
    	static uint16_t idle_cnt = 0;

    	uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);
    	if (first_run)
    	{
    		prev = now;
    		first_run = 0;
    		return;
    	}
    	uint32_t delta = now - prev;
    	prev = now;

    	accum   += delta;
    	win_cnt += 1;

    	idle_cnt = (delta == 0) ? (idle_cnt + 1) : 0;
    	if (idle_cnt > RPM_WIN_HZ)
    	{
    		rpm = 0;
    		accum = 0; win_cnt = 0;
    		first_run = 1;
    		return;
    	}

    	if (win_cnt >= ACCUM_WINDOWS)
    	{
    		// rpm = (accum * 60 * RPM_WIN_HZ) / (ENCODER_PPR * ACCUM_WINDOWS)
    		uint64_t num   = (uint64_t)accum * 60ull * RPM_WIN_HZ;
    		uint32_t denom = ENCODER_PPR * ACCUM_WINDOWS;
    		rpm = (uint32_t)((num + denom/2u) / denom);

    		accum = 0;
    		win_cnt = 0;
    	}
    }
}

void Encoder_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_IC_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim5);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
}

// ============================================== Current sensor
void CS_Calibration_0A(void)
{
	uint32_t s = 0;
	for (uint16_t i = 0; i < 1024; i++)
	{
		s += cs_adc_buf[i % CS_ADC_BUF_LEN];
		HAL_Delay(1);
	}
	acs712_C0 = (int32_t)(s / 1024);
	wait = 1;
}

static inline int32_t ACS712_CountsTo_mA(int32_t dC)
{
	static uint32_t gain_ppm = 1160000;
    // 3.3V, 12-bit, 185 mV/A → 4095*185 = 757575
    int64_t num = (int64_t)dC * 3300000;
    int64_t mA_raw = (num + 757575/2) / 757575;
    return (int32_t)((mA_raw * (int64_t)gain_ppm + 500000) / 1000000);
}

static void process_block(const uint16_t *p, int len)
{
	uint32_t sum = 0;
	for (int i=0; i < len; i++)
		sum += p[i];
	int32_t C = (int32_t)(sum / len);
	int32_t dC = C - acs712_C0;
	int32_t i = ACS712_CountsTo_mA(dC);
	static int32_t ema = 0;
	ema += (i - ema) >> EMA_SHIFT;
	if (ema < 0)
		i_mA = 0;
	else
		i_mA = ema;
	i_A = (float)i_mA/1000;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && wait)
    	process_block((const uint16_t*)&cs_adc_buf[0], CS_ADC_BUF_LEN/2);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && wait)
    	process_block((const uint16_t*)&cs_adc_buf[CS_ADC_BUF_LEN/2], CS_ADC_BUF_LEN/2);
}

// ============================================== CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
	{
		gear = (rxData[0] >> 6) & 0x03;
		acc_pedal_pos = ((uint16_t)(rxData[0] & 0x3F) << 8) | rxData[1];
		gear &= 0x03;
		acc_pedal_pos &= 0x0FFF;
	}
}

void ConfigCANFilter(void)
{
	  CAN_FilterTypeDef canFilterConfig;
	  canFilterConfig.FilterBank = 0;
	  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  canFilterConfig.FilterIdHigh = (0x123 << 5);
	  canFilterConfig.FilterIdLow = 0x0000;
	  canFilterConfig.FilterMaskIdHigh = (0x7FF << 5);
	  canFilterConfig.FilterMaskIdLow = 0x0000;
	  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	  canFilterConfig.FilterActivation = ENABLE;

	  if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK)
	  {
	      Error_Handler();
	  }
}

void CAN_SendRPMCurrent(uint32_t rpm, int32_t i_mA)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox;

    // --- pack rpm (uint16_t) ---
	txData[0] = (uint8_t)(rpm & 0xFF);
	txData[1] = (uint8_t)((rpm >> 8) & 0xFF);

	// --- pack i_mA (int16_t) ---
	txData[2] = (uint8_t)(i_mA & 0xFF);
	txData[3] = (uint8_t)((i_mA >> 8) & 0xFF);

    // Header CAN
    txHeader.StdId = 0x125;
    txHeader.ExtId = 0x00;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 4;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
    	HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
    }
}
/* USER CODE END 4 */

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
