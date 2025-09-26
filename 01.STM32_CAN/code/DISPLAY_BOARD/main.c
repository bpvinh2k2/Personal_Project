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
#include "LCD_driver.h"
#include <stdio.h>
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
CAN_HandleTypeDef hcan;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
volatile uint8_t 	gear 			= 0;
volatile uint16_t 	acc_pedal_pos 	= 0;
volatile uint16_t 	app_percent 	= 0;
volatile int16_t 	rpm 			= 0;
char 				gearChar 		= 'N';
volatile int16_t  	i_mA 			= 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static inline uint16_t Acc_Pedal_Pos(uint16_t app);
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
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  LCD_Init(&hi2c1);
  LCD_Clear();

  /* Configure CAN filter to receive ID 0x123 */
  CAN_FilterTypeDef f0;
  f0.FilterActivation 		= ENABLE;
  f0.FilterBank 			= 0;             			// Use filter bank 0
  f0.FilterFIFOAssignment 	= CAN_FILTER_FIFO0; 		// Send to FIFO0
  f0.FilterIdHigh 			= (0x123 << 5);     		// 11-bit ID shifted by 5
  f0.FilterIdLow 			= 0;
  f0.FilterMaskIdHigh 		= (0x7FF << 5);         	// Exact match mask
  f0.FilterMaskIdLow 		= 0;
  f0.FilterMode 			= CAN_FILTERMODE_IDMASK;	// Mask mode
  f0.FilterScale 			= CAN_FILTERSCALE_32BIT;	// 32-bit scale
  HAL_CAN_ConfigFilter(&hcan, &f0);

  /* Configure CAN filter to receive ID 0x125 */
  CAN_FilterTypeDef f1 		= f0;
  f1.FilterBank 			= 1;
  f1.FilterIdHigh 			= (0x125 << 5);
  f1.FilterIdLow  			= 0;
  HAL_CAN_ConfigFilter(&hcan, &f1);

  /* Start the CAN peripheral */
  HAL_CAN_Start(&hcan);

  /* Enable interrupt when a message is pending in FIFO0 */
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	gearChar = (gear == 0) ? 'N' : (gear == 1 ? 'D' : 'R');
	app_percent = Acc_Pedal_Pos(acc_pedal_pos);

	uint16_t rpm_disp = (rpm > 9999u) ? 9999u : (uint16_t)rpm;
	uint32_t A_i      = (uint32_t)(i_mA / 1000);
	if (A_i > 9u) A_i = 9u;
	uint32_t A_d2     = (uint32_t)((i_mA % 1000u) / 10u);
	if (A_d2 > 99u) A_d2 = 99u;

	char row[17];
	int n;

	n = snprintf(row, sizeof(row), "Gear:%c   RPM:%3u",
	             gearChar, (unsigned)rpm_disp);
	if (n < 0) n = 0;
	for (int k = n; k < 16; ++k) row[k] = ' ';
	row[16] = '\0';
	LCD_SetCursor(0, 0);
	LCD_SendString(row);

	n = snprintf(row, sizeof(row), "Gas:%3u%% I:%u.%02uA",
	             (unsigned)app_percent, (unsigned)A_i, (unsigned)A_d2);
	if (n < 0) n = 0;
	for (int k = n; k < 16; ++k) row[k] = ' ';
	row[16] = '\0';
	LCD_SetCursor(1, 0);
	LCD_SendString(row);

	HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* This callback is automatically called when a CAN message is received in FIFO0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* Read the message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
			return;

	if (rxHeader.IDE != CAN_ID_STD || rxHeader.RTR != CAN_RTR_DATA)
	        return;

	if (rxHeader.StdId == 0x123 && rxHeader.DLC >= 2)
    {
        gear = (rxData[0] >> 6) & 0x03;
        acc_pedal_pos  = (uint16_t)((rxData[0] & 0x3F) << 8 | rxData[1]);
        gear &= 0x03;
        acc_pedal_pos &= 0x0FFF;
    }
	else if (rxHeader.StdId == 0x125 && rxHeader.DLC >= 4)
	{
		rpm  = (uint16_t)( ((uint16_t)rxData[0]) | ((uint16_t)rxData[1] << 8) );
		i_mA = (int16_t) ( ((uint16_t)rxData[2]) | ((uint16_t)rxData[3] << 8) );
	}
}

static inline uint16_t Acc_Pedal_Pos(uint16_t app)
{
	if (app < 2200)
		return 0;
	if (app > 3600)
		return 100;
	return (app / 14 - 1100/7);
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
