/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "arm.h"
#include "cdc_message.h"
#include "path_command.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#ifndef HSEM_ID_0
//#define HSEM_ID_0 (0U) /* HW semaphore 0*/
//#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t res_buf[1024];
char speed[5], j1b[5], j2b[5], j3b[5], j4b[5], j5b[5], j6b[5];

void ReciveCommand(){
	if (res_buf[0] == 'f'){
		if (res_buf[1] == 'H'){
			FindHomeArm();
		}
		else if (res_buf[1] == '1'){
			FindHomeJoint(JOINT1);
		}
		else if (res_buf[1] == '2'){
			FindHomeJoint(JOINT2);
		}
		else if (res_buf[1] == '3'){
			FindHomeJoint(JOINT3);
		}
		else if (res_buf[1] == '4'){
			FindHomeJoint(JOINT4);
		}
		else if (res_buf[1] == '5'){
			FindHomeJoint(JOINT5);
		}
		else if (res_buf[1] == '6'){
			FindHomeJoint(JOINT6);
		}
	}
	else if (res_buf[0] == 'F'){
		if (res_buf[1] == 'H'){
			FindHomeArmForce();
		}
		else if (res_buf[1] == '1'){
			FindHomeForce(JOINT1);
		}
		else if (res_buf[1] == '2'){
			FindHomeForce(JOINT2);
		}
		else if (res_buf[1] == '3'){
			FindHomeForce(JOINT3);
		}
		else if (res_buf[1] == '4'){
			FindHomeForce(JOINT4);
		}
		else if (res_buf[1] == '5'){
			FindHomeForce(JOINT5);
		}
		else if (res_buf[1] == '6'){
			FindHomeForce(JOINT6);
		}
	}
	else if (res_buf[0] == 's'){
		if (res_buf[1] == '1'){
			ReportJointState(JOINT1);
		}
		else if (res_buf[1] == '2'){
			ReportJointState(JOINT2);
		}
		else if (res_buf[1] == '3'){
			ReportJointState(JOINT3);
		}
		else if (res_buf[1] == '4'){
			ReportJointState(JOINT4);
		}
		else if (res_buf[1] == '5'){
			ReportJointState(JOINT5);
		}
		else if (res_buf[1] == '6'){
			ReportJointState(JOINT6);
		}
		else if (res_buf[1] == 't'){
			ReportJointsState();
		}
	}
	else if (res_buf[0] == 'M'){
		if (res_buf[1] == 'E'){
			strncpy((char *)&j1b, (char *)&res_buf[3], 5);
			strncpy((char *)&j2b, (char *)&res_buf[9], 5);
			strncpy((char *)&j3b, (char *)&res_buf[15], 5);
			strncpy((char *)&j4b, (char *)&res_buf[21], 5);
			strncpy((char *)&j5b, (char *)&res_buf[27], 5);
			strncpy((char *)&j6b, (char *)&res_buf[33], 5);
			SetTargetPointEqSpeed(atoi((char *)&j1b), atoi((char *)&j2b), atoi((char *)&j3b), atoi((char *)&j4b), atoi((char *)&j5b), atoi((char *)&j6b));
		}
		else if (res_buf[1] == 'M'){
			//
		}
		else if (res_buf[1] == 'S'){
			//
		}
	}
	else if (res_buf[0] == 'P'){
		if (res_buf[1] == 'A'){
			strncpy((char *)&speed, (char *)&res_buf[3], 5);
			strncpy((char *)&j1b, (char *)&res_buf[9], 5);
			strncpy((char *)&j2b, (char *)&res_buf[15], 5);
			strncpy((char *)&j3b, (char *)&res_buf[21], 5);
			strncpy((char *)&j4b, (char *)&res_buf[27], 5);
			strncpy((char *)&j5b, (char *)&res_buf[33], 5);
			strncpy((char *)&j6b, (char *)&res_buf[39], 5);
			AddPathPoint(atoi((char *)&speed) / CLOCK_STEP,
					atoi((char *)&j1b),
					atoi((char *)&j2b),
					atoi((char *)&j3b),
					atoi((char *)&j4b),
					atoi((char *)&j5b),
					atoi((char *)&j6b));
		}
		else if (res_buf[1] == 'M'){
			StartMovePath();
		}
		else if (res_buf[1] == 'C'){
			ClearPath();
		}
		else if (res_buf[1] == 'P'){
			PongMessage();
		}
	}
	else if (res_buf[0] == 'Z'){
		if (res_buf[1] == '1'){
			SetBackToZeroJoint(JOINT1, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '2'){
			SetBackToZeroJoint(JOINT2, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '3'){
			SetBackToZeroJoint(JOINT3, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '4'){
			SetBackToZeroJoint(JOINT4, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '5'){
			SetBackToZeroJoint(JOINT5, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '6'){
			SetBackToZeroJoint(JOINT6, atoi((char *)&res_buf[2]));
		}
	}
	else if (res_buf[0] == 'S'){
		if (res_buf[1] == '1'){
			SetFullStepsJoint(JOINT1, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '2'){
			SetFullStepsJoint(JOINT2, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '3'){
			SetFullStepsJoint(JOINT3, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '4'){
			SetFullStepsJoint(JOINT4, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '5'){
			SetFullStepsJoint(JOINT5, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == '6'){
			SetFullStepsJoint(JOINT6, atoi((char *)&res_buf[2]));
		}
		else if (res_buf[1] == 'F'){
			int s = atoi((char *)&res_buf[2]);
			SetFindHomeSpeed((double)s / CLOCK_STEP);
		}
		else if (res_buf[1] == 'S'){
			int s = atoi((char *)&res_buf[2]);
			SetMaxSpeed((double)s / CLOCK_STEP);
		}
	}
	else if (res_buf[0] == 't'){
		if (res_buf[1] == '1'){
			//
		}
		else if (res_buf[1] == '2'){
			//
		}
		else if (res_buf[1] == '3'){
			//
		}
		else if (res_buf[1] == '4'){
			//
		}
		else if (res_buf[1] == '5'){
			//
		}
		else if (res_buf[1] == '6'){
			//
		}
	}
	//if (strcmp((char *)&res_buf, "PP\n") == 0){
	//	PongMessage();
	//}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
    {
		switch(htim->Channel){
			case HAL_TIM_ACTIVE_CHANNEL_1:
			{
				TickTimerChennel1();
				break;
			}
			case HAL_TIM_ACTIVE_CHANNEL_2:
			{
				TickTimerChennel2();
				break;
			}
			default:
				break;
		}
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  //int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  //timeout = 0xFFFF;
  //while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  //if ( timeout < 0 )
  //{
  //Error_Handler();
  //}
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
//__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
//HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
//HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
//timeout = 0xFFFF;
//while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
//if ( timeout < 0 )
//{
//Error_Handler();
//}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  InitArm();
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //CDC_Transmit_FS((uint8_t *)data, strlen(data));
	  bool f = false;
	  f |= HAL_GPIO_ReadPin(JL1_GPIO_Port, JL1_Pin) == GPIO_PIN_RESET ? true : false;
	  f |= HAL_GPIO_ReadPin(JL2_GPIO_Port, JL2_Pin) == GPIO_PIN_RESET ? true : false;
	  f |= HAL_GPIO_ReadPin(JL3_GPIO_Port, JL3_Pin) == GPIO_PIN_RESET ? true : false;
	  f |= HAL_GPIO_ReadPin(JL4_GPIO_Port, JL4_Pin) == GPIO_PIN_RESET ? true : false;
	  f |= HAL_GPIO_ReadPin(JL5_GPIO_Port, JL5_Pin) == GPIO_PIN_RESET ? true : false;
	  f |= HAL_GPIO_ReadPin(JL6_GPIO_Port, JL6_Pin) == GPIO_PIN_RESET ? true : false;
	  if (f){
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
	  }else{
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
	  }
	  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET){
		  //FindHomeJoint(JOINT6);
		  //JointFindedHome(1);
	  }
	  HAL_Delay(1);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, J4P_Pin|J2P_Pin|J4D_Pin|J3P_Pin
                          |J3D_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, J1P_Pin|J2D_Pin|J1D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|J5D_Pin|J5P_Pin|J6D_Pin
                          |J6P_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : J4P_Pin J2P_Pin J4D_Pin J3P_Pin
                           J3D_Pin */
  GPIO_InitStruct.Pin = J4P_Pin|J2P_Pin|J4D_Pin|J3P_Pin
                          |J3D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : J1P_Pin J2D_Pin J1D_Pin */
  GPIO_InitStruct.Pin = J1P_Pin|J2D_Pin|J1D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JL6_Pin JL5_Pin JL4_Pin JL3_Pin
                           JL2_Pin */
  GPIO_InitStruct.Pin = JL6_Pin|JL5_Pin|JL4_Pin|JL3_Pin
                          |JL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : JL1_Pin */
  GPIO_InitStruct.Pin = JL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(JL1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : J5D_Pin J5P_Pin J6D_Pin J6P_Pin */
  GPIO_InitStruct.Pin = J5D_Pin|J5P_Pin|J6D_Pin|J6P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
