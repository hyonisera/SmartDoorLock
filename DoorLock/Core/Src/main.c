/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define STATE1	0
#define STATE2	1
#define STATE3	2

#define DEBOUNCE_DELAY	200
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t state = STATE1;
uint8_t chk = 0;

char tx_buf[50];
char num = '\0';

char password[9] = "12345678";
char input_pw[9] = "";
uint8_t pw_index = 0;

uint8_t is_door_lock = 1;
uint8_t is_door_open = 0;
uint8_t setup_pw = 0;
char new_pw[9] = "";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
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
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(is_door_lock == 0) {
//		  TIM4->CCR1 = 1499;
//		  HAL_TIM_Base_Start_IT(&htim2);
//
//		  if(is_door_open == 1) {
//			  HAL_TIM_Base_Stop_IT(&htim2);
//		  }
//		  else {
//			  is_door_lock = 1;
//			  TIM4->CCR1 = 499;
//			  HAL_TIM_Base_Stop_IT(&htim2);
//		  }
//	  }
//	  else {
//		  TIM4->CCR1 = 499;
//	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 159;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KEYPAD6_Pin|KEYPAD5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KEYPAD7_GPIO_Port, KEYPAD7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEYPAD6_Pin KEYPAD5_Pin LD2_Pin */
  GPIO_InitStruct.Pin = KEYPAD6_Pin|KEYPAD5_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD1_Pin KEYPAD2_Pin KEYPAD3_Pin KEYPAD4_Pin */
  GPIO_InitStruct.Pin = KEYPAD1_Pin|KEYPAD2_Pin|KEYPAD3_Pin|KEYPAD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD7_Pin */
  GPIO_InitStruct.Pin = KEYPAD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KEYPAD7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SETUP_SW_Pin DOOR_SW_Pin */
  GPIO_InitStruct.Pin = SETUP_SW_Pin|DOOR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : STATE_SW_Pin */
  GPIO_InitStruct.Pin = STATE_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(STATE_SW_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	GPIO_PinState KEYPAD1, KEYPAD2, KEYPAD3, KEYPAD4, KEYPAD5, KEYPAD6, KEYPAD7;

	if(htim->Instance == TIM3) {
		if(chk == 0) {		// 스캔
			if(state == STATE1) {
				KEYPAD5 = 0; KEYPAD6 = 1; KEYPAD7 = 1;
				state = STATE2;
			}
			else if(state == STATE2) {
				KEYPAD5 = 1; KEYPAD6 = 0; KEYPAD7 = 1;
				state = STATE3;
			}
			else if(state == STATE3) {
				KEYPAD5 = 1; KEYPAD6 = 1; KEYPAD7 = 0;
				state = STATE1;
			}

			HAL_GPIO_WritePin(KEYPAD5_GPIO_Port, KEYPAD5_Pin, KEYPAD5);
			HAL_GPIO_WritePin(KEYPAD6_GPIO_Port, KEYPAD6_Pin, KEYPAD6);
			HAL_GPIO_WritePin(KEYPAD7_GPIO_Port, KEYPAD7_Pin, KEYPAD7);
		}
		else {
			KEYPAD1 = HAL_GPIO_ReadPin(KEYPAD1_GPIO_Port, KEYPAD1_Pin);
			KEYPAD2 = HAL_GPIO_ReadPin(KEYPAD2_GPIO_Port, KEYPAD2_Pin);
			KEYPAD3 = HAL_GPIO_ReadPin(KEYPAD3_GPIO_Port, KEYPAD3_Pin);
			KEYPAD4 = HAL_GPIO_ReadPin(KEYPAD4_GPIO_Port, KEYPAD4_Pin);

			if(KEYPAD1 == 1 && KEYPAD2 == 1 && KEYPAD3 == 1 && KEYPAD4 == 1) {	// 키가 눌리지 않았을 때
				chk = 0;
			}
		}
	}
	else if(htim->Instance == TIM2) {
//		if(is_door_lock == 0 && is_door_open == 0) {
//			is_door_lock = 1;
////			TIM4->CCR1 = 499;
//
//			sprintf(tx_buf, "\n\r시간 초과로 문이 닫혔습니다.\n\r");
//			HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
//
////			HAL_TIM_Base_Stop_IT(&htim2);
//		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t last_interrupt_time;	// 값 유지, 자동으로 0으로 초기화
	uint32_t current_time = HAL_GetTick();

	if((current_time - last_interrupt_time) < DEBOUNCE_DELAY) {
		return;
	}
	last_interrupt_time = current_time;

//	if(is_door_open == 1 && GPIO_Pin == SETUP_SW_Pin) {
//		setup_pw = 1;
//		pw_index = 0;
//		sprintf(tx_buf, "\n\r새 비밀번호를 입력하세요.\n\r");
//		HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
//		return;
//	}

	chk = 1;	// 스캔 중단하고 눌린 키의 상태 확인

	if(GPIO_Pin == KEYPAD1_Pin) {
		if(state == STATE2) num = '1';
		else if(state == STATE3) num = '2';
		else if(state == STATE1) num = '3';
	}
	else if(GPIO_Pin == KEYPAD2_Pin) {
		if(state == STATE2) num = '4';
		else if(state == STATE3) num = '5';
		else if(state == STATE1) num = '6';
	}
	else if(GPIO_Pin == KEYPAD3_Pin) {
		if(state == STATE2) num = '7';
		else if(state == STATE3) num = '8';
		else if(state == STATE1) num = '9';
	}
	else if(GPIO_Pin == KEYPAD4_Pin) {
		if(state == STATE2) num = '*';
		else if(state == STATE3) num = '0';
		else if(state == STATE1) num = '#';
	}

	if(GPIO_Pin == KEYPAD1_Pin || GPIO_Pin == KEYPAD2_Pin || GPIO_Pin == KEYPAD3_Pin || GPIO_Pin == KEYPAD4_Pin) {
		if(num != '*') {
			if(pw_index < 8) {
				input_pw[pw_index++] = num;
				sprintf(tx_buf, "%c", num);
				HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
			}
			else {
				sprintf(tx_buf, "\n\r비밀번호가 틀렸습니다.\n\r");
				HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
				pw_index = 0;
			}
		}
		else {
			input_pw[pw_index] = '\0';

			if(strcmp(input_pw, password) == 0) {
				is_door_lock = 0;		// 문 열림 활성화
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				sprintf(tx_buf, "\n\r비밀번호가 맞았습니다.\n\r");
			}
			else {
				pw_index = 0;
				memset(input_pw, 0, sizeof(input_pw));
				sprintf(tx_buf, "\n\r비밀번호가 틀렸습니다.\n\r");
			}
			HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
			pw_index = 0;
		}
	}

	if(GPIO_Pin == STATE_SW_Pin) {
		if(is_door_lock == 0) {
			GPIO_PinState STATE_SW = HAL_GPIO_ReadPin(STATE_SW_GPIO_Port, STATE_SW_Pin);

			if(STATE_SW == 0) {
				is_door_open = 1;
				sprintf(tx_buf, "\n\r문이 열렸습니다.\n\r");
			}
			else {
				is_door_open = 0;
				sprintf(tx_buf, "\n\r문이 닫혔습니다.\n\r");
				is_door_lock = 1;
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			}
			HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
		}
		else {
			sprintf(tx_buf, "\n\r문이 잠겨있습니다.\n\r");
			HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
		}
	}
//	GPIO_PinState STATE_SW = HAL_GPIO_ReadPin(STATE_SW_GPIO_Port, STATE_SW_Pin);
//
//	if(STATE_SW == 0) {
//		is_door_open = 1;
//
//		sprintf(tx_buf, "\n\r문이 열렸습니다.\n\r");
//		HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
////		if(is_door_lock == 0) {
////			is_door_open = 1;
////			sprintf(tx_buf, "\n\r문이 열렸습니다.\n\r");
////			HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
////			HAL_TIM_Base_Stop_IT(&htim2);
////		}
//	}
//	else {
//		is_door_open = 0;
//		sprintf(tx_buf, "\n\r문이 닫혔습니다.\n\r");
//		HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
//	}

//	if(setup_pw == 1) {
//		if(num != '*') {
//			if(pw_index < 8) {
//				new_pw[pw_index++] = num;
//				sprintf(tx_buf, "%c", num);
//				HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
//			}
//		}
//		else {
//			new_pw[pw_index] = '\0';
//			strcpy(password, new_pw);	// 기존 비밀번호를 새 비밀번호로 교체
//			sprintf(tx_buf, "\n\r비밀번호가 설정되었습니다.\n\r");
//			HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
//			setup_pw = 0;	// 비밀번호 설정 모드 비활성화
//			pw_index = 0;
//		}
//	}
//	else {
//		if(num != '*') {
//			if(pw_index < 8) {
//				input_pw[pw_index++] = num;
//				sprintf(tx_buf, "%c", num);
//				HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
//			}
//		}
//		else {
//			input_pw[pw_index] = '\0';
//
//			if(strcmp(input_pw, password) == 0) {
//				sprintf(tx_buf, "\n\r문이 열립니다.\n\r");
//				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//				is_door_open = 1;
//			}
//			else {
//				sprintf(tx_buf, "\n\r비밀번호가 틀렸습니다.\n\r");
//				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//			}
//			HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
//			pw_index = 0;
//		}
//	}
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
