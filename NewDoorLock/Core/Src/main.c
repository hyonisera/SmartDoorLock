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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATE1 2
#define STATE2 0
#define STATE3 1

#define EEPROM_ADDR 0xA0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t state = STATE1;
GPIO_PinState KEYPAD1, KEYPAD2, KEYPAD3, KEYPAD4, KEYPAD5, KEYPAD6, KEYPAD7;
uint8_t data = 0;
uint8_t table[] = {'1', '4', '7', '*', '2', '5', '8', '0', '3', '6', '9', '#'};
char tx_buf[30];
uint32_t last_key_time = 0;
uint8_t chk = 0;

uint8_t initial_password[] = {'1', '2', '3', '4', '5', '6', '7', '8'};
uint8_t read_password[8] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
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
	uint32_t current_time = HAL_GetTick();
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, 0, 2, (uint8_t*)read_password, sizeof(read_password), 1000) != HAL_OK) {
	  sprintf(tx_buf, "Error reading password from EEPROM.\n\r");
  }

  uint8_t needs_initialization = 1;
  for(int i = 0; i < 8; i++) {
	  if(read_password[i] != 0xFF) {
		  needs_initialization = 0;
		  break;
	  }
  }

  if(needs_initialization) {
	  if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, 0, 2, initial_password, sizeof(initial_password), 1000) == HAL_OK) {
		  sprintf(tx_buf, "Initial password '12345678' written to EEPROM\n\r");
	  }
	  HAL_Delay(10);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GetTick() - current_time >= 10) {		// 10ms마다 state 변경
		  current_time = HAL_GetTick();

		  if(chk == 0) {
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

		  KEYPAD1 = HAL_GPIO_ReadPin(KEYPAD1_GPIO_Port, KEYPAD1_Pin);
		  KEYPAD2 = HAL_GPIO_ReadPin(KEYPAD2_GPIO_Port, KEYPAD2_Pin);
		  KEYPAD3 = HAL_GPIO_ReadPin(KEYPAD3_GPIO_Port, KEYPAD3_Pin);
		  KEYPAD4 = HAL_GPIO_ReadPin(KEYPAD4_GPIO_Port, KEYPAD4_Pin);

		  data = 0;
		  data |= (KEYPAD1 << 0) | (KEYPAD2 << 1) | (KEYPAD3 << 2) | (KEYPAD4 << 3);

		  if(data != 0x0F && chk == 0) {
			  if((HAL_GetTick() - last_key_time) > 50) {
				  last_key_time = HAL_GetTick();

				  uint8_t index = -1;
				  uint8_t exponent = 0;

				  data = (~data & 0x0F);		// 반전한 값을 0x0F와 &연산해서 1, 2, 4, 8 뽑아냄

				  while(data > 1) {				// 지수 0, 1, 2, 3 뽑아냄
					  data >>= 1;
					  exponent++;
				  }

				  index = state * 4 + exponent;

				  sprintf(tx_buf, "%c\n\r", table[index]);
				  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, strlen(tx_buf), 100);
			  }
			  chk = 1;
		  }
		  else if(data == 0x0F && chk == 1) {
			  chk = 0;
		  }
	  }

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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /*Configure GPIO pins : KEYPAD6_Pin KEYPAD5_Pin */
  GPIO_InitStruct.Pin = KEYPAD6_Pin|KEYPAD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD1_Pin KEYPAD2_Pin KEYPAD3_Pin KEYPAD4_Pin */
  GPIO_InitStruct.Pin = KEYPAD1_Pin|KEYPAD2_Pin|KEYPAD3_Pin|KEYPAD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD7_Pin */
  GPIO_InitStruct.Pin = KEYPAD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KEYPAD7_GPIO_Port, &GPIO_InitStruct);

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
