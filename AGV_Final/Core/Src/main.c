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
#include "stdio.h"
#include "string.h"
#include <stdint.h>
#include <stdbool.h>
#include "modbus_crc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	_RS485_USART									huart2
#define kp	5
#define ki	0
#define kd	0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t		TxBuffer[16];
uint8_t		RxBuffer[32];
int16_t 	pre_distance;
int16_t 		distance;
int32_t		 buffer;
int32_t		 timer=0;
uint16_t 	Data[12];
uint32_t 	avgSpeed = 1000, L_speed, R_speed;
int8_t 		state = 0, eff = 1;
int a;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	Data[0]	 	= 	RxBuffer[3]<<8  | RxBuffer[4];		//LCP1
	Data[1] 	= 	RxBuffer[5]<<8  | RxBuffer[6];		//LCP2
	Data[2] 	= 	RxBuffer[7]<<8  | RxBuffer[8];		//LCP3
	Data[3] 	= 	RxBuffer[9]<<8  | RxBuffer[10];		//#LCP
	Data[4] 	= 	RxBuffer[11]<<8 | RxBuffer[12];		//Status
	Data[5] 	= 	RxBuffer[13]<<8 | RxBuffer[14];		//Line width 1
	Data[6] 	= 	RxBuffer[15]<<8 | RxBuffer[16];		//Line width 2
	Data[7] 	= 	RxBuffer[17]<<8 | RxBuffer[18];		//Line width 3
	Data[8] 	= 	RxBuffer[19]<<8 | RxBuffer[20];		//Magnetic field strength of track 1
	Data[9] 	= 	RxBuffer[21]<<8 | RxBuffer[22];		//Magnetic field strength of track 2
	Data[10] 	= 	RxBuffer[23]<<8 | RxBuffer[24];		//Magnetic field strength of track 3
	  if(Data[3] == 0)
	  {
		  HAL_GPIO_WritePin(GPIOE, OUT1_Pin, 0);
		  HAL_GPIO_WritePin(GPIOE, OUT2_Pin, 0);
		  HAL_GPIO_WritePin(GPIOE, OUT3_Pin, 0);
		  HAL_GPIO_WritePin(GPIOE, OUT4_Pin, 0);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOE, OUT1_Pin, 1);
		  HAL_GPIO_WritePin(GPIOE, OUT2_Pin, 0);
		  HAL_GPIO_WritePin(GPIOE, OUT3_Pin, 0);
		  HAL_GPIO_WritePin(GPIOE, OUT4_Pin, 1);
	  }
	distance = Data[1];
	buffer = kp*distance + ki*(distance + pre_distance)*0.01/2 + kd*(distance-pre_distance)/0.01;
	L_speed = (avgSpeed - buffer) / eff;
	R_speed = (avgSpeed + buffer) / eff;
	if(	L_speed < 0)	L_speed = 0;
	if( R_speed < 0)	R_speed = 0;
	memset (RxBuffer, 0, 32*sizeof(RxBuffer[0]));
	memset (Data, 0, 12*sizeof(Data[0]));
	HAL_UARTEx_ReceiveToIdle_IT(&_RS485_USART, RxBuffer,32);
}

void sendData(uint8_t *data)
{
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&_RS485_USART, TxBuffer, 8, 500);
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
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
  MX_USART2_UART_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_IT(&_RS485_USART, RxBuffer,32);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

//  TxBuffer[0] = 0x0a;
//  TxBuffer[1] = 0x03;
//  TxBuffer[2] = 0x0f;		//Reading offset line zero point
//  TxBuffer[3] = 0xa1;
//  TxBuffer[4] = 0x00;
//  TxBuffer[5] = 0x01;

//  TxBuffer[0] = 0x0a;
//  TxBuffer[1] = 0x04;
//  TxBuffer[2] = 0x00;
//  TxBuffer[3] = 0xc0;   // Reading Line guidance
//  TxBuffer[4] = 0x00;
//  TxBuffer[5] = 0x0b;
//
//  uint16_t crc = crc16(TxBuffer, 6);
//  TxBuffer[6] = crc&0xFF;   // CRC LOW
//  TxBuffer[7] = (crc>>8)&0xFF;  // CRC HIGH
////
//  sendData(TxBuffer);

  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(state == 0)
	  {
		  TxBuffer[0] = 0x0a;
		  TxBuffer[1] = 0x04;
		  TxBuffer[2] = 0x00;
		  TxBuffer[3] = 0xc0;   // Reading Line guidance
		  TxBuffer[4] = 0x00;
		  TxBuffer[5] = 0x0b;

		  uint16_t crc = crc16(TxBuffer, 6);
		  TxBuffer[6] = crc&0xFF;   // CRC LOW
		  TxBuffer[7] = (crc>>8)&0xFF;  // CRC HIGH

		  sendData(TxBuffer);

		  HAL_Delay(10);

		  memset(TxBuffer, 0, 16*sizeof(TxBuffer[0]));
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); //R_speed		//Line test
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0); //L_speed
	  }

	  else
	  {
		  if(HAL_GPIO_ReadPin(GPIOE, Button_1_Pin) == 0)
		  {
			  if(HAL_GPIO_ReadPin(GPIOE, IN1_Pin) == 0 )
			  {
				  HAL_GPIO_WritePin(GPIOE, OUT1_Pin, 1);
				  HAL_GPIO_WritePin(GPIOE, OUT2_Pin, 0);
			  }
			  else
			  {
				  HAL_GPIO_WritePin(GPIOE, OUT1_Pin, 0);
				  HAL_GPIO_WritePin(GPIOE, OUT2_Pin, 1);
			  }
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, avgSpeed);
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, avgSpeed);
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOE, OUT1_Pin, 0);
			  HAL_GPIO_WritePin(GPIOE, OUT2_Pin, 0);
//			  avgSpeed = 0;
		  }
		  if(HAL_GPIO_ReadPin(GPIOE, Button_2_Pin) == 0)			//Button test
		  {
			  if(HAL_GPIO_ReadPin(GPIOE, IN1_Pin) == 0 )
			  {
				  HAL_GPIO_WritePin(GPIOE, OUT3_Pin, 0);
				  HAL_GPIO_WritePin(GPIOE, OUT4_Pin, 1);
			  }
			  else
			  {
				  HAL_GPIO_WritePin(GPIOE, OUT3_Pin, 1);
				  HAL_GPIO_WritePin(GPIOE, OUT4_Pin, 0);
			  }
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, avgSpeed);
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, avgSpeed);
		  }
		  else
		  {
					HAL_GPIO_WritePin(GPIOE, OUT3_Pin, 0);
					HAL_GPIO_WritePin(GPIOE, OUT4_Pin, 0);
//					avgSpeed = 0;
		  }
	  }
//	  if(HAL_GetTick() - timer > 10 && avgSpeed < 1000)
//	  {
//		  avgSpeed += 100;
//		  timer = HAL_GetTick();
//	  }
	  if( HAL_GPIO_ReadPin(GPIOA, IN14_Pin) == 0)
	  {
		  eff = 2;
	  }
	  else
	  {
		  eff = 1;
	  }
	  while(HAL_GPIO_ReadPin(GPIOE, IN4_Pin) == 1 )
	  {
		  HAL_GPIO_WritePin(GPIOE, OUT1_Pin, 0);
		  HAL_GPIO_WritePin(GPIOE, OUT2_Pin, 0);
		  HAL_GPIO_WritePin(GPIOE, OUT3_Pin, 0);
		  HAL_GPIO_WritePin(GPIOE, OUT4_Pin, 0);
//		  avgSpeed = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS485_EN_Pin|Break_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Break_1_GPIO_Port, Break_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUT1_Pin|OUT2_Pin|OUT3_Pin|OUT4_Pin
                          |OUT5_Pin|OUT6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin Button_1_Pin Button_2_Pin IN4_Pin
                           IN5_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|Button_1_Pin|Button_2_Pin|IN4_Pin
                          |IN5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : IN6_Pin IN7_Pin */
  GPIO_InitStruct.Pin = IN6_Pin|IN7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_EN_Pin Break_2_Pin */
  GPIO_InitStruct.Pin = RS485_EN_Pin|Break_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Break_1_Pin */
  GPIO_InitStruct.Pin = Break_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Break_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT1_Pin OUT2_Pin OUT3_Pin OUT4_Pin
                           OUT5_Pin OUT6_Pin */
  GPIO_InitStruct.Pin = OUT1_Pin|OUT2_Pin|OUT3_Pin|OUT4_Pin
                          |OUT5_Pin|OUT6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : IN14_Pin */
  GPIO_InitStruct.Pin = IN14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN14_GPIO_Port, &GPIO_InitStruct);

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
