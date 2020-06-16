/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "CanLibrary.h"
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
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef CanTxHeader;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void CAN_Tx(uint32_t CanID, uint8_t CanDLC, uint8_t CANmsg[]);
void CAN_Rx(void);
void CAN_Filter_Conifg(void);
void Print_CAN_Frame(char CanFrameName[], uint32_t CanID, uint32_t CanDlc, uint8_t CanMsg[]);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
//void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CAN_MessageTypeDef CANsampleData;

uint8_t CANmsg[8] = {0};
uint8_t CANmsgPrintTx[8] = {0}; // char table only for printing from interrupts.

/* NOT USED
uint8_t CANmsg_KeyON[] = {0x50,0x10};
uint8_t CANmsg_EnvCond[] = {0xAA,0x66,0x00,0x00};
 */

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
  MX_CAN_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Conifg();

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING  | CAN_IT_BUSOFF) != HAL_OK )
  {
	  Error_Handler("CAN interrupts activation error");
  }

  if ((HAL_CAN_Start(&hcan)) != HAL_OK)
  {
	  Error_Handler("CAN start error");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	CAN_Tx(IPC_StatusBCM.ID, IPC_StatusBCM.DLC, IPC_StatusBCM.CAN_Tx);
	HAL_Delay(50);

	CAN_Tx(IPC_SeatBelts.ID, IPC_SeatBelts.DLC, IPC_SeatBelts.CAN_Tx);
	HAL_Delay(50);

	CAN_Tx(IPC_EngineInfo.ID, IPC_EngineInfo.DLC, IPC_EngineInfo.CAN_Tx);
	HAL_Delay(50);

	CAN_Tx(IPC_StatusB_EPS.ID, IPC_StatusB_EPS.DLC, IPC_StatusB_EPS.CAN_Tx);
	HAL_Delay(50);

	CAN_Tx(IPC_StatusB_BSM.ID, IPC_StatusB_BSM.DLC, IPC_StatusB_BSM.CAN_Tx);
	HAL_Delay(50);

	CAN_Tx(IPC_SpeedOdometerInfo.ID, IPC_SpeedOdometerInfo.DLC, IPC_SpeedOdometerInfo.CAN_Tx);
	HAL_Delay(50);

	CAN_Tx(IPC_Ligths.ID, IPC_Ligths.DLC, IPC_Ligths.CAN_Tx);
	HAL_Delay(50);

	CAN_Tx(IPC_HeartBeat.ID, IPC_HeartBeat.DLC, IPC_HeartBeat.CAN_Tx);
	HAL_Delay(50);

	HAL_Delay(20);

	/*
	  CAN_Tx(0xC3D4000,2,CANmsg_KeyON);
	  HAL_Delay(100);
	  CAN_Tx(0x63D4000,4,CANmsg_EnvCond);
	  HAL_Delay(100);
	*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler("System Clock initialization error");
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler("System Clock Config error");
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
	  Error_Handler("System Clock Config error");
  }
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
	/**
	 *  DWI: Paramters described below are based on http://www.bittiming.can-wiki.info/
	 *  Paramters were adjusted for CAN LowSpeed Network in FIAT GRANDE PUNTO: 50kbps
	 */
	 
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 45;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  /* USER CODE END CAN_Init 1 */
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE; // DWI: New message will overwrite existing message in FIFO memory
  hcan.Init.TransmitFifoPriority = DISABLE; // DWI: Priority driven by the identifier of the message
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler("CAN initialization error");
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler("TIMER 6 initialization error");
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
	Error_Handler("TIMER 6 configuration error");
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
	 Error_Handler("TIMER 7 initialization error");
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
	  Error_Handler("TIMER 7 configuration error");
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 500000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler("USART 2 initialization error");
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief DWI: Print CAN message on UART
  *
  * @param CanFrameName[] - name of CAN frame as string (f.e.: "Tx","Rx",...)
  * @param CanID - identifier of CAN frame
  * @param CanDLC - data length of CAN frame
  * @param CanMsg[] - usefull data of CAN frame
  *
  * @retval Function prints CAN message on UART
  */
void Print_CAN_Frame(char CanFrameName[], uint32_t CanID, uint32_t CanDlc, uint8_t CANmsg[])
{
	char buffer[100] = {0};
	sprintf(buffer,"CAN_%s| ID:0x%02X| DLC:%d| FRAME: ",CanFrameName,(unsigned int)CanID,(unsigned int)CanDlc); // DWI: Initialize first few elements of frame
	for (int i=0; i<CanDlc; i++)
	{
		sprintf(buffer+strlen(buffer),"%02X ",*(CANmsg+i)); // print all DATA elements one by one
	}
	sprintf(buffer+strlen(buffer),"\n\r"); // add in the end of each frame new line and ....
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief by DWI: PeroidElapseCallback to callback after definied perioid of time elapse. Driver will call this function.
  * @param None
  * @retval None
  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}

/**
  * @brief by DWI: CAN TX handle function for CAN Extended
  * @param None
  * @retval None
  */

void CAN_Tx(uint32_t CanID, uint8_t CanDLC, uint8_t CANmsg[])
{
	uint32_t pTxMailbox;
	CanTxHeader.DLC = CanDLC;
	CanTxHeader.ExtId = CanID;
	CanTxHeader.IDE = CAN_ID_EXT;
	CanTxHeader.RTR = CAN_RTR_DATA;

	if ((HAL_CAN_AddTxMessage(&hcan, &CanTxHeader, CANmsg, &pTxMailbox)) != HAL_OK )
	{
		Error_Handler("CAN TX error");
	}

/* KAPI:
 *  ej, potrzebuje ramki z CANmsg[] do przerwania gdzie mi pisze po UART.
 * wymkiniłem żeby to przepisywać na chwile do zmiennej globalnej
 * masz jakiś pomysł jak to lepiej ogarnac bo teraz jest chyba tak se :/
*/
	for (int i=0; i < CanDLC; i++)
	{
		CANmsgPrintTx[i] = CANmsg[i]; // DWI: Only copy value of CANmsg to global variable for UART communication purposes.
	}

//	while(HAL_CAN_IsTxMessagePending(&hcan, pTxMailbox));
//	Print_CAN_Frame("Tx",CanTxHeader.ExtId, CanTxHeader.DLC, CANmsg);

}

/**
  * @brief by DWI: CAN RX handle function
  * @param None
  * @retval None
  */
void CAN_Rx(void)
{
/* DWI: Communication based on interrupts have been implemented. Function no longer needed.

	CAN_RxHeaderTypeDef CanRxHeader;
	uint8_t CANmsgRCVbuffer[8] = {};

	// DWI: wait for minimum one message in RX FIFO memory
	while(!(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)));

	// DWI: Function to handle receiving messages.
	if ((HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &CanRxHeader, CANmsgRCVbuffer)) != HAL_OK)
	{
		Error_Handler();
	}
	Print_CAN_Frame("Rx",CanRxHeader.ExtId, CanRxHeader.DLC, CANmsgRCVbuffer);
*/
}

/**
  * @brief by DWI: CAN Acceptance Filter Configuration - v.1: Accept all frames
  * @param None
  * @retval None
  */
void CAN_Filter_Conifg(void)
{
	CAN_FilterTypeDef CANfilter;


	CANfilter.FilterBank = 0;
	CANfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CANfilter.FilterIdHigh = 0x0000;
	CANfilter.FilterIdLow = 0x0000;
	CANfilter.FilterMaskIdHigh = 0x0000;
	CANfilter.FilterMaskIdLow = 0x0000;
	CANfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	CANfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	CANfilter.FilterActivation = CAN_FILTER_ENABLE;

	if (HAL_CAN_ConfigFilter(&hcan, &CANfilter) != HAL_OK)
	{
		Error_Handler("CAN Filter configuration error");
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	Print_CAN_Frame("Tx0",CanTxHeader.ExtId, CanTxHeader.DLC, CANmsgPrintTx);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	Print_CAN_Frame("Tx1",CanTxHeader.ExtId, CanTxHeader.DLC, CANmsgPrintTx);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	Print_CAN_Frame("Tx2",CanTxHeader.ExtId, CanTxHeader.DLC, CANmsgPrintTx);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef CanRxHeader;
	uint8_t CANmsgRCVbuffer[8] = {};

	// DWI: wait for minimum one message in RX FIFO memory
	// while(!(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)));

	// DWI: Function to handle receiving messages.
	if ((HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxHeader, CANmsgRCVbuffer)) != HAL_OK)
	{
		Error_Handler("CAN RX error");
	}
	Print_CAN_Frame("Rx ",CanRxHeader.ExtId, CanRxHeader.DLC, CANmsgRCVbuffer);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	char buffer[50] = {0};
	sprintf(buffer, "************* CAN ERRROR *************\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(char ErrorName[])
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	char buffer[100] = {0};
	sprintf(buffer, "************* ERRROR: %s *************\n\r",ErrorName);

	while(1)
	{
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	HAL_Delay(500);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
