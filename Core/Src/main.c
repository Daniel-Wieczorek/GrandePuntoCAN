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
#include "stm32f3xx_it.h"
#include "CanLibrary.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f3xx_hal.h"
#include "stm32f303xe.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0
#define FIFO_BUFFER g_rxFifo;
#define USART_HEADER_LENGTH 6
#define USART_ID_LENGTH 7
#define USART_DLC_LENGTH 1
#define USART_MAX_MESSAGE_LENGTH 17
#define USART_DLC_LOCATION 23
#define LENGTH_CARRIGE_RETURN_SIGN 1
#define MAX_BUFFER_LENGTH 70
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
CAN_MessageTypeDef canUartBuffer;
uint8_t timer200ms = 0;
uint8_t timer1000ms = 0;
uint8_t bufferUINT8[70];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void SystemClockConfig(void);
void UART2_Init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void Clear_Array(uint8_t array[], uint16_t size);
void Print_CAN_Frame(char CanFrameName[], uint32_t CanID, uint32_t CanDlc, uint8_t CANmsg[]);
void Parse_From_UART(char CanFrame[]);
uint8_t* Convert_To_HEX(char *string);
void Save_Data_To_CAN_Frame(CAN_MessageTypeDef *canBuffer);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CAN_MessageTypeDef CANsampleData;
uint8_t CANmsg[8] = {0};
uint8_t CANmsgPrintTx[8] = {0}; // char table only for printing from interrupts.
uint8_t data_buffer[MAX_BUFFER_LENGTH];
uint16_t count = 0;
uint8_t rcvd_data;
uint8_t flag_UART_TX_COMPLETED = FALSE;
uint8_t flag_UART_RX_COMPLETED = FALSE;
uint8_t flag_UART_SEND_DATA = TRUE;
CAN_MessageTypeDef canUartBuffer;
HAL_UART_StateTypeDef state;
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
HAL_TIM_Base_Start_IT(&htim6);
HAL_UART_Receive_IT(&huart2, &rcvd_data, 1); // przerwanie obslugujące wiadomosci przychodzace po UART.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  if (flag_UART_SEND_DATA == TRUE)
	  	  {
		  if (timer200ms >= 1) // When 200ms elapses
			  {
			  	  /* TEST DATA 200ms */
			CAN_Tx(IPC_StatusBCM.ID, IPC_StatusBCM.DLC, IPC_StatusBCM.CAN_Tx);
	  	    HAL_Delay(10);

	        CAN_Tx(IPC_Ligths.ID, IPC_Ligths.DLC, IPC_Ligths.CAN_Tx);
	  	    HAL_Delay(10);

	        CAN_Tx(IPC_Vehicle_Setup.ID, IPC_Vehicle_Setup.DLC, IPC_Vehicle_Setup.CAN_Tx);
	  	    HAL_Delay(10);

	        CAN_Tx(STATUS_IPC.ID, STATUS_IPC.DLC, STATUS_IPC.CAN_Tx);
	  	    HAL_Delay(10);
        /*
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

          Print_CAN_Frame("Tx", IPC_Ligths.ID, IPC_Ligths.DLC, IPC_Ligths.CAN_Tx);
          HAL_Delay(1000);

        */
			  	 timer200ms = 0;

			  }
		  else if (timer1000ms >= 5) // When 1000ms elapses
			  {
			  	  /* TEST DATA 1000ms */
				  

				  timer200ms = 0;
				  timer1000ms = 0;

			  }
	  	 }
    while((flag_UART_RX_COMPLETED && flag_UART_TX_COMPLETED))
	  		{
	  			if (data_buffer[0] == 'S' || data_buffer[0] =='E') // if received data is START / END tx Transmission:
	  			{
	  				Clear_Array(data_buffer, (uint16_t)(strlen(data_buffer)));
	  				count = 0;
	  				flag_UART_RX_COMPLETED = FALSE;
	  				flag_UART_TX_COMPLETED = FALSE;
	  				HAL_UART_Receive_IT(&huart2, &rcvd_data, 1);
	  			}
	  			else
	  			{
				//	HAL_Delay(10); // Wait for message being sent fully
				//	Parse_From_UART(data_buffer);
					CAN_Tx(canUartBuffer.ID, canUartBuffer.DLC,canUartBuffer.CAN_Tx);
					Save_Data_To_CAN_Frame(&canUartBuffer);
					Clear_Array(data_buffer, MAX_BUFFER_LENGTH);
					count = 0;
					flag_UART_RX_COMPLETED = FALSE;
					flag_UART_TX_COMPLETED = FALSE;
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

//  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

   /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 224; // 200ms Interrupt
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = (63999);
 htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler("TIMER 6 initialization error");
  }
 // sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 // sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 // if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
 // {
//	Error_Handler("TIMER 6 configuration error");
 // }
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
#if 0
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
#endif
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
	char buffer[70] = {0};
	sprintf(buffer,"CAN_%s| ID:0x%02X| DLC:%d| FRAME: ",CanFrameName,(unsigned int)CanID,(unsigned int)CanDlc);
	for (uint16_t i = 0; i<CanDlc; i++)
		{
			sprintf(buffer+strlen(buffer),"%02X ",*(CANmsg+i)); // SAVE all DATA elements one by one
		}
	sprintf(buffer+strlen(buffer),"\n\r");

	/* WORKAROUND: HAL_UART_Transmit_IT have probelms to accept char array ( even typecased by memcpy() ). Manual typecasting implemented */
	Clear_Array(bufferUINT8, 70);
	for (uint8_t i = 0U; i<strlen(buffer);i++)
		{
			bufferUINT8[i] = (uint8_t)buffer[i];
		}

	if (HAL_UART_Transmit_IT(&huart2, bufferUINT8, strlen(buffer)) != HAL_OK)
	 	 {
	 		HAL_Delay(10);
	 		//Error_Handler();
	 	 }
}

/**
  * @brief by DWI: PeroidElapseCallback to callback after definied perioid of time elapse. Driver will call this function.
  * @param None
  * @retval None
  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	timer200ms=timer200ms+1;
	timer1000ms=timer1000ms+1;

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

	for (int i=0; i < CanDLC; i++)
	{
		CANmsgPrintTx[i] = CANmsg[i]; // DWI: Only copy value of CANmsg to global variable for UART communication purposes.
	}

//	while(HAL_CAN_IsTxMessagePending(&hcan, pTxMailbox));
//	Print_CAN_Frame("Tx",CanTxHeader.ExtId, CanTxHeader.DLC, CANmsg);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (rcvd_data == '\r')
	{
		data_buffer[count++] = '\r';
		flag_UART_RX_COMPLETED = TRUE;
		if (data_buffer[0] == 'S' && data_buffer[1] == 'T')
			{
				flag_UART_SEND_DATA = TRUE;
			}
		else if ((data_buffer[0] == 'E' && data_buffer[1] == 'D') || (data_buffer[0] == 'E' && data_buffer[1] == 'E'))
			{
				flag_UART_SEND_DATA = FALSE;
				HAL_UART_Transmit_IT(&huart2, data_buffer, count);
				while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_BUSY_TX);
				count = 0;
				HAL_UART_Receive_IT(&huart2, &rcvd_data, 1);
				return;
			}

		/* Waiting for ready of USART 2 port. Without waiting (retrasmission) code working better */
		while ((HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) && (flag_UART_TX_COMPLETED != 1));
		if (HAL_UART_Transmit_IT(&huart2, data_buffer, count) != HAL_OK)
		{
			 HAL_Delay(10); // if HAL_BUSY wait 10ms and wait for retransmission
			// Error_Handler(); // if HAL_ERROR
		}
		count = 0;
	}
	else
	{
		data_buffer[count++] = rcvd_data;
	}
	HAL_UART_Receive_IT(&huart2, &rcvd_data, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
		flag_UART_TX_COMPLETED = TRUE;
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
	if (1)
	{
		Print_CAN_Frame("Tx0",CanTxHeader.ExtId, CanTxHeader.DLC, CANmsgPrintTx);
	}
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (1)
	{
		Print_CAN_Frame("Tx1",CanTxHeader.ExtId, CanTxHeader.DLC, CANmsgPrintTx);
	}
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (1)
	{
		Print_CAN_Frame("Tx2",CanTxHeader.ExtId, CanTxHeader.DLC, CANmsgPrintTx);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (flag_UART_SEND_DATA == TRUE)
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
		HAL_Delay(5);
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	char buffer[50] = {0};
	sprintf(buffer, "************* CAN ERRROR *************\n\r");
	//HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
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
	//HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	//HAL_Delay(500);
	}

  /* USER CODE END Error_Handler_Debug */
}

void Clear_Array(uint8_t array[], uint16_t size)
{
	for (int i = 0; i < size; ++i)
	{
		array[i] = 0;
	}
}

void Parse_From_UART(char CanFrame[]) {
    char headerBuffer[USART_HEADER_LENGTH+LENGTH_CARRIGE_RETURN_SIGN] = {0};
    uint8_t index = 0;
    memcpy(headerBuffer, CanFrame, USART_HEADER_LENGTH);

    if (strcmp(headerBuffer, "CAN_Tx") == 0)
    {
    	uint8_t bufferIndex = 0;
        char idBuffer[USART_ID_LENGTH] = {0};
        char dlcBuffer[USART_DLC_LENGTH] = {0};
        char messageBuffer[USART_MAX_MESSAGE_LENGTH+LENGTH_CARRIGE_RETURN_SIGN] = {0};
        uint8_t *pDataConverter;

        memcpy(idBuffer,CanFrame+11,USART_ID_LENGTH);
        dlcBuffer[0] = CanFrame[24];
		for (index = 35; CanFrame[index] != '\r'; ++index)
		{
			if (CanFrame[index] == ' ')
				continue;
			else
			{
				messageBuffer[bufferIndex] = CanFrame[index];
				bufferIndex++;
			}
		}
        canUartBuffer.DLC = dlcBuffer[0] - '0';
        sscanf(idBuffer, "%lX", &canUartBuffer.ID);
        pDataConverter = Convert_To_HEX(messageBuffer);
        for (size_t var = 0; var < canUartBuffer.DLC; var++)
        {
        	canUartBuffer.CAN_Tx[var] = *(pDataConverter+var);
        }
    }
		else if (strcmp(headerBuffer, "_RESET") == 0)
		{
			NVIC_SystemReset();								//Perorm reset of a Nucleo board
		}
		else if (strcmp(headerBuffer, "FIL_CF") == 0)
		{
			printf("Set Filer\n"); // ToDo: Perform some basic operations for filters
		}
		else
		{
			//Error_Handler();
		}
}

uint8_t* Convert_To_HEX(char *string) {
	static uint8_t val[MAX_CAN_MESSAGE_SIZE];
	memset(val, 0, MAX_CAN_MESSAGE_SIZE * sizeof(uint8_t));

	uint8_t index = 0;
	while (*string) {
		// get current character then increment
		uint8_t byte = *string++;
		// transform hex character to the 4bit equivalent number, using the ascii table indexes
		if (byte >= '0' && byte <= '9')
			byte = byte - '0';
		else if (byte >= 'a' && byte <= 'f')
			byte = byte - 'a' + 10;
		else if (byte >= 'A' && byte <= 'F')
			byte = byte - 'A' + 10;
		// shift 4 to make space for new digit, and add the 4 bits of the new digit
		val[index / 2] = (val[index / 2] << 4) | (byte & 0xF);
		index++;
	}
	return val;
}

void Save_Data_To_CAN_Frame(CAN_MessageTypeDef *canBuffer)
{
	// ToDo: switch case implementation

	switch (canBuffer->ID) {
	case (0x2214000):									// IPC Ligths
		IPC_Ligths.DLC = canBuffer->DLC;
		for (uint8_t i = 0; i < IPC_Ligths.DLC; ++i) {
			IPC_Ligths.CAN_Tx[i] = canBuffer->CAN_Tx[i];
		}
		break;
	case (0x4294000): 									// IPC SpeedOdometerInfo
			IPC_SpeedOdometerInfo.DLC = canBuffer->DLC;
			for (uint8_t i = 0; i < IPC_SpeedOdometerInfo.DLC; ++i) {
				IPC_SpeedOdometerInfo.CAN_Tx[i] = canBuffer->CAN_Tx[i];
			}
			break;
	case (0x6214000):									// IPC StatusBCM
			IPC_StatusBCM.DLC = canBuffer->DLC;
			for (uint8_t i = 0; i < IPC_StatusBCM.DLC; ++i) {
				IPC_StatusBCM.CAN_Tx[i] = canBuffer->CAN_Tx[i];
			}
			break;
	case (0x6314003):									// IPC StatusBCM
				STATUS_IPC.DLC = canBuffer->DLC;
			for (uint8_t i = 0; i < STATUS_IPC.DLC; ++i) {
				STATUS_IPC.CAN_Tx[i] = canBuffer->CAN_Tx[i];
			}
			break;
	default:
		break;
	}
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
