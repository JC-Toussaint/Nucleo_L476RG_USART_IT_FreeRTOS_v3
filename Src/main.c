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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	char name[10];
	uint16_t nbArgs;
	void (*pCmdFunc)();
} CMD_T;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

osThreadId UartTaskHandle;
osThreadId LedTaskHandle;
osMessageQId RxQueueHandle;
osMessageQId Queue01Handle;
osSemaphoreId BinarySem01Handle;
/* USER CODE BEGIN PV */

__IO uint16_t cmdIndex=0;
#define DEBUG 1
#define MAXARGS 3 // 3 maximum number of arguments

void CMD_on(   uint16_t nbArgs, int32_t *args );
void CMD_off(  uint16_t nbArgs, int32_t *args );

const CMD_T CmdTable[] = {
		{"on",   0, (void*) CMD_on},
		{"off",  0, (void*) CMD_off},
		{"",     0, NULL}
};

uint8_t rxBuff[256];

static uint8_t rxInProgress = 0;
static uint_fast16_t rxItr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartUartTask(void const * argument);
void StartLedTask(void const * argument);

/* USER CODE BEGIN PFP */

void UART_ReadString(char* str);
void CMD_Parser(char* str);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// printf under freeRTOS
// https://github.com/leech001/STM32-FreeRTOS-float

//1. Remove the heap_4.c file from the
//\Middlewares\Third_Party\FreeRTOS\Source\portable\MemMang\ directory of your project
//2. Copy the heap_useNewlib.c file from the given repository (src) to the above directory;
//3. Open the FreeRTOS configuration file (FreeRTOSConfig.h) and add an additional define to the section;

// /* USER CODE BEGIN Defines */
// /* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
// #define configUSE_NEWLIB_REENTRANT 1
// /* USER CODE END Defines */
#define PRINTF

int _write(int file, char *ptr, int len)
{
	for (uint8_t i=0; i<len; i++){
		LL_USART_TransmitData8(USART2, ptr[i]);
		while ( !LL_USART_IsActiveFlag_TXE(USART2) )
		{
		}
	}
	return len;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySem01 */
  osSemaphoreDef(BinarySem01);
  BinarySem01Handle = osSemaphoreCreate(osSemaphore(BinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  osSemaphoreWait(BinarySem01Handle, osWaitForever);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of RxQueue */
  osMessageQDef(RxQueue, 128, uint8_t);
  RxQueueHandle = osMessageCreate(osMessageQ(RxQueue), NULL);

  /* definition and creation of Queue01 */
  osMessageQDef(Queue01, 16, uint16_t);
  Queue01Handle = osMessageCreate(osMessageQ(Queue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of UartTask */
  osThreadDef(UartTask, StartUartTask, osPriorityIdle, 0, 512);
  UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

  /* definition and creation of LedTask */
  osThreadDef(LedTask, StartLedTask, osPriorityIdle, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),6, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

	/* Polling USART initialisation */
	while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
	{
	}

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
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
  * @brief  This function prints user info on PC com port and initiates RX transfer
  * @param  None
  * @retval None
  */
void StartReception(void)
{
	rxBuff[0]='\0';
	rxItr = 0;
	rxInProgress = 1;

  /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(USART2);

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_ERROR(USART2);
}

/**
 * @brief  Function called from USART IRQ Handler when RXNE flag is set
 *         Function is in charge of reading character received on USART RX line.
 * @param  None
 * @retval None
 */
void USART_CharReception_Callback(void)
{
	__IO uint8_t received_char;

	/* Read Received character. RXNE flag is cleared by reading of RDR register */
	received_char = LL_USART_ReceiveData8(USART2);

	if(rxInProgress)
	{
		rxBuff[rxItr++] = received_char;
		if ((received_char == '\r') || (received_char == '\n')){
			rxInProgress = 0;
			rxBuff[rxItr] = '\0';
			rxItr=0;
			osSemaphoreRelease(BinarySem01Handle);
		}
	}

	/* Echo received character on TX */
	//LL_USART_TransmitData8(USART2, received_char);
}

/**
 * @brief  Function called in case of error detected in USART IT Handler
 * @param  None
 * @retval None
 */
void Error_Callback(void)
{

	while(1) { ; }

}

//void USART2_IRQHandler(void)
//{
//  long xHigherPriorityTaskWoken = pdFALSE;
//  uint8_t ch;
//  //if Receive interrupt
//  if (UART_GetITStatus(&huart2, UART_IT_RXNE) != RESET)
//    {
//      ch=(uint8_t)UART_ReceiveData(&huart2);
//      xQueueSendToBackFromISR( RxQueueHandle, &ch, &xHigherPriorityTaskWoken );
//    }
//
////  if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
////        {
////      if( xQueueReceiveFromISR( TxQueue, &ch, &xHigherPriorityTaskWoken ) )
////        {
////          USART_SendData(USART1, ch);
////        }else{
////           //disable Transmit Data Register empty interrupt
////           USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
////             }
////        }
//  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	osSemaphoreRelease(BinarySem01Handle);
////	if (readBuf[0]=='\n'){
////		osSemaphoreRelease(BinarySem01Handle);
////	    }
//}

/**
 * @brief  Parses a command string
 * @param char array
 * @retval None
 */
void CMD_Parser(char* buffer){
	// get the first token
	char cmd[256], str[256];
	const char token[] = " ,"; // token[3]={' ' ,  ',' , '\0'}

	buffer[strcspn(buffer, "\r\n")] = 0; // works for LF, CR, CRLF, LFCR, ...
	/* Renvoie la longueur de la plus grande sous-chaîne (en partant du début de la chaîne initiale)
	   ne contenant aucun des caractères spécifiés dans la liste des caractères en rejet \r\n
	 */

	strncpy(cmd, buffer, 256);
	char *sCmd = strtok(cmd, token);

	snprintf(str, 256, "Cmd Name #%s#\n", sCmd);
	printf("%s\n", str);

	/* Command Callback identification */
	uint8_t MATCH=0;
	for(cmdIndex=0; CmdTable[cmdIndex].pCmdFunc!=NULL; cmdIndex++) {
		if (strcmp(CmdTable[cmdIndex].name, sCmd) == 0 ){
			MATCH=1;
			break;
		}
	}
	if (!MATCH) return;

	uint16_t nbArgs=CmdTable[cmdIndex].nbArgs;

#if DEBUG
	snprintf(str, 256, "Function Call %d Parameters Number %d\n", cmdIndex, nbArgs);
	printf("%s\n", str);
#endif

	int32_t funcArgs[MAXARGS];
	for (uint16_t n=0; n<nbArgs; n++){
		sCmd = strtok(NULL, token);
		funcArgs[n]=atoi(sCmd);
	}

#if DEBUG
	snprintf(str, 256, "Function Pointer %p\n", CmdTable[cmdIndex].pCmdFunc);
	printf("%s\n", str);
#endif

	if(CmdTable[cmdIndex].pCmdFunc!=NULL){
#if DEBUG
		snprintf(str, 256, "Function Call %d\n", cmdIndex);
		printf("%s\n", str);
#endif
		CmdTable[cmdIndex].pCmdFunc(nbArgs, funcArgs);
	}
	else{
		Error_Handler();
	}
}

void CMD_on(uint16_t nbArgs, int32_t *args ){
	osStatus status;
	status=osMessagePut(Queue01Handle, 0x01, 200);
	if (status) Error_Handler();

	//printf("Cmd %s\n", CmdTable[cmdIndex].name);
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	//printf("CPLT\n");
}

void CMD_off(uint16_t nbArgs, int32_t *args ){
	osStatus status;
	status=osMessagePut(Queue01Handle, 0x02, 200);
	if (status) Error_Handler();

	//printf("Cmd %s\n", CmdTable[cmdIndex].name);
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	//printf("CPLT\n");
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUartTask */
/**
 * @brief  Function implementing the UartTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	StartReception();

	for(;;)
	{
		osDelay(1);
		rxInProgress = 1;
		osSemaphoreWait(BinarySem01Handle, osWaitForever);  // osWaitForever -> 100
		printf("%s\n", rxBuff);

		CMD_Parser(rxBuff);
		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
 * @brief Function implementing the LedTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
	osEvent ev;
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
		ev=osMessageGet(Queue01Handle, 400);
		if (!(ev.status | (osEventTimeout | osEventMessage))) Error_Handler();   // Timeout exclu
		printf("status %u msg %lu \n\r", ev.status, ev.value.v);

		switch (ev.value.v) {
		case 0x01:
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			break;
		case 0x02:
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			break;
		}
	}
  /* USER CODE END StartLedTask */
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
  if (htim->Instance == TIM6) {
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

	while(1){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_Delay(1000);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
