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
#include "cmsis_os.h"
#include "modbus_crc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{

	uint8_t slaveID;
	uint16_t start_address_readHreg;
	uint16_t num_readHreg;
	uint16_t start_address_writeHreg;
	uint16_t num_writeHreg;
	//uint16_t write_data[16];

}NewSlave;

typedef enum{
	CAR,
	HALL,
	MQTT,
	DRIVER
}current_slave;

current_slave read_state = CAR;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for modbusTask */
osThreadId_t modbusTaskHandle;
const osThreadAttr_t modbusTask_attributes = {
  .name = "modbusTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for processTask */
osThreadId_t processTaskHandle;
const osThreadAttr_t processTask_attributes = {
  .name = "processTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void vmodbusTask(void *argument);
void vprocessTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RxData[32];
//uint8_t TxData[4][];
uint8_t read_TxFrame[16][32];
volatile uint16_t Data[32][32];

#define RESPONSE_TIMEOUT 20

NewSlave CAR_STA = {
		.slaveID = 0x01,
		.start_address_readHreg = 0x0001,
		.num_readHreg = 0x0001,
		.start_address_writeHreg = 0x0000,
		.num_writeHreg = 0x0001,
};

NewSlave HALL_STA = {
		.slaveID = 0x02,
		.start_address_readHreg = 0x0004,
		.num_readHreg = 0x0003,
		.start_address_writeHreg = 0x0000,
		.num_writeHreg = 0x0004,
};
NewSlave MQTT_STA = {
		.slaveID = 0x03,
		.start_address_readHreg = 0x0000,
		.num_readHreg = 0x0001,
		.start_address_writeHreg = 0x0000,
		.num_writeHreg = 0x0001,
};
NewSlave SERVO_DRIVER = {
		.slaveID = 0x7F,
		.start_address_readHreg = 0x0000,
		.num_readHreg = 0x0001,
		.start_address_writeHreg = 0x0000,
		.num_writeHreg = 0x0001,
};

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if (Size < 5) {
	        HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 32);
	        return;
	    }

    uint16_t crc_received = RxData[Size - 2] | (RxData[Size - 1] << 8);
    uint16_t crc_calculated = crc16(RxData, Size - 2);

    if (crc_received != crc_calculated) {
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 32);
        return;
    }

	int id = RxData[0];
	int inx = 3;
	int num = RxData[2];
	int reg_num = num/2;
	if(RxData[1] == 0x03){
		for(int i=0; i<reg_num; i++){
				Data[id][i] = RxData[inx]<<8 | RxData[inx+1];
				inx = inx+2;
			}
	}
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 32);

}



void parseEndian(uint16_t val, uint8_t *hi, uint8_t *lo){
     *hi = (val >> 8) & 0xFF;
     *lo = val & 0xFF;
}

uint8_t read_Hreg(NewSlave *a, uint8_t *frame[32])
{
	int len = 0;
	   frame[len++] = a->slaveID;
	   frame[len++] = 0x03;

	   parseEndian(a->start_address_readHreg, &frame[len], &frame[len+1]);
	   len+=2;
	   parseEndian(a->num_readHreg, &frame[len], &frame[len+1]);
	   len+=2;

	   uint16_t crc = crc16(frame, len);
	   frame[len++] = crc&0xFF;
	   frame[len++] = (crc>>8)&0xFF;

	   return len;
}

uint8_t write_singleHreg(NewSlave *a, uint32_t data, uint8_t *frame)
{

	int len = 0;
	   frame[len++] = a->slaveID;
	   frame[len++] = 0x06;

	   parseEndian(a->start_address_writeHreg, &frame[len], &frame[len+1]);
	   len+=2;

	   frame[len++] = (data >> 8) & 0xFF;
	   frame[len++] = data & 0xFF;

	   uint16_t crc = crc16(frame, len);
	   frame[len++] = crc&0xFF;
	   frame[len++] = (crc>>8)&0xFF;

	   return len;
}

uint8_t write_MultipleHreg(NewSlave *a, uint16_t data[16], uint8_t *frame)
{

	int len = 0;
	   frame[len++] = a->slaveID;
	   frame[len++] = 0x10;

	   parseEndian(a->start_address_writeHreg, &frame[len], &frame[len+1]);
	   len+=2;
	   parseEndian(a->num_writeHreg, &frame[len], &frame[len+1]);
	   len+=2;

	   frame[len++] = a->num_writeHreg * 2;

	   for (int i = 0; i < a->num_writeHreg; i++) {
		   frame[len++] = (data[i] >> 8) & 0xFF;
		   frame[len++] = data[i] & 0xFF;
	   }

	   uint16_t crc = crc16(frame, len);
	   frame[len++] = crc&0xFF;
	   frame[len++] = (crc>>8)&0xFF;

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 32);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of modbusTask */
  modbusTaskHandle = osThreadNew(vmodbusTask, NULL, &modbusTask_attributes);

  /* creation of processTask */
  processTaskHandle = osThreadNew(vprocessTask, NULL, &processTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vmodbusTask */
/**
* @brief Function implementing the modbusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vmodbusTask */
void vmodbusTask(void *argument)
{
  /* USER CODE BEGIN vmodbusTask */
  /* Infinite loop */
  for(;;)
  {

	uint8_t len;
	switch (read_state){
		case CAR:
			len = read_Hreg(&CAR_STA, &read_TxFrame[0]);
			HAL_UART_Transmit(&huart1, read_TxFrame[0], len, RESPONSE_TIMEOUT);
			read_state = HALL;
			break;

		case HALL:
			len = read_Hreg(&HALL_STA, &read_TxFrame[1]);
			HAL_UART_Transmit(&huart1, read_TxFrame[1], len, RESPONSE_TIMEOUT);
			read_state = MQTT;
			break;

		case MQTT:
			read_Hreg(&MQTT_STA, &read_TxFrame[2]);
			HAL_UART_Transmit(&huart1, read_TxFrame[2], len, RESPONSE_TIMEOUT);
			read_state = DRIVER;
			break;

		case DRIVER:
			read_Hreg(&SERVO_DRIVER, &read_TxFrame[3]);
			HAL_UART_Transmit(&huart1, read_TxFrame[3], len, RESPONSE_TIMEOUT);
			read_state = CAR;
			break;
		}
	vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END vmodbusTask */
}

/* USER CODE BEGIN Header_vprocessTask */
/**
* @brief Function implementing the processTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vprocessTask */
void vprocessTask(void *argument)
{
  /* USER CODE BEGIN vprocessTask */
  /* Infinite loop */
  for(;;)
  {

    osDelay(10);
  }
  /* USER CODE END vprocessTask */
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
