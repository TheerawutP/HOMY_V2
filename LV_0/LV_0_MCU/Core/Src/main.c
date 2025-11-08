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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus_crc.h"
#include "queue.h"
#include "semphr.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint8_t slaveID;
	uint16_t start_address_readHreg;
	uint16_t num_readHreg;
	uint16_t start_address_writeHreg;
	uint16_t num_writeHreg;
}NewSlave;

typedef struct{
    uint8_t data[32];
    uint16_t size;
}UART_Frame;

typedef enum{
	CAR,
	HALL,
	MQTT,
	DRIVER
}current_slave;

typedef enum{
	HALL_UI,
	CABIN
}call_by;

typedef enum{
	IDLE,
	UP,
	DOWN
}direction;

//typedef struct{
//	direction dir;
//	uint16_t toque;
//	uint16_t speed1;
//	uint16_t speed2;
//	void (*on)(void);
//	void (*rotate)(direction r);
//}SERVO;

typedef struct{
	uint8_t from;
	uint8_t dest;
	direction dir;
	uint8_t target;
	call_by requestBy;
}transitReq;

typedef struct{
	uint8_t request[32];
	uint8_t front;
	uint8_t rear;
}SERVE_QUEUE;

typedef struct{
	uint8_t max_fl;
	uint8_t min_fl;
	uint8_t pos;
	direction dir;
}ELEVATOR_CAR;

NewSlave CAR_STA = {
		.slaveID = 0x01,
		.start_address_readHreg = 0x0002,
		.num_readHreg = 0x0002,
		.start_address_writeHreg = 0x0000,
		.num_writeHreg = 0x0002,
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

SERVE_QUEUE queue_UP;
SERVE_QUEUE queue_DW;

uint8_t curr_transit_to;

ELEVATOR_CAR cabin_1 = {
		.max_fl = 8,
		.min_fl = 1,
		.pos = 1,
		.dir = IDLE
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_SIZE 32
#define RESPONSE_TIMEOUT 20
#define QUEUE_SIZE 32

//servo macro
//#define SERVO_ON(obj) (obj).on(&(obj))
//#define SERV0_ROTATE(obj, dir) (obj).rotate(&(obj), dir, (obj).speed)
//#define SERVO_SET_SPD(obj, spd) (obj).speed = spd

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DIR(car_pos, dest)  ((car_pos) > (dest) ? DOWN : UP)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for ParsingTask */
osThreadId_t ParsingTaskHandle;
const osThreadAttr_t ParsingTask_attributes = {
  .name = "ParsingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ServeQueueTask */
osThreadId_t ServeQueueTaskHandle;
const osThreadAttr_t ServeQueueTask_attributes = {
  .name = "ServeQueueTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TransitTask */
osThreadId_t TransitTaskHandle;
const osThreadAttr_t TransitTask_attributes = {
  .name = "TransitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_WriteTask */
osThreadId_t UART_WriteTaskHandle;
const osThreadAttr_t UART_WriteTask_attributes = {
  .name = "UART_WriteTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_ReadTask */
osThreadId_t UART_ReadTaskHandle;
const osThreadAttr_t UART_ReadTask_attributes = {
  .name = "UART_ReadTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ProcessTask */
osThreadId_t ProcessTaskHandle;
const osThreadAttr_t ProcessTask_attributes = {
  .name = "ProcessTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xStartTransitTimer */
osTimerId_t xStartTransitTimerHandle;
const osTimerAttr_t xStartTransitTimer_attributes = {
  .name = "xStartTransitTimer"
};
/* Definitions for xReachTimer */
osTimerId_t xReachTimerHandle;
const osTimerAttr_t xReachTimer_attributes = {
  .name = "xReachTimer"
};
/* USER CODE BEGIN PV */
uint8_t RxData[32];
uint8_t read_TxFrame[16][32];
uint8_t write_TxFrame[16][32];
uint8_t read_RxFrame[16][32];
uint16_t CAR_TxFrame[16];
uint16_t HALL_TxFrame[16];
current_slave read_state = CAR;
current_slave write_state = CAR;

QueueHandle_t xUART_QueueHandle;
QueueHandle_t xServe_QueueHandle;
SemaphoreHandle_t xQueueSem;
SemaphoreHandle_t xQueueMutex;
SemaphoreHandle_t xTransitDoneSem;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void vParsing(void *argument);
void vServeQueue(void *argument);
void vTransit(void *argument);
void vUART_Write(void *argument);
void vUART_Read(void *argument);
void vProcess(void *argument);
void vStartTransit(void *argument);
void vReach(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if(RxData[1] != 0x03){
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 32);
		return;
	}
	UART_Frame localBuf;
	uint16_t crc_received = RxData[Size - 2] | (RxData[Size - 1] << 8);
	uint16_t crc_calculated = crc16(RxData, Size - 2);

    if (crc_received != crc_calculated) {
    		HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 32);
		return;
    }
	if (Size < 5) {
	        HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, FRAME_SIZE);
	    return;
	}
	localBuf.size = Size;
	    memcpy(localBuf.data, RxData, Size);
	    xQueueSendFromISR(xUART_QueueHandle, &localBuf, NULL);

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 32);

}

void writeBit(uint16_t *package, uint8_t loc, uint8_t state){
    if (state) {
        *package = *package | (1 << loc);      // set
    } else {
        *package = *package & ~(1 << loc);     // clear
    }
}
void parseEndian(uint16_t val, uint8_t *hi, uint8_t *lo){
     *hi = (val >> 8) & 0xFF;
     *lo = val & 0xFF;
}

void extractBits(uint16_t val, uint8_t *arr, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        arr[i] = (val & (1 << i)) ? 1 : 0;
    }
}


uint8_t read_Hreg(NewSlave *a, uint8_t *frame)
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

int isFull(SERVE_QUEUE *queue){
    int next = (queue->rear + 1) % QUEUE_SIZE;
    return(next == queue->front);
}

int isEmpty(SERVE_QUEUE *queue){
    return(queue->front == queue->rear);
}


int enqueue(transitReq task, SERVE_QUEUE *queue) {
    if(isFull(queue)) return 0;

    int next = queue->rear + 1;
    queue->request[queue->rear] = task.target;
    queue->rear = next;
    return 1;
}

int dequeue(SERVE_QUEUE *queue) {
	if(isEmpty(queue)) return 0;
    queue->request[queue->front] = queue->request[queue->front+1];
    queue->front = (queue->front + 1) % QUEUE_SIZE;
    return 1;
}

int insertqueue(transitReq task, SERVE_QUEUE *queue, int pos){
    int next = (queue->rear + 1) % QUEUE_SIZE;
    if(next == queue->front) return 0;

    int curr = queue->rear;
    while (curr != pos) {
    	int next = curr+1;
    	int prev = curr-1;
    	queue->request[next] = queue->request[curr];
    	queue->request[curr] = queue->request[prev];
    	curr = prev;
    }

    queue->request[pos] = task.target;
	return 1;
}

void sortByProximity(uint8_t arr[], uint8_t pos) {
    uint8_t n = QUEUE_SIZE;
    uint8_t i, j, temp;
    for (i = 0; i < n - 1; i++) {
        for (j = 0; j < n - i - 1; j++) {
        	int diff1 = abs(arr[j] - pos);
        	int diff2 = abs(arr[j + 1] - pos);
            if (diff1 > diff2) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

int elevatorQueueManage(ELEVATOR_CAR *car, transitReq task, SERVE_QUEUE *up, SERVE_QUEUE *dw){

	if(car->dir == IDLE){
		car->dir = task.dir;
	}

	if(task.dir == UP){
		enqueue(task, up);
		sortByProximity(up->request, car->pos);
	}else{
		enqueue(task, dw);
		sortByProximity(dw->request, car->pos);
	}

	return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  xUART_QueueHandle = xQueueCreate(32, sizeof(UART_Frame));
  xServe_QueueHandle = xQueueCreate(32, sizeof(transitReq));
  xQueueMutex = xSemaphoreCreateMutex();
  xQueueSem = xSemaphoreCreateBinary();
  xTransitDoneSem = xSemaphoreCreateBinary();


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
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of xStartTransitTimer */
  xStartTransitTimerHandle = osTimerNew(vStartTransit, osTimerOnce, NULL, &xStartTransitTimer_attributes);

  /* creation of xReachTimer */
  xReachTimerHandle = osTimerNew(vReach, osTimerOnce, NULL, &xReachTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ParsingTask */
  ParsingTaskHandle = osThreadNew(vParsing, NULL, &ParsingTask_attributes);

  /* creation of ServeQueueTask */
  ServeQueueTaskHandle = osThreadNew(vServeQueue, NULL, &ServeQueueTask_attributes);

  /* creation of TransitTask */
  TransitTaskHandle = osThreadNew(vTransit, NULL, &TransitTask_attributes);

  /* creation of UART_WriteTask */
  UART_WriteTaskHandle = osThreadNew(vUART_Write, NULL, &UART_WriteTask_attributes);

  /* creation of UART_ReadTask */
  UART_ReadTaskHandle = osThreadNew(vUART_Read, NULL, &UART_ReadTask_attributes);

  /* creation of ProcessTask */
  ProcessTaskHandle = osThreadNew(vProcess, NULL, &ProcessTask_attributes);

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

/* USER CODE BEGIN Header_vParsing */
/**
  * @brief  Function implementing the ParsingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vParsing */
void vParsing(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		UART_Frame frameBuf;
		if (xQueueReceive(xUART_QueueHandle, &frameBuf, portMAX_DELAY) == pdTRUE)
		    {
		    	int id = frameBuf.data[0];
			    memcpy(read_RxFrame[id], frameBuf.data, frameBuf.size);

		    }

	vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vServeQueue */
/**
* @brief Function implementing the ServeQueueTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vServeQueue */
void vServeQueue(void *argument)
{
  /* USER CODE BEGIN vServeQueue */
  /* Infinite loop */
  transitReq request;
  for(;;)
  {
		if (xQueueReceive(xServe_QueueHandle, &request, portMAX_DELAY) == pdTRUE)
		    {
				if(xSemaphoreTake(xQueueMutex, portMAX_DELAY) == pdTRUE){
					if(elevatorQueueManage(&cabin_1, request, &queue_UP, &queue_DW)){
		                if(cabin_1.dir == IDLE){
		                	osTimerStart(xStartTransitTimerHandle, 6000);
		                }
		                else osTimerStart(xStartTransitTimerHandle, 0);
					}
		            xSemaphoreGive(xQueueMutex);
				}
		    }


		if(xSemaphoreTake(xTransitDoneSem, portMAX_DELAY) == pdTRUE)
			{
				if(xSemaphoreTake(xQueueMutex, portMAX_DELAY) == pdTRUE){

					direction curr_dir = cabin_1.dir;
					switch (curr_dir){
					case IDLE:
						break;
					case UP:
						dequeue(&queue_UP);
						break;
					case DOWN:
						dequeue(&queue_DW);
						break;
					}

					if(!isEmpty(&queue_UP) || !isEmpty(&queue_DW)){

						if((isEmpty(&queue_UP) == 0) && (curr_dir == UP)){
							curr_transit_to = queue_UP.request[0];
			                xSemaphoreGive(xQueueSem);

						}else if((isEmpty(&queue_UP) == 1) && (curr_dir == UP)){
							if(!isEmpty(&queue_DW)) cabin_1.dir = DOWN;

						}else if((isEmpty(&queue_DW) == 0) && (curr_dir == DOWN)){
							curr_transit_to = queue_DW.request[0];
			                xSemaphoreGive(xQueueSem);

						}else if((isEmpty(&queue_DW) == 1) && (curr_dir == DOWN)){
							if(!isEmpty(&queue_UP)) cabin_1.dir = UP;
						}
					}else{
						cabin_1.dir = IDLE;
					}
		            xSemaphoreGive(xQueueMutex);
				}
			}
  }
  /* USER CODE END vServeQueue */
}

/* USER CODE BEGIN Header_vTransit */
/**
* @brief Function implementing the TransitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTransit */
void vTransit(void *argument)
{
  /* USER CODE BEGIN vTransit */
  /* Infinite loop */
  uint8_t dest;
  for(;;)
  {
	  if(xSemaphoreTake(xQueueSem, portMAX_DELAY) == pdTRUE){
		  if(xSemaphoreTake(xQueueMutex, portMAX_DELAY) == pdTRUE){
			  dest  = curr_transit_to;
		      xSemaphoreGive(xQueueMutex);
		  }


		   //mocking cabin transit
		  osTimerStart(xReachTimerHandle, 5000);

		  //xSemaphoreGive(xTransitDoneSem); //done transit flag
	  }
  }
  /* USER CODE END vTransit */
}

/* USER CODE BEGIN Header_vUART_Write */
/**
* @brief Function implementing the UART_WriteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vUART_Write */
void vUART_Write(void *argument)
{
  /* USER CODE BEGIN vUART_Write */
  /* Infinite loop */
  uint8_t len;
  uint8_t up;
  uint8_t dw;
  uint8_t idle;

  for(;;)
  {

	  	switch(cabin_1.dir){
	  	case UP:
	  		up = 1;
	  		dw = 0;
	  		idle = 0;
	  		break;

	  	case DOWN:
	  		up = 0;
	  		dw = 1;
	  		idle = 0;
	  		break;

	  	case IDLE:
	  		up = 0;
	  		dw = 0;
	  		idle = 1;
	  		break;
	  	}

	  	uint8_t pos_b0 = (cabin_1.pos>>0) & 1;
	  	uint8_t pos_b1 = (cabin_1.pos>>1) & 1;
	  	uint8_t pos_b2 = (cabin_1.pos>>2) & 1;
	  	uint8_t pos_b3 = (cabin_1.pos>>3) & 1;

		switch (write_state){
			case CAR:
				//packing for car
				//car first frame
				writeBit(&CAR_TxFrame[0], 4, dw); //dir dw
				writeBit(&CAR_TxFrame[0], 5, up); //dir up
				writeBit(&CAR_TxFrame[0], 6, pos_b0); //car pos b0
				writeBit(&CAR_TxFrame[0], 7, pos_b1);
				writeBit(&CAR_TxFrame[0], 8, pos_b2);
				writeBit(&CAR_TxFrame[0], 9, pos_b3);

				len = write_MultipleHreg(&CAR_STA, CAR_TxFrame, write_TxFrame[0]);
				HAL_UART_Transmit(&huart1, write_TxFrame[0], len, RESPONSE_TIMEOUT);
				write_state = HALL;
				break;

			case HALL:
				writeBit(&HALL_TxFrame[3], 0, dw); //dir dw
				writeBit(&HALL_TxFrame[3], 1, up); //dir up
				writeBit(&HALL_TxFrame[3], 2, pos_b0); //car pos b0
				writeBit(&HALL_TxFrame[3], 3, pos_b1);
				writeBit(&HALL_TxFrame[3], 4, pos_b2);
				writeBit(&HALL_TxFrame[3], 5, pos_b3);

				len = write_MultipleHreg(&HALL_STA, HALL_TxFrame, write_TxFrame[1]);
				HAL_UART_Transmit(&huart1, write_TxFrame[1], len, RESPONSE_TIMEOUT);
				write_state = MQTT;
				break;
			case MQTT:
//				len = read_Hreg(&MQTT_STA, read_TxFrame[2]);
//				HAL_UART_Transmit(&huart1, read_TxFrame[2], len, RESPONSE_TIMEOUT);
				write_state = DRIVER;
				break;

			case DRIVER:
//				len = read_Hreg(&SERVO_DRIVER, read_TxFrame[3]);
//				HAL_UART_Transmit(&huart1, read_TxFrame[3], len, RESPONSE_TIMEOUT);
				write_state = CAR;
				break;

		}
	  	vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END vUART_Write */
}

/* USER CODE BEGIN Header_vUART_Read */
/**
* @brief Function implementing the UART_ReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vUART_Read */
void vUART_Read(void *argument)
{
  /* USER CODE BEGIN vUART_Read */
  /* Infinite loop */
	  for(;;)
	  {

		uint8_t len;
		switch (read_state){
			case CAR:
  				len = read_Hreg(&CAR_STA, read_TxFrame[0]);
				HAL_UART_Transmit(&huart1, read_TxFrame[0], len, RESPONSE_TIMEOUT);
				read_state = HALL;
				break;

			case HALL:
				len = read_Hreg(&HALL_STA, read_TxFrame[1]);
				HAL_UART_Transmit(&huart1, read_TxFrame[1], len, RESPONSE_TIMEOUT);
				read_state = MQTT;
				break;

			case MQTT:
				len = read_Hreg(&MQTT_STA, read_TxFrame[2]);
				HAL_UART_Transmit(&huart1, read_TxFrame[2], len, RESPONSE_TIMEOUT);
				read_state = DRIVER;
				break;

			case DRIVER:
				len = read_Hreg(&SERVO_DRIVER, read_TxFrame[3]);
				HAL_UART_Transmit(&huart1, read_TxFrame[3], len, RESPONSE_TIMEOUT);
				read_state = CAR;
				break;
			}
		vTaskDelay(pdMS_TO_TICKS(10));
	  }
  /* USER CODE END vUART_Read */
}

/* USER CODE BEGIN Header_vProcess */
/**
* @brief Function implementing the ProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vProcess */
void vProcess(void *argument)
{
  /* USER CODE BEGIN vProcess */
  /* Infinite loop */
  transitReq request;
  uint8_t hall_calling_UP[16] = {0};
  uint8_t hall_calling_DW[16] = {0};
  uint8_t car_aiming[16] = {0};
  for(;;)
  {
      /* --- Hall UP --- */
      uint16_t val_up = (read_RxFrame[2][5] << 8) | read_RxFrame[2][6];
      extractBits(val_up, hall_calling_UP, cabin_1.max_fl);

      /* --- Hall DOWN --- */
      uint16_t val_dw = (read_RxFrame[2][7] << 8) | read_RxFrame[2][8];
      extractBits(val_dw, hall_calling_DW, cabin_1.max_fl);

      /* --- Car Aiming --- */
      uint16_t val_car = (read_RxFrame[1][5] << 8) | read_RxFrame[1][6];
      extractBits(val_car, car_aiming, cabin_1.max_fl);

	  if(hall_calling_UP[0] == 1){
		 for(int i = 1; i<=8; i++){
			 if(hall_calling_UP[i] == 1){
				 request.target = i;
				 request.dir = UP;
				 request.requestBy = HALL_UI;
				 xQueueSend(xServe_QueueHandle, &request, 0);
			 }
		 }
	  }

	  if(hall_calling_DW[0] == 1){
		 for(int i = 1; i<=8; i++){
			 if(hall_calling_DW[i] == 1){
				 request.target = i;
				 request.dir = DOWN;
				 request.requestBy = HALL_UI;
				 xQueueSend(xServe_QueueHandle, &request, 0);
			 }
		 }
	  }

	  if(car_aiming[0] == 1){
		 for(int i = 1; i<=8; i++){
			 if(car_aiming[i] == 1){
				 request.target = i;
				 request.dir = DIR(cabin_1.pos, i); //macro
				 request.requestBy = CABIN;
				 xQueueSend(xServe_QueueHandle, &request, 0);
			 }
		 }
	  }

	vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END vProcess */
}

/* vStartTransit function */
void vStartTransit(void *argument)
{
  /* USER CODE BEGIN vStartTransit */
	xSemaphoreGive(xQueueSem);
  /* USER CODE END vStartTransit */
}

/* vReach function */
void vReach(void *argument)
{
  /* USER CODE BEGIN vReach */
	xSemaphoreGive(xTransitDoneSem);
  /* USER CODE END vReach */
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
