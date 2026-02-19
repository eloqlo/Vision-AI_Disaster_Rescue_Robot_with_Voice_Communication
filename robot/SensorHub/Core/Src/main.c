/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_CO_PRIO		10
#define TASK_IMU_PRIO		11
#define TASK_SONAR_PRIO		12
#define TASK_REPORT_PRIO	13

#define LIS3DH_ADDR			0x18
#define LIS3DH_REG_STATUS	0x27	// 데이터 상태 레지스터
#define LIS3DH_REG_OUT_X_L	(0x28 | 0x80)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

// CO SENSOR
static TaskHandle_t 	xCOHandle;
static uint8_t 			CO_UART_RxBuffer[9];	// CO 센서 데이터 읽어올 변수
static uint16_t 		CO_PPM;

// IMU SENSOR
static TaskHandle_t 	xIMUHandle;
static uint8_t			IMU_I2C_RxBuffer[6];	// xyz 데이터 읽어올 변수
static uint8_t			IMU_accident_bool;
static int16_t 			x, y, z;

// Sonar SENSOR
static TaskHandle_t 	xSonarHandle;
static int 				distance;

static TaskHandle_t 	xReportHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void Task_DEBUG(void *pvParameters);

void CO_Init(void);
void Task_CO( void *pvParameters );

void IMU_Init(void);
void Task_IMU( void *pvParameters );

void Sonar_Init(void);
void Task_Sonar( void *pvParameters );

void Task_Report( void *pvParameters );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
 if ( ch == '\n' )
	 HAL_UART_Transmit(&huart2, (uint8_t*)&"\r", 1, HAL_MAX_DELAY);
 HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
 return ch;
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  CO_Init();
  IMU_Init();
  Sonar_Init();
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  BaseType_t xReturnedCO;
  xReturnedCO = xTaskCreate(		(TaskFunction_t)Task_CO,
									"Task_CO",
									512+256,
									NULL,
									TASK_CO_PRIO,
									&xCOHandle);
  BaseType_t xReturnedIMU;
  xReturnedIMU = xTaskCreate(		(TaskFunction_t)Task_IMU,
									"Task_IMU",
									1024,
									NULL,
									TASK_IMU_PRIO,
									&xIMUHandle);
  BaseType_t xReturnedSonar;
  xReturnedSonar = xTaskCreate(		(TaskFunction_t)Task_Sonar,
  									"Task_Sonar",
  									512,
  									NULL,
									TASK_SONAR_PRIO,
  									&xSonarHandle);
  BaseType_t xReturnedReport;
  xReturnedReport = xTaskCreate(	(TaskFunction_t)Task_Report,
									"Task_Report",
									800,
									NULL,
									TASK_REPORT_PRIO,
									&xReportHandle);
  BaseType_t xReturnedDebug;
  xReturnedDebug = xTaskCreate(	(TaskFunction_t)Task_DEBUG,
  									"Task_Debug",
  									128,
  									NULL,
  									1,
  									&xReportHandle);
  if(xReturnedCO == pdPASS){
	  printf("CO Task created! %l\n", xReturnedCO);
  }
  if(xReturnedIMU == pdPASS){
  	  printf("IMU Task created! %l\n", xReturnedIMU);
  }
  if(xReturnedSonar == pdPASS){
  	  printf("Sonar Task created! %l\n", xReturnedSonar);
  }
  if(xReturnedReport == pdPASS){
	  printf("Report Task created! %l\n", xReturnedReport);
  }
  if(xReturnedDebug == pdPASS){
  	  printf("Debug Task created! %l\n", xReturnedReport);
  }
  else{
	  printf("debug task not created\n");
  }

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim3.Init.Prescaler = 36-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1F_ED;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|SPI_IRQ_Pin|Sonar_IRQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin SPI_IRQ_Pin Sonar_IRQ_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|SPI_IRQ_Pin|Sonar_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Trig_Pin */
  GPIO_InitStruct.Pin = Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trig_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */




void Task_DEBUG(void *pvParameters){
	for(;;){
		printf("\n");
		printf("CO : %u/10 ppm\n", CO_PPM);
		printf("IMU: %u\n", IMU_accident_bool);
		printf("Sonar: %d\n", distance);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}






void CO_Init(void){
	  HAL_UART_Receive_DMA(&huart3, CO_UART_RxBuffer, 9);	// CO Sensor UART 수신, TODO: Interrupt 방식 DMA로 바꾸기
}

// Deferred interrupt Processing
void Task_CO( void *pvParameters )
{
	uint32_t ulNotifiedValue;
	pvParameters = pvParameters;	// for compiler warning
	uint16_t temp_co_ppm;

	for(;;) {
		ulNotifiedValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1500));	// 인터럽트 처리기로부터 이벤트를 기다린다. & 1.5초 대기
//		printf("(0=CO timeout): %u\n", ulNotifiedValue);

		temp_co_ppm = ((CO_UART_RxBuffer[4] << 8) | CO_UART_RxBuffer[5]);		// Corrected formula

		taskENTER_CRITICAL();
		CO_PPM = temp_co_ppm;
		taskEXIT_CRITICAL();
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	static portBASE_TYPE xHigherPriorityTaskWoken;

	if (huart->Instance == USART3)
	{
		xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(xCOHandle, &xHigherPriorityTaskWoken);
		HAL_UART_Receive_DMA(&huart3, CO_UART_RxBuffer, 9);		// 9 byte 읽어오기
		portYIELD_FROM_ISR(&xHigherPriorityTaskWoken);
	}
}









// (26.02.18)
void IMU_Init(void){
	uint16_t 	IMU_ctrl_reg1_addr = 0x20;	// CTRL_RRG1 주소
	uint8_t 	config = 0b01010111;	// ODR 100Hz, xyz 축 활성화(Normal 모드), datasheet p35
	HAL_I2C_Mem_Write(&hi2c2, (LIS3DH_ADDR << 1), IMU_ctrl_reg1_addr, I2C_MEMADD_SIZE_8BIT, &config, 1, 100);
}

void Task_IMU( void *pvParameters )
{
//	const char *pcTaskName = "Task_IMU";
//	printf("%s is running\n", pcTaskName);fflush(stdout);
	pvParameters = pvParameters;	// for compiler warning

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	HAL_StatusTypeDef status;

	for(;;){
		// I2C(DMA)로 IMU데이터 읽어오기
		status = HAL_I2C_Mem_Read_DMA(&hi2c2, (LIS3DH_ADDR << 1), (0x28 | 0x80),
										I2C_MEMADD_SIZE_8BIT, IMU_I2C_RxBuffer, 6);	// IMU 레지스터 6byte 읽어오기
		if (status == HAL_OK) {
			if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5))) {
				// 전복 여부 계산하기
				x = (int16_t)((IMU_I2C_RxBuffer[1] << 8) | IMU_I2C_RxBuffer[0]);
				y = (int16_t)((IMU_I2C_RxBuffer[3] << 8) | IMU_I2C_RxBuffer[2]);
				z = (int16_t)((IMU_I2C_RxBuffer[5] << 8) | IMU_I2C_RxBuffer[4]);
				if (y > 14000 || y < -14000 || x > 14000 || x < -14000){
//					printf("ACCIDENT OCCURED!\n");
					taskENTER_CRITICAL();
					IMU_accident_bool = 1;
					taskEXIT_CRITICAL();
				}
				else{
					taskENTER_CRITICAL();
					IMU_accident_bool = 0;
					taskEXIT_CRITICAL();
				}
			}
			else{
				// DMA 완료 알림 안오는 경우
				printf("IMU DMA Timeout!\n");
			}
		}
		else{
			// I2C 통신 실패
			printf("I2C Error status: %d\n", status);	// 2(dma busy), 1(error)
		}

		// Task 주기 10ms
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30));
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	static portBASE_TYPE xHigherPriorityTaskWoken;

    if (hi2c->Instance == I2C2) {
		xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(xIMUHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(&xHigherPriorityTaskWoken);
    }
}








// (26.02.18)
void Sonar_Init(void){
	// 없음.
}

void Task_Sonar( void *pvParameters ){
//	const char *pcTaskName = "Task_Sonar";
//	printf("%s is running\n", pcTaskName);
	pvParameters = pvParameters;	// for compiler warning

	for (;;){
		// Input Capture 인터럽트 활성화
		HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

		// 10us 펄스 생성
		HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		while (__HAL_TIM_GET_COUNTER(&htim4) < 10);	// delay 10us
		HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

		// 인터럽트 콜백 도착 대기 (최대 40ms)
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(40)) == pdPASS)
		{
			if (distance < 15 && distance > 0) {
				HAL_GPIO_WritePin(Sonar_IRQ_GPIO_Port, Sonar_IRQ_Pin, GPIO_PIN_SET);
//				printf("넘나 가까운것\n");
			}
			else if (distance > 400){
				HAL_GPIO_WritePin(Sonar_IRQ_GPIO_Port, Sonar_IRQ_Pin, GPIO_PIN_RESET);
				taskENTER_CRITICAL();
				distance = -1;
				taskEXIT_CRITICAL();
			}
			else{
				HAL_GPIO_WritePin(Sonar_IRQ_GPIO_Port, Sonar_IRQ_Pin, GPIO_PIN_RESET);
			}
		}
		else
		{
			// 타임아웃 발생 시 다음 측정을 위해 인터럽트 중지
			HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
//			printf("Echo 도착 안했음\n");
			HAL_GPIO_WritePin(Sonar_IRQ_GPIO_Port, Sonar_IRQ_Pin, GPIO_PIN_RESET);
			taskENTER_CRITICAL();
			distance = -1;
			taskEXIT_CRITICAL();
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM4){

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
		{
			// Rising Edge
			// 타이머 카운터를 0으로 리셋 (신호 시작점)
			__HAL_TIM_SET_COUNTER(htim, 0);

			// 다음 인터럽트는 Falling Edge에서 발생하도록 설정 변경
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else
		{
			// Falling Edge
			// 현재까지의 카운트 값을 읽어옴 (us 단위)
			uint32_t duration = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			distance = duration / 58;

			// 다음 측정을 위해 다시 Rising Edge로 설정 초기화
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

			// 측정이 끝났으므로 인터럽트 중지
			HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);

			// Task에 알림 전달
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(xSonarHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}







void Task_Report( void *pvParameters ){
//	const char *pcTaskName = "Task_Report";
//	printf("%s is running\n", pcTaskName);
	pvParameters = pvParameters;	// for compiler warning
	static uint8_t report_spi[7];		// 패킷 구성: CO_PPM(2) + distance(4) + IMU_accident_bool(1) = 총 7바이트
	TickType_t xLastWakeTime = xTaskGetTickCount();

	HAL_StatusTypeDef status = HAL_TIM_Base_Start(&htim3);
	if (status != HAL_OK){
		printf("TIM3 실행 에러!!\n");
	}

	for (;;){
		taskENTER_CRITICAL();
		report_spi[0] = (uint8_t)(CO_PPM >> 8);
		report_spi[1] = (uint8_t)(CO_PPM & 0xFF);
		report_spi[2] = (uint8_t)(distance >> 24);
		report_spi[3] = (uint8_t)(distance >> 16);
		report_spi[4] = (uint8_t)(distance >> 8);
		report_spi[5] = (uint8_t)(distance & 0xFF);
		report_spi[6] = IMU_accident_bool;
		taskEXIT_CRITICAL();


		// 여기서 lock 걸리는 것 같다.
		HAL_GPIO_WritePin(GPIOA, SPI_IRQ_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		while (__HAL_TIM_GET_COUNTER(&htim3) < 10);	// delay 10us
		HAL_GPIO_WritePin(GPIOA, SPI_IRQ_Pin, GPIO_PIN_RESET);

		if(HAL_SPI_Transmit_IT(&hspi2, report_spi, 7) != HAL_OK){
//			printf("SPI Transmit Error\n");
		}
		else{
//			printf("SPI Report HAL_OK\n");
		}

//		printf("CO: %d/10 ppm\n", CO_PPM);
//		printf("IMU: %u \n", IMU_accident_bool);
//		printf("Sonar: %d cm\n",distance);

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	const char *pcTaskName = "Default_LED_Task";
	printf("%s is running\r\n", pcTaskName);

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(500);
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  // FOR SANITY CHECK!
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  vTaskDelayUntil( &xLastWakeTime, xFrequency);
  }
  /* USER CODE END 5 */
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
