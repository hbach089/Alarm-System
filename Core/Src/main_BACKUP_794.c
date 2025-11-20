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
#include "stm32f4xx_hal.h"
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ARMED 0x00
#define DISARMED 0x01
#define OLD_PASSWORD 0x10
#define NEW_PASSWORD 0x11
#define ENTER_KEY 0xFF

#define MAX_PASSWORD_LENGTH 6
#define MIN_PASSWORD_LENGTH 4

#define START_ADDRESS 0x08060000

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for KeyPadIptTask */
osThreadId_t KeyPadIptTaskHandle;
const osThreadAttr_t KeyPadIptTask_attributes = {
  .name = "KeyPadIptTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCDLine2Task */
osThreadId_t LCDLine2TaskHandle;
const osThreadAttr_t LCDLine2Task_attributes = {
  .name = "LCDLine2Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Red_LEDTask */
osThreadId_t Red_LEDTaskHandle;
const osThreadAttr_t Red_LEDTask_attributes = {
  .name = "Red_LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Green_LEDTask */
osThreadId_t Green_LEDTaskHandle;
const osThreadAttr_t Green_LEDTask_attributes = {
  .name = "Green_LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCDLine1Task */
osThreadId_t LCDLine1TaskHandle;
const osThreadAttr_t LCDLine1Task_attributes = {
  .name = "LCDLine1Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for resetPwordTask */
osThreadId_t resetPwordTaskHandle;
const osThreadAttr_t resetPwordTask_attributes = {
  .name = "resetPwordTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PIRsensorTask */
osThreadId_t PIRsensorTaskHandle;
const osThreadAttr_t PIRsensorTask_attributes = {
  .name = "PIRsensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myMutex */
osMutexId_t myMutexHandle;
const osMutexAttr_t myMutex_attributes = {
  .name = "myMutex"
};
/* Definitions for isArmedMutex */
osMutexId_t isArmedMutexHandle;
const osMutexAttr_t isArmedMutex_attributes = {
  .name = "isArmedMutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
void StartKeyPadIptTask(void *argument);
void StartLCDLine2Task(void *argument);
void StartRED_LEDTask(void *argument);
void StartGreen_LEDTask(void *argument);
void StartLCDLine1Task(void *argument);
void StartResetPwordTask(void *argument);
void StartPIRsensorTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==BLUE_PUSH_BTN_Pin){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(LCDLine2TaskHandle,0xFE,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

// Function to convert ASCII value to char (integer from 0-9 inclusive)
char ASCIItoChar(uint8_t asciiVal){
    return (char)(asciiVal+0);
}

// Function to write to flash

void Write_to_Flash(uint32_t startMemAddress,char* data){
	FLASH_EraseInitTypeDef flash_erase_struct={0};

	// 1) Unlock the flash
	HAL_FLASH_Unlock();

	// 2) Set necessary fields in the flash data structure
	flash_erase_struct.NbSectors=1;
	flash_erase_struct.Sector=FLASH_SECTOR_7;
	flash_erase_struct.VoltageRange=FLASH_VOLTAGE_RANGE_3;
	flash_erase_struct.TypeErase=FLASH_TYPEERASE_SECTORS;

	// 3) Erase the flash sector that we will write to (in this case last sector)
	uint32_t  error_status;
	HAL_FLASHEx_Erase(&flash_erase_struct, &error_status);


	// 4) Write to the flash starting at the sector address
	//    Writing is done in contiguous memory
	uint32_t memAddress=startMemAddress;
	for(int i=0;i<strlen(data);i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, memAddress, data[i]);
		memAddress++;
	}

	// 5) Lock the flash when done writing
	HAL_FLASH_Lock();
}

// Function to read from Flash memory
// Returns the string stored in the Flash address
char* Read_from_Flash(uint32_t startMemAddress){

	// Pointer to first address in the sector in Flash
	uint8_t* flash_address = (uint8_t*)startMemAddress;

	// Static array so that it is not destroyed upon return from the function
	// If so it would have been stored on the stack,
	// but now it is stored on BSS segment
	static char pwordInFlash[6];

	int cnt=0;
	while(1){
		// When we reach ASCII value larger than 57 (9 in decimal)
		// We reach an area in Flash that contains garbage values as nothing
		// was written to it.
		if(*flash_address>57){
			break;
		}

		// Convert from ASCII to decimal
		// Increase the pointer to go to next value in Flash memory
		pwordInFlash[cnt++]=ASCIItoChar(*(flash_address++));
	}
	// Null terminated array
	pwordInFlash[cnt]='\0';

	return pwordInFlash;
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
  MX_I2C1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myMutex */
  myMutexHandle = osMutexNew(&myMutex_attributes);

  /* creation of isArmedMutex */
  isArmedMutexHandle = osMutexNew(&isArmedMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (6, sizeof(uint16_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of KeyPadIptTask */
  KeyPadIptTaskHandle = osThreadNew(StartKeyPadIptTask, NULL, &KeyPadIptTask_attributes);

  /* creation of LCDLine2Task */
  LCDLine2TaskHandle = osThreadNew(StartLCDLine2Task, NULL, &LCDLine2Task_attributes);

  /* creation of Red_LEDTask */
  Red_LEDTaskHandle = osThreadNew(StartRED_LEDTask, NULL, &Red_LEDTask_attributes);

  /* creation of Green_LEDTask */
  Green_LEDTaskHandle = osThreadNew(StartGreen_LEDTask, NULL, &Green_LEDTask_attributes);

  /* creation of LCDLine1Task */
  LCDLine1TaskHandle = osThreadNew(StartLCDLine1Task, NULL, &LCDLine1Task_attributes);

  /* creation of resetPwordTask */
  resetPwordTaskHandle = osThreadNew(StartResetPwordTask, NULL, &resetPwordTask_attributes);

  /* creation of PIRsensorTask */
  PIRsensorTaskHandle = osThreadNew(StartPIRsensorTask, NULL, &PIRsensorTask_attributes);

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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOA, RED_LED_Pin|GREEN_LED_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C1_OPT_Pin|C2_OPT_Pin|C3_OPT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUE_PUSH_BTN_Pin */
  GPIO_InitStruct.Pin = BLUE_PUSH_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_PUSH_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_Pin GREEN_LED_Pin LD2_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|GREEN_LED_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_OPT_Pin C2_OPT_Pin C3_OPT_Pin */
  GPIO_InitStruct.Pin = C1_OPT_Pin|C2_OPT_Pin|C3_OPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : R2_IPT_Pin */
  GPIO_InitStruct.Pin = R2_IPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(R2_IPT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R4_IPT_Pin R3_IPT_Pin */
  GPIO_InitStruct.Pin = R4_IPT_Pin|R3_IPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PIR_Sensor_Pin */
  GPIO_InitStruct.Pin = PIR_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PIR_Sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R1_IPT_Pin */
  GPIO_InitStruct.Pin = R1_IPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(R1_IPT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t pword[6];
uint8_t is_armed=0;

// Function that handles keypad presses
// Returns array of keys pressed forming password
char* keyPressHandler(uint32_t ulNotificationValue,int cnt){
	//static to preserve  password state across function calls
	static char pword[6];

	char temp[5];

	if(ulNotificationValue==ENTER_KEY || ulNotificationValue==0xFE || ulNotificationValue==0x11){
		pword[cnt]='\0';
		return pword;
	}
	else{
		if(ulNotificationValue==0x00){
			pword[cnt]='0';
		}
		else if(ulNotificationValue==0x01){
			pword[cnt]='1';
		}
		else if(ulNotificationValue==0x02){
			pword[cnt]='2';
		}
		else if(ulNotificationValue==0x03){
			pword[cnt]='3';
		}
		else if(ulNotificationValue==0x04){
			pword[cnt]='4';
		}
		else if(ulNotificationValue==0x05){
			pword[cnt]='5';
		}
		else if(ulNotificationValue==0x06){
			pword[cnt]='6';
		}
		else if(ulNotificationValue==0x07){
			pword[cnt]='7';
		}
		else if(ulNotificationValue==0x08){
			pword[cnt]='8';
		}
		else if(ulNotificationValue==0x09){
			pword[cnt]='9';
		}
		return pword;
	}
}

// read keypad function that returns pressed key value (in hex)
uint8_t readKeypadChar(){
	uint8_t val;

	//Infinite loop so that it continuously prompts user to enter
	//value and only breaks when keypad is pressed
	while(1){
		HAL_GPIO_WritePin(C1_OPT_GPIO_Port, C1_OPT_Pin, 0);
		HAL_GPIO_WritePin(C2_OPT_GPIO_Port, C2_OPT_Pin, 1);
		HAL_GPIO_WritePin(C3_OPT_GPIO_Port, C3_OPT_Pin, 1);

		if(!HAL_GPIO_ReadPin(R1_IPT_GPIO_Port, R1_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R1_IPT_GPIO_Port, R1_IPT_Pin));
			val=0x01;
			break;
		}
		else if(!HAL_GPIO_ReadPin(R2_IPT_GPIO_Port, R2_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R2_IPT_GPIO_Port, R2_IPT_Pin));
			val=0x04;
			break;
		}
		else if(!HAL_GPIO_ReadPin(R3_IPT_GPIO_Port, R3_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R3_IPT_GPIO_Port, R3_IPT_Pin));
			val=0x07;
			break;
		}
		else if(!HAL_GPIO_ReadPin(R4_IPT_GPIO_Port, R4_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R4_IPT_GPIO_Port, R4_IPT_Pin));
			val=ENTER_KEY;
			break;
		}

		HAL_GPIO_WritePin(C1_OPT_GPIO_Port, C1_OPT_Pin, 1);
		HAL_GPIO_WritePin(C2_OPT_GPIO_Port, C2_OPT_Pin, 0);
		HAL_GPIO_WritePin(C3_OPT_GPIO_Port, C3_OPT_Pin, 1);

		if(!HAL_GPIO_ReadPin(R1_IPT_GPIO_Port, R1_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R1_IPT_GPIO_Port, R1_IPT_Pin));
			val=0x02;
			break;
		}
		else if(!HAL_GPIO_ReadPin(R2_IPT_GPIO_Port, R2_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R2_IPT_GPIO_Port, R2_IPT_Pin));
			val=0x05;
			break;
		}
		else if(!HAL_GPIO_ReadPin(R3_IPT_GPIO_Port, R3_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R3_IPT_GPIO_Port, R3_IPT_Pin));
			val=0x08;
			break;
		}
		else if(!HAL_GPIO_ReadPin(R4_IPT_GPIO_Port, R4_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R4_IPT_GPIO_Port, R4_IPT_Pin));
			val=0x00;
			break;
		}

		HAL_GPIO_WritePin(C1_OPT_GPIO_Port, C1_OPT_Pin, 1);
		HAL_GPIO_WritePin(C2_OPT_GPIO_Port, C2_OPT_Pin, 1);
		HAL_GPIO_WritePin(C3_OPT_GPIO_Port, C3_OPT_Pin, 0);

		if(!HAL_GPIO_ReadPin(R1_IPT_GPIO_Port, R1_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R1_IPT_GPIO_Port, R1_IPT_Pin));
			val=0x03;
			break;
		}
		else if(!HAL_GPIO_ReadPin(R2_IPT_GPIO_Port, R2_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R2_IPT_GPIO_Port, R2_IPT_Pin));
			val=0x06;
			break;
		}
		else if(!HAL_GPIO_ReadPin(R3_IPT_GPIO_Port, R3_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R3_IPT_GPIO_Port, R3_IPT_Pin));
			val=0x09;
			break;
		}
		else if(!HAL_GPIO_ReadPin(R4_IPT_GPIO_Port, R4_IPT_Pin)){
			while(!HAL_GPIO_ReadPin(R4_IPT_GPIO_Port, R4_IPT_Pin));
			val=0x11;
			break;
		}
	}
	return val;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartKeyPadIptTask */
/**
  * @brief  Function implementing the KeyPadIptTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartKeyPadIptTask */
void StartKeyPadIptTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	HD44780_Init(2);
	HD44780_Clear();
	HD44780_Backlight();

	//System starts in DISARMED mode
	uint8_t val;
	xTaskNotify(Green_LEDTaskHandle,DISARMED,eSetValueWithOverwrite);
/* Infinite loop */
	for(;;)
	{
	  //blocking function until keypad press
	  val=readKeypadChar();

	  uint8_t arr[10];
	  sprintf(arr,"%d\r\n",val);
	  HAL_UART_Transmit(&huart2, arr, strlen(arr), osWaitForever);
	  xTaskNotify(LCDLine2TaskHandle,val,eSetValueWithOverwrite);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLCDLine2Task */

// Function writes * symbols on screen every keypad press
void Write_To_LCD(int cnt){
	//Write respective amount of * characters on screen
	char screenContent[cnt];
	memset(screenContent,'*',cnt);
	screenContent[cnt]='\0';

	// use mutex to synchronize use of shared ressource (LCD screen)
	osMutexAcquire(myMutexHandle, osWaitForever);
	HD44780_SetCursor(0,1);
	HD44780_PrintStr(screenContent);
	osMutexRelease(myMutexHandle);
}

// Clear Line 2 of LCD Screen
void Clear_LCD_Line2Screen(){

	// use mutex to synchronize use of shared ressource (LCD screen)
	osMutexAcquire(myMutexHandle, osWaitForever);
	//Erase line 2 of LCD Screen with spaces.
	HD44780_SetCursor(0,1);
	for(int i=0;i<16;i++){
		HD44780_PrintStr(" ");
	}
	osMutexRelease(myMutexHandle);
}

/**
* @brief Function implementing the LCDLine2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDLine2Task */
void StartLCDLine2Task(void *argument)
{
  /* USER CODE BEGIN StartLCDLine2Task */
	uint32_t ulNotificationValue;
	int cnt=0;
	int resetPasswordReadyFlag=1,newPasswordFlag=0;
	char pword[MAX_PASSWORD_LENGTH];

	uint8_t temp_is_armed;
	HAL_UART_Transmit(&huart2, "\r\n", 2, osWaitForever);
	HAL_UART_Transmit(&huart2, Read_from_Flash(START_ADDRESS), strlen(Read_from_Flash(START_ADDRESS)), osWaitForever);
  /* Infinite loop */
  for(;;)
  {
	/* Infinite loop */
	if(xTaskNotifyWait(0, 0xffffffff, &ulNotificationValue, portMAX_DELAY)){

		//Copy value of keyPressHandler into pword array
		strcpy(pword,keyPressHandler(ulNotificationValue,cnt));

		osMutexAcquire(isArmedMutexHandle, osWaitForever);
		temp_is_armed=is_armed;
		osMutexRelease(isArmedMutexHandle);
		
		// User presses # key in the DISARMED state.
		// Alternates between OLD_PASSWORD and DISARMED states.
		if(ulNotificationValue==NEW_PASSWORD && !temp_is_armed){
			cnt=0;
			Clear_LCD_Line2Screen();
			if(resetPasswordReadyFlag){
				HAL_UART_Transmit(&huart2, "NEWPASSWORD!!!!\r\n", strlen("NEWPASSWORD!!!!\r\n"), osWaitForever);
				resetPasswordReadyFlag=0;
				xTaskNotify(resetPwordTaskHandle,OLD_PASSWORD,eSetValueWithOverwrite);
			}
			else{
				HAL_UART_Transmit(&huart2, "SECOND ELSE \r\n", strlen("SECOND ELSE \r\n"), osWaitForever);
				newPasswordFlag=0;
				resetPasswordReadyFlag=1;
				xTaskNotify(Green_LEDTaskHandle,DISARMED,eSetValueWithOverwrite);
			}
		}

		// If exceeded MAX_PASSWORD_LENGTH (6) or user presses enter key.
		else if(cnt==(MAX_PASSWORD_LENGTH-1) || ulNotificationValue==ENTER_KEY){
				// User presses enter and/or password length is between 4 and 6
				if((ulNotificationValue==ENTER_KEY && cnt>=MIN_PASSWORD_LENGTH) || cnt==(MAX_PASSWORD_LENGTH-1)){
					cnt=0;
					Clear_LCD_Line2Screen();

					// After user sets new password, we go back to DISARMED state
					if(newPasswordFlag){
						HAL_UART_Transmit(&huart2, "idk this is newpasswordflag\n\r", strlen("idk this is newpasswordflag\n\r"), osWaitForever);

						Write_to_Flash(START_ADDRESS, pword);
						xTaskNotify(Green_LEDTaskHandle,DISARMED,eSetValueWithOverwrite);
						newPasswordFlag=0;
						resetPasswordReadyFlag=1;
					}
					// If user is entering old password (to arm, disarm or to change passwords).
					else{
						
						// If user arms the alarm with the correct password (strcmp verifies correctness)
						if(!temp_is_armed && resetPasswordReadyFlag && !strcmp(pword,Read_from_Flash(START_ADDRESS))){
							uint8_t temp[100];
							sprintf(temp,"resetPasswordFlag = %d 1st if\n\r",resetPasswordReadyFlag);
							HAL_UART_Transmit(&huart2, temp, strlen(temp), osWaitForever);

							osMutexAcquire(isArmedMutexHandle, osWaitForever);
							is_armed=1;
							osMutexRelease(isArmedMutexHandle);
							xTaskNotify(Red_LEDTaskHandle,ARMED,eSetValueWithOverwrite);

						}
							
						// If user disarms the alarm with the correct password (strcmp verifies correctness)
						else if(temp_is_armed && resetPasswordReadyFlag && !strcmp(pword,Read_from_Flash(START_ADDRESS))){
							HAL_UART_Transmit(&huart2, "NOWAYBRUGHHH\n\r", strlen("NOWAYBRUGHHH\n\r"), osWaitForever);

							osMutexAcquire(isArmedMutexHandle, osWaitForever);
							is_armed=0;
							osMutexRelease(isArmedMutexHandle);
							xTaskNotify(Green_LEDTaskHandle,DISARMED,eSetValueWithOverwrite);
						}
							
						// If user wants to change password.
						// Correct system password must be entered (strcmp verifies correctness)
						else if(!resetPasswordReadyFlag && !strcmp(pword,Read_from_Flash(START_ADDRESS))){
							uint8_t temp[100];
							sprintf(temp,"resetPasswordFlag = %d 3rd elseif\n\r",resetPasswordReadyFlag);

							xTaskNotify(resetPwordTaskHandle,NEW_PASSWORD,eSetValueWithOverwrite);
							newPasswordFlag=1;
						}
					}
				}
			}
			// Generic user input. Cannot accept reset password key (#) when alarm is in AMRED state
			else if(!(temp_is_armed && ulNotificationValue==NEW_PASSWORD)){
				Write_To_LCD(cnt+1);
				cnt++;
			}
		}


  }
  /* USER CODE END StartLCDLine2Task */
}

/* USER CODE BEGIN Header_StartRED_LEDTask */
/**
* @brief Function implementing the Red_LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRED_LEDTask */
void StartRED_LEDTask(void *argument)
{
  /* USER CODE BEGIN StartRED_LEDTask */
	uint32_t ulNotificationValue;
  /* Infinite loop */
  for(;;)
  {
	  // Clear set bits on exit, and notifies the LCD to arm the system
	  if(xTaskNotifyWait(0, 0xffffffff, &ulNotificationValue, portMAX_DELAY)){
		  xTaskNotify(LCDLine1TaskHandle,ARMED,eSetValueWithOverwrite);
		  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
	  }
  }
  /* USER CODE END StartRED_LEDTask */
}

/* USER CODE BEGIN Header_StartGreen_LEDTask */
/**
* @brief Function implementing the Green_LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGreen_LEDTask */
void StartGreen_LEDTask(void *argument)
{
  /* USER CODE BEGIN StartGreen_LEDTask */
	uint32_t ulNotificationValue;
  /* Infinite loop */
  for(;;)
  {
	  // Clear set bits on exit, and notifies the LCD to disarm the system
	  if(xTaskNotifyWait(0, 0xffffffff, &ulNotificationValue, portMAX_DELAY)){
		// If it is a simple disarm, notify the LCD with disarm message,
		// Otherwise simply turn on Green LED
		if(ulNotificationValue==DISARMED){
			xTaskNotify(LCDLine1TaskHandle,DISARMED,eSetValueWithOverwrite);
		}
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
	  }
  }
  /* USER CODE END StartGreen_LEDTask */
}

/* USER CODE BEGIN Header_StartLCDLine1Task */
/**
* @brief Function implementing the LCDLine1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDLine1Task */
void StartLCDLine1Task(void *argument)
{
  /* USER CODE BEGIN StartLCDLine1Task */
	uint32_t ulNotificationValue;

  /* Infinite loop */
  for(;;)
  {
	  if(xTaskNotifyWait(0, 0xffffffff, &ulNotificationValue, portMAX_DELAY)){

		  // use mutex to synchronize use of shared ressource (LCD screen)
		  osMutexAcquire(myMutexHandle, osWaitForever);

		  //SYSTEM ARMED
		  if(ulNotificationValue==ARMED){

			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("SYSTEM ARMED   ");
		  }
		  //SYSTEM DISARMED
		  else if(ulNotificationValue==DISARMED){

			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("SYSTEM DISARMED");
		  }
		  //OLD PASSWORD
		  else if(ulNotificationValue==OLD_PASSWORD){
			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("OLD PASSWORD:    ");
		  }
		  //NEW PASSWORD
		  else if(ulNotificationValue==NEW_PASSWORD){
			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("NEW PASSWORD:  ");
		  }
		  else{
			  //CLEAR SCREEN, ERROR...
			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("SYSTEM ERROR... ");
		  }
		  osMutexRelease(myMutexHandle);
	  }
  }
  /* USER CODE END StartLCDLine1Task */
}

/* USER CODE BEGIN Header_StartResetPwordTask */
/**
* @brief Function implementing the resetPwordTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartResetPwordTask */
void StartResetPwordTask(void *argument)
{
  /* USER CODE BEGIN StartResetPwordTask */
	uint32_t ulNotificationValue;
  /* Infinite loop */
  for(;;)
  {
	  if(xTaskNotifyWait(0, 0xffffffff, &ulNotificationValue, portMAX_DELAY)){
		// Always notify green LED (with different notification value),
		// as we are in DISARMED state by default
		xTaskNotify(Green_LEDTaskHandle,OLD_PASSWORD,eSetValueWithOverwrite);

		// If Old password is being entered
		if(ulNotificationValue==OLD_PASSWORD){
			xTaskNotify(LCDLine1TaskHandle,OLD_PASSWORD,eSetValueWithOverwrite);
		}
		// If New password is being entered
		else if(ulNotificationValue==NEW_PASSWORD){
			xTaskNotify(LCDLine1TaskHandle,NEW_PASSWORD,eSetValueWithOverwrite);
		}
	  }
  }
  /* USER CODE END StartResetPwordTask */
}

/* USER CODE BEGIN Header_StartPIRsensorTask */
/**
* @brief Function implementing the PIRsensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIRsensorTask */
void StartPIRsensorTask(void *argument)
{
  /* USER CODE BEGIN StartPIRsensorTask */
  /* Infinite loop */
	uint32_t pulNotificationValue;
	uint8_t*temp_is_armed;
	uint8_t seconds_cnt=0;

  for(;;)
  {

		osMutexAcquire(isArmedMutexHandle, osWaitForever);
		temp_is_armed=is_armed;
		osMutexRelease(isArmedMutexHandle);

<<<<<<< HEAD

=======
>>>>>>> 5ca35f54417640d78976960d2eef37f787de28f0
		if(temp_is_armed && HAL_GPIO_ReadPin(PIR_Sensor_GPIO_Port, PIR_Sensor_Pin)==GPIO_PIN_SET){
			seconds_cnt=0;
			while(seconds_cnt<10){
				osDelay(1000);
				seconds_cnt++;
				uint8_t lol[100];
				sprintf(lol,(uint8_t*)"This many seconds: %d\r\n",seconds_cnt);
				HAL_UART_Transmit(&huart2, lol, strlen(lol), osWaitForever);
			}
			HAL_UART_Transmit(&huart2, (uint8_t*)"we can do pir!!!!!\r\n", strlen("we can do pir!!!!!\r\n"), osWaitForever);
			if(temp_is_armed && HAL_GPIO_ReadPin(PIR_Sensor_GPIO_Port, PIR_Sensor_Pin)==GPIO_PIN_SET){
				HAL_UART_Transmit(&huart2, (uint8_t*)"ABOUT TO START THE BUZZER\r\n", strlen("ABOUT TO START THE BUZZER\r\n"), osWaitForever);

				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,500);
				osDelay(2000);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
			}
		}
<<<<<<< HEAD


=======
>>>>>>> 5ca35f54417640d78976960d2eef37f787de28f0
  }
  /* USER CODE END StartPIRsensorTask */
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
