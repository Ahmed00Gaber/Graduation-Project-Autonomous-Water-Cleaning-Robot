/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "NRF24L01.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//Motors

uint32_t ADC_Read_speed;
uint32_t Left_motor_speed;

uint32_t ADC_Read_Direction;
uint32_t Right_motor_speed;
//NRF
uint8_t NRF_RxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA}; //Receiver address
uint8_t NRF_RxData[1];

uint8_t rxBuffer[4];  // Buffer to store received data
uint8_t bufferIndex = 0;
uint8_t receivedByte;
int receivedNumber;

// Camera vision
uint16_t centerValue = 350; // This is your center reference value
uint16_t maxMotorSpeed = 800; // Maximum speed of the motor
uint16_t minMotorSpeed = 500; // Minimum speed of the motor
uint16_t speedAdjustment;
uint8_t someThreshold = 30;
uint8_t someMaxError = 200;

#define calibrate 1

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for Task_1 */
osThreadId_t Task_1Handle;
const osThreadAttr_t Task_1_attributes = {
  .name = "Task_1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_2 */
osThreadId_t Task_2Handle;
const osThreadAttr_t Task_2_attributes = {
  .name = "Task_2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
void Task_one(void *argument);
void Task_Two(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
 return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
 }

uint32_t constrain(uint32_t value, uint32_t minValue, uint32_t maxValue) {
    if (value < minValue) {
        return minValue;
    } else if (value > maxValue) {
        return maxValue;
    } else {
        return value;
    }
}
void adjustMotorSpeed(uint32_t potValue, uint32_t joystickXValue) {
	//move right
	if(joystickXValue> 2070)
	{
		Left_motor_speed= map(potValue, 0, 4096, 500, 950);
		Right_motor_speed=map(joystickXValue,2048,4096,650,500);
		//set speed
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, Left_motor_speed);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, Right_motor_speed);
	}
	else if(joystickXValue< 2000)
	{
		Right_motor_speed= map(potValue, 0, 4096, 500, 950);
		Left_motor_speed=map(joystickXValue,0,2048,500,650);
		//set speed
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, Left_motor_speed);
	    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, Right_motor_speed);
	}
	else{
		Right_motor_speed= map(potValue, 0, 4096, 500, 950);
		Left_motor_speed=map(potValue, 0, 4096, 500, 950);
		//set speed
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, Left_motor_speed);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, Right_motor_speed);
	}

}



/*
 * breif: Set motor speeds based on the direction of the error
 * breif::Object is to the left, move right motor
 */
void camera_move_left_motor(uint16_t speedAdjustment){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, maxMotorSpeed - speedAdjustment); //right motor
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, minMotorSpeed + speedAdjustment); // left motor
}

/*
 * breif: Object is to the left, move right motor
 */
void camera_move_right_motor(uint16_t speedAdjustment){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, minMotorSpeed + speedAdjustment);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, maxMotorSpeed - speedAdjustment);
}

//new functions

void SetMotorSpeed(uint16_t speed)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

uint16_t CalculateSpeedAdjustment(uint16_t error)
{
  if(abs(error) > someThreshold)
  {
    uint16_t adjustment = (maxMotorSpeed - minMotorSpeed) * abs(error) / someMaxError;
    return (adjustment > (maxMotorSpeed - minMotorSpeed)) ? (maxMotorSpeed - minMotorSpeed) : adjustment;
  }
  else
  {
    return 0;
  }
}
void AdjustMotorSpeedBasedOnError(int16_t error)
{
  uint16_t speedAdjustment = CalculateSpeedAdjustment(error);

  if(error > 0)
  {
    // Object is to the right, adjust left motor
    // Assuming camera_move_left_motor adjusts the motor speed
    camera_move_left_motor(speedAdjustment);
  }
  else if(error < 0)
  {
    // Object is to the left, adjust right motor
    camera_move_right_motor(speedAdjustment);
  }
  else
  {
    // Object is centered, set default speed
    SetMotorSpeed(550);
    //set timer
  }
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //NRF intialization
    NRF24_Init();//init NRF
    NRF24_RxMode(NRF_RxAddress, 10); //NRF RX mode

    //Timer Intilallization
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //start PWM timer
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //start PWM timer

    //Calibration for Beeb of motos
  #if calibrate
  	TIM2->CCR1 =1000; //set max pulse (2ms)
  	TIM2->CCR2 =1000;
  	HAL_Delay(2000);  //wait for 1 beeb
  	TIM2->CCR1 =500; //set min pulse (1ms)
  	TIM2->CCR2 =500;
  	HAL_Delay(1000);  //wait for 2 beeb
  //	TIM2->CCR1 =0;  // reset to 0, so it can be controlled via ADC
  //	TIM2->CCR2 =0;

  #endif
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
  /* creation of Task_1 */
  Task_1Handle = osThreadNew(Task_one, NULL, &Task_1_attributes);

  /* creation of Task_2 */
  Task_2Handle = osThreadNew(Task_Two, NULL, &Task_2_attributes);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task_one */
/**
  * @brief  Function implementing the Task_1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task_one */
void Task_one(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	for(;;)
	  {
		HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, &receivedByte, 1, HAL_MAX_DELAY);

		    if (status == HAL_OK)
		    {
		      // Toggle LED on receiving data for visual indication
		      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

		      // Check if end of line or buffer is about to overflow
		      if (receivedByte == '\n' || bufferIndex >= sizeof(rxBuffer) - 1)
		      {
		        // Null-terminate the received data
		        rxBuffer[bufferIndex] = '\0';
		        receivedNumber = atoi((char *)rxBuffer);
		        int16_t error = receivedNumber - centerValue;

		        // Handle special command
		        if(receivedNumber == 1)
		        {
		          SetMotorSpeed(550); // Set both motors to speed 550
		        }
		        else
		        {
		          AdjustMotorSpeedBasedOnError(error);
		        }

		        bufferIndex = 0; // Reset the buffer index for next message
		      }
		      else
		      {
		        // Store received byte and increment buffer index
		        rxBuffer[bufferIndex++] = receivedByte;
		      }
		    }

		osDelay(20);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_Two */
/**
* @brief Function implementing the Task_2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Two */
void Task_Two(void *argument)
{
  /* USER CODE BEGIN Task_Two */
  /* Infinite loop */
	for(;;)
	  {
		  //Remote controll code
		  if(isDataAvailable(1) == 1 )
		  {
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			  NRF24_Receive(NRF_RxData);
			  ADC_Read_speed = map(NRF_RxData[0],0,255,0,4096);
			  ADC_Read_Direction = map(NRF_RxData[1],0,255,0,4096);
			  adjustMotorSpeed(ADC_Read_speed,ADC_Read_Direction);
		  }
		  osDelay(5);
	  }
  /* USER CODE END Task_Two */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
