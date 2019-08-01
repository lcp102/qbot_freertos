/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
UART_HandleTypeDef huart2;
osThreadId defaultTaskHandle;
QueueHandle_t moveQueue = NULL;
QueueHandle_t xPosQueue = NULL;
QueueHandle_t yPosQueue = NULL;
QueueHandle_t zAngQueue = NULL;
QueueHandle_t vMaxQueue = NULL;

deviceParams_t initDeviceParameters =
{
		PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B,
		{L6206_CONF_PARAM_FREQ_PWM1A, L6206_CONF_PARAM_FREQ_PWM2A, L6206_CONF_PARAM_FREQ_PWM1B, L6206_CONF_PARAM_FREQ_PWM2B},
		{100,100,100,100},
		{FORWARD,BACKWARD,FORWARD,BACKWARD},
		{INACTIVE,INACTIVE,INACTIVE,INACTIVE},
		{TRUE,TRUE}
};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
static void MyFlagInterruptHandler(void);
void ButtonHandler(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void led_on_off(void *pvParameters)
{
	TickType_t xDelay = 1000;
	for (;;) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		vTaskDelay(xDelay);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		vTaskDelay(xDelay);
	}
}

/*
 * controls the motors
 * 1. receive updates of max velocity
 * 2. receive updates of x distance
 * 3. receive updates of y distance
 * 4. receive updates of z angle
 * 5. receive command to move
 * 5. move == 'mndg' --> move state:
 * 			g = 0 for stop, 1 for go
 * 			d = 0 for forward, 1 for reverse
 * 			n = 0 for motor 0, 1 for motor 1
 */
void motor_control(void *pvParameters)
{
	TickType_t loop_delay = 100;
	TickType_t command_delay = 75;
	uint16_t max_speed = 0;
	BaseType_t queueStatus;
	int cmd_vel = 0;
	uint8_t move = STOP;
	uint8_t motor_number = MOTOR0;
	motorDir_t direction = BACKWARD;
	const TickType_t xTicksToWait = 0;
	/*
	 * loop continuously
	 * update x, y, and, and max velocity values as they appear in their queues
	 * move when a true value is received in it's queue
	 */
	for (;;) {
		/*
		 * receive max velocity
		 */
		queueStatus = xQueueReceive(vMaxQueue, &cmd_vel, xTicksToWait);
		if (queueStatus == pdPASS) {
			if (cmd_vel > 100) {
				cmd_vel = 100;
			}
		}
		/*
		 * receive move command state (true = move, false = stop)
		 */
		queueStatus = xQueueReceive(moveQueue, &move, xTicksToWait);
		if (queueStatus == pdPASS) {
			if ((move & GO) == 1) {
				max_speed = (uint16_t)cmd_vel;
			}
			else {
				max_speed = 0;
			}
			if ((move & MOVE_FORWARD) > 0) {
				direction = FORWARD;
			}
			else {
				direction = BACKWARD;
			}
			if ((move & DRIVE_MOTOR1) > 0) {
				motor_number = MOTOR1;
			}
			else {
				motor_number = MOTOR0;
			}
		}
		BSP_MotorControl_SetMaxSpeed(motor_number, max_speed);
		vTaskDelay(command_delay);
		BSP_MotorControl_Run(motor_number, direction);
		vTaskDelay(command_delay);
		vTaskDelay(loop_delay);
	}
}

/*
 * receive bytes into the UART port
 * check the leading char
 * 1. == 'v' --> max velocity
 * 2. == 'x' --> x distance, i.e. linear.x
 * 3. == 'y' --> y distance, i.e. linear.y
 * 4. == 'a' --> angle, i.e. angular.z
 * 5. == 'mndg' --> move state:
 * 			g = 0 for stop, 1 for go
 * 			d = 0 for forward, 1 for reverse
 * 			n = 0 for motor 0, 1 for motor 1
 */
void rec_bytes(void *pvParameters)
{
	TickType_t loop_delay = 200;
	uint16_t buff_size = 5;
	uint8_t rec_buff[buff_size];
	uint8_t trans_buff[buff_size];
	uint32_t transmit_delay = 100;
	HAL_StatusTypeDef ret_value = HAL_OK;
	BaseType_t queueStatus;
	int cmd_vel = 0;
	uint8_t move = 0;
	const TickType_t xTicksToWait = 0;
	for (;;) {
		ret_value = HAL_UART_Receive(&huart2,
				rec_buff,
				buff_size,
				HAL_MAX_DELAY);
		/*
		 * if the receive was successful, read leading char to decode message
		 */
		if (ret_value == HAL_OK) {
			/*
			 * check for max velocity change
			 */
			if ((uint8_t)'v' == rec_buff[HEAD_CHAR_BIT_POS]) {
				/* convert string to decimal number */
				rec_buff[HEAD_CHAR_BIT_POS] = (uint8_t)'0';
				int base = 10;
				cmd_vel = (int)strtol((const char *)rec_buff, NULL, base);
				queueStatus = xQueueSendToBack(vMaxQueue, &cmd_vel, xTicksToWait);
				rec_buff[HEAD_CHAR_BIT_POS] = (uint8_t)'V';
				if (queueStatus != pdPASS) {
					HAL_UART_Transmit(&huart2,
							(uint8_t *)"vel send fail\n\r",
							15,
							transmit_delay);
				}
			}
			/*
			 * check for move state
			 */
			else if ((uint8_t)'m' == rec_buff[HEAD_CHAR_BIT_POS]) {
				/* convert string to decimal number */
				rec_buff[HEAD_CHAR_BIT_POS] = (uint8_t)'0';
				/*
				 * reset move before setting new command value
				 */
				move = 0;
				/*
				 * stop or go state
				 */
				if ((uint8_t)'1' == rec_buff[HEAD_CHAR_BIT_POS + 3]) {
					move = move | GO;
				}
				else {
					move = move | STOP;
				}
				/*
				 * motor rotation direction
				 */
				if ((uint8_t)'1' == rec_buff[HEAD_CHAR_BIT_POS + 2]) {
					move = move | MOVE_FORWARD;
				}
				else {
					move = move | BACKWARD;
				}
				/*
				 * select which motor to drive
				 */
				if ((uint8_t)'1' == rec_buff[HEAD_CHAR_BIT_POS + 1]) {
					move = move | DRIVE_MOTOR1;
				}
				else {
					move = move | MOTOR0;
				}
				queueStatus = xQueueSendToBack(moveQueue, &move, xTicksToWait);
				rec_buff[HEAD_CHAR_BIT_POS] = (uint8_t)'M';
				if (queueStatus != pdPASS) {
					HAL_UART_Transmit(&huart2,
							(uint8_t *)"move send fail\n\r",
							16,
							transmit_delay);
				}
			}
			else {
				rec_buff[HEAD_CHAR_BIT_POS] = (uint8_t)'n';
				rec_buff[HEAD_CHAR_BIT_POS + 1] = (uint8_t)'o';
				rec_buff[HEAD_CHAR_BIT_POS + 2] = (uint8_t)'p';
			}
			/* copy receive buffer contents to transmit buffer */
			memcpy(trans_buff, rec_buff, buff_size);
			/* clear the receive buffer */
			memset(rec_buff, 0, buff_size);
			HAL_UART_Transmit(&huart2,
					trans_buff,
					buff_size,
					transmit_delay);
		}
		else {
			HAL_UART_Transmit(&huart2,
					(uint8_t *)"fail\n\r",
					6,
					transmit_delay);
		}
		vTaskDelay(loop_delay);
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
  
    /* Configure KEY Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Set Systick Interrupt to the highest priority to have HAL_Delay working*/
  /* under the user button handler */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0);

  //----- Init of the Motor control library
  /* Set the L6208 library to use 1 device */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6206, 2);
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the L6206 parameters are set with the predefined values from file        */
  /* l6206_target_config.h, otherwise the parameters are set using the        */
  /* initDeviceParameters structure values.                                   */
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6206, &initDeviceParameters);

  /* Select the configuration with paralleling of bridge input IN1A with IN2A,
  and with paralleling of bridge input IN1B with IN2B with the use of one
  bidirectional motor */
  BSP_MotorControl_SetDualFullBridgeConfig(PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B);

  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function Error_Handler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(Error_Handler);

  /* Set PWM Frequency of bridge A inputs to 10000 Hz */
  BSP_MotorControl_SetBridgeInputPwmFreq(0,10000);

  /* Set PWM Frequency of bridge B inputs to 10000 Hz */
  BSP_MotorControl_SetBridgeInputPwmFreq(1,10000);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  vMaxQueue = xQueueCreate(1, sizeof(int));
  //moveQueue = xQueueCreate(1, sizeof(bool));
  moveQueue = xQueueCreate(1, sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /*
   * if the Queue was created successfully, create the tasks
   */
  if (moveQueue != NULL) {
	  xTaskCreate(led_on_off,
			  	  (const char * const)"led_control",
				  configMINIMAL_STACK_SIZE,
				  0,
				  2,
				  0);
	  xTaskCreate(motor_control,
	  		  	  (const char * const)"motor_control",
	  			  configMINIMAL_STACK_SIZE,
	  			  0,
	  			  2,
	  			  0);
	  xTaskCreate(rec_bytes,
	  		  	  (const char * const)"uart_interface",
	  			  configMINIMAL_STACK_SIZE,
	  			  0,
	  			  2,
	  			  0);
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(0);
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler(0);
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
    Error_Handler(0);
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

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
//void Error_Handler(void)
//{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
//}

void Error_Handler(uint16_t error)
{

  /* Infinite loop */
  while(1)
  {
  }
}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  /* Code to be customised */
  /************************/
  /* Get the state of bridge A */
  uint16_t bridgeState  = BSP_MotorControl_CmdGetStatus(0);

  if (bridgeState == 0)
  {
    if (BSP_MotorControl_GetDeviceState(0) != INACTIVE)
    {
      /* Bridge A was disabling due to overcurrent or over temperature */
      /* When  motor was running */
        Error_Handler(0XBAD0);
    }
  }
 }

/**
  * @brief  This function is executed when the Nucleo User button is pressed
  * @param  error number of the error
  * @retval None
  */
void ButtonHandler(void)
{

  /* Let 300 ms before clearing the IT for key debouncing */
  HAL_Delay(300);
  __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
  HAL_NVIC_ClearPendingIRQ(KEY_BUTTON_EXTI_IRQn);
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
