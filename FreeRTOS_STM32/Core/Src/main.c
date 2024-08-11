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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NTC_R 10000.0f
#define A 0.0008736528f
#define B 0.000253893f
#define C 0.0000001816f
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define tick 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;

osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task3Handle;
osThreadId Task4Handle;
osThreadId Task5Handle;
osThreadId Task6Handle;
osThreadId Task7Handle;
osSemaphoreId UARTHandle;
/* USER CODE BEGIN PV */

char Tx_data1[2]= "s";//
char Tx_data2[2]= "o";  									// control on
char Tx_data3[2]= "a";
char Rx_data[2];
char Tx_data4[2]= "f"; 										// control off

uint32_t ntc_adc = 0 ;
int temp = 0 ;												// bien trung gian doi tu adc => do C
uint8_t cbhn = 1 ;    										// 1 -  khong phat hien lua , 0 - phat hien lua
int warning ;										// 0 -  khong canh bao	, 1 - co canh bao

int connection = 0 ;
int time_check = 0 ;

int write_on	=	0	;
int write_off	=	0	;
int ntc_trunggian = 3000;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
void Task1function(void const * argument);
void task2functiom(void const * argument);
void task3function(void const * argument);
void task4function(void const * argument);
void task5function(void const * argument);
void task6function(void const * argument);
void task7function(void const * argument);

/* USER CODE BEGIN PFP */
void alarm(){
	  if (warning  == 1 ){
		  HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_14);
		  HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_15);
		// if(write_on==0){
			 // write_on =1;
			// write_off=0;
			  HAL_UART_Transmit(&huart1,(uint8_t*)Tx_data2,1,1000);

		 // }  		// o

	 }
	  else if(warning == 0){
		  HAL_GPIO_WritePin(GPIOB , GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB , GPIO_PIN_15, GPIO_PIN_RESET);

		  //f(write_off==0){
			 // write_off =1;
		//write_on=0;
			  HAL_UART_Transmit(&huart1,(uint8_t*)Tx_data4,1,1000);
		  //} 	// f
		  }
	  }

void check_connection(){
	if(connection < 5){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET ) ;
	
	}	
	else{
	
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	}
}
void ADC_convert(){
		int Ntc_R = 0;
		float Ntc_Tmp = 0 ;
		Ntc_R = ((NTC_R)/((4095.0/ntc_adc) - 1));
		float Ntc_Ln = log(Ntc_R);
		Ntc_Tmp = (1.0/(A + B*Ntc_Ln + C*Ntc_Ln*Ntc_Ln*Ntc_Ln)) - 273;
		temp = (int) Ntc_Tmp;
}
void write_check(){
	  	HAL_UART_Transmit(&huart1, (uint8_t*)Tx_data3, 1, 1000);	//&huart1,(uint8_t*)Tx_data3,1,1000
	  	connection++ ;
}

void read_CBHN(){
		//cbhn = 1;
		cbhn = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
}
void read_ADC(){
		int i; 
		int count =0;
		ntc_adc = 4000 ;
		for( i =0 ;i<100;i++) {count ++ ;}
		HAL_ADC_Start(&hadc1);
		for( i =0 ;i<10;i++) {count ++ ;}
		ntc_adc = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		ntc_trunggian = (int)ntc_adc;
	  }
void temp_check(){
	  if(cbhn == 0 ) {
		  warning = 1 ;
	  }
	  if(ntc_trunggian < 300 ) {
		  warning  = 1 ;
	  }
	  }

void warning_check(){
		  alarm();
	  }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
  if(GPIO_PIN == GPIO_PIN_5){
		
		if (warning == 0){
				warning = 1;
				HAL_UART_Transmit(&huart1,(uint8_t*)Tx_data2,1,1000);
				//alarm();
			}
			else //if ( warning == 1 )
				{
					HAL_GPIO_WritePin(GPIOB , GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB , GPIO_PIN_15, GPIO_PIN_RESET);
				warning = 0;
			HAL_UART_Transmit(&huart1,(uint8_t*)Tx_data4,1,1000);
				//alarm();
			}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
				//BaseType_t xHigher;
					osSemaphoreWait(UARTHandle,osWaitForever);
					if(huart->Instance == USART1){
						
					//	osSemaphoreRelease(BinarySemaphoreUARTHandle);
							//Rx_data[0] = 'z' ; // khoi tao gia tri ban dau
            	HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_data,2);
            	if(Rx_data[0]=='a') {
								connection = 0 ;
            	}
            	 if(Rx_data[0]=='o') { 
								//if(warning ==0 ){
								warning = 1;
							//}
            	}
							if(Rx_data[0]=='f') {
								//if(warning ==1){
								warning = 0 ;
								//}
            	}
  }
						 osSemaphoreRelease(UARTHandle);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
warning =0 ;
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
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_data,2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of UART */
  osSemaphoreDef(UART);
  UARTHandle = osSemaphoreCreate(osSemaphore(UART), 1);

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
  /* definition and creation of Task1 */
  osThreadDef(Task1, Task1function, osPriorityIdle, 0, 128);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, task2functiom, osPriorityLow, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* definition and creation of Task3 */
  osThreadDef(Task3, task3function, osPriorityBelowNormal, 0, 128);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  /* definition and creation of Task4 */
  osThreadDef(Task4, task4function, osPriorityBelowNormal, 0, 128);
  Task4Handle = osThreadCreate(osThread(Task4), NULL);

  /* definition and creation of Task5 */
  osThreadDef(Task5, task5function, osPriorityNormal, 0, 128);
  Task5Handle = osThreadCreate(osThread(Task5), NULL);

  /* definition and creation of Task6 */
  osThreadDef(Task6, task6function, osPriorityNormal, 0, 128);
  Task6Handle = osThreadCreate(osThread(Task6), NULL);

  /* definition and creation of Task7 */
  osThreadDef(Task7, task7function, osPriorityAboveNormal, 0, 128);
  Task7Handle = osThreadCreate(osThread(Task7), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 624;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task1function */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task1function */
void Task1function(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task2functiom */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task2functiom */
void task2functiom(void const * argument)
{
  /* USER CODE BEGIN task2functiom */
  /* Infinite loop */
  for(;;)
  {
		alarm();
    osDelay(1000);
  }
  /* USER CODE END task2functiom */
}

/* USER CODE BEGIN Header_task3function */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task3function */
void task3function(void const * argument)
{
  /* USER CODE BEGIN task3function */
  /* Infinite loop */
  for(;;)
  {	check_connection();
    osDelay(1000);
  }
  /* USER CODE END task3function */
}

/* USER CODE BEGIN Header_task4function */
/**
* @brief Function implementing the Task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task4function */
void task4function(void const * argument)
{
  /* USER CODE BEGIN task4function */
  /* Infinite loop */
  for(;;)
  {
		read_CBHN();
    osDelay(200);
  }
  /* USER CODE END task4function */
}

/* USER CODE BEGIN Header_task5function */
/**
* @brief Function implementing the Task5 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task5function */
void task5function(void const * argument)
{
  /* USER CODE BEGIN task5function */
  /* Infinite loop */
  for(;;)
  {	
		read_ADC();
    osDelay(200);
  }
  /* USER CODE END task5function */
}

/* USER CODE BEGIN Header_task6function */
/**
* @brief Function implementing the Task6 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task6function */
void task6function(void const * argument)
{
  /* USER CODE BEGIN task6function */
  /* Infinite loop */
  for(;;)
  {
		temp_check();
    osDelay(800);
  }
  /* USER CODE END task6function */
}

/* USER CODE BEGIN Header_task7function */
/**
* @brief Function implementing the Task7 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task7function */
void task7function(void const * argument)
{
  /* USER CODE BEGIN task7function */
  /* Infinite loop */
  for(;;)
  {
		write_check();
    osDelay(100);
		HAL_IWDG_Refresh(&hiwdg);
		
  }
  /* USER CODE END task7function */
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
