
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//sinusoidal data for 256 point
const uint16_t PWMdata[256]={
2884,2954,3025,3095,3165,3235,3305,3375,3444,3512,
3581,3648,3715,3782,3848,3914,3978,4042,4105,4168,
4229,4290,4349,4408,4466,4522,4578,4633,4686,4738,
4789,4839,4887,4934,4980,5024,5067,5109,5149,5187,
5224,5260,5294,5326,5357,5387,5414,5440,5465,5487,
5508,5528,5545,5561,5575,5588,5598,5607,5614,5614,
5614,5614,5614,5614,5614,5614,5607,5598,5588,5575,
5561,5545,5528,5508,5487,5465,5440,5414,5387,5357,
5326,5294,5260,5224,5187,5149,5109,5067,5024,4980,
4934,4887,4839,4789,4738,4686,4633,4578,4522,4466,
4408,4349,4290,4229,4168,4105,4042,3978,3914,3848,
3782,3715,3648,3581,3512,3444,3375,3305,3235,3165,
3095,3025,2954,2884,2813,2742,2672,2601,2531,2461,
2391,2321,2251,2182,2114,2045,1978,1911,1844,1778,
1712,1648,1584,1521,1458,1397,1336,1277,1218,1160,
1104,1048,993 ,940 ,888 ,837 ,787 ,739 ,692 ,646 ,
602 ,559 ,517 ,477 ,439 ,402 ,366 ,332 ,300 ,269 ,
239 ,212 ,186 ,161 ,139 ,118 ,98 ,81 ,65 ,51 ,
38 ,28 ,19 ,12 ,10 ,10 ,10 ,10 ,10 ,10 ,
12 ,19 ,28 ,38 ,51 ,65 ,81 ,98 ,118 ,139 ,
161 ,186 ,212 ,239 ,269 ,300 ,332 ,366 ,402 ,439 ,
477 ,517 ,559 ,602 ,646 ,692 ,739 ,787 ,837 ,888 ,
940 ,993 ,1048,1104,1160,1218,1277,1336,1397,1458,
1521,1584,1648,1712,1778,1844,1911,1978,2045,2114,
2182,2251,2321,2391,2461,2531,2601,2672,2742,2813,
2884,2954,3025,3095,3165,3235,
};

const uint16_t PWMdata2[256]={
2812, 2882, 2952, 3022, 3093, 3163, 3232, 3302, 3371, 3440, 3508, 3576, 3644, 3711, 3777, 3843, 3908, 3972, 4036, 4099, 4161, 4223, 4283, 4343, 4401,
4459, 4515, 4571, 4625, 4678, 4730, 4781, 4831, 4879, 4926, 4972, 5016, 5059, 5101, 5141, 5180, 5217, 5253, 5287, 5319, 5350, 5380, 5408, 5434, 5459,
5481, 5503, 5522, 5540, 5556, 5571, 5583, 5594, 5604, 5611, 5617, 5621, 5623, 5623, 5622, 5619, 5614, 5608, 5599, 5589, 5577, 5564, 5548, 5531, 5513,
5492, 5470, 5446, 5421, 5394, 5365, 5335, 5303, 5270, 5235, 5198, 5161, 5121, 5080, 5038, 4994, 4949, 4903, 4855, 4806, 4756, 4704, 4652, 4598, 4543,
4487, 4430, 4372, 4313, 4253, 4192, 4130, 4068, 4004, 3940, 3875, 3810, 3744, 3677, 3610, 3542, 3474, 3405, 3336, 3267, 3197, 3128, 3058, 2987, 2917,
2847, 2776, 2706, 2636, 2565, 2495, 2426, 2356, 2287, 2218, 2149, 2081, 2013, 1946, 1879, 1813, 1748, 1683, 1619, 1555, 1493, 1431, 1370, 1310, 1251,
1193, 1136, 1080, 1025, 971, 919, 867, 817, 768, 720, 674, 629, 585, 543, 502, 462, 425, 388, 353, 320, 288, 258, 229, 202, 177, 153, 131, 110, 92,
75, 59, 46, 34, 24, 15, 9, 4, 1, 0, 0, 2, 6, 12, 19, 29, 40, 52, 67, 83, 101, 120, 142, 164, 189, 215, 243, 273, 304, 336, 370, 406, 443, 482, 522,
564, 607, 651, 697, 744, 792, 842, 893, 945, 998, 1052, 1108, 1164, 1222, 1280, 1340, 1400, 1462, 1524, 1587, 1651, 1715, 1780, 1846, 1912, 1979, 2047,
2115, 2183, 2252, 2321, 2391, 2460, 2530, 2601, 2671, 2741, 2812};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i = 0;
  int v = 84;
  int s = 168;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  for (int j = 0;j<=10000;j++){}

	  htim2.Instance->CCR4 = PWMdata2[s]/37;
	  htim2.Instance->CCR2 = PWMdata2[v]/37;
	  htim2.Instance->CCR3 = PWMdata2[i]/37;

	  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWMdata[s]);
	  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWMdata[v]);
	  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWMdata[i]);
	  i ++;
	  v ++;
	  s ++;
	  if (i == 251){
		  i = 0;
	  }
	  if (v == 251){
		  v = 0;
	  }
	  if (s == 251){
		  s = 0;
	  }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim2.Init.Period = 11248;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_7B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin PB6 */
  GPIO_InitStruct.Pin = LED1_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */

	int i = 0;
	int v = 84;
	int s = 168;

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
	for (int j = 0; j <= 10000; j++) {
	}

	htim2.Instance->CCR4 = PWMdata2[s] / 37;
	htim2.Instance->CCR2 = PWMdata2[v] / 37;
	htim2.Instance->CCR3 = PWMdata2[i] / 37;

	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWMdata[s]);
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWMdata[v]);
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWMdata[i]);
	i++;
	v++;
	s++;
	if (i == 251) {
		i = 0;
	}
	if (v == 251) {
		v = 0;
	}
	if (s == 251) {
		s = 0;

	}
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
