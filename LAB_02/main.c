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
volatile int counts = 0;
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void EXTI0_1_IRQHandler(void) {
    
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);			// Toggle green LED (PC9)
    
    
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); 				// Toggle orange LED (PC8)
    
		// Delay loop									//  PART 2 //
    //for (volatile uint32_t i = 0; i < 1500000; i++) {
																													// This loop provides a rough delay of 1-2 seconds
    //}

    
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);			// Toggle green LED again (PC9)

    
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);		// Toggle orange LED again (PC8)
		
    
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);				// Clear the EXTI interrupt flag 
}
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
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock 
	__HAL_RCC_GPIOA_CLK_ENABLE();  //Enable the GPIOA clock 
	__HAL_RCC_SYSCFG_CLK_ENABLE();  //Enable the SYSCFG clock 
	
	
	GPIO_InitTypeDef initStr = {GPIO_PIN_6| GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6, PC7, PC8, & PC9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);//sets pin 9 to start high
	
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
		
  /* USER CODE END 2 */
	GPIO_InitTypeDef initStrs = {GPIO_PIN_0, 
	                             GPIO_MODE_INPUT, 
	                             GPIO_SPEED_FREQ_LOW, 
	                             GPIO_PULLDOWN};
	HAL_GPIO_Init(GPIOA,&initStrs); // Initialize pin 0
															 
	EXTI->IMR |= (1 << 0);  //SET EXTI0 TO ALLOW INTERRUPTS
	EXTI->RTSR |= (1 << 0);	  // SET EXTI0 TO TRIGGER ON A RISING EDGE
	
	
															 
	SYSCFG->EXTICR[0] |= (0 << 0);  //SET SYSCFG TO ALLOW EXTI0 
															 
	NVIC_EnableIRQ(EXTI0_1_IRQn);  //ENABLE THE EXTI0 INTERRUPT
  NVIC_SetPriority(EXTI0_1_IRQn, 1);  // SETTING PRIORITY TO 1
													
	//NVIC_SetPriority(SysTick_IRQn, 2);//  PART 2 //SET EXTI0_1 to prioirty 3 in part 2
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);//sets pin 8 to start high													 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_Delay(500);			//DELAY OF 500 ms
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);  // TOGGLE PC6
		
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
