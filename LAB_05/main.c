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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  //ENABLE GPIO B
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;   //ENABLE GPIO C
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;  //ENABLE I2C2

  SystemClock_Config();
	

// Configure the leds and button
GPIOC->MODER |= (1<<12) | (1<<14) | (1<<16) | (1<<18);
GPIOC->MODER &= ~((1<<13) | (1<<15) | (1<<17) | (1<<19));
GPIOC->OTYPER &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));
GPIOC->OSPEEDR &= ~((1<<12) | (1<<14) | (1<<16) | (1<<18));
GPIOC->PUPDR &= ~((1<<12) | (1<<14) | (1<<16) | (1<<18) | (1<<13) | (1<<15) | (1<<17) | (1<<19));
GPIOC->OSPEEDR &= ~((1<<0) | (1<<1));


// SETTING PINS TO ALTERNATE MODE
	GPIOB->MODER |= (1<<23) |(0<<22) |(0<<26) | (1<<27); 		//PB11 and PB13 to alternate mode
	GPIOB->MODER |= (0<<25) |(1<<24) ; 		   //PB14 TO OUTPUT MODE
	GPIOC->MODER |= (1<<0) | (0<<1) ;				//PC0 TO OUTPUT MODE 
	GPIOB->MODER &= ~((0<<22) | (0<<26));								
	GPIOB->OTYPER &= ~ (1<<11) | (1<<13);       					//PB11 AND PB13 TO OUTPUT OPEN DRAIN 
	GPIOB->OSPEEDR &= ~((1<<20) | (1<<21) | (1<<24) | (1<<25));   
	GPIOB->PUPDR &= ((1<<25) | (1<<24) );			//PB14 TO PUPD
	GPIOC->PUPDR &=  ((1<<0) | (1<<1));			//PC0 TO PUPD
	GPIOB->ODR |= (1<<14);			//PB14 TO HIGH
	GPIOC->ODR |=  (1<<0);			//PC0 TO HIGH
	GPIOB->AFR[1] |= (1<<12) | (0<<13) | (0<<14) | (0<<15) ;   //PB11 TO I2C_SDA 
	GPIOB->AFR[1] |= (0<<23)|(1<<22) | (0<<21)| (1<<20);				//PB13 TO I2C_SCL
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
