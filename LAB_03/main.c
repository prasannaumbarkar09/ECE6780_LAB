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
	
void TIM2_IRQHandler(){
	//toggle LEDs green and orange
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9; 
	// Reset pending flag
	TIM2->SR &= ~(1<<0);
}
	
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;	// Enable Timer 2 peripheral
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	//Enable Timer 3 peripheral
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIOC clock
	
	// Clear the bits for PC6, PC7, PC8 and PC9
  GPIOC->MODER &= ~(3 << 16);
  GPIOC->MODER &= ~(3 << 18);
  // Setting PC6, PC7, PC8 and PC9 to General-Purpose Output Mode
  GPIOC->MODER |= (1 << 16) | (1 << 18);
  // Setting PC6, PC7, PC8 and PC9 to Push-Pull Output Type
  GPIOC->OTYPER &= ~(1 << 8);
  GPIOC->OTYPER &= ~(1 << 9);
  // Set PC6, PC7, PC8 and PC9 to Low Speed
  GPIOC->OSPEEDR &= ~(0 << 16);
  GPIOC->OSPEEDR &= ~(0 << 18);
  // Clear the bits for PC6, PC7, PC8 and PC9
  // This also sets the pull-up/pull-down resistors to no pull-up/pull-down since the bits are 00
  GPIOC->PUPDR &= ~(3 << 16);
  GPIOC->PUPDR &= ~(3 << 18);

	// Setting one LED high and other low
	GPIOC->ODR |= (1 << 8);  // Setting orange LED (PC8) high
	GPIOC->ODR &= ~(1 << 9);  // Setting Green LED (PC9) low 
	
	// Set the PSC and ARR for 4Hz
	TIM2->PSC = 7999;  
	TIM2->ARR = 250;
	
	// Generate an interrupt on the UEV event (4Hz)
	TIM2->DIER |= TIM_DIER_UIE;
	
	// Enabling Timer 2 control register to start
	TIM2->CR1 |= TIM_CR1_CEN;
	
	// Enabling the interrupt handler for TIM2
	NVIC_EnableIRQ(TIM2_IRQn);
	
	// Configuring timer channels to PWM mode
	 
	//Set the PSC and ARR for 800Hz
	 TIM3->PSC = 9; 
   TIM3->ARR = 1000; 

    // Configure Timer 3 channel 1 for PWM Mode 2
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk); // Clear bits
    TIM3->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM Mode 2

    // Configure Timer 3 channel 2 for PWM Mode 1
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk); // Clear bits
    TIM3->CCMR1 |= (TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1); // PWM Mode 1

    // Enable output for Timer 3 channel 1 and channel 2
    TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

    // Set initial capture/compare values (CCR) for both channels
    TIM3->CCR1 = 10; // 20% duty cycle for channel 1
    TIM3->CCR2 = 500; // 20% duty cycle for channel 2

    // Enable Timer 3
    TIM3->CR1 |= TIM_CR1_CEN;
	
	// Configure PC6 (red LED) and PC7 (blue LED) as alternate function mode
    GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk); // Clear bits
    GPIOC->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // Set alternate function mode (AF)
		
		// Configure PC6 and PC7 for alternate function AF0 (TIM3_CH1 and TIM3_CH2)
    GPIOC->AFR[0] &= ~((0xF << GPIO_AFRL_AFRL6_Pos) | (0xF << GPIO_AFRL_AFRL7_Pos)); // Clear bits
    GPIOC->AFR[0] |= (0 << GPIO_AFRL_AFRL6_Pos) | (0 << GPIO_AFRL_AFRL7_Pos); // Set AF0

    // Configure PC6 and PC7 as push-pull outputs
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7); // Push-pull mode

    // Configure PC6 and PC7 as low-speed outputs
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk); // Clear bits

    // Configure PC6 and PC7 with no pull-up or pull-down
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk); // Clear bits
	


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
