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
	
							/////PART 1////
int main(void) {
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
												/* This example uses HAL library calls to control
							             the GPIOC peripheral. You’ll be redoing this code
													 with hardware register access. */
	//__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
																// Set up a configuration struct to pass to the initialization function
	//GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7,
	//GPIO_MODE_OUTPUT_PP,
	//GPIO_SPEED_FREQ_LOW,
	//GPIO_NOPULL};
	//HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Start PC8 high
	
	////  PART1 /////
	/*
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;				 //ENABLNG GPIO PORT C BY REGISTER 
	GPIOC->MODER |= GPIO_MODER_MODER8_0  ;		 // SETTING PIN 8 TO GENERAL-PURPOSE OUTPUT IN MODER REG
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 );			//SETTING PIN  8 TO PUSH-PULL OUTPUT
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8);		//SETTING PIN  8 TO LOW SPEED
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);					//SETTING PIN  8 TO PULLUP-PULLDOWN 
	
	
	GPIOC->MODER|= GPIO_MODER_MODER9_0 ;  			// SETTING PIN 9 TO GENERAL-PURPOSE OUTPUT IN MODER REG
	GPIOC->OTYPER &= ~( GPIO_OTYPER_OT_9);				//SETTING PIN  9 TO PUSH-PULL OUTPUT
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR9);		//SETTING PIN  9 TO LOW SPEED
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);				//SETTING PIN  9 TO PULLUP-PULLDOWN 

	GPIOC->BSRR = GPIO_BSRR_BR_8;      //RESET PIN 8
	GPIOC->BSRR = GPIO_BSRR_BS_9; 		// SET PC9 HIGH

while (1) {    
	HAL_Delay(200);	// DELAY OF 200ms
	GPIOC-> ODR ^= GPIO_PIN_8 | GPIO_PIN_9;				// Toggle the output state of both PC8 and PC9
					
																							//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);

} */
	
	
	//// PART2 ///
	
	RCC-> AHBENR |= RCC_AHBENR_GPIOAEN; 				// ENABLING GPIO PORT A BY REGISTER 
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;				 //ENABLNG GPIO PORT C BY REGISTER 
	GPIOC->MODER |= GPIO_MODER_MODER6_0  ;		 // SETTING PIN 6 TO GENERAL-PURPOSE OUTPUT IN MODER REG
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 );			//SETTING PIN  6 TO PUSH-PULL OUTPUT
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6);		//SETTING PIN  6 TO LOW SPEED
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);					//SETTING PIN  6 TO PULLUP-PULLDOWN 

	GPIOC->MODER |= GPIO_MODER_MODER7_0  ;		 // SETTING PIN 7 TO GENERAL-PURPOSE OUTPUT IN MODER REG
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7 );			//SETTING PIN  7 TO PUSH-PULL OUTPUT
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR7);		//SETTING PIN  7 TO LOW SPEED
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);					//SETTING PIN  7 TO PULLUP-PULLDOWN 
	
	// CONFIGURING USER BUTTON PA0 AS NPUT WITH PULL DOWN 
		GPIOA->MODER &= ~GPIO_MODER_MODER0;      
		GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR0;
		GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
		
		GPIOC->BSRR = GPIO_BSRR_BR_6;      //RESET PIN 6
	GPIOC->BSRR = GPIO_BSRR_BS_7; 		// SET PC7 HIGH
	
	uint32_t debouncer = 0;
	
while (1) {    
	
        debouncer = (debouncer << 1); // Always shift every loop iteration
if ((GPIOA->IDR & GPIO_IDR_0) == 0) { // If input signal is set/high
debouncer |= 0x01; // Set lowest bit of bit-vector

        // Debounce conditions
        if (debouncer == 0xFFFFFFFF) {
            // This code triggers repeatedly when the button is steady high!
             
            }
        else if (debouncer == 0x00000000) {
            // This code triggers repeatedly when the button is steady low!
         
        }
else if (debouncer == 0x7FFFFFFF){

GPIOC->ODR ^= (GPIO_ODR_6 | GPIO_ODR_7);    // Toggle the output state of both PC6 and PC7
              
}

}
}

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
