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
void init_I2C_GPIO(void);
void initI2C(void);

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

/* USER CODE END 0 */
void init_I2C_GPIO(void) {
  // Enable peripheral clock to PC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	// set PB6 to AF mode, 0x10
	GPIOB->MODER |= (1 << 13);
	GPIOB->MODER &= ~(1 << 12);
	// set PB7 to AF mode, 0x10
	GPIOB->MODER |= (1 << 15);
	GPIOB->MODER &= ~(1 << 14);
	// set PB8 to AF mode, 0x10
	GPIOB->MODER |= (1 << 17);
	GPIOB->MODER &= ~(1 << 16);
	// set PB9 to AF mode, 0x10
	GPIOB->MODER |= (1 << 19);
	GPIOB->MODER &= ~(1 << 18);
	
	// set PB6 AFRL to 0001: AF1
	GPIOB->AFR[0] &= ~(0xf << GPIO_AFRL_AFRL6_Pos);
	GPIOB->AFR[0] |= (0x1 << GPIO_AFRL_AFRL6_Pos);
	// set PB7 AFRL to 0001: AF1
	GPIOB->AFR[0] &= ~(0xf << GPIO_AFRL_AFRL7_Pos);
	GPIOB->AFR[0] |= (0x1 << GPIO_AFRL_AFRL7_Pos);
	// set PB8 AFRL to 0001: AF1
	GPIOB->AFR[1] &= ~(0xf << GPIO_AFRH_AFSEL8_Pos);
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL8_Pos);
	// set PB9 AFRL to 0001: AF1
	GPIOB->AFR[1] &= ~(0xf << GPIO_AFRH_AFSEL9_Pos);
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL9_Pos);
	
	// set PB6, 7, 8, 9 to output open-drain
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_6);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_7);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_8);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_9);
}

void initI2C(void) {
  init_I2C_GPIO();
	
	// Config I2C Bus Timing
	I2C1->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos);
	I2C1->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);
	
	// Enable I2C peripheral
	I2C1->CR1 |= I2C_CR1_PECEN;
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
