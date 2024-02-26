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
void setUpSlaveTransaction(uint8_t slaveAddr, uint8_t nBytes, uint8_t wOrR);
uint32_t readReg(uint8_t slaveAddr, uint32_t writeAddr);
void writeReg(uint8_t slaveAddr, uint32_t writeData);

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
  // Enable peripheral clock to PB and PC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// set PB11 to AF mode, 10
	GPIOB->MODER |= (1 << 23);
	GPIOB->MODER &= ~(1 << 22);
	// set PB13 to AF mode, 10
	GPIOB->MODER |= (1 << 27);
	GPIOB->MODER &= ~(1 << 26);
	// Set PB14 to output mode, 01
	GPIOB->MODER &= ~(1 << 29);
	GPIOB->MODER |= (1 << 28);
	// Set PC0 to output mode, 01
	GPIOC->MODER &= ~(1 << 1);
	GPIOC->MODER |= (1 << 0);
	
	// set PB11 AFRR to 0001: AF1
	GPIOB->AFR[1] &= ~(0xf << GPIO_AFRH_AFSEL11_Pos);
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos);
	// set PB13 AFRR to 0101: AF5
	GPIOB->AFR[1] &= ~(0xf << GPIO_AFRH_AFSEL13_Pos);
	GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos);

	// set PB11 and PB13 to output open-drain
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_11);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_13);
	// set PB14 and PC0 to push-pull output type
	GPIOB->OTYPER &= (~GPIO_OTYPER_OT_14);
	GPIOC->OTYPER &= (~GPIO_OTYPER_OT_0);
	
	// initialize the PB14 and PC0 to high.
	GPIOB->ODR |= (1 << 14);
	GPIOC->ODR |= (1 << 0);
}

void transmitOneChar(uint8_t ch) {
  while ((USART3->ISR & USART_ISR_TXE) == 0) {
	}
	USART3->TDR = ch;
}

void transmitCharArray (char *arr) {
  while (*arr != '\0') {
		transmitOneChar(*arr);
		arr++;
	}
}

void initUsart3(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
	// PC4 TX, PC5 RX
	// set pc4 to AF mode, 0x10
	GPIOC->MODER |= (1 << 9);
	GPIOC->MODER &= ~(1 << 8);
	// set pc5 to AF mode, 0x10
	GPIOC->MODER |= (1 << 11);
	GPIOC->MODER &= ~(1 << 10);
	
	// set PC4 AFRL to 0001: AF1
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFRL4_Pos);
	// set PC5 AFRL to 0001: AF1
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFRL5_Pos);
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	uint32_t fClk = HAL_RCC_GetHCLKFreq();
	
	// set baud rate
	uint32_t baudRate = 115200;
	uint32_t usartBRR = fClk / baudRate;
	USART3->BRR = usartBRR;

	// enable the transmitter and receiver hardware of USART3
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	
	// Enable USART peripheral.
	USART3->CR1 |= USART_CR1_UE;
}

char slaveNoRepMsg[20] = "Slave no response!";

void initI2C(void) {
  init_I2C_GPIO();
	// Enable the I2C2 peripheral in the RCC.
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	// Config I2C Bus Timing
	I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos);
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	I2C2->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);
	I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
	I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);
	
	// Enable I2C peripheral
	I2C2->CR1 |= I2C_CR1_PECEN;
}

void lab5_1(void) {
  uint32_t readData = readReg(0x6B, 0x0F);
	if (readData == 0xD4) {
		transmitCharArray("lab5_1: read data match");
	} else {
	  transmitCharArray("lab5_1: read data not match");
	}
}

void lab5_2(void) {
  
}

// wOrR = 0, write; wOrR = 1, read
void setUpSlaveTransaction(uint8_t slaveAddr, uint8_t nBytes, uint8_t wOrR) {
	// Set the slave address in the SADD[7:1] bit field
	I2C1->CR2 &= (~I2C_CR2_SADD_Msk);
  I2C1->CR2 |= (slaveAddr << 1);  //[7:1] is the slave address

	I2C1->CR2 &= (~I2C_CR2_NBYTES_Msk);
	// Set the number of data byte to be transmitted in the NBYTES[7:0] bit field.
	I2C1->CR2 |= (nBytes << I2C_CR2_NBYTES_Pos);

	I2C1->CR2 &= (~I2C_CR2_RD_WRN_Msk);
	// Configure the RD_WRN to indicate a read/write operation.
	I2C1->CR2 |= (wOrR << I2C_CR2_RD_WRN_Pos);

	// Do not set the AUTOEND bit, this lab requires software start/stop operation. 
	I2C1->CR2 &= (~I2C_CR2_AUTOEND_Msk);
	
	// Setting the START bit to begin the address frame.
	I2C1->CR2 |= I2C_CR2_START;
}


void writeReg(uint8_t slaveAddr, uint32_t writeData) {
  setUpSlaveTransaction(slaveAddr, 1, 0);
	// Wait until either of the TXIS or NACKF flags are set
	while ((I2C2->ISR & I2C_ISR_TXE) == 0 && (I2C2->ISR & I2C_ISR_NACKF) == 0) {
	}
	if (I2C2->ISR & I2C_ISR_NACKF) {
	  transmitCharArray(slaveNoRepMsg);
	}
	// Write the address of the “WHO_AM_I” register into the I2C transmit register
	I2C2->TXDR = writeData;
	
	// Wait until the TC (Transfer Complete) flag is set.
	while ((I2C2->ISR & I2C_ISR_TC) == 0) {
	}

	// Set the STOP bit in the CR2 register to release the I2C bus.
	I2C1->CR2 |= I2C_CR2_STOP;
}


uint32_t readReg(uint8_t slaveAddr, uint32_t writeAddr) {
	setUpSlaveTransaction(slaveAddr, 1, 0);
	// Wait until either of the TXIS or NACKF flags are set
	while ((I2C2->ISR & I2C_ISR_TXE) == 0 && (I2C2->ISR & I2C_ISR_NACKF) == 0) {
	}
	if (I2C2->ISR & I2C_ISR_NACKF) {
	  transmitCharArray(slaveNoRepMsg);
	}
	// Write the address of the “WHO_AM_I” register into the I2C transmit register
	I2C2->TXDR = writeAddr;
	// Wait until the TC (Transfer Complete) flag is set.
	while ((I2C2->ISR & I2C_ISR_TC) == 0) {
	}
	// start read operation
	setUpSlaveTransaction(slaveAddr, 1, 1);
	
	// Wait until either of the RXNE or NACKF flags are set
	while ((I2C2->ISR & I2C_ISR_RXNE) == 0 && (I2C2->ISR & I2C_ISR_NACKF) == 0) {
	}
	
	if (I2C2->ISR & I2C_ISR_NACKF) {
	  transmitCharArray(slaveNoRepMsg);
	}
	
	// Wait until the TC (Transfer Complete) flag is set.
	while ((I2C2->ISR & I2C_ISR_TC) == 0) {
	}
	
	uint32_t readData = I2C2->RXDR;
	// Set the STOP bit in the CR2 register to release the I2C bus.
	I2C1->CR2 |= I2C_CR2_STOP;
	return readData;
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
