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

#define THRESHOLD 20
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
uint32_t readReg(uint8_t slaveAddr, uint8_t writeAddr, uint8_t nbytes);
void writeReg(uint8_t slaveAddr, uint8_t writeRegAddr, uint8_t writeData);
void lab5_1(void);
void transmitCharArray (char *arr);

void turnUint32ToBinaryStr(uint32_t x);
void lab5_2(void);
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
	lab5_1();
	lab5_2();
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
	// set PB15 to input mode(reset) 00
  GPIOB->MODER &= ~(1 << 31);
	GPIOB->MODER &= ~(1 << 30);
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
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	// Config I2C Bus Timing
	I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos);
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	I2C2->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);
	I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
	I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);
	// Enable I2C peripheral sing the PE bit in the CR1 register
	I2C2->CR1 |= I2C_CR1_PE;
}

void lab5_1(void) {
	initUsart3();
	initI2C();
	transmitCharArray("lab5_1: hello");
  uint32_t readData = readReg(0x69, 0xF, 1);
	if (readData == 0xD3) {
		transmitCharArray("read data match");
	} else {
	  transmitCharArray("read data not match");
	}
}

void initGyroscope(void) {
  uint8_t l3gd20_ctrl_reg1 = 0;
	// Enable the X and Y sensing axes in the CTRL_REG1 register.
	l3gd20_ctrl_reg1 |= (0x1 | (0x1 << 1));
	// Set the sensor into “normal or sleep mode” using the PD bit
	l3gd20_ctrl_reg1 |= (0x1 << 3);
	writeReg(0x69, 0x20, l3gd20_ctrl_reg1);
	uint8_t reg = readReg(0x69, 0x20, 1);
	while (reg != l3gd20_ctrl_reg1) {
		if (reg == l3gd20_ctrl_reg1) {
		  transmitCharArray("wrtie data match");
			break;
	  } else {
	    transmitCharArray("write data not match, reg:\n");
			turnUint32ToBinaryStr(reg);
		  writeReg(0x69, 0x20, l3gd20_ctrl_reg1);
	    reg = readReg(0x69, 0x20, 1);
	  }
	}
	
}

int16_t twosCompToDec(uint16_t two_compliment_val)
{
    // [0x0000; 0x7FFF] corresponds to [0; 32,767]
    // [0x8000; 0xFFFF] corresponds to [-32,768; -1]
    uint16_t sign_mask = 0x8000;
    // if positive
    if ( (two_compliment_val & sign_mask) == 0 ) {
        return two_compliment_val;
    //  if negative
    } else {
        // invert all bits, add one, and make negative
        return -(~two_compliment_val + 1);
    }
}

void initLED(void) {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
  // set the MODER, 01: General purpose output mode
	// init PC6 MODER
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER &= ~(1 << 13);
  // init PC7 MODER
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER &= ~(1 << 15);
	// init PC8 MODER
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 17);
	// init PC9 MODER
	GPIOC->MODER |= (1 << 18);
	GPIOC->MODER &= ~(1 << 19);
	// Set the pins to low speed in the OSPEEDR register
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 13));
	GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 15));
	GPIOC->OSPEEDR &= ~((1 << 16) | (1 << 17));
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 19));
	
		// Set LED to no pull-up/down resistors in the PUPDR register
	// 00: No pull-up, pull-down
	GPIOC->PUPDR &= ~((1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15));
	// set PC6-9 to 0
	GPIOC->ODR |= (1 << 6);
	GPIOC->ODR |= (1 << 7);
	GPIOC->ODR |= (1 << 8);
	GPIOC->ODR |= (1 << 9);
}

void lab5_2(void) {
	initLED();
  initGyroscope();
	// blue LED (PC7), channel 2, red LED PC6, Channel 1, green LED PC9, orange LED PC8
	while(1) {
	  uint32_t lrdx = readReg(0x69, 0x28, 1);
		transmitCharArray("lx:\n");
	  turnUint32ToBinaryStr(lrdx);
		uint32_t hrdx = readReg(0x69, 0x29, 1);
		transmitCharArray("hx:\n");
		turnUint32ToBinaryStr(hrdx);
		uint32_t rdx = ((hrdx << 8) | lrdx);
		transmitCharArray("rdx:");
		turnUint32ToBinaryStr(rdx);
		transmitCharArray("\n");
		int16_t xDec = twosCompToDec((uint16_t)rdx);
		if (xDec > 0 && xDec > THRESHOLD) {
			GPIOC->ODR |= (1 << 8);
			GPIOC->ODR &= ~(1 << 9);
		} else if (xDec < 0 && xDec < -THRESHOLD) {
		  GPIOC->ODR |= (1 << 9);
			GPIOC->ODR &= ~(1 << 8);
		} else {
		  GPIOC->ODR &= ~(1 << 9);
			GPIOC->ODR &= ~(1 << 8);
		}
		uint32_t lrdy = readReg(0x69, 0x2A, 1);
		transmitCharArray("ly:\n");
		turnUint32ToBinaryStr(lrdx);
		uint32_t hrdy = readReg(0x69, 0x2B, 1);
		transmitCharArray("hy:\n");
		turnUint32ToBinaryStr(hrdx);
		uint32_t rdy = ((hrdy << 8) | lrdy);
		transmitCharArray("rdy:");
		turnUint32ToBinaryStr(rdx);
		transmitCharArray("\n");
		
		int16_t yDec = twosCompToDec((uint16_t)rdy);
		if (yDec > 0 && yDec > THRESHOLD) {
			GPIOC->ODR |= (1 << 6);
			GPIOC->ODR &= ~(1 << 7);
		} else if (yDec < 0 && yDec < -THRESHOLD) {
		  GPIOC->ODR |= (1 << 7);
			GPIOC->ODR &= ~(1 << 6);
		} else {
		  GPIOC->ODR &= ~(1 << 7);
			GPIOC->ODR &= ~(1 << 6);
		}
		HAL_Delay(100);
	}
}

// wOrR = 0, write; wOrR = 1, read
void setUpSlaveTransaction(uint8_t slaveAddr, uint8_t nBytes, uint8_t wOrR) {
	// Set the slave address in the SADD[7:1] bit field
	I2C2->CR2 &= (~I2C_CR2_SADD_Msk);
	
  I2C2->CR2 |= (slaveAddr << 1);  //[7:1] is the slave address
	// I2C2->CR2 &= (~I2C_CR2_NBYTES);
	I2C2->CR2 &= ~((0x7F << I2C_CR2_NBYTES_Pos));
	// Set the number of data byte to be transmitted in the NBYTES[7:0] bit field.
	I2C2->CR2 |= (nBytes << I2C_CR2_NBYTES_Pos);
	I2C2->CR2 &= (~I2C_CR2_RD_WRN);
	// Configure the RD_WRN to indicate a read/write operation.
	if (wOrR == 1) {
		I2C2->CR2 |= (0x1 << I2C_CR2_RD_WRN_Pos);
	}

	// Do not set the AUTOEND bit, this lab requires software start/stop operation. 
	I2C2->CR2 &= (~I2C_CR2_AUTOEND_Msk);

	// Set the START bit to begin the address frame.
	I2C2->CR2 |= (I2C_CR2_START);
}


void writeReg(uint8_t slaveAddr, uint8_t writeRegAddr, uint8_t writeData) {
  setUpSlaveTransaction(slaveAddr, 1, 0);
	// Wait until either of the TXIS or NACKF flags are set
	transmitCharArray("Wait until either of the TXIS or NACKF flags are set.\n");
	while ((I2C2->ISR & I2C_ISR_TXIS) == 0 && (I2C2->ISR & I2C_ISR_NACKF) == 0) {
	}
	transmitCharArray("either of the TXIS or NACKF flags are set.\n");
	if (I2C2->ISR & I2C_ISR_NACKF) {
	  transmitCharArray(slaveNoRepMsg);
	}
	// Write the address of the register into the I2C transmit register
	I2C2->TXDR = writeRegAddr;
	transmitCharArray("Wait until the I2C_ISR_TXIS flag is set.");
	// Wait until the TC (Transfer Complete) flag is set.
	while ((I2C2->ISR & I2C_ISR_TXIS) == 0) { 
	}
  transmitCharArray("I2C_ISR_TXIS is set.");
	// Set the STOP bit in the CR2 register to release the I2C bus.
	
	I2C2->TXDR = writeData;
	transmitCharArray("Wait until the I2C_ISR_TXIS flag is set.");
	// Wait until the TC (Transfer Complete) flag is set.
	while ((I2C2->ISR & I2C_ISR_TXIS) == 0) { 
	}
  transmitCharArray("I2C_ISR_TXIS is set.");
  transmitCharArray("Wait until the TC (Transfer Complete) flag is set.");
	// Wait until the TC (Transfer Complete) flag is set.
	while ((I2C2->ISR & I2C_ISR_TC) == 0) {
	}
	transmitCharArray("TC (Transfer Complete) flag is set.");
	I2C2->CR2 |= I2C_CR2_STOP;
	transmitCharArray("Write Reg Stop CR2\n");
	turnUint32ToBinaryStr(I2C2->CR2);
}

void turnUint32ToBinaryStr(uint32_t x){
	char str[32];
	uint32_t i = 0;
	while (i < 32) {
		str[31-i] = '0' + ((x >> i) & 0x1);
	  i++;
	}
	transmitCharArray(str);
}

// nbytes <= 4
uint32_t readReg(uint8_t slaveAddr, uint8_t writeAddr, uint8_t nbytes) {
	setUpSlaveTransaction(slaveAddr, 1, 0);
  transmitCharArray("In readReg function.\n");
	//transmitCharArray("I2C2->CR2:");
	//turnUint32ToBinaryStr(I2C2->CR2);
  transmitCharArray("wait either the TXIS or NACKF flags are set.");
	while (1) {
		if ((I2C2->ISR & I2C_ISR_TXIS) != 0) {
			transmitCharArray("(I2C2->ISR & I2C_ISR_TXIS) != 0");
			break;
		}
		if ((I2C2->ISR & I2C_ISR_NACKF) != 0) {
			transmitCharArray("(I2C2->ISR & I2C_ISR_NACKF) != 0");
			break;
		}
	}
	
	transmitCharArray("the TXIS or NACKF flags are set.\n");
	if (I2C2->ISR & I2C_ISR_NACKF) {
	  transmitCharArray(slaveNoRepMsg);
	}
	//transmitCharArray("ISR:\n");
	//turnUint32ToBinaryStr(I2C2->ISR);
	
	// Write the address of the “WHO_AM_I” register into the I2C transmit register
	I2C2->TXDR = writeAddr;
	// Wait until the TC (Transfer Complete) flag is set.
	transmitCharArray("Wait the TC (Transfer Complete) flag is set.");
	while ((I2C2->ISR & I2C_ISR_TC) == 0) {
	}
	transmitCharArray("the TC (Transfer Complete) flag is set.");
	// start read operation
	setUpSlaveTransaction(slaveAddr, 1, nbytes);
	transmitCharArray("Wait until either of the RXNE or NACKF flags are set");
	// Wait until either of the RXNE or NACKF flags are set
	while ((I2C2->ISR & I2C_ISR_RXNE) == 0 && 0) {
	}

	uint32_t readData = 0;
	transmitCharArray("start read");
	while (nbytes > 0) {
	  //wait for RXNE or NACKF flag are set
	  while ((I2C2->ISR & I2C_ISR_RXNE) == 0 && (I2C2->ISR & I2C_ISR_NACKF) == 0) {
	  }
	  if (I2C2->ISR & I2C_ISR_NACKF) {
	    transmitCharArray(slaveNoRepMsg);
			break;
	  }
	  transmitCharArray("either of the RXNE flags are set");
	  uint8_t rd = I2C2->RXDR;
	  readData = ((readData << 8) | rd);
		nbytes--;
	}
	// Wait until the TC (Transfer Complete) flag is set.
	while ((I2C2->ISR & I2C_ISR_TC) == 0) {
	}
	// Set the STOP bit in the CR2 register to release the I2C bus.
	I2C2->CR2 |= I2C_CR2_STOP;
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
