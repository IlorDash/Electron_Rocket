/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MEASURMENTS_NUM 50
#define NUM_OF_FLASH_CELLS 2048
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

BMP280_HandleTypedef bmp280;

/* USER CODE BEGIN PV */
float pressure, temperature, humidity;

uint32_t data_to_flash;
uint8_t data_to_uart[4];
uint16_t size_to_uart;
float altitude_0 = 0;
float altitude_above_ground = 0;
float altitude_above_ground_prev = 0;
float altitude_t = 0;
float altitude_current = 0;
float altitude_to_flash = 0;
uint8_t flash_step = 4; // 4 bytes for measurment

char * status = "";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */


bool SaveFlashData(uint32_t dest, uint32_t *pSrc, uint16_t size) {
	while (HAL_FLASH_Unlock() != HAL_OK) {	}

	uint32_t *pDest = (uint32_t *)dest;
	uint32_t *pDestEnd = pDest + size;
	uint32_t pageToBeErase = (uint32_t)pDest & ~(FLASH_PAGE_SIZE - 1);
	while (pDest < pDestEnd) {
		if (FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE * 2) != HAL_OK) {
			break;
		}

		if (pageToBeErase == (uint32_t)pDest) {
			int cmpSize = (pDestEnd - pDest) * sizeof(uint32_t);
			if (cmpSize > FLASH_PAGE_SIZE) {
				cmpSize = FLASH_PAGE_SIZE;
			}
			bool pageIsEqual = memcmp(pDest, pSrc, cmpSize) == 0;
			if (pageIsEqual) {
				pDest += FLASH_PAGE_SIZE / sizeof(uint32_t);
				pSrc += FLASH_PAGE_SIZE / sizeof(uint32_t);
				pageToBeErase = (uint32_t)pDest & ~(FLASH_PAGE_SIZE - 1);
				continue;
			} else {
				SET_BIT(FLASH->PECR, FLASH_PECR_ERASE); /* Set the ERASE bit */
				SET_BIT(FLASH->PECR, FLASH_PECR_PROG);  /* Set PROG bit */

				*(__IO uint32_t *)(uint32_t)pageToBeErase = 0x00000000; /* Write 00000000h to the first word of the program page to erase */
				if (FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE * 2) != HAL_OK) {
					break;
				}
				CLEAR_BIT(FLASH->PECR, FLASH_PECR_PROG);
				CLEAR_BIT(FLASH->PECR, FLASH_PECR_ERASE);
				pageToBeErase = ((uint32_t)pDest & ~(FLASH_PAGE_SIZE - 1)) + FLASH_PAGE_SIZE;
			}
		}
		*(__IO uint32_t *)pDest = *pSrc;
		pDest++;
		pSrc++;
	}

	//timeout = SysTickCount;
	/*while (HAL_FLASH_Lock() != HAL_OK) {
		if (PeriodInRange(timeout, SYSTICK_mS(2))) {
			return false;
		}
	}*/
	return true;
}

uint32_t flash_read(uint32_t address) {
	return (*(__IO uint32_t*) address);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  int i = 0;
  GPIO_PinState currentButtonState;
  while(i < 50){
      currentButtonState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
       if (currentButtonState == GPIO_PIN_RESET) {
         break;
       }
      HAL_Delay(100);
      i++;
  }
  
  
      //MAIN CONDITION
    if (currentButtonState == GPIO_PIN_RESET) {
        status = "measurment";
      	bmp280_init_user_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		HAL_Delay(2000);
	}
	//bool bme280p = bmp280.id == BME280_CHIP_ID;

	// findind null altitude
	HAL_Delay(100);
	i = 0;
	while (i < MEASURMENTS_NUM) {
		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			HAL_Delay(2000);
		}
		altitude_t += calculate_altitude(temperature, pressure);
		i++;
		HAL_Delay(100);
	}
	altitude_0 = altitude_t / MEASURMENTS_NUM;
        altitude_above_ground_prev = altitude_0;
        i = 0;
        while (i < NUM_OF_FLASH_CELLS) {
		HAL_Delay(100);
		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			HAL_Delay(2000);
		}
		altitude_above_ground = calculate_altitude(temperature, pressure) - altitude_0;
                altitude_to_flash = (altitude_above_ground - altitude_above_ground_prev)*10;
                altitude_above_ground_prev = altitude_above_ground;
                data_to_flash = altitude_to_flash;
                //SaveFlashData(0x0800D000 + i, &data_to_flash, 1);
                i += sizeof(uint32_t);
	}
    }
    else {
      status = "transmiting to UART";
      i = 0;
      while (i < NUM_OF_FLASH_CELLS)
        
         //size_to_uart = sprintf((char *)data_to_uart,"%d\n", flash_read(0x0800D000 + i));
         size_to_uart = sprintf((char *)data_to_uart,"aa%d\n", i);
         HAL_UART_Transmit(&huart2, data_to_uart, size_to_uart, 100);
         i += sizeof(uint32_t);
     
    }


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
