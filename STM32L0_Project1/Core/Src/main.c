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
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pulse_max 630 // 640 is counter period
#define pulse_min 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim21;

TSC_HandleTypeDef htsc;

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TSC_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static int PWM_DC_Step(int dir, int size);
static int inc_Con(int *voltage, int *current);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t aResultDMA[4];
uint32_t ADC_COUNT = 0;
uint32_t ADC_DMA_HALF_COUNT = 0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* Converted value declaration */
	uint32_t ConvertedValue;
	/* Input voltage declaration */
	uint32_t InputVoltage;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	uint8_t ret = BSP_EPD_Init();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_TSC_Init();
	MX_ADC_Init();
	MX_TIM21_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	int mode = 0;
	int voltage = rand() % (10 + 1 - 1) + 1;  //Rand is used to simulate ADC reads. Replace with actual reads for production code.
	int current = rand() % (10 + 1 - 1) + 1;
	int *voltPtr = &voltage;
	int *currentPtr = &current;
	uint32_t adcSamples[4];
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_Delay(10);
		int result = inc_Con(voltPtr, currentPtr);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_12;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = ENABLE;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerFrequencyMode = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */
	HAL_ADC_Start_DMA(&hadc, aResultDMA, 4);
	/* USER CODE END ADC_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */
}
/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 65535;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */
	if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}
	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM21 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM21_Init(void) {

	/* USER CODE BEGIN TIM21_Init 0 */

	/* USER CODE END TIM21_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM21_Init 1 */

	/* USER CODE END TIM21_Init 1 */
	htim21.Instance = TIM21;
	htim21.Init.Prescaler = 0;
	htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim21.Init.Period = 640;
	htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim21) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim21) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 320;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM21_Init 2 */
	if (HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}

	/* USER CODE END TIM21_Init 2 */
	HAL_TIM_MspPostInit(&htim21);
}

/**
 * @brief TSC Initialization Function
 * @param None
 * @retval None
 */
static void MX_TSC_Init(void) {

	/* USER CODE BEGIN TSC_Init 0 */

	/* USER CODE END TSC_Init 0 */

	/* USER CODE BEGIN TSC_Init 1 */

	/* USER CODE END TSC_Init 1 */

	/** Configure the TSC peripheral
	 */
	htsc.Instance = TSC;
	htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
	htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
	htsc.Init.SpreadSpectrum = DISABLE;
	htsc.Init.SpreadSpectrumDeviation = 1;
	htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
	htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
	htsc.Init.MaxCountValue = TSC_MCV_8191;
	htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
	htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
	htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
	htsc.Init.MaxCountInterrupt = DISABLE;
	htsc.Init.ChannelIOs = TSC_GROUP1_IO3 | TSC_GROUP2_IO3 | TSC_GROUP3_IO2;
	htsc.Init.ShieldIOs = 0;
	htsc.Init.SamplingIOs =
	TSC_GROUP1_IO4 | TSC_GROUP2_IO4 | TSC_GROUP3_IO3;
	if (HAL_TSC_Init(&htsc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TSC_Init 2 */

	/* USER CODE END TSC_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	ePD1_RESET_Pin | ePD1_PWR_ENn_Pin | ePD1_D_C_Pin | GPIO_LOAD_CTL_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : ePD1_RESET_Pin ePD1_PWR_ENn_Pin ePD1_D_C_Pin GPIO_LOAD_CTL_Pin */
	GPIO_InitStruct.Pin = ePD1_RESET_Pin | ePD1_PWR_ENn_Pin | ePD1_D_C_Pin
			| GPIO_LOAD_CTL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : ePD1_BUSY_Pin */
	GPIO_InitStruct.Pin = ePD1_BUSY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ePD1_BUSY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MFX_I2C_SCL_Pin MFX_I2C_SDA_Pin */
	GPIO_InitStruct.Pin = MFX_I2C_SCL_Pin | MFX_I2C_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// function definitions funcdef
/* increments the duty cycle of the PWM in direction "dir" by value "size"
 * @ param dir = 1 for up, 0 for down
 * @ param size is increment value
 * @ returns 0 for success, 1 if limit is hit */
static int PWM_DC_Step(int dir, int size) {
	int status = 0;
	int pulse = TIM21->CCR1;
	if (dir) {
		pulse += size;
	} else {
		pulse -= size;
	}
	if (pulse >= pulse_max) {
		pulse = pulse_max;
		status = 1;
	}
	if (pulse <= pulse_min) {
		pulse = pulse_min;
		status = 1;
	}
	TIM21->CCR1 = pulse;
	return status;
}
/* MPPT Algorithm
 * @Takes a pointer to the voltage and current from the output
 * @of the solar panel and performs the incremental conductance algorithm.
 * @Modifies the PWM output based on dV and dI.  */
static int inc_Con(int *voltage, int *current) {
	char exit = 0;
	char status = 0;
	char state = 0;
	int dV = 0;
	int dI = 0;
	float dIdV = 0;
	float IV = 0;
	int oldVolt = *voltage;
	int oldCurrent = *current;
	HAL_Delay(1000);
	*voltage = rand() % (10 + 1 - 1) + 1; //Rand is used to simulate ADC reads. Replace with actual reads for production code.
	*current = rand() % (10 + 1 - 1) + 1;
	dV = *voltage - oldVolt;
	dI = *current - oldCurrent;
	dIdV = dI/dV;
	IV = -*current / *voltage;
	while (exit == 0){
	switch (state) {
	case 0:
		if (dV == 0) {
			state = 1;
			break;
		} else {
			state = 2;
			break;
		}
	case 1:
		if (dI == 0) {
			state = 0;
			status = 0;
			exit = 1;
			break;
		} else {
			state = 3;
			break;
		}
	case 2:
		if (dIdV == IV) {
			state = 0;
			status = 0;
			exit = 1;
			break;
		} else {
			state = 4;
			break;
		}
	case 3:
		if (dI > 0) {
			status = PWM_DC_Step(0, 1);
			state = 0;
			exit = 1;
			break;
		} else {
			status = PWM_DC_Step(1, 1);
			state = 0;
			exit = 1;
			break;
		}
	case 4:
		if (dIdV > IV) {
			status = PWM_DC_Step(1, 1);
			state = 0;
			exit = 1;
			break;
		} else {
			status = PWM_DC_Step(0, 1);
			state = 0;
			exit = 1;
			break;
		}
	default:
		status = 1;
		exit = 1;
		break;
	}
	}
	return (status);

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
