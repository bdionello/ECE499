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
#include <math.h>
#include <string.h>
#include "stm32l0538_discovery_epd.h"
#include "stm32l0538_discovery.h"
#include "images.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { VBAT_LOW, VBAT_OK, VBAT_HIGH, VSOL_LOW, VSOL_OK, IBAT_LOW, NOTHING, MAX_EVENT } Event ;
typedef enum { START, IDLE , CHARGE_M, CHARGE_T, CHARGE_F, MAX_STATE } State ;
typedef void (* Action ) ( void) ;
typedef struct {
	Action to_do ; // function pointer to current-state action
	State next_state ; // next-state enumerator
} Table_Cell ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pulse_max 630 // 640 is counter period
#define pulse_min 10
#define ADC_MAX 4095
#define VSOL_MAX 20.7
#define VBAT_MAX 15.0
#define VDDA 3.3

#define VSOL_OK_VOLTAGE 776
#define VBAT_OK_VOLTAGE 3447//12.5
#define VBAT_LOW_VOLTAGE 3240//11.75
#define VBAT_HIGH_VOLTAGE 4054//14.7
#define VOLTAGE_TOPPING 3916
#define VOLTAGE_FLOAT 3723
#define IBAT_LOW_CURRENT 81

#define VSOL_SCALE 6.446414182
#define VBAT_SCALE 3.626107977
#define ISOL_SCALE 2.014504432
#define IBAT_SCALE 4.029008864
#define ILOAD_SCALE 8.058017728

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

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
char aTxBuffer[100];
//uint8_t aHeaderBuffer[] = "Event,State,Duty_Cycle,Voltage_Solar,Current_Solar,Voltage_Batter,Current_Battery\n";
uint8_t adc_conv_complt_flag;
uint32_t aResultDMA[5];
//uint16_t HEADERBUFFERSIZE = (sizeof(aTxBuffer)/sizeof(*aTxBuffer)) - 1;
uint16_t TXBUFFERSIZE = (sizeof(aTxBuffer)/sizeof(*aTxBuffer)) - 1;
uint16_t v_sol, v_bat, i_sol, i_bat, i_load;
uint8_t loadState = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static int PWM_DC_Step(int dir, int size);
static int inc_Con(void);
static void do_nothing( void );
static void pwm_on( void );
static void pwm_off( void );
static void charge_t ( void );
static void charge_f ( void );
static void update_inputs( void );
static Event update_events( State current_s );
static void load_on ( void );
static void load_off ( void );
static void refreshDisplay(void);
static void logger(State current_state, Event current_event);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Table_Cell state_table [ MAX_STATE ][ MAX_EVENT ] = {
	/*[0] VBAT_LOW            [1] VBAT_OK           [2] VBAT_HIGH           [3] VSOL_LOW            [4] VSOL_OK            5] IBAT_LOW             [6] NOTHING <--EVENTS |                  STATES */
	{ { do_nothing , START }, {  load_on , IDLE }, { do_nothing , START }, { do_nothing , START }, { pwm_on , CHARGE_M }, { do_nothing , START }, { do_nothing , START }  } ,             // START
	{ { load_off , START }, { do_nothing , IDLE }, { do_nothing , IDLE }, { do_nothing , IDLE }, { pwm_on , CHARGE_M }, { do_nothing , IDLE } , { do_nothing , IDLE } },                   // IDLE
	{ { load_off , CHARGE_M }, { load_on , CHARGE_M }, { do_nothing , CHARGE_T }, { pwm_off , IDLE }, { do_nothing , CHARGE_M }, { do_nothing , CHARGE_M} , { do_nothing , CHARGE_M } }, // CHARGE_M
	{ { load_off , CHARGE_M }, { do_nothing , CHARGE_T  }, { do_nothing , CHARGE_T }, { pwm_off , IDLE }, { do_nothing , CHARGE_T }, { do_nothing , CHARGE_F }, { do_nothing , CHARGE_T } } ,// CHARGE_T
	{ { load_off , CHARGE_M }, { do_nothing , CHARGE_F }, { do_nothing , CHARGE_F }, { pwm_off , IDLE }, { do_nothing , CHARGE_F }, { do_nothing , CHARGE_F } , { do_nothing , CHARGE_F } } // CHARGE_F
	};
const char* event_names[] = { "VBAT_LOW", "VBAT_OK", "VBAT_HIGH", "VSOL_LOW", "VSOL_OK", "IBAT_LOW", "NOTHING"};
const char* state_names[] = { "START", "IDLE" , "CHARGE_M", "CHARGE_T", "CHARGE_F", "MAX_STATE" };
/*
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/* State machine variables*/
    Table_Cell state_cell;
    Event current_event, last_event;
    State current_state, last_state;
    current_state = START ; // initial state

	int numWritten;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  MX_TIM21_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  BSP_EPD_Init();
  refreshDisplay();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {

		current_event = update_events(current_state);

		if(current_event != last_event){
			logger(current_state, current_event);
			last_event = current_event;
		}

        state_cell = state_table [ current_state ][ current_event ];
        state_cell.to_do() ; // execute the appropriate action
        current_state = state_cell.next_state ; // transition to the new state

		switch(current_state){
			case CHARGE_M:
				int result = inc_Con();
				break;
			case CHARGE_T:
				charge_t();
				break;
			case CHARGE_F:
				charge_f();
				break;
			default:
				break;
		}
		refreshDisplay();
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
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
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
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
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8192;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 640;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 320;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */
  /* USER CODE END TIM21_Init 2 */
  HAL_TIM_MspPostInit(&htim21);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */


  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ePD1_RESET_Pin|ePD1_PWR_ENn_Pin|ePD1_D_C_Pin|GPIO_LOAD_CTL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ePD1_CS_GPIO_Port, ePD1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ePD1_RESET_Pin ePD1_PWR_ENn_Pin ePD1_D_C_Pin GPIO_LOAD_CTL_Pin */
  GPIO_InitStruct.Pin = ePD1_RESET_Pin|ePD1_PWR_ENn_Pin|ePD1_D_C_Pin|GPIO_LOAD_CTL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ePD1_BUSY_Pin */
  GPIO_InitStruct.Pin = ePD1_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ePD1_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ePD1_CS_Pin */
  GPIO_InitStruct.Pin = ePD1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ePD1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MFX_I2C_SCL_Pin MFX_I2C_SDA_Pin */
  GPIO_InitStruct.Pin = MFX_I2C_SCL_Pin|MFX_I2C_SDA_Pin;
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
static int inc_Con(void) {
	char exit = 0;
	char status = 0;
	char state = 0;
	int dV = 0;
	int dI = 0;
	float dIdV = 0;
	float IV = 0;
	update_inputs();
	uint16_t  oldVolt = v_sol;
	uint16_t  oldCurrent = i_sol;
	HAL_Delay(1000);
	update_inputs();
	uint16_t voltage = v_sol;
	uint16_t current = i_sol;
	dV = voltage - oldVolt;
	dI = current - oldCurrent;
	dIdV = dI/dV;
	IV = -current / voltage;
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

/* Refreshes screen
 * @Uses a temp char array to hold value of current and voltage scaled to milli.
 * @Displays a sun or moon depending on if solar voltage is higher or lower than battery voltage
 * @Refreshes that bad boy  */
void refreshDisplay(void) {
	//v_sol = (rand() % (4095 - 0 + 1)) + 0;
	int tempVal = v_sol * VSOL_SCALE;
	char tempStr[6]; //Should be good up to 99999 mA/mV
	sprintf(tempStr, "%i", tempVal);
	BSP_EPD_Clear(EPD_COLOR_WHITE);
	BSP_EPD_SetFont(&Font12);
	BSP_EPD_DisplayStringAt(1, 0, "Panel Voltage is: ", LEFT_MODE);
	BSP_EPD_DisplayStringAt(130, 0, tempStr, LEFT_MODE);
	BSP_EPD_DisplayStringAt(170, 0, "mV", LEFT_MODE);
	//test = 1337; //test variable, replace with I_sol and scale to mA
	//i_sol = (rand() % (4095 - 0 + 1)) + 0;
	tempVal = i_sol * ISOL_SCALE;
	sprintf(tempStr, "%i", tempVal);
	BSP_EPD_DisplayStringAt(1, 3, "Panel Current is: ", LEFT_MODE);
	BSP_EPD_DisplayStringAt(130, 3, tempStr, LEFT_MODE);
	BSP_EPD_DisplayStringAt(170, 3, "mA", LEFT_MODE);
	//test = 12312; //test variable, replace with v_bat and scale to mV
	//v_bat = (rand() % (4095 - 0 + 1)) + 0;
	tempVal = v_bat * VBAT_SCALE;
	sprintf(tempStr, "%i", tempVal);
	BSP_EPD_DisplayStringAt(1, 6, "Battery Voltage is: ", LEFT_MODE);
	BSP_EPD_DisplayStringAt(140, 6, tempStr, LEFT_MODE);
	BSP_EPD_DisplayStringAt(180, 6, "mV", LEFT_MODE);
	//test = 1613; //test variable, replace with I_bat and scale to mA
	//i_bat = (rand() % (4095 - 0 + 1)) + 0;
	tempVal = i_bat * IBAT_SCALE;
	sprintf(tempStr, "%i", tempVal);
	BSP_EPD_DisplayStringAt(1, 9, "Battery Current is: ", LEFT_MODE);
	BSP_EPD_DisplayStringAt(140, 9, tempStr, LEFT_MODE);
	BSP_EPD_DisplayStringAt(180, 9, "mA", LEFT_MODE);
	if (v_sol >= VSOL_OK_VOLTAGE) {
		BSP_EPD_DisplayStringAt(1, 12, "Solar Present", LEFT_MODE);
		BSP_EPD_DrawImage(200, 0, 30, 45, Sun);
	} else {
		BSP_EPD_DisplayStringAt(1, 12, "Solar Hidden", LEFT_MODE);
		BSP_EPD_DrawImage(200, 0, 30, 45, Moon);
	}
	if (v_bat < VBAT_LOW_VOLTAGE) {
		BSP_EPD_DisplayStringAt(95, 12, "|Batt Low", LEFT_MODE);
		BSP_EPD_DrawImage(200, 6, 30, 45, batLow);
	} else if (v_bat >= VBAT_OK_VOLTAGE && v_bat <= VBAT_HIGH_VOLTAGE) {
		BSP_EPD_DisplayStringAt(95, 12, "|Batt Charge", LEFT_MODE);
		BSP_EPD_DrawImage(200, 6, 30, 45, batCharge);
	} else {
		BSP_EPD_DisplayStringAt(95, 12, "|Batt Full", LEFT_MODE);
		BSP_EPD_DrawImage(200, 6, 30, 45, batFull);
	}
	if (loadState == 0){
		BSP_EPD_DisplayStringAt(185, 12, "|Load Off", LEFT_MODE);
	}
	else {
		BSP_EPD_DisplayStringAt(185, 12, "|Load On", LEFT_MODE);
	}
	BSP_EPD_RefreshDisplay();
}

/*    [0] VBAT_LOW    [1] VBAT_OK   [2] VBAT_HIGH  [3] VSOL_LOW     [4] VSOL_OK    [5] IBAT_LOW     <--EVENTS | STATES */
static Event update_events( State current_s ){ // START, IDLE , CHARGE_M, CHARGE_T, CHARGE_F, MAX_STATE
	update_inputs();
	Event return_event = NOTHING;
	switch(current_s) {
		case START:
			if(v_bat >= VBAT_OK_VOLTAGE){
				return_event=  VBAT_OK;
			}
			else if(v_sol >= VSOL_OK_VOLTAGE){
				return_event = VSOL_OK;
			}
			break;
		case IDLE:
			if(v_bat < VBAT_LOW_VOLTAGE){
				return_event = VBAT_LOW;
			}
			else if(v_sol >= VSOL_OK_VOLTAGE){
				return_event = VSOL_OK;
			}
			break;
		case CHARGE_M:
			if(v_bat >= VBAT_HIGH_VOLTAGE){
				return_event = VBAT_HIGH;
			}
			else if(v_sol < VSOL_OK_VOLTAGE){
				return_event = VSOL_LOW;
			}
			else if(v_bat >= VBAT_OK_VOLTAGE){
				return_event = VBAT_OK;
			}
			else if(v_bat < VBAT_LOW_VOLTAGE){
				return_event = VBAT_LOW;
			}
			break;
		case CHARGE_T:
			if(i_bat < IBAT_LOW_CURRENT){
				return_event = IBAT_LOW;
			}
			else if(v_sol < VSOL_OK_VOLTAGE){
				return_event = VSOL_LOW;
			}
			else if(v_bat < VBAT_LOW_VOLTAGE){
				return_event = VBAT_LOW;
			}
			break;
		case CHARGE_F:
			if(v_bat < VBAT_LOW_VOLTAGE){
				return_event = VBAT_LOW;
			}
			else if(v_sol < VSOL_OK_VOLTAGE){
				return_event = VSOL_LOW;
			}
			break;
		default:
			return_event = NOTHING; // maybe add a fault state
			break;
	}
	return return_event;
}
void pwm_on( void ){
    // Turn on PWM
	if (HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}
}
void pwm_off( void ){
    // Turn on PWM
	if (HAL_TIM_PWM_Stop(&htim21, TIM_CHANNEL_1) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}
}
void load_on ( void ){
    // Turn load on GPIO PIN
	HAL_GPIO_WritePin(GPIOB, GPIO_LOAD_CTL_Pin, GPIO_PIN_SET);
	loadState = 1;
}
void load_off ( void ){
    // Turn load off GPIO PIN
	HAL_GPIO_WritePin(GPIOB, GPIO_LOAD_CTL_Pin, GPIO_PIN_RESET);
	loadState = 0;
}
void do_nothing( void ){
    printf("do_nothing\n");
}
void charge_t ( void ){
	float error;
	float step;
	char status;
	update_inputs();
	error = VOLTAGE_TOPPING - v_bat;
	step = abs(round(error));
	if(step>64){
		step = 64;
	}
	if(error > 0){
		status = PWM_DC_Step(1, step);
	}
	else if (error < 0){
		status = PWM_DC_Step(0, step);
	}
}
void charge_f ( void ){
	float error;
	float step;
	char status;
	update_inputs();
	error = VOLTAGE_FLOAT - v_bat;
	step = abs(round(error));
	if(step>64){
		step = 64;
	}
	if(error > 0){
		status = PWM_DC_Step(1, step);
	}
	else if (error < 0){
		status = PWM_DC_Step(0, step);
	}
}
void update_inputs( void ){
	uint8_t sample_factor = 8;
	uint16_t NSAMPLES = 1<<sample_factor;
	uint32_t sum0 = 0;
	uint32_t sum1 = 0;
	uint32_t sum2 = 0;
	uint32_t sum3 = 0;
	uint32_t sum4 = 0;
	HAL_ADC_Start_DMA(&hadc, aResultDMA, 5);
	for(uint16_t i = 0; i < NSAMPLES; i++){
		adc_conv_complt_flag = 1;
		while(adc_conv_complt_flag){}; // wait for ADC conversion to complete
		sum0 += aResultDMA[0];
		sum1 += aResultDMA[1];
		sum2 += aResultDMA[2];
		sum3 += aResultDMA[3];
		sum4 += aResultDMA[4];
	}
	HAL_ADC_Stop_DMA(&hadc);
    // update inputs from ADC values
	v_sol = (sum0 >> sample_factor);
	i_sol = (sum1 >> sample_factor);
	v_bat = (sum2 >> sample_factor);
	i_bat = (sum3 >> sample_factor);
	i_load = (sum4 >> sample_factor);
}

static void logger(State current_state, Event current_event){
	memset(aTxBuffer,0,strlen(aTxBuffer));
	sprintf(aTxBuffer, "%s,%s,%lu,%u,%u,%u,%u,%u\n", state_names[current_state], event_names[current_event], TIM21->CCR1, v_sol, i_sol, v_bat, i_bat, i_load);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)aTxBuffer, strlen(aTxBuffer));
}

void USARTx_IRQHandler(void){
	HAL_UART_IRQHandler(&huart1);
	return;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	adc_conv_complt_flag = 0;
	return;
}
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
