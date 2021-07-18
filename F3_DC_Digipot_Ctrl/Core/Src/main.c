/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "Lcd/stm32_adafruit_lcd.h"
#include "Digipot/MCP4261.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float Tcyc;
	float D;
	float Vnorm;
	float Tacc;
	float Tnorm;
	float v1;
	float t1;
	float t2;
	float t3;
	float a1;
	float a2;
	float a3;
	float angle;
	float velocity;
	uint8_t running;
	uint8_t stopping;
	uint8_t start_rq;
	uint32_t start_rq_cycles;
	uint32_t encoder[10] ;
} MOT_Control_Params;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_TIMEOUT       100
#define ADC_VREF          3.3     // V
#define MOT_ACCEL         6.28    // rad/s^2
#define MOT_MAX_CYC_TIME  10.0    // s
#define MOT_MAX_DPT       1.0     // 0..1 (0..100%)
#define MOT_ENCODER_360   2.92    // V
#define MOT_CTRL_Kp       2.0     // proportional coefficient
#define PI                3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ADCx conversion results
uint32_t ADC1_VALUE = 0;
uint32_t ADC2_VALUE = 0;
uint32_t ADC3_VALUE = 0;
// Control
MOT_Control_Params MOT;
// Time measurements
uint32_t LOOP_START_TIME = 0;
uint32_t LOOP_EXEC_TIME = 0;
uint32_t LOOP_CYCLE_TIME = 0;
uint32_t MAIN_START_TIME = 0;
uint32_t MAIN_EXEC_TIME = 0;
uint32_t MAIN_CYCLE_TIME = 0;

// Display diagnostic strings
char DPY_adc1[24];
char DPY_par0[24];
char DPY_par1[24];
char DPY_par2[24];
char DPY_par3[24];
char DPY_par4[24];
char DPY_par5[24];
char DPY_par6[24];
char DPY_par7[24];
char DPY_par8[24];
char DPY_par9[24];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Main control loop
void CONTROL_Main_Loop() {
	// ! ATTENTION ! MAIN CONTROL CYCLE called by interrupt from TIM4
	HAL_StatusTypeDef halst;   // In case some diagnostic data needed

	// Profiling
	uint32_t timer_start = htim2.Instance->CNT;
	LOOP_CYCLE_TIME = htim2.Instance->CNT - LOOP_START_TIME;
	LOOP_START_TIME = timer_start;
	HAL_GPIO_WritePin(LOOP_EXEC_GPIO_Port, LOOP_EXEC_Pin, GPIO_PIN_SET);

	// *********************************************************************
	// *                           Read all inputs                         *
	// *********************************************************************
	// Read encoder feedback from ADC1
	halst = HAL_ADC_Start(&hadc1);
	halst = HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT);
	ADC1_VALUE = HAL_ADC_GetValue(&hadc1);
	// Read speed setting value from ADC2 (connected to potentiometer)
	halst = HAL_ADC_Start(&hadc2);
	halst = HAL_ADC_PollForConversion(&hadc2, ADC_TIMEOUT);
	ADC2_VALUE = HAL_ADC_GetValue(&hadc2);
	// Read diagnostic value from ADC3 (connected to potentiometer in parallel to ADC2)
	halst = HAL_ADC_Start(&hadc3);
	halst = HAL_ADC_PollForConversion(&hadc3, ADC_TIMEOUT);
	ADC3_VALUE = HAL_ADC_GetValue(&hadc3);

	// *********************************************************************
	// *                         CONTROL LOGIC                             *
	// *********************************************************************

	// Current motor rotation angle
	float adc1_volt = (float) ADC1_VALUE * (ADC_VREF / 0xFFF);
	MOT.angle = PI * 2 * (adc1_volt / MOT_ENCODER_360);

	// Current speed
	uint16_t encoder_size = sizeof(MOT.encoder) / sizeof(typeof(MOT.encoder[0]));
	uint16_t iter_end = encoder_size - 1;
	float iter_sum = 0;
	for (uint16_t i = 1; i <= iter_end; i++) {
		MOT.encoder[i - 1] = MOT.encoder[i];
		iter_sum = iter_sum + MOT.encoder[i - 1];
	}
	MOT.encoder[iter_end] = MOT.angle;
	iter_sum = iter_sum + MOT.encoder[iter_end];
	MOT.velocity = iter_sum / encoder_size;


	// Read rotation cycle time value from potentiometer connected to ADC2
	MOT.Tcyc = MOT_MAX_CYC_TIME * ((float) ADC2_VALUE / 0xFFF);
	// Calculate speed profile parameters
	MOT.D = MOT.Tcyc * MOT.Tcyc - (PI * 8 / MOT_ACCEL);
	if (MOT.D >= 0) {
		MOT.Vnorm = MOT_ACCEL * (MOT.Tcyc - sqrt(MOT.D));
		MOT.Tacc = MOT.Vnorm / MOT_ACCEL;
		if (MOT.Tcyc >= (MOT.Tacc * 2)) {
			MOT.Tnorm = MOT.Tcyc - (MOT.Tacc * 2);
			MOT.v1 = MOT.Vnorm;
			MOT.t1 = MOT.Tacc;
			MOT.t2 = MOT.Tacc + MOT.Tnorm;
			MOT.t3 = MOT.Tcyc;
			MOT.a1 = PI * 2 * MOT.t1 / MOT.Tcyc;
			MOT.a2 = PI * 2 * MOT.t2 / MOT.Tcyc;
			MOT.a3 = PI * 2;
		}
	}

	// Blue button press detection
	if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
		if (MOT.running == 0) {
			MOT.start_rq_cycles++;
			if (MOT.start_rq_cycles > 250) {
				MOT.running = 1;
				MOT.stopping = 0;
			}
		} else {
			MOT.start_rq_cycles = 0;
		}
	} else {
		MOT.start_rq_cycles = 0;
	}

	// Regulator output calculation
	float regulator = 0;
	float Vref = 0;
	if (MOT.running > 0) {
		if ((MOT.angle >= 0) && (MOT.angle < MOT.a1) && (MOT.stopping == 0)) {
			Vref = MOT.v1 * MOT.angle / MOT.a1;
		}
		if ((MOT.angle >= MOT.a1) && (MOT.angle < MOT.a2)
				&& (MOT.stopping == 0)) {
			Vref = MOT.v1;
		}
		if ((MOT.angle >= MOT.a2) && (MOT.angle < MOT.a3)) {
			Vref = MOT.v1 * (MOT.a3 - MOT.angle) / (MOT.a3 - MOT.a2);
		} else if (MOT.stopping > 0) {
			MOT.running = 0;
			MOT.stopping = 0;
		}
		float Vdiff = Vref - MOT.velocity;
		regulator = Vdiff * MOT_CTRL_Kp;
	}

	// Regulator output clipping
	if (regulator > 1)
		regulator = 1;
	if (regulator < 0)
		regulator = 0;

	// Calculate output value of the regulator
	uint16_t regulator_output = regulator * MOT_MAX_DPT * 0xFF;

	// *********************************************************************
	// *                        Write all outputs                          *
	// *********************************************************************
	// Write output to the digital potentiometer
	// Black core is the floor level
	MCP4261_wiper_write(0, regulator_output);

	// Profiling
	uint32_t timer_stop = htim2.Instance->CNT;
	LOOP_EXEC_TIME = timer_stop - timer_start;
	HAL_GPIO_WritePin(LOOP_EXEC_GPIO_Port, LOOP_EXEC_Pin, GPIO_PIN_RESET);
}

// Display initialization
void DPY_Init() {
	BSP_LCD_Init();
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_Clear(LCD_COLOR_BLUE);
	//BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	//BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	sprintf(DPY_adc1, "-");
	sprintf(DPY_par0, "-");
	sprintf(DPY_par1, "-");
	sprintf(DPY_par2, "-");
	sprintf(DPY_par3, "-");
	sprintf(DPY_par4, "-");
	sprintf(DPY_par5, "-");
	sprintf(DPY_par6, "-");
	sprintf(DPY_par7, "-");
	sprintf(DPY_par8, "-");
	sprintf(DPY_par9, "-");
}

// Display update
void DPY_ScreenUpdate() {
	// Screen update
	//BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	//BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), 90);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(0, 0, (uint8_t*) DPY_adc1, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 15, (uint8_t*) DPY_par0, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 30, (uint8_t*) DPY_par1, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 45, (uint8_t*) DPY_par2, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 60, (uint8_t*) DPY_par3, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 75, (uint8_t*) DPY_par4, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 90, (uint8_t*) DPY_par5, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 105, (uint8_t*) DPY_par6, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 120, (uint8_t*) DPY_par7, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 135, (uint8_t*) DPY_par8, LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, 150, (uint8_t*) DPY_par9, LEFT_MODE);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI3_Init();
	MX_SPI2_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_ADC2_Init();
	MX_TIM4_Init();
	MX_ADC3_Init();
	/* USER CODE BEGIN 2 */
	// Calibrate The ADC On Power-Up For Better Accuracy
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	// Display initialization
	MCP4261_Init(&hspi2);
	DPY_Init();
	// Current speed init array
	uint16_t iter_end = sizeof(MOT.encoder) / sizeof(typeof(MOT.encoder[0])) - 1;
	for (uint16_t i = 0; i <= iter_end; i++) {
		MOT.encoder[i] = 0;
	}
	// Start timer TIM2 for microseconds counting
	HAL_TIM_Base_Start_IT(&htim2);
	// Start timer TIM3 for LD2 blinking
	HAL_TIM_Base_Start_IT(&htim3);
	// Start timer MIN4 for main control loop execution
	HAL_TIM_Base_Start_IT(&htim4);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// Profiling
		uint32_t timer_start = htim2.Instance->CNT;
		MAIN_CYCLE_TIME = htim2.Instance->CNT - MAIN_START_TIME;
		MAIN_START_TIME = timer_start;

		// ****************************
		// * Diagnostic information   *
		// ****************************
		//
		// ADC values
		// 12-bit conversion 000h...FFFh
		// 3.3V reference voltage
		float adc1_volt = (float) ADC1_VALUE * (ADC_VREF / 0xFFF);
		float adc2_volt = (float) ADC2_VALUE * (ADC_VREF / 0xFFF);
		float adc3_volt = (float) ADC3_VALUE * (ADC_VREF / 0xFFF);
		//
		// Digital potentiometer values
		uint16_t st = MCP4261_status_read();
		uint16_t tc = MCP4261_tcon_read();
		uint16_t wp = MCP4261_wiper_read(0);
		sprintf(DPY_adc1, "AD1 ENC %3X %.2fV   ", ADC1_VALUE, adc1_volt);
		sprintf(DPY_par0, "AD2 POT %3X %.2fV   ", ADC2_VALUE, adc2_volt);
		sprintf(DPY_par1, "AD3 DIA %3X %.2fV   ", ADC3_VALUE, adc3_volt);
		sprintf(DPY_par2, "DP: %4X %4X %3X     ", st, tc, wp);
		sprintf(DPY_par3, "   (1) (2) (3)");
		sprintf(DPY_par4, "T> %.1f %.1f %.1f      ", MOT.t1, MOT.t2, MOT.t3);
		sprintf(DPY_par5, "A> %.1f %.1f %.1f      ", MOT.a1, MOT.a2, MOT.a3);
		sprintf(DPY_par6, "D=%.1f Vn= %.1f  ", MOT.D, MOT.Vnorm);
		sprintf(DPY_par7, "RUN=%d A=%.1f V=%.1f   ", MOT.running, MOT.angle, MOT.velocity);
		sprintf(DPY_par8, "Ctrl: %4d/%4d us    ", LOOP_CYCLE_TIME,
				LOOP_EXEC_TIME);
		sprintf(DPY_par9, "Main: %4d/%4d ms    ", MAIN_CYCLE_TIME / 1000,
				MAIN_EXEC_TIME / 1000);
		DPY_ScreenUpdate();

		// Profiling
		MAIN_EXEC_TIME = htim2.Instance->CNT - MAIN_START_TIME;

		HAL_Delay(100);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC34 | RCC_PERIPHCLK_TIM2
			| RCC_PERIPHCLK_TIM34;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
	PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
	PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 71;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 7199;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 9999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 71;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LOOP_EXEC_Pin | GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | DPT_WP_Pin | GPIO_PIN_15,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin | DPT_SHDN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LOOP_EXEC_Pin PC11 */
	GPIO_InitStruct.Pin = LOOP_EXEC_Pin | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin DPT_WP_Pin PA15 */
	GPIO_InitStruct.Pin = LD2_Pin | DPT_WP_Pin | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI2_CS_Pin DPT_SHDN_Pin */
	GPIO_InitStruct.Pin = SPI2_CS_Pin | DPT_SHDN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Callback from roll over timers
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {

	}
	if (htim == &htim3) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	if (htim == &htim4) {
		CONTROL_Main_Loop();
	}
}

/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  AdcHandle : AdcHandle handle
 * @note   This example shows a simple way to report end of conversion
 *         and get conversion result. You can add your own implementation.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc == &hadc1) {

	}
	if (hadc == &hadc2) {

	}
	if (hadc == &hadc3) {

	}
}

/**
 * @brief  Conversion DMA half-transfer callback in non blocking mode
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {

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
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_SetBackColor(LCD_COLOR_RED);
		BSP_LCD_SetFont(&Font24);
		BSP_LCD_DisplayStringAt(0, 0, (uint8_t*) "! ERROR !", LEFT_MODE);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
