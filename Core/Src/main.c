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
 * Notes:
 * RTC is not accurate. Runs at ~x1.07 speed.
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>

#include "fonts.h"
#include "ssd1306.h"
#include "bmp280.h"
//#include "ds18b20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN(A, B) A < B? A : B
#define MAX(A, B) A > B? A : B

#define TEMP_FAILED -404
#define TEMP_MIN -20.0
#define TEMP_MAX 80.0
#define ADC_CALIB_SLOPE -0.1169
#define ADC_CALIB_BIAS 374.24
#define CYCLE_PERIOD_MS (int)(5 * 60 * 1000) // Min cycle period is 2000
#define CYCLES_PER_DAY (int)(24 * 3600 * 1000 / CYCLE_PERIOD_MS)

#define EEPROM_ADDRESS 0xA0                         // 0b1010(A2)(A1)(A0)(R/W) = 0b1010000
#define EEPROM_SIZE_BYTES 32768                     // 65536 bytes for 512kbit version
#define EEPROM_PAGE_SIZE 64                         // Although AT24C512 page size is 128B but 64B is more portable
#define EEPROM_PAGE_TEMP_COUNT EEPROM_PAGE_SIZE / 2 // Temperature readings per page
#define EEPROM_BLOCK_ERASE_SIZE 64                  // Can be <=EEPORM_PAGE_SIZE
#define EEPROM_ERASED_VALUE 0xFF
#define EEPROM_ERASED_TEMP 0xFFFF

#define CYCLE_MEM_SIZE_BYTES 6 // Amount of data needed to store data one cycle
#define ADDR_TEMP 1024
//#define ADDR_BOOT_DATA 0          // Address of boot data
//#define ADDR_BOOT_DATA_LAST 1023  // Last address of boot data

#define FONT_SPACING 13

#define DEBOUNCE_PERIOD_MS 29
#define BTN_COUNT 5 // Number of buttons on the keypad
#define NO_PIN 0x00 // Dummy pin. Should be mapped to no pin in HAL library (check to be sure)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*
 typedef struct
 {
 // Do not change order
 uint8_t hr :5;
 uint8_t min :6;
 uint8_t sec :6;
 // padding 1 byte
 uint16_t ms :10;
 } Time;
 */

typedef struct
{
	uint32_t ms_change;  // Last time button state changed in ms
	uint8_t state :1;
	uint8_t prev_state :1;
	uint8_t do_debounce_checks :1; // If false skips global debounce check to preserve prev_state
} ButtonDebounced;

typedef struct
{
	uint16_t temp_1w;
	uint16_t temp_i2c;
	uint16_t temp_adc;
} Temps16;

BMP280_HandleTypedef bmp280;
volatile ButtonDebounced btn_arr[BTN_COUNT + 1] = {};  // All set to 0

static const uint16_t btn_mapping[] = { NO_PIN, BTN_UP_Pin, BTN_LEFT_Pin, BTN_CENTER_Pin, BTN_RIGHT_Pin, BTN_DOWN_Pin };

volatile uint32_t ms_ticks = 0; // Good for 49.7 days
static int cycle_count = 0;
static uint32_t next_cycle_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void Reset_Cycles(void);

void BMP280_Init(void);
void ADC_Init(void);
float DS18B20_Read_Temp(void);
float BMP280_Read_Temp(void);
float ADC_Read_Temp(void);

void EEPROM_Read(uint16_t mem_addr, uint8_t *p_data, uint16_t data_size);
void EEPROM_Write(uint16_t mem_addr, uint8_t *p_data, uint16_t data_size);
void EEPROM_Erase(void);

void Temp_Save_Single(uint16_t place, uint16_t temp);
void Temp_Save(uint16_t place, Temps16 temps);
uint16_t Temp_Read_Single(uint16_t place);
Temps16 Temp_Read(uint16_t place);
uint16_t Temp_Next_Free_Place(void);
void Temp_Send_UART(void);
uint16_t Temp_Enc(const float temp); // Encode temperature
float Temp_Dec(const uint16_t temp); // Decode temperature

void Print_Info(const char *info);
void Print_Time(uint16_t time, uint16_t off_v);
void Print_Simple(uint8_t line, const char *format, ...);
void Print_Temps(uint8_t line, Temps16 temps);
void Print_Booting_Info(void);
void Clear_Booting_Info(void);

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Delay_Us(uint16_t us);

uint8_t DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);

//Time Get_Time(void);
float Clamp(float val, float min, float max);

void Btn_Init(void);
uint8_t Btn_Index(const uint16_t btn_pin);
uint8_t Btn_Read(const uint16_t btn_pin);
uint8_t Btn_Hold(const uint16_t btn_pin, const uint16_t hold_time_ms);
uint8_t Btn_Falling(const uint16_t btn_pin);
uint8_t Btn_Rising(const uint16_t btn_pin);
void Button_Actions(void);

Temps16 Temp_Max_All_Time(void);
Temps16 Temp_Max_Last_Day(void);
Temps16 Temp_Min_All_Time(void);
Temps16 Temp_Min_Last_Day(void);

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
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_I2C2_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	Btn_Init();
	SSD1306_Init();
	Print_Booting_Info();
	HAL_TIM_Base_Start(&htim1);
	BMP280_Init();
	ADC_Init();

	// Send temperature data via UART
	if (Btn_Read(BTN_UP_Pin))
	{
		Print_Info("Sending\n");
		Temp_Send_UART();
		Print_Info("Reboot to run\n");

		while (1)
			; // Halt
	}

	HAL_Delay(1000);
	Clear_Booting_Info();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	Reset_Cycles();
	next_cycle_tick += CYCLE_PERIOD_MS;
	cycle_count = Temp_Next_Free_Place();
	Temps16 temps;
	while (1)
	{
		cycle_count++;
		Print_Simple(0, "RUNNING: %04u\n", cycle_count);

		// Check is there space left
		if (cycle_count >= EEPROM_SIZE_BYTES / sizeof(Temps16))
		{
			while (1)
			{
				Print_Simple(0, " -DONE-: %04u\n", cycle_count);
				Print_Temps(1, temps);
				Print_Simple(2, " Reboot to restart");
				Print_Simple(3, " Or Reboot holding");
				Print_Simple(4, "Btn Up to send data");
				Button_Actions();
				SSD1306_UpdateScreen();
				HAL_Delay(200);
			}
		}

		// Measuring and saving temperature to EEPROM
		temps.temp_1w = Temp_Enc(DS18B20_Read_Temp());
		temps.temp_i2c = Temp_Enc(BMP280_Read_Temp());
		temps.temp_adc = Temp_Enc(ADC_Read_Temp()); // First reading is off by 3deg

		Temp_Save(cycle_count - 1, temps);

		// Displaying current info
		Print_Temps(1, temps);
		SSD1306_DrawLine(0, FONT_SPACING * 2 - 2, 128, FONT_SPACING * 2 - 2, 1);
		SSD1306_UpdateScreen();

		// Idling & polling buttons for actions
		while (ms_ticks < next_cycle_tick)
			Button_Actions();

		next_cycle_tick += CYCLE_PERIOD_MS;

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
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 64 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0xffff - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 1000 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = (64 * BTN_CHECK_PERIOD_MS) - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

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
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_BUILTIN_Pin */
	GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DS18B20_Pin */
	GPIO_InitStruct.Pin = DS18B20_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_UP_Pin BTN_LEFT_Pin BTN_CENTER_Pin BTN_RIGHT_Pin
	 BTN_DOWN_Pin */
	GPIO_InitStruct.Pin = BTN_UP_Pin | BTN_LEFT_Pin | BTN_CENTER_Pin | BTN_RIGHT_Pin | BTN_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void Reset_Cycles(void)
{
	cycle_count = 0;
	next_cycle_tick = ms_ticks;
}

void BMP280_Init(void)
{
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c2;

	if (!bmp280_init(&bmp280, &bmp280.params))
	{
		Print_Info("I2C init FAILED\n");
		HAL_Delay(1000);
	}

//	bool bme280p = (bmp280.id == BME280_CHIP_ID);
//	Print_Simple(9, (char*)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
}

void ADC_Init(void)
{
	// Discards first bad reading
	ADC_Read_Temp();
}

float DS18B20_Read_Temp(void)
{
	// Start temperature sensing and wait until it's done
	int s1 = DS18B20_Start();
	DS18B20_Write(0xCC); // Skip ROM
	DS18B20_Write(0x44); // Convert t
	HAL_Delay(800); // Waiting for conversion to end

	// Read acquired temperature
	int s2 = DS18B20_Start();
	DS18B20_Write(0xCC); // Skip ROM
	DS18B20_Write(0xBE); // Read Scratch-pad
	uint8_t temp_byte1 = DS18B20_Read();
	uint8_t temp_byte2 = DS18B20_Read();

	if (!s1 || !s2)
	{
		Print_Info("1-Wire Failed");
		HAL_Delay(500);
		return TEMP_FAILED;
	}

	// Decode temperature
	uint16_t temp = (temp_byte2 << 8) | temp_byte1;
	float temperature = (float)temp / 16.0;

	return temperature;
}

float BMP280_Read_Temp(void)
{
	float pressure, temperature, humidity;

	if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity))
	{
		Print_Info("I2C read FAILED\n");
		HAL_Delay(500);
		temperature = TEMP_FAILED;
	}

	return temperature;
}

float ADC_Read_Temp(void)
{
	uint8_t readings_num = 10;
	float temperature;
	uint32_t adc_val = 0;

	HAL_ADC_Start(&hadc1);
	for (int i = 0; i < readings_num; i++)
	{
		HAL_ADC_PollForConversion(&hadc1, 10);
		adc_val += HAL_ADC_GetValue(&hadc1);
		HAL_Delay(1);
	}
	adc_val /= readings_num;

	temperature = ADC_CALIB_SLOPE * adc_val + ADC_CALIB_BIAS; // ADC value to temperature conversion

	return temperature;
}

void EEPROM_Read(uint16_t mem_addr, uint8_t *p_data, uint16_t data_size)
{
	while (HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, mem_addr, I2C_MEMADD_SIZE_16BIT, p_data, data_size, 1000) != HAL_OK)
		HAL_Delay(1);
}

void EEPROM_Write(uint16_t mem_addr, uint8_t *p_data, uint16_t data_size)
{
	// TODO maybe: Needs checking for page crossing; page = 64B. Or else use carefully
	while (HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDRESS, mem_addr, I2C_MEMADD_SIZE_16BIT, p_data, data_size, 1000) != HAL_OK)
		HAL_Delay(1);
}

void EEPROM_Erase(void)
{
	uint8_t erased[EEPROM_BLOCK_ERASE_SIZE];
	memset(erased, EEPROM_ERASED_VALUE, sizeof(erased));

	for (int i = 0; i < (int)(EEPROM_SIZE_BYTES / EEPROM_BLOCK_ERASE_SIZE); i++)
	{
		while (HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDRESS, i * EEPROM_BLOCK_ERASE_SIZE,
		I2C_MEMADD_SIZE_16BIT, erased, EEPROM_BLOCK_ERASE_SIZE, 1000) != HAL_OK)
			HAL_Delay(1);
	}
}

void Temp_Save_Single(uint16_t place, uint16_t temp)
{
	EEPROM_Write(ADDR_TEMP + 2 * place, (uint8_t*)&temp, 2); // Won't cross page boundary 64 % 2 = 0
}

void Temp_Save(uint16_t place, Temps16 temps)
{
	// Saved separately to prevent page boundary crossing
	Temp_Save_Single(3 * place + 0, temps.temp_1w);
	Temp_Save_Single(3 * place + 1, temps.temp_i2c);
	Temp_Save_Single(3 * place + 2, temps.temp_adc);
}

uint16_t Temp_Read_Single(uint16_t place)
{
	uint16_t temp_encoded;
	EEPROM_Read(ADDR_TEMP + sizeof(temp_encoded) * place, (uint8_t*)&temp_encoded, sizeof(temp_encoded));
	return temp_encoded;
}

Temps16 Temp_Read(uint16_t place)
{
	Temps16 temps;
	EEPROM_Read(ADDR_TEMP + sizeof(temps) * place, (uint8_t*)&temps, sizeof(temps));
	return temps;
}

uint16_t Temp_Next_Free_Place(void)
{
	int i;
	for (i = 0; i < EEPROM_SIZE_BYTES / sizeof(Temps16); i++)
	{
		Temps16 t = Temp_Read(i);
		if (t.temp_adc == EEPROM_ERASED_TEMP)
			break;
	}

	return i;
}

// Send temperature data via UART in CSV format
void Temp_Send_UART(void)
{
	const uint8_t num_of_temp_channels = 3;
	char text[16];
	uint8_t text_len;

	// Reading number
	const uint16_t readings_count = Temp_Next_Free_Place();
	for (int i = 0; i < readings_count; i++)
	{
		text_len = sprintf(text, "%d,", i);
		HAL_UART_Transmit(&huart1, (uint8_t*)text, text_len, 10);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 1, 10);

	// Temperatures in 3 rows
	for (int i = 0; i < num_of_temp_channels; i++)
	{
		for (int j = i; j < EEPROM_SIZE_BYTES; j += num_of_temp_channels)
		{
			uint16_t temp = Temp_Read_Single(j);
			if (temp == EEPROM_ERASED_TEMP)
				break;

			float temp_float = Temp_Dec(temp);
			text_len = sprintf(text, "%.3f,", temp_float);
			HAL_UART_Transmit(&huart1, (uint8_t*)text, text_len, 10);

			if (j % 100 == 0)
				HAL_Delay(10);
		}
		HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 1, 10);
	}
}

// Encode temperature
uint16_t Temp_Enc(const float temp)
{
	float temp_clamped = Clamp(temp, TEMP_MIN, TEMP_MAX);
	return (temp_clamped + 20) * 655;
}

// Decode temperature
float Temp_Dec(const uint16_t temp)
{
	return (float)temp / 655.0 - 20.0;
}

void Print_Info(const char *info)
{
	Print_Simple(0, info);
	SSD1306_UpdateScreen();
}

// For debugging
void Print_Time(uint16_t time, uint16_t off_v)
{
	char text[50];
	sprintf(text, "us: %x", time);
	SSD1306_GotoXY(0, off_v);
	SSD1306_Puts(text, &Font_7x10, 1);
}

// For debugging
void Print_Simple(uint8_t line, const char *format, ...)
{
	char text[128];
	int text_len;

	va_list args;
	va_start(args, format);

// Send via UART and display on screen
	text_len = vsprintf(text, format, args);
	HAL_UART_Transmit(&huart1, (uint8_t*)text, text_len, 1000);

// Clear line
	SSD1306_GotoXY(0, FONT_SPACING * line);
	SSD1306_Puts("                    ", &Font_7x10, 1);

// Clear \n character for display
	if (text[text_len - 1] == '\n')
		text[text_len - 1] = '\0';

	SSD1306_GotoXY(0, FONT_SPACING * line);
	SSD1306_Puts(text, &Font_7x10, 1);
	SSD1306_UpdateScreen(); // Could be removed for performance and updated only when needed

	va_end(args);
}

void Print_Temps(uint8_t line, Temps16 temps)
{
	Print_Simple(line, "%0.2f %0.2f %0.2f\n", Temp_Dec(temps.temp_1w), Temp_Dec(temps.temp_i2c), Temp_Dec(temps.temp_adc));
}

void Print_Booting_Info(void)
{
	Print_Info("BOOTING\n");
	Print_Simple(2, " Reboot holding\n");
	Print_Simple(3, "  Button UP to\n");
	Print_Simple(4, "send data via UART\n");
	SSD1306_UpdateScreen();
}

void Clear_Booting_Info(void)
{
	Print_Info("");
	Print_Simple(2, "");
	Print_Simple(3, "");
	Print_Simple(4, "");
	SSD1306_UpdateScreen();
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Delay_Us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0); // Set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
		; // Wait for the counter to reach the us input in the parameter
}

uint8_t DS18B20_Start(void)
{
	uint8_t presence = 0;

	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 0);
	Delay_Us(480);

	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);
	Delay_Us(100); // min: 30us max ~240us

	if (!(HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin)))
		presence = 1;
	else
		presence = 0;

	Delay_Us(380); // Min total delay 480 us

	return presence;
}

void DS18B20_Write(uint8_t data)
{
	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);

	for (int i = 0; i < 8; i++)
	{
		if ((data & (1 << i)) != 0)
		{
			// Write 1
			Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);
			HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 0);
			Delay_Us(2); // Mininum 1us to pull low
			Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);
			Delay_Us(60); // DS18B20 samples state after 15-60us from falling edge
		}
		else
		{
			// Write 0
			Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);
			HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 0);
			Delay_Us(60); // DS18B20 samples state after 15-60us from falling edge
			Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);
			Delay_Us(1);
		}
	}
}

uint8_t DS18B20_Read(void)
{
	uint8_t value = 0;
	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);

	for (int i = 0; i < 8; i++)
	{
		Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);

		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 0);
		Delay_Us(2);
		Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);
		if (HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin))  // If the pin is HIGH
			value |= 1 << i;  // read = 1

		Delay_Us(60);
	}

	return value;
}

/*
 Time Get_Time(void)
 {
 Time t;
 t.ms = ms_ticks % 1000;
 t.sec = (ms_ticks / 1000) % 60;
 t.min = (ms_ticks / 1000 / 60) % 60;
 t.hr = (ms_ticks / 1000 / 60 / 60) % 24;

 return t;
 }
 */

float Clamp(float val, float min, float max)
{
	val = val < min ? min : val;
	return val > max ? max : val;
}

void HAL_IncTick(void)
{
	ms_ticks++;
	uwTick += uwTickFreq;  // DO NOT MODIFY THIS LINE
}

// Starts button debouncing routine
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t ind = Btn_Index(GPIO_Pin);
	if (ind != 0)
	{
		btn_arr[ind].ms_change = ms_ticks;
		btn_arr[ind].do_debounce_checks = true;
		HAL_TIM_Base_Start_IT(&htim4);  // Burst check for bouncing
	}
}

// Handles buttons debouncing
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t keep_running;

	keep_running = false;
	for (int i = 1; i < sizeof(btn_mapping); i++)
	{
		if (btn_arr[i].do_debounce_checks)
		{
			keep_running = true;
			if (ms_ticks - btn_arr[i].ms_change >= DEBOUNCE_PERIOD_MS + 1)
			{
				btn_arr[i].prev_state = btn_arr[i].state;
				btn_arr[i].state = !HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, btn_mapping[i]); // Invert because pull-up
				btn_arr[i].do_debounce_checks = false;  // To preserve prev_state
			}
		}
	}

// Stop timer/debouncing button because no new input is received
	if (!keep_running)
		HAL_TIM_Base_Stop_IT(&htim4);
}

// Get initial button states
void Btn_Init(void)
{
	for (int i = 1; i < BTN_COUNT + 1; i++)
	{
		btn_arr[i].ms_change = ms_ticks;
		btn_arr[i].do_debounce_checks = true;
	}
	HAL_TIM_Base_Start_IT(&htim4);  // Burst check for bouncing
}

uint8_t Btn_Index(const uint16_t btn_pin)
{
	for (int i = 1; i < sizeof(btn_mapping); i++)
		if (btn_mapping[i] == btn_pin)
			return i;

	return 0;  // NO_PIN
}

uint8_t Btn_Read(const uint16_t btn_pin)
{
	const uint8_t ind = Btn_Index(btn_pin);
	btn_arr[ind].prev_state = btn_arr[ind].state;
	return btn_arr[ind].state;
}

uint8_t Btn_Hold(const uint16_t btn_pin, const uint16_t hold_time_ms)
{
	const uint8_t ind = Btn_Index(btn_pin);
	const uint8_t hold = ms_ticks - btn_arr[ind].ms_change >= hold_time_ms;
	return hold && btn_arr[ind].state;

}

uint8_t Btn_Falling(const uint16_t btn_pin)
{
// WARNING: this makes that falling egde can't be detected more than once but that's a problem if
// one wants to use multiple falling edge detections for a single button press

	const uint8_t ind = Btn_Index(btn_pin);
	const uint8_t is_falling = (btn_arr[ind].prev_state == 0 && btn_arr[ind].state == 1);
	btn_arr[ind].prev_state = btn_arr[ind].state;

	return is_falling;
}

uint8_t Btn_Rising(const uint16_t btn_pin)
{
// WARNING: this makes that rising egde can't be detected more than once but that's a problem if
// one wants to use multiple rising edge detections for a single button press

	const uint8_t ind = Btn_Index(btn_pin);
	const uint8_t is_rising = (btn_arr[ind].prev_state == 1 && btn_arr[ind].state == 0);
	btn_arr[ind].prev_state = btn_arr[ind].state;
	return is_rising;
}

void Button_Actions(void)
{
	Temps16 temps;

	if (Btn_Falling(BTN_UP_Pin))
	{
		temps = Temp_Max_All_Time();
		Print_Simple(3, " Max of All Time");
		Print_Temps(4, temps);
	}

	if (Btn_Falling(BTN_LEFT_Pin))
	{
		temps = Temp_Max_Last_Day();
		Print_Simple(3, " Max of The Day");
		Print_Temps(4, temps);
	}

	while (Btn_Read(BTN_CENTER_Pin))
	{
		const uint16_t erase_hold_time = 3000;
		int erase_in = erase_hold_time - ms_ticks + btn_arr[Btn_Index(BTN_CENTER_Pin)].ms_change;

		Print_Simple(1, "     WARNING         \n");
		Print_Simple(2, "   KEEP HOLDING      \n");
		Print_Simple(3, " TO ERASE EEPROM     \n");
		Print_Simple(4, " ERASING IN: %04d    \n", MAX(0, erase_in));
		SSD1306_UpdateScreen();

		if (Btn_Hold(BTN_CENTER_Pin, erase_hold_time))
		{
			Print_Simple(1, "                     \n");
			Print_Simple(2, "   EEPROM ERASED     \n");
			Print_Simple(3, "                     \n");
			Print_Simple(4, "                     \n");
			SSD1306_UpdateScreen();

			EEPROM_Erase();
			Reset_Cycles();

			SSD1306_Fill(0);
			break;
		}

		HAL_Delay(100);
		SSD1306_Fill(0);
	}

	if (Btn_Falling(BTN_RIGHT_Pin))
	{
		temps = Temp_Min_Last_Day();
		Print_Simple(3, " Min of Last Day");
		Print_Temps(4, temps);
	}

	if (Btn_Falling(BTN_DOWN_Pin))
	{
		temps = Temp_Min_All_Time();
		Print_Simple(3, " Min of All Time");
		Print_Temps(4, temps);
	}

	SSD1306_UpdateScreen();
}

Temps16 Temp_Max_All_Time(void)
{
	Temps16 t_max = { 0, 0, 0 };
	for (int i = 0; i < EEPROM_SIZE_BYTES / sizeof(Temps16); i++)
	{
		Temps16 t = Temp_Read(i);
		if (t.temp_adc == EEPROM_ERASED_TEMP)
			break;

		t_max.temp_1w = MAX(t.temp_1w, t_max.temp_1w);
		t_max.temp_i2c = MAX(t.temp_i2c, t_max.temp_i2c);
		t_max.temp_adc = MAX(t.temp_adc, t_max.temp_adc);
	}

	return t_max;
}

Temps16 Temp_Max_Last_Day(void)
{
	Temps16 t_max = { 0, 0, 0 };
	int mem_start = MAX(0, cycle_count - CYCLES_PER_DAY);

	for (int i = mem_start; i < EEPROM_SIZE_BYTES / sizeof(Temps16); i++)
	{
		Temps16 t = Temp_Read(i);
		if (t.temp_adc == EEPROM_ERASED_TEMP)
			break;

		t_max.temp_1w = MAX(t.temp_1w, t_max.temp_1w);
		t_max.temp_i2c = MAX(t.temp_i2c, t_max.temp_i2c);
		t_max.temp_adc = MAX(t.temp_adc, t_max.temp_adc);
	}

	return t_max;
}

Temps16 Temp_Min_All_Time(void)
{
	Temps16 t_min = { UINT16_MAX, UINT16_MAX, UINT16_MAX };

	for (int i = 0; i < EEPROM_SIZE_BYTES / sizeof(Temps16); i++)
	{
		Temps16 t = Temp_Read(i);
		if (t.temp_adc == EEPROM_ERASED_TEMP)
			break;

		t_min.temp_1w = MIN(t.temp_1w, t_min.temp_1w);
		t_min.temp_i2c = MIN(t.temp_i2c, t_min.temp_i2c);
		t_min.temp_adc = MIN(t.temp_adc, t_min.temp_adc);
	}

	return t_min;
}

Temps16 Temp_Min_Last_Day(void)
{
	Temps16 t_min = { UINT16_MAX, UINT16_MAX, UINT16_MAX };
	int mem_start = MAX(0, cycle_count - CYCLES_PER_DAY);

	for (int i = mem_start; i < EEPROM_SIZE_BYTES / sizeof(Temps16); i++)
	{
		Temps16 t = Temp_Read(i);
		if (t.temp_adc == EEPROM_ERASED_TEMP)
			break;

		t_min.temp_1w = MIN(t.temp_1w, t_min.temp_1w);
		t_min.temp_i2c = MIN(t.temp_i2c, t_min.temp_i2c);
		t_min.temp_adc = MIN(t.temp_adc, t_min.temp_adc);
	}

	return t_min;
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
