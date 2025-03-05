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
#include "lcd1602_i2c.h"
#include "myTypes.h"
#include "Ptimer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS1621_ADDR 0x48 << 1
#define BOUNCE_TIME 30
#define ANTIBOUNCE_TIMER 3
#define WAITER_TIMEOUT 5000
#define WAITER_TIMER 6
#define TEMP_TIMEOUT 2000
#define TEMP_TIMER 5
#define LCD_TIMEOUT 1000
#define LCD_TIMER 4
#define TEMPERATURE_HISTORY_SIZE 720
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
lcd1602_HandleTypeDef lcd1602_Handle;
volatile globalFlags_t gflag = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

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
	buttonStates_t button_state = notpressed;
	buttonAntibounceStates_t button_antibounce_state = wait;
	buttonPinState_t button_pin = {GPIO_PIN_RESET, GPIO_PIN_RESET};
	buttonFlags_t button_flag = {0, 0};
	buttonAntibounceFlags_t  button_antibounce_flag = {0, 0};
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	int temperatureHistory[TEMPERATURE_HISTORY_SIZE];
	int historyIndex = 0;
	int mode = 0;
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_I2C3_Init();
	/* USER CODE BEGIN 2 */
	DS1621_Init();
	lcd1602_Init(&lcd1602_Handle, &hi2c1, PCF8574_ADDRESS);
	int temperature = DS1621_ReadTemp();
	int currentTemp;
	int maxTemp = temperature;
	int minTemp = temperature;
	for (int i = 0; i < TEMPERATURE_HISTORY_SIZE; i++) {
		temperatureHistory[i] = temperature;
	}
	InitPTimer();
	SetPTimer(TEMP_TIMER, TEMP_TIMEOUT);
	SetPTimer(LCD_TIMER, LCD_TIMEOUT);
	SetPTimer(WAITER_TIMER, WAITER_TIMEOUT);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if ((ptimerFlag.timer) & (1 << TEMP_TIMER)){
			ptimerFlag.timer &= ~(1 << TEMP_TIMER);
			KillPTimer (TEMP_TIMER);

			temperature = DS1621_ReadTemp();

			SetPTimer(TEMP_TIMER, TEMP_TIMEOUT);
		}

		if ((ptimerFlag.timer) & (1 << WAITER_TIMER)){
			ptimerFlag.timer &= ~(1 << WAITER_TIMER);
			KillPTimer (WAITER_TIMER);

			temperatureHistory[historyIndex] = temperature;
			historyIndex = (historyIndex + 1) % TEMPERATURE_HISTORY_SIZE;

			maxTemp = temperature;
			minTemp = temperature;

			for (int i = 0; i < TEMPERATURE_HISTORY_SIZE; i++) {
				if (temperatureHistory[i] > maxTemp) {
					maxTemp = temperatureHistory[i];
				}
				if (temperatureHistory[i] < minTemp) {
					minTemp = temperatureHistory[i];
				}
			}
			SetPTimer(WAITER_TIMER, WAITER_TIMEOUT);
		}

		/* Button state machine*/
		switch (button_antibounce_state)
		{
		case wait:
		{
			if (gflag.button_action)
			{
				button_antibounce_state = antibounce;
				button_pin.previous_state = HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_3);
				SetPTimer (ANTIBOUNCE_TIMER, BOUNCE_TIME);
				gflag.button_action = 0;
			}
			break;
		}
		case antibounce:
		{
			if ((ptimerFlag.timer) & (1 << ANTIBOUNCE_TIMER))
			{
				button_pin.current_state = HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_3);
				gflag.button_action = 0;
				if (button_pin.current_state == button_pin.previous_state)
				{
					if (button_pin.current_state == GPIO_PIN_SET)
						button_antibounce_flag.falling_edge = 1;
					else
						button_antibounce_flag.rising_edge = 1;

					ptimerFlag.timer &= ~(1 << ANTIBOUNCE_TIMER);
					button_antibounce_state = wait;
				}
				else
				{
					button_pin.previous_state = button_pin.current_state;
					SetPTimer (ANTIBOUNCE_TIMER, BOUNCE_TIME);
				}
			}
			break;
		}
		default:
			break;
		}

		/* Button state machine*/
		switch (button_state)
		{
		case notpressed:
		{
			if (button_antibounce_flag.rising_edge)
			{
				button_state = pressed;
				button_flag.pressed = 1;
				button_antibounce_flag.rising_edge = 0;
			}
			else if (button_antibounce_flag.falling_edge)
			{
				button_antibounce_flag.falling_edge = 0;
			}
			break;
		}
		case pressed:
		{
			if (button_antibounce_flag.falling_edge)
			{
				button_state = notpressed;
				button_flag.released = 1;
				button_antibounce_flag.falling_edge = 0;
			}
			else if (button_antibounce_flag.rising_edge)
			{
				button_antibounce_flag.rising_edge = 0;
			}
			break;
		}
		default:
			break;
		}

		if (button_flag.pressed)
		{
			mode = (mode + 1) % 3;
			button_flag.pressed = 0;
		}

		if ((ptimerFlag.timer) & (1 << LCD_TIMER)){
			ptimerFlag.timer &= ~(1 << LCD_TIMER);
			KillPTimer (LCD_TIMER);
			lcd1602_Clear(&lcd1602_Handle);
			switch (mode)
			{
			case 0:
			{
				currentTemp = temperature;
				lcd1602_Print(&lcd1602_Handle, "Current");
				SendTemperature(currentTemp);
				break;
			}
			case 1:
			{
				lcd1602_Print(&lcd1602_Handle, "Max");
				SendTemperature(maxTemp);
				break;
			}
			case 2:
			{
				lcd1602_Print(&lcd1602_Handle, "Min");
				SendTemperature(minTemp);
				break;
			}
			default:
				break;
			}
			SetPTimer(LCD_TIMER, LCD_TIMEOUT);
		}
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

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
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
	hi2c1.Init.Timing = 0x00B07CB4;
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
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00B07CB4;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

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
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DS1621_Init() {
	uint8_t cmd = 0xEE;

	HAL_I2C_Master_Transmit(&hi2c3, DS1621_ADDR, &cmd, 1, 10000);

	return;
}
int DS1621_ReadTemp() {
	uint8_t cmd = 0xAA;
	uint8_t temp;

	HAL_I2C_Master_Transmit(&hi2c3, DS1621_ADDR, &cmd, 1, HAL_MAX_DELAY);

	HAL_I2C_Master_Receive(&hi2c3, DS1621_ADDR, &temp, 1, HAL_MAX_DELAY);

	return temp;
}
void SendTemperature(int temp) {
	char lcdBuffer[16];

	sprintf(lcdBuffer, "Temp: %d C", temp);
	lcd1602_SetCursor(&lcd1602_Handle, 0, 1);
	lcd1602_Print(&lcd1602_Handle, lcdBuffer);
}
inline void HAL_SYSTICK_Callback(void)
{
  PTimer();
}

inline void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 /* Prevent unused argument(s) compilation warning */
 UNUSED(GPIO_Pin);
 /* Set button pressed flag and disable EXTI0 IRQ */
 gflag.button_action = 1;

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
