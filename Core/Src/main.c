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
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_LEN   1

#define LSM9DS1_ADDR	0x6B<<1	//module's address - ACC/GYR
#define WHO_AM_I		0x0F	//if 0x68 is returned - communication OK

#define CTRL_REG1_G		0x10
#define CTRL_REG2_G		0x11
#define CTRL_REG3_G		0x12

#define CTRL_REG4		0X1E
#define CTRL_REG5_XL	0X1F
#define CTRL_REG6_XL	0X20
#define CTRL_REG7_XL	0X21

#define OUT_X_G		0x18
#define OUT_Y_G		0x1A
#define OUT_Z_G		0x1C

#define OUT_X_A		0x28
#define OUT_Y_A		0x2A
#define OUT_Z_A		0x2C

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

#define LINE_MAX_LENGTH	80

static char line_buffer[LINE_MAX_LENGTH + 1];
static uint32_t line_length;
static uint8_t TX_BUFFER[BUFFER_LEN] = {0};
static uint8_t read;
static uint8_t distance = 0;
static uint8_t prev_distance = 0;
static bool ROBOT = false;

/*-----UART1-USB-------*/
/*-----UART2-MODUL-BLUETOOTH-----*/

void line_append(uint8_t value)
{
	if (value == '\r' || value == '\n')
	{
		// odebraliśmy znak końca linii
		if (line_length > 0)
		{
			// jeśli bufor nie jest pusty to dodajemy 0 na końcu linii
			line_buffer[line_length] = '\0';
			// przetwarzamy dane
			printf("Otrzymano: %s\n", line_buffer);
			// zaczynamy zbieranie danych od nowa
			line_length = 0;
		}
	}else{
		if (line_length >= LINE_MAX_LENGTH)
		{
			// za dużo danych, usuwamy wszystko co odebraliśmy dotychczas
			line_length = 0;
		}
		// dopisujemy wartość do bufora
		line_buffer[line_length++] = value;
	}
}

//This function enables using 'printf()' for sending data by uart
int __io_putchar(int ch)
{
    if (ch == '\n')
    {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart1, &ch2, 1, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

//read from LSM9DS1 module
uint8_t lsm_read_reg(uint8_t reg)
{
	uint8_t value = 0;
	HAL_I2C_Mem_Read(&hi2c1, LSM9DS1_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);

	return value;
}

//write to LSM9DS1 module
void lps_write_reg(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1, LSM9DS1_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //przerwanie od receivera uart
{
	//uint8_t value;
	//if (HAL_UART_Receive(&huart2, &received, 1, 0) == HAL_OK)
	//line_append(received);
	//atoi((char*)&received)
}

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //HAL_UART_Receive_IT(&huart1, &received, 1); //wlaczenie nasluchiwania na kanale UART

  //zegar dla czujnika odleglosci
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  //36 kHz signal IR diode
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  //HAL_UART_Receive_IT(&huart2, &received, 1); //wlaczenie nasluchiwania na kanale UART

  //volatile static uint16_t value[2];

  //HAL_ADC_Start_DMA(&hadc3, (uint32_t*)value, 1);

  /*
  //Checking communication with LSM9DS1 module
  printf("Searching for LSM9DS1...\n");
  uint8_t who_am_i = lsm_read_reg(WHO_AM_I);

  if (who_am_i == 0x68){
	  printf("Found: LSM9DS1\n");

	  //Set 476Hz and 2000dsp
	  //lps_write_reg(CTRL_REG1, 0xB8);
	  lps_write_reg(CTRL_REG1_G, 0xC0);
	  HAL_Delay(100);
	  lps_write_reg(CTRL_REG2_G, 0x00);
	  HAL_Delay(100);
	  lps_write_reg(CTRL_REG3_G, 0x00);
	  HAL_Delay(100);
	  lps_write_reg(CTRL_REG4, 0x38);
	  HAL_Delay(100);
	  lps_write_reg(CTRL_REG5_XL, 0x38);
	  HAL_Delay(100);
	  lps_write_reg(CTRL_REG6_XL, 0x00);
	  HAL_Delay(100);
	  lps_write_reg(CTRL_REG7_XL, 0x00);
	  HAL_Delay(100);
  }else {
	  printf("Error: (0x%02X)\n", who_am_i);
  }

  int16_t gyro[3];
   */


/*  double gyro[3];
  uint8_t data[6];
  int16_t i = 0, gyro_data[3];
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*-------------czytanie co przychodzi z UART---------------*/
	  if (HAL_UART_Receive(&huart2, &read, 1, 0) == HAL_OK)
		  line_append(read);

	  /*--------------VOLTAGE MEASUREMENT------------PA3*/
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	  uint32_t value = HAL_ADC_GetValue(&hadc1);
	  float voltage = 3.3f * value / 4096.0f;

	  //printf("ADC = %lu (%.3f V)\n", value, voltage);
	  //HAL_Delay(250);

	  /*--------------CZUJNIK ODLEGLOSCI----------------*/
	  uint32_t start = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
	  uint32_t stop = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);

	  distance = (stop - start) / 58;
	  //printf("STACJA: %u\n", distance);

	  if (prev_distance != distance)
		  prev_distance = distance;

	  if (ROBOT == false && prev_distance < 4)
	  {
		  ROBOT = true;

		  //wyslanie info, ze robot jest przed stykami
		  TX_BUFFER[0] = '1';
		  HAL_UART_Transmit(&huart2, TX_BUFFER, sizeof(TX_BUFFER), HAL_MAX_DELAY);
	  }
	  else if (ROBOT == true && distance > 4)
	  {
		  ROBOT = false;

		  //wyslanie info, ze robot oddalil sie od stykow
		  TX_BUFFER[0] = '2';
		  HAL_UART_Transmit(&huart2, TX_BUFFER, sizeof(TX_BUFFER), HAL_MAX_DELAY);
	  }

	  /*--------------------STEROWANIE ROBOTEM Z POZIOMU KOMPTERA-USB-UART-------------*/
	  uint8_t uart1;
	  if (HAL_UART_Receive(&huart1, &uart1, 1, 0) == HAL_OK)
	  {
		  line_append(uart1);

		  TX_BUFFER[0] = uart1;
		  HAL_UART_Transmit(&huart2, TX_BUFFER, 1, HAL_MAX_DELAY);
	  }



/*---------------9 DOF HERE----------------
	  if (who_am_i == 0x68){

//		HAL_I2C_Mem_Read(&hi2c1, LSM9DS1_ADDR, 0x18, 1, data, 6, 0x100);
//
//		for (i = 0; i < 3; i++){
//			gyro_data[i] = (data[2*i+1]) | data[2*i];
//			gyro[i] = (double)gyro_data[i]*0.00875;
//		}
//		printf("G1: vx=%2.2lf, vy=%2.2lf, vz=%2.2lf,\n", gyro[0], gyro[1], gyro[2]);

		HAL_I2C_Mem_Read(&hi2c1, LSM9DS1_ADDR, OUT_X_G | 0x80, 1, (uint8_t*)&gyro[0], sizeof(gyro[0]), HAL_MAX_DELAY);
		HAL_I2C_Mem_Read(&hi2c1, LSM9DS1_ADDR, OUT_Y_G | 0x80, 1, (uint8_t*)&gyro[1], sizeof(gyro[1]), HAL_MAX_DELAY);
		HAL_I2C_Mem_Read(&hi2c1, LSM9DS1_ADDR, OUT_Z_G | 0x80, 1, (uint8_t*)&gyro[2], sizeof(gyro[2]), HAL_MAX_DELAY);

		//printf("G: %2.2lf, %2.2lf, %2.2lf,\n", gyro[0], gyro[1], gyro[2]);
		HAL_Delay(500);
	  } else {
		printf("Error: (0x%02X)\n", who_am_i);
	  }
----------------END 9 DOF HERE----------------*/


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
