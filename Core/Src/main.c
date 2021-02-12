/*
 * main.c
 *
 *  Created on: Feb 9, 2021
 *      Author: Mario A. Lugo
 *       Brief: MPU6050 data acquisition & transmit
 *       AccX,AccY & GyroZ via UART
 */
#include "main.h"
#include "MPU6050_lib.h"

void SystemClock_Config(void);
void UART1_Config(void);
void I2C_Config(void);


UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

MPU6050_typeDef mpu_data;

uint8_t start_frame = 0x24;	//"$"
uint8_t data_frame [6] = {};
uint8_t end_frame [2] = {0x0D, 0x0A};//"\r\n"


int main(void)
{

  HAL_Init();
  SystemClock_Config();
  UART1_Config();
  I2C_Config();

  MPU6050_Init(&hi2c1);
  MPU6050_Set_DLPF(&hi2c1, MPU6050_DLPF_6);

  while (1)
  {

	  MPU6050_Read(&hi2c1, &mpu_data);

	  data_frame[0] = (uint8_t) mpu_data.Accel_X_RAW;
	  data_frame[1] = (uint8_t) (mpu_data.Accel_X_RAW >> 8);
	  data_frame[2] = (uint8_t) mpu_data.Accel_Y_RAW;
	  data_frame[3] = (uint8_t) (mpu_data.Accel_Y_RAW >> 8);
	  data_frame[4] = (uint8_t) mpu_data.Gyro_Z_RAW;
	  data_frame[5] = (uint8_t) (mpu_data.Gyro_Z_RAW >> 8);

	  HAL_UART_Transmit(&huart1, &start_frame, 1, HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart1, data_frame, 6, HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart1, end_frame, 2, HAL_MAX_DELAY);

	  HAL_Delay(20);

  }

}

/**
  * @brief UART1 configuration
  * @retval None
  */
void UART1_Config(void)
{
	GPIO_InitTypeDef tx;
	tx.Mode = GPIO_MODE_AF_PP;
	tx.Pin  = GPIO_PIN_9;
	tx.Pull = GPIO_PULLUP;
	tx.Speed =GPIO_SPEED_FREQ_LOW;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &tx);

	__HAL_RCC_USART1_CLK_ENABLE();
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 57600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&huart1);

}


/**
  * @brief I2C1 configuration
  * @retval None
  */
void I2C_Config(void)
{
	HAL_I2C_MspInit(&hi2c1);
	__HAL_RCC_I2C1_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef scl_pin,sda_pin;

	scl_pin.Mode = GPIO_MODE_AF_OD;
	scl_pin.Pin  = GPIO_PIN_6;
	scl_pin.Pull = GPIO_NOPULL;
	scl_pin.Speed =GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &scl_pin);

	sda_pin.Mode = GPIO_MODE_AF_OD;
	sda_pin.Pin  = GPIO_PIN_7;
	sda_pin.Pull = GPIO_NOPULL;
	sda_pin.Speed =GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &sda_pin);

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 200000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;

	HAL_I2C_Init(&hi2c1);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }

}

