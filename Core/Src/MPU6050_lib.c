/*
 * MPU6050_lib.c
 *
 *  Created on: Feb 9, 2021
 *      Author: Mario A. Lugo
 */

#include "MPU6050_lib.h"


/******************************************************************************
 * @fn				- MPU6050_Init
 * @brief			- This function configure the MPU6050
 *
 * @param[in]		- I2C_HandleTypeDef
 *
 * @return			- none
 *
 * @note			-
 *****************************************************************************/

void MPU6050_Init(I2C_HandleTypeDef *pI2C_Handle)
{

	HAL_I2C_Mem_Write(pI2C_Handle, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, (uint8_t*) 0x07, 1, HAL_MAX_DELAY);// Setting The Sample (Data) Rate
	HAL_I2C_Mem_Write(pI2C_Handle, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, (uint8_t*) 0x00, 1, HAL_MAX_DELAY);// Setting The Clock Source
	HAL_I2C_Mem_Write(pI2C_Handle, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, (uint8_t*) 0x00, 1, HAL_MAX_DELAY);// Configure The ACCEL (FSR= +-2g)
	HAL_I2C_Mem_Write(pI2C_Handle, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, (uint8_t*) 0x18, 1, HAL_MAX_DELAY);// Configure The GYRO (FSR= +-2000d/s)
	HAL_I2C_Mem_Write(pI2C_Handle, MPU6050_ADDR, MPU6050_INT_ENABLE, 1, (uint8_t*) 0x01, 1, HAL_MAX_DELAY);// Enable Data Ready Interrupts

}


/******************************************************************************
 * @fn				- MPU6050_Set_DLPF
 * @brief			- This function configure the MPU6050 internal low pass filter
 *
 * @param[in]		- I2C_HandleTypeDef
 * @param[in]		- MPU6050_DLPF_macro
 *
 * @return			- none
 *
 * @note			-
 *****************************************************************************/

void MPU6050_Set_DLPF(I2C_HandleTypeDef *pI2C_Handle,uint8_t MPU6050_DLPF_macro)
{
	HAL_I2C_Mem_Write(pI2C_Handle, MPU6050_ADDR, MPU6050_CONFIG, 1, &MPU6050_DLPF_macro, 1, HAL_MAX_DELAY);
}

/******************************************************************************
 * @fn				- MPU6050_Read
 * @brief			- This function read the MPU6050 data (Accelerometer & Gyroscope)
 *
 * @param[in]		- I2C_HandleTypeDef
 * @param[in]		- MPU6050_typeDef variable
 *
 * @return			- none
 *
 * @note			-
 *****************************************************************************/

void MPU6050_Read(I2C_HandleTypeDef *pI2C_Handle, MPU6050_typeDef *pDataStruct)
{
	uint8_t mpu_data[12];

	HAL_I2C_Mem_Read(pI2C_Handle, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, mpu_data, 12, HAL_MAX_DELAY);

	pDataStruct->Accel_X_RAW = (int16_t) (mpu_data[0] << 8 | mpu_data[1]);
	pDataStruct->Accel_Y_RAW = (int16_t) (mpu_data[2] << 8 | mpu_data[3]);
	pDataStruct->Accel_Z_RAW = (int16_t) (mpu_data[4] << 8 | mpu_data[5]);
	pDataStruct->Gyro_X_RAW  = (int16_t) (mpu_data[6] << 8 | mpu_data[7]);
	pDataStruct->Gyro_Y_RAW  = (int16_t) (mpu_data[8] << 8 | mpu_data[9]);
	pDataStruct->Gyro_Z_RAW  = (int16_t) (mpu_data[10] << 8 | mpu_data[11]);
}



