/*
 * MPU6050_lib.h
 *
 *  Created on: Feb 9, 2021
 *      Author: Mario A. Lugo
 */

#ifndef INC_MPU6050_LIB_H_
#define INC_MPU6050_LIB_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"


#define MPU6050_ADDR 		0xD0
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_INT_ENABLE          0x38

#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48

#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_PWR_MGMT_2          0x6C



#define MPU6050_DLPF_0				0
#define MPU6050_DLPF_1				1
#define MPU6050_DLPF_2				2
#define MPU6050_DLPF_3				3
#define MPU6050_DLPF_4				4
#define MPU6050_DLPF_5				5
#define MPU6050_DLPF_6				6



typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

} MPU6050_typeDef;

void MPU6050_Init(I2C_HandleTypeDef *pI2C_Handle);
void MPU6050_Set_DLPF(I2C_HandleTypeDef *pI2C_Handle,uint8_t DLPF_CFG_macro);
void MPU6050_Read(I2C_HandleTypeDef *pI2C_Handle, MPU6050_typeDef *pDataStruct);


#endif /* INC_MPU6050_LIB_H_ */
