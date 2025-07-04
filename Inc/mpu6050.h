/*
 * mpu6050.h
 *
 *  Created on: Jul 3, 2025
 *      Author: ilia1 (iliawork112005@gmail.com)
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include <stdbool.h>

#define FIFO_SAMPLE_SIZE 14


#define LSB_SENSITIVITY_ACC 16384 //(+/- 2g), 8192 (+/- 4g), 4096 (+/- 8g), 2048 (+/- 16g)
#define LSB_SENSITIVITY_GYRO 131//(+/- 250 */s), 65.5 (+/- 500 */s), 32.8 (+/- 1000 */s), 16.4 (+/- 2000 */s)
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C


#define MPU6050_ADDR_WO_SHIFT 0x68
#define MPU6050_ADDR 0x68 << 1 // 0xD0
#define WHO_AM_I 0x75


#define FIFO_ENABLE 0x23
#define FIFO_COUNT_H 0x72
#define FIFO_COUNT_L 0x73
#define FIFO_R_W 0x74
//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) (what we put in register)
//where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz
//when the DLPF is enabled (see Register 26).
#define SMPLRT_DIV 0x19

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define USER_CTRL 0x6A

//This register configures the external Frame Synchronization (FSYNC) pin sampling and the Digital
//Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers.
#define CONFIG 0x1A

// Registers from which accelerometer measurements are read
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

// To read temperature
#define TEMP_H 0x41
#define TEMP_L 0x42

// Registers from which gyroscope measurements are read
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

typedef struct {
    float Ax;
    float Ay;
    float Az;
} accelerometer_t;

typedef struct {
    float Gx;
    float Gy;
    float Gz;
} gyroscope_t;

typedef struct mpu6050_t {
    accelerometer_t accelerometer;
    gyroscope_t gyroscope;
    float temperature;
    bool isInit;
    I2C_HandleTypeDef *_I2C;
} mpu6050_t;

void MPU6050_Init (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1);

void MPU6050_Configure_DLPF(I2C_HandleTypeDef *hi2c1, uint8_t dlpf_value);

void MPU6050_Reset_FIFO(I2C_HandleTypeDef *hi2c1);

float MPU6050_Read_Accel_X (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1);

float MPU6050_Read_Accel_Y (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1);

float MPU6050_Read_Accel_Z (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1);

float MPU6050_Read_Gyro_X (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1);

float MPU6050_Read_Gyro_Y (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1);

float MPU6050_Read_Gyro_Z (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1);

void MPU6050_Enable_FIFO(I2C_HandleTypeDef *hi2c1);

void MPU6050_configure_Fifo (I2C_HandleTypeDef *hi2c1);

uint16_t MPU6050_Get_FIFO_Count(I2C_HandleTypeDef *hi2c1);

void MPU6050_Read_Fifo(mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1);


#endif /* INC_MPU6050_H_ */
