/*
 * mpu6050.c
 *
 *  Created on: Jul 3, 2025
 *      Author: ilia1 (iliawork112005@gmail.com)
 */

#include "mpu6050.h"
#include <stdint.h>
#include <stdbool.h>



void MPU6050_Init (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1) // WHO AM I is to verify the identity of device
{
  uint8_t check;
  uint8_t Data;
  mpu6050->_I2C = hi2c1;

  HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 1000);  // read WHO_AM_I

  HAL_Delay(100); // Small delay

   if (check == MPU6050_ADDR_WO_SHIFT)  // 0x68 will be returned by the sensor if everything goes well
  {
	  Data = 0x80; // Device reset
	  HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 1000);
	  HAL_Delay(100);

	  // Wake up and set clock source to PLL with X-axis gyro reference
	  Data = 0x01; // Use PLL with X-axis gyro, not internal oscillator
	  HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 1000);
	  HAL_Delay(100);

	  // Ensure all sensors are enabled
	  Data = 0x00; // Enable all accelerometer and gyroscope axes
	  HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, PWR_MGMT_2, 1, &Data, 1, 1000);
	  HAL_Delay(50);

	  // Configure DLPF first (affects sample rate)
	  MPU6050_Configure_DLPF(hi2c1, 0x03); // 42Hz bandwidth instead of 0x01

	  // Set sample rate: Sample Rate = 1kHz / (1 + SMPLRT_DIV)
	  // For 100Hz: SMPLRT_DIV = 9
	  Data = 0x09; // 100Hz sample rate
	  HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, 1000);

	  // Set accelerometer configuration in ACCEL_CONFIG Register
	  Data = 0x00;  // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> <strong>±</strong> 2g
	  HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

	  // Set Gyroscopic configuration in GYRO_CONFIG Register
	  Data = 0x00;  // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> <strong>±</strong> 250 ̐/s
	  HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	  mpu6050->isInit = true;

	  mpu6050->accelerometer.Ax = 0;
	  mpu6050->accelerometer.Ay = 0;
	  mpu6050->accelerometer.Az = 0;

	  mpu6050->gyroscope.Gx = 0;
	  mpu6050->gyroscope.Gy = 0;
	  mpu6050->gyroscope.Gz = 0;
	  mpu6050->temperature = 0;

  }
}

void MPU6050_Configure_DLPF(I2C_HandleTypeDef *hi2c1, uint8_t dlpf_value)
{
    // Configure DLPF - Register 0x1A (CONFIG)
    // dlpf_value: 1=188Hz, 2=98Hz, 3=42Hz, 4=20Hz, 5=10Hz, 6=5Hz
    HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, CONFIG, 1, &dlpf_value, 1, 1000);
}

void MPU6050_Reset_FIFO(I2C_HandleTypeDef *hi2c1) {
    uint8_t current_value;

    // Read current USER_CTRL register
    HAL_I2C_Mem_Read(hi2c1, MPU6050_ADDR, USER_CTRL, 1, &current_value, 1, 1000);

    // Disable FIFO
    current_value &= ~0x40; // Clear FIFO_EN bit
    HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, USER_CTRL, 1, &current_value, 1, 1000);
    HAL_Delay(1);

    // Reset FIFO
    current_value |= 0x04; // Set FIFO_RESET bit
    HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, USER_CTRL, 1, &current_value, 1, 1000);
    HAL_Delay(1);

    // Clear reset bit and re-enable FIFO
    current_value &= ~0x04; // Clear FIFO_RESET bit
    current_value |= 0x40;  // Set FIFO_EN bit
    HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, USER_CTRL, 1, &current_value, 1, 1000);
}

float MPU6050_Read_Accel_X (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1)
{
	uint8_t Rec_Data[2];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 2, 1000);

	// So 0x3B is a register which stores Higher and Lower bytes (all two)
	// example of content 0x1234 where 0x12 is higher and  0x34 is lower
	// As we need to get the whole 0x1234 we will "concatenate" two bytes: lower and higher
	// How? We will shift higher byte to the left on 8 bits so there is place for lower byte
	// Example: 0001 0010 (is 0x12) - higher byte
	// Then shift to 8
	// 0001 0010 0000 0000 and then logical OR with data in lower register (0011 0100)
	// At the end we got all value
	int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	// The same goes for Y and Z
	// logically as array addressing is organised in C: 0x3B is XOUT_H, 0x3B+"1" is XOUT_L
	// Then 0x3B+"2" is YOUT_H, 0x3B+"3" is YOUT_L
	float Ax = (float)Accel_X_RAW/LSB_SENSITIVITY_ACC;

	mpu6050->accelerometer.Ax = Ax;
	return Ax;
}


float MPU6050_Read_Accel_Y (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, ACCEL_YOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Accel_Y_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Ay = (float)Accel_Y_RAW/LSB_SENSITIVITY_ACC;
	mpu6050->accelerometer.Ay = Ay;
	return Ay;
}

float MPU6050_Read_Accel_Z (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, ACCEL_ZOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Accel_Z_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Az = (float)Accel_Z_RAW/LSB_SENSITIVITY_ACC;
	mpu6050->accelerometer.Az = Az;
	return Az;
}


float MPU6050_Read_Gyro_X (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Gx = (float)Gyro_X_RAW/LSB_SENSITIVITY_GYRO;
	mpu6050->gyroscope.Gx = Gx;
	return Gx;
}

float MPU6050_Read_Gyro_Y (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, GYRO_YOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Gy = (float)Gyro_Y_RAW/LSB_SENSITIVITY_GYRO;
	mpu6050->gyroscope.Gy = Gy;
	return Gy;
}


float MPU6050_Read_Gyro_Z (mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, GYRO_ZOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Gz = (float)Gyro_Z_RAW/LSB_SENSITIVITY_GYRO;
	mpu6050->gyroscope.Gz = Gz;
	return Gz;
}

void MPU6050_Enable_FIFO(I2C_HandleTypeDef *hi2c1) {
    uint8_t current_value;
    // Read current USER_CTRL register value
    HAL_I2C_Mem_Read(hi2c1, MPU6050_ADDR, USER_CTRL, 1, &current_value, 1, 1000);

    // Set FIFO_EN bit (bit 6) while preserving other bits
    current_value |= 0x40;
    HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, USER_CTRL, 1, &current_value, 1, 1000);
}

void MPU6050_configure_Fifo (I2C_HandleTypeDef *hi2c1) // temperature is first 1, next 111 is enable gyroscope from Gx to Gz, and last 1 is for acceleration (000 is about i2c slaves - not relevant in my case)
{
	uint8_t Data = 0xF8; // To enable fifo on Temperature, gyro and accel (1111 1000)
	// If want to change Data to read different in FIFO DON'T FORGET to change FIFO_SAMPLE_SIZE in mpu6050.h
	HAL_I2C_Mem_Write (hi2c1, MPU6050_ADDR, FIFO_ENABLE, 1, &Data, 1, 1000); // 1 byte to transmit
}

uint16_t MPU6050_Get_FIFO_Count (I2C_HandleTypeDef *hi2c1)
{
    uint8_t Data_H, Data_L;
    uint16_t FIFO_Count;

    // Read FIFO_COUNT_H first (this updates both registers)
    HAL_I2C_Mem_Read(hi2c1, MPU6050_ADDR, FIFO_COUNT_H, 1, &Data_H, 1, 1000);
    // Then read FIFO_COUNT_L
    HAL_I2C_Mem_Read(hi2c1, MPU6050_ADDR, FIFO_COUNT_L, 1, &Data_L, 1, 1000);

    FIFO_Count = (Data_H << 8) | Data_L; // same trick with assembling data from low and high register
    return FIFO_Count;
}

void MPU6050_Read_Fifo(mpu6050_t *mpu6050, I2C_HandleTypeDef *hi2c1) {
    uint8_t fifo_buffer[FIFO_SAMPLE_SIZE];
    uint16_t fifo_count = MPU6050_Get_FIFO_Count(hi2c1);
    int16_t raw_data[FIFO_SAMPLE_SIZE / 2];

    // Check for FIFO overflow
    if (fifo_count >= 1024) {
        MPU6050_Reset_FIFO(hi2c1);
        HAL_Delay(100); // Allow time for new data
        return;
    }

    // Only read if we have at least one complete sample
    if (fifo_count >= FIFO_SAMPLE_SIZE) {
        // Read multiple samples if available to prevent overflow
        while (fifo_count >= FIFO_SAMPLE_SIZE && fifo_count < 1024) {
            HAL_I2C_Mem_Read(hi2c1, MPU6050_ADDR, FIFO_R_W, 1,
                           fifo_buffer, FIFO_SAMPLE_SIZE, 1000);

            // Process the data (your existing processing code)
            for (int i = 0; i < FIFO_SAMPLE_SIZE / 2; i++) {
                raw_data[i] = (int16_t)((fifo_buffer[2 * i] << 8) |
                                      fifo_buffer[2 * i + 1]);
            }

            mpu6050->accelerometer.Ax = (float)raw_data[0] / LSB_SENSITIVITY_ACC;
            mpu6050->accelerometer.Ay = (float)raw_data[1] / LSB_SENSITIVITY_ACC;
            mpu6050->accelerometer.Az = (float)raw_data[2] / LSB_SENSITIVITY_ACC;

            mpu6050->temperature      = ((float)raw_data[3] / 340.0f) + 36.53f;

            mpu6050->gyroscope.Gx     = (float)raw_data[4] / LSB_SENSITIVITY_GYRO;
            mpu6050->gyroscope.Gy     = (float)raw_data[5] / LSB_SENSITIVITY_GYRO;
            mpu6050->gyroscope.Gz     = (float)raw_data[6] / LSB_SENSITIVITY_GYRO;

            // Check remaining FIFO count
            fifo_count = MPU6050_Get_FIFO_Count(hi2c1);
        }
    }
}
