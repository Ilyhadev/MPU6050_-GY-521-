/*
 * mpu6050.c
 *
 *  Created on: Jul 3, 2025
 *      Author: ilia1 (iliawork112005@gmail.com)
 */

#include "mpu6050.h"

typedef struct {
    float Ax;
    float Ay;
    float Az;
    float accelerometer [3]; // All 3 accelerations measurements are here (x, y, z)
} accelerometer_t;

typedef struct {
    float Gx;
    float Gy;
    float Gz;
    float gyroscope [3];
} gyroscope_t;

typedef struct {
    accelerometer_t accelerometer;
    gyroscope_t gyroscope;
    float temperature;
    bool isInit;
} mpu6050_t;


static mpu6050_t mpu6050;




void MPU6050_Init (void) // WHO AM I is to verify the identity of device
{
  uint8_t check;
  uint8_t Data;

  HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 1000);  // read WHO_AM_I

  HAL_DELAY(100); // Small delay

   if (check == MPU6050_ADDR_WO_SHIFT)  // 0x68 will be returned by the sensor if everything goes well
  {
	  // power management register 0X6B we should write all 0's to wake the sensor up
	  Data = 0;
	  /*
	   * Select the internal clock source of 8 MHz.
	   * The Temperature sensor will be enabled.
       * The CYCLE between sleep mode and wakeup will be enabled.
       * The SLEEP mode will be disabled.
       * Also we are not performing the RESET.
       * CONFIG 0x1A is to configure DLPF is by default 0
      */
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 1000);

	  // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	  // output rate = 8000 Hz when ?DLPF? is disabled
	  Data = 0x07; // it's divisor (SMPLR_DIV)
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, 1000);

	  // Set accelerometer configuration in ACCEL_CONFIG Register
	  Data = 0x00;  // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> <strong>±</strong> 2g
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

	  // Set Gyroscopic configuration in GYRO_CONFIG Register
	  Data = 0x00;  // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> <strong>±</strong> 250 ̐/s
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	  mpu6050.isInit = true;
  }
}


float MPU6050_Read_Accel_X (void)
{
	uint8_t Rec_Data[2];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 2, 1000);

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

	mpu6050.accelerometer.Ax = Ax;
	return Ax;
}


float MPU6050_Read_Accel_Y (void)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_YOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Accel_Y_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Ay = (float)Accel_Y_RAW/LSB_SENSITIVITY_ACC;
	mpu6050.accelerometer.Ay = Ay;
	return Ay;
}

float MPU6050_Read_Accel_Z (void)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Accel_Z_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Az = (float)Accel_Z_RAW/LSB_SENSITIVITY_ACC;
	mpu6050.accelerometer.Az = Az;
	return Az;
}


float MPU6050_Read_Gyro_X (void)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_YOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Gx = (float)Gyro_X_RAW/LSB_SENSITIVITY_GYRO;
	mpu6050.gyroscope.Gx = Gx;
	return Gx;
}

float MPU6050_Read_Gyro_Y (void)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Gy = (float)Gyro_Y_RAW/LSB_SENSITIVITY_GYRO;
	mpu6050.gyroscope.Gy = Gy;
	return Gy;
}


float MPU6050_Read_Gyro_Z (void)
{
	uint8_t Rec_Data[2];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H, 1, Rec_Data, 2, 1000);
	int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	float Gz = (float)Gyro_Z_RAW/LSB_SENSITIVITY_GYRO;
	mpu6050.gyroscope.Gz = Gz;
	return Gz;
}

void MPU6050_Enable_FIFO(void)
{
    uint8_t Data = 0x40; // Set FIFO_EN bit (bit 6) to 1 (0100 0000)
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, &Data, 1, 1000); // USER_CTRL register
}

void MPU6050_configure_Fifo (void) // temperature is first 1, next 111 is enable gyroscope from Gx to Gz, and last 1 is for acceleration (000 is about i2c slaves - not relevant in my case)
{
	uint8_t Data = 0xF8; // To enable fifo on Temperature, gyro and accel (1111 1000)
	HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, FIFO_ENABLE, 1, &Data, 1, 1000); // 1 byte to transmit
}

uint16_t MPU6050_Get_FIFO_Count(void)
{
    uint8_t Data_H, Data_L;
    uint16_t FIFO_Count;

    // Read FIFO_COUNT_H first (this updates both registers)
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_COUNT_H, 1, &Data_H, 1, 1000);
    // Then read FIFO_COUNT_L
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_COUNT_L, 1, &Data_L, 1, 1000);

    FIFO_Count = (Data_H << 8) | Data_L; // same trick with assembling data from low and high register
    return FIFO_Count;
}

void MPU6050_Read_Fifo (void)
{

}
