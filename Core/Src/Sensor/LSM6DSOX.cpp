#include "LSM6DSOX.h"
#include <math.h>

// Constructor
LSM6DSOX::LSM6DSOX(I2C_HandleTypeDef *handle) {
	i2cHandle = handle;
}

// Write a register (blocking)
void LSM6DSOX::writeRegister(uint8_t reg, uint8_t value) {
	HAL_I2C_Mem_Write(i2cHandle, LSM6DSOX_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

// Read multiple registers into rxBuffer (internal use)
void LSM6DSOX::readSensorData(uint8_t reg, uint8_t len) {
	HAL_I2C_Mem_Read(i2cHandle, LSM6DSOX_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, rxBuffer, len, HAL_MAX_DELAY);
}

// Initialize the sensor
bool LSM6DSOX::init() {
	uint8_t whoAmI = 0;
	HAL_I2C_Mem_Read(i2cHandle, LSM6DSOX_ADDRESS, LSM6DSOX_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &whoAmI, 1, HAL_MAX_DELAY);
	if (whoAmI != LSM6DSOX_WHO_AM_I_VAL) {
		return false;  // Device not found
	}

	// Accelerometer: ±2g, 416 Hz
	writeRegister(LSM6DSOX_CTRL1_XL, 0x60);  // ODR_XL = 416Hz, FS_XL = ±2g
	HAL_Delay(10);

	// Gyroscope: ±2000 dps, 416 Hz
	writeRegister(LSM6DSOX_CTRL2_G, 0x6C);  // ODR_G = 416Hz, FS_G = 2000 dps
	HAL_Delay(10);

	return true;
}

// Process gyro data (rxBuffer[0..5])
void LSM6DSOX::processGyroData() {
	int16_t rawX = (rxBuffer[1] << 8) | rxBuffer[0];
	int16_t rawY = (rxBuffer[3] << 8) | rxBuffer[2];
	int16_t rawZ = (rxBuffer[5] << 8) | rxBuffer[4];

	// ±2000 dps → 70 mdps/LSB → 0.07 dps/LSB
	AngularVelocity_X = rawX * 0.07f;
	AngularVelocity_Y = rawY * 0.07f;
	AngularVelocity_Z = rawZ * 0.07f;
}

// Process accel data (rxBuffer[6..11])
void LSM6DSOX::processAccelData() {
	int16_t rawX = (rxBuffer[7] << 8) | rxBuffer[6];
	int16_t rawY = (rxBuffer[9] << 8) | rxBuffer[8];
	int16_t rawZ = (rxBuffer[11] << 8) | rxBuffer[10];

	// ±2g → 0.061 mg/LSB = 0.000061 g/LSB
	Acceleration_X = rawX * 0.000061f;
	Acceleration_Y = rawY * 0.000061f;
	Acceleration_Z = rawZ * 0.000061f;
}

// Update gyro + accel + angle error
void LSM6DSOX::updateData() {
	uint8_t dataStatus = 0;

	// Check STATUS_REG to see if new gyro/accel data is available
	HAL_I2C_Mem_Read(i2cHandle, LSM6DSOX_ADDRESS, LSM6DSOX_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &dataStatus, 1, HAL_MAX_DELAY);

	if ((dataStatus & 0x03) == 0) {
		// No new data available
		return;
	}

	// Read 12 bytes: gyro (6) + accel (6)
	readSensorData(LSM6DSOX_OUTX_L_G, 12);
	processGyroData();
	processAccelData();

	if (dataStatus == 0x02) {
		HeaderAngleError += AngularVelocity_X / 416.0f;  // 416Hz update rate
	}
}

// Reset angle error
void LSM6DSOX::resetHeading() {
	HeaderAngleError = 0.0f;
}
