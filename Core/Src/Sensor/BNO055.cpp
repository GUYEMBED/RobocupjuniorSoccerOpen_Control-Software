#include "BNO055.h"

// Constructor
BNO055::BNO055() {
}

// Write to a register
void BNO055::writeRegister(uint8_t reg, uint8_t value) {
	uint8_t data[2] = { reg, value };
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_ADDRESS, data, 2, HAL_MAX_DELAY);
	HAL_Delay(10);
}

// Read from a register
void BNO055::readRegister(uint8_t reg, uint8_t *buffer, uint8_t len) {
	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buffer, len, HAL_MAX_DELAY);
}

// Initialize the sensor
void BNO055::init() {
	// Step 1: Soft reset
	writeRegister(0x3F, 0x20); // SYS_TRIGGER: Reset
	HAL_Delay(650);            // Delay >650ms after reset

	// Step 2: Check chip ID (optional but helpful)
	uint8_t id = 0;
	readRegister(0x00, &id, 1); // CHIP_ID
	if (id != 0xA0) {
		// Retry logic or error handling here
	}

	// Step 3: Set to config mode
	writeRegister(BNO055_OPR_MODE, BNO055_CONFIG_MODE);
	HAL_Delay(25);

	// Step 4: Set power mode to NORMAL (0x00)
	writeRegister(BNO055_PWR_MODE, 0x00);
	HAL_Delay(10);

	// Step 5: Set units (optional)
	writeRegister(BNO055_UNIT_SEL, 0x01); // Deg + DPS

	// Step 6: Set operation mode to NDOF (0x0C) or IMU (0x08)
	writeRegister(BNO055_OPR_MODE, BNO055_NDOF_MODE); // or BNO055_IMU_MODE
	HAL_Delay(50);
}

// Check if new data is available
bool BNO055::isDataReady() {
	uint8_t status = 0;
	readRegister(BNO055_INT_STA, &status, 1);
	return (status & 0x01) || (status & 0x02);  // GYR_INT or ACC_INT
}

// Read gyroscope data
void BNO055::readGyro() {
	uint8_t buffer[6];
	readRegister(BNO055_GYR_DATA_X_LSB, buffer, 6);

	int16_t rawX = (int16_t)((buffer[1] << 8) | buffer[0]);
	int16_t rawY = (int16_t)((buffer[3] << 8) | buffer[2]);
	int16_t rawZ = (int16_t)((buffer[5] << 8) | buffer[4]);

	AngularVelocity_Xaxis = rawX / 16.0f;
	AngularVelocity_Yaxis = rawY / 16.0f;
	AngularVelocity_Zaxis = rawZ / 16.0f;
}

// Read compass (Euler angles)
void BNO055::readCompass() {
	uint8_t buffer[6];
	readRegister(BNO055_EUL_DATA_X_LSB, buffer, 6);

	int16_t rawH = (buffer[1] << 8) | buffer[0];
	int16_t rawP = (buffer[3] << 8) | buffer[2];
	int16_t rawR = (buffer[5] << 8) | buffer[4];

	Heading = rawH / 16.0f;
	Pitch = rawP / 16.0f;
	Roll = rawR / 16.0f;

	static float HeaderAngle = Heading;
	HeaderAngleError = calculateAngleDifference(Heading, HeaderAngle);
}

// Normalize angle to 0–360
float BNO055::normalizeAngle(float angle) {
	angle = fmod(angle, 360.0f);
	if (angle < 0) {
		angle += 360.0f;
	}
	return angle;
}

// Calculate signed shortest angle difference
float BNO055::calculateAngleDifference(float heading, float headerAngle) {
	heading = normalizeAngle(heading);
	headerAngle = normalizeAngle(headerAngle);
	float diff = heading - headerAngle;

	if (diff > 180.0f) {
		diff -= 360.0f;
	} else if (diff < -180.0f) {
		diff += 360.0f;
	}
	return diff;
}

// Public updateData method (calls internal reads if new data ready)
void BNO055::updateData() {
	readGyro();
	readCompass();
	/*
	 uint8_t calib;
	 readRegister(0x35, &calib, 1);

	 gyroCalibration = (calib >> 4) & 0x03;
	 */

}
