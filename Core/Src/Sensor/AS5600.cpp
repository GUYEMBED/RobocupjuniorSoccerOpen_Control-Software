#include "AS5600.h"

AS5600::AS5600(I2C_HandleTypeDef *hi2c, float wheelCircumference) {
	this->hi2c = hi2c;
	this->wheel_circumference = wheelCircumference;
	lastTime = SystemTimer.getMicroseconds();
	lastAngle = 0;
	angularVelocity = 0.0f;
}

void AS5600::update() {
	float filteredHighPass = 0.0f;
	float prevAngularVelocity = 0.0f;
	const float alphaHP = 0.9f;  // High-pass smoothing factor (tweak between 0.8–0.99)
	uint8_t status = getStatus();
	if ((status & 0x20) == 0) {
		return; // No new data
	}

	uint8_t rxBuffer[2];
	uint8_t reg = AS5600_RAW_ANGLE_L;
	HAL_I2C_Mem_Read(hi2c, AS5600_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, rxBuffer, 2, HAL_MAX_DELAY);

	uint16_t rawAngle = ((uint16_t) rxBuffer[1] << 8) | rxBuffer[0];
	rawAngle = (rawAngle >> 4) & 0x0FFF;

	int16_t deltaRaw = rawAngle - lastAngle;
	if (deltaRaw > 2048) {
		deltaRaw -= 4096;
	} else if (deltaRaw < -2048) {
		deltaRaw += 4096;
	}

	float newVelocity = ((float) deltaRaw * 360.0f) / 4096.0f * (1000.0f / (float) 10);

	// High-pass filter
	filteredHighPass = alphaHP * (filteredHighPass + newVelocity - prevAngularVelocity);
	prevAngularVelocity = newVelocity;

	angularVelocity = newVelocity; // Optional: assign high-passed value

	lastAngle = rawAngle;
}

// Blocking (once) read of the status register
uint8_t AS5600::getStatus() {
	uint8_t status;
	HAL_I2C_Mem_Read(hi2c, AS5600_ADDR << 1, AS5600_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
	return status;
}
