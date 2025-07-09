#ifndef BNO055_H
#define BNO055_H

#include "stm32f4xx_hal.h"
#include <cmath>
#include "main.h"

class BNO055 {
public:
	BNO055();

	void init();
	void updateData();  // Only updates if new data is available

	float AngularVelocity_Xaxis = 0;
	float AngularVelocity_Yaxis = 0;
	float AngularVelocity_Zaxis = 0;

	float Heading = 0;
	float Pitch = 0;
	float Roll = 0;
	float HeaderAngleError = 0;
	uint8_t gyroCalibration = 0;

private:
	void writeRegister(uint8_t reg, uint8_t value);
	void readRegister(uint8_t reg, uint8_t *buffer, uint8_t len);
	float calculateAngleDifference(float heading, float headerAngle);
	float normalizeAngle(float angle);

	void readGyro();
	void readCompass();
	bool isDataReady();  // New method to check INT_STA

	// I2C Address
	static constexpr uint8_t BNO055_ADDRESS = 0x29 << 1; // 0x29

	// Register addresses
	static constexpr uint8_t BNO055_OPR_MODE = 0x3D;
	static constexpr uint8_t BNO055_PWR_MODE = 0x3E;
	static constexpr uint8_t BNO055_UNIT_SEL = 0x3B;
	static constexpr uint8_t BNO055_GYR_DATA_X_LSB = 0x14;
	static constexpr uint8_t BNO055_EUL_DATA_X_LSB = 0x1A;
	static constexpr uint8_t BNO055_INT_STA = 0x37;  // NEW: Interrupt status register

	// Operation modes
	static constexpr uint8_t BNO055_CONFIG_MODE = 0x00;
	static constexpr uint8_t BNO055_NDOF_MODE = 0x0C;
};

extern BNO055 IMU;
#endif
