#ifndef INC_LSM6DSOX_H_
#define INC_LSM6DSOX_H_

#include "stm32f4xx_hal.h"  // Adjust if you're using a different STM32 series
#include <stdint.h>
#include "main.h"

// I2C device address (SA0 = GND → 0x6A << 1 = 0xD4)
#define LSM6DSOX_ADDRESS (0x6A << 1)

// Register addresses
#define LSM6DSOX_CTRL1_XL      0x10
#define LSM6DSOX_CTRL2_G       0x11
#define LSM6DSOX_OUTX_L_G      0x22
#define LSM6DSOX_OUTX_L_XL     0x28
#define LSM6DSOX_STATUS_REG    0x1E
#define LSM6DSOX_WHO_AM_I      0x0F
#define LSM6DSOX_WHO_AM_I_VAL  0x6C

class LSM6DSOX {
public:
	// Constructor now takes I2C handle as input
	LSM6DSOX(I2C_HandleTypeDef *handle);

	bool init();
	void updateData();
	void resetHeading();
	float HeaderAngleError = 0.0f;
	float getAngularVelocityX() const {
		return AngularVelocity_X;
	}
	float getAngularVelocityY() const {
		return AngularVelocity_Y;
	}
	float getAngularVelocityZ() const {
		return AngularVelocity_Z;
	}

	float getAccelerationX() const {
		return Acceleration_X;
	}
	float getAccelerationY() const {
		return Acceleration_Y;
	}
	float getAccelerationZ() const {
		return Acceleration_Z;
	}

	float getHeadingError() const {
		return HeaderAngleError;
	}

private:
	void writeRegister(uint8_t reg, uint8_t value);
	void readSensorData(uint8_t reg, uint8_t len);
	void processGyroData();
	void processAccelData();

	I2C_HandleTypeDef *i2cHandle;

	uint8_t rxBuffer[12];  // Buffer for gyro (6) + accel (6)

	float AngularVelocity_X = 0.0f;
	float AngularVelocity_Y = 0.0f;
	float AngularVelocity_Z = 0.0f;

	float Acceleration_X = 0.0f;
	float Acceleration_Y = 0.0f;
	float Acceleration_Z = 0.0f;
};


#endif /* INC_LSM6DSOX_H_ */
