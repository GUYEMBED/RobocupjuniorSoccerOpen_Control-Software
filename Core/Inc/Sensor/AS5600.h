#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "Timer.h"

#define AS5600_ADDR           0x36
#define AS5600_RAW_ANGLE_L    0x0C
#define AS5600_RAW_ANGLE_H    0x0D
#define AS5600_STATUS_REG     0x0B

class AS5600 {
public:
	AS5600(I2C_HandleTypeDef *hi2c, float wheelCircumference);

	void update();              // Read angle and compute velocity
	uint8_t getStatus();        // Read status register

	float getAngularVelocity() const {
		return angularVelocity;
	}
	float getSpeed() const {
		return (angularVelocity / 360.0f) * wheel_circumference;
	}

private:
	I2C_HandleTypeDef *hi2c;
	uint16_t lastAngle = 0;
	float angularVelocity = 0.0f;
	float wheel_circumference;
	uint32_t lastTime = 0;
};

extern AS5600 EncoderX;
extern AS5600 EncoderY;

#endif // INC_AS5600_H_
