#ifndef SSD1306_SOFTWARE_I2C_H
#define SSD1306_SOFTWARE_I2C_H

#include "main.h"  // For GPIO port/pin definitions and HAL functions

class SSD1306_SoftwareI2C {
public:
	SSD1306_SoftwareI2C(GPIO_TypeDef *sclPort, uint16_t sclPin,
			GPIO_TypeDef *sdaPort, uint16_t sdaPin);

	void init();
	void clearBuffer();
	void displayBuffer();
	void drawText(uint8_t x, uint8_t y, const char *str);

private:
	GPIO_TypeDef *_sclPort;
	uint16_t _sclPin;
	GPIO_TypeDef *_sdaPort;
	uint16_t _sdaPin;

	static const uint8_t Font5x7[][5];

	uint8_t buffer[128 * 8] = { 0 };

	// Low-level software I2C
	void i2cDelay();
	void i2cStart();
	void i2cStop();
	bool i2cWriteByte(uint8_t byte);

	// SSD1306 low-level commands
	void sendCommand(uint8_t cmd);
	void sendData(uint8_t *data, uint16_t size);

	// Graphics
	void setPixel(uint8_t x, uint8_t y, bool on);
	void drawChar(uint8_t x, uint8_t y, char c);
};

#endif
