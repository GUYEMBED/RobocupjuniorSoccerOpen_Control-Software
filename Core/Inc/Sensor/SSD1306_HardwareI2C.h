#ifndef SSD1306_HARDWARE_I2C_H
#define SSD1306_HARDWARE_I2C_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include <cstring>
#include <stdio.h>

class SSD1306_HardwareI2C {
public:
	SSD1306_HardwareI2C();
	void init();
	void clear();
	void setCursor(uint8_t x, uint8_t y);
	void writeChar(char c);
	void writeString(const char *str);
	void updateScreen();
	int intPow10(int power);
	void writeFloat(float value, uint8_t x, uint8_t y, uint8_t decimals);

	static const uint8_t Font5x7[39][5]; // 26 lowercase + 10 numbers + symbols

private:
	void writeCommand(uint8_t cmd);
	void writeData(const uint8_t *data, size_t size);

	uint8_t _address = 0x3C << 1;
	uint8_t _buffer[1024]; // 128x64 screen
	uint8_t _cursorX;
	uint8_t _cursorY;
};

extern SSD1306_HardwareI2C OLED;

#endif
