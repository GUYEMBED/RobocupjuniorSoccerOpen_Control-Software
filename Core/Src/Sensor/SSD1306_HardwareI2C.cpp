#include "SSD1306_HardwareI2C.h"

SSD1306_HardwareI2C::SSD1306_HardwareI2C() :
		_cursorX(0), _cursorY(0) {
	memset(_buffer, 0, sizeof(_buffer));
}

void SSD1306_HardwareI2C::init() {
	HAL_Delay(100);
	writeCommand(0xAE); // Display off
	writeCommand(0x20); // Set memory addressing mode
	writeCommand(0x10); // Page addressing mode
	writeCommand(0xB0); // Set page start
	writeCommand(0xC8); // COM scan direction
	writeCommand(0x00); // Low col
	writeCommand(0x10); // High col
	writeCommand(0x40); // Start line address
	writeCommand(0x81); // Contrast
	writeCommand(0xFF);
	writeCommand(0xA1); // Segment remap
	writeCommand(0xA6); // Normal display
	writeCommand(0xA8); // Multiplex ratio
	writeCommand(0x3F);
	writeCommand(0xA4); // Output RAM to display
	writeCommand(0xD3); // Display offset
	writeCommand(0x00);
	writeCommand(0xD5); // Clock divide
	writeCommand(0xF0);
	writeCommand(0xD9); // Precharge
	writeCommand(0x22);
	writeCommand(0xDA); // COM pins
	writeCommand(0x12);
	writeCommand(0xDB); // VCOM detect
	writeCommand(0x20);
	writeCommand(0x8D); // Charge pump
	writeCommand(0x14);
	writeCommand(0xAF); // Display on
	clear();
}

void SSD1306_HardwareI2C::clear() {
	memset(_buffer, 0, sizeof(_buffer));
	updateScreen();
}

void SSD1306_HardwareI2C::setCursor(uint8_t x, uint8_t y) {
	_cursorX = x;
	_cursorY = y;
}

void SSD1306_HardwareI2C::writeChar(char c) {
	if (c >= 'a' && c <= 'z') {
		const uint8_t *font = Font5x7[c - 'a'];
		for (int i = 0; i < 5; ++i)
			_buffer[_cursorX++ + (_cursorY * 128)] = font[i];
		_buffer[_cursorX++ + (_cursorY * 128)] = 0x00;
	} else if (c >= '0' && c <= '9') {
		const uint8_t *font = Font5x7[26 + (c - '0')];
		for (int i = 0; i < 5; ++i)
			_buffer[_cursorX++ + (_cursorY * 128)] = font[i];
		_buffer[_cursorX++ + (_cursorY * 128)] = 0x00;
	} else if (c == '.') {
		const uint8_t *font = Font5x7[36];
		for (int i = 0; i < 5; ++i)
			_buffer[_cursorX++ + (_cursorY * 128)] = font[i];
		_buffer[_cursorX++ + (_cursorY * 128)] = 0x00;
	} else if (c == ',') {
		const uint8_t *font = Font5x7[37];
		for (int i = 0; i < 5; ++i)
			_buffer[_cursorX++ + (_cursorY * 128)] = font[i];
		_buffer[_cursorX++ + (_cursorY * 128)] = 0x00;
	} else if (c == '-') {
		const uint8_t *font = Font5x7[38];
		for (int i = 0; i < 5; ++i)
			_buffer[_cursorX++ + (_cursorY * 128)] = font[i];
		_buffer[_cursorX++ + (_cursorY * 128)] = 0x00;
	} else if (c == ' ') {
		_cursorX += 6; // blank space
	}
}

void SSD1306_HardwareI2C::writeString(const char *str) {
	while (*str) {
		writeChar(*str++);
	}
}

void SSD1306_HardwareI2C::updateScreen() {
	for (uint8_t page = 0; page < 8; page++) {
		writeCommand(0xB0 + page);
		writeCommand(0x00);
		writeCommand(0x10);
		writeData(&_buffer[page * 128], 128);
	}
}

void SSD1306_HardwareI2C::writeCommand(uint8_t cmd) {
	uint8_t data[2] = { 0x00, cmd };
	if (HAL_I2C_Master_Transmit(&hi2c1, _address, data, 2, HAL_MAX_DELAY) != HAL_OK) {
		// Handle error here if needed
	}
}

void SSD1306_HardwareI2C::writeData(const uint8_t *data, size_t size) {
	static uint8_t temp[129];
	temp[0] = 0x40;
	memcpy(&temp[1], data, size);

	if (HAL_I2C_Master_Transmit(&hi2c1, _address, temp, size + 1, HAL_MAX_DELAY) != HAL_OK) {
		// Handle error here if needed
	}
}

int SSD1306_HardwareI2C::intPow10(int power) {
	int result = 1;
	while (power-- > 0) {
		result *= 10;
	}
	return result;
}

void SSD1306_HardwareI2C::writeFloat(float value, uint8_t x, uint8_t y, uint8_t decimals) {
	setCursor(x, y);

	int multiplier = intPow10(decimals);
	int intPart = (int) value;
	int fracPart = (int) ((value - intPart) * multiplier);

	if (fracPart < 0)
		fracPart = -fracPart; // handle negative floats

	char buffer[16];
	snprintf(buffer, sizeof(buffer), "%d.%0*d", intPart, decimals, fracPart);

	writeString(buffer);
}

// Font5x7 remains unchanged...

// Font5x7: 'a'-'z' (26) + '0'-'9' (10) + '.' (1) + ',' (1)
const uint8_t SSD1306_HardwareI2C::Font5x7[39][5] = {
// a-z
		{ 0x20, 0x54, 0x54, 0x54, 0x78 }, // a
		{ 0x7F, 0x48, 0x44, 0x44, 0x38 }, // b
		{ 0x38, 0x44, 0x44, 0x44, 0x20 }, // c
		{ 0x38, 0x44, 0x44, 0x48, 0x7F }, // d
		{ 0x38, 0x54, 0x54, 0x54, 0x18 }, // e
		{ 0x08, 0x7E, 0x09, 0x01, 0x02 }, // f
		{ 0x0C, 0x52, 0x52, 0x52, 0x3E }, // g
		{ 0x7F, 0x08, 0x04, 0x04, 0x78 }, // h
		{ 0x00, 0x44, 0x7D, 0x40, 0x00 }, // i
		{ 0x20, 0x40, 0x44, 0x3D, 0x00 }, // j
		{ 0x7F, 0x10, 0x28, 0x44, 0x00 }, // k
		{ 0x00, 0x41, 0x7F, 0x40, 0x00 }, // l
		{ 0x7C, 0x04, 0x18, 0x04, 0x78 }, // m
		{ 0x7C, 0x08, 0x04, 0x04, 0x78 }, // n
		{ 0x38, 0x44, 0x44, 0x44, 0x38 }, // o
		{ 0x7C, 0x14, 0x14, 0x14, 0x08 }, // p
		{ 0x08, 0x14, 0x14, 0x18, 0x7C }, // q
		{ 0x7C, 0x08, 0x04, 0x04, 0x08 }, // r
		{ 0x48, 0x54, 0x54, 0x54, 0x20 }, // s
		{ 0x04, 0x3F, 0x44, 0x40, 0x20 }, // t
		{ 0x3C, 0x40, 0x40, 0x20, 0x7C }, // u
		{ 0x1C, 0x20, 0x40, 0x20, 0x1C }, // v
		{ 0x3C, 0x40, 0x30, 0x40, 0x3C }, // w
		{ 0x44, 0x28, 0x10, 0x28, 0x44 }, // x
		{ 0x0C, 0x50, 0x50, 0x50, 0x3C }, // y
		{ 0x44, 0x64, 0x54, 0x4C, 0x44 }, // z
		// 0-9
		{ 0x3E, 0x51, 0x49, 0x45, 0x3E }, // 0
		{ 0x00, 0x42, 0x7F, 0x40, 0x00 }, // 1
		{ 0x42, 0x61, 0x51, 0x49, 0x46 }, // 2
		{ 0x21, 0x41, 0x45, 0x4B, 0x31 }, // 3
		{ 0x18, 0x14, 0x12, 0x7F, 0x10 }, // 4
		{ 0x27, 0x45, 0x45, 0x45, 0x39 }, // 5
		{ 0x3C, 0x4A, 0x49, 0x49, 0x30 }, // 6
		{ 0x01, 0x71, 0x09, 0x05, 0x03 }, // 7
		{ 0x36, 0x49, 0x49, 0x49, 0x36 }, // 8
		{ 0x06, 0x49, 0x49, 0x29, 0x1E }, // 9
		// Extra symbols
		{ 0x00, 0x60, 0x60, 0x00, 0x00 }, // . (dot)
		{ 0x00, 0x80, 0x60, 0x00, 0x00 },  // , (comma)
		{ 0x00, 0x08, 0x08, 0x08, 0x00 }  // - (minus sign)
};

