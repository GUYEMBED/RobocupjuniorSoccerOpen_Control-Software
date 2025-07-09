#ifndef UARTCOMM_H
#define UARTCOMM_H

#include "stm32f4xx_hal.h"
#include <string>

#define TX_BUFFER_SIZE 128
#define RX_LINE_BUFFER_SIZE 128

class UartComm {
public:
	UartComm(UART_HandleTypeDef *huart);

	void send(const char *str);
	void receiveAndProcessLine();  // Blocking receive until newline

	// Parsed data
	bool ball_is_detected;
	bool ball_in_control;
	float ball_x, ball_y, ball_vx, ball_vy, ball_displacement, ball_angle;
	char rxLineBuffer[RX_LINE_BUFFER_SIZE];
	volatile bool lineReady = true;
	volatile uint16_t rxIndex = 0;
	volatile bool isTransmitting = true;
	void processReceivedData();
	void init();
	uint8_t c;
private:

	UART_HandleTypeDef *_huart;

	char txBuffer[TX_BUFFER_SIZE];
};

extern UartComm openMV_com;

#endif
