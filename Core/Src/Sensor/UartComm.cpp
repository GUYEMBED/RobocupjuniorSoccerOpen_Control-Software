#include "UartComm.h"
#include <cstring>
#include <cstdlib>

UartComm::UartComm(UART_HandleTypeDef *huart) {
	_huart = huart;

	memset(txBuffer, 0, TX_BUFFER_SIZE);
	memset(rxLineBuffer, 0, RX_LINE_BUFFER_SIZE);

	ball_is_detected = false;
	ball_x = ball_y = ball_vx = ball_vy = ball_displacement = ball_angle = 0.0f;
}

void UartComm::send(const char *str) {
	if (isTransmitting)
		return;

	size_t len = strlen(str);
	if (len >= TX_BUFFER_SIZE)
		len = TX_BUFFER_SIZE - 1;

	strncpy(txBuffer, str, len);
	txBuffer[len] = '\0';

	isTransmitting = true;
	HAL_UART_Transmit_IT(_huart, (uint8_t*) txBuffer, len);
}

void UartComm::receiveAndProcessLine() {
	HAL_UART_Receive_IT(_huart, &c, 1);  // Re-enable interrupt for next byte

	if (c == '\n') {
		rxLineBuffer[rxIndex] = '\0';  // Null terminate
		lineReady = true;
		rxIndex = 0;
	} else if (rxIndex < RX_LINE_BUFFER_SIZE - 1) {
		rxLineBuffer[rxIndex++] = c;
	} else {
		rxIndex = 0;  // Buffer overflow, reset index or handle error
	}
}
void UartComm::init() {
	HAL_UART_Receive_IT(_huart, &c, 1);
}
void UartComm::processReceivedData() {
	// Temporary buffer for each token
	if (!lineReady) {
		return;
	}

	char token[16];
	int token_index = 0;
	int field_index = 0;

	// Storage for parsed values
	bool ball_detected_flag = false;
	bool ball_in_control_flag = false;
	float values[6] = { 0 };

	for (uint16_t i = 0; i <= strlen(rxLineBuffer); i++) {
		char c = rxLineBuffer[i];

		if (c == ',' || c == '\0') {
			token[token_index] = '\0';  // Null-terminate the token

			switch (field_index) {
			case 0:
				ball_detected_flag = (strcmp(token, "true") == 0);
				break;
			case 1:
				ball_in_control_flag = (strcmp(token, "true") == 0);
				break;
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
				values[field_index - 2] = atof(token);
				break;
			default:
				break;
			}

			token_index = 0;
			field_index++;
		} else {
			if (token_index < sizeof(token) - 1) {
				token[token_index++] = c;
			}
		}
	}

	// Store parsed results
	ball_is_detected = ball_detected_flag;
	ball_in_control = ball_in_control_flag;

	if (ball_is_detected) {
		ball_x = values[0];
		ball_y = values[1];
		ball_vx = values[2];
		ball_vy = values[3];
		ball_displacement = values[4];
		ball_angle = values[5];
	}
}


