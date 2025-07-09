#include "UartComm.h"
#include "PeripheralCallbackHandler.h"
#include "LidarReader.h"
#include "AS5600.h"
#include "LSM6DSOX.h"
#include "SSD1306_HardwareI2C.h"

void PeripheralCallbackHandler::UartRxCplt(UART_HandleTypeDef *huart) {

	if (huart == &huart2) {
		lidar.onUartReceiveComplete();
	} else if (huart == &huart6) {
		openMV_com.receiveAndProcessLine();
		openMV_com.isTransmitting = false;  // Reset transmission flag
	} else if (huart == &huart1) {
		//  bluetooth_com.processReceivedData();  // Or set a flag to process later
		//	HAL_UART_Receive_IT(huart, (uint8_t*) bluetooth_com.rxBuffer, RX_BUFFER_SIZE); // Re-enable reception
	}

}

void PeripheralCallbackHandler::I2CMemRxCplt(I2C_HandleTypeDef *hi2c) {
}

void PeripheralCallbackHandler::I2CMasterTxCplt(I2C_HandleTypeDef *hi2c) {
}

void PeripheralCallbackHandler::I2CMasterRxCplt(I2C_HandleTypeDef *hi2c) {
}

void PeripheralCallbackHandler::UartTxCplt(UART_HandleTypeDef *huart) {
	if (huart == &huart6) {
		//	openMV_com.isTransmitting = false;  // Reset transmission flag
	} else if (huart == &huart1) {
		//	bluetooth_com.isTransmitting = false;  // Reset transmission flag
	}
}
