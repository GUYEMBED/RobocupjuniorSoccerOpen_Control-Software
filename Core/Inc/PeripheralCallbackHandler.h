#pragma once
#include "stm32f4xx_hal.h"
#include "main.h"

class PeripheralCallbackHandler {
public:
	static void UartRxCplt(UART_HandleTypeDef *huart);
	static void I2CMemRxCplt(I2C_HandleTypeDef *hi2c);
	static void I2CMasterTxCplt(I2C_HandleTypeDef *hi2c);
	static void I2CMasterRxCplt(I2C_HandleTypeDef *hi2c);
	static void UartTxCplt(UART_HandleTypeDef *huart);
};
