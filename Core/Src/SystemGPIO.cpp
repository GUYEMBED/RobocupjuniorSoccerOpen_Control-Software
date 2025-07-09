#include "SystemGPIO.h"  // Include the header file

void digitalWrite(const std::string &PIN, bool State) {
	if (PIN == "PA4") {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PA5") {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PB15") {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PB12") {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PC13") {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PB13") {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PC14") {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PA0") {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PA1") {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PB6") {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (PIN == "PB2") {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

bool digitalRead(const std::string &PIN) {
	GPIO_PinState pinState;
	if (PIN == "PC0") {
		pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
	} else if (PIN == "PC1") {
		pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
	} else if (PIN == "PC2") {
		pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
	} else if (PIN == "PC3") {
		pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
	} else if (PIN == "PC13") {
		pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	} else if (PIN == "PC14") {
		pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
	} else if (PIN == "PC15") {
		pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
	} else if (PIN == "PA0") {
		pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	} else if (PIN == "PB0") {
		pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	} else if (PIN == "PB1") {
		pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	} else if (PIN == "PB14") {
		pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	} else if (PIN == "PA6") {
		pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
	} else if (PIN == "PB12") {
		pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	} else if (PIN == "PB15") {
		pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	} else if (PIN == "PB13") {
		pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	} else if (PIN == "PA0") {
		pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	}

	return pinState == GPIO_PIN_SET; // Return true if the pin is high, false if low
}

void PWM_Write(const std::string &PIN, uint32_t timeWidth) {
	TIM_HandleTypeDef *htim = nullptr;
	uint32_t channel;

	// Match pin to timer and channel
	if (PIN == "PA8") {
		//htim = &htim1;
		//channel = TIM_CHANNEL_1;
	} else if (PIN == "PA9") {
		//htim = &htim1;
		//channel = TIM_CHANNEL_2;
	} else if (PIN == "PA10") {
		//htim = &htim1;
		//channel = TIM_CHANNEL_3;
	} else if (PIN == "PA11") {
		//htim = &htim1;
		//channel = TIM_CHANNEL_4;
	} else if (PIN == "PA15") {
		htim = &htim2;
		channel = TIM_CHANNEL_1;
	} else if (PIN == "PB3") {
		htim = &htim2;
		channel = TIM_CHANNEL_2;
	} else if (PIN == "PB6") {
		htim = &htim4;
		channel = TIM_CHANNEL_1;
	} else if (PIN == "PB7") {
		htim = &htim4;
		channel = TIM_CHANNEL_2;
	} else {
		return; // Invalid pin
	}

	__HAL_TIM_SET_COMPARE(htim, channel, timeWidth);
	HAL_TIM_PWM_Start(htim, channel);

}

