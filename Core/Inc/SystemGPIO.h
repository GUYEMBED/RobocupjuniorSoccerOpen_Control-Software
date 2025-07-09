#ifndef SYSTEM_GPIO_H
#define SYSTEM_GPIO_H

#include "stm32f4xx_hal.h"  // STM32 HAL header for STM32F4 series
#include "Timer.h"
#include "main.h"
#include <iostream>

void digitalWrite(const std::string &PIN, bool State);
bool digitalRead(const std::string &PIN);
void PWM_Write(const std::string &PIN, uint32_t timeWidth);

#endif // SYSTEM_GPIO_H
