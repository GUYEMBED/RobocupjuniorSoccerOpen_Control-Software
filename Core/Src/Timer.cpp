#include "Timer.h"

Timer::Timer() :
		overflowCount(0), milliseconds(0) {
}

void Timer::setup() {
	HAL_TIM_Base_Start_IT(&htim3);  // Enable timer with interrupt
}

void Timer::reset() {
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	overflowCount = 0;
}

// Optional, for HAL_GetTick() if you want ms
void Timer::update() {
	milliseconds = HAL_GetTick();
}

// Called from TIM3 IRQ Handler
void Timer::onOverflow() {
	overflowCount++;
}

uint32_t Timer::getMicroseconds() const {
	uint32_t high1, low, high2;

	do {
		high1 = overflowCount;
		low = __HAL_TIM_GET_COUNTER(&htim3);
		high2 = overflowCount;
	} while (high1 != high2);

	return (high1 << 16) | low;
}

uint32_t Timer::getMilliseconds(){
	return HAL_GetTick();
}
bool Timer::Flag(uint32_t period) {
	uint32_t currentTime = getMilliseconds();
	if (currentTime - lastTime >= period) {
		lastTime = currentTime;
		return true;
	}
	return false;
}
