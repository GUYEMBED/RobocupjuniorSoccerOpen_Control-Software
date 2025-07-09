#ifndef TIMER_H
#define TIMER_H

#include "stm32f4xx_hal.h"
#include "main.h"

class Timer {
public:
	Timer();
	void setup();
	void update();         // Optionally still keep this for ms
	void reset();

	uint32_t getMicroseconds() const;  // Now returns 32-bit
	uint32_t getMilliseconds();

	void onOverflow();  // To be called in TIM3 IRQ handler
	bool Flag(uint32_t period_ms);
	uint32_t milliseconds;
private:
	uint32_t lastTime = 0;
	volatile uint32_t overflowCount = 0;  // Count of TIM3 overflows
};

extern Timer SystemTimer;

#endif
