#ifndef ENCODER_H
#define ENCODER_H

#include <SystemGPIO.h>
#include <Timer.h>
#include "math.h"
#include "string.h"

class Encoder {
public:
	// Constructor
	Encoder(float Encoder_Circumference, float Encoder_PulsePerRevolution, uint32_t thresholdTimeforCompleteStop, const std::string &PinNumbersPhase1, const std::string &PinNumbersPhase2);

	// Returns direction: -1, 0, or 1
	int8_t checkDirection();

	// Returns speed (with direction) in units per second
	double getSpeed(uint32_t CompletestopThreshold);
	double getSpeed_IT();
	void EncoderISR(uint16_t TriggerPin, uint16_t interruptPin);

private:
	float EncoderCircumference;
	float EncoderPulsePerRevolution;
	uint32_t thresholdTime_forCompleteStop;

	std::string PinNum1;
	std::string PinNum2;

	int8_t lastDirection;

	// -----------Interrupt method-------------
	volatile uint32_t Last_time = 0;
	volatile uint32_t ElapseTime = 0;
	volatile int8_t DirectionCheck = 0;
	volatile uint32_t Last_time_ms = 0;
	double speed = 0;
};

extern Encoder EncoderMotor_TL;
extern Encoder EncoderMotor_TR;
extern Encoder EncoderMotor_DL;
extern Encoder EncoderMotor_DR;

#endif
