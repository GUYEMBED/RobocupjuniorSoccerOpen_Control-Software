#include "Encoder.h"

Encoder::Encoder(float Encoder_Circumference, float Encoder_PulsePerRevolution, uint32_t thresholdTimeforCompleteStop, const std::string &PinNumbersPhase1, const std::string &PinNumbersPhase2) :
		EncoderCircumference(Encoder_Circumference), EncoderPulsePerRevolution(Encoder_PulsePerRevolution), thresholdTime_forCompleteStop(thresholdTimeforCompleteStop), PinNum1(PinNumbersPhase1), PinNum2(
				PinNumbersPhase2), lastDirection(0) {
}

int8_t Encoder::checkDirection() {
	static uint8_t last_state = 0;
	static uint32_t last_state_change_time = 0;

	uint8_t current_state = (digitalRead(PinNum1) << 1) | digitalRead(PinNum2);
	int8_t direction = 0;

	if (current_state != last_state) {
		const int8_t transition_table[4][4] = { { 0, -1, 1, 0 }, { 1, 0, 0, -1 }, { -1, 0, 0, 1 }, { 0, 1, -1, 0 } };

		direction = transition_table[last_state][current_state];
		last_state = current_state;
		last_state_change_time = SystemTimer.getMilliseconds();
		lastDirection = direction;  // Cache the latest direction
	}

	return lastDirection;
}

double Encoder::getSpeed(uint32_t CompletestopThreshold) {
	static bool previous_pin1_state = false;
	static uint16_t last_rising_edge_time_us = 0;
	static uint32_t last_update_time_ms = 0;
	static double speed = 0;

	uint8_t current_state = (digitalRead(PinNum1) << 1) | digitalRead(PinNum2);
	bool current_pin1_state = (current_state & 0x02) != 0;

	uint16_t now_us = SystemTimer.getMicroseconds();
	uint32_t now_ms = SystemTimer.getMilliseconds();

	if (!previous_pin1_state && current_pin1_state) {
		uint16_t delta_time = (uint16_t) (now_us - last_rising_edge_time_us);
		last_rising_edge_time_us = now_us;

		if (delta_time > 0) {
			speed = (1.0 / (delta_time * EncoderPulsePerRevolution)) * EncoderCircumference * 1e6;
			last_update_time_ms = now_ms;
		}
	}
	previous_pin1_state = current_pin1_state;

	if ((now_ms - last_update_time_ms) > thresholdTime_forCompleteStop) {
		speed = 0;
	}

	checkDirection();  // Updates lastDirection if needed
	return speed * lastDirection;
}

void Encoder::EncoderISR(uint16_t TriggerPin, uint16_t interruptPin) {
	if (TriggerPin == interruptPin) {
		uint32_t us_timer = SystemTimer.getMicroseconds();

		ElapseTime = (uint32_t) (us_timer - Last_time);
		if (ElapseTime == 0) {
			ElapseTime = 1;
		}
		Last_time = us_timer;
		Last_time_ms = SystemTimer.getMilliseconds();

		if (digitalRead(PinNum2) == true) {
			DirectionCheck = 1;
		} else {
			DirectionCheck = -1; //  recently -1
		}
	}
}

double Encoder::getSpeed_IT() {
	if (SystemTimer.getMilliseconds() - Last_time_ms >= thresholdTime_forCompleteStop) {
		DirectionCheck = 0;
	}
	if ((ElapseTime * EncoderPulsePerRevolution) == 0) {
		ElapseTime = 1;
	}
	speed = ((1.0 / (ElapseTime * EncoderPulsePerRevolution)) * EncoderCircumference * 1e6) * DirectionCheck;
	return speed;
	//return ((1 / EncoderPulsePerRevolution) / ElapseTime) * 1e6 * 60;

}

