#include "PIDController.h"
#include <math.h>

PIDController::PIDController() {
	//update(target, current, threshold, kp, kd, ki, period_ms);
}

void PIDController::update(float target, float current, float threshold, float kp, float kd, float ki, uint32_t period_ms) {
	float error = target - current;

	if (period_ms == 0) {
		period_ms = 50;
	}

	if (SystemTimer.getMilliseconds() - updatetime >= period_ms) {
		previousError = error;
		updatetime = SystemTimer.getMilliseconds();
	}

	if (abs(error) >= threshold) {
		Adjustment_output = (error * kp) + (kd * (error - previousError) / float(period_ms));
	} else {
		Adjustment_output = 0;
	}
}
