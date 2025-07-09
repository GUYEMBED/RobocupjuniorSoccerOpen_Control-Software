#ifndef PIDController_h
#define PIDController_h

#include <Timer.h>

class PIDController {
public:
	float Adjustment_output = 0;
	PIDController();
	void update(float target, float current, float threshold, float kp, float kd, float ki, uint32_t period_ms);

private:
	float previousError = 0;
	uint32_t updatetime = 0;
};

#endif
