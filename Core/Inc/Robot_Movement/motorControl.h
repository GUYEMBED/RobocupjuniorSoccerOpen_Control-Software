#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "PIDController.h"
#include "SystemGPIO.h"
#include <string>
#include "iostream"
#include <math.h>
#include "Timer.h"

#define MAX_INPUT 501
extern int16_t sigmoidArray[500];

class motorControl {
private:
	std::string in1pin;
	std::string in2pin;
	float kp_pos;
	float ki_pos;
	float kd_pos;
	int periodMS_pos;
	float threshold_pos;
	float kp_neg;
	float ki_neg;
	float kd_neg;
	int periodMS_neg;
	float threshold_neg;
	bool signbit(float x);
	bool preCalculationState = true;
	float addedSpeed = 0;
	float addedSpeed_pos = 0;
	float addedSpeed_neg = 0;
	bool startmotorstate = true;

	void preCalculateSigmoidArray();
	int16_t getSigmoidOutput(int input);

	PIDController Motor_PIDcontrol_positive;
	PIDController Motor_PIDcontrol_negative;
	PIDController AddedSpeed_PIDcontrol_positive;
	PIDController AddedSpeed_PIDcontrol_negative;

	uint32_t updateTime = 0;
	float integral_pos = 0;
	float integral_neg = 0;
public:
	//float totalSpeed = 0;
	motorControl();
	void spin(int16_t speed, float PosOFFset, float NegOFFset);
	void drive(float speed, float currentSpeed, float posaddgain, float negaddgain);
	void Setup(std::string in1, std::string in2, float p_pos, float i_pos, float d_pos, int period_msPos, float thresholdPos, float current_speed, float p_neg, float i_neg, float d_neg,
			int period_msNeg, float thresholdNeg);
	float totalSpeed = 0;
	float currentSpeed = 0;
	float posoffset = 0;
	float negoffset = 0;
};

#endif
