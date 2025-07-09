#include "motorControl.h"
#include "Timer.h"
int16_t sigmoidArray[500];

motorControl::motorControl() {
	//preCalculateSigmoidArray();
}

void motorControl::preCalculateSigmoidArray() {
	constexpr float scale = 210.0f; // scaling factor to convert output to desired range in this case 360 is the largest that would still optimal
	constexpr float k = 0.0008f;   // steepness of main ramp up (rate of speed boost)
	constexpr float m = 0.6f;   // steepness of decreasing rate (suppression rate)
	constexpr float i0 = 480;     // main ramp up midpoint (speed boost at 250)
	constexpr float i1 = 360;     // decreasing rate cutoff (suppression effect decreases after 200)
	//constexpr float e = 2.718281828459045f;

	for (int i = 0; i <= MAX_INPUT; i++) {
		float S1 = 1.0f / (1.0f + expf(-k * (i - i0)));
		float S2 = 1.0f / (1.0f + expf(m * (i1 - i)));
		float output = scale * S1 * S2;

		sigmoidArray[i] = (int16_t) (output + 0.5f);
	}
}

int16_t motorControl::getSigmoidOutput(int input) {
	int absInput = abs(input);
	if (absInput > MAX_INPUT)
		absInput = MAX_INPUT;

	return sigmoidArray[absInput] * (input < 0 ? -1 : 1);
	//return sigmoidArray[absInput];
}

bool motorControl::signbit(float x) {
	return x < 0;
}
void motorControl::Setup(std::string in1, std::string in2, float p_pos, float i_pos, float d_pos, int period_msPos, float thresholdPos, float current_speed, float p_neg, float i_neg, float d_neg,
		int period_msNeg, float thresholdNeg) {
	in1pin = in1;
	in2pin = in2;
	kp_pos = p_pos;
	ki_pos = i_pos;
	kd_pos = d_pos;
	periodMS_pos = period_msPos;
	threshold_pos = thresholdPos;
	currentSpeed = current_speed;
	kp_neg = p_neg;
	ki_neg = i_neg;
	kd_neg = d_neg;
	periodMS_neg = period_msNeg;
	threshold_neg = thresholdNeg;
}
void motorControl::spin(int16_t speed, float PosOFFset, float NegOFFset) {
	float totalSpeed = 0;
	posoffset = PosOFFset;
	negoffset = NegOFFset;
//	if (abs(speed) > 500) {
//		speed = 500 * abs(speed) / speed;
//	}
	// approrpriate PosOFFset = 185, // NegOffset = 105
	if (speed > 0) {
		totalSpeed = 1500 + (speed + PosOFFset); // 186, 160
		//PWM_Write(in1pin, 1500 + (speed + 155));
	} else if (speed < 0) {
		totalSpeed = 1500 + (speed - NegOFFset); // 114, 105
		//PWM_Write(in1pin, 1500 + (speed - 95));
	} else {
		totalSpeed = 1500;
		//PWM_Write(in1pin, 1500);
	}
	/*
	 if (totalSpeed > 2000) {
	 totalSpeed = 2000;
	 }
	 if (totalSpeed < 1000) {
	 totalSpeed = 1000;
	 }*/
	PWM_Write(in1pin, totalSpeed);
}
void motorControl::drive(float speed, float currentSpeed, float posaddgain, float negaddgain) {
	Motor_PIDcontrol_positive.update(speed, currentSpeed, threshold_pos, kp_pos, kd_pos, ki_pos, periodMS_pos);
	Motor_PIDcontrol_negative.update(speed, currentSpeed, threshold_neg, kp_neg, kd_neg, ki_neg, periodMS_neg);

	AddedSpeed_PIDcontrol_positive.update(speed, currentSpeed, threshold_pos, posaddgain, 0.32, 0.0f, 10); // 0.23
	AddedSpeed_PIDcontrol_negative.update(speed, currentSpeed, threshold_neg, negaddgain, 0.41, 0.0f, 10); // 0.28

	integral_pos += Motor_PIDcontrol_positive.Adjustment_output;
	integral_neg += Motor_PIDcontrol_negative.Adjustment_output;

	addedSpeed_pos = abs(AddedSpeed_PIDcontrol_positive.Adjustment_output); // (speed - currentSpeed) * posaddgain; // 0.75
	addedSpeed_neg = abs(AddedSpeed_PIDcontrol_negative.Adjustment_output); // (speed - currentSpeed) * negaddgain; // 0.58

	float MaxAddedSpeed_pos = 167;
	float MaxAddedSpeed_neg = 107;

	if (addedSpeed_pos > MaxAddedSpeed_pos) {
		addedSpeed_pos = MaxAddedSpeed_pos;
	}
	if (addedSpeed_neg > MaxAddedSpeed_neg) {
		addedSpeed_neg = MaxAddedSpeed_neg;
	}

	float unmoveSpeed = 10;
	float startMotorSpeed_dif = 10;

	bool startmotorCheck = (abs(currentSpeed) < unmoveSpeed) && (abs(currentSpeed - speed) > startMotorSpeed_dif);
	bool DirectionChangeCheck = (speed * currentSpeed) < 0; // check if the direction of the speed has changed

	if (DirectionChangeCheck) {
		if (speed > 0) {
			totalSpeed = (int16_t) addedSpeed_pos;
		} else if (speed < 0) {
			totalSpeed = (int16_t) -addedSpeed_neg;
		}
	}
	if (startmotorCheck && startmotorstate) {
		startmotorstate = false;
		if (speed > 0) {
			totalSpeed = (int16_t) addedSpeed_pos;
		} else if (speed < 0) {
			totalSpeed = (int16_t) -addedSpeed_neg;
		}
	} else {
		startmotorstate = true;
	}

	if (speed >= 0) {
		totalSpeed = integral_pos; // + getSigmoidOutput(speed - currentSpeed);
	} else {
		totalSpeed = integral_neg; // + getSigmoidOutput(speed - currentSpeed);
	}

	if (totalSpeed > 500) {
		totalSpeed = 500;
	} else if (totalSpeed < -500) {
		totalSpeed = -500;
	}
	spin((int16_t) totalSpeed, posoffset, negoffset); // + getSigmoidOutput(speed - currentSpeed)

}
/*
 void motorControl::drive(float speed, float currentSpeed) {
 const uint16_t AssistUpdatePeriod_ms = 100;
 const uint16_t AssistThreshold = 250;

 Motor_PIDcontrol_positive.update(speed, currentSpeed, threshold_pos, kp_pos, kd_pos, ki_pos, periodMS_pos);
 Motor_PIDcontrol_negative.update(speed, currentSpeed, threshold_neg, kp_neg, kd_neg, ki_neg, periodMS_neg);

 if ((SystemTimer.milliseconds - updateTime >= AssistUpdatePeriod_ms) && (abs(speed - currentSpeed) >= AssistThreshold)) {
 addedSpeed_pos = (speed - currentSpeed) * 0.08;
 addedSpeed_neg = (speed - currentSpeed) * 0.058;
 updateTime = SystemTimer.milliseconds;
 } else {
 addedSpeed_pos = 0;
 addedSpeed_neg = 0;
 }

 if (abs(addedSpeed_pos) > 190) {
 addedSpeed_pos = 190 * (addedSpeed_pos < 0 ? -1 : 1);
 }
 if (abs(addedSpeed_neg) > 130) {
 addedSpeed_neg = 130 * (addedSpeed_neg < 0 ? -1 : 1);
 }
 if (speed >= 0) {
 totalSpeed += (Motor_PIDcontrol_positive.Adjustment_output) + (int16_t) addedSpeed_pos;
 } else {
 totalSpeed += (Motor_PIDcontrol_negative.Adjustment_output) + (int16_t) addedSpeed_neg;
 }

 if (totalSpeed > 500) {
 totalSpeed = 500;
 } else if (totalSpeed < -500) {
 totalSpeed = -500;
 }

 spin((int16_t) totalSpeed, posoffset, negoffset); // + getSigmoidOutput(speed - currentSpeed)
 }
 */
