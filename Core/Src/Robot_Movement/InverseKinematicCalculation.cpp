#include "InverseKinematicCalculation.h"

InverseKinematicCalculation::InverseKinematicCalculation() {
	RobotRadius = KinematicState.robotRadius;
	MaxSpeed = KinematicState.maxSpeed;
}

float InverseKinematicCalculation::HigherValueOF(float value1, float value2) {
	if (abs(value1) > abs(value2)) {
		return value1;
	} else if (abs(value1) < abs(value2)) {
		return value2;
	}
	return value1;
}

float InverseKinematicCalculation::Imb_Vector_LowerWHspd(float Imb_Vector_Velocity, float Angular_Velocity) {
	return abs(Imb_Vector_Velocity) - (RobotRadius * 2 * PI * abs(Angular_Velocity));
}

char InverseKinematicCalculation::Quadant(float Vector_Angle) {
	if (Vector_Angle > -45 && Vector_Angle < 45) {
		return 'F';
	} else if ((Vector_Angle > 135 && Vector_Angle < 180) || (Vector_Angle < -135 && Vector_Angle > -180)) {

		return 'B';
	} else if (Vector_Angle < -45 && Vector_Angle > -135) {
		return 'L';
	} else if (Vector_Angle > 45 && Vector_Angle < 135) {
		return 'R';
	}
	return 'E';
}

void InverseKinematicCalculation::CalculateVector1_Vector2(float Xspeed, float Yspeed) {
	if (quadant == 'F' || quadant == 'B') {
		vector1 = abs(Yspeed) / cos45;
		vector2 = abs(Xspeed) + (vector1 * cos45);
	} else if (quadant == 'L' || quadant == 'R') {
		vector1 = abs(Xspeed) / cos45;
		vector2 = abs(Yspeed) + (vector1 * cos45);
	}
}

void InverseKinematicCalculation::Apply_wheelDirection() {
	if (quadant == 'F') {
		TR = abs(TR);
		TL = abs(TL);
		DR = abs(DR);
		DL = abs(DL);
	} else if (quadant == 'L') {
		TR = abs(TR);
		TL = -TL;
		DR = -DR;
		DL = abs(DL);
	} else if (quadant == 'R') {
		TR = -TR;
		TL = abs(TL);
		DR = abs(DR);
		DL = -DL;
	} else if (quadant == 'B') {
		TR = -TR;
		TL = -TL;
		DR = -DR;
		DL = -DL;
	}
}

void InverseKinematicCalculation::Apply_wheelSpeed(float ImbVector, float balancedVector, bool Imb_IsHigher, float Xspeed, float Yspeed, float Angular_Velocity, bool Is_Clockwise) {
	float LowerWHspeed = Imb_Vector_LowerWHspd(ImbVector, Angular_Velocity);

	if (quadant == 'F' || quadant == 'L' || quadant == 'R') {
		if ((Imb_IsHigher && (Xspeed > 0 || Yspeed != 0)) || (!Imb_IsHigher && (Xspeed < 0 || Yspeed != 0))) {
			DR = LowerWHspeed;
			TL = DR + ImbVector_SpeedDiff;
			if ((quadant == 'F' && !Is_Clockwise) || (quadant == 'L' && Is_Clockwise) || (quadant == 'R' && !Is_Clockwise)) {
				TL = LowerWHspeed;
				DR = TL + ImbVector_SpeedDiff;
			}

			TR = balancedVector;
			DL = balancedVector;
		} else {
			TR = LowerWHspeed;
			DL = TR + ImbVector_SpeedDiff;
			if ((quadant == 'F' && !Is_Clockwise) || (quadant == 'L' && !Is_Clockwise) || (quadant == 'R' && Is_Clockwise)) {
				DL = LowerWHspeed;
				TR = DL + ImbVector_SpeedDiff;
			}

			TL = balancedVector;
			DR = balancedVector;
		}
	} else {
		if ((Imb_IsHigher && Xspeed > 0) || (!Imb_IsHigher && Xspeed < 0)) {
			DL = LowerWHspeed;
			TR = DL + ImbVector_SpeedDiff;
			if (!Is_Clockwise) {
				TR = LowerWHspeed;
				DL = TR + ImbVector_SpeedDiff;
			}

			TL = balancedVector;
			TR = balancedVector;
		} else {
			TL = LowerWHspeed;
			DR = TL + ImbVector_SpeedDiff;
			if (!Is_Clockwise) {
				DR = LowerWHspeed;
				TL = DR + ImbVector_SpeedDiff;
			}

			TR = balancedVector;
			DL = balancedVector;
		}
	}
	float Vector_Velocity = sqrt((Xspeed * Xspeed) + (Yspeed * Yspeed));
	if (VectorAngle == -(PI / 4)) {
		DL = Vector_Velocity;
		TR = Vector_Velocity;

		DR = 0;
		TL = 0;
	} else if (VectorAngle == PI / 4) {
		DR = Vector_Velocity;
		TL = Vector_Velocity;

		DL = 0;
		TR = 0;
	} else if (VectorAngle == PI - (PI / 4)) {
		DL = -Vector_Velocity;
		TR = -Vector_Velocity;

		DR = 0;
		TL = 0;
	} else if (VectorAngle == -PI + (PI / 4)) {
		DR = -Vector_Velocity;
		TL = -Vector_Velocity;

		DL = 0;
		TR = 0;
	}

	Apply_wheelDirection();
}

void InverseKinematicCalculation::CalculateWheelSpeed(float Xspeed, float Yspeed, float Angular_Velocity) {

	const float cos45 = sqrt(2) / 2.0f;
	const float Deg_Rad_Ratio = 57.2957795131;
	float AngularVelocity_Rad = Angular_Velocity / Deg_Rad_Ratio;
	float Rotational_LinearSpeed = AngularVelocity_Rad * 90; // RobotRadius = 90;

	TL = ((cos45 * Yspeed) + (cos45 * Xspeed)) + Rotational_LinearSpeed;
	TR = ((cos45 * Yspeed) - (cos45 * Xspeed)) - Rotational_LinearSpeed;
	DR = ((cos45 * Yspeed) + (cos45 * Xspeed)) - Rotational_LinearSpeed;
	DL = ((cos45 * Yspeed) - (cos45 * Xspeed)) + Rotational_LinearSpeed;

	if (Xspeed == 0 && Yspeed == 0) {
		TL = 0;
		TR = 0;
		DR = 0;
		DL = 0;
	}

}
void InverseKinematicCalculation::CalculateRobotSpeed(float TL, float TR, float DR, float DL) {

	const float cos45 = 0.70710678f;
	const float RadToDeg = 180.0f / 3.14159265f;
	const float RobotRadius = 90.0f;

	float Vx = (TL - DL + TR - DR) * 0.25f / cos45;
	float Vy = (TL + DL - TR - DR) * 0.25f / cos45;
	float Rotation = (TL + TR + DR + DL) * 0.25f;

	VectorSpeed = sqrt((Vx * Vy) + (Vy * Vy));
	AngularVelocity = (Rotation / RobotRadius) * RadToDeg;
}

