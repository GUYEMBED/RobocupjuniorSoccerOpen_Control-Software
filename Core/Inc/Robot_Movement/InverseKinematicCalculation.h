#ifndef InverseKinematicCalculation_h
#define InverseKinematicCalculation_h

#define PI 3.14159265358979323846
#include "KinematicData.h"
#include <math.h>

class InverseKinematicCalculation {
private:
	float cos45 = cos(PI / 4);
	float vector1 = 0;
	float vector2 = 0;
	float ImbVector_SpeedDiff = 0;
	float Xspeed = 0;
	float Yspeed = 0;
	float VectorAngle = 0;
	char quadant = '\0';
	float RobotRadius = 0;
	float MaxSpeed = 0;
	bool Is_Clockwise = true;


	float HigherValueOF(float value1, float value2);
	float Imb_Vector_LowerWHspd(float Imb_Vector_Velocity, float Angular_Velocity);
	char Quadant(float Vector_Angle);
	void CalculateVector1_Vector2(float Xspeed, float Yspeed);
	void Apply_wheelDirection();
	void Apply_wheelSpeed(float ImbVector, float balancedVector, bool Imb_IsHigher, float Xspeed, float Yspeed, float Angular_Velocity, bool Is_Clockwise);

public:
	float TR = 0;
	float TL = 0;
	float DR = 0;
	float DL = 0;

	float VectorSpeed = 0;
	float AngularVelocity = 0;

	InverseKinematicCalculation();
	void CalculateRobotSpeed(float TL, float TR, float DR, float DL);
	void CalculateWheelSpeed(float Xspeed, float Yspeed, float Angular_Velocity);
};

#endif
