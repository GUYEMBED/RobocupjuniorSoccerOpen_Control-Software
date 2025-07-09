#ifndef KINEMATIC_DATA_H
#define KINEMATIC_DATA_H
#define PI 3.14159265358979323846
#include "math.h"
#include <iostream>
#include <string>

class KinematicData {
public:
	KinematicData();
	float Xspeed;
	float Yspeed;
	float AngularVelocity;
	float TLmotor_spd;
	float TRmotor_spd;
	float DLmotor_spd;
	float DRmotor_spd;
	float VectorVelocity;
	float VectorAngle;

	float robotRadius;
	float maxSpeed;

	void update(float Xencode, float Yencode, float Angular_Velocity, float TR, float TL, float DR, float DL, std::string TL_in1, std::string TL_in2, std::string TR_in1, std::string TR_in2,
			std::string DL_in1, std::string DL_in2, std::string DR_in1, std::string DR_in2, float robot_Radius, float maxspeed_);

	std::string TLin1;
	std::string TLin2;
	std::string TRin1;
	std::string TRin2;
	std::string DLin1;
	std::string DLin2;
	std::string DRin1;
	std::string DRin2;

private:
	void calculateVectorProperties();
};

extern KinematicData KinematicState;

#endif  // KINEMATIC_DATA_H
