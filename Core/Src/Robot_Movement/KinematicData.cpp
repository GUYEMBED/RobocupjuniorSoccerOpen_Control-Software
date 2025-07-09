#include "KinematicData.h"

KinematicData::KinematicData() {
}
void KinematicData::update(float Xencode, float Yencode, float Angular_Velocity, float TR, float TL, float DR, float DL, std::string TL_in1, std::string TL_in2, std::string TR_in1, std::string TR_in2,
		std::string DL_in1, std::string DL_in2, std::string DR_in1, std::string DR_in2, float robot_Radius, float maxspeed_) {
	Xspeed = Xencode;
	AngularVelocity = Angular_Velocity;
	TRmotor_spd = TR;
	TLmotor_spd = TL;
	DRmotor_spd = DR;
	DLmotor_spd = DL;
	TRin1 = TR_in1;
	TRin2 = TR_in2;
	TLin1 = TL_in1;
	TLin2 = TL_in2;
	DRin1 = DR_in1;
	DRin2 = DR_in2;
	DLin1 = DL_in1;
	DLin2 = DL_in2;
	robotRadius = robot_Radius;
	maxSpeed = maxspeed_;

	if (Angular_Velocity != 0) {
		Yspeed = Yencode - ((2 * PI * 80) / Angular_Velocity);
	} else {
		Yspeed = Yencode;
	}

	calculateVectorProperties();
}
void KinematicData::calculateVectorProperties() {
	VectorVelocity = sqrt((Xspeed * Xspeed) + (Yspeed * Yspeed));

	if (Xspeed == 0) {
		if (Yspeed > 0) {
			VectorAngle = 0;
		} else if (Yspeed < 0) {
			VectorAngle = PI / 2;
		} else {
			VectorAngle = 0;
		}
	} else {
		float Angle = atan(Yspeed / Xspeed);
		if (Angle > 0) {
			VectorAngle = (PI / 2) - Angle;
		} else if (Angle < 0) {
			VectorAngle = -(PI / 2) - Angle;
		} else {
			VectorAngle = (PI / 2) * (Xspeed / abs(Xspeed));
		}
	}
}
