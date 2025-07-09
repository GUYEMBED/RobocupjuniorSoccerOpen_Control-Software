#include "MovementSystem.h"

movementSystem::movementSystem() {
}
void movementSystem::CalculateSlipAvoid_AdjustmentSpeed() {
	InverseKinematic.CalculateWheelSpeed(KinematicState.Xspeed, KinematicState.Yspeed, KinematicState.AngularVelocity);

	Ideal_TL = InverseKinematic.TL;
	Ideal_TR = InverseKinematic.TR;
	Ideal_DL = InverseKinematic.DL;
	Ideal_DR = InverseKinematic.DR;

	TLmotor_sync.update(Ideal_TL, KinematicState.TLmotor_spd, 60, 0.1, 0, 0, 50);
	TRmotor_sync.update(Ideal_TR, KinematicState.TRmotor_spd, 60, 0.1, 0, 0, 50);
	DLmotor_sync.update(Ideal_DL, KinematicState.DLmotor_spd, 60, 0.1, 0, 0, 50);
	DRmotor_sync.update(Ideal_DR, KinematicState.DRmotor_spd, 60, 0.1, 0, 0, 50);

	synAdjust_TL = TLmotor_sync.Adjustment_output;
	synAdjust_TR = TRmotor_sync.Adjustment_output;
	synAdjust_DL = DLmotor_sync.Adjustment_output;
	synAdjust_DR = DRmotor_sync.Adjustment_output;
}
void movementSystem::move(float inputXspeed, float inputYspeed, float HeadingAngle) {
	// if heading is on the left side of field orientation means HeaderAngleError must be negative
	flag = 1;
	//const float Deg_Rad_Ratio = 57.2957795131;
	//float tempX = inputXspeed;
	//float tempY = inputYspeed;
	//inputXspeed = (tempX * cos(IMU.HeaderAngleError / Deg_Rad_Ratio)) + (tempY * sin(IMU.HeaderAngleError / Deg_Rad_Ratio));
	//inputYspeed = (tempY * cos(IMU.HeaderAngleError / Deg_Rad_Ratio)) - (tempX * sin(IMU.HeaderAngleError / Deg_Rad_Ratio));

	//CalculateSlipAvoid_AdjustmentSpeed();
	//float XspeedDiff = inputXspeed - movementXspeed;
	//float YspeedDiff = inputYspeed - movementYspeed;
	// float vectorSpeed = sqrt((inputXspeed * inputXspeed) + (inputYspeed * inputYspeed));

	//if (SystemTimer.getMilliseconds() - updatetime >= 30) {
	//movementXspeed += XspeedDiff * 0.1; // 0.49 is the gain factor for X speed adjustment
	//movementYspeed += YspeedDiff * 0.1; // 0.49 is the gain factor for Y speed adjustment
	//updatetime = SystemTimer.getMilliseconds();
	//}
	/*
	 TLmotor.Setup(KinematicState.TLin1, KinematicState.TLin2, 0.0001, 0, 0.00007, 10, 0, KinematicState.TLmotor_spd, 0.0001, 0, 0.00007, 10, 0); // D 0.025 neg
	 TRmotor.Setup(KinematicState.TRin1, KinematicState.TRin2, 0.0001, 0, 0.00007, 10, 0, KinematicState.TRmotor_spd, 0.0001, 0, 0.00007, 10, 0);
	 DLmotor.Setup(KinematicState.DLin1, KinematicState.DLin2, 0.0001, 0, 0.00007, 10, 0, KinematicState.DLmotor_spd, 0.0001, 0, 0.00007, 10, 0);
	 DRmotor.Setup(KinematicState.DRin1, KinematicState.DRin2, 0.0001, 0, 0.00007, 10, 0, KinematicState.DRmotor_spd, 0.0001, 0, 0.00007, 10, 0);
	 */

	TLmotor.Setup(KinematicState.TLin1, KinematicState.TLin2, 0.00071, 0, 0.00029, 10, 0, KinematicState.TLmotor_spd, 0.00071, 0, 0.00029, 10, 0); // D 0.025 neg
	TRmotor.Setup(KinematicState.TRin1, KinematicState.TRin2, 0.00071, 0, 0.00029, 10, 0, KinematicState.TRmotor_spd, 0.00071, 0, 0.00029, 10, 0);
	DLmotor.Setup(KinematicState.DLin1, KinematicState.DLin2, 0.00071, 0, 0.00029, 10, 0, KinematicState.DLmotor_spd, 0.00071, 0, 0.00029, 10, 0);
	DRmotor.Setup(KinematicState.DRin1, KinematicState.DRin2, 0.00071, 0, 0.00029, 10, 0, KinematicState.DRmotor_spd, 0.00071, 0, 0.00029, 10, 0);

	HeadingController.update(HeadingAngle, IMU.HeaderAngleError, 1.1, 2.9, 1.1, 0, 10);

	float minVectorSpeed = 200;
	float vectorSpeed = sqrt((inputXspeed * inputXspeed) + (inputYspeed * inputYspeed));

	const float epsilon = 0.0001f;
	if (vectorSpeed < minVectorSpeed && vectorSpeed > epsilon) {
		float scale = minVectorSpeed / vectorSpeed;
		inputXspeed *= scale;
		inputYspeed *= scale;
	}

	InverseKinematic.CalculateWheelSpeed(inputXspeed, inputYspeed, HeadingController.Adjustment_output);

	total_TL = InverseKinematic.TL;
	total_TR = InverseKinematic.TR;
	total_DL = InverseKinematic.DL;
	total_DR = InverseKinematic.DR;

	// calculate Vectordiff

	float speedDif_Gain = 0.19; //0.08
	float VectorDif_ratio = 0;
	float Vector2Adjust = 0;
	float Vector1Adjust = 0;

	float averageVector1_currentspd = (abs(KinematicState.TLmotor_spd) + abs(KinematicState.DRmotor_spd)) / 2;
	float averageVector2_currentspd = (abs(KinematicState.TRmotor_spd) + abs(KinematicState.DLmotor_spd)) / 2;

	float avgVector1_speedDif = abs(total_TL) - averageVector1_currentspd;
	float avgVector2_speedDif = abs(total_TR) - averageVector2_currentspd;

	if (avgVector2_speedDif == 0 && avgVector1_speedDif == 0) {
		Vector1Adjust = 0;
		Vector2Adjust = 0;
	}
	if (avgVector2_speedDif != 0 && avgVector1_speedDif == 0) {
		Vector2Adjust = (avgVector2_speedDif * speedDif_Gain) * (total_TR < 0 ? -1 : 1);
		Vector1Adjust = 0;
	}
	if (avgVector2_speedDif == 0 && avgVector1_speedDif != 0) {
		Vector1Adjust = (avgVector1_speedDif * speedDif_Gain) * (total_TL < 0 ? -1 : 1);
		Vector2Adjust = 0;
	}
	if (avgVector1_speedDif != 0 && avgVector2_speedDif != 0) {
		VectorDif_ratio = avgVector1_speedDif / avgVector2_speedDif;

		if ((Vector1Adjust * Vector2Adjust > 0) && (Vector1Adjust > 0)) {

			if (avgVector1_speedDif > avgVector2_speedDif) {

				Vector1Adjust = (Vector2Adjust * VectorDif_ratio) * (total_TL < 0 ? 1 : -1);
				Vector2Adjust = (avgVector2_speedDif * speedDif_Gain) * (total_TR < 0 ? -1 : 1);
			} else if (avgVector2_speedDif > avgVector1_speedDif) {

				Vector1Adjust = (Vector2Adjust * VectorDif_ratio) * (total_TL < 0 ? -1 : 1);
				Vector2Adjust = (avgVector2_speedDif * speedDif_Gain) * (total_TR < 0 ? 1 : -1);
			}
		} else {
			Vector2Adjust = (avgVector2_speedDif * speedDif_Gain) * (total_TR < 0 ? -1 : 1);
			Vector1Adjust = (Vector2Adjust * VectorDif_ratio) * (total_TL < 0 ? -1 : 1);
		}
	}

	float maxSpeed = fmaxf(fabsf(total_TL), fmaxf(fabsf(total_TR), fmaxf(fabsf(total_DL), fabsf(total_DR))));
	if (maxSpeed > 1000.0f) {
		float scale = 1000.0f / maxSpeed;
		total_TL *= scale;
		total_TR *= scale;
		total_DL *= scale;
		total_DR *= scale;
	}
	float TL_DR_Dif = abs(KinematicState.TLmotor_spd) - abs(KinematicState.DRmotor_spd);
	float TR_DL_Dif = abs(KinematicState.TRmotor_spd) - abs(KinematicState.DLmotor_spd);

	PairMotorTLDR_sync.update(0, -TL_DR_Dif, 0, 0.28, 0.27, 0, 50); // kp = 0.48, kd = 0.4
	PairMotorTRDL_sync.update(0, -TR_DL_Dif, 0, 0.28, 0.27, 0, 50);

	float TL_DR_adjust = PairMotorTLDR_sync.Adjustment_output * (total_TL < 0 ? -1 : 1); // 0.385
	float TR_DL_adjust = PairMotorTRDL_sync.Adjustment_output * (total_TR < 0 ? -1 : 1);

	if (abs(IMU.HeaderAngleError) >= 4) {
		TL_DR_adjust = 0;
		TR_DL_adjust = 0;
		Vector2Adjust = 0;
		Vector1Adjust = 0;
	}

	TLmotor.posoffset = 20;
	TRmotor.posoffset = 20;
	DLmotor.posoffset = 20;
	DRmotor.posoffset = 65;

	TLmotor.negoffset = 14;
	TRmotor.negoffset = 14;
	DLmotor.negoffset = 14;
	DRmotor.negoffset = 55;

	//TL_DR_adjust = 0;
	//TR_DL_adjust = 0;

	//Vector1Adjust = 0;
	//Vector2Adjust = 0;

	if (inputXspeed == 0 && inputYspeed == 0) {
		TLmotor.spin(0, 0, 0);
		TRmotor.spin(0, 0, 0);
		DLmotor.spin(0, 0, 0);
		DRmotor.spin(0, 0, 0);
	} else {
		TLmotor.drive((total_TL + Vector1Adjust) - TL_DR_adjust, KinematicState.TLmotor_spd, 1, 1);
		TRmotor.drive((total_TR + Vector2Adjust) - TR_DL_adjust, KinematicState.TRmotor_spd, 1, 1);
		DLmotor.drive((total_DL + Vector2Adjust) + TR_DL_adjust, KinematicState.DLmotor_spd, 1, 1);
		DRmotor.drive((total_DR + Vector1Adjust) + TL_DR_adjust, KinematicState.DRmotor_spd, 0.6, 0.6);
	}

}

void movementSystem::TestMove(float Xspeed, float Yspeed) {

	TLmotor.Setup(KinematicState.TLin1, KinematicState.TLin2, 0.00077, 0, 0.00024, 0, 0, KinematicState.TLmotor_spd, 0.00065, 0, 0.000195, 0, 0); // D 0.025 neg
	TRmotor.Setup(KinematicState.TRin1, KinematicState.TRin2, 0.00077, 0, 0.00024, 0, 0, KinematicState.TRmotor_spd, 0.00065, 0, 0.000195, 0, 0);
	DLmotor.Setup(KinematicState.DLin1, KinematicState.DLin2, 0.00077, 0, 0.00024, 0, 0, KinematicState.DLmotor_spd, 0.00065, 0, 0.000195, 0, 0);
	DRmotor.Setup(KinematicState.DRin1, KinematicState.DRin2, 0.00077, 0, 0.00024, 0, 0, KinematicState.DRmotor_spd, 0.00077, 0, 0.000195, 0, 0);

	//InverseKinematic.CalculateWheelSpeed(Xspeed, Yspeed, 0);

	//TLmotor.spin(Yspeed, 187, 125); // 126
	//TRmotor.spin(Yspeed, 187, 134); // 131
	//DLmotor.spin(Yspeed, 187, 125); // 126
	//DRmotor.spin(Yspeed, 187, 134); // 134

	TLmotor.posoffset = 0;
	TRmotor.posoffset = 0;
	DLmotor.posoffset = 0;
	DRmotor.posoffset = 0;

	TLmotor.negoffset = 0;
	TRmotor.negoffset = 0;
	DLmotor.negoffset = 0;
	DRmotor.negoffset = 0;

	//TLmotor.drive(300, KinematicState.TLmotor_spd);
	//TRmotor.drive(300, KinematicState.TRmotor_spd);
	//DLmotor.drive(300, KinematicState.DLmotor_spd);
	//DRmotor.drive(300, KinematicState.DRmotor_spd);

}
