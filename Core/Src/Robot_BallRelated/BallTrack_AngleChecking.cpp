#include "BallTrack_AngleChecking.h"

float BallTrack_AngleChecking::toRadians(float degrees) {
	return degrees * M_PI / 180.0;
}

float BallTrack_AngleChecking::toDegrees(float rad) {
	return rad * 180.0 / M_PI;
}

float BallTrack_AngleChecking::normalizeDegrees(float degrees) {
	int n = (int) (degrees / 360.0f);  // number of full rotations
	float normalized = degrees - n * 360.0f;
	// Adjust if result is negative
	if (normalized < 0.0f) {
		normalized += 360.0f;
	}
	// Step 2: Convert to range (-180, 180]
	if (normalized > 180.0f) {
		normalized -= 360.0f;
	}

	return normalized;
}

int BallTrack_AngleChecking::sign(float x) {
	if (x > 0) {
		return 1;
	}
	if (x < 0) {
		return -1;
	}
	return 0;
}

void BallTrack_AngleChecking::update(float TargetYdisFromBall, float Deviation_Distance) {
	targetYdisFromBall = TargetYdisFromBall;
	DeviationDistance = Deviation_Distance;
	robotRadius = KinematicState.robotRadius;
}

BallTrack_AngleChecking::BallTrack_AngleChecking() {
}

float BallTrack_AngleChecking::effectiveAngleFromYaxis(int *CalculationState) {
	if (BallState.Ydistance > targetYdisFromBall) {
		float effectiveAngle = toDegrees(atan2(BallState.Xdistance, BallState.Ydistance - targetYdisFromBall));
		return effectiveAngle;
	}
	bool checkState1 = ((DeviationDistance < BallState.displacement) || (DeviationDistance == BallState.displacement));
	bool checkState2 = ((DeviationDistance > -BallState.displacement) || (DeviationDistance == -BallState.displacement));

	if (checkState1 && checkState2) {
		float effectiveAngle = normalizeDegrees(BallState.AngleFrom_Yaxis + (sign(BallState.AngleFrom_Yaxis) * toDegrees(asin(DeviationDistance / BallState.displacement))));
		return effectiveAngle;
	}

	float effectiveAngle = normalizeDegrees(BallState.AngleFrom_Yaxis + sign(BallState.AngleFrom_Yaxis) * toDegrees(asin(1)));
	return effectiveAngle;
}

bool BallTrack_AngleChecking::moveAngle_IS_effective(float MoveAngle, float effectiveAngle) {
	if (sign(MoveAngle) != sign(effectiveAngle)) {
		return false;
	}

	return abs(MoveAngle) >= abs(effectiveAngle);
}
