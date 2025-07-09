#ifndef BALLTRACK_ANGLECHECKING_H
#define BALLTRACK_ANGLECHECKING_H

#include "BallData.h"
#include "KinematicData.h"

class BallTrack_AngleChecking {
private:
	float targetYdisFromBall = 0;
	float DeviationDistance = 0;
	float robotRadius = 0;

	float toRadians(float degrees);
	float toDegrees(float rad);
	float normalizeDegrees(float degrees);
	int sign(float x);

public:
	BallTrack_AngleChecking();
	void update(float TargetYdisFromBall, float Deviation_Distance);
	float effectiveAngleFromYaxis(int *CalculationState);
	bool moveAngle_IS_effective(float MoveAngle, float effectiveAngle);
};

#endif  // BALLTRACK_ANGLECHECKING_H
