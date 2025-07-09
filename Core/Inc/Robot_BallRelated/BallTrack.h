#ifndef BALLTRACK_H
#define BALLTRACK_H

#include "movementSystem.h"
#include "PIDController.h"
#include "BallTrack_AngleChecking.h"
#include "BallData.h"
#include "BNO055.h"

class BallTrack {
private:
	float kpx, kix, kdx;
	float kpy, kiy, kdy;
	float thresholdX, thresholdY;
	float periodX, periodY;

	float YdisFrom_Ball;
	float DeviationDistance;

public:
	float effectiveMovementAngle = 0;
	float outputXspeed = 0, outputYspeed = 0;
	int AngleCalculationState = 0;

	BallTrack_AngleChecking movementAngle;
	PIDController XspeedControl;
	PIDController YspeedControl;
	PIDController ShortRange;

	BallTrack(float Px, float Ix, float Dx, float Py, float Iy, float Dy, float threshold_X, float threshold_Y, float period_X, float period_Y, float YdisFromBall, float Deviation_Distance);
	float CalculateAngle(float Xspeed, float Yspeed);
	float toRadians(float degrees);
	float toDegrees(float rad);

	void tracking();
	void updateTrackingState();
};

#endif
