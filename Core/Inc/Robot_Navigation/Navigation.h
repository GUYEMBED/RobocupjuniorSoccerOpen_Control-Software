#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <cmath>
#include "movementSystem.h"
#include "PIDController.h"
#include "LidarReader.h"

class Navigation {
private:
	float targetXpos = 0;
	float targetYpos = 0;

	float currentXpos = 0;
	float currentYpos = 0;

	float othertargetXpos = 0;
	float othertargetYpos = 0;

	float othercurrentXpos = 0;
	float othercurrentYpos = 0;

	float kp = 0;
	float ki = 0;
	float kd = 0;
	float collisionDis_threshold = 0;
	float threshold = 0;

	float calculateAngle(float startX, float startY, float endX, float endY);
	float calculateDisplacement(float startX, float startY, float endX, float endY);
	bool CollisionCheck(float pathAngle_fromYaxis, float *newPathAngle);
	float CalculateAvoidAngle(float ObstacleDis);
	float ToRadians(float degrees);
	float ToDegrees(float radians);

public:
	movementSystem movement;
	PIDController NavigationPID;
	Navigation(float currentX, float currentY, float collisionDisThreshold, float threshold_, float P, float I, float D);

	void moveTO(float targetX, float targetY);
};

#endif // NAVIGATION_H
