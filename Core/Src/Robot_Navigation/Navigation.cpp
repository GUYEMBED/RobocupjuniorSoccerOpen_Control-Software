#include "Navigation.h"

Navigation::Navigation(float currentX, float currentY, float collisionDisThreshold, float threshold_, float P, float I, float D) {
	currentXpos = currentX;
	currentYpos = currentY;

	collisionDis_threshold = collisionDisThreshold;
	threshold = threshold_;
	kp = P;
	ki = I;
	kd = D;
}

float Navigation::calculateAngle(float startX, float startY, float endX, float endY) {
	float Xdistance = endX - startX;
	float Ydistance = startY - endY;
	return atan(Xdistance / Ydistance);
}

float Navigation::calculateDisplacement(float startX, float startY, float endX, float endY) {
	float Xdistance = endX - startX;
	float Ydistance = startY - endY;
	return sqrt((Xdistance * Xdistance) + (Ydistance * Ydistance));
}

void Navigation::moveTO(float targetX, float targetY) {
	static float speedX = 0;
	static float speedY = 0;

	float pathAngle = calculateAngle(currentXpos, currentYpos, targetX, targetY);
	float pathDisplacement = calculateDisplacement(currentXpos, currentYpos, targetX, targetY);

	NavigationPID.update(pathDisplacement, 0, threshold, kp, kd, ki, 200);

	CollisionCheck(pathAngle, &pathAngle);
	speedY = NavigationPID.Adjustment_output * cos(pathAngle);
	speedX = NavigationPID.Adjustment_output * sin(pathAngle);
	movement.move(speedX, speedY, 0);
}

float Navigation::CalculateAvoidAngle(float ObstacleDis) {
	float robotDiameter = 180;
	float diameterThreshold = 10;
	return ToDegrees(asin((robotDiameter + diameterThreshold) / ObstacleDis));
}

bool Navigation::CollisionCheck(float pathAngle_fromYaxis, float *newPathAngle) {
	static float smallestDis;
	static float AngleAt_smallestDis;

	lidar.FindSmallestDis_Angle(&smallestDis, &AngleAt_smallestDis);
	float path_obstacle_AngleDif = abs(pathAngle_fromYaxis - AngleAt_smallestDis);

	float avoidAngle = CalculateAvoidAngle(smallestDis);

	if (smallestDis <= collisionDis_threshold) {
		if (avoidAngle <= path_obstacle_AngleDif) {
			return false;
		}
		if (abs(pathAngle_fromYaxis) > AngleAt_smallestDis) {
			*newPathAngle = (abs(AngleAt_smallestDis) + avoidAngle) * (abs(pathAngle_fromYaxis) / pathAngle_fromYaxis);
		} else if (abs(pathAngle_fromYaxis) < AngleAt_smallestDis) {
			*newPathAngle = (abs(AngleAt_smallestDis) - avoidAngle) * (abs(pathAngle_fromYaxis) / pathAngle_fromYaxis);
		} else {
			// choosable
		}
		return true;
	} else {
		return false;
	}
}

float Navigation::ToRadians(float degrees) {
	return degrees * (PI / 180.0);
}

float Navigation::ToDegrees(float radians) {
	return radians * (180.0 / PI);
}
