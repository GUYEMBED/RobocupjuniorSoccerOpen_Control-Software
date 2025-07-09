#include "Localization.h"

Localization::Localization(float overdriftThreshold) {
	overdrift_threshold = overdriftThreshold;
}

float Localization::Displacement(float X1, float Y1, float X2, float Y2) {
	return sqrt(((X1 - X2) * (X1 - X2)) + ((Y1 - Y2) * (Y1 - Y2)));
}

float Localization::multiplyTimes(int iterationTime, float value) {
	float result = 0;
	for (int i = 0; i < abs(iterationTime); i++) {
		result *= value;
	}
	if (iterationTime < 0) {
		return 1 / result;
	} else if (iterationTime == 0) {
		return 1;
	}
	return result;
}

void Localization::updateEncoder(float UpdatePeriod_milliSec) {
	static uint32_t updatetime = 0;
	static float SpeedX = 0;
	static float SpeedY = 0;

	if (SystemTimer.getMilliseconds() - updatetime >= UpdatePeriod_milliSec) { // update rate of 11 ms is optimal
		SpeedX = KinematicState.Xspeed * sin(ToRadians(IMU.HeaderAngleError));
		SpeedY = KinematicState.Yspeed * cos(ToRadians(IMU.HeaderAngleError));

		encoderCurrentX += SpeedX;
		encoderCurrentY += SpeedY;
		updatetime = SystemTimer.getMilliseconds();
	}
}

void Localization::localize() {
	static bool RelocationCalled = false;

	if (!RelocationCalled) {
		relocateEncoder(lidar.currentX, lidar.currentY);
		RelocationCalled = true;
	}

	updateEncoder(11);
	encoderDriftX = abs(encoderCurrentX - lidar.currentX);
	encoderDriftY = abs(encoderCurrentY - lidar.currentY);
	encoderDrift_displacement = Displacement(encoderCurrentX, encoderCurrentY, lidar.currentX, lidar.currentY);

	float AlphaX = 1 / (1 + multiplyTimes(-encoderDriftX, e));
	float AlphaY = 1 / (1 + multiplyTimes(-encoderDriftY, e));

	currentX = (AlphaX * encoderCurrentX) + ((1 - AlphaX) * lidar.currentX);
	currentY = (AlphaY * encoderCurrentY) + ((1 - AlphaY) * lidar.currentY);

	if (encoderDrift_displacement >= overdrift_threshold) {
		relocateEncoder(lidar.currentX, lidar.currentY);
	}
}

void Localization::relocateEncoder(float newCoordX, float newCoordY) {
	encoderCurrentX = newCoordX;
	encoderCurrentY = newCoordY;
}
float Localization::ToRadians(float degrees) {
	return degrees * (PI / 180.0);
}

float Localization::ToDegrees(float radians) {
	return radians * (180.0 / PI);
}
