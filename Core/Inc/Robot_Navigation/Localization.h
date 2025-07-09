#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <cmath>
#include "KinematicData.h"
#include "BNO055.h"
#include "LSM6DSOX.h"
#include "LidarReader.h"

class Localization {
private:
	static constexpr double e = 2.718281828459045;
	float encoderCurrentX = 0;
	float encoderCurrentY = 0;

	float encoderDriftX = 0;
	float encoderDriftY = 0;
	float encoderDrift_displacement = 0;
	float overdrift_threshold = 0;

	float Alpha = 0;

	float Displacement(float X1, float Y1, float X2, float Y2);
	float multiplyTimes(int iterationTime, float value);
	float ToRadians(float degrees);
	float ToDegrees(float radians);

	void updateEncoder(float UpdatePeriod_milliSec);
	void relocateEncoder(float newCoordX, float newCoordY);

public:
	float currentX = 0;
	float currentY = 0;

	Localization(float overdriftThreshold);

	void localize();
};

extern Localization Field_Localization;

#endif
