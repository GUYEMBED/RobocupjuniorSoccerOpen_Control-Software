#include "BallTrack.h"

BallTrack::BallTrack(float Px, float Ix, float Dx, float Py, float Iy, float Dy, float threshold_X, float threshold_Y, float period_X, float period_Y, float YdisFromBall, float Deviation_Distance) :
		kpx(Px), kix(Ix), kdx(Dx), kpy(Py), kiy(Iy), kdy(Dy), thresholdX(threshold_X), thresholdY(threshold_Y), periodX(period_X), periodY(period_Y), YdisFrom_Ball(YdisFromBall), DeviationDistance(
				Deviation_Distance) {
}

float BallTrack::CalculateAngle(float Xspeed, float Yspeed) {
	return toDegrees(atan2(Xspeed, Yspeed));
}

float BallTrack::toRadians(float degrees) {
	return degrees * (3.1415926 / 180.0);
}

float BallTrack::toDegrees(float rad) {
	return rad * 180.0 / 3.1415926;
}

void BallTrack::updateTrackingState() {
	effectiveMovementAngle = movementAngle.effectiveAngleFromYaxis(&AngleCalculationState);
}

void BallTrack::tracking() {
	movementAngle.update(YdisFrom_Ball, DeviationDistance);

	// tuning parameters : YdisFromBall, DeviationDistance, kpx, kdx, periodX, kpy, kdy, periodY

	//if (AngleCalculationState) {
	XspeedControl.update(0, -BallState.Xdistance, thresholdX, kpx, kdx, kix, periodX);
	YspeedControl.update(0, -BallState.Ydistance, thresholdY, kpy, kdy, kiy, periodY); // - YdisFrom_Ball
	ShortRange.update(0, -BallState.AngleFrom_Yaxis, 0, 3.5, 1.3, 0, 10);

	float currentMovementAngle = CalculateAngle(XspeedControl.Adjustment_output, YspeedControl.Adjustment_output);
	float currentTotalSpeed = sqrt(XspeedControl.Adjustment_output * XspeedControl.Adjustment_output + YspeedControl.Adjustment_output * YspeedControl.Adjustment_output);

	if (!movementAngle.moveAngle_IS_effective(currentMovementAngle, effectiveMovementAngle)) {
		if (effectiveMovementAngle != 0 || abs(effectiveMovementAngle) != 90 || effectiveMovementAngle != 180) {
			outputXspeed = XspeedControl.Adjustment_output;
			outputYspeed = outputXspeed / tan(toRadians(effectiveMovementAngle));
		}
		if (effectiveMovementAngle == 0 || effectiveMovementAngle == 180) {
			outputXspeed = 0;
			outputYspeed = YspeedControl.Adjustment_output;
		}
		if (abs(effectiveMovementAngle) == 90) {
			outputXspeed = XspeedControl.Adjustment_output;
			outputYspeed = 0;
		}
	} else {
		outputXspeed = XspeedControl.Adjustment_output;
		outputYspeed = YspeedControl.Adjustment_output;
	}
	if (BallState.displacement >= 15) {
		//Robotmovement.move(outputXspeed, outputYspeed, 0);
	} else {
		/*
		 float YXratio = 0;
		 if (outputXspeed != 0) {
		 YXratio = outputYspeed / outputXspeed;
		 } else {
		 YXratio = 0;
		 }
		 float Xspeed_lift = 180 * (outputXspeed < 0 ? -1 : 1);
		 float Yspeed_lift = Xspeed_lift * YXratio * (BallState.Ydistance < 0 ? -1 : 1);
		 if (abs(currentMovementAngle) < 10) {
		 Yspeed_lift = 180 * (BallState.Ydistance < 0 ? -1 : 1);
		 if (YXratio == 0) {
		 Xspeed_lift = 0;
		 }
		 }
		 */

		//float VectorSpeed_lift = 220;
		//float Xspeed_lift = abs(VectorSpeed_lift * sin(toRadians(currentMovementAngle))) * (outputXspeed < 0 ? -1 : 1);
		//float Yspeed_lift = abs(VectorSpeed_lift * cos(toRadians(currentMovementAngle))) * (BallState.Ydistance < 0 ? -1 : 1);
		//YdisFrom_Ball = 9;

		//Robotmovement.move(Xspeed_lift, Yspeed_lift, 0);
	}
	Robotmovement.move(outputXspeed, outputYspeed, 0);
}
