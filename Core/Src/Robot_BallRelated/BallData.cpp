#include "BallData.h"

BallData::BallData() {
}
void BallData::update(float X_distance, float Y_distance, float X_speed, float Y_speed, float Displacement, float AngleFromYaxis) {
	Xdistance = X_distance;
	Ydistance = Y_distance;
	Xspeed = X_speed;
	Yspeed = Y_speed;
	displacement = Displacement;
	AngleFrom_Yaxis = AngleFromYaxis;
}
