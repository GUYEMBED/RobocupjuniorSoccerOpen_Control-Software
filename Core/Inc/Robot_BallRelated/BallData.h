#ifndef BALLDATA_H
#define BALLDATA_H
#include "math.h"

class BallData {
public:
	float Xdistance;
	float Ydistance;
	float Xspeed;
	float Yspeed;
	float displacement;
	float AngleFrom_Yaxis;

	BallData();
	void update(float X_distance, float Y_distance, float X_speed, float Y_speed, float Displacement, float AngleFromYaxis);
};

extern BallData BallState;

#endif  // BALLDATA_H
