#ifndef MovementSystem_h
#define MovementSystem_h

#include "math.h"
#include "KinematicData.h"
#include "PIDController.h"
#include "Encoder.h"
#include "InverseKinematicCalculation.h"
#include "motorControl.h"
#include "BNO055.h"

class movementSystem {
private:
	float Ideal_TL = 0;
	float Ideal_TR = 0;
	float Ideal_DL = 0;
	float Ideal_DR = 0;

	float synAdjust_TL = 0;
	float synAdjust_TR = 0;
	float synAdjust_DL = 0;
	float synAdjust_DR = 0;

	float total_TL = 0;
	float total_TR = 0;
	float total_DL = 0;
	float total_DR = 0;
	float movementXspeed = 0;
	float movementYspeed = 0;

	uint32_t updatetime = 0;
	void CalculateSlipAvoid_AdjustmentSpeed();

public:
	motorControl TLmotor;
	motorControl TRmotor;
	motorControl DLmotor;
	motorControl DRmotor;

	PIDController HeadingController;
	PIDController TLmotor_sync;
	PIDController TRmotor_sync;
	PIDController DLmotor_sync;
	PIDController DRmotor_sync;

	PIDController PairMotorTLDR_sync;
	PIDController PairMotorTRDL_sync;

	InverseKinematicCalculation InverseKinematic;

	float flag = 0;

	movementSystem();
	void move(float inputXspeed, float inputYspeed, float HeadingAngle);
	void TestMove(float Xspeed, float Yspeed);
};

extern movementSystem Robotmovement;

#endif
