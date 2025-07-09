#ifndef LIDARREADER_H
#define LIDARREADER_H

#include "main.h"
#include "BNO055.h"
#include "LSM6DSOX.h"
#include "Timer.h"
#include <cstring>
#include <cmath>
#include "fieldData.h"

#define PACKET_SIZE 200
#define HEADER_SIZE 10
#define MAX_PACKET_SIZE 200

class LidarReader {
public:
	LidarReader();

	void init();
	void onUartReceiveComplete();
	void processData();
	bool isAngleInCurrentFrame(float angle);
	uint16_t getDistanceAtAngle(float angle);
	void FindLargestDis_AngleInRange(float *largestDis, float *angleAt_largestDis, float minRelAngle, float maxRelAngle);
	void FindSmallestDis_Angle(float *smallestDis, float *angleAt_smallestDis);
	bool FindLargestDis_CallState = false;
	void localize();
	bool CheckComplete = true;
	float lastLargestDistance = 0;
	float lastAngleAtLargestDistance = 0;
	float angleBuffer[PACKET_SIZE] = { 0 };

	float currentX = 0;
	float currentY = 0;
	volatile bool newFrameAvailable = false;

private:
	void CalculateXYcoord(float CornerDis, float CornerAngle, float *X, float *Y);
	void resetBuffers();
	float decodeAngle(uint8_t low, uint8_t high);

	uint8_t SampleAmount = 0;
	float fsa = 0;
	float lsa = 0;
	float AngleStep = 0;
	uint16_t SampleDistance[PACKET_SIZE] = { 0 };
	float SampleAngles[PACKET_SIZE] = { 0 };  // same size as SampleDistance
	int globalStart = 0;
	float maxDis = 0;
	float maxAngle = 0;
	int iterateAngle = 0;

	uint8_t rx;
	uint8_t tempBuffer[PACKET_SIZE];
	uint16_t expectedPacketLength = 0;
	int16_t packetIndex = 0;
};

extern LidarReader lidar;

#endif
