#include "LidarReader.h"
#include <cstdio> // For printf debug

LidarReader::LidarReader() :
		currentX(0), currentY(0), packetIndex(0) {
	resetBuffers();
}

void LidarReader::resetBuffers() {
	memset(tempBuffer, 0, sizeof(tempBuffer));
	rx = 0;
	packetIndex = 0;
}

void LidarReader::init() {
	HAL_UART_Receive_IT(&huart2, &rx, 1);

	uint8_t startScanCmd[2] = { 0xA5, 0x60 };  // Continuous scan command
	HAL_UART_Transmit(&huart2, startScanCmd, 2, HAL_MAX_DELAY);
}

void LidarReader::onUartReceiveComplete() {
	uint8_t byte = rx;

	// Step 1: Detect header
	if (packetIndex == 0) {
		if (byte == 0xAA) { // 0x55
			tempBuffer[packetIndex] = byte;
			packetIndex++;
		}
	} else if (packetIndex == 1) {
		if (byte == 0x55) { // 0xAA
			tempBuffer[packetIndex] = byte;
			packetIndex++;
		} else {
			packetIndex = 0;  // Reset if second byte isn't 0xAA
		}
	} else {
		// Step 2: Collect rest of packet
		tempBuffer[packetIndex] = byte;

		if (packetIndex == HEADER_SIZE) {
			// Step 3: Extract LSN and calculate expected full packet size
			uint8_t lsn = tempBuffer[3];  // LSN is at index 3
			expectedPacketLength = HEADER_SIZE + (lsn * 3);
			if (expectedPacketLength > MAX_PACKET_SIZE) {
				packetIndex = 0;  // Safety reset
			}
		}

		// Step 4: Check if full packet received
		if (expectedPacketLength > 0 && packetIndex == expectedPacketLength) {
			processData();  // full packet ready
			packetIndex = -1; // Reset for next packet
			expectedPacketLength = 0;
		}
		packetIndex++;
	}

	// Continue receiving next byte
	HAL_UART_Receive_IT(&huart2, &rx, 1);
}

float LidarReader::decodeAngle(uint8_t low, uint8_t high) {
	uint16_t raw = (high << 8) | low;
	return ((raw >> 1) / 64.0f);  // angle in degrees
}
void LidarReader::processData() {
	// Checksum = sum of first 6 bytes must equal 7th byte
	fsa = decodeAngle(tempBuffer[4], tempBuffer[5]);
	lsa = decodeAngle(tempBuffer[6], tempBuffer[7]);
	SampleAmount = tempBuffer[3];
	/*
	 SampleAmount = tempBuffer[3] & 0x0F;  // Only lower 4 bits are valid
	 SampleAmount += 1;
	 */
	float angle_diff = lsa - fsa;
	if (angle_diff < 0) {
		angle_diff += 360.0f;
	}
	AngleStep = angle_diff / SampleAmount;

	uint8_t *sample_start = &tempBuffer[10];

	for (uint8_t i = 0; i < SampleAmount; ++i) {
		uint8_t *sample = &sample_start[i * 3];

		uint8_t dist_low = sample[1];
		uint8_t dist_high = sample[2];

		uint16_t distance_mm = (dist_high << 6) | (dist_low >> 2);

		// Calculate angle
		float angle = fsa + AngleStep * i;
		if (angle >= 360.0f)
			angle -= 360.0f;
		if (angle < 0)
			angle += 360.0f;

		int angle_index = static_cast<int>(angle + 0.5f) % 360;
		angleBuffer[angle_index] = distance_mm;
	}

	newFrameAvailable = true;
}
bool LidarReader::isAngleInCurrentFrame(float angle) {
	if (!newFrameAvailable || SampleAmount == 0)
		return false;

	float start = fsa;
	float end = lsa;

	// Normalize wrap-around
	if (end < start) {
		end += 360.0f;
		if (angle < start)
			angle += 360.0f;
	}

	return (angle >= start && angle <= end);
}

uint16_t LidarReader::getDistanceAtAngle(float angle) {
	int index = static_cast<int>(angle);
	if (index < 0) {
		index += 360;
	} else if (index >= 360) {
		index -= 360;
	}
	return angleBuffer[index];
}
/*
 int globalStart = ((int) (IMU.HeaderAngleError - minRelAngle) + 360) % 360; // lidarStart
 int globalEnd = ((int) (IMU.HeaderAngleError - maxRelAngle) + 360) % 360; // lidarEnd
 */

void LidarReader::FindLargestDis_AngleInRange(float *largestDis, float *angleAt_largestDis, float minRelAngle, float maxRelAngle) {
	int globalStart = ((int) (IMU.HeaderAngleError - minRelAngle) + 360) % 360; // lidarStart
	int globalEnd = ((int) (IMU.HeaderAngleError - maxRelAngle) + 360) % 360; // lidarEnd

	do {
		iterateAngle = (iterateAngle - 1 + 360) % 360;
		float dis = getDistanceAtAngle(iterateAngle);
		if (dis > maxDis) {
			maxDis = dis;
			maxAngle = iterateAngle;
		}
	} while (iterateAngle != (globalEnd - 1 + 360) % 360);

	float relAngle = maxAngle;
	// connvert 0 -> 360 to 0 -> +- 180 angle
	if (relAngle > 180) {
		relAngle -= 360;
	} else if (relAngle < -180) {
		relAngle += 360;
	}

	*angleAt_largestDis = relAngle;
	*largestDis = maxDis;
}

/*
 *
 if (CheckComplete) {
 iterateAngle = globalStart;
 maxDis = 0;
 maxAngle = 0;
 }
 if (isAngleInCurrentFrame(iterateAngle)) {
 float dis = getDistanceAtAngle(iterateAngle);
 if (dis > maxDis) {
 maxDis = dis;
 maxAngle = iterateAngle;
 }
 if (iterateAngle != (globalEnd - 1 + 360) % 360) {
 iterateAngle = (iterateAngle - 1 + 360) % 360;
 CheckComplete = false;
 }
 if (iterateAngle == (globalEnd - 1 + 360) % 360) {
 float relAngle = maxAngle - IMU.HeaderAngleError;
 if (relAngle > 180)
 relAngle -= 360;
 if (relAngle < -180)
 relAngle += 360;

 *angleAt_largestDis = -relAngle;
 *largestDis = maxDis;
 CheckComplete = true;
 newFrameAvailable = false;
 }
 }
 */

void LidarReader::FindSmallestDis_Angle(float *smallestDis, float *angleAt_smallestDis) {
	float minDis = 999999;
	float minAngle = 0;

	for (int i = 0; i < 360; i++) {
		float dis = getDistanceAtAngle(i);
		if (dis > 0 && dis < minDis) {
			minDis = dis;
			minAngle = i;
		}
	}

	float relAngle = minAngle - IMU.HeaderAngleError;
	if (relAngle > 180) {
		relAngle -= 360;
	} else if (relAngle < -180) {
		relAngle += 360;
	}
	*smallestDis = minDis;
	*angleAt_smallestDis = relAngle;
}

void LidarReader::CalculateXYcoord(float CornerDis, float CornerAngle, float *X, float *Y) {
	const double Deg_Rad_Ratio = 57.2957795131;
	float XaxisRelativeAngle = (fabs(CornerAngle) - 90) / Deg_Rad_Ratio;

	*X = CornerDis * cos(XaxisRelativeAngle);
	*Y = CornerDis * sin(XaxisRelativeAngle);
}

void LidarReader::localize() {
	static float DL_CornerDis = 0;
	static float DR_CornerDis = 0;
	static float DL_CornerAngle = 0;
	static float DR_CornerAngle = 0;

	static float tempX1 = 0;
	static float tempX2 = 0;
	static float tempY1 = 0;
	static float tempY2 = 0;

	FindLargestDis_AngleInRange(&DL_CornerDis, &DL_CornerAngle, -90, 180);
	FindLargestDis_AngleInRange(&DR_CornerDis, &DR_CornerAngle, 90, 180);

	CalculateXYcoord(DL_CornerDis, fabs(DL_CornerAngle), &tempX1, &tempY1);
	CalculateXYcoord(DR_CornerDis, fabs(DR_CornerAngle), &tempX2, &tempY2);

	if (!((tempY1 + tempY2 < fieldState.length) && (tempX1 + tempX2 < fieldState.width))) {
		if (tempY1 > tempY2) {
			currentY = tempY1;
			currentX = tempX1;
		} else if (tempY1 < tempY2) {
			currentY = tempY2;
			currentX = fieldState.width - tempX2;
		} else if (DL_CornerDis > DR_CornerDis) {
			currentY = tempY1;
			currentX = tempX1;
		} else {
			currentY = tempY2;
			currentX = fieldState.width - tempX2;
		}
	}
}
