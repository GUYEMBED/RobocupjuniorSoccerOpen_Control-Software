// fieldData.h
#ifndef FIELDDATA_H
#define FIELDDATA_H

class fieldData {
public:
	float width;
	float length;
	float GoalCornerX;
	float GoalCornerY;
	float BorderWidth;
	float BorderLength;

	fieldData();
	void update(float _width, float _length, float _GoalCornerX, float _GoalCornerY, float _borderWidth, float _borderLength);
};

extern fieldData fieldState;

#endif // FIELDDATA_H
