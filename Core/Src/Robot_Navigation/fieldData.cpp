#include "fieldData.h"

fieldData::fieldData() {
}
void fieldData::update(float _width, float _length, float _GoalCornerX, float _GoalCornerY, float _borderWidth, float _borderLength) {
	width = _width;
	length = _length;
	BorderWidth = _borderWidth;
	BorderLength = _borderLength;
	GoalCornerX = _GoalCornerX;
	GoalCornerY = _GoalCornerY;
}
