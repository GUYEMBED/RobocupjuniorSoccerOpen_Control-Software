#include "bno05.h"
#include "string.h"

static int BNO05_WriteReg(BNO05_HandleTypeDef *bno, uint8_t reg, uint8_t data) {
	return HAL_I2C_Mem_Write(bno->hi2c, bno->address, reg, 1, &data, 1, HAL_MAX_DELAY) == HAL_OK ? BNO05_OK : BNO05_ERROR;
}

static int BNO05_ReadReg(BNO05_HandleTypeDef *bno, uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Read(bno->hi2c, bno->address, reg, 1, data, 1, HAL_MAX_DELAY) == HAL_OK ? BNO05_OK : BNO05_ERROR;
}

static int BNO05_ReadRegs(BNO05_HandleTypeDef *bno, uint8_t reg, uint8_t *buffer, uint8_t length) {
	return HAL_I2C_Mem_Read(bno->hi2c, bno->address, reg, 1, buffer, length, HAL_MAX_DELAY) == HAL_OK ? BNO05_OK : BNO05_ERROR;
}

int BNO05_Init(BNO05_HandleTypeDef *bno) {
	uint8_t id = 0;

	// Check chip ID
	if (BNO05_ReadReg(bno, BNO05_CHIP_ID_ADDR, &id) != BNO05_OK) {
		return BNO05_ERROR;
	}
	if (id != BNO05_CHIP_ID) {
		HAL_Delay(1000);  // Sensor might still be starting up, wait and retry once
		if (BNO05_ReadReg(bno, BNO05_CHIP_ID_ADDR, &id) != BNO05_OK || id != BNO05_CHIP_ID) {
			return BNO05_ERROR;
		}
	}

	// Set to config mode to configure sensor
	if (BNO05_WriteReg(bno, BNO05_OPR_MODE_ADDR, BNO05_OPR_MODE_CONFIG) != BNO05_OK)
		return BNO05_ERROR;
	HAL_Delay(25);

	// Reset system trigger register (optional, can do soft reset here if needed)
	if (BNO05_WriteReg(bno, BNO05_SYS_TRIGGER_ADDR, 0x00) != BNO05_OK)
		return BNO05_ERROR;
	HAL_Delay(10);

	// Set power mode to normal
	if (BNO05_WriteReg(bno, BNO05_PWR_MODE_ADDR, BNO05_PWR_MODE_NORMAL) != BNO05_OK)
		return BNO05_ERROR;
	HAL_Delay(10);

	// Select page 0 (default)
	if (BNO05_WriteReg(bno, BNO05_PAGE_ID_ADDR, 0x00) != BNO05_OK)
		return BNO05_ERROR;
	HAL_Delay(10);

	// Set operation mode to NDOF (Fusion mode with accel, gyro, magnetometer)
	if (BNO05_WriteReg(bno, BNO05_OPR_MODE_ADDR, BNO05_OPR_MODE_NDOF) != BNO05_OK)
		return BNO05_ERROR;
	HAL_Delay(20);

	return BNO05_OK;
}

int BNO05_ReadQuaternion(BNO05_HandleTypeDef *bno, BNO05_Quaternion *quat) {
	uint8_t buffer[8];
	if (BNO05_ReadRegs(bno, BNO05_QUATERNION_DATA_W_LSB, buffer, 8) != BNO05_OK)
		return BNO05_ERROR;

	// Convert bytes to int16_t (LSB first)
	int16_t qw = (int16_t) (buffer[1] << 8 | buffer[0]);
	int16_t qx = (int16_t) (buffer[3] << 8 | buffer[2]);
	int16_t qy = (int16_t) (buffer[5] << 8 | buffer[4]);
	int16_t qz = (int16_t) (buffer[7] << 8 | buffer[6]);

	// Scale according to datasheet (1 unit = 1/16384)
	const float scale = 1.0f / 16384.0f;
	quat->w = qw * scale;
	quat->x = qx * scale;
	quat->y = qy * scale;
	quat->z = qz * scale;

	return BNO05_OK;
}
