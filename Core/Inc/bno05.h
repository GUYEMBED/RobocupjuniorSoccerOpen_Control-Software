#ifndef BNO05_H
#define BNO05_H

#include "stm32f4xx_hal.h"  // Change to your MCU family HAL

// I2C address of BNO05 (same as BNO055, assuming you keep same hardware)
#define BNO05_ADDRESS 0x29 << 1  // STM HAL requires 8-bit shifted address

// Register addresses (only essential ones)
#define BNO05_CHIP_ID_ADDR            0x00
#define BNO05_OPR_MODE_ADDR           0x3D
#define BNO05_PWR_MODE_ADDR           0x3E
#define BNO05_SYS_TRIGGER_ADDR        0x3F
#define BNO05_PAGE_ID_ADDR            0x07

// Operation modes
#define BNO05_OPR_MODE_CONFIG         0x00
#define BNO05_OPR_MODE_NDOF           0x0C

// Power modes
#define BNO05_PWR_MODE_NORMAL         0x00

// Data registers for quaternion (w,x,y,z - each 2 bytes, LSB first)
#define BNO05_QUATERNION_DATA_W_LSB   0x20

// Chip ID value
#define BNO05_CHIP_ID                 0xA0

// Return codes
#define BNO05_OK                     0
#define BNO05_ERROR                 -1

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
} BNO05_HandleTypeDef;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} BNO05_Quaternion;

int BNO05_Init(BNO05_HandleTypeDef *bno);
int BNO05_ReadQuaternion(BNO05_HandleTypeDef *bno, BNO05_Quaternion *quat);

#endif
