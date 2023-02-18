#ifndef INCLUDE_GUARD_MPU_H
#define INCLUDE_GUARD_MPU_H

#include "stm32f4xx_hal.h"

typedef struct MpuDevice {
    I2C_HandleTypeDef *hi2c;
    uint16_t address;
    SPI_HandleTypeDef *hspi;
    float accelRange;
    float gyroRange;
} MpuDevice;

typedef struct MpuXYZ {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} MpuXYZ;

typedef struct MpuData {
    MpuXYZ accel;
    MpuXYZ gyro;
    MpuXYZ mag;
    uint16_t temp;
} MpuData;

typedef struct MpuFloatXYZ {
    float x;
    float y;
    float z;
} MpuFloatXYZ;

typedef struct MpuDataFloat {
    MpuFloatXYZ accel;
    MpuFloatXYZ gyro;
    MpuFloatXYZ mag;
    uint16_t temp;
} MpuDataFloat;

void MpuInitI2C(MpuDevice *dev, I2C_HandleTypeDef *hi2c);
void MpuInitSPI(MpuDevice *dev, SPI_HandleTypeDef *hspi);
void MpuReadID(MpuDevice *dev, uint8_t *id);
void MpuRead(MpuDevice *dev, MpuData *data);
void MpuReadFloat(MpuDevice *dev, MpuDataFloat *data);

#endif /* INCLUDE_GUARD_MPU_H */
