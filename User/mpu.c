#include "mpu.h"
#include "main.h" // For MPU_CS

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00 
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
 
#define ACC_FULL_SCALE_2_G 0x00 
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

static void MpuRegReadDataI2C(MpuDevice *dev, uint8_t reg, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c,
        dev->address, reg, I2C_MEMADD_SIZE_8BIT,
        data, size, 0xffff);

    if (status != HAL_OK) {
        ;
    }
}

static void MpuRegWriteDataI2C(MpuDevice *dev, uint8_t reg, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c,
        dev->address, reg, I2C_MEMADD_SIZE_8BIT,
        data, size, 0xffff);

    if (status != HAL_OK) {
        ;
    }
}

static void SpiCsOn(MpuDevice *dev) {
    HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
    (void) dev;
}

static void SpiCsOff(MpuDevice *dev) {
    HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
    (void) dev;
}

static uint8_t SpiXferByte(MpuDevice *dev, uint8_t val) {
    uint8_t res = 0xff;
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->hspi,
        &val, &res, 1, 0x1000);
    return (status == HAL_OK) ? res : 0xff;
}

static void MpuRegReadDataSPI(MpuDevice *dev, uint8_t reg, uint8_t *data, uint16_t size) {
    SpiCsOn(dev);
    SpiXferByte(dev, reg | 0x80);
    while (size) {
        *data = SpiXferByte(dev, 0x00);
        --size;
        ++data;
    }
    SpiCsOff(dev);
}

static void MpuRegWriteDataSPI(MpuDevice *dev, uint8_t reg, uint8_t *data, uint16_t size) {
    SpiCsOn(dev);
    SpiXferByte(dev, reg);
    while (size) {
        SpiXferByte(dev, *data);
        --size;
        ++data;
    }
    SpiCsOff(dev);
}

static void MpuRegReadData(MpuDevice *dev, uint8_t reg, uint8_t *data, uint16_t size) {
    //MpuRegReadDataI2C(dev, reg, data, size);
    MpuRegReadDataSPI(dev, reg, data, size);
}

static void MpuRegWriteData(MpuDevice *dev, uint8_t reg, uint8_t *data, uint16_t size) {
    //MpuRegWriteDataI2C(dev, reg, data, size);
    MpuRegWriteDataSPI(dev, reg, data, size);
}

static void MpuRegRead(MpuDevice *dev, uint8_t reg, uint8_t *val) {
    MpuRegReadData(dev, reg, val, sizeof *val);
}

static void MpuRegWrite(MpuDevice *dev, uint8_t reg, uint8_t val) {
    MpuRegWriteData(dev, reg, &val, sizeof val);
}

static void MpuInitCommon(MpuDevice *dev) {
    // Set accelerometers low pass filter at 5Hz
    MpuRegWrite(dev, 29, 0x06);
    // Set gyroscope low pass filter at 5Hz
    MpuRegWrite(dev, 26, 0x06);

    // Configure accelerometers range
    MpuRegWrite(dev, 28, ACC_FULL_SCALE_4_G);
    dev->accelRange = 4.0;
    // Configure gyroscope range
    MpuRegWrite(dev, 27, GYRO_FULL_SCALE_1000_DPS);
    dev->gyroRange = 1000.0;
    // Set by pass mode for the magnetometers
    MpuRegWrite(dev, 0x37, 0x02);

    // Request continuous magnetometer measurements in 16 bits
    ///I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
}

void MpuInitI2C(MpuDevice *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;
    dev->address = MPU9250_ADDRESS;
    dev->hspi = NULL;
    MpuInitCommon(dev);
}

void MpuInitSPI(MpuDevice *dev, SPI_HandleTypeDef *hspi) {
    dev->hi2c = NULL;
    dev->hspi = hspi;
    MpuInitCommon(dev);
}

static inline int16_t I16(const uint8_t val[2]) {
    return val[0] << 8 | val[1];
}

void MpuReadID(MpuDevice *dev, uint8_t *id) {
    MpuRegRead(dev, 0x75, id);
}

void MpuRead(MpuDevice *dev, MpuData *data) {

    uint8_t buf[14];
    MpuRegReadData(dev, 0x3B, buf, sizeof buf);

    data->accel.x = I16(&buf[0]);
    data->accel.y = I16(&buf[2]);
    data->accel.z = I16(&buf[4]);

    data->gyro.x = I16(&buf[ 8]);
    data->gyro.y = I16(&buf[10]);
    data->gyro.z = I16(&buf[12]);

    data->mag.x = 8;
    data->mag.y = 1;
    data->mag.z = 42;

    data->temp = I16(&buf[6]);
}

static float MpuAccelGet(const MpuDevice *dev, const uint8_t val[2]) {
  int16_t v = I16(val);
  return ((float) -v) * dev->accelRange / 32768.0;
}

static float MpuGyroGet(const MpuDevice *dev, const uint8_t val[2]) {
  int16_t v = I16(val);
  return ((float) -v) * dev->gyroRange / 32768.0;
}

void MpuReadFloat(MpuDevice *dev, MpuDataFloat *data) {
    uint8_t buf[14];
    MpuRegReadData(dev, 0x3B, buf, sizeof buf);

    data->accel.x = MpuAccelGet(dev, &buf[0]);
    data->accel.y = MpuAccelGet(dev, &buf[2]);
    data->accel.z = MpuAccelGet(dev, &buf[4]);

    data->gyro.x = MpuGyroGet(dev, &buf[ 8]);
    data->gyro.y = MpuGyroGet(dev, &buf[10]);
    data->gyro.z = MpuGyroGet(dev, &buf[12]);

    data->mag.x = 8;
    data->mag.y = 1;
    data->mag.z = 42;

    data->temp = I16(&buf[6]);
}
