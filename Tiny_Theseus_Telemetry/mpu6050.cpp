#include "mpu6050.h"

static void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static void readBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU6050_ADDR, (int)len);
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

bool MPU6050_Init(void) {
    Wire.beginTransmission(MPU6050_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) return false;

    writeReg(MPU6050_PWR_MGMT_1, 0x00);
    return true;
}

bool MPU6050_Read(MPU6050_Data_t *out) {
    uint8_t raw[6];

    readBurst(MPU6050_ACCEL_XOUT, raw, 6);
    out->ax = (float)((int16_t)(raw[0] << 8) | raw[1]) / MPU6050_ACCEL_SCALE;
    out->ay = (float)((int16_t)(raw[2] << 8) | raw[3]) / MPU6050_ACCEL_SCALE;
    out->az = (float)((int16_t)(raw[4] << 8) | raw[5]) / MPU6050_ACCEL_SCALE;

    readBurst(MPU6050_GYRO_XOUT, raw, 6);
    out->gx = (float)((int16_t)(raw[0] << 8) | raw[1]) / MPU6050_GYRO_SCALE;
    out->gy = (float)((int16_t)(raw[2] << 8) | raw[3]) / MPU6050_GYRO_SCALE;
    out->gz = (float)((int16_t)(raw[4] << 8) | raw[5]) / MPU6050_GYRO_SCALE;

    return true;
}