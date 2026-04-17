#pragma once
#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDR        0x68
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT  0x3B
#define MPU6050_GYRO_XOUT   0x43
#define MPU6050_WHO_AM_I    0x75

#define MPU6050_ACCEL_SCALE 16384.0f  // ±2g range
#define MPU6050_GYRO_SCALE  131.0f    // ±250 deg/s range

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} MPU6050_Data_t;

bool MPU6050_Init(void);
bool MPU6050_Read(MPU6050_Data_t *out);