#pragma once
#include <Arduino.h>
#include <Wire.h>

#define BMP390_ADDR         0x77
#define BMP390_CHIP_ID_REG  0x00
#define BMP390_CHIP_ID_VAL  0x60
#define BMP390_CMD          0x7E
#define BMP390_PWR_CTRL     0x1B
#define BMP390_OSR          0x1C
#define BMP390_ODR          0x1D
#define BMP390_CONFIG       0x1F
#define BMP390_STATUS       0x03
#define BMP390_DATA_0       0x04
#define BMP390_CALIB_0      0x31

#define BMP390_DRDY_PRESS   (1 << 5)
#define BMP390_DRDY_TEMP    (1 << 6)

typedef struct {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4;
    float par_p5, par_p6, par_p7, par_p8;
    float par_p9, par_p10, par_p11;
    float t_lin;
} BMP390_Calib_t;

typedef struct {
    float temperature_c;
    float pressure_pa;
    float altitude_m;
} BMP390_Data_t;

bool BMP390_Init(void);
bool BMP390_Read(BMP390_Data_t *out);