#include "bmp390.h"

static BMP390_Calib_t _cal;

static void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(BMP390_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(BMP390_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP390_ADDR, 1);
    return Wire.read();
}

static void readBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(BMP390_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP390_ADDR, (int)len);
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

static void readCalib(void) {
    uint8_t raw[21];
    readBurst(BMP390_CALIB_0, raw, 21);

    uint16_t t1 = (uint16_t)(raw[1] << 8) | raw[0];
    uint16_t t2 = (uint16_t)(raw[3] << 8) | raw[2];
    int8_t   t3 = (int8_t)raw[4];

    int16_t  p1  = (int16_t)((raw[6]  << 8) | raw[5]);
    int16_t  p2  = (int16_t)((raw[8]  << 8) | raw[7]);
    int8_t   p3  = (int8_t)raw[9];
    int8_t   p4  = (int8_t)raw[10];
    uint16_t p5  = (uint16_t)((raw[12] << 8) | raw[11]);
    uint16_t p6  = (uint16_t)((raw[14] << 8) | raw[13]);
    int8_t   p7  = (int8_t)raw[15];
    int8_t   p8  = (int8_t)raw[16];
    int16_t  p9  = (int16_t)((raw[18] << 8) | raw[17]);
    int8_t   p10 = (int8_t)raw[19];
    int8_t   p11 = (int8_t)raw[20];

    _cal.par_t1 = (float)t1 / powf(2.0f, -8.0f);
    _cal.par_t2 = (float)t2 / powf(2.0f,  30.0f);
    _cal.par_t3 = (float)t3 / powf(2.0f,  48.0f);

    _cal.par_p1  = ((float)p1  - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
    _cal.par_p2  = ((float)p2  - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
    _cal.par_p3  = (float)p3  / powf(2.0f, 32.0f);
    _cal.par_p4  = (float)p4  / powf(2.0f, 37.0f);
    _cal.par_p5  = (float)p5  / powf(2.0f, -3.0f);
    _cal.par_p6  = (float)p6  / powf(2.0f,  6.0f);
    _cal.par_p7  = (float)p7  / powf(2.0f,  8.0f);
    _cal.par_p8  = (float)p8  / powf(2.0f, 15.0f);
    _cal.par_p9  = (float)p9  / powf(2.0f, 48.0f);
    _cal.par_p10 = (float)p10 / powf(2.0f, 48.0f);
    _cal.par_p11 = (float)p11 / powf(2.0f, 65.0f);
}

static float compensateTemp(uint32_t raw_temp) {
    float p1   = (float)raw_temp - _cal.par_t1;
    float p2   = p1 * _cal.par_t2;
    _cal.t_lin = p2 + (p1 * p1) * _cal.par_t3;
    return _cal.t_lin;
}

static float compensatePress(uint32_t raw_press) {
    float p1   = _cal.par_p6 * _cal.t_lin;
    float p2   = _cal.par_p7 * (_cal.t_lin * _cal.t_lin);
    float p3   = _cal.par_p8 * (_cal.t_lin * _cal.t_lin * _cal.t_lin);
    float out1 = _cal.par_p5 + p1 + p2 + p3;

    p1         = _cal.par_p2 * _cal.t_lin;
    p2         = _cal.par_p3 * (_cal.t_lin * _cal.t_lin);
    p3         = _cal.par_p4 * (_cal.t_lin * _cal.t_lin * _cal.t_lin);
    float out2 = (float)raw_press * (_cal.par_p1 + p1 + p2 + p3);

    p1         = (float)raw_press * (float)raw_press;
    p2         = _cal.par_p9 + _cal.par_p10 * _cal.t_lin;
    p3         = p1 * p2;
    float out3 = p3 + ((float)raw_press * (float)raw_press * (float)raw_press) * _cal.par_p11;

    return out1 + out2 + out3;
}

bool BMP390_Init(void) {
    uint8_t id = readReg(BMP390_CHIP_ID_REG);
    if (id != BMP390_CHIP_ID_VAL) return false;

    writeReg(BMP390_CMD, 0xB6);
    delay(10);

    writeReg(BMP390_OSR,     0x03);
    writeReg(BMP390_ODR,     0x02);
    writeReg(BMP390_CONFIG,  0x04);
    writeReg(BMP390_PWR_CTRL, 0x33);

    readCalib();
    return true;
}

bool BMP390_Read(BMP390_Data_t *out) {
    uint8_t status = readReg(BMP390_STATUS);
    if (!(status & BMP390_DRDY_PRESS) || !(status & BMP390_DRDY_TEMP))
        return false;

    uint8_t raw[6];
    readBurst(BMP390_DATA_0, raw, 6);

    uint32_t raw_press = ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | raw[0];
    uint32_t raw_temp  = ((uint32_t)raw[5] << 16) | ((uint32_t)raw[4] << 8) | raw[3];

    out->temperature_c = compensateTemp(raw_temp);
    out->pressure_pa   = compensatePress(raw_press);

    float ratio      = (out->pressure_pa / 100.0f) / 1013.25f;
    out->altitude_m  = 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));

    return true;
}