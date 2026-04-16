#include "bmp390.h"
#include <math.h>

//static means read calib func is private to bmp390.c, static at file scope means internal linkage
//	Any helper function not apart of public API in header should be static
static HAL_StatusTypeDef BMP390_ReadCalib(I2C_HandleTypeDef *hi2c, BMP390_Calib_t *cal) {
	uint8_t raw[21]; //21 calibration bytes in consecutive registers starting at 0x31. One burst read gets all in one I2C transaction instead of 21 separate reads

	HAL_StatusTypeDef ret; //stores the return value of each I2C call

	ret = I2C_ReadReg(hi2c, BMP390_ADDR, BMP390_CALIB_0, raw, 21);
	if (ret != HAL_OK) return ret; //error code, early return keeps success path flat (not nested

	//BMP390 compensation math - datasheet table 5
	uint16_t t1 = (uint16_t)(raw[1] << 8) | raw[0];
	uint16_t t2 = (uint16_t)(raw[3] << 8) | raw[2];
	int8_t   t3 = (int8_t)raw[4]; //note the signed int

    int16_t  p1  = (int16_t) ((raw[6]  << 8) | raw[5]);
    int16_t  p2  = (int16_t) ((raw[8]  << 8) | raw[7]);
    int8_t   p3  = (int8_t)raw[9];
    int8_t   p4  = (int8_t)raw[10];
    uint16_t p5  = (uint16_t)((raw[12] << 8) | raw[11]);
    uint16_t p6  = (uint16_t)((raw[14] << 8) | raw[13]);
    int8_t   p7  = (int8_t)raw[15];
    int8_t   p8  = (int8_t)raw[16];
    int16_t  p9  = (int16_t) ((raw[18] << 8) | raw[17]);
    int8_t   p10 = (int8_t)raw[19];
    int8_t   p11 = (int8_t)raw[20];

    /*
     * if you have struct directly
     * BMP390_Calib_t cal;
     * cal.par_t1 = 1.0f; dot notation
     *
     * if you have pointer to the struct
     * BMP390_Calib_t *cal;
     * cal->par_t1 = 1.0f; arrow notation
     *
     * -> dereferences the pointer and then accesses the member
     * cal->par_t1 = 1.0f;
     * same as
     * (*cal).par_t1 = 1.0f;
     *
     *
     * THE POINT is that we define the structs parameter as a pointer so that we only pass a 4byte address regardless how big the struct is
     */

    //powf to stay iin signle percision, not pow
    //	powf(2.0f, -8.0f) etc are fixed-point scaling factors from datasheet, calibration coeff stored as int to save NVM space, so we divide by 2 to convert to floating point range the compensation equations expect. PORTABILITY
    cal->par_t1 = (float)t1 / powf(2.0f, -8.0f);
	cal->par_t2 = (float)t2 / powf(2.0f,  30.0f);
	cal->par_t3 = (float)t3 / powf(2.0f,  48.0f);

	cal->par_p1  = ((float)p1  - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
	cal->par_p2  = ((float)p2  - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
	cal->par_p3  = (float)p3  / powf(2.0f, 32.0f);
	cal->par_p4  = (float)p4  / powf(2.0f, 37.0f);
	cal->par_p5  = (float)p5  / powf(2.0f, -3.0f);
	cal->par_p6  = (float)p6  / powf(2.0f,  6.0f);
	cal->par_p7  = (float)p7  / powf(2.0f,  8.0f);
	cal->par_p8  = (float)p8  / powf(2.0f, 15.0f);
	cal->par_p9  = (float)p9  / powf(2.0f, 48.0f);
	cal->par_p10 = (float)p10 / powf(2.0f, 48.0f);
	cal->par_p11 = (float)p11 / powf(2.0f, 65.0f);

	return HAL_OK;


}

//reference bmp390 datasheet bosch
HAL_StatusTypeDef BMP390_Init(I2C_HandleTypeDef *hi2c){
	HAL_StatusTypeDef ret; //ret is a 4 byte int (enum 0,1,2,3), typdef wraps enums for readability
	uint8_t val;

	//use ret for ID checks
	ret = I2C_ReadReg(hi2c, BMP390_ADDR, BMP390_CHIP_ID_REG, &val, 1);
	if (ret != HAL_OK) return ret;
	if (val != BMP390_CHIP_ID_VAL) return HAL_ERROR;

	//soft reset
	ret = I2C_WriteReg(hi2c, BMP390_ADR, BMP390_CMD, 0xB6);
	if (ret != HAL_OK) return ret;
	HAL_Delay(10);

	//oversampling: pg 37 011 pressure x8
	ret = I2C_WriteReg(hi2c, BMP390_ADDR, BMP390_OSR, 0x03);
	if (ret != HAL_OK) return ret;

	//output data rate: 50 Hz pg 38 0x02 for ODR 50Hz and 20 ms sampling period
	ret = I2C_WriteReg(hi2c, BMP390_ADDR, BMP390_ODR, 0x02);
	if (ret != HAL_OK) return ret;

	//IIR filter coeff 3 - 010 Register 0x1F
	ret = I2C_WriteReg(hi2c, BMP390_ADDR, BMP390_CONFIG, 0x02)

	//pg 36 Enable pressure + temperature, normal mode = 110011
	ret = I2C_WriteReg(hi2c, BMP390_ADDR, BMP390_PWR_CTRL, 0x33);
	if (ret != HAL_OK) return ret;

	return HAL_OK;
}


static float BMP390_CompensateTemp(uint32_t raw_temp, BMP390_Calib_t *cal){
	float partial_data1 = (float) raw_temp - cal->par_t1; //calib temp by subtracting dt of chip from zero temp (par_t1) by the raw ADC value
	float partial_data2 = partial_data1 * cal->par_t2; //calib temp by scaling temp by the gain (par_t2 = degrees per ADC count)
	cal->t_lin			= partial_data2 + 
}

