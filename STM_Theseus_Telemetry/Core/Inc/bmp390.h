// Include Guards
#ifndef INC_BMP390_H
#define INC_BMP390_H

#include "i2c.h"
//#define removes magic numbers, which helps clarify typos by returning a compiler error instead of silently compiiling and failing at runtime
//#define directives create macros that are preprocessor-level instructions replaced by their defeined content BEFORE compilation
#define BMP390_ADDR		(0x77 << 1) //0111_0111  -> 1110_1110 to 0xEE
#define BMP390_CHIP_ID_REG	0x00 //0000_0000
#define BMP390_CHIP_ID_VAL  0x60 //0110_0000 , 0x50 on BMP388, stored on NVM
#define BMP390_CMD			0x7E //0111_1110
#define BMP390_PWR_CTRL		0x1B //0001_1011
#define BMP390_OSR			0x1C //0001_1100
#define BMP390_ODR			0x1D //0001_1101
#define BMP390_CONFIG		0x1F //0001_1111
#define BMP390_STATUS		0x03 //0000_0011
#define BMP390_DATA_0		0x04 //0000_0100
#define BMP390_CALIB_0		0x31 //0011_0001

#define BMP390_DRDY_PRESS	(1 << 5) //0010_0000
#define BMP390_DRDY_TEMP	(1 << 6) //0100_0000

// _t suffix means type in other codebases, here ST uses typedef
typedef struct {
	//3 temperature compensation coefficients from NVM
	float par_t1;
	float par_t2;
	float par_t3;
	//11 pressure compensation coefficients from NVM
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
	float t_lin;
} BMP390_Calib_t; //read once at init and stored in struct, every subsequent read uses them to convert raw ADC counts to real units (filtering)

typedef struct {
	float temperature_c;
	float pressure_pa;
	float altitude_m;
} BMP390_Data_t;

//I2C public API
HAL_StatusTypeDef BMP390_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMP390_Read(I2C_HandleTypeDef *hi2c, BMP390_Data_t *out); //pointer to data struct passed to be filled by READ function


#endif /* INC_BMP390_H */
