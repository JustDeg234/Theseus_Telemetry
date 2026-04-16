#include "i2c.h"

//Board Support Package

HAL_StatusTypeDef I2C_ReadReg(I2C_HandleTypeDef *hi2c,
							   uint8_t dev_addr,
							   uint8_t reg_addr,
							   uint8_t *data,
							   uint16_t len)
{
	return HAL_I2C_Mem_Read(hi2c,
							 dev_addr,
							 reg_addr,
							 I2C_MEMADD_SIZE_8BIT,
							 data,
							 len,
							 100);
}

HAL_StatusTypeDef I2C_WriteReg(I2C_HandleTypeDef *hi2c,
								uint8_t dev_addr,
								uint8_t reg_addr,
								uint8_t data)
{
	return HAL_I2C_Mem_Write(hi2c,
			  	  	  	  	  dev_addr,
							  reg_addr,
							  I2C_MEMADD_SIZE_8BIT,
							  &data,
							  1,
							  100);
}

HAL_StatusTypeDef I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
									 uint8_t dev_addr)
{
	return HAL_I2C_IsDeviceReady(hi2c, dev_addr, 3, 100); //3 attempts with 100 ms delay before giving up
}
