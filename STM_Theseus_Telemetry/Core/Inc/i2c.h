// Include Guard - Prevents compiler from including header twice in event that multiple c files include it
#ifndef INC_I2C_H_
#define INC_I2c_H_

HAL_StatusTypeDef I2C_ReadReg(I2C_HandleTypeDef *hi2c,
							   uint8_t dev_addr,
							   uint8_t reg_addr,
							   uint8_t *data,
							   uint16_t len);

HAL_StatusTypeDef I2C_WriteReg(I2C_HandleTypeDef *hi2c,
							   uint8_t dev_addr,
							   uint8_t reg_addr,
							   uint8_t data);

HAL_StatusTypeDef I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
									 uint8_t dev_addr);

#endif /* INC_I2C_H_ */
