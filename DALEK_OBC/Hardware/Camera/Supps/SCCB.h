/*
 * Sccb.h
 *
 *  Created on: 12.01.2015
 *      Author: Andreas Schartel
 */

#ifndef PERIPHERAL_SCCB_H_
#define PERIPHERAL_SCCB_H_

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"

class Sccb {
private:
	void delayx(unsigned int ms);
	void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
	void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
	uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
	uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
	void I2C_stop(I2C_TypeDef* I2Cx);
public:
	void I2CInit(void);
	uint8_t ov7670_get(uint8_t reg);
	uint8_t ov7670_set(uint8_t reg, uint8_t data);
};

#endif /* PERIPHERAL_SCCB_H_ */
