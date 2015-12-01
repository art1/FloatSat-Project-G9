/*
 * SCCB.h
 *
 *  Created on: Nov 28, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_CAMERA_SUPPS_SCCB_H_
#define HARDWARE_CAMERA_SUPPS_SCCB_H_

#include "../../../basic.h"
#include "stm32f4xx_i2c.h"

class SCCB{
public:
		SCCB();
		virtual ~SCCB();
	void init(void);
	void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
	void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
	uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
	uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
	void I2C_stop(I2C_TypeDef* I2Cx);
	void delayx(unsigned int ms);
	uint8_t get(uint8_t reg);


};





//
//#include "../../../basic.h"
//// RODOS i2c not working -> 9th bit is no NACK/ACK but is ignored -> implement sccb by myself
//#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_i2c.h"
//
//
//#define CAM_ADDR		0x21
//#define CAM_READ		0x43
//#define CAM_WRITE		0x42
//
//
//class SCCB {
//public:
//	SCCB();
//	virtual ~SCCB();
//	void init();
////	int camera_reg_write(uint8_t reg, uint8_t data, uint8_t len);
////	int camera_reg_read(uint8_t reg, uint8_t *dest, uint8_t len);
//private:
//	int retVal;
//	uint8_t recBuf[512];
//	uint8_t transBuf[512];
//    RCC_ClocksTypeDef RCC_Clocks;
//
//
//	int i2c_startCondition(uint8_t addr, uint8_t dir);
//	int i2c_stopCondition();
//	int i2c_writeReg(uint8_t reg, uint8_t data);
//	uint8_t i2c_readReg(uint8_t reg);
//	void delayUS(uint32_t us);
//
//	// context for initializing i2c channel 1 for cam SCCB bus
//	struct I2C1_Context{
//		I2C_IDX I2C_idx;
//		I2C_TypeDef* I2Cx;
//		uint32_t I2C_CLK; // RCC_APB1Periph_I2C1
//
//		uint16_t I2C_SCL_PIN;
//		GPIO_TypeDef* I2C_SCL_GPIO_PORT;
//		uint32_t I2C_SCL_GPIO_CLK; //RCC_AHB1Periph_GPIOA
//		uint8_t I2C_SCL_SOURCE;
//
//		uint16_t I2C_SDA_PIN;
//		GPIO_TypeDef* I2C_SDA_GPIO_PORT;
//		uint32_t I2C_SDA_GPIO_CLK;
//		uint8_t I2C_SDA_SOURCE;
//
//		uint8_t I2C_AF;
//		uint16_t I2C_SLAVE_ADDRESS7;
//		uint32_t I2C_SPEED;
//	}*context;
//
//};

#endif /* HARDWARE_CAMERA_SUPPS_SCCB_H_ */
