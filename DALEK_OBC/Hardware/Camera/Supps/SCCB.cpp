///*
// * SCCB.cpp
// *
// *  Created on: Nov 28, 2015
// *      Author: arthur
// */
//
//#include "SCCB.h"
//#include "inttypes.h"
//
////#include "stm32f4xx.h"
////#include "stm32f4xx_usart.h"
////#include "stm32f4xx_gpio.h"
////#include "stm32f4xx_rcc.h"
////#include "stm32f4xx_i2c.h"
//
//void SCCB::I2CInit(void) {
//	GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as I2C1SDA and I2C1SCL
//	I2C_InitTypeDef I2C_InitStructure; // this is for the I2C1 initilization
//
//	/* enable APB1 peripheral clock for I2C1*/
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
//
//	/* enable the peripheral clock for the pins used by
//	 PB6 for I2C SCL and PB9 for I2C1_SDL*/
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//
//	/* This sequence sets up the I2C1SDA and I2C1SCL pins
//	 * so they work correctly with the I2C1 peripheral
//	 */
//	GPIO_StructInit(&GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // Pins 10(I2C1_SCL) and 11(I2C1_SDA)
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // this defines the IO speed and has nothing to do with the baudrate!
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // this defines the output type as open drain
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // this activates the pullup resistors on the IO pins
//	GPIO_Init(GPIOB, &GPIO_InitStructure); // now all the values are passed to the GPIO_Init()
//
//	/* The I2C1_SCL and I2C1_SDA pins are now connected to their AF
//	 * so that the I2C1 can take over control of the
//	 * pins
//	 */
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); //
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
//
//	/* Configure I2C1 */
//	I2C_StructInit(&I2C_InitStructure);
//	I2C_DeInit(I2C1);
//
//	/* Enable the I2C peripheral */
//	I2C_Cmd(I2C1, ENABLE);
//
//	/* Set the I2C structure parameters */
//	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
//	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//	I2C_InitStructure.I2C_AcknowledgedAddress =
//	I2C_AcknowledgedAddress_7bit;
//	I2C_InitStructure.I2C_ClockSpeed = 100000;
//	/* I2C Peripheral Enable */
//	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
//	/* Initialize the I2C peripheral w/ selected parameters */
//	I2C_Init(I2C1, &I2C_InitStructure);
//	I2C_Cmd(I2C1, ENABLE);
//}
//
//void SCCB::I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
//	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
//		asm("nop");
//	I2C_GenerateSTART(I2Cx, ENABLE);
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
//		asm("nop");
//	I2C_Send7bitAddress(I2Cx, address, direction);
//	if (direction == I2C_Direction_Transmitter) {
//		while (!I2C_CheckEvent(I2Cx,
//		I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//			asm("nop");
//	} else if (direction == I2C_Direction_Receiver) {
//		while (!I2C_CheckEvent(I2Cx,
//		I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
//			asm("nop");
//	}
//}
//
//void SCCB::I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
//	I2C_SendData(I2Cx, data);
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//		asm("nop");
//}
//
//uint8_t SCCB::I2C_read_ack(I2C_TypeDef* I2Cx) {
//	uint8_t data;
//	I2C_AcknowledgeConfig(I2Cx, ENABLE);
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
//		asm("nop");
//	data = I2C_ReceiveData(I2Cx);
//	return data;
//}
//
//uint8_t SCCB::I2C_read_nack(I2C_TypeDef* I2Cx) {
//	uint8_t data;
//	I2C_AcknowledgeConfig(I2Cx, DISABLE);
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
//		asm("nop");
//	data = I2C_ReceiveData(I2Cx);
//	return data;
//}
//
//void SCCB::I2C_stop(I2C_TypeDef* I2Cx) {
//	I2C_GenerateSTOP(I2Cx, ENABLE);
//}
//
//uint8_t SCCB::ov7670_get(uint8_t reg) {
//	uint8_t data = 0;
//	delayx(1000);
//	I2C_start(I2C1, 0x42, I2C_Direction_Transmitter);
//	delayx(1000);
//	I2C_write(I2C1, reg);
//	delayx(1000);
//	I2C_stop(I2C1);
//	delayx(1000);
//	I2C_start(I2C1, 0x43, I2C_Direction_Receiver);
//	delayx(1000);
//	data = I2C_read_nack(I2C1);
//	delayx(1000);
//	I2C_stop(I2C1);
//	delayx(1000);
//	return data;
//}
//
//uint8_t SCCB::ov7670_set(uint8_t reg, uint8_t data) {
//	delayx(1000);
//	I2C_start(I2C1, 0x42, I2C_Direction_Transmitter);
//	delayx(1000);
//	I2C_write(I2C1, reg);
//	delayx(1000);
//	I2C_write(I2C1, data);
//	delayx(1000);
//	I2C_stop(I2C1);
//	delayx(1000);
//	return 0;
//}
//
//void SCCB::delayx(unsigned int ms) {
//	//4694 = 1 ms
//	while (ms > 1) {
//		ms--;
//		asm("nop");
//	}
//}
