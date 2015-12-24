/*
 * mySCCB.h
 *
 *  Created on: Dec 22, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_CAMERA_SUPPS_MYSCCB_H_
#define HARDWARE_CAMERA_SUPPS_MYSCCB_H_
#include "../../../Basic/basic.h"

#define SCL_DIO			GPIO_024	//PB8 data
#define SCL_CLK			GPIO_025 	// PB9 clock
#define OUTPUT			1
#define INPUT			0

#define SCL_H         GPIOB->BSRRH = SCL_CLK	 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         GPIOB->BSRRL  = SCL_CLK  /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         GPIOB->BSRRH = SCL_DIO	 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         GPIOB->BSRRL  = SCL_DIO	 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      GPIOB->IDR  & SCL_CLK	 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      GPIOB->IDR  & SCL_DIO	 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */

#define ADDR_OV7670   0x42


class mySCCB {
public:
	mySCCB();
	virtual ~mySCCB();
	void init();
	int writeByte( uint16_t WriteAddress , uint8_t SendByte , uint8_t DeviceAddress);
	int readByte(uint8_t* pBuffer,   uint16_t length,   uint8_t ReadAddress,  uint8_t DeviceAddress);
private:
	int setDataDirection(int dir);
	void myDelay();
	int start();
	void stop();
	void ack();
	void nack();
	int waitACK();
	void sendByte(uint8_t sendByte);
	uint8_t receiveByte();
};

#endif /* HARDWARE_CAMERA_SUPPS_MYSCCB_H_ */
