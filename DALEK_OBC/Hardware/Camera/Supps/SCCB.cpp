//#include "stm32f4xx.h"
//#include "stm32f4xx_usart.h"
//#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_i2c.h"
#include "SCCB.h"

SCCB::SCCB(){

}

SCCB::~SCCB(){

}


void SCCB::init() {

	PRINTF("initialize SCCB\n");
	GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as I2C1SDA and I2C1SCL
	I2C_InitTypeDef I2C_InitStructure; // this is for the I2C1 initilization

	/* enable APB1 peripheral clock for I2C1*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 PB6 for I2C SCL and PB9 for I2C1_SDL*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the I2C1SDA and I2C1SCL pins
	 * so they work correctly with the I2C1 peripheral
	 */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // Pins 10(I2C1_SCL) and 11(I2C1_SDA)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // this defines the output type as open drain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStructure); // now all the values are passed to the GPIO_Init()

	/* The I2C1_SCL and I2C1_SDA pins are now connected to their AF
	 * so that the I2C1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	/* Configure I2C1 */
	I2C_StructInit(&I2C_InitStructure);
	I2C_DeInit(I2C1);

	/* Enable the I2C peripheral */
	I2C_Cmd(I2C1, ENABLE);

	/* Set the I2C structure parameters */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress =
	I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	/* I2C Peripheral Enable */
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
	/* Initialize the I2C peripheral w/ selected parameters */
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);






//    GPIO_InitTypeDef GPIO_InitStructure;
//	I2C_InitTypeDef I2C_InitStructure;
//
//
//	/* Enable GPIO clock */
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//
//	/* Enable UART clock */
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
//
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
//
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//	I2C_StructInit(&I2C_InitStructure);
//	I2C_InitStructure.I2C_ClockSpeed = 100000;
//	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
//	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//
//	I2C_ITConfig(I2C1, I2C_IT_ERR,ENABLE);
//
//	/* Enable I2C */
//	I2C_Cmd(I2C1, ENABLE);
//	I2C_Init(I2C1, &I2C_InitStructure);
}

void SCCB::I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
	PRINTF("Starting i2c thingy\n");
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		asm("nop");
	I2C_GenerateSTART(I2Cx, ENABLE);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		asm("nop");
	PRINTF("Sending Adress!\n");
	delayx(500);
	I2C_Send7bitAddress(I2Cx, address, direction);
	PRINTF("Adress sent, no setting directions n shit\n");
	delayx(500);

	uint32_t i2cEvent;


//	if (direction == I2C_Direction_Transmitter) {
//		PRINTF("im transmitter\n");
//		while (!(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == SUCCESS)) asm("nop");
//	} else {
//		PRINTF("im receiver\n");
//		while (!(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == SUCCESS)) asm("nop");
//	}
	delayx(1000);
	PRINTF("and now this fuck is here\n");
}

void SCCB::I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
	PRINTF("now sending data\n");
	I2C_SendData(I2Cx, data);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		asm("nop");
}

uint8_t SCCB::I2C_read_ack(I2C_TypeDef* I2Cx) {
	uint8_t data;
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
		asm("nop");
	data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t SCCB::I2C_read_nack(I2C_TypeDef* I2Cx) {
	uint8_t data;
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
		asm("nop");
	data = I2C_ReceiveData(I2Cx);
	return data;
}

void SCCB::I2C_stop(I2C_TypeDef* I2Cx) {
	I2C_GenerateSTOP(I2Cx, ENABLE);
}


uint8_t SCCB::get(uint8_t reg){
	uint8_t data = 0;
	I2C_start(I2C1, 0x42, I2C_Direction_Transmitter);
	PRINTF("now writing reg\n");
	I2C_write(I2C1, reg);
	I2C_stop(I2C1);
	PRINTF("delaying and starting again and send data\n");
	delayx(1000);
	I2C_start(I2C1, 0x43, I2C_Direction_Receiver);
	PRINTF("now read data and send nack\n");
	data = I2C_read_nack(I2C1);
	PRINTF("read data %d and now stpping\n",data);
	I2C_stop(I2C1);
	delayx(1000);
	return data;
}







void SCCB::delayx(unsigned int ms){
	while(ms > 1){
		ms--;
		asm("nop");
	}
}












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
//SCCB::SCCB() {
//	// TODO Auto-generated constructor stub
//
//}
//
//SCCB::~SCCB() {
//	// TODO Auto-generated destructor stub
//}
//
//void SCCB::init(){
//
//
//
//    /* Get system clocks */
//    RCC_GetClocksFreq(&RCC_Clocks);
//
//
//	context->I2Cx = I2C1;
//	context->I2C_CLK = RCC_APB1Periph_I2C1;
//	context->I2C_AF = GPIO_AF_I2C1;
//
//    /*
//     * Discovery uses PB6 & PB9 for DAC
//     */
//    /* SCL pin config -> possible pins: PB6 & PB8 */
//    context->I2C_SCL_GPIO_PORT = GPIOB;
//    context->I2C_SCL_GPIO_CLK = RCC_AHB1Periph_GPIOB;
//    //context->I2C_SCL_PIN = GPIO_Pin_6;
//    //context->I2C_SCL_SOURCE = GPIO_PinSource6;
//    context->I2C_SCL_PIN = GPIO_Pin_8;
//    context->I2C_SCL_SOURCE = GPIO_PinSource8;
//
//    /* SDA pin config -> possible pins: PB7 & PB9 */
//    context->I2C_SDA_GPIO_PORT = GPIOB;
//    context->I2C_SDA_GPIO_CLK = RCC_AHB1Periph_GPIOB;
//    //context->I2C_SDA_PIN = GPIO_Pin_7;
//    //context->I2C_SDA_SOURCE = GPIO_PinSource7;
//    context->I2C_SDA_PIN = GPIO_Pin_9;
//    context->I2C_SDA_SOURCE = GPIO_PinSource9;
//
//	retVal = 0;
//
//	// i2c init stuff
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//	/* ********************************************************* !!!!!!!!!!!!!!!!!!!!!!!!!!!! *******/
//	/* Changed I2C_InitStructure.I2C_Ack = I2C_Ack_Disable in hw_hal_i2c.cpp!!!!!!!!!!!!!!!!!       */
//
//	/* ********************************************************* !!!!!!!!!!!!!!!!!!!!!!!!!!!! *******/
////	i2c1.init(400000);
//
//
//
//	//400khz speed, fast i2c mode
//	context->I2C_SPEED = 400000;
//
//	/* enable peripheral clock for I2C module */
//	RCC_APB1PeriphClockCmd(context->I2C_CLK, ENABLE);
//
//	/* enable peripheral clock for I2C GPIOs */
//	RCC_AHB1PeriphClockCmd(context->I2C_SCL_GPIO_CLK | context->I2C_SDA_GPIO_CLK, ENABLE);
//
//	/* GPIO configuration */
//	/* Connect PXx to I2C_SCL*/
//	GPIO_PinAFConfig(context->I2C_SCL_GPIO_PORT, context->I2C_SCL_SOURCE, context->I2C_AF);
//	/* Connect PXx to I2C_SDA*/
//	GPIO_PinAFConfig(context->I2C_SDA_GPIO_PORT, context->I2C_SDA_SOURCE, context->I2C_AF);
//
//	/* Configure I2C pin: SCL */
//	GPIO_InitStructure.GPIO_Pin = context->I2C_SCL_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
////	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(context->I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
//
//	/* Configure I2C pin: SDA */
//	GPIO_InitStructure.GPIO_Pin = context->I2C_SDA_PIN;
//	GPIO_Init(context->I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
//
//	/* I2C configuration */
//	I2C_InitTypeDef I2C_InitStructure;
//	I2C_StructInit(&I2C_InitStructure);
//	I2C_Cmd(context->I2Cx, ENABLE);
//	I2C_DeInit(context->I2Cx);
//
//	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//	I2C_InitStructure.I2C_OwnAddress1 = context->I2C_SLAVE_ADDRESS7;
//	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;					// -> diable ack!
//	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStructure.I2C_ClockSpeed = context->I2C_SPEED;
//	I2C_Init(context->I2Cx, &I2C_InitStructure);
//
//	/* I2C module Enable */
//	I2C_Cmd(context->I2Cx, ENABLE);
//
//
//
//	// read reg 0x01, return should be 0x80
//	recBuf[0] = i2c_readReg(0x01);
//	PRINTF("retVal of 0x01 is %"PRIu8"\n",recBuf[0]);
//
//}
//
////int SCCB::camera_reg_read(uint8_t reg, uint8_t *dest, uint8_t len){
////	return i2c1.writeRead(CAM_READ,&reg,1,dest,len);
////
////}
////
////int SCCB::camera_reg_write(uint8_t reg, uint8_t data, uint8_t len){
////	transBuf[0] = reg;
////	transBuf[1] = data;
////	return i2c1.write(CAM_WRITE,&reg,2);
////}
//
//
///******************* I2C Re-implemented Functions, using I2C1 -> set this during init!! *************/
//int SCCB::i2c_startCondition(uint8_t addr, uint8_t dir){
//	/** TODO set I2Cx during init!*/
//	// first warten bis I2c nich mehr busy
//	PRINTF("Im here!\n");
//	while(I2C_GetFlagStatus(context->I2Cx,I2C_FLAG_BUSY)){
//		PRINTF("Flag status: %d\n",I2C_GetFlagStatus(context->I2Cx,I2C_FLAG_BUSY));
////		delayUS(50000000);
//		asm("nop");
//	}
//	PRINTF("Im here2!\n");
//	I2C_GenerateSTART(context->I2Cx,ENABLE);
//	// wait for EV5
//	PRINTF("Im here3!\n");
//	while(!I2C_CheckEvent(context->I2Cx,I2C_EVENT_MASTER_MODE_SELECT)){
//		PRINTF("Event status: %d\n",I2C_CheckEvent(context->I2Cx,I2C_EVENT_MASTER_MODE_SELECT));
////		delayUS(50000000);
//		asm("nop");
//	}
//	I2C_Send7bitAddress(context->I2Cx,addr,dir);
//	// wait for EV6 (ack from slave for receiving resp. sending direction -> see datasheet for better explaination)
//	if(dir == 0x01){ // direction receiver
//		while(!I2C_CheckEvent(context->I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
//	} else if (dir == 0x00){ // direction transmit
//		while(!I2C_CheckEvent(context->I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//	}
//	return 0;
//}
//
//
//int SCCB::i2c_stopCondition(){
//	I2C_GenerateSTOP(context->I2Cx,ENABLE);
//	return 0;
//}
//
//int SCCB::i2c_writeReg(uint8_t reg, uint8_t data){
//	i2c_startCondition(CAM_WRITE,I2C_Direction_Transmitter);
//	//send register
//	I2C_SendData(context->I2Cx,reg);
//	// wait until data is send (EV8_2)
//	while(!(I2C_CheckEvent(context->I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
//	/** TODO delay? */
//	// send data
//	I2C_SendData(context->I2Cx,data);
//	// wait until data is send (EV8_2)
//	while(!(I2C_CheckEvent(context->I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
//	i2c_stopCondition();
//	/** TODO implement check function , read out data again from reg? */
//	return 0;
//}
//
//uint8_t SCCB::i2c_readReg(uint8_t reg){
//	i2c_startCondition(CAM_WRITE,I2C_Direction_Transmitter);
//	// send the reg address
//	I2C_SendData(context->I2Cx,reg);
//	// wait until data is send (EV8_2)
//	while(!(I2C_CheckEvent(context->I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
//	i2c_stopCondition();
//	// start in receive mode and send nack (no requesting further bytes after one byte)
//	i2c_startCondition(CAM_READ,I2C_Direction_Receiver);
//	while(!(I2C_CheckEvent(context->I2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED)));
//	recBuf[0] = I2C_ReceiveData(context->I2Cx);
//	i2c_stopCondition();
//	return recBuf[0];
//}
//
//
//void SCCB::delayUS(uint32_t us){
//	us = us * (RCC_Clocks.HCLK_Frequency) / 4000000;
//	/** TODO calculate ticks for multiplication and division! */
//	while(us--);
//}
//
