/*
 * SCCB.cpp
 *
 *  Created on: Nov 28, 2015
 *      Author: arthur
 */

#include "SCCB.h"
#include "inttypes.h"

//#include "stm32f4xx.h"
//#include "stm32f4xx_usart.h"
//#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_i2c.h"

SCCB::SCCB(){

}

SCCB::~SCCB(){

}


void SCCB::init() {
	PRINTF("initialize SCCB\n");
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as clock and data pins for sccb
	GPIO_InitStructure.GPIO_Pin = SIO_C | SIO_D;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}



int SCCB::writeReg(uint8_t reg, uint8_t *data,int dataBytesToWrite){
//	int k = 0;
//	startCondition();
//	/** TODO check the return codes for errors and then abort communication! */
//	k = write(CAM_WRITE);
//	delay_microseconds(SomeArbitraryDelayValue);
//	k |= write(reg);
//	for(int i =0;i<dataBytesToWrite;i++){
//		k |= write(data[i]);
//	}
//	stopCondition();
//	return k;

}

uint8_t SCCB::readReg(uint8_t reg){
//	PRINTF("now going high\n");
//	SIO_D_High;
//	delay_microseconds(10000);
//	PRINTF("now going low\n");
//	SIO_D_Low;

	int k=0;
	uint8_t ret = 0x00;
	startCondition();
	k = write(CAM_WRITE);
	delay_microseconds(SomeArbitraryDelayValue);
	k |= write(reg);
	stopCondition();
	delay_microseconds(SomeArbitraryDelayValue); // delay min 1.3 us before restart
	startCondition();
	k |= write(CAM_READ);
	delay_microseconds(SomeArbitraryDelayValue);
	PRINTF("k is here %d\n",k);
	ret = read();
	stopCondition();
	return ret;
}

void SCCB::startCondition(){
	setDataDirection(SIO_OUT);
	SIO_D_High;
	delay_microseconds(SomeArbitraryDelayValue); // setup time braucht mindestens 600ns!
	SIO_C_High;
	delay_microseconds(SomeArbitraryDelayValue); // mindestens 600ns halten!
	SIO_D_Low;
	delay_microseconds(SomeArbitraryDelayValue);
	SIO_C_Low;
	delay_microseconds(SomeArbitraryDelayValue);
}

void SCCB::stopCondition(){
	setDataDirection(SIO_OUT);
	SIO_D_Low;
	delay_microseconds(SomeArbitraryDelayValue);
	SIO_C_High;
	delay_microseconds(SomeArbitraryDelayValue); // mindestens 600ns für stop condition setup time!
	SIO_D_High;
	delay_microseconds(SomeArbitraryDelayValue);
}

/**
 * sets the SIO Data Pin direction
 * params:	1	Output
 * 			0	Input
 * return:  0	SUCCESS
 * 			-1	ERROR
 */
int SCCB::setDataDirection(int dir){
	if((dir < 0) || (dir > 1) ) return -1;
	if(dir == 1){ // then output
		GPIO_Struct.GPIO_Pin = SIO_D;
		GPIO_Struct.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_Struct.GPIO_OType = GPIO_OType_PP; // opendrain
		GPIO_Struct.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;// pullup already on chip?
	} else {
		GPIO_Struct.GPIO_Pin = SIO_D;
		GPIO_Struct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Struct.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup already on chip?
	}
	GPIO_Init(GPIOB,&GPIO_Struct);
	return 0;
}

void SCCB::delay_microseconds(uint16_t t){
	/** TODO proper timer delay function*/
	// assembler magic happens here (-> andy )
//	//4694 = 1 ms
//	while (t > 1) {
//		t--;
//		asm("nop");
//	}

	   uint16_t i = 0;
	   while(t--)
	   {
		   // Calculate for 168MHz sysclk
		   i = 5;	//72Mhz i = 10		35
		   while(i--) ;
	   }
}


uint8_t SCCB::read(){
	setDataDirection(SIO_IN);
	delay_microseconds(SomeArbitraryDelayValue); // geht ohne nicht?!
	// read 8 bits by hand
	uint8_t ret = 0;
	for(int i=0;i<8;i++){
		delay_microseconds(SomeArbitraryDelayValue);
		SIO_C_High;
		delay_microseconds(SomeArbitraryDelayValue); // hold high time mindestens 600ns!
		// read value at SIO_D Pin and then shift by one
		ret = ret << 1;
		if(SIO_D_CHECK == 1) ret += 1;
		int k = GPIO_ReadInputDataBit(GPIOB,SIO_D);
		uint16_t r[0];
		r[0] = GPIO_ReadInputData(GPIOB);
		PRINTF("ret is %d and k is %d\n",ret,k);
		for(int i=0;i<sizeof(r);i++){
			PRINTF("r %d: %d\n",i,r[i]);
		}



		SIO_C_Low;
		delay_microseconds(SomeArbitraryDelayValue); // min 2* setup time und 1* data setup time -> mindestens 700ns
	}
	// now send NACK -> drive high
	setDataDirection(SIO_OUT);
	SIO_D_High;
	delay_microseconds(SomeArbitraryDelayValue);
	SIO_C_High;
	delay_microseconds(SomeArbitraryDelayValue);
	SIO_C_Low;
	delay_microseconds(SomeArbitraryDelayValue);
	SIO_D_Low;
	delay_microseconds(SomeArbitraryDelayValue);
	return ret;
}

int SCCB::write(uint8_t data){
	setDataDirection(SIO_OUT);
	PRINTF("sending data: ");
	for(int i=0;i<8;i++){
		if((data & 0x80) == 0x80) {
			SIO_D_High
		} else SIO_D_Low;
		PRINTF(" %d ",data);
		data = data << 1; // shifts the next bit to position 0x80
		delay_microseconds(SomeArbitraryDelayValue); // data out hold time min 50ns
		SIO_C_High;
		delay_microseconds(SomeArbitraryDelayValue);
		SIO_C_Low;
		delay_microseconds(SomeArbitraryDelayValue);// clock low to valid data out between 100 and 900 ns!
	}
	// should the master drive now SIO_D low or just read it because slave drives it low??? fucking bitch ass documentation
	delay_microseconds(SomeArbitraryDelayValue);
	setDataDirection(SIO_IN);
	delay_microseconds(SomeArbitraryDelayValue);
	SIO_C_High;
	delay_microseconds(SomeArbitraryDelayValue);
	int ret = 0;
	uint16_t t = SIO_D_CHECK;
	uint16_t u = GPIO_ReadInputDataBit(GPIOB,SIO_D);
	PRINTF("Check: %d %u\n",t,u);
	if(SIO_D_CHECK) ret = -1;
	else ret = 0;
	SIO_C_Low;
	delay_microseconds(SomeArbitraryDelayValue);
	setDataDirection(SIO_OUT);
	return ret;
}




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
