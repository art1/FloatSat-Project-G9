/*
 * mySCCB.cpp
 *
 *  Created on: Dec 22, 2015
 *      Author: arthur
 */

#include "mySCCB.h"

mySCCB::mySCCB() {


}

mySCCB::~mySCCB() {

}

void mySCCB::init(){
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Configure I2C2 pins: PB10->SCL and PB11->SDA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  SCL_DIO | SCL_CLK;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/**
 * sets the SIO Data Pin direction
 * params:	1	Output
 * 			0	Input
 * return:  0	SUCCESS
 * 			-1	ERROR
 */
int mySCCB::setDataDirection(int dir){
	if((dir < 0) || (dir > 1) ) return -1;
	GPIO_InitTypeDef  GPIO_Struct;

	if(dir == 1){ // then output
		GPIO_Struct.GPIO_Pin = SCL_DIO;
		GPIO_Struct.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_Struct.GPIO_OType = GPIO_OType_OD; // opendrain
		GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	} else {
		GPIO_Struct.GPIO_Pin = SCL_DIO;
		GPIO_Struct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Struct.GPIO_Speed = GPIO_Speed_2MHz;
//		GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}
	GPIO_Init(GPIOB,&GPIO_Struct);
	return 0;
}


int mySCCB::writeByte(uint16_t WriteAddress , uint8_t SendByte , uint8_t DeviceAddress){
	if(!start())
	{
		PRINTF("start did not went well!\n");
		return DISABLE;
	}
	sendByte( DeviceAddress );                    /* ∆˜º˛µÿ÷∑ */
	if( !waitACK() )
	{
		PRINTF("wait for ack wasnt succesfull\n");
		stop();
		return DISABLE;
	}
	sendByte((uint8_t)(WriteAddress & 0x00FF));   /* …Ë÷√µÕ∆ ºµÿ÷∑ */
	waitACK();
	sendByte(SendByte);
	waitACK();
	stop();
	/* ◊¢“‚£∫“ÚŒ™’‚¿Ô“™µ»¥˝EEPROM–¥ÕÍ£¨ø…“‘≤…”√≤È—ØªÚ—” ±∑Ω Ω(10ms)	*/
	/* Systick_Delay_1ms(10); */
	return ENABLE;
}


int mySCCB::readByte(uint8_t* pBuffer,   uint16_t length,   uint8_t ReadAddress,  uint8_t DeviceAddress){
	if(!start())
	{
		return DISABLE;
	}
	sendByte( DeviceAddress );         /* ∆˜º˛µÿ÷∑ */
	if( !waitACK() )
	{
		stop();
		return DISABLE;
	}
	sendByte( ReadAddress );           /* …Ë÷√µÕ∆ ºµÿ÷∑ */
	waitACK();
	stop();

	if(!start())
	{
		return DISABLE;
	}
	sendByte( DeviceAddress + 1 );               /* ∆˜º˛µÿ÷∑ */
	if(!waitACK())
	{
		stop();
		return DISABLE;
	}
	while(length)
	{
		*pBuffer = receiveByte();
		if(length == 1)
		{
			nack();
		}
		else
		{
			ack();
		}
		pBuffer++;
		length--;
	}
	stop();
	return ENABLE;
}
/************************** PRIVATE FUNCTINS ****************************/
void mySCCB::myDelay(){
	uint8_t i = 100; /* ’‚¿Ôø…“‘”≈ªØÀŸ∂» */
	while(i)
	{
		i--;
	}
}
int mySCCB::start(){
	SDA_H;
	SCL_H;
	myDelay();
//	PRINTF("reading SDA now 1\n");
	if(!SDA_read)return DISABLE;	/* SDAœﬂŒ™µÕµÁ∆Ω‘Ú◊‹œﬂ√¶,ÕÀ≥ˆ */
	SDA_L;
	myDelay();
	//	Delay_millis(10);
//	PRINTF("reading SDA now 2\n");
	if(SDA_read) return DISABLE;	/* SDAœﬂŒ™∏ﬂµÁ∆Ω‘Ú◊‹œﬂ≥ˆ¥Ì,ÕÀ≥ˆ */
	SDA_L;
	myDelay();
//	PRINTF("returning now enabled\n");
	return ENABLE;
}
void mySCCB::stop(){
	SCL_L;
	myDelay();
	SDA_L;
	myDelay();
	SCL_H;
	myDelay();
	SDA_H;
	myDelay();
}
void mySCCB::ack(){
	SCL_L;
	myDelay();
	SDA_L;
	myDelay();
	SCL_H;
	myDelay();
	SCL_L;
	myDelay();
}
void mySCCB::nack(){
	SCL_L;
	myDelay();
	SDA_H;
	myDelay();
	SCL_H;
	myDelay();
	SCL_L;
	myDelay();
}
int mySCCB::waitACK(){
	SCL_L;
	myDelay();
	SDA_H;
	myDelay();
	SCL_H;
	myDelay();
	if(SDA_read)
	{
		SCL_L;
		return DISABLE;
	}
	SCL_L;
	return ENABLE;
}
void mySCCB::sendByte(uint8_t sByte){
	uint8_t i=8;
	while(i--)
	{
		SCL_L;
		myDelay();
		if(sByte&0x80)
			SDA_H;
		else
			SDA_L;
		sByte<<=1;
		myDelay();
		SCL_H;
		myDelay();
	}
	SCL_L;
}
uint8_t mySCCB::receiveByte(){
	uint8_t i=8;
	uint8_t ReceiveByte=0;

	SDA_H;
	while(i--)
	{
		ReceiveByte<<=1;
		SCL_L;
		myDelay();
		SCL_H;
		myDelay();
		if(SDA_read)
		{
			ReceiveByte|=0x01;
		}
	}
	SCL_L;
	return ReceiveByte;
}

