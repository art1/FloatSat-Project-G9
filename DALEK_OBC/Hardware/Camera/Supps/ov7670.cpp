/*
 * ov7670.cpp
 *
 *  Created on: Dec 22, 2015
 *      Author: arthur
 */

#include "ov7670.h"
#include "inttypes.h"




static const uint8_t init_registers[][2]=
{		{0x3a, 0x04},
		{0x40, 0xd0}, //
		{0x32, 0x80},
		{0x17, 0x16},
		{0x18, 0x04},
		{0x19, 0x02},
		{0x1a, 0x7b},//0x7a,
		{0x03, 0x06},//0x0a,
		{0x12, 0x10}, // YUV
		{ 0x0c, 0x04 }, // COM3
		{ 0x3e, 0x1A }, // COM14
		{ 0x70, 0x3a }, // SCALING_XSC
		{ 0x71, 0x35 }, // SCALING_YSC
		{ 0x72, 0xee }, // SCALING_DCWCTR
		{ 0x73, 0xf2 }, // SCALING_PCLK_DIV
		{ 0xa2, 0x02 }, // SCALING_PCLK_DELAY
		{0x11, 0x01},
		{0x7a, 0x20},
		{0x7b, 0x1c},
		{0x7c, 0x28},
		{0x7d, 0x3c},
		{0x7e, 0x55},
		{0x7f, 0x68},
		{0x80, 0x76},
		{0x81, 0x80},
		{0x82, 0x88},
		{0x83, 0x8f},
		{0x84, 0x96},
		{0x85, 0xa3},
		{0x86, 0xaf},
		{0x87, 0xc4},
		{0x88, 0xd7},
		{0x89, 0xe8},
		{0x13, 0xe0},
		{0x00, 0x00},//AGC
		{0x10, 0x00},
		{0x0d, 0x00},
		{0x14, 0x20},//0x38, limit the max gain
		{0xa5, 0x05},
		{0xab, 0x07},
		{0x24, 0x75},
		{0x25, 0x63},
		{0x26, 0xA5},
		{0x9f, 0x78},
		{0xa0, 0x68},
		{0xa1, 0x03},//0x0b,
		{0xa6, 0xdf},//0xd8,
		{0xa7, 0xdf},//0xd8,
		{0xa8, 0xf0},
		{0xa9, 0x90},
		{0xaa, 0x94},
		{0x13, 0xe5},
		{0x0e, 0x61},
		{0x0f, 0x4b},
		{0x16, 0x02},
		{0x1e, 0x37},//0x07,
		{0x21, 0x02},
		{0x22, 0x91},
		{0x29, 0x07},
		{0x33, 0x0b},
		{0x35, 0x0b},
		{0x37, 0x1d},
		{0x38, 0x71},
		{0x39, 0x2a},//
		{0x3c, 0x78},
		{0x4d, 0x40},
		{0x4e, 0x20},
		{0x69, 0x0c},///////////////////////
		{0x6b, 0x80},//PLL  30 FPS bei DIV 1
//		{0x6b, 0x60},//PLL	20 FPS bei DIV 2
		{0x74, 0x19},
		{0x8d, 0x4f},
		{0x8e, 0x00},
		{0x8f, 0x00},
		{0x90, 0x00},
		{0x91, 0x00},
		{0x92, 0x00},//0x19,//0x66
		{0x96, 0x00},
		{0x9a, 0x80},
		{0xb0, 0x84},
		{0xb1, 0x0c},
		{0xb2, 0x0e},
		{0xb3, 0x82},
		{0xb8, 0x0a},
		{0x43, 0x14},
		{0x44, 0xf0},
		{0x45, 0x34},
		{0x46, 0x58},
		{0x47, 0x28},
		{0x48, 0x3a},
		{0x59, 0x88},
		{0x5a, 0x88},
		{0x5b, 0x44},
		{0x5c, 0x67},
		{0x5d, 0x49},
		{0x5e, 0x0e},
		{0x64, 0x04},
		{0x65, 0x20},
		{0x66, 0x05},
		{0x94, 0x04},
		{0x95, 0x08},
		{0x6c, 0x0a},
		{0x6d, 0x55},
		{0x6e, 0x11},
		{0x6f, 0x9f},//0x9e for advance AWB
		{0x6a, 0x40},
		{0x01, 0x40},
		{0x02, 0x40},
		{0x13, 0xe7},
		{0x15, 0x08},
		{0x4f, 0x80},
		{0x50, 0x80},
		{0x51, 0x00},
		{0x52, 0x22},
		{0x53, 0x5e},
		{0x54, 0x80},
		{0x58, 0x9e},
		{0x41, 0x08},
		{0x3f, 0x00},
		{0x75, 0x05},
		{0x76, 0xe1},
		{0x4c, 0x00},
		{0x77, 0x01},
		{0x3d, 0xc2},	//0xc0,
		{0x4b, 0x09},
		{0xc9, 0xc0},
		{0x41, 0x38},
		{0x56, 0x40},//0x40,  change according to Jim's request
		{0x34, 0x11},
		{0x3b, 0x02},//0x00,//0x02,
		{0xa4, 0x89},//0x88,
		{0x96, 0x00},
		{0x97, 0x30},
		{0x98, 0x20},
		{0x99, 0x30},
		{0x9a, 0x84},
		{0x9b, 0x29},
		{0x9c, 0x03},
		{0x9d, 0x4c},
		{0x9e, 0x3f},
		{0x78, 0x04},

		{0x79, 0x01},
		{0xc8, 0xf0},
		{0x79, 0x0f},
		{0xc8, 0x00},
		{0x79, 0x10},
		{0xc8, 0x7e},
		{0x79, 0x0a},
		{0xc8, 0x80},
		{0x79, 0x0b},
		{0xc8, 0x01},
		{0x79, 0x0c},
		{0xc8, 0x0f},
		{0x79, 0x0d},
		{0xc8, 0x20},
		{0x79, 0x09},
		{0xc8, 0x80},
		{0x79, 0x02},
		{0xc8, 0xc0},
		{0x79, 0x03},
		{0xc8, 0x40},
		{0x79, 0x05},
		{0xc8, 0x30},
		{0x79, 0x26},
		{0x09, 0x03},
		{0x55, 0x00},
		{0x56, 0x40},
		{0x3b, 0x42},//0x82,//0xc0,//0xc2,	//night mode
		// #########################################################################
		// END MARKER
		{ 0xff, 0xff },

};




ov7670::ov7670() {


}

ov7670::~ov7670() {
}


int ov7670::Sensor_Init(void){
	uint16_t i=0;
	uint8_t Sensor_IDCode = 0;

	PRINTF("init i2c\n");
	sccb2.I2CInit();
	xprintf("Done!\n");
	xprintf("Init OV7670...");
	delayx(1000);

	xprintf("starting InitOV7670 init\n");
	uint16_t x = 0;
	int res = 0;
	res = sccb2.ov7670_set(0x12, 0x80);
	res = sccb2.ov7670_set(0x12, 0x00);

	while (init_registers[x][0] != 0xFF && init_registers[x][1] != 0xFF) {
		xprintf("init register: status x=%d\n", x);

		res = sccb2.ov7670_set((unsigned char) init_registers[x][0],
				(unsigned char) init_registers[x][1]);
		uint8_t read = sccb2.ov7670_get((unsigned char) init_registers[x][0]);
		xprintf("SCCB Init %d: reg 0x%x = 0x%x = 0x%x \n", x,
				init_registers[x][0], init_registers[x][1], read);
		if (res) {
			xprintf("ERROR I2C %d\n", res);
		}
		x++;

	}
	xprintf("done with InitOV7670 init\n");



//	//	GPIO_Configuration();
//	FIFO_GPIO_Configuration();
//
//
//
//	sccb2.I2CInit();
//	for(uint8_t i=0;i<OV7670_REG_NUM;i++){
//		sccb2.ov7670_set(OV7670_Reg[i][0], OV7670_Reg[i][1]);
//
//	}
//	sccb2.ov7670_set(0x70,0x7a);
//	sccb2.ov7670_set(0x71,0x75);
//
//
//
//	uint8_t t = sccb2.ov7670_get(REG_PID);
//	PRINTF("read ID-Code at 0x0a: %d - should be 118\n",t);
//
//
//	// resets all camera registers
//	sccb2.ov7670_set(0x12,0x80);
//	Delay_millis(200);
//	sccb2.ov7670_set(0x12,0x80);
//	Delay_millis(200);
//
//
//	PRINTF("reading all registers again now:\n");
//	uint8_t tmp;
//
//	for(uint8_t i=0;i< OV7670_REG_NUM;i++){
//		tmp = ReadReg(OV7670_Reg[i][0]);
//		PRINTF("Reg: %02x, Val: %02x\n",OV7670_Reg[i][0],tmp);
//	}

	return 1;
}



void ov7670::FIFO_GPIO_Configuration(void){

//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOC, ENABLE);
//	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//
//
//	/* FIFO_RCLK : PA8  */
//	GPIO_InitStructure.GPIO_Pin = FIFO_RCLK_PIN;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	/* Other Fifo PINs  */
//	GPIO_InitStructure.GPIO_Pin = FIFO_RRST_PIN | FIFO_CS_PIN | FIFO_WE_PIN;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//
//	/* FIFO D6 D7 D4 D2 D3 ->  PE5 PE6 PE4 PE0 PE1 */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOE, &GPIO_InitStructure);
//
//	/* D0 and D1 */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//	/* FIFO_HREF : PA4 */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void ov7670::OV7670_PB7_Configuration(void){

	//	HAL_GPIO testPB7(GPIO_023);
	//	testPB7.init(false);
	//	testPB7.interruptEnable(true);
	//	--> gives you XMalloc after System initialization - wtf?
	//  --> try to comment out all the 9-5 IRQ interrupt stuff from rodos!
	//  --> yes, this helps a lot^
//
//
//	/* Set variables used */
//	GPIO_InitTypeDef GPIO_InitStruct;
//	EXTI_InitTypeDef EXTI_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
//
//	/* Enable clock for GPIOB */
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	/* Enable clock for SYSCFG */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//
//	/* Set pin as input */
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_Pin = PIN_VSYNC_CMOS;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	/* Tell system that you will use PB7 for EXTI_Line7 */
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);
//
//	/* PB12 is connected to EXTI_Line12 */
//	EXTI_InitStruct.EXTI_Line = EXTI_Line7;
//	/* Enable interrupt */
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//	/* Interrupt mode */
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//	/* Triggers on rising and falling edge */
//	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//	/* Add to EXTI */
//	EXTI_Init(&EXTI_InitStruct);
//
//	/* Add IRQ vector to NVIC */
//	/* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
//	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
//	/* Set priority */
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
//	/* Set sub priority */
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
//	/* Enable interrupt */
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	/* Add to NVIC */
//	NVIC_Init(&NVIC_InitStruct);

}

uint8_t  ov7670::ReadReg(uint8_t _reg){
	return sccb2.ov7670_get(_reg);
}
int  ov7670::WriteReg(uint8_t _reg,uint8_t _regVal){
	sccb2.ov7670_set(_reg,_regVal);
	return 1;
}
