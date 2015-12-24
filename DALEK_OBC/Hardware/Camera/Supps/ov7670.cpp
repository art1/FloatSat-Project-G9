/*
 * ov7670.cpp
 *
 *  Created on: Dec 22, 2015
 *      Author: arthur
 */

#include "ov7670.h"
#include "inttypes.h"




static const uint8_t OV7670_Reg[OV7670_REG_NUM][2]=
{
		/*“‘œ¬Œ™OV7670 QVGA RGB565≤Œ ˝  */
		{0x3a, 0x04},//
		{0x40, 0x30}, // COM15 -> 00110000 -> RGB555
		{0x12, 0x14}, // COM7 -> 00010100 -> RGB Format QVGA
		{0x32, 0x80},
		{0x17, 0x16},

		{0x18, 0x04},
		{0x19, 0x02},
		{0x1a, 0x7b},
		{0x03, 0x06},
		{0x0c, 0x0c},
		{0x15, 0x02},
		{0x3e, 0x00},
		{0x70, 0x00},
		{0x71, 0x01},
		{0x72, 0x11},
		{0x73, 0x09},

		{0xa2, 0x02},
		{0x11, 0x00},
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
		{0x00, 0x00}, /* AGC */

		{0x10, 0x00},
		{0x0d, 0x00},
		{0x14, 0x20},
		{0xa5, 0x05},
		{0xab, 0x07},

		{0x24, 0x75},
		{0x25, 0x63},
		{0x26, 0xA5},
		{0x9f, 0x78},
		{0xa0, 0x68},

		{0xa1, 0x03},
		{0xa6, 0xdf},
		{0xa7, 0xdf},
		{0xa8, 0xf0},
		{0xa9, 0x90},

		{0xaa, 0x94},
		{0x13, 0xe5},
		{0x0e, 0x61},
		{0x0f, 0x4b},
		{0x16, 0x02},

		{0x1e, 0x37},
		{0x21, 0x02},
		{0x22, 0x91},
		{0x29, 0x07},
		{0x33, 0x0b},

		{0x35, 0x0b},
		{0x37, 0x1d},
		{0x38, 0x71},
		{0x39, 0x2a},
		{0x3c, 0x78},

		{0x4d, 0x40},
		{0x4e, 0x20},
		{0x69, 0x5d},
		{0x6b, 0x40},
		{0x74, 0x19},
		{0x8d, 0x4f},

		{0x8e, 0x00},
		{0x8f, 0x00},
		{0x90, 0x00},
		{0x91, 0x00},
		{0x92, 0x00},

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

		{0x4f, 0x80},
		{0x50, 0x80},
		{0x51, 0x00},
		{0x52, 0x22},
		{0x53, 0x5e},
		{0x54, 0x80},

		{0x6e, 0x11},
		{0x6f, 0x9f},
		{0x55, 0x00}, /* ¡¡∂» */
		{0x56, 0x40}, /* ∂‘±»∂» */
		{0x57, 0x80}, /* change according to Jim's request */
};




ov7670::ov7670() {
	// TODO Auto-generated constructor stub

}

ov7670::~ov7670() {
	// TODO Auto-generated destructor stub
}


int ov7670::Sensor_Init(void){
	uint16_t i=0;
	uint8_t Sensor_IDCode = 0;


	//	GPIO_Configuration();
	FIFO_GPIO_Configuration();





	sccb2.I2CInit();
	//	for(uint8_t i=0;i<30;i++){
	uint8_t t = sccb2.ov7670_get(0x0b);
	PRINTF("read ID-Code at 0x0b: %d - should be 115\n",t);
	//	}

	sccb2.ov7670_set(OV7670_Reg[i][0], OV7670_Reg[i][1]);

	//	//	I2C_Configuration();
	//	sccb.init();
	//	PRINTF("hello 0\n");
	//	if( 0 == sccb.writeByte ( 0x12, 0x80 , CAM_WRITE ) ) /* Reset SCCB */
	//	{
	//		return 0 ;
	//	}
	//	PRINTF("hello 1\n");
	//	Delay_millis(50);
	//
	//	if( 0 == sccb.readByte( &Sensor_IDCode, 1, 0x0b,  CAM_READ ) )	 /* read ID */
	//	{
	//		PRINTF("error reading bytes for cam!\n");
	//		return 0;	                              /* error*/
	//	}
	//	PRINTF("hello 2\n");
	//	if(Sensor_IDCode == OV7670)				  /* ID = OV7670 */
	//	{
	//		for( i=0 ; i < OV7670_REG_NUM ; i++ )
	//		{
	//			if( 0 == sccb.writeByte(  OV7670_Reg[i][0], OV7670_Reg[i][1] , ADDR_OV7670 ) )
	//			{
	//				return 0;
	//				PRINTF("something went wrong during setting the registers!\n");
	//			}
	//		}
	//	}
	//	else										  /* NO ID */
	//	{
	//		PRINTF("sensor ID code does not match, is %d!\n",Sensor_IDCode);
	//		uint8_t tst = 0x33;
	//		for(uint8_t i=0;i<20;i++){
	//			sccb.readByte(&tst,1,i,0x42);
	//			PRINTF("read %d: %" PRIu8 "\n",i,tst);
	//		}
	//
	//		return 0;
	//	}

	return 1;
}

void ov7670::GPIO_Configuration(){
	//	GPIO_InitTypeDef GPIO_InitStructure;
	//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_PORT_VSYNC_CMOS, ENABLE);
	//	GPIO_InitStructure.GPIO_Pin =  PIN_VSYNC_CMOS;
	//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//	//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//	GPIO_Init(PORT_VSYNC_CMOS, &GPIO_InitStructure);
}


void ov7670::FIFO_GPIO_Configuration(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOC, ENABLE);
	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	/* FIFO_RCLK : PA8  */
	GPIO_InitStructure.GPIO_Pin = FIFO_RCLK_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Other Fifo PINs  */
	GPIO_InitStructure.GPIO_Pin = FIFO_RRST_PIN | FIFO_CS_PIN | FIFO_WE_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	/* FIFO D6 D7 D4 D2 D3 ->  PE5 PE6 PE4 PE0 PE1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* D0 and D1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* FIFO_HREF : PA4 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void ov7670::OV7670_PB7_Configuration(void){

//	HAL_GPIO testPB7(GPIO_023);
//	testPB7.init(false);
//	testPB7.interruptEnable(true);
//	--> gives you XMalloc after System initialization - wtf?
//  --> try to comment out all the 9-5 IRQ interrupt stuff from rodos!
//  --> yes, this helps a lot^


	/* Set variables used */
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Enable clock for GPIOB */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = PIN_VSYNC_CMOS;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Tell system that you will use PB7 for EXTI_Line7 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);

	/* PB12 is connected to EXTI_Line12 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line7;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

	/* Add IRQ vector to NVIC */
	/* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);

}

int  ov7670::OV7670_ReadReg(uint8_t reg,uint16_t regValue){
	PRINTF("ov7670 read reg TODO\n");
	return -1;
}
int  ov7670::OV7670_WriteReg(uint8_t reg,uint16_t regValue){
	PRINTF("ov7670 write reg TODO\n");
	return -1;
}
