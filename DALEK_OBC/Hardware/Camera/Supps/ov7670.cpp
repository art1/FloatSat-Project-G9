/*
 * ov7670.cpp
 *
 *  Created on: Dec 22, 2015
 *      Author: arthur
 */

#include "ov7670.h"
#include "inttypes.h"

#define YUV422


static const uint8_t init_registers[][2]=
{		{0x12, 0x80}, // COM7 default: 0x00		reset
		{0x11, 0x00}, // CLKRC default: 0x80	30 fps
		{0x3A, 0x04}, // TSLB default: 0x0D
		{0x12, 0x00}, // COM7 default: 0x00		VGA

		/* Set the hardware window */
		{0x17, 0x39}, // HSTART default: 0x11
		{0x18, 0x03}, // HSTOP default: 0x61
		{0x32, 0x92}, // HREF default: 0x80
		{0x19, 0x03}, // VSTRT default: 0x03
		{0x1A, 0x7B}, // VSTOP default: 0x7B
		{0x03, 0x0A}, // VREF default: 0x00

		{0x0C, 0x0C}, // COM3 default: 0x00
		{0x3E, 0x11}, //0b00011001}, // COM14 default: 0x00

		/* Mystery scaling numbers */
		{0x70, 0x3A}, // SCALING_XSC default: 0x3A
		{0x71, 0x35}, // SCALING_YSC default: 0x35
		{0x72, 0x11}, // SCALING_DCWCTR default: 0x11
		{0x73, 0xF1}, // SCALING_PCLK_DIV default: 0x00 F1
		{0xA2, 0x52}, // SCALING_PCLK_DELAY default: 0x02
		{0x15, 0x00}, // COM10 default: 0x00

		/* Gamma curve values */
		{0x7A, 0x20}, // SLOP default: 0x24
		{0x7B, 0x1C}, // GAM1 default: 0x04
		{0x7C, 0x28}, // GAM2 default: 0x07
		{0x7D, 0x3C}, // GAM3 default: 0x10
		{0x7E, 0x5A}, // GAM4 default: 0x28
		{0x7F, 0x68}, // GAM5 default: 0x36
		{0x80, 0x76}, // GAM6 default: 0x44
		{0x81, 0x80}, // GAM7 default: 0x52
		{0x82, 0x88}, // GAM8 default: 0x60
		{0x83, 0x8F}, // GAM9 default: 0x6C
		{0x84, 0x96}, // GAM10 default: 0x78
		{0x85, 0xA3}, // GAM11 default: 0x8C
		{0x86, 0xAF}, // GAM12 default: 0x9E
		{0x87, 0xC4}, // GAM13 default: 0xBB
		{0x88, 0xD7}, // GAM14 default: 0xD2
		{0x89, 0xE8}, // GAM15 default: 0xE5

		/* AGC and AEC parameters. Disable -> Tweak values -> Enable */
		{0x13, 0x80 | 0x40 | 0x20}, // COM8 default: 0x8F
		{0x00, 0x00}, // GAIN default: 0x00
		{0x10, 0x00}, // AECH default: 0x40
		{0x0D, 0x40}, // COM4 default: 0x00
		{0x14, 0x28}, // COM9 default: 0x4A		4x gain
		{0xA5, 0x05}, // BD50MAX default: 0x0F
		{0xAB, 0x07}, // BD60MAX default: 0x0F
		{0x24, 0x95}, // AEW default: 0x75
		{0x25, 0x33}, // AEB default: 0x63
		{0x26, 0xE3}, // VPT default: 0xD4
		{0x9F, 0x78}, // HAECC1 default: 0xC0
		{0xA0, 0x68}, // HAECC2 default: 0x90
		{0xA1, 0x0B}, // RSVD					magic
		{0xA6, 0xD8}, // HAECC3 default: 0xF0
		{0xA7, 0xD8}, // HAECC4 default: 0xC1
		{0xA8, 0xF0}, // HAECC5 default: 0xF0
		{0xA9, 0x90}, // HAECC6 default: 0xC1
		{0xAA, 0x94}, // HAECC7 default: 0x14
		{0x13, 0x80 | 0x40 | 0x20 | 0x04 | 0x01}, // COM8 default: 0x8F

		/* Magic reserved values */
		{0x0E, 0x61}, // COM5 default: 0x01
		{0x0F, 0x4B}, // COM6 default: 0x43
		{0x16, 0x02}, // RSVD
		{0x21, 0x02}, // ADCCTR1 default: 0x02
		{0x22, 0x91}, // ADCCTR2 default: 0x01
		{0x29, 0x07}, // RSVD
		{0x33, 0x03}, // CHLF default: 0x08
		{0x35, 0x0B}, // RSVD
		{0x37, 0x1C}, // ADC default: 0x3F
		{0x38, 0x71}, // ACOM default: 0x01
		{0x3C, 0x78}, // COM12 default: 0x68
		{0x4D, 0x40}, // RSVD
		{0x4E, 0x20}, // RSVD
		{0x69, 0x55}, // GFIX default: 0x00
		{0x6B, 0x4A}, // DBLV default: 0x0A
		{0x74, 0x19}, // REG74 default: 0x00
		{0x8D, 0x4F}, // RSVD
		{0x8E, 0x00}, // RSVD
		{0x8F, 0x00}, // RSVD
		{0x90, 0x00}, // RSVD
		{0x91, 0x00}, // RSVD
		{0x96, 0x00}, // RSVD
		{0x9A, 0x80}, // RSVD
		{0xB0, 0x8C}, // RSVD
		{0xB1, 0x0C}, // ABLC1 default: 0x00
		{0xB2, 0x0E}, // RSVD
		{0xB3, 0x82}, // THL_ST default: 0x80
		{0xB8, 0x0A}, // RSVD

		/* Reserved magic (white balance) */
		{0x43, 0x14}, // AWBC1 default: 0x14
		{0x44, 0xF0}, // AWBC2 default: 0xF0
		{0x45, 0x34}, // AWBC3 default: 0x45
		{0x46, 0x58}, // AWBC4 default: 0x61
		{0x47, 0x28}, // AWBC5 default: 0x51
		{0x48, 0x3A}, // AWBC6 default: 0x79
		{0x59, 0x88}, // RSVD
		{0x5A, 0x88}, // RSVD
		{0x5B, 0x44}, // RSVD
		{0x5C, 0x67}, // RSVD
		{0x5D, 0x49}, // RSVD
		{0x5E, 0x0E}, // RSVD
		{0x6C, 0x0A}, // AWBCTR3 default: 0x02
		{0x6D, 0x55}, // AWBCTR2 default: 0x55
		{0x6E, 0x11}, // AWBCTR1 default: 0xC0
		{0x6F, 0x9F}, // AWBCTR0 default: 0x9A (0x9E for advance AWB)
		{0x6A, 0x40}, // GGAIN default: 0x00
		{0x01, 0x40}, // BLUE default: 0x80
		{0x02, 0x40}, // RED default: 0x80
		{0x13, 0x80 | 0x40 | 0x20 | 0x01}, // COM8 default: 0x8F

		/* Matrix coefficients */
		{0x4F, 0x80}, // MTX1 default: 0x40
		{0x50, 0x80}, // MTX2 default: 0x34
		{0x51, 0x00}, // MTX3 default: 0x0C
		{0x52, 0x22}, // MTX4 default: 0x17
		{0x53, 0x5E}, // MTX5 default: 0x29
		{0x54, 0x80}, // MTX6 default: 0x40
		{0x58, 0x9E}, // MTXS default: 0x1E

		{0x41, 0x08}, // COM16 default: 0x08
		{0x3F, 0x00}, // EDGE default: 0x00
		{0x75, 0x05}, // REG75 default: 0x0F
		{0x76, 0x61}, // REG76 default: 0x01
		{0x4C, 0x00}, // DNSTH default: 0x00
		{0x77, 0x01}, // REG77 default: 0x10
		{0x3D, 0xC2}, // COM13 default: 0x88
		{0x4B, 0x09}, // REG4B default: 0x00
		{0xC9, 0x60}, // SATCTR default: 0xC0
		{0x41, 0x38}, // COM16 default: 0x08
		{0x56, 0x40}, // CONTRAS default: 0x40

		{0x34, 0x11}, // ARBLM default: 0x11
		{0x3B, 0x02}, // COM11 default: 0x00
		{0xA4, 0x89}, // NT_CTRL default: 0x00
		{0x96, 0x00}, // RSVD
		{0x97, 0x30}, // RSVD
		{0x98, 0x20}, // RSVD
		{0x99, 0x20}, // RSVD
		{0x9A, 0x84}, // RSVD
		{0x9B, 0x29}, // RSVD
		{0x9C, 0x03}, // RSVD
		{0x9D, 0x4C}, // BD50ST default: 0x99
		{0x9E, 0x3F}, // BD60ST default: 0x7F
		{0x78, 0x04}, // RSVD

		/* Extra-weird stuff (sort of multiplexor register) */
		{0x79, 0x01}, // RSVD
		{0xC8, 0xF0}, // RSVD
		{0x79, 0x0F}, // RSVD
		{0xC8, 0x20}, // RSVD
		{0x79, 0x10}, // RSVD
		{0xC8, 0x7E}, // RSVD
		{0x79, 0x0B}, // RSVD
		{0xC8, 0x01}, // RSVD
		{0x79, 0x0C}, // RSVD
		{0xC8, 0x07}, // RSVD
		{0x79, 0x0D}, // RSVD
		{0xC8, 0x20}, // RSVD
		{0x79, 0x09}, // RSVD
		{0xC8, 0x80}, // RSVD
		{0x79, 0x02}, // RSVD
		{0xC8, 0xC0}, // RSVD
		{0x79, 0x03}, // RSVD
		{0xC8, 0x40}, // RSVD
		{0x79, 0x05}, // RSVD
		{0xC8, 0x30}, // RSVD
		{0x79, 0x26}, // RSVD

		{0x19, 0x02}, // VSTRT default: 0x03
		{0x03, 0x0C}, // VREF default: 0x00
		{0x1A, 0x7C}, // VSTOP default: 0x7B

#ifdef YUV422
		{0x12, 0x00}, // COM7			Selects YUV mode
		{0x8C, 0x00}, // RGB444			No RGB444 please
		{0x04, 0x00}, // COM1			CCIR601
		{0x40, 0xC0}, // COM15			[00] to [FF]
		{0x14, 0x48}, // COM9			32x gain ceiling
		{0x4F, 0x80}, // MTX1
		{0x50, 0x80}, // MTX2
		{0x51, 0x00}, // MTX3
		{0x52, 0x22}, // MTX4
		{0x53, 0x5E}, // MTX5
		{0x54, 0x80}, // MTX6
		{0x3D, 0x80 | 0x40}, // COM13
#endif

		{0xFF, 0xFF}, // END MARKER

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
