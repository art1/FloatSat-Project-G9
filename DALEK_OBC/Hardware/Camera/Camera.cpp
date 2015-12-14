/*
 * camera.cpp
 *
 *  Created on: 08.12.2015
 *      Author: akynos
 */


#include "Camera.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "Supps/initRegister.h"
#include "stdio.h"

#define ONE_BYTE_REG_ADDR 0x01
#define TWO_BYTE_REG_ADDR 0x02

//Camera camera("camera_thread");



Camera::Camera(){
	ledo = HAL_GPIO(GPIO_061); //PD13
	reset = HAL_GPIO(GPIO_010); //PA10
	power = HAL_GPIO(GPIO_033); //PC01
	active = false;
	processData = false;
	sendPic = true;
}

void Camera::initTimer(){
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_ClockSecuritySystemCmd(ENABLE);

	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	/* Configure MCO (PA8) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);
}


void Camera::OV7670_SCCB() {
	PRINTF("starting InitOV7670 init\n");
	uint16_t x = 0;
	int res = 0;
	res = sccb.ov7670_set(0x12, 0x80);
	res = sccb.ov7670_set(0x12, 0x00);

	while (init_registers[x][0] != 0xFF && init_registers[x][1] != 0xFF) {
		PRINTF("init register: status x=%d\n", x);

		res = sccb.ov7670_set((unsigned char) init_registers[x][0],
				(unsigned char) init_registers[x][1]);
		uint8_t read = sccb.ov7670_get((unsigned char) init_registers[x][0]);
		PRINTF("SCCB Init %d: reg 0x%x = 0x%x = 0x%x \n", x,
				init_registers[x][0], init_registers[x][1], read);
		if (res) {
			PRINTF("ERROR I2C %d\n", res);
		}
		x++;
		delayx(5);

	}

}

void Camera::init() {
	PRINTF("starting cam init\n");
	//initTimer();
	ledo.init(true);
	reset.init(true);
	power.init(true);
	reset.setPins(1);
	power.setPins(0);
	PRINTF("Init GPIOs...");
	dcmi.InitGPIO();
	PRINTF("Done!\n");
	PRINTF("Init DCMI...");
	dcmi.InitDCMI();
	PRINTF("Done!\n");
	PRINTF("Init I2C...");
	delayx(1000);
	sccb.I2CInit();
	PRINTF("Done!\n");
	PRINTF("Init OV7670...");
	delayx(1000);
	OV7670_SCCB();
	PRINTF("Done!\n");
	PRINTF("Enable DCMI...");
	delayx(1000);
	dcmi.EnableDCMI();

	PRINTF("Done with cam init!\n");
}

void Camera::ProcessData() {
	processData = true;

}


void Camera::run(){


	while(1){
		uint8_t ret = sccb.ov7670_get(0x0F);
		PRINTF("0x0F is %d\n",ret);
		suspendCallerUntil(NOW()+500*MILLISECONDS);
	}

//	while(1){
//		if (processData) {
//
//			processData = false;
//			if (sendPic) { // If picture was requested, send
//				char tmpVal[4];
//				PRINTF("CAMERA\n");
//				for (int i = 0; i < IMAGESIZE; i ++) {
//
//					sprintf(tmpVal, "%03u", DCMI_Buffer[i]);
//					TeleUART.write(tmpVal, 4);
//					while (!TeleUART.isWriteFinished()) {
//					}
//					sprintf(tmpVal, "\r\n");
//					TeleUART.write(tmpVal, 4);
//					while (!TeleUART.isWriteFinished()) {
//					}
//
//				}
//				PRINTF("CAMEND\n");
//				sendPic = false;
//			}
//
//			if (active) { // Continue captureing/processing if cam is still active
//				Capture();
//			}
//
//			suspendCallerUntil(NOW()+200*MILLISECONDS); // Could run even faster but 200ms is suficient for mission mode
//		}
//	}
}


void Camera::sendPicture() {
	sendPic = true;
	//tm.turnOn();
}

uint8_t* Camera::getPicture() {
	return DCMI_Buffer;
}

void Camera::Capture() {
	DCMI_CaptureCmd(ENABLE);
}


void Camera::delayx(unsigned int ms) {
	//4694 = 1 ms
	while (ms > 1) {
		ms--;
		asm("nop");
	}
}
