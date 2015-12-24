/*
 * camera.cpp
 *
 *  Created on: 08.12.2015
 *      Author: akynos
 */


#include "Camera.h"


#define ONE_BYTE_REG_ADDR 0x01
#define TWO_BYTE_REG_ADDR 0x02

//Camera camera("camera_thread");



Camera::Camera(){
	reset = HAL_GPIO(GPIO_010); //PA10
	power = HAL_GPIO(GPIO_033); //PC01
	isActive = false;
	captureImage = false;
	//	processData = false;
	//	sendPic = true;
}

void Camera::initTimer(){
	//	GPIO_InitTypeDef GPIO_InitStructure;
	//
	//	RCC_ClockSecuritySystemCmd(ENABLE);
	//
	//	/* Enable GPIOs clocks */
	//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//
	//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
	//
	//	/* Configure MCO (PA8) */
	//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//
	//	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);
}


void Camera::OV7670_SCCB() {
	//	PRINTF("starting InitOV7670 init\n");
	//	uint16_t x = 0;
	//	int res = 0;
	//	res = sccb.ov7670_set(0x12, 0x80);
	//	res = sccb.ov7670_set(0x12, 0x00);
	////	PRINTF("res is %d\n",res);
	//	while (init_registers[x][0] != 0xFF && init_registers[x][1] != 0xFF) {
	//		PRINTF("init register: status x=%d\n", x);
	//
	//		res = sccb.ov7670_set((unsigned char) init_registers[x][0],
	//				(unsigned char) init_registers[x][1]);
	//		uint8_t read = sccb.ov7670_get((unsigned char) init_registers[x][0]);
	//		PRINTF("SCCB Init %d: reg 0x%x = 0x%x = 0x%x \n", x,
	//				init_registers[x][0], init_registers[x][1], read);
	//		if (res) {
	//			PRINTF("ERROR I2C %d\n", res);
	//		}
	//		x++;
	//		delayx(5);
	//
	//	}

}

void Camera::init() {

}


void Camera::setNewData(CAM_CONTROL _data){
	this->daten = _data;
	/** TODO activate Camera! */
	this->isActive = this->daten.activateCamera;
	this->captureImage = this->daten.capture;
}

void Camera::run(){


	while(!isActive){
		suspendCallerUntil(END_OF_TIME);
	}
	reset.setPins(1);
	power.setPins(0);
	BLUE_ON;
	PRINTF("starting cam init\n");
//	cam.GPIO_Configuration();
	FIFO_CS_L();

	FIFO_CS_H();
	PRINTF("now sensor init\n");
	while(1 != cam.Sensor_Init());
	PRINTF("sensor init complete!\n");
	PRINTF("configuring interrupts\n");

	cam.OV7670_PB7_Configuration();
	PRINTF("Done\n");
	VSync = 0;

	while(1){
//		suspendCallerUntil(END_OF_TIME);
//		suspendCallerUntil(NOW() + 1500*MILLISECONDS);
		BLUE_TOGGLE;
		if(captureImage){
			PRINTF("capturing imageo!\n");
			uint32_t count;
			uint16_t CMOS_Data;
			PRINTF("VSyncs is %d\n",VSync);
			PRINTF("test is %d\n",test);
			if( VSync == 2 )
			{
				PRINTF("Vsync is 2!\n");
				FIFO_RRST_L();
				FIFO_RCLK_L();

				FIFO_RCLK_H();
				FIFO_RRST_H();
				FIFO_RCLK_L();

				FIFO_RCLK_H();
				PRINTF("start CMOS Data:\n");
				for( count = 0; count < 76800; count++ )
				{
					FIFO_RCLK_L();

					CMOS_Data = (GPIOC->IDR<<8) & 0xff00;	  /* ( GPIO_ReadInputData(GPIOC) << 8 ) & 0xff00; */
					FIFO_RCLK_H();

					FIFO_RCLK_L();
					CMOS_Data |= (GPIOC->IDR) & 0x00ff;	  /* ( GPIO_ReadInputData(GPIOC) ) & 0x00ff; */
					FIFO_RCLK_H();
					PRINTF("%d;",CMOS_Data);

				}
				suspendCallerUntil(END_OF_TIME);
				VSync = 0;
			}


//			captureImage = false;
		}
	}
}




