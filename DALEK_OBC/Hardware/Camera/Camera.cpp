/*
 * camera.cpp
 *
 *  Created on: Nov 28, 2015
 *      Author: arthur
 */

#include "Camera.h"


//SCCB sccb;


HAL_GPIO camPWR(CAM_POWER);
HAL_GPIO camReset(CAM_RESET);

Camera::Camera() {
//	// TODO Auto-generated constructor stub
//	camPWR(CAM_POWER);
//	camReset(CAM_RESET);

}

Camera::~Camera() {
	// TODO Auto-generated destructor stub
}

void Camera::init(){

	// init clock -> PA8
//	 GPIO_InitTypeDef GPIO_InitStructure;
//
//	 RCC_ClockSecuritySystemCmd(ENABLE);
//
//	 /* Enable GPIOs clocks */
//	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//
//	 GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
//
//	 /* Configure PA8 */
//	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	 GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	 RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);

}




void Camera::run(){
//	i2c1.init(400000);
	camPWR.init(true,1,0);
	camReset.init(true,1,0);
	camReset.setPins(1);
	camPWR.setPins(0); // low is power on!
//	i2c1.reset();
	while(1){
		suspendCallerUntil(NOW()+500*MILLISECONDS);
		test();
		BLUE_TOGGLE;
	}
}

void Camera::test(){
	camReset.setPins(0);
	camPWR.setPins(0); // low is power on!
	PRINTF("init SCCB\n");
//	sccb.init();
	suspendCallerUntil(NOW()+500*MILLISECONDS);
	PRINTF("wirting to cam\n");

//
//
//
//	 recBuf[0] = sccb.readReg(0x01);
//
//////
//	transBuf[0] = 0x01;
//	i2c1.writeRead(CAM_READ,transBuf,1,recBuf,1);
////	int k = i2c1.writeRead(CAM_READ,transBuf,1,recBuf,1);
	PRINTF("read 0x01 : %d\n",recBuf[0]);
////
//	transBuf[0] = 0x02;
////	k = i2c1.writeRead(CAM_READ,transBuf,1,recBuf,1);
//	PRINTF("read 0x02 : %d\n",recBuf[0]);

}
