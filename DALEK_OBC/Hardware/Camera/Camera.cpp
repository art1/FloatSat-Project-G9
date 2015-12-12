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
	this->dat.shoot = false;
	memset(transBuf,0,sizeof(transBuf));
	memset(recBuf,0,sizeof(recBuf));

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


void Camera::setNewData(CAM_CONTROL data){
	PRINTF("setting cam data: %d\n",data.shoot);
	this->dat = data;
}


void Camera::run(){
//			i2c1.init(400000);
	camPWR.init(true);
	camReset.init(true);
	camReset.setPins(1);
	camPWR.setPins(0); // low is power on!
	PRINTF("init SCCB\n");
		sccb.init();
	while(1){
		suspendCallerUntil(NOW()+100*MILLISECONDS);
		if(dat.shoot)test();
		BLUE_TOGGLE;
	}
}

void Camera::test(){

//	suspendCallerUntil(NOW()+500*MILLISECONDS);
	PRINTF("wirting to cam\n");

	//
	//	//
		transBuf[0] = 0x0A;
		recBuf[0] = 0xFF;
		recBuf[0] = sccb.readReg(transBuf[0]);
		PRINTF("read 0x0A : %d\n",recBuf[0]);
//		recBuf[1] = sccb.readReg(0x0B);
//		PRINTF("read 0x0B : %d\n",recBuf[1]);

	//
	//////
//		transBuf[0] = 0x0A;
//	recBuf[0] = 0xFF;
	//		int k = i2c1.writeRead((0x21),transBuf,1,recBuf,1);
	//		PRINTF("1 read 0x01 : %d  k:%d\n",recBuf[0],k);
//	int k = i2c1.writeRead((CAM_READ),transBuf,1,recBuf,1);
//	PRINTF("2 read 0x01 : %d  k:%d\n",recBuf[0],k);

	////
	//	transBuf[0] = 0x02;
	////	k = i2c1.writeRead(CAM_READ,transBuf,1,recBuf,1);
	//	PRINTF("read 0x02 : %d\n",recBuf[0]);

}
