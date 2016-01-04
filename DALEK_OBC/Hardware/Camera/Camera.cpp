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



Camera::Camera() : Thread("Camera",99){
	reset = HAL_GPIO(GPIO_010); //PA10
	power = HAL_GPIO(GPIO_033); //PC01
	isActive = false;
	captureImage = false;
	//	processData = false;
	//	sendPic = true;
	initDone = false;
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


void Camera::setNewData(CAM_DATA _data){
	this->daten = _data;
	/** TODO activate Camera! */
	this->isActive = this->daten.activateCamera;
	this->captureImage = this->daten.capture;
}


bool Camera::initFinished(){
	return initDone;
}


void Camera::run(){



	memset(picture,0,sizeof(picture));
	reset.setPins(1);
	power.setPins(0);
	PRINTF("starting cam init\n");
	FIFO_CS_L();

	FIFO_CS_H();
	PRINTF("now sensor init\n");
	while(1 != cam.Sensor_Init());
	PRINTF("sensor init complete!\n");
	PRINTF("configuring interrupts\n");

	cam.OV7670_PB7_Configuration();
	PRINTF("Done\n");
	VSync = 0;
	INTERCOMM comm;
	uint8_t data[8];
	uint8_t tmp;
	initDone = true;
	while(!isActive){
		suspendCallerUntil(END_OF_TIME);
	}


	while(1){
//		suspendCallerUntil(END_OF_TIME);
//		suspendCallerUntil(NOW() + 1500*MILLISECONDS);
		ORANGE_ON;
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
				int toSend = 0;
				int consFrame = 0;
				for( count = 0; count < IMAGESIZE; count++ )
				{
					FIFO_RCLK_L();

//					CMOS_Data = (GPIOC->IDR<<8) & 0xff00;	  /* ( GPIO_ReadInputData(GPIOC) << 8 ) & 0xff00; */
					// read Data Pins
					data[0] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6); 	// D0
					data[1] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7); 	// D1
					data[2] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);	// D2
					data[3] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1);	// D3
					data[4] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);  // D4
					data[5] = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);	// D5
					data[6] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5);	// D6
					data[7] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6);  // D7
					tmp = 0;
					for(int i=0;i<8;i++){
						if(data[i]) tmp |= 1 << i;
					}
					PRINTF("%d;",tmp);
//					CMOS_Data = ((uint16_t)tmp << 8) & 0xff00;
					FIFO_RCLK_H();

					FIFO_RCLK_L();
//					CMOS_Data |= (GPIOC->IDR) & 0x00ff;	  /* ( GPIO_ReadInputData(GPIOC) ) & 0x00ff; */
					data[0] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6); 	// D0
					data[1] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7); 	// D1
					data[2] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);	// D2
					data[3] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1);	// D3
					data[4] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);  // D4
					data[5] = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);	// D5
					data[6] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5);	// D6
					data[7] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6);  // D7
					tmp = 0;
					for(int i=0;i<8;i++){
						if(data[i]) tmp |= 1 << i;
					}
					CMOS_Data = (tmp) & 0x00ff;
					FIFO_RCLK_H();
					PRINTF("%d;",tmp);
//					PRINTF("%d;",CMOS_Data);
//					picture[toSend++] = CMOS_Data;
//					if(toSend == 100){ // 100 since uint16_t and there must be some overhead because telemetry!
//						comm.camData.activateCamera = false;
//						comm.camData.capture = false;
//						comm.camData.sendImage = true;
//						comm.camData.picture = picture;
//						comm.camData.width = WIDTH;
//						comm.camData.height = HEIGHT;
//						comm.camData.consecutiveFrame = consFrame;
//						comm.changedVal = CAM_CHANGED;
//						interThreadComm.publish(comm);
//						suspendCallerUntil(NOW()+50*MILLISECONDS);
////						suspendCallerUntil(END_OF_TIME);
//						consFrame++;
//						toSend = 0;
//					}

				}
				ORANGE_OFF;
				suspendCallerUntil(END_OF_TIME);
				VSync = 0;
			}


//			captureImage = false;
		}
	}
}




