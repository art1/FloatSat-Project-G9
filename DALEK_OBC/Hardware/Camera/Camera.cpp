/*
 * camera.cpp
 *
 *  Created on: 08.12.2015
 *      Author: akynos
 */


#include "Camera.h"


#define ONE_BYTE_REG_ADDR 0x01
#define TWO_BYTE_REG_ADDR 0x02



Camera::Camera() : Thread("Camera",50){
	reset = HAL_GPIO(GPIO_010); //PA10
	power = HAL_GPIO(GPIO_033); //PC01
	isActive = false;
	captureImage = false;
	//	//	processData = false;
	//	//	sendPic = true;
	initDone = false;
	//	VSync = -1;
}




void Camera::init() {
	initCamera();

}


void Camera::setNewData(CAM_DATA _data){
	this->daten = _data;
	this->isActive = this->daten.activateCamera;
	this->captureImage = this->daten.capture;
}


bool Camera::initFinished(){
	return initDone;
}


void Camera::initCamera(){
//	suspendCallerUntil(NOW()+2*SECONDS);
	PRINTF("starting cam init\n");

	//	memset(picture,0,sizeof(picture));
	reset.init(true);
	power.init(true);


	xprintf("starting cam init\n");

	reset.setPins(1);
	power.setPins(0);


	xprintf("Init GPIOs...");
	dcmi.InitGPIO();
	xprintf("Done!\n");
	xprintf("Init DCMI...");
	dcmi.InitDCMI();
	xprintf("Done!\n");

	xprintf("Init I2C...");
	delayx(1000);
	cam.Sensor_Init();
	xprintf("Done!\n");

	xprintf("Enable DCMI...");
	delayx(1000);
	dcmi.EnableDCMI();

	xprintf("Done with cam init!\n");


	//	FIFO_CS_L();
	//
	//	FIFO_CS_H();
	//	while(1 != cam.Sensor_Init());
	//
	//	PRINTF("cam init complete!\n");
	//	PRINTF("configuring interrupts\n");
	//
	//	cam.OV7670_PB7_Configuration();
	//
	//	PRINTF("Done\n");
	initDone = true;
}

void Camera::run(){
	PRINTF("cam thread started!\n");
	//	initCamera();
//	suspendCallerUntil(NOW()+1000*MILLISECONDS);


	INTERCOMM comm;
	uint8_t data[8];
	uint8_t tmp;

	while(!isActive){
		suspendCallerUntil(END_OF_TIME);
	}

	VSync = 0;

	while(1){
		//		suspendCallerUntil(END_OF_TIME);
		//		suspendCallerUntil(NOW() + 1500*MILLISECONDS);
		if(captureImage){
			ORANGE_ON;
			PRINTF("capturing imageo!\n");

			DCMI_CaptureCmd(ENABLE);
			while(!captureDone){
				suspendCallerUntil(NOW()+10*MILLISECONDS);
			}

			PRINTF("Capturing Done, sending Image:\n");
			for(int i=0;i< IMAGESIZE;i+=2){
				PRINTF("%d,",DCMI_Buffer[i]);
			}


			ORANGE_OFF;
			VSync = 0;
			suspendCallerUntil(END_OF_TIME);
		}


		//			captureImage = false;
	}

}

void Camera::sendImage(){
	PRINTF("Sending image\n");
	captureDone = true;
}



