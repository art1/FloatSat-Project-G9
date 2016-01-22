/*
 * camera.cpp
 *
 *  Created on: 08.12.2015
 *      Author: akynos
 */


#include "Camera.h"


#define ONE_BYTE_REG_ADDR 0x01
#define TWO_BYTE_REG_ADDR 0x02




Camera::Camera() : Thread("Camera",99){
	reset = HAL_GPIO(GPIO_010); //PA10
	power = HAL_GPIO(GPIO_033); //PC01
	isActive = false;
	captureImage = false;
	//	processData = false;
	//	sendPic = true;
	initDone = false;
	VSync = -1;
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

void Camera::initCamera(){
	memset(picture,0,sizeof(picture));
	reset.setPins(1);
	power.setPins(0);
	PRINTF("starting cam init\n");
	FIFO_CS_L();

	FIFO_CS_H();
	while(1 != cam.Sensor_Init());
	PRINTF("cam init complete!\n");
	PRINTF("configuring interrupts\n");

	cam.OV7670_PB7_Configuration();
	PRINTF("Done\n");
	initDone = true;
}

void Camera::run(){



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
					data[7] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6); 	// D0 - R4
					data[6] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7); 	// D1 - R3
					data[5] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);	// D2 - R2
					data[4] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1);	// D3 - R1
					data[3] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);  // D4 - R0
					data[2] = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);	// D5 - G5
					data[1] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5);	// D6 - G4
					data[0] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6);  // D7 - G3
					tmp = 0;

					for(int i=0;i<8;i++){
						if(data[i]) tmp |= 1 << i;
					}
					//					PRINTF("%d;",tmp);
					CMOS_Data = (((uint16_t)tmp) << 8) & 0xff00;
					FIFO_RCLK_H();
//										if(!(count%WIDTH)){
//					//						PRINTF("%d;",CMOS_Data);
//											PRINTF("%d\n",tmp);
//										}else{
											PRINTF("%d,",tmp);
//					//						PRINTF("%d,",CMOS_Data);
//										}
					FIFO_RCLK_L();
					//					CMOS_Data |= (GPIOC->IDR) & 0x00ff;	  /* ( GPIO_ReadInputData(GPIOC) ) & 0x00ff; */
					data[7] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6); 	// D0 - G2
					data[6] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7); 	// D1 - G1
					data[5] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);	// D2 - G0
					data[4] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1);	// D3 - B4
					data[3] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);  // D4 - B3
					data[2] = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);	// D5 - B2
					data[1] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5);	// D6 - B1
					data[0] = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6);  // D7 - B0
					tmp = 0;
					for(int i=0;i<8;i++){
						if(data[i]) tmp |= 1 << i;
					}
					CMOS_Data |= ((uint16_t)tmp) & 0x00ff;
					FIFO_RCLK_H();
//					if(!(count%WIDTH)){
////						PRINTF("%d;",CMOS_Data);
//						PRINTF("%d\n",tmp);
//					}else{
						PRINTF("%d,",tmp);
////						PRINTF("%d,",CMOS_Data);
//					}
//					if(!(count%WIDTH)){
//						PRINTF("%d\n",CMOS_Data);
//						//						PRINTF("%d;\n",tmp);
//					}else{
//						//						PRINTF("%d,",tmp);
//						PRINTF("%d,",CMOS_Data);
//					}

//						suspendCallerUntil(NOW()+10*MILLISECONDS);


					//					PRINTF("%d;",tmp);
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




