/*
 * camera.hpp
 *
 *  Created on: 30.11.2015
 *      Author: akynos
 */

#ifndef CAMERA_HPP_

#include "../../Basic/basic.h"
//#include "Supps/Dcmi.h"
//#include "Supps/mySCCB.h"
//#include "Supps/ov7670.h"
//#include "Supps/initRegister.h"
#include "Supps/SCCB.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"


extern "C" void DCMI_IRQHandler();

//extern "C" uint8_t init_registers[][2];


#define DCMI_DR_ADDRESS      		0x50050028
#define IMAGESIZE					(HEIGHT*WIDTH*2)
#define THRESHOLD					165
#define MINPIXELTHRESHOLD			80

#define Q1							0.25f
#define HALF						0.5f
#define Q3							0.75f


class Camera : public Thread {
public:
	Camera();
	void init();
	void run();
	void Capture();

	void ProcessData();
	void setNewData(CAM_DATA data);
	void initCamera();
	bool initFinished();
	void sendImage();
private:
	int retVal;
	uint8_t read[2];
	uint8_t tmp[4];
	uint8_t startTransmission[13];// = {36, 36, 36, (uint8_t)PL, 0, (uint8_t)160};
	int startTransmissionLength;
	uint8_t endTransmission[11];// = {36, 35, 36, 35};
	int endTransmissionLength;
	uint8_t DCMI_Buffer[IMAGESIZE];
//	Dcmi dcmi = Dcmi(IMAGESIZE, (uint32_t) DCMI_Buffer, FRAMERATE, CAPTUREMODE);
	CAM_DATA daten;
//	ov7670 cam;
	Sccb sccb;
	HAL_GPIO ledo;
	HAL_GPIO reset;
	HAL_GPIO power;

	void initPeripherals();

	bool initDone;
	bool isActive;
	bool captureImage;
	bool processData;
	bool sendPic;
	uint8_t picture[160];
	int consFrame;
	int toSend;
	void transmitPicture();
};

//extern Camera camera;


#endif /* CAMERA_HPP_ */
