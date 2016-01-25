/*
 * camera.hpp
 *
 *  Created on: 30.11.2015
 *      Author: akynos
 */

#ifndef CAMERA_HPP_

#include "../../Basic/basic.h"
#include "Supps/Dcmi.h"
#include "Supps/mySCCB.h"
#include "Supps/ov7670.h"
#include "Supps/initRegister.h"




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
	uint8_t DCMI_Buffer[IMAGESIZE];
	Dcmi dcmi = Dcmi(IMAGESIZE, (uint32_t) DCMI_Buffer, FRAMERATE, CAPTUREMODE);
	CAM_DATA daten;
	ov7670 cam;
	HAL_GPIO ledo;
	HAL_GPIO reset;
	HAL_GPIO power;

	bool initDone;
	bool isActive;
	bool captureImage;
	bool processData;
	bool captureDone;
	bool sendPic;
};

//extern Camera camera;

#endif /* CAMERA_HPP_ */
