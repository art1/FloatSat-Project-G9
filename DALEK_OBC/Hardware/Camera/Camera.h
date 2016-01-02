/*
 * camera.hpp
 *
 *  Created on: 30.11.2015
 *      Author: akynos
 */

#ifndef CAMERA_HPP_

#include "../../Basic/basic.h"
#include "Supps/mySCCB.h"
#include "Supps/ov7670.h"
#include "Supps/initRegister.h"



#define WIDTH						320
#define HEIGHT						240

//#define CAPTUREMODE					DCMI_CaptureMode_SnapShot
//#define FRAMERATE					DCMI_CaptureRate_All_Frame
//#define CAPTUREMODE				DCMI_CaptureMode_Continuous
//#define FRAMERATE					DCMI_CaptureRate_1of4_Frame


#define DCMI_DR_ADDRESS      		0x50050028
//#define IMAGESIZE					(HEIGHT*WIDTH*2)
#define IMAGESIZE					(HEIGHT*WIDTH)

class Camera : public Thread {
public:
	Camera();
	void init();
	void run();
	void Capture();

	void ProcessData();
	void setNewData(CAM_DATA data);
private:
	CAM_DATA daten;
	void initTimer();
	void OV7670_SCCB();
	ov7670 cam;
	uint16_t picture[100];
	HAL_GPIO ledo;
	HAL_GPIO reset;
	HAL_GPIO power;

	bool isActive;
	bool captureImage;
	bool processData;
	bool sendPic;
};

//extern Camera camera;

#endif /* CAMERA_HPP_ */
