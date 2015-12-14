///*
// * camera.hpp
// *
// *  Created on: 30.11.2015
// *      Author: akynos
// */
//
//#ifndef CAMERA_HPP_
//
//#include "../../Basic/basic.h"
//#include "Supps/SCCB.h"
//#include "Supps/myDCMI.h"
//
//#define WIDTH						160
//#define HEIGHT						121
//
//#define CAPTUREMODE					DCMI_CaptureMode_SnapShot
//#define FRAMERATE					DCMI_CaptureRate_All_Frame
////#define CAPTUREMODE				DCMI_CaptureMode_Continuous
////#define FRAMERATE					DCMI_CaptureRate_1of4_Frame
//#define DCMI_DR_ADDRESS      		0x50050028
//#define IMAGESIZE					(HEIGHT*WIDTH*2)
//
//
//class Camera : public Thread {
//public:
//	OV7670(const char* name);
//	void init();
//	void run();
//	void Capture();
//
//	void ProcessData();
//private:
//	void initTimer();
//	void OV7670_SCCB();
//	void delayx(unsigned int ms);
//	uint8_t* getPicture();
//	void sendPicture();
//	void turnOn(void);
//	void turnOff(void);
//
//	SCCB sccb = SCCB();
//	myDCMI dcmi = myDCMI(IMAGESIZE, (uint32_t) DCMI_Buffer, FRAMERATE, CAPTUREMODE);
//	uint8_t DCMI_Buffer[IMAGESIZE];
//
//	HAL_GPIO ledo;
//	HAL_GPIO reset;
//	HAL_GPIO power;
//
//	bool active;
//	bool processData;
//	bool sendPic;
//};
//
//extern OV7670 camera;
//
//#endif /* CAMERA_HPP_ */
