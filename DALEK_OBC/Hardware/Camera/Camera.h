/*
 * camera.h
 *
 *  Created on: Nov 28, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_CAMERA_CAMERA_H_
#define HARDWARE_CAMERA_CAMERA_H_

#include "../../Basic/basic.h"
#include "Supps/SCCB.h"
//#include "Supps/myDCMI.h"


#define CAM_ADDR		0x21 //#define CAM_READ		0x43 #define CAM_WRITE		0x42

#define CAM_POWER		GPIO_033 // PC01
#define CAM_RESET		GPIO_010 // PA10

#define IMAGESIZE

class Camera : public Thread {
public:
	Camera();
	virtual ~Camera();
	void init();
	void run();
	void test();
	void setNewData(CAM_CONTROL data);
private:
	CAM_CONTROL dat;
//	HAL_GPIO camPWR;
//	HAL_GPIO camReset;
	SCCB sccb;
//	myDCMI dcmi;
	uint8_t recBuf[10];
	uint8_t transBuf[10];
};

#endif /* HARDWARE_CAMERA_CAMERA_H_ */
