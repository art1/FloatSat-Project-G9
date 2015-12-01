/*
 * camera.h
 *
 *  Created on: Nov 28, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_CAMERA_CAMERA_H_
#define HARDWARE_CAMERA_CAMERA_H_

#include "../../basic.h"
#include "Supps/SCCB.h"
#define CAM_ADDR		0x21
#define CAM_READ		0x43
#define CAM_WRITE		0x42
#define CAM_POWER		GPIO_033 // PC01
#define CAM_RESET		GPIO_010 // PA10



class Camera : public Thread {
public:
	Camera();
	virtual ~Camera();
	void init();
	void run();
	void test();
private:
//	HAL_GPIO camPWR;
//	HAL_GPIO camReset;
	SCCB sccb;
	uint8_t recBuf[10];
	uint8_t transBuf[10];
};

#endif /* HARDWARE_CAMERA_CAMERA_H_ */
