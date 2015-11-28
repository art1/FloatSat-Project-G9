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



class Camera : public Thread {
public:
	Camera();
	virtual ~Camera();
	void init();
	void run();

private:

};

#endif /* HARDWARE_CAMERA_CAMERA_H_ */
