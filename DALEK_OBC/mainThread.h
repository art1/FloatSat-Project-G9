/*
 * mainThread.h
 *
 *  Created on: Nov 1, 2015
 *      Author: arthur
 */

#ifndef MAINTHREAD_H_
#define MAINTHREAD_H_

#include "basic.h"
//#include "Hardware/GPIO_LED.h"
#include "Hardware/IMU.h"
#include "Communication/TTnC.h"
#include "Hardware/Filter/sensorFusion.h"

class mainThread : public Thread {
public:
	mainThread(const char* name);
	void init();
	void run();
};

mainThread mainT("main");

#endif /* MAINTHREAD_H_ */
