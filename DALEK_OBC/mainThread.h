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
#include "Hardware/AHRS/IMU.h"
#include "Communication/TTnC.h"
#include "Hardware/Camera/Camera.h"
#include "Hardware/AHRS/Filter/sensorFusion.h"
#include "Hardware/lightSensor.h"
#include "Hardware/Motor/MotorControlThread.h"
#include "Hardware/SolarPanels.h"
#include "Hardware/InfraredSensors.h"

class mainThread : public Thread {
public:
	mainThread(const char* name);
	void init();
	void run();
};

mainThread mainT("main");

#endif /* MAINTHREAD_H_ */
