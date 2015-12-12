/*
 * mainThread.h
 *
 *  Created on: Nov 1, 2015
 *      Author: arthur
 */

#ifndef MAINTHREAD_H_
#define MAINTHREAD_H_

#include "Basic/basic.h"
#include "Communication/telecommand.h"
#include "Communication/telemetry.h"
#include "Communication/WiFi.h"
#include "Hardware/AHRS/IMU.h"
#include "Hardware/Camera/Camera.h"
#ifdef FUSION_ENABLE
#include "Hardware/AHRS/Filter/sensorFusion.h"
#endif
#include "Hardware/LightSensor.h"
#include "Hardware/Motor/MotorControlThread.h"
#include "Hardware/SolarPanels.h"
#include "Hardware/InfraredSensors.h"
#include "ControlThread.h"


class mainThread : public Thread {
public:
	mainThread(const char* name);
	void init();
	void run();
};

mainThread mainT("main");

#endif /* MAINTHREAD_H_ */
