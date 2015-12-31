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
#ifndef BLUETOOTH_FALLBACK
#include "Communication/WiFi.h"
#else
#include "Communication/Bluetooth.h"
#endif
#include "Hardware/IMU.h"
#include "Hardware/Camera/Camera.h"
#ifdef FUSION_ENABLE
#include "AHRS/Filter/sensorFusion.h"
#endif
#include "Hardware/LightSensor.h"
#include "ACS/MotorControlThread.h"
#include "Hardware/SolarPanels.h"
#include "Hardware/InfraredSensors.h"
#include "Hardware/ThermalKnife.h"
#include "Mission/SunFinder.h"

INTERCOMM tempComm;


class mainThread : public Thread {
public:
	mainThread(const char* name);
	void init();
	void run();
	void setNewData(COMMAND_FRAME _t);
private:
	COMMAND_FRAME cmd;
	ACTIVE_SYSTEM_MODE currentSystemMode;

};

mainThread mainT("main");

#endif /* MAINTHREAD_H_ */
