/*
 * currentSensors.h
 *
 *  Created on: Jan 4, 2016
 *      Author: arthur
 */

#ifndef HARDWARE_SENSORS_CURRENTSENSORS_H_
#define HARDWARE_SENSORS_CURRENTSENSORS_H_

#include "../../Basic/basic.h"

#define CURRENT_BATTERY_ADRESS			0x40
#define CURRENT_POWER_REG				0x03
#define CURRENT_CURRENT_REG				0x04


class currentSensors : public Thread {
public:
	currentSensors();
	virtual ~currentSensors();
	void init();
	void run();
private:
	void configSensors();
	void readRawData();
	void scaleData();
	uint8_t txBuf[10];
	uint8_t rxBuf[10];
	CURRENT_DATA data;
	INTERCOMM current;
};

#endif /* HARDWARE_SENSORS_CURRENTSENSORS_H_ */
