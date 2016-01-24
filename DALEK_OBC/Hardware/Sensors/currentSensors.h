/*
 * currentSensors.h
 *
 *  Created on: Jan 4, 2016
 *      Author: arthur
 */

#ifndef HARDWARE_SENSORS_CURRENTSENSORS_H_
#define HARDWARE_SENSORS_CURRENTSENSORS_H_

#include "../../Basic/basic.h"

#define CURRENT_SENSOR_ADRESS			0x40
#define CURRENT_CONFIG_REG				0x00
#define CURRENT_BUS_VOLTAGE				0x02
#define CURRENT_POWER_REG				0x03
#define CURRENT_CURRENT_REG				0x04
#define CURRENT_CALIBRATION				0x05
#define SHUNT_RESISTOR					0.002 		// R15 on Extension Board Schematic, in Ohm




class currentSensors : public Thread {
public:
	currentSensors();
	virtual ~currentSensors();
	void init();
	void run();
private:
	void configSensors();
	void readRawData();
	uint8_t txBuf[10];
	uint8_t rxBuf[10];
	CURRENT_DATA data;
	INTERCOMM current;
	int currentDivider_mA;
	int powerDivider_mW;
	uint8_t calVal;
};

#endif /* HARDWARE_SENSORS_CURRENTSENSORS_H_ */
