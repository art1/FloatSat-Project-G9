/*
 * currentSensors.cpp
 *
 *  Created on: Jan 4, 2016
 *      Author: arthur
 */

#include "currentSensors.h"

currentSensors::currentSensors() : Thread("Current Sensor Thread",111,500) {
	// TODO Auto-generated constructor stub
	current.changedVal = CURRENT_CHANGED;
}

currentSensors::~currentSensors() {
	// TODO Auto-generated destructor stub
}

void currentSensors::init(){

}

void currentSensors::run(){
	configSensors();
	while(1){
		readRawData();
		scaleData();
		interThreadComm.publish(current);
		suspendCallerUntil(NOW()+CURRENT_SAMPLERATE);
	}
}

void currentSensors::configSensors(){

}

void currentSensors::readRawData(){
	txBuf[0] = CURRENT_CURRENT_REG;
	i2c1.writeRead(CURRENT_BATTERY_ADRESS,txBuf,1,rxBuf,2);
	/** TODO read the values and scale them */
	txBuf[0] = CURRENT_POWER_REG;
	i2c1.writeRead(CURRENT_BATTERY_ADRESS,txBuf,1,rxBuf,2);
}

void currentSensors::scaleData(){

	current.currentData = data;
}
