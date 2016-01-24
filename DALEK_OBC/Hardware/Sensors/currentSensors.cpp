/*
 * currentSensors.cpp
 *
 *  Created on: Jan 4, 2016
 *      Author: arthur
 */

#include "currentSensors.h"

currentSensors::currentSensors() : Thread("Current Sensor Thread",111,500) {

	current.changedVal = CURRENT_CHANGED;
}

currentSensors::~currentSensors() {

}

void currentSensors::init(){

}

void currentSensors::run(){
	suspendCallerUntil(NOW()+1000*MILLISECONDS);
	configSensors();
	txBuf[0] = 0x00;
	int b = i2c1.writeRead(CURRENT_BATTERY_ADRESS,txBuf,1,rxBuf,1);
	PRINTF("Current read return: %d, reg: %d\n", b,rxBuf[0]);
	while(b < 0){
		i2c1.reset();
		i2c1.init();
		int b = i2c1.writeRead(CURRENT_BATTERY_ADRESS,txBuf,1,rxBuf,1);
		PRINTF("Current read return: %d, reg: %d\n", b,rxBuf[0]);
		suspendCallerUntil(NOW()+500*MILLISECONDS);
	}
	suspendCallerUntil(END_OF_TIME);
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
	PRINTF("read current reg: %d %d\n",rxBuf[0],rxBuf[1]);
	/** TODO read the values and scale them */
	txBuf[0] = CURRENT_POWER_REG;
	i2c1.writeRead(CURRENT_BATTERY_ADRESS,txBuf,1,rxBuf,2);
}

void currentSensors::scaleData(){

	current.currentData = data;
}
