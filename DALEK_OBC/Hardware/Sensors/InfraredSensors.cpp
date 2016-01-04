/*
 * InfraredSensors.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

#include "InfraredSensors.h"

InfraredSensors::InfraredSensors() : Thread("Infrared Thread",111,1000){
	// TODO Auto-generated constructor stub

}

InfraredSensors::~InfraredSensors() {
	// TODO Auto-generated destructor stub
}

void InfraredSensors::init(){
	irData.activated = false;
	irData.sensorOne = 0;
	irData.sensorTwo = 0;
	irData.sensorThree = 0;
}

void InfraredSensors::run(){
	adc1.init(IR_ONE);
	adc1.init(IR_TWO);
	adc1.init(IR_THREE);
	INTERCOMM tmp;
	tmp.changedVal = IR_CHANGED;
	if(!isActive()) suspendCallerUntil(END_OF_TIME);
	while(1){
		suspendCallerUntil(NOW()+IR_SAMPLERATE*MILLISECONDS);
		if(isActive()){
			irData.sensorOne = (float)adc1.read(IR_ONE);
			irData.sensorTwo = (float)adc1.read(IR_TWO);
			irData.sensorThree = (float)adc1.read(IR_THREE);
//			PRINTF("read IR Data: %d, %d, %d\n",irData.sensorOne,irData.sensorTwo,irData.sensorThree);
			tmp.irData = irData;
			linearizeData();
			interThreadComm.publish(tmp);
		} else suspendCallerUntil(END_OF_TIME);
	}
}

void InfraredSensors::linearizeData(){
	/** TODO measure the linearization for correct value */
	irData.sensorOne = powf((3027.4/irData.sensorOne),1.2134);
	irData.sensorTwo = powf((3027.4/irData.sensorTwo),1.2134);
	irData.sensorThree= powf((3027.4/irData.sensorThree),1.2134);
}

void InfraredSensors::setNewData(IR_DATA at){
	this->irData = at;
}

bool InfraredSensors::isActive(){
	return irData.activated;
}
