/*
 * InfraredSensors.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

#include "InfraredSensors.h"

InfraredSensors::InfraredSensors() {
	// TODO Auto-generated constructor stub
	irData.activated = false;
	irData.sensorOne = 0;
	irData.sensorTwo = 0;
	irData.sensorThree = 0;
}

InfraredSensors::~InfraredSensors() {
	// TODO Auto-generated destructor stub
}

void InfraredSensors::init(){

}

void InfraredSensors::run(){
	adc1.init(IR_ONE);
	adc1.init(IR_TWO);
	adc1.init(IR_THREE);
	if(!isActive()) suspendCallerUntil(END_OF_TIME);
	while(1){
		suspendCallerUntil(NOW()+IR_SAMPLERATE*MILLISECONDS);
		if(isActive()){
			irData.sensorOne = adc1.read(IR_ONE);
			irData.sensorTwo = adc1.read(IR_TWO);
			irData.sensorThree = adc1.read(IR_THREE);
//			PRINTF("read IR Data: %d, %d, %d\n",irData.sensorOne,irData.sensorTwo,irData.sensorThree);
			ir_data.publish(irData);
		} else suspendCallerUntil(END_OF_TIME);
	}
}

void InfraredSensors::setActive(IR_DATA at){
	this->irData.activated = at.activated;
}

bool InfraredSensors::isActive(){
	return irData.activated;
}
