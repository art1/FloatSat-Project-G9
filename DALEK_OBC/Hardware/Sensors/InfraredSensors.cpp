/*
 * InfraredSensors.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

// ADC1 linearization equation: y= (366184 / (x - 176)) - 11
// ADC2 linearization equation: y= (349312 /( x -72)) -11
// ADC3 linearization equation: y= 367980 /( x -114) -11


#include "InfraredSensors.h"

InfraredSensors::InfraredSensors() : Thread("Infrared Thread",111,1000){
}

InfraredSensors::~InfraredSensors() {
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
			irData.sensorOne = adc1.read(IR_ONE);
			irData.sensorTwo = adc1.read(IR_TWO);
			irData.sensorThree = adc1.read(IR_THREE);
			linearizeData();
			tmp.irData = irData;
			interThreadComm.publish(tmp);
		} else suspendCallerUntil(END_OF_TIME);
	}
}

void InfraredSensors::linearizeData(){
	/** TODO measure the linearization for correct value */
//	irData.sensorOne = powf((3027.4/irData.sensorOne),1.2134);
//	irData.sensorTwo = powf((3027.4/irData.sensorTwo),1.2134);
//	irData.sensorThree= powf((3027.4/irData.sensorThree),1.2134);

	irData.sensorOne = (float)(366184 / (irData.sensorOne -176)) -11;
	irData.sensorTwo = (float)(349312 / (irData.sensorTwo -72)) -11;
	irData.sensorThree = (float)(367980 / (irData.sensorThree -114)) -11;
}

void InfraredSensors::setNewData(IR_DATA at){
	this->irData = at;
}

bool InfraredSensors::isActive(){
	return irData.activated;
}
