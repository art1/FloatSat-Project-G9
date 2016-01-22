/*
 * SolarPanels.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

#include "SolarPanels.h"

SolarPanels::SolarPanels() : Thread("SolarPanels",104, 500){
	solData.activated = false;
	solData.Voltage = 0;

}

SolarPanels::~SolarPanels() {
}

void SolarPanels::init(){
//	adc1.config(ADC_PARAMETER_RESOLUTION,12);
}

void SolarPanels::run(){
	INTERCOMM tmp;
	tmp.changedVal = SOLAR_CHANGED;
	suspendCallerUntil(END_OF_TIME);
//	if(!isActive) init();

	while(1){
		suspendCallerUntil(NOW()+SOLAR_SAMPLERATE*MILLISECONDS);

		if(isActive()){
			solData.Voltage = adc1.read(SolarVoltageADC);
//			PRINTF("read solar Voltage: %d\n",solData.Voltage);
			tmp.solData = solData;
			interThreadComm.publish(tmp);
		} else suspendCallerUntil(END_OF_TIME);

	}

}


void SolarPanels::setNewData(SOLAR_DATA sol){
	this->solData.activated = sol.activated;
}

bool SolarPanels::isActive(){
	return this->solData.activated;
}
