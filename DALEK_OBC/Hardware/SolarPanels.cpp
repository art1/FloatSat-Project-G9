/*
 * SolarPanels.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

#include "SolarPanels.h"

SolarPanels::SolarPanels() {
	// TODO Auto-generated constructor stub
	solData.activated = false;
	solData.Voltage = 0;

}

SolarPanels::~SolarPanels() {
	// TODO Auto-generated destructor stub
}

void SolarPanels::init(){
//	adc1.config(ADC_PARAMETER_RESOLUTION,12);
}

void SolarPanels::run(){
	suspendCallerUntil(END_OF_TIME);
//	if(!isActive) init();

	while(1){
		suspendCallerUntil(NOW()+SOLAR_SAMPLERATE*MILLISECONDS);
		if(isActive()){
			solData.Voltage = adc1.read(SolarVoltageADC);
			PRINTF("read solar Voltage: %d\n",solData.Voltage);
			solar_data.publish(solData);
		} else suspendCallerUntil(END_OF_TIME);
	}

}


void SolarPanels::setActive(SOLAR_DATA sol){
	this->solData.activated = sol.activated;
}

bool SolarPanels::isActive(){
	return this->solData.activated;
}
