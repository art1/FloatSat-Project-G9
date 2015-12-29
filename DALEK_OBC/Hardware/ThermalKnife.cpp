/*
 * ThermalKnife.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: arthur
 */

#include "ThermalKnife.h"

//HAL_GPIO HBRIDGE_EN2(GPIO_066);

HAL_GPIO HBRIDGE_C_INA (GPIO_072);
HAL_PWM knifePWM(PWM_IDX14);


ThermalKnife::ThermalKnife() : Thread("Thermal Knife",105, 500){
	data.activated = false;
}

ThermalKnife::~ThermalKnife() {

}


void ThermalKnife::init(){

	knifePWM.init(5000, 1000);
	HBRIDGE_C_INA.init(true, 1, 0);

}
void ThermalKnife::setNewData(KNIFE_DATA _data){
	this->data = _data;
}



void ThermalKnife::run(){
	while(1){

		suspendCallerUntil(END_OF_TIME);

		if(data.activated){
			PRINTF("cutting wire\n");
			suspendCallerUntil(NOW()+500*MILLISECONDS);
			knifePWM.write(999);
			HBRIDGE_C_INA.setPins(1);
			suspendCallerUntil(NOW()+5000*MILLISECONDS);
			suspendCallerUntil(NOW()+5000*MILLISECONDS);
			suspendCallerUntil(NOW()+5000*MILLISECONDS);
			HBRIDGE_C_INA.setPins(0);
			knifePWM.write(0);
			PRINTF("stopping\n");
			suspendCallerUntil(END_OF_TIME);
		}
	}

}
