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


ThermalKnife::ThermalKnife() {
	// TODO Auto-generated constructor stub

}

ThermalKnife::~ThermalKnife() {
	// TODO Auto-generated destructor stub
}


void ThermalKnife::init(){
	//    HBRIDGE_EN2.init(true, 1, 1);

	knifePWM.init(5000, 1000);
	//    HBRIDGE_EN.init(true, 1, 1);
	HBRIDGE_C_INA.init(true, 1, 0);
	//    HBRIDGE_A_INB.init(true, 1, 1);
}

void ThermalKnife::run(){
	while(1){
		suspendCallerUntil(NOW()+10*SECONDS);
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
