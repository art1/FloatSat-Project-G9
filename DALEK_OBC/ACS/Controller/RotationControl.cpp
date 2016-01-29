/*
 * RotationControl.cpp
 *
 *  Created on: Dec 31, 2015
 *      Author: arthur
 */

#include "RotationControl.h"

RotationControl::RotationControl() : Thread("Rotation Control",98,1000){

	active = false;
	desSpeed = 0.0f;
	err = 0.0f;
	lastError = 0.0f;

//	MAX = 700;
//	MIN = -700;

	pPart = 0.0f;
	iPart = 0.0f;
	dPart = 0.0f;

	pGain = 106.03f; //1
	iGain = 28.3f; //5
	dGain = 1.6f;


	//i = 0.0f;


	U_1 = 0.0f;
	e_1= 0.0f;
	e_2 = 0.0f;
	Ts = 0.02f;

	a = 0.0f;
	b = 0.0f;
	//c = 0.0f;
	piOut = 0.0f;
}

RotationControl::~RotationControl() {
}


void RotationControl::init(){

}

float RotationControl::PI(float setPoint, float feedback){
    e = setPoint - feedback;
    a= pGain + (iGain*(Ts/2));
    b= -pGain + (iGain*(Ts/2));
    //c= dGain/Ts;
    piOut = U_1 + a*e + b*e_1;
    //PRINTF(" ae %f be %f ce %f", a*e,b*e_1,c*e_2);
    U_1 = piOut;
    e_1 = e;
    //e_2 = e_1;

    return piOut;

}

void RotationControl::run(){
	IMU_DATA_RAW raw;
	int cnt = 0;

	while(1){
		if(!isActive()) suspendCallerUntil(END_OF_TIME);

		imuData.get(raw);
		controlOut = PI(desSpeed, raw.ANGULAR_RAW_Z);

//
//		err = desSpeed - (raw.ANGULAR_RAW_Z);
		period = SECONDS_NOW() - lastTime;
//
//		if(!(cnt % 100)) PRINTF("dps error: %f, des: %f, current: %f\n",err,desSpeed,raw.ANGULAR_RAW_Z);
//
//		if((err > 0.1) || (err < -0.1)){
//			i += (err * period);
//		}
//
//		dt = (err - lastError) / period;
//
//		pPart = err * pGain;
//		iPart = i * iGain;
//		dPart = dt * dGain;
//
//
//		controlOut = pPart + iPart + dPart;
//
//		// control output deckeln
//		//		if(controlOut > 1000) controlOut = 1000;
//		//Saturation filter
//		if (controlOut > MAX) {
//			controlOut = MAX;
//		}
//		else if (controlOut < MIN) {
//			controlOut = MIN;
//		}
//
//		if(!(cnt % 100)) PRINTF("control output: %f, pPart %f, iPart %f, dPart %f\n",controlOut,pPart,iPart,dPart);
//		if(!(cnt%100)) PRINTF("p: %f, i: %f, d: %f\n",pGain,iGain,dGain);
//		cnt++;


		motor.setspeed(controlOut);

		lastTime = SECONDS_NOW();
//		lastError = err;
		suspendCallerUntil(NOW()+IMU_SAMPLERATE*MILLISECONDS);

	}
}

void RotationControl::setRotSpeed(float _speed){
	this->desSpeed = _speed;
}
void RotationControl::setNewData(VAR_CONTROL *_val){
	switch (_val->changedVal) {
	case SET_ROTAT_P:
		pGain = _val->value;
		break;
	case SET_ROTAT_I:
		iGain = _val->value;
		break;
	default:
		break;
	}
}

void RotationControl::setNewData(IMU_DATA_RAW _val){
	this->imuData.put(_val);
}

bool RotationControl::isActive(){
	return active;
}
void RotationControl::setActive(bool _val){
	this->active = _val;
}
