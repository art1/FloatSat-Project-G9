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

	pPart = 0.0f;
	iPart = 0.0f;

	pGain = 2.0f;
	iGain = 1.4f;

	i = 0.0f;
}

RotationControl::~RotationControl() {
}


void RotationControl::init(){

}

void RotationControl::run(){
	IMU_DATA_RAW raw;
	int cnt = 0;

	while(1){
		if(!isActive()) suspendCallerUntil(END_OF_TIME);

		imuData.get(raw);
		err = (raw.ANGULAR_RAW_Z*TO_DEG) - desSpeed;
		period = SECONDS_NOW() - lastTime;

		if(!(cnt % 100)) PRINTF("dps error: %f, des: %f, current: %f\n",err,desSpeed,raw.ANGULAR_RAW_Z*TO_DEG);

		if(abs(err) > I_ERROR_LIMITATION){
			i += (err * period);
		}


		pPart = err * pGain;
		iPart = i * iGain;

		controlOut = pPart + iPart;

		// control output deckeln
		if(controlOut > 1000) controlOut = 1000;

		if(!(cnt % 100)) PRINTF("control output: %f, pPart %f, iPart %f\n",controlOut,pPart,iPart);
		cnt++;

		motor.setspeed(controlOut);

		lastTime = SECONDS_NOW();
		lastError = err;
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
