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
	error = 0.0f;
	lastError = 0.0f;

	pPart = 0.0f;
	iPart = 0.0f;
	dPart = 0.0f;

	pGain = 0.01f;
	iGain = 0.0f;
	dGain = 0.0f;

	i = 0.0f;
	period = 0.0f;
	dt = 0.0f;
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
		error = raw.ANGULAR_RAW_Z - desSpeed;
		period = SECONDS_NOW() - lastTime;

		if(!(cnt % 100)) PRINTF("dps error: %f, des: %f, current: %f\n",error,desSpeed,raw.ANGULAR_RAW_Z);

		i += error * period;
		dt = (error -lastError) / period;

		pPart = error * pGain;
		iPart = i * iGain;
		dPart = dt * dGain;

		controlOut = pPart + iPart + dPart;

		if(!(cnt % 100)) PRINTF("control output: %f\n",controlOut);
		cnt++;

		motor.setspeed(controlOut);

		lastTime = SECONDS_NOW();
		lastError = error;
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
		case SET_ROTAT_D:
			dGain = _val->value;
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
