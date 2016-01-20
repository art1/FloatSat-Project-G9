/*
 * AngleControl.cpp
 *
 *  Created on: Dec 31, 2015
 *      Author: arthur
 */

#include "AngleControl.h"

AngleControl::AngleControl() : Thread("Angle Control",97,1000){
	active = false;
	error = 0.0f;
	desAng = 0.0f;
	heading = 0.0f;

	pPart = 0.0f;
	iPart = 0.0f;

	pGain = 0.01f;
	iGain = 0.0f;

}

AngleControl::~AngleControl() {

}

void AngleControl::init(){

}

void AngleControl::run(){
	IMU_RPY_FILTERED rpy;
	int cnt = 0;
	while(1){
		if(!isActive()) suspendCallerUntil(END_OF_TIME);
		imuData.get(rpy);
		error = rpy.YAW - desAng;
		if(!(cnt % 100))PRINTF("angle error: %f, des: %f, current: %f\n",error,desAng,rpy.YAW);
		pPart = error * pGain;
		controlOut = pPart;
		if(!(cnt % 100))PRINTF("control output: %f\n",controlOut);
		cnt++;

		motor.setspeed((int16_t)controlOut);
		suspendCallerUntil(NOW()+IMU_SAMPLERATE*MILLISECONDS);
	}
}


void AngleControl::setNewData(IMU_RPY_FILTERED _imu){
	this->imuData.put(_imu);
}
void AngleControl::setNewData(VAR_CONTROL *_val){
	switch (_val->changedVal) {
		case SET_ANGLE_P:
			pGain = _val->value;
			break;
		case SET_ANGLE_I:
			iGain = _val->value;
			break;
		case SET_ANGLE_D:

			break;
		default:
			break;
	}
}

void AngleControl::setDesAngle(float _val){
	this->desAng = _val;
	while(desAng < 0) desAng += 360.0;
	while(desAng > 360.0) desAng -= 360.0;
}

bool AngleControl::isActive(){
	return active;
}

void AngleControl::setActive(bool _val){
	this->active = _val;
}
