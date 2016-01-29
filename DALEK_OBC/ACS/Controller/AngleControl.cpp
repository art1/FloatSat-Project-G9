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
	lastError = 0.0f;
	desAng = 0.0f;
	heading = 0.0f;

	pPart = 0.0f;
	iPart = 0.0f;
	dPart = 0.0f;

	pGain = 125.0f;//-1060.2792518954f / 1000.0f;
	iGain = 0.0f;//0.009f;//-283.266890153188f/ 1000.0f;
	dGain = 0.5f;//-160.381399303853/ 1000.0f;

	//	    MAX = 700;
	//	    MIN = -700;

	i = 0.0f;
	period = 0.0f;
	dt = 0.0f;

	U_1 = 0.0f;
	e_1= 0.0f;
	e_2 = 0.0f;
	Ts = 0.02f;

	a = 0.0f;
	b = 0.0f;
	c = 0.0f;
	pidOut = 0.0f;


}

AngleControl::~AngleControl() {

}

void AngleControl::init(){

}

float AngleControl::PID(float setPoint, float feedback){
	e = setPoint - feedback;
	a= pGain + (iGain*(Ts/2)) + (dGain/Ts);
	b= -pGain + (iGain*(Ts/2)) - (2*dGain/Ts);
	c= dGain/Ts;

	//if(abs(e) > I_ERROR_LIMITATION){
	//    i += error*period;
	//}

	pidOut = U_1 + a*e + b*e_1 + c*e_2;
	PRINTF(" ae %f be %f ce %f", a*e,b*e_1,c*e_2);
	U_1 = pidOut;
	e_1 = e;
	e_2 = e_1;
	//	PRINTF("e: %f a %f b %f c %f U1 %fpidOut %f\n",e,a,b,c,U_1,pidOut);
	return pidOut;
}

void AngleControl::run(){
	IMU_RPY_FILTERED rpy;
	int cnt = 0;

	while(1){

		if(!isActive()) suspendCallerUntil(END_OF_TIME);

		imuData.get(rpy);

		//		controlOut = PID(rpy.YAW,desAng);
		//		controlOut = PID(desAng, rpy.YAW);
		period = SECONDS_NOW() - lastTime;

		error = desAng*TO_RAD - rpy.YAW*TO_RAD;

		if(error > 180.0) error -= 360.0;
		else if(error < -180.0) error += 360.0;

		period = SECONDS_NOW() - lastTime;

		if(!(cnt % 100)) PRINTF("angle error: %f, des: %f, current: %f\n",error,desAng*TO_RAD,rpy.YAW*TO_RAD);

		if((error > 0.5) || (error < 0.5)){
			i += error*period;
		}

		dt = (error - lastError) / period;

		pPart = error * pGain;
		iPart = i * iGain;
		dPart = dt * dGain;

		controlOut = pPart + iPart + dPart;
		if(!(cnt % 100)) PRINTF("pPart %f iPart %f dPart %f\n",pPart,iPart,dPart);

		//Saturation filter
		if (controlOut > MAX) {
			controlOut = MAX;
		}
		else if (controlOut < MIN) {
			controlOut = MIN;
		}

		if(!(cnt % 100)) PRINTF("control output: %f\n",controlOut);
		cnt++;

		motor.setspeed((int16_t)controlOut);

		lastTime = SECONDS_NOW();
		lastError = error;
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
		dGain = _val->value;
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
