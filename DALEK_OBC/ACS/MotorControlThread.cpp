/*
 * controlThread.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

#include "MotorControlThread.h"

MotorControlThread::MotorControlThread() : Thread("Motor Control",105,1000){


}

MotorControlThread::~MotorControlThread() {

}

void MotorControlThread::init(){

}

void MotorControlThread::run(){
	PRINTF("init Motor Thread done...\n");
	motor.init();
	int cnt = 0;
	bool spin = false;
//	forLoop(i,10)suspendCallerUntil(NOW()+1*SECONDS);

	while(1){
		suspendCallerUntil(END_OF_TIME);
//		suspendCallerUntil(NOW()+100*MILLISECONDS);
//		motor.setspeed(cnt);
//		cnt++;
//
//		if(cnt == 1000){
//			while(1){
//				motor.switchDirection(cnt);
//				suspendCallerUntil(NOW()+5000*MILLISECONDS);
//				PRINTF("switching direction\n");
//			}
//		}
//		PRINTF("couting.. %d\n",cnt);
//		ORANGE_TOGGLE;
	}
}

void MotorControlThread::setRotationSpeed(float speedCylce){
	if(angCon.isActive()) angCon.setActive(false); // deactivate Angle Control
	rotCon.setRotSpeed(speedCylce);
	rotCon.setActive(true);
	rotCon.resume();
}

void MotorControlThread::gotoAngle(float _angle){
	if(rotCon.isActive()) rotCon.setActive(false);
	angCon.setDesAngle(_angle);
	angCon.setActive(true);
	angCon.resume();

}

void MotorControlThread::setNewData(IMU_RPY_FILTERED _imu){
	angCon.setNewData(_imu);
//	PRINTF("heading: %f ",_imu.YAW);
}

void MotorControlThread::setNewData(VAR_CONTROL *_varC){
	switch (_varC->changedVal) {
		case SET_ANGLE_P:
		case SET_ANGLE_I:
		case SET_ANGLE_D:
			angCon.setNewData(_varC);
			break;
		case SET_ROTAT_P:
		case SET_ROTAT_I:
		case SET_ROTAT_D:
			rotCon.setNewData(_varC);
			break;
		default:
			break;
	}
}



void MotorControlThread::setMotor(bool _val){
	if(_val){
		motor.startMotor();
	} else motor.stopMotor();
}
