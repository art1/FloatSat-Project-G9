/*
 * controlThread.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

#include "MotorControlThread.h"

MotorControlThread::MotorControlThread() {
	// TODO Auto-generated constructor stub

}

MotorControlThread::~MotorControlThread() {
	// TODO Auto-generated destructor stub
}

void MotorControlThread::init(){

}

void MotorControlThread::run(){
	PRINTF("init Motor Thread done...\n");
	motor.init();
	int cnt = 0;
	bool spin = false;

	while(1){
//		suspendCallerUntil(NOW()+100*MILLISECONDS);
//		motor.setspeed(cnt);
//		cnt++;
//
//		if(cnt == 1000){
//			while(1){
//				motor.switchDirection(cnt);
//				suspendCallerUntil(NOW()+5000*MILLISECONDS);
////				PRINTF("switching direction\n");
//			}
//		}
//		PRINTF("couting.. %d\n",cnt);
		ORANGE_TOGGLE;
	}
}

void MotorControlThread::setMotorSpeed(float speedCylce){
	PRINTF("setting motor speed to %f\n",speedCylce);

	motor.setspeed(speedCylce);
}

void MotorControlThread::setMotor(bool _val){
	if(_val){
		motor.startMotor();
	} else motor.stopMotor();
}
