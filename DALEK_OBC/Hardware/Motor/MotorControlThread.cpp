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
	while(1){
		suspendCallerUntil(NOW()+10*MILLISECONDS);
		motor.setspeed(cnt);
		cnt++;
		if(cnt > 999) cnt = -999;
		PRINTF("couting.. %d\n",cnt);
		ORANGE_TOGGLE;

	}
}

void MotorControlThread::setMotorSpeed(int16_t speedCylce){
	motor.setspeed(speedCylce);
}
