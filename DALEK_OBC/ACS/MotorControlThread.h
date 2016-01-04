/*
 * controlThread.h
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

#ifndef ACS_MOTORCONTROLTHREAD_H_
#define ACS_MOTORCONTROLTHREAD_H_

#include "../Basic/basic.h"
#include "../Hardware/Motor.h"
#include "Controller/AngleControl.h"
#include "Controller/RotationControl.h"


class MotorControlThread :public Thread{
public:
	MotorControlThread();
	virtual ~MotorControlThread();
	void init();
	void run();
	void setMotorSpeed(float _speedCycle);
	void setMotor(bool _val);
private:
	Motor motor;
	int16_t currentDutyCycle;
};

#endif /* ACS_MOTORCONTROLTHREAD_H_ */
