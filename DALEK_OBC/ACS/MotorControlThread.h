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

extern "C" Motor motor;

class MotorControlThread :public Thread{
public:
	MotorControlThread();
	virtual ~MotorControlThread();
	void init();
	void run();
	void setRotationSpeed(float _speedCycle);
	void gotoAngle(float _angle);
	void setMotor(bool _val);
	void setNewData(IMU_RPY_FILTERED _imu);
	void setNewData(VAR_CONTROL *_varC);
private:
	AngleControl angCon;
	RotationControl rotCon;
	int16_t currentDutyCycle;
};

#endif /* ACS_MOTORCONTROLTHREAD_H_ */
