/*
 * motor.h
 *
 *  Created on: Nov 30, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_MOTOR_H_
#define HARDWARE_MOTOR_H_

#include "../Basic/basic.h"

#define MOTOR_RAMP_THRESHOLD	150		// if change in duty cycle exceeds 15% the motorspeed is slowly increased/decreased to the set value ( otherwise motor draws too much current)

class Motor/* : public Thread*/{
public:
	Motor();
	virtual ~Motor();
	void init();
	void run();
	int setspeed(int16_t duty);
	int startMotor();
	int stopMotor();
	int switchDirection(int currentSpeed);
	void spinDownTo(int _currentSpeed, int _finalVal);
	void spinUpTo(int _currentSpeed, int _finalVal);
private:
	int16_t dutyCycle;
	bool clockwise;
	int16_t lastDutyCycle;



};


#endif /* HARDWARE_MOTOR_H_ */
