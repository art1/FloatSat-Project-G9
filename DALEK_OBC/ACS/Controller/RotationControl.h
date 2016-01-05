/*
 * RotationControl.h
 *
 *  Created on: Dec 31, 2015
 *      Author: arthur
 */

#ifndef ACS_CONTROLLER_ROTATIONCONTROL_H_
#define ACS_CONTROLLER_ROTATIONCONTROL_H_

#include "../../Basic/basic.h"
#include "../../Hardware/Motor.h"

extern "C" Motor motor;

class RotationControl : public Thread{
public:
	RotationControl();
	virtual ~RotationControl();
	void init();
	void run();
	bool isActive();
	void setActive(bool _val);
	void setRotSpeed(float _speed);
private:
	bool active;
	float desSpeed;
	float currentSpeed;

};

#endif /* ACS_CONTROLLER_ROTATIONCONTROL_H_ */
