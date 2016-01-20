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
	void setNewData(VAR_CONTROL *_val);
	void setNewData(IMU_DATA_RAW _val);
private:

	bool active;
	float desSpeed;
	float currentSpeed;

	float error;
	float lastError;
	float controlOut;
	float pPart;
	float pGain;
	float iPart;
	float iGain;
	float dPart;
	float dGain;

	float i;
	float dt;
	float period;

	float lastTime;


	CommBuffer<IMU_DATA_RAW> imuData;

};

#endif /* ACS_CONTROLLER_ROTATIONCONTROL_H_ */
