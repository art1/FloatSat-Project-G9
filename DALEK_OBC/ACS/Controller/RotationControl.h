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
	float PI(float setPoint, float feedback);
	bool active;
	float desSpeed;
	float currentSpeed;

	float err;
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

	float U_1;
	float e;
	float e_1;
	float e_2;
	float a,b,c;
	float Ts;

	float piOut;


	CommBuffer<IMU_DATA_RAW> imuData;

};

#endif /* ACS_CONTROLLER_ROTATIONCONTROL_H_ */
