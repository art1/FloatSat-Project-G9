/*
 * SunFinder.h
 *
 *  Created on: Dec 30, 2015
 *      Author: arthur
 */

#ifndef MISSION_SUNFINDER_H_
#define MISSION_SUNFINDER_H_

#include "../Basic/basic.h"


#define SUNFINDER_BUFFERSIZE			300

class SunFinder : public Thread{
public:
	SunFinder();
	virtual ~SunFinder();
	void init();
	void run();
	bool isActive();
	void setActive(bool _val);
	void setNewData(LUX_DATA _lux);
	int setNewData(IMU_RPY_FILTERED _imu);
	void isStabilized();

private:
	IMU_RPY_FILTERED imu;
	int16_t luxVal;
	float initPos;
	float tempPos;
	bool threeSixty;
	bool activated;
	bool stabilized;
	Fifo<uint16_t, SUNFINDER_BUFFERSIZE> lux;

	float heading[SUNFINDER_BUFFERSIZE];
	int currentHeadingIndex;

	float findSunAngle();
	SUNFINDER_TM sunTm;
	INTERCOMM intCm;
};

#endif /* MISSION_SUNFINDER_H_ */
