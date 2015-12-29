/*
 * InfraredSensors.h
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_INFRAREDSENSORS_H_
#define HARDWARE_INFRAREDSENSORS_H_

#include "../Basic/basic.h"


class InfraredSensors : public Thread {
public:
	InfraredSensors();
	virtual ~InfraredSensors();
	void init();
	void run();
	void setNewData(IR_DATA dat);
	bool isActive();

private:
	IR_DATA irData;
};

#endif /* HARDWARE_INFRAREDSENSORS_H_ */
