/*
 * ThermalKnife.h
 *
 *  Created on: Dec 14, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_THERMALKNIFE_H_
#define HARDWARE_THERMALKNIFE_H_

#include "../Basic/basic.h"

class ThermalKnife : public Thread{
public:
	ThermalKnife();
	virtual ~ThermalKnife();
	void init();
	void run();
	void setNewData(KNIFE_DATA _dat);


private:
	KNIFE_DATA data;
};

#endif /* HARDWARE_THERMALKNIFE_H_ */
