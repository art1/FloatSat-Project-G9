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
};

#endif /* HARDWARE_THERMALKNIFE_H_ */
