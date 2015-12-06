/*
 * SolarPanels.h
 *
 *  Created on: Dec 1, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_SOLARPANELS_H_
#define HARDWARE_SOLARPANELS_H_

#include "../Basic/basic.h"


class SolarPanels : public Thread {
public:
	SolarPanels();
	virtual ~SolarPanels();
	void init();
	void run();
	void setActive(SOLAR_DATA sol);
	bool isActive();

private:
	SOLAR_DATA solData;
};

#endif /* HARDWARE_SOLARPANELS_H_ */
