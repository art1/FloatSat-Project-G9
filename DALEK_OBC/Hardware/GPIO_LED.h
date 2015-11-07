/*
 * Template.h
 *
 *  Created on: Oct 30, 2015
 *      Author: arthur
 */

#ifndef GPIO_LED_H_
#define GPIO_LED_H_

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
//#include "Hardware/IMU.h"

class GPIO_LED: public Thread {

public:
	GPIO_LED(const char* name);
	void init();
	void run();
	void switchLED(HAL_GPIO led,int on);
	void blinkAll(int speedInMsec, int stayOn);
	void crossblink(int speedInMSec, int stayOn);

};


#endif /* GPIO_LED_H_ */
