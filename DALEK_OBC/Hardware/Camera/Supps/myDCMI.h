/*
 * DCMI.h
 *
 *  Created on: Dec 6, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_CAMERA_SUPPS_MYDCMI_H_
#define HARDWARE_CAMERA_SUPPS_MYDCMI_H_

#include "stm32f4xx.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "../../../Basic/basic.h"

class myDCMI {
public:
	myDCMI();
	virtual ~myDCMI();
	void init();
	void enable();
	void disable();
	void startClock();
	void stopClock();
};

#endif /* HARDWARE_CAMERA_SUPPS_MYDCMI_H_ */
