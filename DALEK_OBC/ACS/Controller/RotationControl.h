/*
 * RotationControl.h
 *
 *  Created on: Dec 31, 2015
 *      Author: arthur
 */

#ifndef ACS_CONTROLLER_ROTATIONCONTROL_H_
#define ACS_CONTROLLER_ROTATIONCONTROL_H_

#include "../../Basic/basic.h"

class RotationControl : public Thread{
public:
	RotationControl();
	virtual ~RotationControl();
	void init();
	void run();

};

#endif /* ACS_CONTROLLER_ROTATIONCONTROL_H_ */
