/*
 * AngleControl.h
 *
 *  Created on: Dec 31, 2015
 *      Author: arthur
 */

#ifndef ACS_CONTROLLER_ANGLECONTROL_H_
#define ACS_CONTROLLER_ANGLECONTROL_H_

#include "../../Basic/basic.h"

class AngleControl : public Thread {
public:
	AngleControl();
	virtual ~AngleControl();
	void init();
	void run();
private:
};

#endif /* ACS_CONTROLLER_ANGLECONTROL_H_ */
