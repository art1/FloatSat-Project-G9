/*
 * ControlThread.h
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#ifndef CONTROLTHREAD_H_
#define CONTROLTHREAD_H_
#include "Basic/basic.h"


class ControlThread : public Thread{
public:
	ControlThread();
	virtual ~ControlThread();
	void init();
	void run();
	void setNewData(COMMAND_FRAME _cmd);
private:
	COMMAND_FRAME cmd;
};

#endif /* CONTROLTHREAD_H_ */
