/*
 * ControlThread.cpp
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#include "ControlThread.h"

ControlThread::ControlThread() {

}

ControlThread::~ControlThread() {

}

void ControlThread::run(){
	while(1){
		//check which mode
		if(cmd.command == GOTO_MODE){
			currentSystemMode.activeMode = (int) cmd.commandValue;
		} else {
			switch (currentSystemMode.activeMode) {
			case STANDBY:

				break;
			case SUN_FINDING:
				// here: search for sun,
				break;
			case MOTOR_CONTROL:

				break;
			case MISSION:

				break;
			default:
				break;
			}
		}
		suspendCallerUntil(END_OF_TIME);
	}
}

void ControlThread::init(){

}

void ControlThread::setNewData(COMMAND_FRAME _cmd){
	this->cmd = _cmd;
}
