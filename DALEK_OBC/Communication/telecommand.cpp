/*
 * TTnC.cpp
 *
 *  Created on: Nov 6, 2015
 *      Author: arthur
 */

#include "telecommand.h"

#include "../support_libs/wifi/wf121.h"

//#include "linkinterfacewf121.h"


//HAL_UART uart3(UART_IDX3); // USB-UART
//WF121 wf121(&uart3);

Telecommand::Telecommand() {
	// TODO Auto-generated constructor stub


}

Telecommand::~Telecommand() {
	// TODO Auto-generated destructor stub
}

void Telecommand::init(){
	retStat = 0;

}

void Telecommand::setNewData(UDPMsg frame){
	this->tcBuf.put(frame);
}



void Telecommand::run(){

	bool checkFrame = true;
	bool isValid = true;

	while(1){
		suspendCallerUntil(END_OF_TIME);
		// parse Commands
		tcBuf.get(currentMsg);
		forLoop(i,3){
			if(currentMsg.data[i] != FRAME_START) checkFrame = false;
			if(currentMsg.data[currentMsg.length-i] != FRAME_END) checkFrame = false;
			PRINTF("received Frame is invalid!\n");
		}
		if(!checkFrame){
			// message seems to be valid
			int currentPos = 0;
			int lastSeparator = 1;
			int valLength = 0;
			// remove start and end chars

			for(int i=3;i<currentMsg.length-3;i++){
				tmpBuf[i-3] = currentMsg.data[i];
			}
			int messageLength = currentMsg.length-6;// -6 because frame start end end chars
			for(int i=0;i<5;i++){
				switch (i) {
				case CMD_FRAMETYPE:
					tempFrame.frameType = tmpBuf[currentPos++];
					break;
				case CMD_FRAMENUMBER:
					forLoop(j,4) smaBuf[j] = tmpBuf[currentPos++];
					tempFrame.frameNumber = charToLong(smaBuf);
					break;
				case COMMAND:
					tempFrame.command = tmpBuf[currentPos++];
					break;
				case COMMAND_VALUE:
					forLoop(j,4) smaBuf[j] = tmpBuf[currentPos++];
					tempFrame.commandValue = charToFloat(smaBuf);
					break;
				case CMD_LOCALTIME:
					forLoop(j,8) smaBuf[j] = tmpBuf[currentPos++];
					tempFrame.localTime = charToLongLong(smaBuf);
					break;
				default:
					isValid = false;
					sendErrorMessage(currentMsg);
					break;
				}
			}
			if(isValid) commandFrame.publish(tempFrame);

		} else sendErrorMessage(currentMsg);
	}
}

void Telecommand::sendErrorMessage(UDPMsg invalidMsg){

}
