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

Telecommand::Telecommand() : Thread("Telecommand",102,1000){
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
		checkFrame = true;
		isValid = true;
		forLoop(i,3){
//			PRINTF("comparing %d with %d\n",currentMsg.data[i],FRAME_START);
//			PRINTF("comparing %d with %d\n",currentMsg.data[currentMsg.length-i-1],FRAME_END);
			if(currentMsg.data[i] != FRAME_START) checkFrame = false;
			if(currentMsg.data[currentMsg.length-i-1] != FRAME_END) checkFrame = false;
			//			PRINTF("received Frame is invalid!\n");
		}
//		PRINTF("checkframe %d",checkFrame);
//				for(int i=0;i<currentMsg.length;i++){
//					PRINTF("%d",currentMsg.data[i]);
//				}
		if(checkFrame){
			// message seems to be valid
			PRINTF("got valid message!\n");
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
					PRINTF("frametype %d\n",tempFrame.frameType);
					break;
				case CMD_FRAMENUMBER:
					forLoop(j,4) smaBuf[j] = tmpBuf[currentPos++];
					tempFrame.frameNumber = charToLong(smaBuf);
					PRINTF("framenumber %d\n",tempFrame.frameNumber);
					break;
				case COMMAND:
					tempFrame.command = tmpBuf[currentPos++];
					PRINTF("command is %d\n",tempFrame.command);
					break;
				case COMMAND_VALUE:
					forLoop(j,4) smaBuf[j] = tmpBuf[currentPos++];
					tempFrame.commandValue = charToFloat(smaBuf);
					PRINTF("command value %f\n",tempFrame.commandValue);
					break;
				case CMD_LOCALTIME:
					forLoop(j,8) smaBuf[j] = tmpBuf[currentPos++];
					tempFrame.localTime = charToLongLong(smaBuf);
					break;
				default:
					isValid = false;
					sendErrorMessageHere(currentMsg);
					break;
				}
			}
			if(isValid) commandFrame.publish(tempFrame);

		} else {
			PRINTF("received Frame is invalid!\n");
			sendErrorMessageHere(currentMsg);
		}
	}
}

void Telecommand::sendErrorMessageHere(UDPMsg invalidMsg){

}
