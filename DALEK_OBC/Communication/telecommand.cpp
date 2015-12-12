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

	bool isValid = true;

	while(1){
		suspendCallerUntil(END_OF_TIME);
		// parse Commands
		tcBuf.get(currentMsg);
		if((currentMsg.data[0] == FRAME_START) && (currentMsg.data[1] == VALUE_SEPERATOR)
				&& (currentMsg.data[currentMsg.length-1] == FRAME_END)){
			// message seems to be valid
			int pos = CMD_FRAMETYPE;
			int lastSeparator = 1;
			int valLength = 0;
			for(int i=2;i<currentMsg.length;i++){
				if(currentMsg.data[i] == VALUE_SEPERATOR){
					valLength = i - lastSeparator;
					for(int j=0;j<valLength;j++){
						tmpBuf[j] = currentMsg.data[j];
					}
					switch (pos) {
						case CMD_FRAMETYPE:
							if(valLength > 1) {isValid = false;}
							else if(tmpBuf[0] != CMD) {isValid = false;}
							else{
								tempFrame.frameType = tmpBuf[0];
							}
							break;
						case CMD_FRAMENUMBER:
							/** TODO uint32_t aus uint8_ts zusammenbasteln */
							break;
						case COMMAND:
							if(valLength > 1) {isValid = false;}
							else{
								tempFrame.command = tmpBuf[0];
							}
							break;
						case COMMAND_VALUE:
							if(valLength != 4){isValid = false;}
							else{
								tempFrame.commandValue = charToFloat(tmpBuf);
							}
							break;
						case CMD_LOCALTIME:
							/** TODO zusammenbasteln der localtime */
							break;
						default:
							isValid = false;
							sendErrorMessage(currentMsg);
							break;
					}
					lastSeparator = i;
					pos++;
				}
				if(!isValid){
					sendErrorMessage(currentMsg);
					break; // ??????? TODO ?
				}
			}
			if(isValid) commandFrame.publish(tempFrame);

		} else sendErrorMessage(currentMsg);
	}

}

void Telecommand::sendErrorMessage(UDPMsg invalidMsg){

}
/**
 * puts double number to target as 8 bytes
 */
void Telecommand::doubleToChar(uint8_t* _target, double _number){
	char *tmp = (char *) &_number;
	for(int i=0;i<8;i++){
		_target[i] = tmp[i];
	}
}

/**
 * puts float-bytes to target array
 */
void Telecommand::floatToChar(uint8_t* _target, float _number){
	char *tmp = (char *) &_number;
	for(int i=0;i<4;i++){
		_target[i] = tmp[i];
	}
}

float Telecommand::charToFloat(uint8_t* _number){
	float out;
	uint8_t * ptr = (uint8_t *) &out;
	for(int i=0;i<4;i++){
		ptr[i] = _number[i];
	}
	return out;
}
double Telecommand::charToDouble(uint8_t* _number){
	double out;
	uint8_t *ptr = (uint8_t *) &out;
	for(int i=0;i<8;i++){
		ptr[i] = _number[i];
	}
	return out;
}
