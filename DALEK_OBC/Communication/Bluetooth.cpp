/*
 * Bluetooth.cpp
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#include "Bluetooth.h"

const uint8_t Bluetooth::invalidBuf = {'I','N','V','A','L','I','D'};
const uint8_t Bluetooth::echoBuf = {'E','C','H','O'};


Bluetooth::Bluetooth() {

}

Bluetooth::~Bluetooth() {
	// TODO Auto-generated destructor stub
}

void Bluetooth::init(){

}

void Bluetooth::run(){

	while(1){
		while(!btBuf.isEmpty()){
			btBuf.get(tmp);
			sendNewMessage(&tmp);
			if(bt_uart.isDataReady())break;
		}
		if(bt_uart.isDataReady()) getNewMessages();
		suspendCallerUntil(NOW() + TTNC_SAMPLERATE*MILLISECONDS);
	}
}

void Bluetooth::setNewData(UDPMsg msg){
	btBuf.put(msg);
	/** TODO implement check if FIFO is full -> then stop the Telemetry! */
}

void Bluetooth::getNewMessages(){
	UDPMsg temp;

	while(bt_uart.isDataReady()){
		bt_uart.read(recBuf,BLUETOOTH_BUFFER);
		for(int i=0;i<BLUETOOTH_BUFFER;i++){
			if((recBuf[i] == FRAME_END) && (recBuf[i-1] == VALUE_SEPERATOR)){
				temp.length = i;
				break;
			}
		}

		for(int i=0;i<temp.length;i++){
			temp.data[i] = recBuf[i];
		}

		if(temp.length<5) {sendErrorMessage(temp);}
		else {
#ifdef COMMAND_ECHO
			sendEchoMessage(temp);
#endif
			tcRaw.publish(temp);
		}
	}
}

void Bluetooth::sendErrorMessage(UDPMsg invalidMsg){
	UDPMsg tmp;

	tmp.length = invalidMsg.length+7;

	uint8_t tmpArray[tmp.length];

	for(int i=0;i<tmp.length;i++){
		if(i<7){
			tmpArray[i] = invalidBuf[i];
		} else {
			tmpArray[i] = invalidMsg.data[i];
		}
	}

	sendNewMessage(&tmp);
}

void Bluetooth::sendEchoMessage(UDPMsg echoMsg){
	UDPMsg tmp;

	tmp.length = echoMsg.length+4;

	uint8_t tmpArray[tmp.length];

	for(int i=0;i<tmp.length;i++){
		if(i<4){
			tmpArray[i] = echoBuf[i];
		} else {
			tmpArray[i] = echoMsg.data[i];
		}
	}

	sendNewMessage(&tmp);
}

void Bluetooth::sendNewMessage(UDPMsg *msg){
	bt_uart.write(reinterpret_cast<const char*>(msg->data),msg->length);
}

