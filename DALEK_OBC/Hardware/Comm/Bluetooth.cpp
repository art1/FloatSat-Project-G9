/*
 * Bluetooth.cpp
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#include "Bluetooth.h"

const uint8_t invalidBuf[7] = {'I','N','V','A','L','I','D'};
const uint8_t echoBuf[4] = {'E','C','H','O'};


Bluetooth::Bluetooth() : Thread("Bluetooth",110,2000) {

}

Bluetooth::~Bluetooth() {

}

void Bluetooth::init(){

}

void Bluetooth::run(){
#ifndef WIFI_ENABLE
	while(1){
		while(!btBuf.isEmpty()){
			btBuf.get(tmp);
			sendNewMessage(&tmp);
			if(bt_uart.isDataReady())break;
		}
		if(bt_uart.isDataReady()) getNewMessages();
		suspendCallerUntil(NOW() + TTNC_SAMPLERATE*MILLISECONDS);
	}
#endif
}

void Bluetooth::setNewData(UDPMsg msg){
	btBuf.put(msg);
	/** TODO implement check if FIFO is full -> then stop the Telemetry! */
}

void Bluetooth::getNewMessages(){
#ifndef WIFI_ENABLE
	UDPMsg temp;
	temp.length = 0;
//	memset(temp.data,0,sizeof(temp.data));
	memset(recBuf,0,sizeof(recBuf));

	PRINTF("Received!\n");
	int number = 0;

	if(bt_uart.isDataReady()){

		int k = bt_uart.read(recBuf,BLUETOOTH_BUFFER);
		PRINTF("read %d chars\n",k);

		if(k >= COMMAND_FRAME_SIZE){
			for(int i=0;i<COMMAND_FRAME_SIZE;i++)
			{
				temp.data[i] = recBuf[i];
				PRINTF("%d ",recBuf[i]);
				temp.length++;
			}
			PRINTF("\n length is %d\n",temp.length);



			if(temp.length<5) {
				sendErrorMessageHere(temp);

			}
			else {
#ifdef COMMAND_ECHO
				sendErrorMessageHere(temp);
#endif
				tcRaw.publish(temp);
			}
		}
	}
#endif
}

void Bluetooth::sendErrorMessageHere(UDPMsg invalidMsg){
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
#ifndef WIFI_ENABLE
	bt_uart.write(reinterpret_cast<const char*>(msg->data),msg->length);
#endif
}

