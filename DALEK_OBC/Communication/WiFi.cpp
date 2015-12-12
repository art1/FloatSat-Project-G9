/*
 * WiFi.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: arthur
 */

#include "WiFi.h"


HAL_UART uart3(UART_IDX3);
WF121 wf121(&uart3);


WiFi::WiFi() {
	// TODO Auto-generated constructor stub
}

WiFi::~WiFi() {
	// TODO Auto-generated destructor stub
}


void WiFi::init(){
	//	wifi.init("YETENet","yeteyete");
	//	wifi.enableUDPConnection(0xFF01A8C0,12345);
	//	int k = wifi.status();
	//	PRINTF("wifi state is: %d\n",k);
}

void WiFi::run(){
	while(1){
		while(!wifiBuf.isEmpty()){
			wifiBuf.get(tmp);
			sendNewMessage(&tmp);
			if(wf121.isDataReady())break;
		}
		if(wf121.isDataReady())	getNewMessages();
		suspendCallerUntil(NOW() + WIFI_SAMPLERATE*MILLISECONDS);
	}
}

void WiFi::setNewData(UDPMsg msg){
	wifiBuf.put(msg);
	/** TODO implement check if FIFO is full -> then stop the Telemetry! */
}

void WiFi::getNewMessages(){
	UDPMsg temp;
	while(wf121.isDataReady()){
		wf121.read(&temp);
		if(temp.length<5) {sendErrorMessage(temp);}
		else tcRaw.publish(temp);
	}
}

void WiFi::sendErrorMessage(UDPMsg invalidMsg){
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
