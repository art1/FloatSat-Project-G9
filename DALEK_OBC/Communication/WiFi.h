/*
 * WiFi.h
 *
 *  Created on: Dec 2, 2015
 *      Author: arthur
 */

#ifndef COMMUNICATION_WIFI_H_
#define COMMUNICATION_WIFI_H_

#include "../support_libs/wifi/wf121.h"
#include "../Basic/basic.h"



class WiFi : public Thread {
public:
	WiFi();
	virtual ~WiFi();
	void init();
	void run();
	void setNewData(UDPMsg msg);
private:
	const uint8_t invalidBuf[7] = {'I','N','V','A','L','I','D'};
	static Fifo<UDPMsg,50> wifiBuf;
	void getNewMessages();
	UDPMsg tmp; // can be tcp message as well!
	COMMAND_FRAME cmdTmp;
	void sendNewMessage(UDPMsg *msg);
	void sendErrorMessage(UDPMsg invalidMsg);
};

#endif /* COMMUNICATION_WIFI_H_ */
