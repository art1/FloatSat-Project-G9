/*
 * Bluetooth.h
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#ifndef COMMUNICATION_BLUETOOTH_H_
#define COMMUNICATION_BLUETOOTH_H_

#include "../Basic/basic.h"

class Bluetooth : public Thread{
public:
	Bluetooth();
	virtual ~Bluetooth();
	void run();
	void init();
	void setNewData(UDPMsg msg);
private:
	const uint8_t invalidBuf[7] = {'I','N','V','A','L','I','D'};
	Fifo<UDPMsg,50> btBuf;
	void getNewMessages();
	UDPMsg tmp;
	COMMAND_FRAME cmdTmp;
	char recBuf[BLUETOOTH_BUFFER];
	void sendNewMessage(UDPMsg *msg);
	void sendErrorMessage(UDPMsg invalidMsg);
};

#endif /* COMMUNICATION_BLUETOOTH_H_ */
