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
	static const uint8_t invalidBuf[7];
	static const uint8_t echoBuf[4];

	Fifo<UDPMsg,50> btBuf;
	void getNewMessages();
	UDPMsg tmp;
	COMMAND_FRAME cmdTmp;
	char recBuf[BLUETOOTH_BUFFER];
	void sendNewMessage(UDPMsg *msg);
	void sendErrorMessage(UDPMsg invalidMsg);
	void sendEchoMessage(UDPMsg echoMsg);
};

#endif /* COMMUNICATION_BLUETOOTH_H_ */
