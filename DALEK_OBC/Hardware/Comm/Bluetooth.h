/*
 * Bluetooth.h
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_COMM_BLUETOOTH_H_
#define HARDWARE_COMM_BLUETOOTH_H_

#include "../../Basic/basic.h"




class Bluetooth : public Thread{
public:
	Bluetooth();
	virtual ~Bluetooth();
	void run();
	void init();
	void setNewData(UDPMsg msg);
private:
//	static const uint8_t invalidBuf[7];
//	static const uint8_t echoBuf[4];

	Fifo<UDPMsg,50> btBuf;
	void getNewMessages();
	UDPMsg tmp;
	COMMAND_FRAME cmdTmp;
	char recBuf[BLUETOOTH_BUFFER];
	void sendNewMessage(UDPMsg *msg);
	void sendErrorMessageHere(UDPMsg invalidMsg);
	void sendEchoMessage(UDPMsg echoMsg);
};

#endif /* HARDWARE_COMM_BLUETOOTH_H_ */
