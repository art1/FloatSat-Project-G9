/*
 * TTnC.h
 *
 *  Created on: Nov 6, 2015
 *      Author: arthur
 */

#ifndef COMMUNICATION_TELECOMMAND_H_
#define COMMUNICATION_TELECOMMAND_H_
#include "../Basic/basic.h"
#include "WiFi.h"


class Telecommand : public Thread {
public:
	Telecommand();
	virtual ~Telecommand();
	void init();
	void run();
//	void receive(string message);
	void setNewData(UDPMsg frame);
//	COMMAND_FRAME getLatestData();

private:
	int retStat;
	CommBuffer<UDPMsg> tcBuf;
	UDPMsg currentMsg;
	COMMAND_FRAME tempFrame;
	uint8_t tmpBuf[255]; //255 max value from UDPMsg
	string message;
	uint8_t recBuf[512];
	uint8_t transBuf[512];

	void floatToChar(uint8_t* target, float number);
	void doubleToChar(uint8_t* target, double number);
	float charToFloat(uint8_t *_number);
	double charToDouble(uint8_t *_number);
	void sendErrorMessage(UDPMsg invalidMsg);


};

#endif /* COMMUNICATION_TELECOMMAND_H_ */
