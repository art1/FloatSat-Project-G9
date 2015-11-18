/*
 * TTnC.h
 *
 *  Created on: Nov 6, 2015
 *      Author: arthur
 */

#ifndef COMMUNICATION_TTNC_H_
#define COMMUNICATION_TTNC_H_
#include "../basic.h"
#include "rodos.h"
#include "../support_libs/wifi/wf121.h"


class TTnC : public Thread {
public:
	TTnC();
	virtual ~TTnC();
	void init();
	void run();
private:
	int retStat;
	uint8_t recBuf[512];
	uint8_t transBuf[512];
};

#endif /* COMMUNICATION_TTNC_H_ */
