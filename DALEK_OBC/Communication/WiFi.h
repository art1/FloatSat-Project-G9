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



class WiFi {
public:
	WiFi();
	virtual ~WiFi();
	void init();
};

#endif /* COMMUNICATION_WIFI_H_ */
