/*
 * TTnC.cpp
 *
 *  Created on: Nov 6, 2015
 *      Author: arthur
 */

#include "TTnC.h"


HAL_SPI spi2(SPI_IDX3);
HAL_UART uart3(UART_IDX3);

WF121 wifi(&uart3);

TTnC::TTnC() {
	// TODO Auto-generated constructor stub

}

TTnC::~TTnC() {
	// TODO Auto-generated destructor stub
}

void TTnC::init(){
	retStat = 0;

}

void TTnC::run(){
	PRINTF("init wifi\n");
	retStat = wifi.init(TTNC_SSID,TTNC_SSID_PW);
	PRINTF("restat wifi: %d",retStat);
	RED_ON;
	while(1){
		PRINTF("current adress: %d.%d.%d.%d\n",wifi.getAdress().a[0],wifi.getAdress().a[1],wifi.getAdress().a[2],wifi.getAdress().a[3]);
		suspendCallerUntil(NOW()+1000*MILLISECONDS);
	}


}

