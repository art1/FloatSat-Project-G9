/*
 * TTnC.cpp
 *
 *  Created on: Nov 6, 2015
 *      Author: arthur
 */

#include "TTnC.h"


#include "../support_libs/wifi/wf121.h"
//#include "linkinterfacewf121.h"


//HAL_UART uart3(UART_IDX3); // USB-UART
//WF121 wf121(&uart3);

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
//	wifi.init();
//	wf121.init("YETENet","yeteyete");

}

