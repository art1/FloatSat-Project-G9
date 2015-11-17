/*
 * TTnC.cpp
 *
 *  Created on: Nov 6, 2015
 *      Author: arthur
 */

#include "TTnC.h"


HAL_SPI spi2(SPI_IDX3);


TTnC::TTnC() {
	// TODO Auto-generated constructor stub

}

TTnC::~TTnC() {
	// TODO Auto-generated destructor stub
}

void TTnC::init(){
	int k=0;
	k = spi2.init(1250000);
	PRINTF("init: %d\n",k);
	//	 setting SPI Mode:
	k = spi2.config(SPI_PARAMETER_MODE,1);
	PRINTF("config: %d\n",k);
	transBuf[0] = 0x08;
	transBuf[1] = 0x00;
	transBuf[2] = 0x01;
	transBuf[3] = 0x02;
	spi2.write(transBuf,4);
	spi2.read(recBuf,4);
	PRINTF("received bytes: ");
	for(int i = 0;i< 4;i++){
		PRINTF(" %d ",recBuf[i]);
	}
	PRINTF("\n");
}

void TTnC::run(){

}

