/*
 * SCCB.cpp
 *
 *  Created on: Nov 28, 2015
 *      Author: arthur
 */

#include "SCCB.h"

SCCB::SCCB() {
	// TODO Auto-generated constructor stub

}

SCCB::~SCCB() {
	// TODO Auto-generated destructor stub
}

void SCCB::init(){
#ifndef LIGHT_ENABLE
	i2c1.init(400000);
#endif
	retVal = 0;
	// read reg 0x01, return should be 0x80
	retVal = camera_reg_read(0x01,recBuf,1);
	PRINTF("retVal of 0x01 is %d, and readretVal %d\n",recBuf[0],retVal);

}

int SCCB::camera_reg_read(uint8_t reg, uint8_t *dest, uint8_t len){
	return i2c1.writeRead(CAM_READ,&reg,1,dest,len);

}

int SCCB::camera_reg_write(uint8_t reg, uint8_t data, uint8_t len){
	transBuf[0] = reg;
	transBuf[1] = data;
	return i2c1.write(CAM_WRITE,&reg,2);
}
