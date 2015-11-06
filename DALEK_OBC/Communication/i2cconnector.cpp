/*
 * i2cconnector.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: arthur
 */

#include "i2cconnector.h"

HAL_I2C i2c1(I2C_IDX1);
HAL_I2C i2c2(I2C_IDX2);
HAL_I2C i2c3(I2C_IDX3);

i2c_connector::i2c_connector() {
	// TODO Auto-generated constructor stub

}

i2c_connector::~i2c_connector() {
	// TODO Auto-generated destructor stub
}

int i2c_connector::init(int channel){
	switch(channel){
	case 0:
		this->i2c = &i2c1;
		break;
	case 1:
		this->i2c = &i2c2;
		break;
	case 2:
		this->i2c = &i2c3;
		break;
	default:

		break;
	}
	this->i2c->init(400000);
	memset(recBuf,0,sizeof(recBuf));
	memset(transBuf,0,sizeof(transBuf));
}


int i2c_connector::read(uint8_t adress,uint8_t *txBuf,int bytesToSend,uint8_t *recBuf,int bytesToRead){
	return i2c->writeRead(adress,txBuf,bytesToSend,recBuf,bytesToRead);
//	uint8_t temp[bytesToRead];
//	for(int i=0;i<bytesToRead;i++){
//		PRINTF("read byte i2c: %d at %d\n",recBuf[i],adress);
//		temp[i] = recBuf[i];
//	}

}

int i2c_connector::write(uint8_t adress,uint8_t *txBuf,int bytesToSend){
	return i2c->write(adress,txBuf,bytesToSend);
}
