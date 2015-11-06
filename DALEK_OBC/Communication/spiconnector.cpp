/*
 * spiconnector.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: arthur
 */

#include "spiconnector.h"

HAL_SPI spi1(SPI_IDX1);
HAL_SPI spi2(SPI_IDX2);
HAL_SPI spi3(SPI_IDX3);



//TODO: correct Warning: not initialized in constructor!
/**
 * needs the appropriate channel and a vector of GPIO Pins you intend to use as ChipSelect Pins, and the baudrate
 */
spi_connector::spi_connector(){

}

int spi_connector::init(int spi_channel, HAL_GPIO cs1,HAL_GPIO cs2,int baud) {
	PRINTF("SPI Connector constructor called \n\r");
	switch(spi_channel){
	case 1:
		this->spi = &spi1;
		break;
	case 2:
		this->spi = &spi2;
		break;
	case 3:
		this->spi = &spi3;
		break;
	default:
		break;
	}
//	*cs_pin = HAL_GPIO(cs);
	retCode = 0;

	// init SPI Channel with Chipselects as ActiveLow Config
//	for(std::list<GPIO_PIN>::const_iterator it = cs->begin(), end = cs->end(); it!= end; ++it){
//		HAL_GPIO(*it).init(true,1,1);
//	}
//	for(int i=0;i<cs->size();i++){
//		HAL_GPIO(cs->)
//		HAL_GPIO(cs->at(i)).init(true,1,1);
//	}


	cs1.init(true,1,1);
	cs2.init(true,1,1);
	this->baud = baud;
	this->spi->init(baud);
//	this->spi->config(SPI_PARAMETER_MODE,3);

}

spi_connector::~spi_connector() {

}

/**
 * used to set baudrate after initialization
 * 0 = success
 * -1 = failure
 */
int spi_connector::setBaudrate(int baud){
	this->baud = baud;
	return spi->config(SPI_PARAMETER_BAUDRATE,baud);
}
/**
 * return the current baudrate
 */
int spi_connector::getBaudrate(){
	return this->baud;
}

/**
 * return the number of bytes which had been read
 * or -255 if numOfBytesToRead <= 0
 * or -254 if didn't all required bytes
 * or errorcode if reading went wrong
 * or if positive: number of received bytes
 */
uint8_t spi_connector::spi_read(HAL_GPIO cs,uint8_t regAdress,int bytesToRead){
	if(bytesToRead<=0) return -1;

	cs.setPins(0);
	uint8_t *readBufTemp;

	retCode = spi->writeRead(&regAdress,1,readBufTemp,bytesToRead);

//	retCode = spi->write(&regAdress,1);
	PRINTF("writing return: %d\n",retCode);
//	retCode = spi->read(readBufTemp,bytesToRead);
//	PRINTF("read return: %d\n",retCode);
	cs.setPins(1);

	if(retCode<0) return retCode;
	if(retCode != bytesToRead) return -2;
	uint8_t ret[bytesToRead];

	//TODO: better without copying that shit to another array
	for(int i=0;i<bytesToRead;i++){
		PRINTF("read byte: %d",readBufTemp[i]);
		ret[i] = readBuf[i];
	}
	return *ret;
}

/**
 * return
 * 		-255	mismatch of bytesToWrite and given Bytes in Array
 * 		<0		error while transmitting
 * 		>0		sentBytes
 */
int spi_connector::spi_write(HAL_GPIO cs,uint8_t regAdress,uint8_t bytesToWrite,int numOfBytesToWrite){
	if(sizeof(bytesToWrite) != numOfBytesToWrite) return -255;
	cs.setPins(0);
	retCode = spi->write(&regAdress,1);
	retCode |= spi->write(&bytesToWrite,numOfBytesToWrite);
	cs.setPins(1);
	return retCode;
}


uint8_t spi_connector::spi_writeAndRead(HAL_GPIO cs, uint8_t regAdress, uint8_t bytesToWrite, int numOfBytesToWrite, int numOfBytesToRead){
	if(numOfBytesToRead < 0 || numOfBytesToWrite < 0) return -1;
	cs.setPins(0);
	retCode = spi->write(&regAdress,1);
	retCode = spi->writeRead(&bytesToWrite,numOfBytesToWrite,readBuf,numOfBytesToRead);
	cs.setPins(1);

	if(retCode<0) return retCode;
	if(retCode != numOfBytesToRead) return -2;
	uint8_t ret[numOfBytesToRead];

	//TODO: better without copying that shit to another array
	for(int i=0;i<numOfBytesToRead;i++){
		ret[i] = writeBuf[i];
	}
	return *ret;
}

uint8_t spi_connector::spi_writeThenRead(HAL_GPIO cs,uint8_t regAdress,uint8_t bytesToWrite, int numOfBytesToWrite, int numOfBytesToRead){
	cs.setPins(0);
	retCode = spi->write(&regAdress,1);
	retCode |= spi->write(&bytesToWrite,numOfBytesToWrite);
	retCode |= spi->read(readBuf,numOfBytesToRead);
	if(retCode < 0) return retCode;
	cs.setPins(1);

	if(retCode != numOfBytesToRead) return -2;
	uint8_t ret[numOfBytesToRead];

	//TODO: better without copying that shit to another array
	for(int i=0;i<numOfBytesToRead;i++){
		ret[i] = writeBuf[i];
	}
	return *ret;
}

