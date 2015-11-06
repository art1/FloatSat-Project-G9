/*
 * spiconnector.h
 *
 *  Created on: Oct 26, 2015
 *      Author: arthur
 */

#ifndef SPICONNECTOR_H_
#define SPICONNECTOR_H_

#include "../basic.h"
#include "hal.h"
//#include "unistd.h"
//#include <vector>
//#include <list>



class spi_connector {
public:
	//TODO get that STL list or vector working!
	spi_connector();
	int init(int spi_channel,HAL_GPIO cs1,HAL_GPIO cs2, int baud);		// constructor and init spi
	virtual ~spi_connector();
	int setBaudrate(int baud);
	int getBaudrate();
	int spi_write(HAL_GPIO cs,uint8_t regAdress,uint8_t bytesToWrite, int numOfBytesToWrite);
	uint8_t spi_read(HAL_GPIO cs,uint8_t regAdress,int bytesToRead);
	uint8_t spi_writeAndRead(HAL_GPIO cs,uint8_t regAdress,uint8_t bytesToWrite, int numOfBytesToWrite, int numOfBytesToRead);
	uint8_t spi_writeThenRead(HAL_GPIO cs,uint8_t regAdress,uint8_t bytesToWrite, int numOfBytesToWrite, int numOfBytesToRead);
private:
	int retCode;
	int baud;
	HAL_SPI* spi;
	//	HAL_GPIO *cs_pin; //-> see hal_gpio.h for PIN Assignment -> that's GPIO_018 for PB2 for IMU
	uint8_t readBuf[512];
	uint8_t writeBuf[512];
};

#endif /* SPICONNECTOR_H_ */
