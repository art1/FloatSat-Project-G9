/*
 * i2cconnector.h
 *
 *  Created on: Oct 26, 2015
 *      Author: arthur
 */

#ifndef I2CCONNECTOR_H_
#define I2CCONNECTOR_H_
#include "../basic.h"

class i2c_connector {
public:
	i2c_connector();
	virtual ~i2c_connector();
	int init(int channel);
	int write(uint8_t adress,uint8_t *txBuf,int bytesToSend);
	int read(uint8_t adress,uint8_t *txBuf,int bytesToSend,uint8_t *recBuf,int bytesToRead);

private:
	HAL_I2C *i2c;
	uint8_t recBuf[512];
	uint8_t transBuf[512];
};

#endif /* I2CCONNECTOR_H_ */
