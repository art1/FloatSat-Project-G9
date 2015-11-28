/*
 * SCCB.h
 *
 *  Created on: Nov 28, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_CAMERA_SUPPS_SCCB_H_
#define HARDWARE_CAMERA_SUPPS_SCCB_H_

#include "../../../basic.h"



#define CAM_ADDR		0x21
#define CAM_READ		0x42
#define CAM_WRITE		0x43


class SCCB {
public:
	SCCB();
	virtual ~SCCB();
	void init();
	int camera_reg_write(uint8_t reg, uint8_t data, uint8_t len);
	int camera_reg_read(uint8_t reg, uint8_t *dest, uint8_t len);
private:
	int retVal;
	uint8_t recBuf[512];
	uint8_t transBuf[512];

};

#endif /* HARDWARE_CAMERA_SUPPS_SCCB_H_ */
