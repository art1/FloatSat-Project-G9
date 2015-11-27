/*
 * lightSensor.h
 *
 *  Created on: Nov 27, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_LIGHTSENSOR_H_
#define HARDWARE_LIGHTSENSOR_H_

#include "../basic.h"

#define LIGHT_INTEGRATION_TIME		101		// values to choose: 402, 13, 101 -> scling then done automatically!
#define LIGHT_INTEGRATION_ADRESS	0x81
#define LIGHT_GAIN					0		// values to choose: 0 -> low gain, 1 -> high gain (16x)



#define DEVICE_ADRESS	0x29

#define TURN_ON			0x03
#define TURN_OFF		0x00

#define REG_CONTROL		0x00
#define REG_TIMING		0x01
#define REG_INT_CONTROL	0x06
#define REG_ID			0x0A
#define REG_DATA0_LOW	0x0C
#define REG_DATA0_HIGH	0x0D
#define REG_DATA1_LOW	0x0E
#define REG_DATA1_HIGH	0x0F

#define LUX_SCALE 		14 		// scale by 2^14
#define RATIO_SCALE 	9 		// scale ratio by 2^9
// Integration time scaling factors
#define CH_SCALE		10 		// scale channel values by 2^10
#define CHSCALE_TINT0	0x7517 	// 322/11 * 2^CH_SCALE
#define CHSCALE_TINT1	0x0fe7 	// 322/81 * 2^CH_SCALE

#define K1C				0x0043 // 0.130 * 2^RATIO_SCALE
#define B1C				0x0204 // 0.0315 * 2^LUX_SCALE
#define M1C				0x01ad // 0.0262 * 2^LUX_SCALE
#define K2C				0x0085 // 0.260 * 2^RATIO_SCALE
#define B2C				0x0228 // 0.0337 * 2^LUX_SCALE
#define M2C				0x02c1 // 0.0430 * 2^LUX_SCALE
#define K3C				0x00c8 // 0.390 * 2^RATIO_SCALE
#define B3C				0x0253 // 0.0363 * 2^LUX_SCALE
#define M3C				0x0363 // 0.0529 * 2^LUX_SCALE

#define K4C				0x010a // 0.520 * 2^RATIO_SCALE
#define B4C				0x0282 // 0.0392 * 2^LUX_SCALE
#define M4C				0x03df // 0.0605 * 2^LUX_SCALE
#define K5C				0x014d // 0.65 * 2^RATIO_SCALE
#define B5C				0x0177 // 0.0229 * 2^LUX_SCALE
#define M5C				0x01dd // 0.0291 * 2^LUX_SCALE
#define K6C				0x019a // 0.80 * 2^RATIO_SCALE
#define B6C				0x0101 // 0.0157 * 2^LUX_SCALE
#define M6C				0x0127 // 0.0180 * 2^LUX_SCALE
#define K7C				0x029a // 1.3 * 2^RATIO_SCALE
#define B7C				0x0037 // 0.00338 * 2^LUX_SCALE
#define M7C				0x002b // 0.00260 * 2^LUX_SCALE
#define K8C				0x029a // 1.3 * 2^RATIO_SCALE
#define B8C				0x0000 // 0.000 * 2^LUX_SCALE
#define M8C				0x0000 // 0.000 * 2^LUX_SCALE


class lightSensor : public Thread {
public:
	lightSensor();
	virtual ~lightSensor();
	void init();
	void run();
	void setActive(LUX_DATA val);
private:
	float lux;
	bool isActive();
	bool activated;
	uint8_t recBuf[512];
	uint8_t transBuf[512];
	uint32_t calculateLux();
	void readRawData();
	uint8_t highByte;
	uint8_t lowByte;
	uint16_t ch0;
	uint16_t ch1;
	LUX_DATA pub_data;
};

#endif /* HARDWARE_LIGHTSENSOR_H_ */
