/*
 * basic.h
 *
 *  Created on: Nov 4, 2015
 *      Author: arthur
 */

#ifndef BASIC_H_
#define BASIC_H_

#include "rodos.h"

#define IMU_RESET_PIN			GPIO_055
#define IMU_G_CS_PIN			GPIO_018
#define IMU_XM_CS_PIN			GPIO_032

#define IMU_GYRO_RANGE			2000			// in DPS, select 245, 500 or 2000 sensitivity is set according to chosen value here
#define IMU_ACCL_RANGE			2				// value in g, select 2,4,6,8 or 16; sensitivity is set according to chosen value
#define IMU_MAGN_RANGE			2				//value in gauss, select 2,4,8 or 13; sensitivity is set according to chosen value

#define IMU_GYRO_DEFAULT_OFFSET	1
#define IMU_ACCL_DEFAULT_OFFSET	1
#define IMU_MAGN_DEFAULT_OFFSET	1

#define CALIBRAION_SAMPLES		1000.0f			// calibration samples for gyro




struct IMU_DATA_RAW{
	float ANGULAR_RAW_X;
	float ANGULAR_RAW_Y;
	float ANGULAR_RAW_Z;

	float MAGNETIC_RAW_X;
	float MAGNETIC_RAW_Y;
	float MAGNETIC_RAW_Z;

	float ACCEL_RAW_X;
	float ACCEL_RAW_Y;
	float ACCEL_RAW_Z;

	float TEMP_RAW;
};

struct LED_SWITCH{
	int COMMAND;
	int GREEN;
	int RED;
	int ORANGE;
	int BLUE;
};
#endif /* BASIC_H_ */

// now define the topics stuff for the RODOS middleware
extern Topic<IMU_DATA_RAW>	imu_rawData;
extern Topic<LED_SWITCH>	led_switch;
