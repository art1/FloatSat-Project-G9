/*
 * basic.cpp
 *
 *  Created on: Nov 7, 2015
 *      Author: arthur
 */

#include "basic.h"

//declaring topics for RODOS middleware inter-Thread Communication
Topic<IMU_DATA_RAW> imu_rawData(1,"IMU Raw Data");
Topic<LED_SWITCH>	led_switch(2,"LED Switch Control");



