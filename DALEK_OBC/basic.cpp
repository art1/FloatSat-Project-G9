/*
 * basic.cpp
 *
 *  Created on: Nov 7, 2015
 *      Author: arthur
 */

#include "basic.h"

HAL_I2C i2c1(I2C_IDX1);
HAL_I2C i2c2(I2C_IDX2);

//declaring topics for RODOS middleware inter-Thread Communication
Topic<IMU_DATA_RAW> imu_rawData(1,"IMU Raw Data");
Topic<LUX_DATA> lux_data(2,"LUX Data");


