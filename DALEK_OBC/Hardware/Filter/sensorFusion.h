/*
 * sensorFusion.h
 *
 *  Created on: Nov 8, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_FILTER_SENSORFUSION_H_
#define HARDWARE_FILTER_SENSORFUSION_H_

#include "../../basic.h"
#include "myMath.h"


class sensorFusion : public Thread{
public:
	sensorFusion();
	virtual ~sensorFusion();
	void init();
	void run();
	void newData(IMU_DATA_RAW data);

private:
	IMU_DATA_RAW raw;
	Vector3D gyro;
	Vector3D accl;
	Vector3D magn;
	Quaternion quat;
	float heading, pitch, bank;
	void dataFusion(Vector3D *gyro, Vector3D *accl, Vector3D *magn);
	void convertToHPB(Quaternion q);
	float propGain;
	double integrationDelta;

};

#endif /* HARDWARE_FILTER_SENSORFUSION_H_ */
