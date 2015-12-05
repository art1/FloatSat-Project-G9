/*
 * sensorFusion.h
 *
 *  Created on: Nov 8, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_FILTER_SENSORFUSION_H_
#define HARDWARE_FILTER_SENSORFUSION_H_

#include "../../../basic.h"
#include "myMath.h"
//#include "math.h"





struct IMU_RPY_FILTERED{
	float YAW;
	float PITCH;
	float ROLL;
};



class sensorFusion : public Thread{
public:
	sensorFusion();
	virtual ~sensorFusion();
	void init();
	void run();
	void newData(IMU_DATA_RAW data);



private:
	struct IMU_RPY_RAW{
		float GYRO_YAW;
		float GYRO_ROLL;
		float GYRO_PITCH;
		float MAG_YAW;
		float ACCL_ROLL;
		float ACCL_PITCH;
	};

	IMU_DATA_RAW raw;
	IMU_RPY_RAW angleRPY;
	IMU_RPY_FILTERED filtered;
	Vector3D gyro;
	Vector3D accl;
	Vector3D magn;
	Quaternion quat;
	float heading, pitch, bank;
	void dataFusion(Vector3D *gyro, Vector3D *accl, Vector3D *magn);
	void convertToHPB(Quaternion q);
	void rawToRPY();												// used only when complementary filter is active!
	float propGain;
	double integrationDelta;
	double samplerateTime;
	double oldSamplerateTime;
	float sampleDiff;
	float cosFactor;
	float deltaYaw;
	float deltaPitch;
	float deltaRoll;



};


#endif /* HARDWARE_FILTER_SENSORFUSION_H_ */
