/*
 * sensorFusion.h
 *
 *  Created on: Nov 8, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_FILTER_SENSORFUSION_H_
#define HARDWARE_FILTER_SENSORFUSION_H_

#include "../../Basic/basic.h"
#include "math.h"
#include "myMath.h"





class sensorFusion : public Thread{
public:
	sensorFusion();
	virtual ~sensorFusion();
	void init();
	void run();
	void newData(IMU_DATA_RAW data);
	/** TODO use CommBuffer insted of setter & resume function */



private:
	volatile float beta;				// algorithm gain
	volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	float integrationTime;
	struct IMU_RPY_RAW{
		float GYRO_YAW;
		float GYRO_ROLL;
		float GYRO_PITCH;
		float MAG_YAW;
		float ACCL_ROLL;
		float ACCL_PITCH;
	};


	//---------------------------------------------------------------------------------------------------
	// Function declarations

	void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);



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

	bool useMagn;
	float averageDrift;



};


#endif /* HARDWARE_FILTER_SENSORFUSION_H_ */
