/*
 * sensorFusion.cpp
 *
 *  Created on: Nov 8, 2015
 *      Author: arthur
 *      Purpose of sensorFusion class is to fuse Accl, Gyro and Magneotmeter by a kalman-based Quaternion-Madgwick-filter -> advantage: not as computation heavy as kalman!
 *      see http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
 */

#include "sensorFusion.h"

#define GAIN	0.4f

//static Application senderName("RPY Publisher",501);




sensorFusion::sensorFusion() : Thread("sensorFusion Thread",101){

	beta = GAIN;
	integrationDelta = 20000.0f;
	sampleDiff = 0.0f;
	quat.q0 = 1.0f;
	quat.q.x = 0.0f;
	quat.q.y = 0.0f;
	quat.q.z = 0.0f;
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	oldSamplerateTime = 0.0f;
	accl.x = 0;
	accl.y = 0;
	accl.z = 0;
	gyro.x = 0;
	gyro.y = 0;
	gyro.z = 0;
	magn.x = 0;
	magn.y = 0;
	magn.z = 0;
	samplerateTime = 0.0f;
	cosFactor = 0.0f;
	deltaYaw = 0.0f;
	deltaPitch = 0.0f;
	deltaRoll = 0.0f;

	angleRPY.ACCL_PITCH = 0.0f;
	angleRPY.ACCL_ROLL = 0.0f;
	angleRPY.MAG_YAW = 0.0f;
	angleRPY.GYRO_PITCH = 0.0f;
	angleRPY.GYRO_ROLL = 0.0f;
	angleRPY.GYRO_YAW = 0.0f;

	useMagn = true;

	averageDrift = 0.0f;  // use for drift correction when not using magnetometer...
}

sensorFusion::~sensorFusion() {

}


void sensorFusion::init(){

}

void sensorFusion::run(){
	int printValues = IMU_PRINT_VALUES/IMU_SAMPLERATE;
	int cnt = 0;
	//	PRINTF("Fusion An!\n");
	while(1){

		suspendCallerUntil(END_OF_TIME);
		//		PRINTF("\nFUSIOOOOOOON\n\n");

		integrationTime = samplerateTime - oldSamplerateTime;

		oldSamplerateTime = samplerateTime;
#ifdef MADGWICK_TWO
		if(!useMagn){
			magn.x = 0.0f;
			magn.y = 0.0f;
			magn.z = 0.0f;
		}
		MadgwickAHRSupdate(gyro.x,gyro.y,gyro.z,accl.x,accl.y,accl.z,magn.x,magn.y,magn.z);
#else
		dataFusion(&gyro,&accl,&magn);
#endif
		float head;
		if((filtered.YAW) < 0) head = 360.0 + filtered.YAW;
		else head = filtered.YAW;
		filtered.YAW = head;
		imu_filtered.publish(filtered);
//		PRINTF("%f\n",head);
		if(cnt>printValues){
			//			PRINTF("QUAT: %f  %f  %f  %f\n",quat.q0,quat.q.x,quat.q.y,quat.q.z);
			//			PRINTF("roll:   %f   pitch:   %f   yaw:   %f\n",bank*TO_DEG,pitch*TO_DEG,heading*TO_DEG);
//			PRINTF("filtered: ROLL    %f    PITCH    %f    YAW     %f\n",filtered.ROLL*TO_DEG,filtered.PITCH*TO_DEG,filtered.YAW*TO_DEG);
//			PRINTF("filtered: ROLL    %f    PITCH    %f    YAW     %f\n",filtered.ROLL,filtered.PITCH,filtered.YAW);
//			PRINTF("heading: %f\n",head);
			//			PRINTF("\nYAW:   %f   PITCH:   %f   ROLL:   %f   \n",angleRPY.MAG_YAW*TO_DEG,angleRPY.ACCL_PITCH*TO_DEG,angleRPY.ACCL_ROLL*TO_DEG);
			//			PRINTF("\nYAW:   %f   PITCH:   %f   ROLL:   %f   \n",angleRPY.MAG_YAW,angleRPY.ACCL_PITCH,angleRPY.ACCL_ROLL);
			cnt = 0;
			BLUE_TOGGLE;
		}
		cnt++;
	}
}


void sensorFusion::setNewData(VAR_CONTROL *_varC){
	if(_varC->changedVal == SET_BETA_GAIN){
		this->beta = _varC->value;
	}
}


void sensorFusion::newData(IMU_DATA_RAW data){
	//	PRINTF("new data for filtering!\n");
	accl.x = data.ACCEL_RAW_X;
	accl.y = data.ACCEL_RAW_Y;
	accl.z = data.ACCEL_RAW_Z;
	gyro.x = data.ANGULAR_RAW_X;
	gyro.y = data.ANGULAR_RAW_Y;
	gyro.z = data.ANGULAR_RAW_Z;
	magn.x = data.MAGNETIC_RAW_X;
	magn.y = data.MAGNETIC_RAW_Y;
	magn.z = data.MAGNETIC_RAW_Z;
	samplerateTime = data.currentSampleTime;
}
#ifdef MADGWICK
void sensorFusion::dataFusion(Vector3D *gyro, Vector3D *accl, Vector3D *magn){


	// normalize acc/mag
	normalize3DVector(accl);
	normalize3DVector(magn);

	float magRefX, magRefY;
	float q0 = quat.q0;
	float q1 = quat.q.x;
	float q2 = quat.q.y;
	float q3 = quat.q.z;




	/** Deprecated - not using Mahony anymore */
	//	magRefX = (magn->x * 2.0f *(0.5f - q2*q2 - q3*q3)) +  (2.0f * magn->y * (q1*q2 - q0*q3))  +  (2.0f * magn->z * (q1*q3 + q0*q2));
	//	magRefY = (2.0f * magn->x * (q1*q2 + q0*q3)) + (2.0f * magn->y * (0.5f - q1*q1 - q3 * q3)) + (2.0f * magn->z * (q2*q3 - q0*q1));
	//	// magRefZ not needed..
	//	float tempNormBX = sqrt(magRefX*magRefX + magRefY*magRefY);
	//	float tempNormBZ = 2.0f * magn->x * (q1*q3 - q0*q2) + 2.0f*magn->y*(q2*q3 - q0*q1) + 2.0f * magn->z * (0.5f - q1*q1 - q2*q2);
	//
	//	//estimation gravity and magnetic field
	//	Vector3D accl_est, magn_est;
	//	accl_est.x = 2.0f *(q1*q3 - q0*q2);
	//	accl_est.y = 2.0f *(q0*q1 + q2*q3);
	//	accl_est.z = q0*q0  - q1*q1 - q2*q2 + q3*q3;
	//	magn_est.x = 2.0f*tempNormBX * (0.5f - q2*q2 - q3*q3) + 2.0f*tempNormBZ*(q1*q3 - q0*q2);
	//	magn_est.y = 2.0f*tempNormBX * (q1*q2 - q0*q3) + 2.0f * tempNormBZ * (q0*q1 + q2*q3);
	//	magn_est.z = 2.0f*tempNormBX * (q0*q2 + q1*q3) + 2.0f * tempNormBZ * (0.5f - q1*q1 - q2*q2);
	//	// now calculate error from estimated direction and measured direction of gravity /Magn field
	//	Vector3D tempA,tempM,err;
	//	cross_product(accl,&accl_est,&tempA);
	//	cross_product(magn,&magn_est,&tempM);
	//	add3DVector(&tempA,&tempM,&err);

	/** Madgwick */



	// reference to mag field earth
	magRefX = magn->x * q0*q0 - (2.0f*q0*magn->y)* q3 + (2.0f * q0 * magn->z) * q2  + magn->x *q1*q1 + (2.0f*q1)*magn->y*q2 + 2.0f*q1*magn->z*q3 - magn->x*q2*q2 - magn->x*q3*q3;
	magRefY = 2.0f*q0*magn->x*q3 + magn->y*q0*q0 - 2.0f*q0*magn->z*q1 + 2.0f*q1*magn->x*q2 - magn->y*q1*q1 + magn->y*q2*q2 + 2.0f*q2*magn->z*q3 - magn->y*q3*q3;
	float tempNormBX = sqrt(magRefX*magRefX + magRefY*magRefY);
	float tempNormBZ = -2.0f*q0*magn->x*q2 + 2.0f*q0*magn->y*q1 + magn->z*q0*q0 + 2.0f*q1*magn->x*q3 - magn->z*q1*q1 + 2.0f*q1*magn->y*q3 - magn->z*q2*q2 + magn->z*q3*q3;
	// GDA corrective Step
	/** TODO reduce calculation amount by outsourcing "constant" factors */
	Vector4D step;
	step.x = -(2.0f*q2) * (2.0f * q1*q3 - (2.0f * q0 * q2) - accl->x)
															+ (2.0f*q1) * (2.0f * q0*q1 + (2.0f * q2*q3) - accl->y)
															- tempNormBZ * q2 * (tempNormBX * (0.5f - (q2*q2) - (q3*q3))
																	+ tempNormBZ * (q1*q3 - (q0*q2)) - magn->x)
																	+ (-tempNormBX * q3 + tempNormBZ * q1) * (tempNormBX * ((q1*q2) - (q0*q3))
																			+ tempNormBZ * (q0*q1 + (q2*q3)) - magn->y)
																			+ tempNormBX * q2 * (tempNormBX * ((q0*q2) + q1*q3)
																					+ tempNormBZ * (0.5f - (q1*q1) - (q2*q2)) - magn->z);
	step.y = (2.0f * q3) * (2.0f * q1*q3 - (2.0f * q0 * q2) - accl->x)
															+ (2.0f * q0) * (2.0f * q0*q1 + (2.0f * q2*q3) - accl->y)
															- 4.0f * q1 * (1 - 2.0f * (q1*q1) - 2.0f * (q2*q2) - accl->z)
															+ tempNormBZ * q3 * (tempNormBX * (0.5f - (q2*q2) - (q3*q3))
																	+ tempNormBZ * (q1*q3 - (q0*q2)) - magn->x)
																	+ (tempNormBX * q2 + tempNormBZ * q0) * (tempNormBX * ((q1*q2) - (q0*q3))
																			+ tempNormBZ * (q0*q1 + (q2*q3)) - magn->y)
																			+ (tempNormBX * q3 - (2.0f * tempNormBZ) * q1) * (tempNormBX * ((q0*q2) + q1*q3)
																					+ tempNormBZ * (0.5f - (q1*q1) - (q2*q2)) - magn->z);
	step.z = -(2.0f * q0) * (2.0f * q1*q3 - (2.0f * q0 * q2) - accl->x)
															+ (2.0f * q3) * (2.0f * q0*q1 + (2.0f * q2*q3) - accl->y)
															- 4.0f * q2 * (1 - 2.0f * (q1*q1) - 2.0f * (q2*q2) - accl->z)
															+ (-(2.0f * tempNormBX) * q2 - tempNormBZ * q0) * (tempNormBX * (0.5f - (q2*q2) - (q3*q3))
																	+ tempNormBZ * (q1*q3 - (q0*q2)) - magn->x)
																	+ (tempNormBX * q1 + tempNormBZ * q3) * (tempNormBX * ((q1*q2) - (q0*q3))
																			+ tempNormBZ * (q0*q1 + (q2*q3)) - magn->y)
																			+ (tempNormBX * q0 - (2.0f * tempNormBZ) * q2) * (tempNormBX * ((q0*q2) + q1*q3)
																					+ tempNormBZ * (0.5f - (q1*q1) - (q2*q2)) - magn->z);
	step.scale = (2.0f*q1) * (2.0f * q1*q3 - (2.0f * q0 * q2) - accl->x)
															+ (2.0f*q2) * (2.0f * q0*q1 + (2.0f * q2*q3) - accl->y)
															+ (-(2.0f * tempNormBX) * q3 + tempNormBZ * q1) * (tempNormBX * (0.5f - (q2*q2) - (q3*q3))
																	+ tempNormBZ * (q1*q3 - (q0*q2)) - magn->x)
																	+ (-tempNormBX * q0 + tempNormBZ * q2) * (tempNormBX * ((q1*q2)
																			- (q0*q3)) + tempNormBZ * (q0*q1 + (q2*q3)) - magn->y)
																			+ tempNormBX * q1 * (tempNormBX * ((q0*q2) + q1*q3)
																					+ tempNormBZ * (0.5f - (q1*q1) - (q2*q2)) - magn->z);
	normalize4DVector(&step);

	// gyro change rate
	Quaternion gyroC;
	gyroC.q0 = 0.5f * (-q1 * gyro->x - q2*gyro->y - q3*gyro->z) - propGain*step.x;
	gyroC.q.x = 0.5f * (q0*gyro->x + q2*gyro->z - q3*gyro->y) - propGain*step.y;
	gyroC.q.y = 0.5f * (q0*gyro->y - q1*gyro->z + q3*gyro->x) - propGain*step.z;
	gyroC.q.z = 0.5f * (q0*gyro->z + q1*gyro->y - q2*gyro->x) - propGain*step.scale; /** TODO Check that value */

	// now apply feedback
	q0 += gyroC.q0 * integrationDelta;
	q1 += gyroC.q.x*integrationDelta;
	q2 += gyroC.q.y*integrationDelta;
	q3 += gyroC.q.z*integrationDelta;
	Vector4D t = Vector4D(q0,q1,q2,q3);
	normalize4DVector(&t);
	quat.q0 = t.x;
	quat.q.x = t.y;
	quat.q.y = t.z;
	quat.q.z = t.scale;
	//	PRINTF("quat: %f %f %f %f",q0,q1,q2,q3);
	//	PRINTF("QUAT: %f  %f  %f  %f\n",quat.q0,quat.q.x,quat.q.y,quat.q.z);
	convertToHPB(quat);

}
#endif
#ifdef COMPLEMENTARY
void sensorFusion::dataFusion(Vector3D *gyro, Vector3D *accl, Vector3D *magn){
	rawToRPY();
	// converts everything first to RPH angles, heading from accl!
	// now do some complementary fusion stuff
	//	filtered.PITCH = COMPL_GAIN*(filtered.PITCH + gyro->x*sampleDiff) + (1.0-COMPL_GAIN)*(angleRPY.ACCL_PITCH);
	//	filtered.ROLL = COMPL_GAIN*(filtered.ROLL + gyro->y*sampleDiff) + (1.0-COMPL_GAIN)*(angleRPY.ACCL_ROLL);
	//	filtered.YAW = COMPL_GAIN*(filtered.YAW + gyro->z*sampleDiff) + (1.0-COMPL_GAIN)*(angleRPY.MAG_YAW);
	filtered.PITCH = COMPL_GAIN*(angleRPY.GYRO_PITCH) + (1.0f-COMPL_GAIN)*(angleRPY.ACCL_PITCH);
	filtered.ROLL = COMPL_GAIN*(angleRPY.GYRO_ROLL) + (1.0f-COMPL_GAIN)*(angleRPY.ACCL_ROLL);
	filtered.YAW = COMPL_GAIN*(angleRPY.GYRO_YAW) + (1.0f-COMPL_GAIN)*(angleRPY.MAG_YAW);

	//	PRINTF("gyro pitch %f \n",angleRPY.GYRO_PITCH);


}
#endif
#ifdef MADGWICK_TWO
void sensorFusion::MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx <= EPSILON_COMPARISON) && (my <= EPSILON_COMPARISON) && (mz <=EPSILON_COMPARISON)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = fastInverseSQRT(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = fastInverseSQRT(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
				+ _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
				- 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
				+ _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
				+ (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
				+ (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = fastInverseSQRT(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (integrationTime);
	q1 += qDot2 * (integrationTime);
	q2 += qDot3 * (integrationTime);
	q3 += qDot4 * (integrationTime);

	// Normalise quaternion
	recipNorm = fastInverseSQRT(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	quat.q0 = q0;
	quat.q.x = q1;
	quat.q.y = q2;
	quat.q.z = q3;
	convertToHPB(quat);
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void sensorFusion::MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = fastInverseSQRT(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = fastInverseSQRT(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (integrationTime);
	q1 += qDot2 * (integrationTime);
	q2 += qDot3 * (integrationTime);
	q3 += qDot4 * (integrationTime);

	// Normalise quaternion
	recipNorm = fastInverseSQRT(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	quat.q0 = q0;
	quat.q.x = q1;
	quat.q.y = q2;
	quat.q.z = q3;
	convertToHPB(quat);
}
#endif

void sensorFusion::rawToRPY(){
	cosFactor = 1/(cosf(angleRPY.GYRO_PITCH));
	deltaPitch = cosFactor * ((cosf(angleRPY.GYRO_ROLL) * cosf(angleRPY.GYRO_PITCH)*gyro.y) - (sinf(angleRPY.GYRO_ROLL)*cosf(angleRPY.GYRO_PITCH)*(gyro.z )));
	deltaRoll = cosFactor * ((cosf(angleRPY.GYRO_PITCH) * (gyro.x)) + (sinf(angleRPY.GYRO_ROLL)*sinf(angleRPY.GYRO_PITCH)*(gyro.y)) + (cosf(angleRPY.GYRO_ROLL)*sin(angleRPY.GYRO_PITCH)*(gyro.z )));
	deltaYaw = cosFactor * ((sinf(angleRPY.GYRO_ROLL) * (gyro.y )) + (cosf(angleRPY.GYRO_ROLL)*(gyro.z)));

	if(abs(oldSamplerateTime) < EPSILON_COMPARISON){
		sampleDiff = 1.0f;
	} else {
		sampleDiff = (float)(samplerateTime - oldSamplerateTime);

	}

	angleRPY.GYRO_YAW += (deltaYaw*sampleDiff);
	angleRPY.GYRO_PITCH += (deltaPitch*sampleDiff);
	angleRPY.GYRO_ROLL += (deltaRoll*sampleDiff);
	//	PRINTF("\ndifference sample:   %f   \n",sampleDiff);

	oldSamplerateTime = samplerateTime;
	//	PRINTF("\nYAW:  %f  PITCH:   %f   ROLL:   %f   ",angleRPY.GYRO_YAW*TO_DEG,angleRPY.GYRO_PITCH*TO_DEG,angleRPY.GYRO_ROLL*TO_DEG);
	//	PRINTF("\ndeltaYaw:   %f   deltaPitch:   %f   deltaRoll:   %f   \n",deltaYaw,deltaPitch,deltaRoll);

	// acclererometer convert to RPY -> yaw angle can not be get by accl, so use magnetometer
	//	angleRPY.MAG_YAW = atan(newData.ACCEL_RAW_Z/(sqrt((newData.ACCEL_RAW_X*newData.ACCEL_RAW_X) + (newData.ACCEL_RAW_Z*newData.ACCEL_RAW_Z))));
	angleRPY.ACCL_PITCH = atan(accl.x/(sqrt((accl.y*accl.y) + (accl.z*accl.z))));
	angleRPY.ACCL_ROLL = atan(accl.y*fastInverseSQRT((float)((accl.x*accl.x) + (accl.z*accl.z))));
	// use accl pitch and roll for tilt compensation
	angleRPY.MAG_YAW = atan(((magn.x*sin(angleRPY.ACCL_ROLL)*sin(angleRPY.ACCL_PITCH))+(magn.y*cos(angleRPY.ACCL_ROLL))-(magn.z*sin(angleRPY.ACCL_ROLL)*cos(angleRPY.ACCL_PITCH)))
			/((magn.x*cos(angleRPY.ACCL_PITCH)) + (magn.z*sin(angleRPY.ACCL_PITCH))));
//	PRINTF("comp1: %f  ",angleRPY.MAG_YAW);

	// normalize accl measurements
	normalize3DVector(&accl);
	Vector3D tmp = (Vector3D){0,-1,0};
	Vector3D tmp2;
	Vector3D tmp3;
	cross_product(&magn,&accl,&tmp2);
	normalize3DVector(&tmp2);
	cross_product(&accl,&tmp2,&tmp3);
	angleRPY.MAG_YAW = atan2((scalar_product3D(&tmp2,&tmp)),(scalar_product3D(&tmp3,&tmp)));
	if(angleRPY.MAG_YAW < 0.0) angleRPY.MAG_YAW += M_PI;
//	PRINTF(" %f  ",angleRPY.MAG_YAW);



	// convert everything to DEGREE

	//	angleRPY.ACCL_PITCH *= TO_DEG;
	//	angleRPY.ACCL_ROLL *= TO_DEG;
	//	angleRPY.MAG_YAW *= TO_DEG;
	//	angleRPY.GYRO_PITCH *= TO_DEG;
	//	angleRPY.GYRO_ROLL *= TO_DEG;
	//	angleRPY.GYRO_YAW *= TO_DEG;


}

// convet to Heading/Pitch/Bank ->
// (bank) roll -> rotate about x-axis
// pitch -> rotate about y-axis
// (heading) yaw -> rotate about z-axis
// -> see here for gimbal situation and how to handle it : http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
void sensorFusion::convertToHPB(Quaternion q){

	//	heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)
	//	attitude = asin(2*qx*qy + 2*qz*qw)
	//	bank = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx2 - 2*qz2)

	// heading first
//	double q0, q1, q2, q3;
//	q0 = q.q0;
//	q1 = q.q.x;
//	q2 = q.q.y;
//	q3 = q.q.y;
	//	float heading, pitch, bank;
	//	heading = atan2f(2.0f*q1*q3 - 2.0f*q0*q2,1.0f - 2.0f*(q1*q1) - 2.0f*(q2*q2));
	//	/** TODO Gimabl lock situation */
	//	//check for gimbal lock situation
	//	//	if(((fabs((q0*q1 + q2*q3))-0.5) < EPSILON_COMPARISON) && (q0*q1 + q2*q3) > 0.0f){
	//	//
	//	//	}
	//	pitch = asin(2.0f*q0*q1 + 2.0f*q2*q3);
	//	bank = atan2f(2.0f*q0*q3 - 2.0f*q1*q2,1.0f -2.0f*(q0*q0) - 2.0f*(q2*q2));
	//	PRINTF("roll:   %f   pitch:   %f   yaw:   %f\n",bank,pitch,heading);

	float temp = 2*q0*q0 - 1;	// Avoid recalculating this
	bank = atan2f(2.0f * (q0*q1 + q2*q3), temp + 2*q3*q3);	// roll
	pitch = asinf( 2.0f * (q0*q2 - q1*q3));					// pitch
	heading = atan2f(2.0f * (q1*q2 + q0*q3), temp + 2*q1*q1);	// yaw
	filtered.PITCH = pitch*TO_DEG;
	filtered.ROLL = bank*TO_DEG;
	filtered.YAW = heading*TO_DEG;

}
