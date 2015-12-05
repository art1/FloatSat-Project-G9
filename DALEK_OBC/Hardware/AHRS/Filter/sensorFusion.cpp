/*
 * sensorFusion.cpp
 *
 *  Created on: Nov 8, 2015
 *      Author: arthur
 *      Purpose of sensorFusion class is to fuse Accl, Gyro and Magneotmeter by a kalman-based Quaternion-Madgwick-filter -> advantage: not as computation heavy as kalman!
 *      see http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
 */

#include "sensorFusion.h"



//static Application senderName("RPY Publisher",501);




sensorFusion::sensorFusion(){
	// TODO Auto-generated constructor stub
	integrationDelta = 20000.0f;
	sampleDiff = 0.0f;
	quat.q0 = 1.0f;
	quat.q.x = 0.0f;
	quat.q.y = 0.0f;
	quat.q.z = 0.0f;
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
}

sensorFusion::~sensorFusion() {
	// TODO Auto-generated destructor stub
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
		dataFusion(&gyro,&accl,&magn);
		if(cnt>printValues){
			//			PRINTF("QUAT: %f  %f  %f  %f\n",quat.q0,quat.q.x,quat.q.y,quat.q.z);
			//			PRINTF("roll:   %f   pitch:   %f   yaw:   %f\n",bank*TO_DEG,pitch*TO_DEG,heading*TO_DEG);
			PRINTF("filtered: ROLL    %f    PITCH    %f    YAW     %f\n",filtered.ROLL,filtered.PITCH,filtered.YAW);
			//			PRINTF("\nYAW:   %f   PITCH:   %f   ROLL:   %f   \n",angleRPY.MAG_YAW*TO_DEG,angleRPY.ACCL_PITCH*TO_DEG,angleRPY.ACCL_ROLL*TO_DEG);
//			PRINTF("\nYAW:   %f   PITCH:   %f   ROLL:   %f   \n",angleRPY.MAG_YAW,angleRPY.ACCL_PITCH,angleRPY.ACCL_ROLL);
			cnt = 0;
			BLUE_TOGGLE;
		}
		cnt++;
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
	//    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	//    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	//    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	//    pitch *= 180.0f / PI;
	//    yaw   *= 180.0f / PI;
	//    yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	//    roll  *= 180.0f / PI;

	//	heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)
	//	attitude = asin(2*qx*qy + 2*qz*qw)
	//	bank = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx2 - 2*qz2)

	// heading first
	double q0, q1, q2, q3;
	q0 = q.q0;
	q1 = q.q.x;
	q2 = q.q.y;
	q3 = q.q.y;
	//	float heading, pitch, bank;
	heading = atan2(2.0f*q1*q3 - 2.0f*q0*q2,1.0f - 2.0f*(q1*q1) - 2.0f*(q2*q2));
	/** TODO Gimabl lock situation */
	//check for gimbal lock situation
	//	if(((fabs((q0*q1 + q2*q3))-0.5) < EPSILON_COMPARISON) && (q0*q1 + q2*q3) > 0.0f){
	//
	//	}
	pitch = asin(2.0f*q0*q1 + 2.0f*q2*q3);
	bank = atan2(2.0f*q0*q3 - 2.0f*q1*q2,1.0f -2.0f*(q0*q0) - 2.0f*(q2*q2));
	//	PRINTF("roll:   %f   pitch:   %f   yaw:   %f\n",bank,pitch,heading);

}
