/*
 * sensorFusion.cpp
 *
 *  Created on: Nov 8, 2015
 *      Author: arthur
 *      Purpose of sensorFusion class is to fuse Accl, Gyro and Magneotmeter by a kalman-based Quaternion-Madgwick-filter -> advantage: not as computation heavy as kalman!
 *      see http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
 */

#include "sensorFusion.h"
#define FILTERGAIN		0.1f



//static Application senderName("RPY Publisher",501);




sensorFusion::sensorFusion(){
	// TODO Auto-generated constructor stub
}

sensorFusion::~sensorFusion() {
	// TODO Auto-generated destructor stub
}


void sensorFusion::init(){
	integrationDelta = 20000.0f;
	quat.q0 = 1.0f;
	quat.q.x = 0.0f;
	quat.q.y = 0.0f;
	quat.q.z = 0.0f;
	propGain = FILTERGAIN;
	accl.x = 0;
	accl.y = 0;
	accl.z = 0;
	gyro.x = 0;
	gyro.y = 0;
	gyro.z = 0;
	magn.x = 0;
	magn.y = 0;
	magn.z = 0;
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
			PRINTF("QUAT: %f  %f  %f  %f\n",quat.q0,quat.q.x,quat.q.y,quat.q.z);
			PRINTF("roll:   %f   pitch:   %f   yaw:   %f\n",bank*TO_DEG,pitch*TO_DEG,heading*TO_DEG);
			cnt = 0;
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
}

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
