/*
 * IMU.cpp
 *
 *  Created on: Oct 31, 2015
 *      Author: arthur
 */


#include "IMU.h"

#include <inttypes.h>


#define IMU_GYRO		0
#define IMU_ACCMAG		1
#define IMU_EXT_MAG		2
#define IMU_EXT_ACC		3

static Application senderName("IMU_Data_Publisher",500);


HAL_GPIO imu_g_cs(IMU_G_CS_PIN);
HAL_GPIO imu_x_cs(IMU_XM_CS_PIN);


HAL_GPIO reset(IMU_RESET_PIN);

uint16_t samples = 0;

#define SET_EXT_MAGN_REG		2
#define SET_EXT_ACCL_REG		1

static const uint8_t setExtMagn[SET_EXT_MAGN_REG][2] = {
		{EXT_CRA_REG_M, 0x18}, 		// Normal Measurement mode ( no bias), 75Hz
		{EXT_MR_REG_M, 0x00}		// continuous conversion mode
};
static const uint8_t setExtAccl[SET_EXT_ACCL_REG][2] = {
		{EXT_CTRL_REG1_A, 0x2F}	// normal mode, 100Hz 74Hz cutoff, all axes
};
float magn_values[3][3] = {{ -1.0081,   -0.1128,   -0.6109},
		{-0.1128    ,1.2625    ,0.6999},
		{-0.6109    ,0.6999    ,13.2807}};




IMU::IMU() : Thread ("IMU Thread",90){
	cnt_failedReads = 0;
	samples = 0;
	k =0;
	oldSamplerateTime = 0.0;
	samplerateTime = 0.0;
	cosFactor = 0.0;
	deltaYaw = 0.0;
	deltaPitch = 0.0;
	deltaRoll = 0.0;
	calibrationFinished = false;
	calGyro = false;
	calAccl = false;
	calMagn = false;
	initDone = false;
}

IMU::~IMU() {
}


void IMU::init(){

}

void IMU::regInit(){


	//init array
	memset(recBuf,0,sizeof(recBuf));
	memset(transBuf,0,sizeof(transBuf));
	memset(gyro_raw,0,sizeof(gyro_raw));
	memset(accl_raw,0,sizeof(accl_raw));
	memset(magn_raw,0,sizeof(magn_raw));
	memset(temp_raw,0,sizeof(temp_raw));

	//init GPIOs
	imu_g_cs.init(true,1,1);
	imu_x_cs.init(true,1,1);
	reset.init(true,1,1);

	//init offsets
	memset(gyroOffset,IMU_GYRO_DEFAULT_OFFSET,sizeof(gyroOffset));
	memset(acclOffset,IMU_ACCL_DEFAULT_OFFSET,sizeof(acclOffset));
	memset(magnOffset,IMU_MAGN_DEFAULT_OFFSET,sizeof(magnOffset));

	gyroOffset[0] = GYRO_OFFSET_X;
	gyroOffset[1] = GYRO_OFFSET_Y;
	gyroOffset[2] = GYRO_OFFSET_Z;
#ifdef USE_EXTERN_ACCL
	//	acclOffset[0] = -1335.75f;
	//	acclOffset[1] = -660.25f;
	//	acclOffset[2] = -5550.5f;
	//	magnOffset[0] = 1614.0f;
	//	magnOffset[1] = -15103.5f;
	//	magnOffset[2] = -3570.0f;
#else
	acclOffset[0] = ACCL_OFFSET_X;
	acclOffset[1] = ACCL_OFFSET_Y;
	acclOffset[2] = ACCL_OFFSET_Z;
	magnOffset[0] = MAGN_SCALE_X;
	magnOffset[1] = MAGN_SCALE_Y;
	magnOffset[2] = MAGN_SCALE_Z;
#endif


#ifdef USE_EXTERN_MAGN
	/* INIT external magnetometer */
	for(int i=0;i<SET_EXT_MAGN_REG;i++){
		transBuf[0] = setExtMagn[i][0];
		transBuf[1] = setExtMagn[i][1];
		k = i2c2.write(EXT_MAG_ADDRESS,transBuf,2);
	}

	transBuf[0] = EXT_CRB_REG_M;
	switch(EXT_MAGN_RANGE){
	case 13:
		transBuf[1] = 0x20; // gain: XY:1055 Z 950
		magnSensitivity = (float) 1.0f/1055.0f;
		break;
	case 19:
		transBuf[1] = 0x40; // gain: XY: 795 Z 710
		magnSensitivity = (float) 1.0f/795.0f;
		break;
	case 25:
		transBuf[1] = 0x60; // gain: XY: 635 Z 570
		magnSensitivity = (float) 1.0f/635.0f;
		break;
	case 40:
		transBuf[1] = 0x80; // gain: XY: 430 Z 385
		magnSensitivity = (float) 1.0f/430.0f;
		break;
	case 47:
		transBuf[1] = 0xA0; // gain: XY: 375 Z 335
		magnSensitivity = (float) 1.0f/375.0f;
		break;
	case 56:
		transBuf[1] = 0xC0; // gain: XY: 320 Z 285
		magnSensitivity = (float) 1.0f/320.0f;
		break;
	case 81:
		transBuf[1] = 0xE0; // gain: XY: 230 Z 205
		magnSensitivity = (float) 1.0f/230.0f;
		break;
	default:
		PRINTF("WRONG EXTERNAL MAGN RANGE DEFINITION IN BASIC.H\n suspending IMU Thread...\n");
		suspendCallerUntil(END_OF_TIME);
		break;
	}

	k = i2c2.write(EXT_MAG_ADDRESS,transBuf,2);
	PRINTF("init external Mag done with k %d\n",k);
#endif



#ifdef USE_EXTERN_ACCL

	// init external accelerometer
	for(int i=0;i<SET_EXT_ACCL_REG;i++){
		transBuf[0] = setExtAccl[i][0];
		transBuf[1] = setExtAccl[i][1];
		k = i2c2.write(EXT_ACC_ADDRESS,transBuf,2);
	}

	transBuf[0] = EXT_CTRL_REG4_A;
	switch (EXT_ACCL_RANGE) {
	case 2:
		transBuf[1] = 0x00; //2G, continuus update
		acclSensitivity = EXT_ACCL_2G_SENSITIVITY;
		break;
	case 4:
		transBuf[1] = 0x04; //4G, continuus update
		acclSensitivity = EXT_ACCL_4G_SENSITIVITY;
		break;
	case 8:
		transBuf[1] = 0x0C; //8G, continuus update
		acclSensitivity = EXT_ACCL_8G_SENSITIVITY;
		break;
	default:
		break;
	}

	k = i2c2.write(EXT_ACC_ADDRESS,transBuf,2);
	PRINTF("init external Accl done with k %d\n",k);

#endif

	/** WHOIS CHECKS *************************************************************** */
	imu_g_cs.setPins(1);
	k=imu_g_cs.readPins();
//	PRINTF("IMU G CS PIN:%d\n",k);
	transBuf[0] = (0x80 | (WHO_AM_I_GYRO));
	k = i2c2.writeRead(GYRO_ADDRESS,transBuf,1,recBuf,1);
	//	PRINTF("whois gyro: k=%d -  %d\n",k,recBuf[0]); // should be 212 ,0xD4
	imu_g_cs.setPins(0);
	imu_x_cs.setPins(1);
	transBuf[0] = ( 0x80 | WHO_AM_I_MAGNACC);
	k = i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,1);
	//	PRINTF("whois accMag: k=%d  -  %d\n",k,recBuf[0]); // should be 73 ,0x49
	imu_x_cs.setPins(0);


	/** ACCELEROMETER, MAGNETIC SENSOR AND TEMP SETUP *********************************************/
	imu_x_cs.setPins(1);
	transBuf[0] = (CTRL_REG5_XM);
	transBuf[1] = 0x94; // -> enable temp readings, set high resolution magnetometer, read frequency mag 100Hz
	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,2);
#ifndef USE_EXTERN_ACCL
	//enable Accelerometer
	transBuf[0] = (CTRL_REG1_XM);
	transBuf[1] = 0x7F;//0b01111111 -> 200Hz, block update reading, all axes enabled
	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,2);
	//	PRINTF("accl enable: %d\n",k);

	// setting Magnetic sensor mode to continuous conversion mode
	transBuf[0] = CTRL_REG7_XM;
	transBuf[1] = 0x00; //0b00000000 -> continuous conversion mode and rest as default
	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,2);
	//	 setting range
	transBuf[0] = CTRL_REG2_XM;
	switch(IMU_ACCL_RANGE){
	case 2:
		transBuf[1] = ACCL_2G;
		acclSensitivity = ACCL_2G_SENSITIVITY;
		break;
	case 4:
		transBuf[1] = ACCL_4G;
		acclSensitivity = ACCL_4G_SENSITIVITY;
		break;
	case 6:
		transBuf[1] = ACCL_6G;
		acclSensitivity = ACCL_6G_SENSITIVITY;
		break;
	case 8:
		transBuf[1] = ACCL_8G;
		acclSensitivity = ACCL_8G_SENSITIVITY;
		break;
	case 16:
		transBuf[1] = ACCL_16G;
		acclSensitivity = ACCL_16G_SENSITIVITY;
		break;
	default:
		PRINTF("WRONG ACCL SCALE DEFINITION IN BASIC.H\n suspending IMU Thread...\n");
		suspendCallerUntil(END_OF_TIME);
		break;
	}
	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,2);

	//now setting magnetic range
	imu_x_cs.setPins(1);
	transBuf[0] = CTRL_REG6_XM;
	switch(IMU_MAGN_RANGE){
	case 2:
		transBuf[1] = MAGN_2GAUSS;
		magnSensitivity = MAGN_2GAUSS_SENSITIVITY;
		break;
	case 4:
		transBuf[1] = MAGN_4GAUSS;
		magnSensitivity = MAGN_4GAUSS_SENSITIVITY;
		break;
	case 8:
		transBuf[1] = MAGN_8GAUSS;
		magnSensitivity = MAGN_8GAUSS_SENSITIVITY;
		break;
	case 12:
		transBuf[1] = MAGN_12GAUSS;
		magnSensitivity = MAGN_12GAUSS_SENSITIVITY;
		break;
	default:
		PRINTF("WRONG MAGN SCALE DEFINITION IN BASIC.H\n suspending IMU Thread...\n");
		suspendCallerUntil(END_OF_TIME);
		break;
	}
	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,2);

	imu_x_cs.setPins(0);
#endif



	/** GYRO SETTINGS *********************************************** */
	imu_g_cs.setPins(1);
	transBuf[0] = CTRL_REG1_G;
	transBuf[1] = 0x6F; //0b01101111 Normal power mode, all axes enabled,  190Hz, 50 cutoff
	i2c2.write(GYRO_ADDRESS,transBuf,2);
	transBuf[0] = CTRL_REG4_G;
	//	PRINTF("Setting Gyro Scale to %d DPS \n",IMU_GYRO_RANGE);
	switch(IMU_GYRO_RANGE){
	case 245:
		transBuf[1] = GYRO_245DPS;
		gyroSensitivity = GYRO_245DPS_SENSITIVITY;
		break;
	case 500:
		transBuf[1] = GYRO_500DPS;
		gyroSensitivity = GYRO_500DPS_SENSITIVITY;
		break;
	case 2000:
		transBuf[1] = GYRO_2000DPS;
		gyroSensitivity = GYRO_2000DPS_SENSITIVITY;
		break;
	default:
		PRINTF("WRONG GYRO SCALE DEFINITION IN BASIC.H\nsuspending IMU Thread...\n");
		suspendCallerUntil(END_OF_TIME);
		break;
	}

	i2c2.write(GYRO_ADDRESS,transBuf,2);
	imu_g_cs.setPins(1);

	//	//check data
	imu_g_cs.setPins(1);
	transBuf[0] =(0x80 | CTRL_REG1_G);
	k = i2c2.writeRead(GYRO_ADDRESS,transBuf,1,recBuf,1);
	PRINTF("init return IMU=%d  -  GYRO REG1:%d\n",k,recBuf[0]); // should be 0xCf -> 207
	imu_g_cs.setPins(0);

	// do only once
	calibrateSensors();
	while(!calibrationFinished);

	initDone = true;
}
bool IMU::initFinished(){
	return initDone;
}

//TODO retcodes
int IMU::resetIMU(){

	// reset I2C lines
	i2c2.reset();
	reset.setPins(0);
	suspendCallerUntil(NOW()+200*MILLISECONDS);

	reset.setPins(1);
	suspendCallerUntil(NOW()+200*MILLISECONDS);
	regInit();

	cnt_failedReads =0;
	return 0;
}


IMU_DATA_RAW IMU::scaleData(){
	IMU_DATA_RAW tmp;
	tmp.TEMP_RAW = temp_raw[0];
	tmp.ANGULAR_RAW_X = (gyro_raw[0] - gyroOffset[0])* gyroSensitivity;
	tmp.ANGULAR_RAW_Y = (gyro_raw[1] - gyroOffset[1])* gyroSensitivity;
	tmp.ANGULAR_RAW_Z = (gyro_raw[2] - gyroOffset[2])* gyroSensitivity;

#ifdef USE_EXTERN_ACCL
	tmp.ACCEL_RAW_X = (accl_raw[0] - acclOffset[0])* acclSensitivity;
	tmp.ACCEL_RAW_Y = ((accl_raw[1] - acclOffset[1])* acclSensitivity) *(-1.0); // invert y axis
	tmp.ACCEL_RAW_Z = ((accl_raw[2] - acclOffset[2])* acclSensitivity);
#else
	tmp.ACCEL_RAW_X = (accl_raw[0] - acclOffset[0])* acclSensitivity;
	tmp.ACCEL_RAW_Y = (accl_raw[1] - acclOffset[1])* acclSensitivity;
	tmp.ACCEL_RAW_Z = ((accl_raw[2] - acclOffset[2])* acclSensitivity);
#endif

#ifdef USE_EXTERN_MAGN
	tmp.MAGNETIC_RAW_X = (magn_raw[0] - magnOffset[0])* magnSensitivity;
	tmp.MAGNETIC_RAW_Y = (magn_raw[1] - magnOffset[1])* magnSensitivity * (-1.0);
	tmp.MAGNETIC_RAW_Z = (magn_raw[2] - magnOffset[2])* magnSensitivity;
#else
	// scale data with hard ironed values
	float temp[3];
    temp[0] = magn_raw[0] - magnOffset[0];
    temp[1] = magn_raw[1] - magnOffset[1];
    temp[2] = magn_raw[2] - magnOffset[2];
	tmp.MAGNETIC_RAW_X = (magn_values[0][0] * temp[0] + magn_values[0][1] * temp[1] + magn_values[0][2] * temp[2]) * magnSensitivity;
	tmp.MAGNETIC_RAW_Y = (magn_values[1][0] * temp[0] + magn_values[1][1] * temp[1] + magn_values[1][2] * temp[2]) * magnSensitivity;
	tmp.MAGNETIC_RAW_Z = (magn_values[2][0] * temp[0] + magn_values[2][1] * temp[1] + magn_values[2][2] * temp[2]) * magnSensitivity*(-1.0);
	/* OLD CALIBRATION METHOD - DEPRECATED
	tmp.MAGNETIC_RAW_X = (magn_raw[0] - magnOffset[0])* magnSensitivity;
	tmp.MAGNETIC_RAW_Y = (magn_raw[1] - magnOffset[1])* magnSensitivity;
	tmp.MAGNETIC_RAW_Z = (magn_raw[2] - magnOffset[2])* magnSensitivity;*/
#endif


	//	tmp.MAGNETIC_RAW_X = (tmp.MAGNETIC_RAW_X - MAG_MIN_X) / (MAG_MAX_X - MAG_MIN_X) * 2 - 1.0;
	//	tmp.MAGNETIC_RAW_Y = (tmp.MAGNETIC_RAW_Y - MAG_MIN_Y) / (MAG_MAX_Y - MAG_MIN_Y) * 2 - 1.0;
	//	tmp.MAGNETIC_RAW_Z = (tmp.MAGNETIC_RAW_Z - MAG_MIN_Z) / (MAG_MAX_Z - MAG_MIN_Z) * 2 - 1.0;
	tmp.ANGULAR_RAW_X *= TO_RAD;
	tmp.ANGULAR_RAW_Y *= TO_RAD;
	tmp.ANGULAR_RAW_Z *= TO_RAD;
	return tmp;

}

IMU_DATA_RAW IMU::readIMU_Data(){
	int k = 0;
#ifdef AUTO_RESET_IMU
	oldData = newData;
#endif
	k = read_multiple_Register(IMU_GYRO,(X_ANGULAR_L),6,gyro_raw);

#ifdef USE_EXTERN_ACCL
	k = read_multiple_Register(IMU_EXT_ACC,(EXT_OUT_X_L_A),6,accl_raw);
#else
	k = read_multiple_Register(IMU_ACCMAG,X_ACCEL_L,6,accl_raw);
#endif


#ifdef USE_EXTERN_MAGN
	k = read_multiple_Register(IMU_EXT_MAG,EXT_OUT_X_H_M,6,magn_raw);
#else
	k = read_multiple_Register(IMU_ACCMAG,X_MAGNETIC_L,6,magn_raw);
#endif


	k = read_multiple_Register(IMU_ACCMAG,TEMP_L,2,temp_raw);

	// print raw values
	//	PRINTF("\nraw Gyro:  %d  %d  %d\n",gyro_raw[0],gyro_raw[1],gyro_raw[2]);
	//	PRINTF("raw Accl:  %d  %d  %d\n",accl_raw[0],accl_raw[1],accl_raw[2]);
	//	PRINTF("raw Magn:  %d  %d  %d\n",magn_raw[0],magn_raw[1],magn_raw[2]);
	//	samples++;
	double tmp = SECONDS_NOW();

	//	samplerateTime = SECONDS_NOW();

	newData = scaleData();
	newData.currentSampleTime = tmp;
	//	PRINTF("seconds now %f\n",tmp);

	//				PRINTF("\nSamples: %d\nGYRO:   %f   %f   %f  rad/sec\nACCL:   %f   %f   %f   G\nMAGN:   %f   %f   %f   gauss\n",samples,newData.ANGULAR_RAW_X,newData.ANGULAR_RAW_Y,newData.ANGULAR_RAW_Z,newData.ACCEL_RAW_X,newData.ACCEL_RAW_Y,newData.ACCEL_RAW_Z,newData.MAGNETIC_RAW_X,newData.MAGNETIC_RAW_Y,newData.MAGNETIC_RAW_Z);

#ifdef AUTO_RESET_IMU
	//check for hangs on each channel -> if hang, try to reset IMU
	if((fabsf(oldData.ACCEL_RAW_X - newData.ACCEL_RAW_X) < EPSILON_COMPARISON) &&
			(fabsf(oldData.ACCEL_RAW_Y - newData.ACCEL_RAW_Y) <EPSILON_COMPARISON)&&
			(fabsf(oldData.ACCEL_RAW_Z - newData.ACCEL_RAW_Z) < EPSILON_COMPARISON)) cnt_failedReads++;
	if((fabsf(oldData.ANGULAR_RAW_X - newData.ANGULAR_RAW_X) < EPSILON_COMPARISON) &&
			(fabsf(oldData.ANGULAR_RAW_Y - newData.ANGULAR_RAW_Y) < EPSILON_COMPARISON) &&
			(fabsf(oldData.ANGULAR_RAW_Z - newData.ANGULAR_RAW_Z) < EPSILON_COMPARISON)) cnt_failedReads++;
	if((fabsf(oldData.MAGNETIC_RAW_X - newData.MAGNETIC_RAW_X) < EPSILON_COMPARISON) &&
			(fabsf(oldData.MAGNETIC_RAW_Y - newData.MAGNETIC_RAW_Y) < EPSILON_COMPARISON) &&
			(fabsf(oldData.MAGNETIC_RAW_Z - newData.MAGNETIC_RAW_Z) < EPSILON_COMPARISON)) cnt_failedReads++;
	if(cnt_failedReads > RESET_IMU_AFTER){
		PRINTF("IMU Hang detected! Resetting IMU\n");
		RED_ON;
		//		suspendCallerUntil(END_OF_TIME);
		long tmp = SECONDS_NOW();
		long timeSinceStart = tmp - debugTime;
		PRINTF("Hang after %d seconds!",SECONDS_NOW());
		suspendCallerUntil(END_OF_TIME);
		this->resetIMU();
		//		init();
		//				i2c2.reset();
		//				cnt_failedReads = 0;
	}
#endif
}

/**
 * converts gyro angles to RPY
 */
void IMU::convertToRPY(){
	cosFactor = 1/(cosf(angleRPY.GYRO_PITCH));
	deltaPitch = cosFactor * ((cosf(angleRPY.GYRO_ROLL) * cosf(angleRPY.GYRO_PITCH)*newData.ANGULAR_RAW_Y) - (sinf(angleRPY.GYRO_ROLL)*cosf(angleRPY.GYRO_PITCH)*(newData.ANGULAR_RAW_Z )));
	deltaRoll = cosFactor * ((cosf(angleRPY.GYRO_PITCH) * (newData.ANGULAR_RAW_X)) + (sinf(angleRPY.GYRO_ROLL)*sinf(angleRPY.GYRO_PITCH)*(newData.ANGULAR_RAW_Y)) + (cosf(angleRPY.GYRO_ROLL)*sin(angleRPY.GYRO_PITCH)*(newData.ANGULAR_RAW_Z )));
	deltaYaw = cosFactor * ((sinf(angleRPY.GYRO_ROLL) * (newData.ANGULAR_RAW_Y )) + (cosf(angleRPY.GYRO_ROLL)*(newData.ANGULAR_RAW_Z )));

	float sampleDiff = samplerateTime - oldSamplerateTime;

	angleRPY.GYRO_YAW += (deltaYaw*sampleDiff);
	angleRPY.GYRO_PITCH += (deltaPitch*sampleDiff);
	angleRPY.GYRO_ROLL += (deltaRoll*sampleDiff);
	//	PRINTF("\ndifference sample:   %f   \n",sampleDiff);

	oldSamplerateTime = samplerateTime;
	//	PRINTF("\nYAW:  %f  PITCH:   %f   ROLL:   %f   ",angleRPY.GYRO_YAW*TO_DEG,angleRPY.GYRO_PITCH*TO_DEG,angleRPY.GYRO_ROLL*TO_DEG);
	//	PRINTF("\ndeltaYaw:   %f   deltaPitch:   %f   deltaRoll:   %f   \n",deltaYaw,deltaPitch,deltaRoll);

#ifdef USE_EXTERN_ACCL
	// calc pitch, then roll then heading, see Application Note AN3192 for the LSM303DLH
	angleRPY.ACCL_PITCH = asinf(-newData.ACCEL_RAW_X);
	angleRPY.ACCL_ROLL = asinf(newData.ACCEL_RAW_Y/cosf(angleRPY.ACCL_PITCH));
	float xh = newData.MAGNETIC_RAW_X*cosf(angleRPY.ACCL_PITCH) + newData.MAGNETIC_RAW_Z*sinf(angleRPY.ACCL_PITCH);
	float yh = newData.MAGNETIC_RAW_X*sinf(angleRPY.ACCL_ROLL)*sinf(angleRPY.ACCL_PITCH) + newData.MAGNETIC_RAW_Y*cosf(angleRPY.ACCL_ROLL) - newData.MAGNETIC_RAW_Z*sinf(angleRPY.ACCL_ROLL)*cosf(angleRPY.ACCL_PITCH);
	float zh =-newData.MAGNETIC_RAW_X*cosf(angleRPY.ACCL_ROLL)*sinf(angleRPY.ACCL_PITCH) + newData.MAGNETIC_RAW_Y*sinf(angleRPY.ACCL_ROLL) - newData.MAGNETIC_RAW_Z*cosf(angleRPY.ACCL_ROLL)*cosf(angleRPY.ACCL_PITCH);

	angleRPY.MAG_YAW = 180.0 * atan2f(yh,xh) / M_PI;
	if(yh <= 0.0) angleRPY.MAG_YAW += 360.0;
#else
	// acclererometer convert to RPY -> yaw angle can not be get by accl, so use magnetometer
	//	angleRPY.MAG_YAW = atan(newData.ACCEL_RAW_Z/(sqrt((newData.ACCEL_RAW_X*newData.ACCEL_RAW_X) + (newData.ACCEL_RAW_Z*newData.ACCEL_RAW_Z))));
	angleRPY.ACCL_PITCH = atan(newData.ACCEL_RAW_X/(sqrt((newData.ACCEL_RAW_Y*newData.ACCEL_RAW_Y) + (newData.ACCEL_RAW_Z*newData.ACCEL_RAW_Z))));
	angleRPY.ACCL_ROLL = atan(newData.ACCEL_RAW_Y/(sqrt((newData.ACCEL_RAW_X*newData.ACCEL_RAW_X) + (newData.ACCEL_RAW_Z*newData.ACCEL_RAW_Z))));
	// use accl pitch and roll for tilt compensation
	angleRPY.MAG_YAW = atan(((newData.MAGNETIC_RAW_X*sin(angleRPY.ACCL_ROLL)*sin(angleRPY.ACCL_PITCH))+(newData.MAGNETIC_RAW_Y*cos(angleRPY.ACCL_ROLL))-(newData.MAGNETIC_RAW_Z*sin(angleRPY.ACCL_ROLL)*cos(angleRPY.ACCL_PITCH)))
			/((newData.MAGNETIC_RAW_X*cos(angleRPY.ACCL_PITCH)) + (newData.MAGNETIC_RAW_Z*sin(angleRPY.ACCL_PITCH))));
	//	PRINTF("\nYAW:   %f   PITCH:   %f   ROLL:   %f   \n",angleRPY.ACCL_YAW*TO_DEG,angleRPY.ACCL_PITCH*TO_DEG,angleRPY.ACCL_ROLL*TO_DEG);
#endif




}

/**
 * function reads multiple register (at least 2)
 * return
 * 			 0 everything went well
 * 			-1 minimum values to read are 2!
 * 			-2 values to read must be an even number! (because complements of high and low bytes are read)
 * 			-3 destination array does not match number of values to read divided by two!
 */
/** TODO destination size check! */
int IMU::read_multiple_Register(int cs,uint8_t reg,int valuesToRead, int16_t *dest){

	if(valuesToRead < 2) return -1;

	if(!(valuesToRead%2 == 0)) return -2;

	int j  = 0;

	if(cs == IMU_GYRO){

		transBuf[0] = (0x80 | (reg & 0x3F));
		imu_g_cs.setPins(1);
		i2c2.writeRead(GYRO_ADDRESS,transBuf,1,recBuf,valuesToRead);

		for(int i=0;i<valuesToRead;i+=2){
			dest[j] =(int16_t)(recBuf[i] | (recBuf[i+1] << 8));
			j++;
		}

		imu_g_cs.setPins(0);
	}else if(cs == IMU_ACCMAG){
		transBuf[0] = (0x80 | (reg & 0x3F));

		imu_x_cs.setPins(1);
		i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,valuesToRead);
		imu_x_cs.setPins(0);

		if(dest == temp_raw){
			temp_raw[0] = (((int16_t) recBuf[1] << 12) | recBuf[0] << 4 ) >> 4; // temperature is signed 12bit integer
		}else {
			for(int i=0;i<valuesToRead;i+=2){
				dest[j] =(int16_t)(recBuf[i] | (recBuf[i+1] << 8));
				j++;
			}
		}

	} else if(cs == IMU_EXT_MAG){

		transBuf[0] = reg;

		i2c2.writeRead(EXT_MAG_ADDRESS,transBuf,1,recBuf,valuesToRead);

		for(int i=0;i<valuesToRead;i+=2){
			dest[j] =(int16_t)((recBuf[i] << 8) | (recBuf[i+1]));
			j++;
		}
	} else if(cs == IMU_EXT_ACC){

		transBuf[0] = reg;

		i2c2.writeRead(EXT_ACC_ADDRESS,transBuf,1,recBuf,valuesToRead);

		for(int i=0;i<valuesToRead;i+=2){
			int16_t tmp = (recBuf[i] | (recBuf[i+1] <<8));
			dest[j] =(int16_t) (tmp >> 4); // raw data is 12 bit -> shift by 4s
			j++;
		}

	}
	return 0;

}






void IMU::calibrateSensors(){
	int16_t temp[3];
	int64_t gyro_temp[3];
	int64_t accl_temp[12];
	float accl_calc_temp[6];
	memset(gyro_temp,0,sizeof(gyro_temp));
	memset(temp,0,sizeof(temp));
	memset(accl_temp,0,sizeof(accl_temp));

	if(calAccl){
		PRINTF("calibrating - move board to standard position!\n");
		int cnt = 0;
		GREEN_OFF;
		while(cnt < 25){
			GREEN_TOGGLE; RED_TOGGLE; BLUE_TOGGLE; ORANGE_TOGGLE;
			suspendCallerUntil(NOW()+100*MILLISECONDS);
			cnt++;
		}
		GREEN_ON; RED_ON;
		PRINTF("calibrate for Z-axis, don't move the board!\n");
		//calibrate Z-axis (top view STM-board -> blue/black on top)
		for(int i=0;i<CALIBRAION_SAMPLES;i++){
			//#ifdef USE_EXTERN_ACCL
			//			read_multiple_Register(IMU_EXT_ACC,EXT_OUT_X_H_A,2,temp);
			//#else
			read_multiple_Register(IMU_ACCMAG,X_ACCEL_L,2,temp);
			//#endif
			accl_temp[0] += temp[0];
			PRINTF("accltemp 0: %"PRId64";  %d\n",accl_temp[0], temp[0]);
			read_multiple_Register(IMU_ACCMAG,Y_ACCEL_L,2,temp);
			accl_temp[1] += temp[0];
			PRINTF("accltemp2 0: %"PRId64";  %d\n",accl_temp[0], temp[0]);
			suspendCallerUntil(NOW()+9*MILLISECONDS);
		}
		GREEN_OFF; RED_OFF;
		PRINTF("please turn board around (invert z-axis)\n");
		cnt = 0;
		while(cnt<50){
			BLUE_TOGGLE; ORANGE_TOGGLE;
			suspendCallerUntil(NOW()+100*MILLISECONDS);
			cnt++ ;
		}
		BLUE_ON; ORANGE_ON;
		for(int i=0;i<CALIBRAION_SAMPLES;i++){
			read_multiple_Register(IMU_ACCMAG,X_ACCEL_L,2,temp);
			accl_temp[2] += temp[0];
			read_multiple_Register(IMU_ACCMAG,Y_ACCEL_L,2,temp);
			accl_temp[3] += temp[0];
			suspendCallerUntil(NOW()+9*MILLISECONDS);
		}

		ORANGE_OFF;
		BLUE_ON;
		PRINTF("please turn board to -y-axis in direction of blue LED\n");
		cnt = 0;
		while(cnt<50){
			BLUE_TOGGLE;
			suspendCallerUntil(NOW()+100*MILLISECONDS);
			cnt++ ;
		}
		BLUE_ON;
		for(int i=0;i<CALIBRAION_SAMPLES;i++){
			read_multiple_Register(IMU_ACCMAG,X_ACCEL_L,2,temp);
			accl_temp[4] += temp[0];
			read_multiple_Register(IMU_ACCMAG,Z_ACCEL_L,2,temp);
			accl_temp[5] += temp[0];
			suspendCallerUntil(NOW()+9*MILLISECONDS);
		}

		BLUE_OFF;
		PRINTF("please turn board to in direction of orange LED\n");
		cnt = 0;
		while(cnt<50){
			ORANGE_TOGGLE;
			suspendCallerUntil(NOW()+100*MILLISECONDS);
			cnt++ ;
		}
		ORANGE_ON;
		for(int i=0;i<CALIBRAION_SAMPLES;i++){
			read_multiple_Register(IMU_ACCMAG,X_ACCEL_L,2,temp);
			accl_temp[6] += temp[0];
			read_multiple_Register(IMU_ACCMAG,Z_ACCEL_L,2,temp);
			accl_temp[7] += temp[0];
			suspendCallerUntil(NOW()+9*MILLISECONDS);
		}
		ORANGE_OFF;
		PRINTF("please turn board to -x-axis in direction of red LED\n");
		cnt = 0;
		while(cnt<50){
			RED_TOGGLE;
			suspendCallerUntil(NOW()+100*MILLISECONDS);
			cnt++ ;
		}
		RED_ON;
		for(int i=0;i<CALIBRAION_SAMPLES;i++){
			read_multiple_Register(IMU_ACCMAG,Y_ACCEL_L,2,temp);
			accl_temp[8] += temp[0];
			read_multiple_Register(IMU_ACCMAG,Z_ACCEL_L,2,temp);
			accl_temp[9] += temp[0];
			suspendCallerUntil(NOW()+9*MILLISECONDS);
		}
		RED_OFF;
		PRINTF("please turn board to in direction of green LED\n");
		cnt = 0;
		while(cnt<50){
			GREEN_TOGGLE;
			suspendCallerUntil(NOW()+100*MILLISECONDS);
			cnt++ ;
		}
		GREEN_ON;
		for(int i=0;i<CALIBRAION_SAMPLES;i++){
			read_multiple_Register(IMU_ACCMAG,Y_ACCEL_L,2,temp);
			accl_temp[10] += temp[0];
			read_multiple_Register(IMU_ACCMAG,Z_ACCEL_L,2,temp);
			accl_temp[11] += temp[0];
			suspendCallerUntil(NOW()+9*MILLISECONDS);
		}
		GREEN_OFF;

		//debugoutput
		for(int i=0;i< 10;i++){
			PRINTF("1: %"PRId64", 2: %"PRId64" \n",accl_temp[i],accl_temp[i+2]);
		}

		// now calculate values
		// x1, x2, y1, y2, z1, z2 offset values, then in the end average them
		accl_calc_temp[0] = ((accl_temp[0]/CALIBRAION_SAMPLES) + (accl_temp[2] / CALIBRAION_SAMPLES))/2.0;
		accl_calc_temp[1] = ((accl_temp[4]/CALIBRAION_SAMPLES) + (accl_temp[6] / CALIBRAION_SAMPLES))/2.0;
		accl_calc_temp[2] = ((accl_temp[1]/CALIBRAION_SAMPLES) + (accl_temp[3] / CALIBRAION_SAMPLES))/2.0;
		accl_calc_temp[3] = ((accl_temp[8]/CALIBRAION_SAMPLES) + (accl_temp[10] / CALIBRAION_SAMPLES))/2.0;
		accl_calc_temp[4] = ((accl_temp[5]/CALIBRAION_SAMPLES) + (accl_temp[7] / CALIBRAION_SAMPLES))/2.0;
		accl_calc_temp[5] = ((accl_temp[9]/CALIBRAION_SAMPLES) + (accl_temp[11] / CALIBRAION_SAMPLES))/2.0;
		// now average that stuff -> that's the final offset corresponding to 0g-measurement
		acclOffset[0] = (accl_calc_temp[0] + accl_calc_temp[1])/2.0;
		acclOffset[1] = (accl_calc_temp[2] + accl_calc_temp[3])/2.0;
		acclOffset[2] = (accl_calc_temp[4] + accl_calc_temp[5])/2.0;
		PRINTF("ACCL CAL: %f, %f, %f\n",acclOffset[0],acclOffset[1],acclOffset[2]);
		calAccl = false;
	}

	if(calGyro){
		PRINTF("now calibrate Gyro; don't touch the board\n");
		int cnt = 0;
		while(cnt < 50){
			GREEN_TOGGLE; RED_TOGGLE; BLUE_TOGGLE; ORANGE_TOGGLE;
			suspendCallerUntil(NOW()+100*MILLISECONDS);
			cnt++;
		}
		GREEN_ON; BLUE_ON; ORANGE_ON; RED_ON;
		suspendCallerUntil(NOW()+1000*MILLISECONDS);
		for(int i=0;i<(CALIBRAION_SAMPLES*5);i++){
			read_multiple_Register(IMU_GYRO,(X_ANGULAR_L),6,temp);
			gyro_temp[0] += temp[0];
			gyro_temp[1] += temp[1];
			gyro_temp[2] += temp[2];
			//		if(i%100 == 0)PRINTF("\novf;  %d,%d,%d",gyro_temp[0],gyro_temp[1],gyro_temp[2]);
			suspendCallerUntil(NOW()+10*MILLISECONDS);
		}
		gyroOffset[0] = (gyro_temp[0] / (CALIBRAION_SAMPLES*5));
		gyroOffset[1] = (gyro_temp[1] / (CALIBRAION_SAMPLES*5));
		gyroOffset[2] = (gyro_temp[2] / (CALIBRAION_SAMPLES*5));
		PRINTF("GYRO CAL: %f, %f, %f\n",gyroOffset[0],gyroOffset[1],gyroOffset[2]);
		calGyro = false;
	}

	int16_t minX,minY,minZ = 0;
	int16_t maxX,maxY,maxZ = 0;
	if(calMagn){
		/** USE Ellipsoid fit for internal magnetometer calibration, credits for the matlab script goes to Christoph Hagen! */

		BLUE_ON; ORANGE_ON;
		PRINTF("calibrating Magnetometer, prepare...\n");
		suspendCallerUntil(NOW()+500*MILLISECONDS);
		PRINTF("you have now 30 seconds to move 360 degree in all directions! go..\n");
		suspendCallerUntil(NOW()+1000*MILLISECONDS);
		int cnt = 0;
		while(cnt <= CALIBRAION_SAMPLES*10){
			read_multiple_Register(IMU_ACCMAG,X_MAGNETIC_L,6,temp);
			//CSV Output
			PRINTF("%d,%d,%d\n",temp[0],temp[1],temp[2]);
			Delay_millis(1);
			BLUE_TOGGLE;
			cnt++;
		}
//		while(cnt <= CALIBRAION_SAMPLES){
//			read_multiple_Register(IMU_ACCMAG,X_MAGNETIC_L,6,temp);
//			if(temp[0] < minX) minX = temp[0];
//			else if(temp[0] > maxX) maxX = temp[0];
//
//			if(temp[1] < minY) minY = temp[1];
//			else if(temp[1] > maxY) maxY = temp[1];
//
//			if(temp[2] < minZ) minZ = temp[2];
//			else if(temp[2] > maxZ) maxZ = temp[2];
//			Delay_millis(5);
//			BLUE_TOGGLE;
//			cnt++;
//		}
		BLUE_OFF; ORANGE_OFF;
		PRINTF("calibration finished, calculating values.., min x %d, max x %d\n",minX,maxX);
		magnOffset[0] = ((float)minX + (float)maxX) / 2.0;
		magnOffset[1] = ((float)minY + (float)maxY) / 2.0;
		magnOffset[2] = ((float)minZ + (float)maxZ) / 2.0;
		PRINTF("MAGN CAL: %d, %d, %d\n",magnOffset[0],magnOffset[1],magnOffset[2]);
		calMagn = false;
	}
	calibrationFinished = true;
}



void IMU::run(){

	int tmp = 0;
	debugTime = NOW()*SECONDS;

	int printValues = IMU_PRINT_VALUES/IMU_SAMPLERATE;
	int cnt =0;
	//	suspendCallerUntil(END_OF_TIME);
	while(!this->initDone);

	while(1){
		cnt++;
		suspendCallerUntil(NOW()+IMU_SAMPLERATE*MILLISECONDS);
		this->readIMU_Data();
#ifdef FUSION_ENABLE
		imu_rawData.publish(newData);
#else
		this->convertToRPY();
#endif
		if(cnt>printValues){
//						PRINTF("\nSamples: %d\nGYRO:   %f   %f   %f  degree/sec\nACCL:   %f   %f   %f   G\nMAGN:   %f   %f   %f   gauss\n",samples,newData.ANGULAR_RAW_X,newData.ANGULAR_RAW_Y,newData.ANGULAR_RAW_Z,newData.ACCEL_RAW_X,newData.ACCEL_RAW_Y,newData.ACCEL_RAW_Z,newData.MAGNETIC_RAW_X,newData.MAGNETIC_RAW_Y,newData.MAGNETIC_RAW_Z);
//						PRINTF("\n\nGYRO YAW:   %f   PITCH:    %f   ROLL:   %f   ",angleRPY.GYRO_YAW*TO_DEG,angleRPY.GYRO_PITCH*TO_DEG,angleRPY.GYRO_ROLL*TO_DEG);
//						PRINTF("\nACCL YAW:   %f   PITCH:    %f   ROLL:   %f   ",angleRPY.MAG_YAW*TO_DEG,angleRPY.ACCL_PITCH*TO_DEG,angleRPY.ACCL_ROLL*TO_DEG);

#ifndef FUSION_ENABLE
			//			PRINTF("\nSamples: %d\nGYRO:   %f   %f   %f  degree/sec\nACCL:   %f   %f   %f   G\nMAGN:   %f   %f   %f   gauss\n",samples,newData.ANGULAR_RAW_X,newData.ANGULAR_RAW_Y,newData.ANGULAR_RAW_Z,newData.ACCEL_RAW_X,newData.ACCEL_RAW_Y,newData.ACCEL_RAW_Z,newData.MAGNETIC_RAW_X,newData.MAGNETIC_RAW_Y,newData.MAGNETIC_RAW_Z);
			//			PRINTF("\n\nGYRO YAW:   %f   PITCH:    %f   ROLL:   %f   ",angleRPY.GYRO_YAW*TO_DEG,angleRPY.GYRO_PITCH*TO_DEG,angleRPY.GYRO_ROLL*TO_DEG);
			//			PRINTF("\nACCL YAW:   %f   PITCH:    %f   ROLL:   %f   ",angleRPY.MAG_YAW*TO_DEG,angleRPY.ACCL_PITCH*TO_DEG,angleRPY.ACCL_ROLL*TO_DEG);
			PRINTF("heading: %f\n",angleRPY.MAG_YAW);

#endif
			cnt =0;
		}

	}
}



