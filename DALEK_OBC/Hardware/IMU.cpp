/*
 * IMU.cpp
 *
 *  Created on: Oct 31, 2015
 *      Author: arthur
 */

#include "IMU.h"
#include <inttypes.h>

//#include <unistd.h>

#define IMU_GYRO	0
#define IMU_ACCMAG	1

HAL_GPIO imu_g_cs(IMU_G_CS_PIN);
HAL_GPIO imu_x_cs(IMU_XM_CS_PIN);


HAL_GPIO reset(IMU_RESET_PIN);
HAL_I2C i2c2(I2C_IDX2);

HAL_SPI spi2(SPI_IDX1);



IMU::IMU() {
	//TODO: insert correct baudrate
}

IMU::~IMU() {
	// TODO Auto-generated destructor stub
}


void IMU::init(){
	int k =0;
	PRINTF("IMU Constructor called!\n");
	uint8_t time = 500*MILLISECONDS;

	/** SPI STUFF */
	k = spi2.init(1250000);
	PRINTF("init: %d\n",k);
	// setting SPI Mode:
	k = spi2.config(SPI_PARAMETER_MODE,1);
	PRINTF("config: %d\n",k);
	//init i2c
	i2c2.init(400000);

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
	gyroOffset = IMU_GYRO_DEFAULT_OFFSET;
	acclOffset = IMU_ACCL_DEFAULT_OFFSET;
	magnOffset = IMU_MAGN_DEFAULT_OFFSET;

	/** WHOIS CHECKS *************************************************************** */
	imu_g_cs.setPins(1);
	k=imu_g_cs.readPins();
	PRINTF("IMU G CS PIN:%d\n",k);
	transBuf[0] = (0x80 | (WHO_AM_I_GYRO));
	k = i2c2.writeRead(GYRO_ADDRESS,transBuf,1,recBuf,1);
	//	k = spi2.writeRead(transBuf,1,recBuf,30);
	PRINTF("whois gyro: k=%d -  %d\n",k,recBuf[0]); // should be 212 ,0xD4
	imu_g_cs.setPins(0);
	imu_x_cs.setPins(1);
	transBuf[0] = ( 0x80 | WHO_AM_I_MAGNACC);
	k = i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,1);
	//	k = spi2.writeRead(transBuf,1,recBuf,2);
	PRINTF("whois accMag: k=%d  -  %d\n",k,recBuf[0]); // should be 73 ,0x49
	imu_x_cs.setPins(0);


	/** ACCELEROMETER, MAGNETIC SENSOR AND TEMP SETUP *********************************************/
	imu_x_cs.setPins(1);
	transBuf[0] = (CTRL_REG5_XM);
	transBuf[1] = 0x94; // -> enable temp readings, set high resolution magnetometer, read frequency 100Hz
	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,2);
	//	k = spi2.write(transBuf,2);
	//	PRINTF("k-val: %d\n",k);
	//enable Accelerometer
	transBuf[0] = (CTRL_REG1_XM);
	transBuf[1] = 0x87;//0b10000111 -> 400Hz, continuous reading, all axes enabled
	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,2);
	//	k = spi2.write(transBuf,2);
	//	PRINTF("k-val 2: %d\n",k);
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



	imu_x_cs.setPins(1);
	//some more debug checks
	//	//	transBuf[0] = TEMP_L;
	//	k = i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,1);
	//	//	k = spi2.writeRead(transBuf,1,recBuf,1);
	//	PRINTF("got k=%d  -  ACC REG1:%d\n",k,recBuf[0]); // should be 0x67 -> 103
	//	transBuf[0] = (0x80 | CTRL_REG5_XM);
	//	k = i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,1);
	//	//	k = spi2.writeRead(transBuf,1,recBuf,1);
	//	PRINTF("got k=%d  -  ACC REG5:%d\n",k,recBuf[0]); // should be 0x94 -> 148
	imu_x_cs.setPins(0);




	/** GYRO SETTINGS *********************************************** */
	imu_g_cs.setPins(1);
	transBuf[0] = CTRL_REG1_G;
	transBuf[1] = 0xCF; //0b11001111 Normal power mode, all axes enabled,  760Hz, 30 cutoff
	i2c2.write(GYRO_ADDRESS,transBuf,2);
	//	spi2.write(transBuf,2);
	transBuf[0] = CTRL_REG4_G;
	PRINTF("Setting Gyro Scale to %d\n",IMU_GYRO_RANGE);
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
	//
	//
	//	//check data
	imu_g_cs.setPins(1);
	transBuf[0] =(0x80 | CTRL_REG1_G);
	k = i2c2.writeRead(GYRO_ADDRESS,transBuf,1,recBuf,1);
	//	k = spi2.writeRead(transBuf,1,recBuf,2);
	PRINTF("got k=%d  -  GYRO REG1:%d\n",k,recBuf[0]); // should be 0xCf -> 207
	imu_g_cs.setPins(0);





}


//TODO retcodes
int IMU::resetIMU(){
	//cycle PD7 off->on for reset //TODO: but how fast???
	reset.setPins(0);
	reset.setPins(1);
	return 0;
}

IMU_DATA IMU::scaleData(){
	IMU_DATA tmp;
	tmp.TEMP_RAW = temp_raw[0];
	tmp.ANGULAR_RAW_X = (gyro_raw[0] - gyroOffset)* gyroSensitivity;
	tmp.ANGULAR_RAW_Y = (gyro_raw[1] - gyroOffset)* gyroSensitivity;
	tmp.ANGULAR_RAW_Z = (gyro_raw[2] - gyroOffset)* gyroSensitivity;
	tmp.ACCEL_RAW_X = (accl_raw[0] - acclOffset)* acclSensitivity;
	tmp.ACCEL_RAW_Y = (accl_raw[1] - acclOffset)* acclSensitivity;
	tmp.ACCEL_RAW_Z = (accl_raw[2] - acclOffset)* acclSensitivity;
	tmp.MAGNETIC_RAW_X = (magn_raw[0] - magnOffset)* magnSensitivity;
	tmp.MAGNETIC_RAW_Y = (magn_raw[1] - magnOffset)* magnSensitivity;
	tmp.MAGNETIC_RAW_Z = (magn_raw[2] - magnOffset)* magnSensitivity;
	return tmp;
}

IMU_DATA IMU::readIMU_Data(){
	int k = 0;
	k = read_multiple_Register(IMU_GYRO,(X_ANGULAR_L),6,gyro_raw);
	k = read_multiple_Register(IMU_ACCMAG,X_ACCEL_L,6,accl_raw);
	k = read_multiple_Register(IMU_ACCMAG,X_MAGNETIC_L,6,magn_raw);
	k = read_multiple_Register(IMU_ACCMAG,TEMP_L,2,temp_raw);

	newData = scaleData();
	PRINTF("\nGYRO:   %f   %f   %f  degree/sec\nACCL:   %f   %f   %f   G\nMAGN:   %f   %f   %f   gauss\n",newData.ANGULAR_RAW_X,newData.ANGULAR_RAW_Y,newData.ANGULAR_RAW_Z,newData.ACCEL_RAW_X,newData.ACCEL_RAW_Y,newData.ACCEL_RAW_Z,newData.MAGNETIC_RAW_X,newData.MAGNETIC_RAW_Y,newData.MAGNETIC_RAW_Z);
}

/**
 * function reads multiple register (at least 2)
 * return
 * 			0 everything went well
 * 			-1 minimum values to read are 2!
 * 			-2 values to read must be an even number! (because complements of high and low bytes are read)
 * 			-3 destination array does not match number of values to read divided by two!
 */
/** TODO destination size check! */
int IMU::read_multiple_Register(int cs,uint8_t reg,int valuesToRead, int16_t *dest){
	if(valuesToRead < 2) return -1;
	if(!(valuesToRead%2 == 0)) return -2;

	// select register and set to read
	transBuf[0] = (0x80 | (reg & 0x3F));
	int j = 0;

	if(cs == IMU_GYRO){
		imu_g_cs.setPins(1);
		i2c2.writeRead(GYRO_ADDRESS,transBuf,1,recBuf,valuesToRead);
		for(int i=0;i<valuesToRead;i+=2){
//			PRINTF("recBufGyro %d:  %d, %d\n",i,recBuf[i],recBuf[i+1]);
			dest[j] =(int16_t)(recBuf[i] | (recBuf[i+1] << 8));
//			PRINTF("converted:%d",(int16_t)(recBuf[i] | (recBuf[i+1] << 8)));
			j++;
		}

		imu_g_cs.setPins(0);
	}else{
		imu_x_cs.setPins(1);

		i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,6);

		imu_x_cs.setPins(0);
		if(dest == temp_raw){
			temp_raw[0] = (((int16_t) recBuf[1] << 12) | recBuf[0] << 4 ) >> 4; // temperature is signed 12bit integer
		}else {
			for(int i=0;i<valuesToRead;i+=2){
				dest[j] =(int16_t)(recBuf[i] | (recBuf[i+1] << 8));
				j++;
			}
		}

	}
	return 0;

}


void IMU::run(){
	PRINTF("run called\n");
	while(1){
		suspendCallerUntil(NOW()+1000*MILLISECONDS);
		this->readIMU_Data();
	}
}

void IMU::setTime(int time){
	this->time = time;
	PRINTF("setting period time to %d\n",time);
}

// setter functions
void IMU::setGyroScale(int scale){

}


void IMU::calibrateSensors(){

}
