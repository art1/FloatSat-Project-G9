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
	memset(recBuf,0,sizeof(recBuf));
	memset(transBuf,0,sizeof(transBuf));

	imu_g_cs.init(true,1,1);
	imu_x_cs.init(true,1,1);
	reset.init(true,1,1);

	// read WHOIS
	imu_g_cs.setPins(1);
	k=imu_g_cs.readPins();
	PRINTF("IMU G CS PIN:%d\n",k);
	transBuf[0] = (0x80 | (WHO_AM_I_GYRO));
	k = i2c2.writeRead(GYRO_ADDRESS,transBuf,1,recBuf,1);
//	k = spi2.writeRead(transBuf,1,recBuf,30);
	PRINTF("whois gyro: k=%d -  %d\n",k,recBuf[0]); // should be 212 ,0xD4
//	for(int i=0;i<30;i++){
//		PRINTF("%d  :  %d\n",i,recBuf[i]);
//	}
	imu_g_cs.setPins(0);
	k = imu_g_cs.readPins();
	PRINTF("IMU G CS PIN:%d\n",k);
	imu_x_cs.setPins(1);
	transBuf[0] = ( 0x80 | WHO_AM_I_MAGNACC);
	k = i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,1);
//	k = spi2.writeRead(transBuf,1,recBuf,2);
	PRINTF("whois accMag: k=%d  -  %d;%d\n",k,recBuf[0],recBuf[1]); // should be 73 ,0x49
	imu_x_cs.setPins(0);


	//set up IMU
	imu_x_cs.setPins(1);

	transBuf[0] = (CTRL_REG5_XM);
	transBuf[1] = 0x78; //0b11110000 -> enable temp readings, set high resolution magnetometer, read frequency 50Hz
	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,2);
//	k = spi2.write(transBuf,2);
	PRINTF("k-val: %d\n",k);
	//enable Accelerometer
	transBuf[0] = (CTRL_REG1_XM);
	transBuf[1] = 0x67;//0b01100111 -> 100Hz, continuous reading, all axes enabled
	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,2);
//	k = spi2.write(transBuf,2);
	PRINTF("k-val 2: %d\n",k);
	imu_x_cs.setPins(1);

	imu_x_cs.setPins(1);
//	transBuf[0] = TEMP_L;
	k = i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,1);
//	k = spi2.writeRead(transBuf,1,recBuf,1);
	PRINTF("got k=%d  -  ACC REG1:%d\n",k,recBuf[0]); // should be 0x67 -> 103
	transBuf[0] = (0x80 | CTRL_REG5_XM);
	k = i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,1);
//	k = spi2.writeRead(transBuf,1,recBuf,1);
	PRINTF("got k=%d  -  ACC REG5:%d\n",k,recBuf[0]); // should be 0x78 -> 120
	imu_x_cs.setPins(0);




	//enable Gyro
	imu_g_cs.setPins(1);
	transBuf[0] = CTRL_REG1_G;
	transBuf[1] = 0x0F; //0b00001111 Normal power mode, all axes enabled
	i2c2.write(GYRO_ADDRESS,transBuf,2);
//	spi2.write(transBuf,2);
//	transBuf[0] = CTRL_REG4_G;
//	transBuf[1] = ;
//	i2c2.write(GYRO_ADDRESS,transBuf,2);
	imu_g_cs.setPins(1);
//
//
//	//check data
	imu_g_cs.setPins(1);
	transBuf[0] =(0x80 | CTRL_REG1_G);
	k = i2c2.writeRead(GYRO_ADDRESS,transBuf,1,recBuf,1);
//	k = spi2.writeRead(transBuf,1,recBuf,2);
	PRINTF("got k=%d  -  GYRO REG1:%d\n",k,recBuf[0]); // should be 0xf -> 15
	imu_g_cs.setPins(0);
	// setting scale in register 2 -> leave 0 for 2g
	//	tmp[0] = CTRL_REG2_XM;
	//	tmp[1] = 0x;//0b01100111 -> 100Hz, continuous reading, all axes enabled
	//	i2c_imu.write((uint8_t)ACC_MAG_ADDRESS,tmp,2);




}


//TODO retcodes
int IMU::resetIMU(){
	//cycle PD7 off->on for reset //TODO: but how fast???
	reset.setPins(0);
	reset.setPins(1);
	return 0;
}

IMU_DATA IMU::readIMU_Data(){
	read_Register(IMU_ACCMAG,X_ACCEL_L);
	read_Register(IMU_GYRO,(X_ANGULAR_L));
	int k = 0;
//	imu_x_cs.setPins(0);
//	transBuf[0] = (0x80 |(TEMP_L));
////	transBuf[1] = (ACC_MAG_ADDRESS | 0x80);
////	k = i2c2.write(ACC_MAG_ADDRESS,transBuf,1);
////	PRINTF("\ngot k:%d");
////	k = i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,1);
//	k = spi2.writeRead(transBuf,1,recBuf,1);
//	imu_x_cs.setPins(1);
//	PRINTF("\ngit k:%d  -  TEMP:%d-%d",k,recBuf[0],recBuf[1]);

}

uint8_t IMU::read_Register(int cs,uint8_t reg){
	//	uint8_t test = spi_imu.spi_read(cs,(0x80 | (reg)),2);
	//	uint8_t test = i2c_imu.read((0x3D),&reg,2,1);
//	PRINTF("reg is:%d",reg);
	transBuf[0] = (0x80 | (reg & 0x3F));
//	PRINTF("\ntransbuf is:%d\n",transBuf[0]);
	//	cs.setPins(1);
	if(cs == IMU_GYRO){
		imu_g_cs.setPins(1);
		//		uint8_t test = i2c_imu.read(GYRO_ADDRESS,txBuf,sizeof(txBuf),1);
//		transBuf[0] = (0x80 | GYRO_ADDRESS);
		i2c2.writeRead(GYRO_ADDRESS,transBuf,1,recBuf,6);
//		spi2.writeRead(transBuf,1,recBuf,6);
		PRINTF("recBuf:\n%d,%d,%d,%d,%d,%d\n",recBuf[0],recBuf[1],recBuf[2],recBuf[3],recBuf[4],recBuf[5]);
//		imu_x_cs.setPins(0);
		int m[3];
		m[0] = (int16_t)(recBuf[0] | recBuf[1] << 8);
		m[1] = (int16_t)(recBuf[2] | recBuf[3] << 8);
		m[2] = (int16_t)(recBuf[4] | recBuf[5] << 8);
		PRINTF("Gyro x:%d, y:%d, z:%d\n",m[0],m[1],m[2]);
		imu_g_cs.setPins(0);
	}else{
		imu_x_cs.setPins(1);

		//		uint8_t test = i2c_imu.read(ACC_MAG_ADDRESS,txBuf,sizeof(txBuf),1);
//		txBuf[1] = (0xC0 | (ACC_MAG_ADDRESS & 0x3F));
		i2c2.writeRead(ACC_MAG_ADDRESS,transBuf,1,recBuf,6);
//		spi2.writeRead(transBuf,1,recBuf,7);
		PRINTF("recBuf:\n%d,%d,%d,%d,%d,%d,%d\n",recBuf[0],recBuf[1],recBuf[2],recBuf[3],recBuf[4],recBuf[5],recBuf[6]);

		imu_x_cs.setPins(0);
		int m[3];
		m[0] = (int16_t)(recBuf[0] | recBuf[1] << 8);
		m[1] = (int16_t)(recBuf[2] | recBuf[3] << 8);
		m[2] = (int16_t)(recBuf[4] | recBuf[5] << 8);
		PRINTF("Acc x:%d, y:%d, z:%d\n",m[0],m[1],m[2]);

	}


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
