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



IMU::IMU() {
	//TODO: insert correct baudrate
}

IMU::~IMU() {
	// TODO Auto-generated destructor stub
}


int IMU::init(){
	PRINTF("IMU Constructor called!");
	//init i2c
	i2c2.init(400000);
	memset(recBuf,0,sizeof(recBuf));
	memset(transBuf,0,sizeof(transBuf));

	//	spi_imu.init(1,imu_g_cs,imu_x_cs,625000);
	imu_g_cs.init(true,1,0);
	imu_x_cs.init(true,1,0);
	reset.init(true,1,0);

	//	resetIMU();
	// set up IMU
	//	int retCore = 0;
	//	retCore = spi_imu.spi_write(imu_g_cs,(CTRL_REG1_G),0xEF,1);//to CTRL_REG1_G 11101111 -> 760 Hz, 50 cutoff
	////	retCore = spi_imu.spi_writeThenRead(imu_g_cs,(CTRL_REG1_G & 0x3F),0xEF,1,1);
	//	PRINTF("750Hz: %d",retCore);
	//	uint8_t ret = spi_imu.spi_read(imu_g_cs,(0x80 | (CTRL_REG1_G)),1);
	//	PRINTF("got ret:%d\n",ret);
	//	spi_imu.spi_write(imu_g_cs,(CTRL_REG2_G),0x00,1);//to CTRL_REG2_G
	//	PRINTF(" CTRL_REG2_G: %d",retCore);
	//	spi_imu.spi_write(imu_g_cs,(CTRL_REG4_G),0x30,1);//to CTRL_REG4_G -> set dps mode to 2000 dps and SPI Mode 00110000
	//	PRINTF(" dps mode: %d\n",retCore);
	//
	//	spi_imu.spi_write(imu_x_cs,(CTRL_REG5_XM),0xF0,1);//11110000 -> enable temperature and set magnetic data rate to 50 Hz
	////	readIMU_Data();

	//set up IMU
	uint8_t tmp[2];
	tmp[0] = CTRL_REG5_XM;
	tmp[1] = 0x78; //0b11110000 -> enable temp readings, set high resolution magnetometer, read frequency 50Hz
	int k=0;
	k = i2c2.write(0x1C,tmp,2);
	PRINTF("k-val: %d\n",k);
	//enable Accelerometer
	tmp[0] = CTRL_REG1_XM;
	tmp[1] = 0x67;//0b01100111 -> 100Hz, continuous reading, all axes enabled
	k = i2c2.write(ACC_MAG_ADDRESS,tmp,2);
	PRINTF("k-val 2: %d\n",k);
	// setting scale in register 2 -> leave 0 for 2g
	//	tmp[0] = CTRL_REG2_XM;
	//	tmp[1] = 0x;//0b01100111 -> 100Hz, continuous reading, all axes enabled
	//	i2c_imu.write((uint8_t)ACC_MAG_ADDRESS,tmp,2);

	// read WHOIS
	imu_g_cs.setPins(1);
	uint8_t tempo[1] = {0};
	uint8_t txBuf[1] = {0};
	txBuf[0] = (0x80 | WHO_AM_I_GYRO);
	i2c2.writeRead(0x6A,txBuf,1,tempo,1);
	PRINTF("whois gyro: %d\n",tmp[0]); // should be 212 ,0xD4
	imu_g_cs.setPins(0);


}


//TODO retcodes
int IMU::resetIMU(){
	//cycle PD7 off->on for reset //TODO: but how fast???
	reset.setPins(0);
	reset.setPins(1);
	return 0;
}

IMU_DATA IMU::readIMU_Data(){
	// read the WHO AM I Data for test purposes
	//	PRINTF("trying to read data");
	//	unsigned int micros = 1000000;
	//	while(1){
	//			read_Register(IMU_GYRO,(WHO_AM_I_GYRO));
	read_Register(IMU_ACCMAG,X_ACCEL_L);
	read_Register(IMU_GYRO,(X_ANGULAR_L));
	//		read_Register(IMU_GYRO,(X_ANGULAR_H));
	//		read_Register(IMU_ACCMAG,TEMP_H);
	//	read_Register(IMU_ACCMAG,TEMP_L);
	//		usleep(micros);
	//	}


}

uint8_t IMU::read_Register(int cs,uint8_t reg){
	//	uint8_t test = spi_imu.spi_read(cs,(0x80 | (reg)),2);
	//	uint8_t test = i2c_imu.read((0x3D),&reg,2,1);
	uint8_t txBuf[1];
	txBuf[0] = (0x80 | reg);
	//	cs.setPins(1);
	if(cs == IMU_GYRO){
		imu_g_cs.setPins(1);
		//		uint8_t test = i2c_imu.read(GYRO_ADDRESS,txBuf,sizeof(txBuf),1);
		uint8_t temp[6];
		memset(temp,0,sizeof(temp));
		i2c2.writeRead(GYRO_ADDRESS,txBuf,1,temp,6);
		imu_x_cs.setPins(0);
		int m[3];
		m[0] = (int16_t)(temp[0] | temp[1] << 8);
		m[1] = (int16_t)(temp[2] | temp[3] << 8);
		m[2] = (int16_t)(temp[4] | temp[5] << 8);
		PRINTF("Gyro x:%d, y:%d, z:%d\n",m[0],m[1],m[2]);
		imu_g_cs.setPins(0);
	}else{
		imu_x_cs.setPins(1);
		//		uint8_t test = i2c_imu.read(ACC_MAG_ADDRESS,txBuf,sizeof(txBuf),1);
		uint8_t temp[6];
		memset(temp,0,sizeof(temp));
		i2c2.writeRead(ACC_MAG_ADDRESS,txBuf,1,temp,6);
		imu_x_cs.setPins(0);
		int m[3];
		m[0] = (int16_t)(temp[0] | temp[1] << 8);
		m[1] = (int16_t)(temp[2] | temp[3] << 8);
		m[2] = (int16_t)(temp[4] | temp[5] << 8);
		PRINTF("Acc x:%d, y:%d, z:%d\n",m[0],m[1],m[2]);

	}
	// first write address 0x3C (if pin20 is not connected to vcc... but i guess it is?
	//	txBuf[1] = 0x3B;
	//	PRINTF("reading %d, at adress 0x3A",txBuf[0]);

	// second write the register adress

}
