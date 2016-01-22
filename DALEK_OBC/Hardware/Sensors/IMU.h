/*
 * IMU.h
 *
 *  Created on: Oct 31, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_SENSORS_IMU_H_
#define HARDWARE_SENSORS_IMU_H_

#include "math.h"
#include "hal.h"
#include "../../Basic/basic.h"

/**
 * This Class takes care of configuring and reading the Build-in IMU on the Extension Board, as well
 * as configuring and reading an external IMU, if connected (LSM303DLH). If the external IMU is used
 * (enabling/disabling in basic.h), its Accelerometer and Magnetometer are used for Sensor Fusion and
 * calculating the Heading (with tilt-compensation), but the Gyroscope from the built-in IMU will be
 * used!
 */

#define GYRO_ADDRESS 				0x6B
#define ACC_MAG_ADDRESS				0x1D
#define EXT_MAG_ADDRESS				0x1E /**  0x3C write, 0x3D read, but RODOS wants the 7bit adress left aligned, and shifts it by one later*/
#define EXT_ACC_ADDRESS				0x18

#define GYRO_245DPS_SENSITIVITY		0.00875f
#define GYRO_500DPS_SENSITIVITY		0.0175f
#define GYRO_2000DPS_SENSITIVITY	0.07f

#define ACCL_2G_SENSITIVITY			0.000061f
#define ACCL_4G_SENSITIVITY			0.000122f
#define ACCL_6G_SENSITIVITY			0.000183f
#define ACCL_8G_SENSITIVITY			0.000244f
#define ACCL_16G_SENSITIVITY		0.000732f

#define MAGN_2GAUSS_SENSITIVITY		0.00008f
#define MAGN_4GAUSS_SENSITIVITY		0.00016f
#define MAGN_8GAUSS_SENSITIVITY		0.00032f
#define MAGN_12GAUSS_SENSITIVITY	0.00048f

#define EXT_ACCL_2G_SENSITIVITY		0.001f
#define EXT_ACCL_4G_SENSITIVITY		0.002f
#define EXT_ACCL_8G_SENSITIVITY		0.0039f



//standard values derived from multiple Calibrations
#define GYRO_OFFSET_X				-241.0f
#define GYRO_OFFSET_Y				104.0f
#define GYRO_OFFSET_Z				208.0f

#define ACCL_OFFSET_X				-243.500f	//old-4800.500f
#define ACCL_OFFSET_Y				-41.0f  	//old339.5f
#define ACCL_OFFSET_Z				-377.25f 	//old-393.0f


//ACCL CAL: -243.500, -41.000, -377.250

#define MAGN_SCALE_X				9120
#define MAGN_SCALE_Y				-534
#define MAGN_SCALE_Z				-1647






/** OLD DEPRECATED VALUES */
//#define MAG_MAX_X 660
//#define MAG_MAX_Y 6922
//#define MAG_MAX_Z 2779
//#define MAG_MIN_X -7194
//#define MAG_MIN_Y -562
//#define MAG_MIN_Z -7053

// Data register adresses
enum IMU_DATA_REG_ADD{
	//config stuff
	WHO_AM_I_GYRO = 0x0F,
	CTRL_REG1_G = 0x20,
	CTRL_REG2_G = 0x21,
	CTRL_REG3_G = 0x22,
	CTRL_REG4_G = 0x23,
	CTRL_REG5_G = 0x24,
	REFERENCE_G = 0x25,
	STATUS_REG_G = 0x27,
	// output gyro
	X_ANGULAR_L = 0x28,
	X_ANGULAR_H = 0x29,
	Y_ANGULAR_L = 0x2A,
	Y_ANGULAR_H = 0x2B,
	Z_ANGULAR_L = 0x2C,
	Z_ANGULAR_H = 0x2D,
	// interrupt settings
	FIFO_CTRL_REG_G = 0x2E,
	FIFO_SRC_REG_G = 0x2E,
	INT1_CFG_G = 0x30,
	INT1_SRC_G = 0x31,
	INT1_TSH_XH_G = 0x32,
	INT1_TSH_XL_G = 0x33,
	INT1_TSH_YH_G = 0x34,
	INT1_TSH_YL_G = 0x35,
	INT1_TSH_ZH_G = 0x36,
	INT1_TSH_ZL_G = 0x37,
	INT1_DURATION_G = 0x38,

	//!! SET APPROPRIATE SLAVE ADRESS FOR READING FOLLOWING REGISTER! -> see Datasheet
	// set Appropriate pins for chaning Lsb of adress
	WHO_AM_I_MAGNACC = 0x0F,	//-> set appropriate slave adress!
	//output temperature
	TEMP_L = 0x05,
	TEMP_H = 0x06,
	STATUS_REG_M = 0x07,
	//output magnetometer
	X_MAGNETIC_L = 0x08,
	X_MAGNETIC_H = 0x09,
	Y_MAGNETIC_L = 0x0A,
	Y_MAGNETIC_H = 0x0B,
	Z_MAGNETIC_L = 0x0C,
	Z_MAGNETIC_H = 0x0D,
	//config n interrupt stuff
	INT_CTRL_REG_M = 0x12,
	INT_SRC_REG_M = 0x13,
	INT_THS_L_M = 0x14,
	INT_THS_H_M = 0x15,
	//offset magnetometer
	X_OFFSET_MAG_L = 0x16,
	X_OFFSET_MAG_H = 0x17,
	Y_OFFSET_MAG_L = 0x18,
	Y_OFFSET_MAG_H = 0x19,
	Z_OFFSET_MAG_L = 0x1A,
	Z_OFFSET_MAG_H = 0x1B,
	// reference magnetometer
	REFERENCE_X = 0x1C,
	REFERENCE_Y = 0x1D,
	REFERENCE_Z = 0x1E,
	//config stuff Mag/Gyro
	CTRL_REG0_XM = 0x1F,
	CTRL_REG1_XM = 0x20,
	CTRL_REG2_XM = 0x21,
	CTRL_REG3_XM = 0x22,
	CTRL_REG4_XM = 0x23,
	CTRL_REG5_XM = 0x24,
	CTRL_REG6_XM = 0x25,
	CTRL_REG7_XM = 0x26,
	STATUS_REG_A = 0x27,
	//output accelerometer
	X_ACCEL_L = 0x28,
	X_ACCEL_H = 0x29,
	Y_ACCEL_L = 0x2A,
	Y_ACCEL_H = 0x2B,
	Z_ACCEL_L = 0x2C,
	Z_ACCEL_H = 0x2D,
	//interrupt stuff
	FIFO_CTRL_REG = 0x2E,
	FIFO_SRC_REG = 0x2F,
	INT_GEN_1_REG = 0x30,
	INT_GEN_1_SRC = 0x31,
	INT_GEN_1_THS = 0x32,
	INT_GEN_1_DURATION = 0x33,
	INT_GEN_2_REG = 0x34,
	INT_GEN_2_SRC = 0x35,
	INT_GEN_2_THS = 0x36,
	INT_GEN_2_DURATION = 0x37,
	CLICK_CFG = 0x38,
	CLICK_SRC = 0x39,
	CLICK_THS = 0x3A,
	TIME_LIMIT = 0x3B,
	TIME_LATENCY = 0x3C,
	TIME_WINDOW = 0x3D,
	Act_THS = 0x3E,
	Act_DUR = 0x3F


};

enum IMU_RANGES{
	GYRO_245DPS = 0x00,		// continuous update, no self-test, 245DPS (for msb 0x40)
	GYRO_500DPS	= 0x10, 	// continuous update, no self-test, 500DPS (for msb 0x50)
	GYRO_2000DPS = 0x20,	// continuous update, no self-test, 200DPS (for msb 0x60)
	ACCL_2G	= 0x00,			// continuous update, no self test, 2G range, anti alias filter bandwidth 773Hz 0b00000000
	ACCL_4G = 0x08,			// continuous update, no self test, 4G range, anti alias filter bandwidth 773Hz 0b00001000
	ACCL_6G = 0x10,			// continuous update, no self test, 6G range, anti alias filter bandwidth 773Hz 0b00010000
	ACCL_8G = 0x18,			// continuous update, no self test, 8G range, anti alias filter bandwidth 773Hz 0b00011000
	ACCL_16G= 0x20,			// continuous update, no self test, 16G range, anti alias filter bandwidth 773Hz 0b00100000
	MAGN_2GAUSS = 0x00,		// Magnetic resolution 2 Gauss 0b00000000
	MAGN_4GAUSS = 0x20,		// Magnetic resolution 4 Gauss 0b00100000
	MAGN_8GAUSS = 0x40,		// Magnetic resolution 4 Gauss 0b01000000
	MAGN_12GAUSS = 0x60,	// Magnetic resolution 4 Gauss 0b01100000
};

struct IMU_OFFSETS{
	uint8_t X_OFFSET_MAG_L;	//16h
	uint8_t X_OFFSET_MAG_H;	//17h
	uint8_t Y_OFFSET_MAG_L;	//18h
	uint8_t Y_OFFSET_MAG_H;	//19h
	uint8_t Z_OFFSET_MAG_L;	//1Ah
	uint8_t Z_OFFSET_MAG_H;	//1Bh
};
enum EXTERNAL_MAGN {
	//external magnetometer/accelerometer used: LSM303DLH
	EXT_CTRL_REG1_A = 0x20,
	EXT_CTRL_REG2_A = 0x21,
	EXT_CTRL_REG3_A = 0x22,
	EXT_CTRL_REG4_A = 0x23,
	EXT_CTRL_REG5_A = 0x24,
	EXT_REFERENCE_A = 0x26,
	EXT_STATUS_REG_A = 0x27,
	EXT_OUT_X_L_A = 0xA8, // enabling increment in register adress with MSB set to 1, normal adress is 0x28
	EXT_OUT_X_H_A = 0x29,
	EXT_OUT_Y_L_A = 0x2A,
	EXT_OUT_Y_H_A = 0x2B,
	EXT_OUT_Z_L_A = 0x2C,
	EXT_OUT_Z_H_A = 0x2D,
	// interrupts aren't needed and so not configured at all
	EXT_CRA_REG_M = 0x00,
	EXT_CRB_REG_M = 0x01,
	EXT_MR_REG_M = 0x02,
	EXT_OUT_X_H_M = 0x03,
	EXT_OUT_X_L_M = 0x04,
	EXT_OUT_Y_H_M = 0x05,
	EXT_OUT_Y_L_M = 0x06,
	EXT_OUT_Z_H_M = 0x07,
	EXT_OUT_Z_L_M = 0x08,
	//the other interrupt register aren't needed either
};



class IMU : public Thread{
public:
	IMU();
	virtual ~IMU();
	void init();
	void regInit();
	void run();
	int resetIMU();
	void calibrateSensors();
	bool initFinished();
	IMU_DATA_RAW readIMU_Data();

private:
	bool initDone;
	uint8_t time;
	int read_multiple_Register(int cs, uint8_t reg,int valuesToRead,int16_t *dest);
	IMU_DATA_RAW scaleData();
	void convertToRPY();
	void setLEDMask(int command,int green,int red,int orange,int blue);
	float gyroSensitivity;
	float acclSensitivity;
	float magnSensitivity;
	float extAcclSensitivity;
	float extMagnSensitivity;
	float gyroOffset[3];
	float acclOffset[3];
	float magnOffset[3];
	uint8_t recBuf[512];
	uint8_t transBuf[512];
	int16_t gyro_raw[3];
	int16_t accl_raw[3];
	int16_t magn_raw[3];
	int16_t temp_raw[1];
	IMU_DATA_RAW oldData;
	IMU_DATA_RAW newData;

	struct IMU_RPY_RAW{
		float GYRO_YAW;
		float GYRO_ROLL;
		float GYRO_PITCH;
		float MAG_YAW;
		float ACCL_ROLL;
		float ACCL_PITCH;
	};

	IMU_RPY_RAW angleRPY;
	float samplerateTime;
	float oldSamplerateTime;
	float cosFactor;
	float deltaYaw;
	float deltaPitch;
	float deltaRoll;
	//more stuff
	int cnt_failedReads;
	bool calibrationFinished;
	int k =0;
	int debugTime =0;

	bool calGyro;
	bool calAccl;
	bool calMagn;



};

#endif /* HARDWARE_SENSORS_IMU_H_ */
