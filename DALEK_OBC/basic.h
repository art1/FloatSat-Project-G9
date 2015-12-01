/*
 * basic.h
 *
 *  Created on: Nov 4, 2015
 *      Author: arthur
 */

#ifndef BASIC_H_
#define BASIC_H_

#include "rodos.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "support_libs.h"


/************* BASIC STUFF AND I2C/SPI/UART Things **************************************/
//extern "C" HAL_I2C i2c1;
extern "C" HAL_I2C i2c2;
extern "C" HAL_ADC adc1; 						// ADC one (the one on the extension board)

#define ADC1_RESOLUTION			12				// Resolution for ADC Channel 1

#define EPSILON_COMPARISON		0.0001			// used to compare two floats or doubles
#define TO_RAD					(M_PI/180.0)
#define TO_DEG					(180.0/M_PI)

/***************************** ENABLE AND DISABLE SHIT ***********************************/
//#define IMU_ENABLE
//#define TTNC_ENABLE
//#define FUSION_ENABLE
//#define LIGHT_ENABLE
//#define CAMERA_ENABLE
#define MOTOR_ENABLE
#define SOLAR_ADC_ENABLE
/****************************** LED STUFF ************************************************/
#define LED_GREEN 				GPIO_Pin_12
#define LED_ORANGE 				GPIO_Pin_13
#define LED_RED 				GPIO_Pin_14
#define LED_BLUE 				GPIO_Pin_15
// took the function from GPIO_SetBits etc; see stm32f4xx_gpio.h
#define GREEN_ON				GPIOD->BSRRL = LED_GREEN
#define GREEN_OFF				GPIOD->BSRRH = LED_GREEN
#define GREEN_TOGGLE      		GPIOD->ODR ^= LED_GREEN
#define ORANGE_ON				GPIOD->BSRRL = LED_ORANGE
#define ORANGE_OFF				GPIOD->BSRRH = LED_ORANGE
#define ORANGE_TOGGLE			GPIOD->ODR ^= LED_ORANGE
#define RED_ON					GPIOD->BSRRL = LED_RED
#define RED_OFF					GPIOD->BSRRH = LED_RED
#define RED_TOGGLE				GPIOD->ODR ^= LED_RED
#define BLUE_ON					GPIOD->BSRRL = LED_BLUE
#define BLUE_OFF				GPIOD->BSRRH = LED_BLUE
#define BLUE_TOGGLE				GPIOD->ODR ^= LED_BLUE

/****************************** WIFI STUFF ***********************************************/
#define TTNC_SSID				"YETENet"
#define TTNC_SSID_PW			"yeteyete"

/****************************** IMU STUFF ************************************************/
#define IMU_RESET_PIN			GPIO_055
#define IMU_G_CS_PIN			GPIO_018
#define IMU_XM_CS_PIN			GPIO_032

#define IMU_GYRO_RANGE			245				// in DPS, select 245, 500 or 2000 sensitivity is set according to chosen value here
#define IMU_ACCL_RANGE			2				// value in g, select 2,4,6,8 or 16; sensitivity is set according to chosen value
#define IMU_MAGN_RANGE			2				// value in gauss, select 2,4,8 or 13; sensitivity is set according to chosen value

#define IMU_GYRO_DEFAULT_OFFSET	1
#define IMU_ACCL_DEFAULT_OFFSET	1
#define IMU_MAGN_DEFAULT_OFFSET	1

#define CALIBRAION_SAMPLES		500				// calibration samples for gyro and accl and mag
#define IMU_SAMPLERATE			20				// read and fuse IMU data every XX milliseconds
#define IMU_PRINT_VALUES		500				// print values over UART USB every XX  ms
#define AUTO_RESET_IMU							// automatically resets the imu after RESET_IMU_AFTER_FAIL times failed to read data
#define RESET_IMU_AFTER			5				// resets the IMU if reading data failed for XXX times (e.g. same data is read, or IMU hangs)

struct IMU_DATA_RAW{
	float ANGULAR_RAW_X;
	float ANGULAR_RAW_Y;
	float ANGULAR_RAW_Z;

	float MAGNETIC_RAW_X;
	float MAGNETIC_RAW_Y;
	float MAGNETIC_RAW_Z;

	float ACCEL_RAW_X;
	float ACCEL_RAW_Y;
	float ACCEL_RAW_Z;

	float TEMP_RAW;
};
/* ***************************************** LIGHT SENSOR STUFF **********************************************/
#define LIGHT_SAMPLERATE		100			// Samplerate in milliseconds
struct LUX_DATA{
	bool activated;
	uint16_t LUX;
};


/* ***************************************** Camera STUFF **********************************************/



/* ***************************************** SolarPanel STUFF **********************************************/
#define SolarVoltageADC			ADC_CH_001	//PA1 Pin
#define SOLAR_SAMPLERATE		100			//Samplerate in milliseconds
struct SOLAR_DATA{
	bool activated;
	int32_t Voltage;
};


/***************************************** TOPICS ***************************************************/
// now define the topics stuff for the RODOS middleware
extern Topic<IMU_DATA_RAW>	imu_rawData;
extern Topic<LUX_DATA> lux_data;
extern Topic<SOLAR_DATA> solar_data;

#endif /* BASIC_H_ */

