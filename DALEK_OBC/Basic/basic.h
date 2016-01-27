/*
 * basic.h
 *
 *  Created on: Nov 4, 2015
 *      Author: arthur
 */

#ifndef BASIC_BASIC_H_
#define BASIC_BASIC_H_

#include "rodos.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "support_libs.h"
#include "protocol.h"
#include "stm324xg_eval.h"
#include "stm32f4xx_rcc.h"
#include "../support_libs/wifi/wf121.h"
#include "sysDelay/systick.h"




/***************************** ENABLE AND DISABLE SHIT ***********************************/
#define IMU_ENABLE

#define TTNC_ENABLE
#define TELEMETRY_ENABLE
#define FUSION_ENABLE
#define LIGHT_ENABLE
#define CURRENT_ENABLE
#define CAMERA_ENABLE							// IMPORTANT!!!! CAMERA HAS TO BE INITIALISED BEFORE I2C Channel One!!!!
#define MOTOR_ENABLE
#define SOLAR_ADC_ENABLE
#define IR_ENABLE
//#define WIFI_ENABLE								// enables Communication via Wifi -> comment to use Bluetooth
//#define KNIFE_ENABLE
//#define SUNFINDER_ENABLE

#ifdef FUSION_ENABLE
//#define MADGWICK								// enables the madgwick filter
#define MADGWICK_TWO							// enbales the other madgwick filter
#ifndef MADGWICK
//		#define COMPLEMENTARY					// enables the complementary filter -> used when madgwick is disabled!
#endif
#endif



/************* BASIC STUFF AND I2C/SPI/UART Things **************************************/
extern "C" HAL_I2C i2c1;
extern "C" HAL_I2C i2c2;
extern "C" HAL_ADC adc1; 						// ADC one (the one on the extension board)
#ifndef WIFI_ENABLE
extern "C" HAL_UART bt_uart;
#endif


#define ADC1_RESOLUTION			12				// Resolution for ADC Channel 1

#define EPSILON_COMPARISON		0.0001			// used to compare two floats or doubles
#define TO_RAD					(M_PI/180.0)
#define TO_DEG					(180.0/M_PI)
#define COMPL_GAIN				0.98f			// Complementary Filter Gain
#define forLoop(x,n)			for(int x=0;x<n;x++)

#define I_ERROR_LIMITATION		5				// add integral error only if error is big -> avoid stupid integration of small errors


/****************************** LED STUFF ************************************************/
#define LED_GREEN 				GPIO_Pin_12					// WARNING: The Extension Board maps these Pins to the H-Bridge-PWM ! Don't use when H-Brdige is needed!!!
#define LED_ORANGE 				GPIO_Pin_13
#define LED_RED 				GPIO_Pin_14
#define LED_BLUE 				GPIO_Pin_15
// took the function from GPIO_SetBits etc; see stm32f4xx_gpio.h
#define GREEN_ON				//GPIOD->BSRRL = LED_GREEN
#define GREEN_OFF				//GPIOD->BSRRH = LED_GREEN
#define GREEN_TOGGLE      		//GPIOD->ODR ^= LED_GREEN
#define ORANGE_ON				GPIOD->BSRRL = LED_ORANGE
#define ORANGE_OFF				GPIOD->BSRRH = LED_ORANGE
#define ORANGE_TOGGLE			GPIOD->ODR ^= LED_ORANGE
#define RED_ON					//GPIOD->BSRRL = LED_RED
#define RED_OFF					//GPIOD->BSRRH = LED_RED
#define RED_TOGGLE				//GPIOD->ODR ^= LED_RED
#define BLUE_ON					GPIOD->BSRRL = LED_BLUE
#define BLUE_OFF				GPIOD->BSRRH = LED_BLUE
#define BLUE_TOGGLE				GPIOD->ODR ^= LED_BLUE



/****************************** WIFI & BT STUFF ******************************************/
#define WIFI_SSID				"THE_DALEK"
#define WIFI_SSID_PW			"EXTERMINATE"
#define WIFI_IP					0xFF01A8C0 // in hex and reverse
#define WIFI_PORT				7777

#define TTNC_SAMPLERATE			500				// milliseconds, check if new messages have arrived

#define BLUETOOTH_BAUDRATE		115200
#define BLUETOOTH_PORT			UART_IDX2
#define BLUETOOTH_BUFFER		512				// in bytes, should be enough for Command Frames! (currently 24 needed)



/****************************** IMU STUFF ************************************************/
//#define USE_EXTERN_MAGN							// use external magnetometer on LSM303DLH board ( connected to I2C1)
//#define USE_EXTERN_ACCL

#define IMU_RESET_PIN			GPIO_055
#define IMU_G_CS_PIN			GPIO_018
#define IMU_XM_CS_PIN			GPIO_032

#define IMU_GYRO_RANGE			500				// in DPS, select 245, 500 or 2000 sensitivity is set according to chosen value here
#define IMU_ACCL_RANGE			6				// value in g, select 2,4,6,8 or 16; sensitivity is set according to chosen value
#define IMU_MAGN_RANGE			8				// value in gauss, select 2,4,8 or 13; sensitivity is set according to chosen value

#define EXT_MAGN_RANGE			25				// value in gauss*10, select 1.3, 1.9, 2.5, 4.0, 4.7, 5.6 or 8.1; sensitivity/gain is set according to chosen value
#define EXT_ACCL_RANGE			2				// value in g, select 2, 4 or 8, gain/sensitivity is then set automatically
/** TODO EXT ACCL_RANGE other than 2 seems not to work DEBUG HERE */

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

	double currentSampleTime;
};
struct IMU_RPY_FILTERED{
	float YAW;
	float PITCH;
	float ROLL;
};




/* ***************************************** LIGHT SENSOR STUFF **********************************************/
#define LIGHT_SAMPLERATE		100				// Samplerate in milliseconds
struct LUX_DATA{
	bool activated;
	uint16_t LUX;
};



/* ***************************************** Camera STUFF **********************************************/
#define CAM_READ				0x43
#define CAM_WRITE				0x42
#define CAM_ADDRESS				0x21
#define WIDTH					160
#define HEIGHT					121
#define CAPTUREMODE				DCMI_CaptureMode_SnapShot
#define FRAMERATE				DCMI_CaptureRate_All_Frame

struct CAM_DATA{
	bool activateCamera;
	bool capture;
	bool sendImage;
//	uint8_t *picture;
//	int width;
//	int height;
//	uint32_t consecutiveFrame;
};
extern "C" bool imFin;							// camera, for IRQ handler
extern "C" bool captureDone;



/* ***************************************** SolarPanel STUFF **********************************************/
#define SolarVoltageADC			ADC_CH_005 		//PA1 Pin
#define SOLAR_SAMPLERATE		200				//Samplerate in milliseconds
struct SOLAR_DATA{
	bool activated;
	int32_t Voltage;
};



/* ***************************************** Infrared Sensor STUFF **********************************************/
#define IR_ONE					ADC_CH_001		// PA2
#define IR_TWO					ADC_CH_002		// PA3
#define IR_THREE				ADC_CH_003		// PA5
#define IR_SAMPLERATE			200				// Samplerate in missileconds
struct IR_DATA{
	bool activated;
	float sensorOne;
	float sensorTwo;
	float sensorThree;
};




/* ***************************************** TM TC STUFF **********************************************/
#ifdef TTNC_ENABLE

#define COMMAND_ECHO							// if not commented, every command is echoed back
#endif
#define TM_SAMPLERATE			500			// in milliseconds

struct ACTIVE_SYSTEM_MODE{
	int activeMode;
};
struct TELEMETRY{
	PAYLOAD_FRAME plFrame;
	TELEMETRY_FRAME tmFrame;
	int updated; // 0 or 1 for pl or tm frame
};




/* ***************************************** Thermal Knife STUFF ******************************************/

struct KNIFE_DATA{
	bool activated;
};



/* ***************************************** Current Sensor STUFF ******************************************/
#define CURRENT_SAMPLERATE			2000 // in milliseconds
struct CURRENT_DATA{
	float batteryCurrent;
	float power;
	float batteryVoltage;
};



/* ***************************************** SUN FINDER TELEMETRY STUFF ******************************************/
struct SUNFINDER_TM{
	uint8_t currentProgress;
	float sunIncidenceAngle;
};




/* ***************************************** Inter-Thread Communication for Sensors ***********************/

/*
 * 	SET_BETA_GAIN,
	SET_ANGLE_P,
	SET_ANGLE_I,
	SET_ANGLE_D,
	SET_ROTAT_P,
	SET_ROTAT_I,
	SET_ROTAT_D for changedVal
 */
struct VAR_CONTROL{
	int changedVal;
	float value;
};

struct INTERCOMM{
	int changedVal;
	LUX_DATA luxData;
	SOLAR_DATA solData;
	IR_DATA irData;
	KNIFE_DATA knifeData;
	CAM_DATA camData;
	IMU_RPY_FILTERED imuData;
	CURRENT_DATA currentData;
	VAR_CONTROL varControlData;
	SUNFINDER_TM sunTM;
};
enum INTERCOMM_CHANGED{
	LUX_CHANGED,
	SOLAR_CHANGED,
	IR_CHANGED,
	KNIFE_CHANGED,
	CAM_CHANGED,
	IMU_CHANGED,
	CURRENT_CHANGED,
	VARIABLE_CHANGED,
	SUNFINDER_TM_CHANGED
};





/***************************************** TOPICS ***************************************************/
// now define the topics stuff for the RODOS middleware
extern Topic<IMU_DATA_RAW>	imu_rawData;
extern Topic<IMU_RPY_FILTERED> imu_filtered;

extern Topic<INTERCOMM> interThreadComm;

extern Topic<TELEMETRY> tm_data;
extern Topic<UDPMsg> tcRaw;
extern Topic<UDPMsg> tmPlFrame;
extern Topic<COMMAND_FRAME> commandFrame;
extern Topic<ACTIVE_SYSTEM_MODE> systemModeControl;








/*********************************** FUNCTIONS ***************************************/
void floatToChar(uint8_t* _target, float _number);
void doubleToChar(uint8_t* _target, double _number);
void longToChar(uint8_t* _target, uint32_t _number);
void shortToChar(uint8_t* _target, uint16_t _number);
void longLongToChar(uint8_t* _target, uint64_t _number);

float charToFloat(uint8_t* _number);
double charToDouble(uint8_t* _number);
uint64_t charToLongLong(uint8_t* _number);
uint16_t charToShort(uint8_t* _number);
uint32_t charToLong(uint8_t* _number);
void delayx(unsigned int _ms);

#endif /* BASIC_BASIC_H_ */
