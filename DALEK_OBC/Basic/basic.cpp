/*
 * basic.cpp
 *
 *  Created on: Nov 7, 2015
 *      Author: arthur
 */

#include "basic.h"

//HAL_I2C i2c1(I2C_IDX1);
HAL_I2C i2c2(I2C_IDX2);
HAL_ADC adc1(ADC_IDX1);
HAL_UART bt_uart(BLUETOOTH_PORT);
uint8_t VSync = 0;

//declaring topics for RODOS middleware inter-Thread Communication
Topic<IMU_DATA_RAW> imu_rawData(1,"IMU Raw Data");
Topic<LUX_DATA> lux_data(2,"LUX Data");
Topic<SOLAR_DATA> solar_data(3,"Solar Voltage Data");
Topic<IR_DATA> ir_data(4,"Infrared Sensor Data");
Topic<CAM_CONTROL> cam_control(5,"Camera Control");
Topic<TELEMETRY> tm_data(6,"Telemetry Data");
Topic<UDPMsg> tcRaw(7,"Command Data Raw");
Topic<UDPMsg> tmPlFrame(8,"Telemetry Payload Frame");
Topic<COMMAND_FRAME> commandFrame(9,"Command Frame Data");
Topic<IMU_RPY_FILTERED> imu_filtered(10,"IMU Data filtered");
Topic<ACTIVE_SYSTEM_MODE> systemMode(11,"active System Mode");
Topic<KNIFE_DATA> knife_data(12,"Thermal Knife Data");




/*********************************** FUNCTIONS ***************************************/
void doubleToChar(uint8_t* target, double number){
	char *tmp = (char *) &number;

	for(int i=0;i<8;i++){
		target[i] = tmp[i];
	}
}

double charToDouble(uint8_t* _number){
	double out;

	uint8_t * ptr = (uint8_t *) &out;

	forLoop(i,8){
		ptr[i] = _number[i];
	}

	return out;
}



void floatToChar(uint8_t* target, float number){
	char *tmp = (char *) &number;

	for(int i=0;i<4;i++){
		target[i] = tmp[i];
	}
}

float charToFloat(uint8_t* _number){
	float out;

	uint8_t * ptr = (uint8_t *) &out;

	forLoop(i,4){
		ptr[i] = _number[i];
	}

	return out;
}

void longLongToChar(uint8_t* _target, uint64_t _number){
	char *tmp = (char*) &_number;

	forLoop(j,8){
		_target[j] = tmp[j];
	}
}

uint64_t charToLongLong(uint8_t* _number){
	uint64_t out;

	uint8_t * ptr = (uint8_t *) &out;

	forLoop(i,8){
		ptr[i] = _number[i];
	}
	return out;
}


void longToChar(uint8_t* _target, uint32_t _number){
	_target[0] = ((_number >> 24) & 0xFF);
	_target[1] = ((_number >> 16) & 0xFF);
	_target[2] = ((_number >> 8) & 0xFF);
	_target[3] = ((_number) & 0xFF);
}

uint32_t charToLong(uint8_t* _number){
	return ((_number[0] << 24 )
			+ (_number[1] << 16)
			+ (_number[2] << 8)
			+ (_number[3] ) );
}


void shortToChar(uint8_t* _target, uint16_t _number){
	_target[0] = _number & 0xFF;
	_target[1] = _number >> 8;
}


uint16_t charToShort(uint8_t* _number){
	return (_number[0] | (_number[1] << 8));
}
