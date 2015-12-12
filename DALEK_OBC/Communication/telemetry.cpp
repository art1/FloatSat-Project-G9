/*
 * tm.cpp
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#include "telemetry.h"

Telemetry::Telemetry() {

}

Telemetry::~Telemetry() {
}



void Telemetry::init(){

}

void Telemetry::run(){
	/** TODO add function to activate and eactivate telemetry */
	while(1){
		buildFrame();
		tmPlFrame.publish(msg);
		suspendCallerUntil(NOW()+TM_SAMPLERATE*MILLISECONDS);
	}
}



void Telemetry::setNewData(IMU_RPY_FILTERED _imu){
	this->imu.put(_imu);
}
void Telemetry::setNewData(LUX_DATA _lux){
	this->lux.put(_lux);
}
void Telemetry::setNewData(SOLAR_DATA _sol){
	this->sol.put(_sol);
}
void Telemetry::setNewData(IR_DATA _ir){
	this->ir.put(_ir);
}

void Telemetry::buildFrame(){
	msg.length = 0;
	memset(msg.data,0,sizeof(msg.data));

	uint8_t tmp[10];
	IMU_RPY_FILTERED rpy;
	imu.get(rpy);
	LUX_DATA l;
	lux.get(l);
	IR_DATA irData;
	ir.get(irData);

	for(int i=-1;i<15;i++){
		switch (i) {
		case -1:
			msg.data[msg.length++] = FRAME_START;
			break;
		case TM_FRAMETYPE:
			msg.data[msg.length++] = TM;
			break;
		case TM_FRAMENUMBER:
			/** TODO */
			break;
		case SYSTEM_MODE:
			/** TODO */
			break;
		case LIGHT:
			/** TODO */
			break;
		case ROLL:
			floatToChar(tmp,rpy.ROLL);
			for(int i=0;i<4;i++){
				msg.data[msg.length++] = tmp[i];
			}
			break;
		case PITCH:
			floatToChar(tmp,rpy.PITCH);
			for(int i=0;i<4;i++){
				msg.data[msg.length++] = tmp[i];
			}
			break;
		case YAW:
			floatToChar(tmp,rpy.YAW);
			for(int i=0;i<4;i++){
				msg.data[msg.length++] = tmp[i];
			}
			break;
		case SOLAR_VOLTAGE:
			/** TODO */
			break;
		case BATTERY_VOLTAGE:
			/** TODO */
			break;
		case CURRENT:
			/** TODO */
			break;
		case MOTOR_SPEED:
			/** TODO */
			break;
		case IR_DATA_ONE:
			/** TODO */
			break;
		case IR_DATA_TWO:
			/** TODO */
			break;
		case IR_DATA_THREE:
			/** TODO */
			break;
		case TM_LOCALTIME:
			/** TODO */
			break;
		default:
			PRINTF("oO this should never ever happen :o\n");
			break;
		}
		msg.data[msg.length++] = VALUE_SEPERATOR;
	}
	msg.data[msg.length++] = FRAME_END;
}

/**
 * puts double number to target as 8 bytes
 */
void Telemetry::doubleToChar(uint8_t* target, double number){
	char *tmp = (char *) &number;
	for(int i=0;i<8;i++){
		target[i] = tmp[i];
	}
}

/**
 * puts float-bytes to target array
 */
void Telemetry::floatToChar(uint8_t* target, float number){
	char *tmp = (char *) &number;
	for(int i=0;i<4;i++){
		target[i] = tmp[i];
	}
}

