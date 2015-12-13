/*
 * tm.cpp
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#include "telemetry.h"

Telemetry::Telemetry() {
	this->frameNumber = 0;
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
		frameNumber++;
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
void Telemetry::setNewData(ACTIVE_SYSTEM_MODE _mode){
	this->systemMode.activeMode = _mode.activeMode;
}
uint32_t Telemetry::getCurrentFrameNumber(){
	return this->frameNumber;
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
	SOLAR_DATA s;
	sol.get(s);
	IMU_DATA_RAW rpyRaw;
	imuRaw.get(rpyRaw);

	for(int i=-1;i<15;i++){
		switch (i) {
		case -1:
			msg.data[msg.length++] = FRAME_START;
			break;
		case TM_FRAMETYPE:
			msg.data[msg.length++] = TM;
			break;
		case TM_FRAMENUMBER:
			longToChar(tmp,getCurrentFrameNumber());
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case SYSTEM_MODE:
			msg.data[msg.length++] = (uint8_t)this->systemMode.activeMode;
			break;
		case LIGHT:
			shortToChar(tmp,l.LUX);
			msg.data[msg.length++] = tmp[0];
			msg.data[msg.length++] = tmp[1];
			break;
		case ROLL:
			floatToChar(tmp,rpy.ROLL);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case PITCH:
			floatToChar(tmp,rpy.PITCH);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case YAW:
			floatToChar(tmp,rpy.YAW);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case GYRO_X:
			floatToChar(tmp,rpyRaw.ANGULAR_RAW_X);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case GYRO_Y:
			floatToChar(tmp,rpyRaw.ANGULAR_RAW_Y);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case GYRO_Z:
			floatToChar(tmp,rpyRaw.ANGULAR_RAW_Z);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case ACCL_X:
			floatToChar(tmp,rpyRaw.ACCEL_RAW_X);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case ACCL_Y:
			floatToChar(tmp,rpyRaw.ACCEL_RAW_Y);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case ACCL_Z:
			floatToChar(tmp,rpyRaw.ACCEL_RAW_Z);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case MAG_X:
			floatToChar(tmp,rpyRaw.MAGNETIC_RAW_X);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case MAG_Y:
			floatToChar(tmp,rpyRaw.MAGNETIC_RAW_Y);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case MAG_Z:
			floatToChar(tmp,rpyRaw.MAGNETIC_RAW_Z);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case TEMP:
			floatToChar(tmp,rpyRaw.TEMP_RAW);
			forLoop(j,4){msg.data[msg.length++] = tmp[j];}
			break;
		case SOLAR_VOLTAGE:
			longToChar(tmp,(uint32_t)s.Voltage);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
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
			longToChar(tmp,(uint32_t)irData.sensorOne);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case IR_DATA_TWO:
			longToChar(tmp,(uint32_t)irData.sensorTwo);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case IR_DATA_THREE:
			longToChar(tmp,(uint32_t)irData.sensorThree);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
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




void Telemetry::doubleToChar(uint8_t* target, double number){
	char *tmp = (char *) &number;

	for(int i=0;i<8;i++){
		target[i] = tmp[i];
	}
}

double Telemetry::charToDouble(uint8_t* _number){
	double out;

	uint8_t * ptr = (uint8_t *) &out;

	forLoop(i,8){
		ptr[i] = _number[i];
	}

	return out;
}



void Telemetry::floatToChar(uint8_t* target, float number){
	char *tmp = (char *) &number;

	for(int i=0;i<4;i++){
		target[i] = tmp[i];
	}
}

float Telemetry::charToFloat(uint8_t* _number){
	float out;

	uint8_t * ptr = (uint8_t *) &out;

	forLoop(i,4){
		ptr[i] = _number[i];
	}

	return out;
}


void Telemetry::longToChar(uint8_t* _target, uint32_t _number){
	_target[0] = ((_number >> 24) & 0xFF);
	_target[1] = ((_number >> 16) & 0xFF);
	_target[2] = ((_number >> 8) & 0xFF);
	_target[3] = ((_number) & 0xFF);
}

uint32_t Telemetry::charToLong(uint8_t* _number){
	return ((_number[0] << 24 )
			+ (_number[1] << 16)
			+ (_number[2] << 8)
			+ (_number[3] ) );
}


void Telemetry::shortToChar(uint8_t* _target, uint16_t _number){
	_target[0] = _number & 0xFF;
	_target[1] = _number >> 8;
}


uint16_t Telemetry::charToShort(uint8_t* _number){
	return (_number[0] | (_number[1] << 8));
}

