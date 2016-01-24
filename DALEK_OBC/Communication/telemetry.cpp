/*
 * tm.cpp
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#include "telemetry.h"

Telemetry::Telemetry() : Thread("Telemetry",109,2000){
	this->frameNumber = 0;
}

Telemetry::~Telemetry(){
}



void Telemetry::init(){
#ifdef TELEMETRY_ENABLE
	active = true;
#else
	active = false;
#endif
}

void Telemetry::run(){
	/** TODO add function to activate and deactivate telemetry */
	while(1){
		while(!isActive()){
			suspendCallerUntil(END_OF_TIME);
		}
		buildFrame();
		tmPlFrame.publish(msg);
		frameNumber++;
//		PRINTF("sent %d bytes\n",msg.length);
		suspendCallerUntil(NOW()+TM_SAMPLERATE*MILLISECONDS);
	}
}

void Telemetry::setActive(bool _val){
	this->active = _val;
}

bool Telemetry::isActive(){
	return this->active;
}

void Telemetry::setNewData(IMU_RPY_FILTERED _imu){
	this->imu.put(_imu);
}

void Telemetry::setNewData(IMU_DATA_RAW _imuRaw){
	this->imuRaw.put(_imuRaw);
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

void Telemetry::setNewData(CURRENT_DATA _current){
	this->current.put(_current);
}

void Telemetry::setNewData(INTERCOMM _interComm){

}

void Telemetry::setNewData(SUNFINDER_TM _sunTM){
	this->sunTMData.put(_sunTM);
}

void Telemetry::setNewData(ACTIVE_SYSTEM_MODE _mode){
	this->systemMode = _mode;
}

uint32_t Telemetry::getCurrentFrameNumber(){
	return this->frameNumber;
}

/**
 * Builds a Frame with the most recent values of all sensors.
 */
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
	SUNFINDER_TM sunData;
	sunTMData.get(sunData);
	CURRENT_DATA cur;
	current.get(cur);

	PRINTF("yaw %f pitch %f roll %f - IR %f %f %f - LUX %d - "
			"SolVolt %f - BattVolt %f - Current %f\n",rpy.YAW,rpy.PITCH,rpy.ROLL,
			irData.sensorOne,irData.sensorTwo,irData.sensorThree,
			l.LUX,s.Voltage,cur.batteryVoltage,cur.batteryCurrent);


	for(int i=-1;i<15;i++){
		switch (i) {
		case -1:
			forLoop(j,3){
				msg.data[msg.length++] = FRAME_START;
			} // adding $$$
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
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
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
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case GYRO_Y:
			floatToChar(tmp,rpyRaw.ANGULAR_RAW_Y);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case GYRO_Z:
			floatToChar(tmp,rpyRaw.ANGULAR_RAW_Z);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case ACCL_X:
			floatToChar(tmp,rpyRaw.ACCEL_RAW_X);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case ACCL_Y:
			floatToChar(tmp,rpyRaw.ACCEL_RAW_Y);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case ACCL_Z:
			floatToChar(tmp,rpyRaw.ACCEL_RAW_Z);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case MAG_X:
			floatToChar(tmp,rpyRaw.MAGNETIC_RAW_X);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case MAG_Y:
			floatToChar(tmp,rpyRaw.MAGNETIC_RAW_Y);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case MAG_Z:
			floatToChar(tmp,rpyRaw.MAGNETIC_RAW_Z);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case TEMP:
			floatToChar(tmp,rpyRaw.TEMP_RAW);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case SOLAR_VOLTAGE:
			longToChar(tmp,(uint32_t)s.Voltage);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case BATTERY_VOLTAGE:
			/** TODO Battery Voltage */
			// adding dummy values here
			forLoop(j,4){
				msg.data[msg.length++] = 0x00;
			}
			break;
		case CURRENT:
			/** TODO CURRENT Telemetry*/
			// adding dummy values here
			forLoop(j,4){
				msg.data[msg.length++] = 0x00;
			}
			break;
		case MOTOR_SPEED:
			/** TODO Motor SPeed*/
			// adding dummy values here
			forLoop(j,4){
				msg.data[msg.length++] = 0x00;
			}
			break;
		case IR_DATA_ONE:
			floatToChar(tmp,irData.sensorOne);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case IR_DATA_TWO:
			floatToChar(tmp,irData.sensorTwo);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case IR_DATA_THREE:
			floatToChar(tmp,irData.sensorThree);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case GS_SUNFINDER:
			msg.data[msg.length++] = sunData.currentProgress;
			break;
		case SUN_INCIDENCE_ANGLE:
			floatToChar(tmp,sunData.sunIncidenceAngle);
			forLoop(j,4){
				msg.data[msg.length++] = tmp[j];
			}
			break;
		case TM_LOCALTIME:
			longLongToChar(tmp,(uint64_t)NOW());
			forLoop(j,8){
				msg.data[msg.length++] = tmp[i];
			}
			break;
		default:
			PRINTF("oO this should never ever happen :o\n");
			break;
		}
		//		msg.data[msg.length++] = VALUE_SEPERATOR;
	}
	forLoop(j,3){msg.data[msg.length++] = FRAME_END;}
//	PRINTF("added %d bytes",msg.length);
}


void Telemetry::sendPayload(CAM_DATA _camData){
	uint8_t tmp[2];
	msg.length = 0;
	memset(msg.data,0,sizeof(msg.data));

	for(int i=-1; i< 6;i++){
		switch (i) {
		case -1:
			forLoop(j,3){msg.data[msg.length++] = FRAME_START;}// adding $$$
			break;
		case PL_FRAMETYPE:
			msg.data[msg.length++] = PL;
			break;
		case PL_FRAMENUMBER:
			longToChar(tmp,getCurrentFrameNumber());
			forLoop(j,4) msg.data[msg.length++] = tmp[j];
			break;
		case PAYLOAD_NUMBER:
			longToChar(tmp,_camData.consecutiveFrame);
			forLoop(j,4) msg.data[msg.length++] = tmp[j];
			break;
		case PAYLOAD_SIZE:
			msg.data[msg.length++] = ((_camData.width * _camData.height )/ 100);
			break;
		case PAYLOAD:
			forLoop(i,100){
				uint16_t *p = _camData.picture;
				shortToChar(tmp,p[i]);
				forLoop(j,2) msg.data[msg.length++] = tmp[j];
			}
			break;
		case PL_LOCALTIME:
			longLongToChar(tmp,(uint64_t)NOW());
			forLoop(j,8){
				msg.data[msg.length++] = tmp[i];
			}
			break;
		default:
			break;
		}
	}
	forLoop(j,3){msg.data[msg.length++] = FRAME_END;}
//	PRINTF("added %d bytes",msg.length);

	tmPlFrame.publish(msg);
	frameNumber++;
}






