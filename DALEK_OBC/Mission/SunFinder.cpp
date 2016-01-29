/*
 * SunFinder.cpp
 *
 *  Created on: Dec 30, 2015
 *      Author: arthur
 */

#include "SunFinder.h"

SunFinder::SunFinder() : Thread("SunFinder",115, 1000){

	activated = false;
	currentHeadingIndex = 0;
	memset(heading,0,sizeof(heading));
	intCm.sunTM.sunIncidenceAngle = -1.0f;
}

SunFinder::~SunFinder() {

}

void SunFinder::setActive(bool _val){
	this->activated = _val;
}

bool SunFinder::isActive(){
	return activated;
}

int SunFinder::setNewData(IMU_RPY_FILTERED _imu){
	imu = _imu;


}
void SunFinder::setNewData(LUX_DATA _lux){
	luxVal = _lux.LUX;
	bool ok = this->lux.put(_lux.LUX);
	if(!ok) PRINTF("increase Buffer size in sunFinder!\n");
}

float SunFinder::findSunAngle(){
	uint16_t tmpLux = 0;
	uint16_t currentLux = 0;

	int maxLuxAt = 0;
	forLoop(i,this->lux.getElementCount()){
		this->lux.get(tmpLux);
		if( tmpLux > currentLux) {
			currentLux = tmpLux;
			maxLuxAt = i;
		}
	}
	return heading[maxLuxAt];
}

void SunFinder::isStabilized(){
	while(1){
		if((imu.YAW - intCm.sunTM.sunIncidenceAngle) < 5){
			suspendCallerUntil(NOW()+2*SECONDS);
			if((imu.YAW - intCm.sunTM.sunIncidenceAngle) < 5){
				suspendCallerUntil(NOW()+2*SECONDS);
				if((imu.YAW - intCm.sunTM.sunIncidenceAngle) < 5){
					stabilized = true;
					break;
				}
			}
		}
	}
}

void SunFinder::init(){

}

void SunFinder::run(){
	COMMAND_FRAME f;
	f.command = SET_ROTATION_SPEED;
	f.commandValue = 1.0f;
	f.frameNumber = 0.0f;
	f.frameType = CMD;
	f.localTime = 0.0f;
	int16_t tempVal = 0;
	float heading = 0;
	int cnt = 0;
	while(1){
		suspendCallerUntil(END_OF_TIME);
		/** TODO include the communication stuff for Toms telemetry **/
		if(isActive()){
			PRINTF("searching for sun...\n");
			//			commandFrame.publish(f);

			while(1){
				suspendCallerUntil(NOW()+20*MILLISECONDS);
				PRINTF("luxVal %d, heading %f\n",luxVal,imu.YAW);
				if(luxVal > tempVal){
					tempVal = luxVal;
					heading = imu.YAW;
				}
				cnt++;
				if(cnt >= 720) break;
			} // 7 seconds to find
			PRINTF("heading with maxluxval %f\n",heading);

			PRINTF("turning motor off\n");
			f.command = CONTROL_MOTOR;
			f.commandValue = 0.0f;
			commandFrame.publish(f);


			PRINTF("rotation complete, calculating angle...");
			//			float agnel = findSunAngle();
			PRINTF("sun is at %f\n",tempVal);

			intCm.sunTM.sunIncidenceAngle = tempVal;
			intCm.changedVal = SUNFINDER_TM_CHANGED;
			interThreadComm.publish(intCm);

			//			f.command = GOTO_ANGLE;
			//			f.commandValue = tempVal;
			//			commandFrame.publish(f);

			// now wait until satellite is stable more or less
			PRINTF("going to buuuurrrn\n");
//			if ( intCm.sunTM.sunIncidenceAngle)
			while(!(imu.YAW -intCm.sunTM.sunIncidenceAngle) < 10.0){
				PRINTF("current: %f to goto %f\n",imu.YAW,intCm.sunTM.sunIncidenceAngle);

				suspendCallerUntil(NOW()+20*MILLISECONDS);
			}
			stabilized = true;

			PRINTF("Seems to be stabilized\n");
			intCm.knifeData.activated = true;
			intCm.changedVal = KNIFE_CHANGED;
			interThreadComm.publish(intCm);
		}
	}


	threeSixty = false;
	this->setActive(false);
}

