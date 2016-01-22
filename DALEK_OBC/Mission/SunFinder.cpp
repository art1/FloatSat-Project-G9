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
	if(currentHeadingIndex >= SUNFINDER_BUFFERSIZE){
		PRINTF("increase Buffer size in sunFinder!\n");
		return -1;
	} else {
		heading[currentHeadingIndex++] = _imu.YAW;
		if(currentHeadingIndex == 0){
			initPos = heading[0];
			threeSixty = false;
		}else{
			tempPos += (heading[currentHeadingIndex-1] - initPos);
			if( tempPos > 360.0)threeSixty = true;
		}
		return 1;
	}

}
void SunFinder::setNewData(LUX_DATA _lux){
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


void SunFinder::init(){

}

void SunFinder::run(){
	while(1){
		suspendCallerUntil(END_OF_TIME);
		/** TODO include the communication stuff for Toms telemetry **/
		if(isActive()){
			/** TODO sunFinder Thread */
			PRINTF("searching for sun...\n");
			while(currentHeadingIndex < 2){
				Delay_millis(10);
			}
			/** TODO set Motor to rotate */
			while(!threeSixty){
				suspendCallerUntil(END_OF_TIME);
			}
			float agnel = findSunAngle();
			/** TODO set Angle for Motor! */
			threeSixty = false;
			this->setActive(false);
		}
	}
}
