/*
 * mainThread.cpp
 *
 *  Created on: Oct 31, 2015
 *      Author: arthur
 */

//#include "Template.h"
#include "mainThread.h"
//#include "Communication/spiconnector.h"
static Application mainThread("mainThread",20);

// load all appropriate thingies
Telemetry telemetry("test");

//extern Telemetry telemetry("test");
IMU imu;

mainThread::mainThread(const char* name) : Thread(name){

}

void mainThread::init(){
	PRINTF("init mainThread\r\n");
}


void mainThread::run(){
	imu.init();
	while(1){
//		telemetry.run();
		imu.readIMU_Data();
		suspendCallerUntil(NOW()+500*MILLISECONDS);
	}
}

