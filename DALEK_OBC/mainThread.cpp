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

// load all appropriate classes (-> must be public, initializing classes within a class is not working... ?)
GPIO_LED leds("LEDs");
IMU imu;




//RESUMER THREADS
/**************************** GPIO MESSAGES *************************************/
CommBuffer<LED_SWITCH> inputMessageBuffer;
struct GPIO_Receiver :  public Subscriber,  public Thread  {
  GPIO_Receiver() : Subscriber(led_switch,"led switcher") { }

  long put(const long topicId, const long len, const void* data, const NetMsgInfo& netMsgInfo) {
      leds.setNextMode(*(LED_SWITCH*)data);
      leds.resume();                         // not to publish from interrupt, call a thread to do it
      return 1;
   }

  void run () { }
} gpio_receiver_thread;
/**************************** IMU MESSAGES **************************************/

mainThread::mainThread(const char* name) : Thread(name){

}

void mainThread::init(){
	PRINTF("init mainThread\r\n");
}


void mainThread::run(){
//	imu.init();
	imu.setTime(500*MILLISECONDS);
//	imu.setLEDs(&leds);
	while(1){
//		telemetry.run();
//		imu.readIMU_Data();
		suspendCallerUntil(NOW()+5000*MILLISECONDS);
	}
}

