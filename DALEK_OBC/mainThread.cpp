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
//GPIO_LED leds("LEDs");
#ifdef IMU_ENABLE
IMU imu;
#endif
#ifdef TTNC_ENABLE
TTnC ttnc;
#endif



//RESUMER THREADS
///**************************** GPIO MESSAGES *************************************/
//CommBuffer<LED_SWITCH> inputMessageBuffer;
//struct GPIO_Receiver :  public Subscriber,  public Thread  {
//  GPIO_Receiver() : Subscriber(led_switch,"led switcher") { }
//
//  long put(const long topicId, const long len, const void* data, const NetMsgInfo& netMsgInfo) {
//      leds.setNextMode(*(LED_SWITCH*)data);
//      leds.resume();                         // not to publish from interrupt, call a thread to do it
//      return 1;
//   }
//
//  void run () { }
//} gpio_receiver_thread;
/**************************** IMU MESSAGES **************************************/

mainThread::mainThread(const char* name) : Thread(name){

}

void mainThread::init(){
	PRINTF("init mainThread\r\n");

	//enable GPIOD channel clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIODEN,ENABLE);
	// init LEDs -> not using rodos because reasons
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = LED_GREEN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		// TBD what speed to use????
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = LED_ORANGE;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = LED_RED;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = LED_BLUE;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
}


void mainThread::run(){
//	imu.init();
#ifdef IMU_ENABLE
	imu.setTime(500*MILLISECONDS);
#endif
//	imu.setLEDs(&leds);
	while(1){
//		imu.readIMU_Data();
		suspendCallerUntil(NOW()+5000*MILLISECONDS);
	}
}

