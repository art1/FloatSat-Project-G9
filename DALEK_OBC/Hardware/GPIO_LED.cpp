/*
 * Template.cpp
 *
 * Created on: 25.06.2014
 * Author: Atheel Redah
 *
 */

#include "GPIO_LED.h"


//static Application module01("Template", 2001);

#define LED_GREEN GPIO_060
#define LED_ORANGE GPIO_061
#define LED_RED GPIO_062
#define LED_BLUE GPIO_063

#define BT_UART UART_IDX2
#define USB_UART UART_IDX3

#define IMU_I2C I2C_IDX2

#define RUNNING_SECS	2

HAL_GPIO GreenLED(LED_GREEN);
HAL_GPIO RedLED(LED_RED);
HAL_GPIO BlueLED(LED_BLUE);
HAL_GPIO OrangeLED(LED_ORANGE);

int cnt = 0;
bool wahr = true;

struct SensorData{

};

//Topic<SensorData> SensorDataTopic(-1, "Sensor Data");
//CommBuffer<SensorData> SensorDataBuffer;
//Subscriber SensorDataSubscriber(SensorDataTopic, SensorDataBuffer);






//class Telemetry: public Thread {
////
//public:

GPIO_LED::GPIO_LED(const char* name) : Thread(name) {
	//		PRINTF("huh, my name is, huh, my name is");
	//		PRINTF(name);
}

void GPIO_LED::init() {
	GreenLED.init(true, 1, 0);
	RedLED.init(true,1,0);
	BlueLED.init(true,1,0);
	OrangeLED.init(true,1,0);
}

void GPIO_LED::run() {

	while (1) {
		if(SECONDS_NOW() < RUNNING_SECS){
			switch (cnt) {
			case 0:
				BlueLED.setPins(0);
				RedLED.setPins(0);
				OrangeLED.setPins(0);
				GreenLED.setPins(1);
				cnt++;
				break;
			case 1:
				BlueLED.setPins(0);
				RedLED.setPins(0);
				GreenLED.setPins(0);
				OrangeLED.setPins(1);
				cnt++;
				break;
			case 2:
				BlueLED.setPins(0);
				GreenLED.setPins(0);
				OrangeLED.setPins(0);
				RedLED.setPins(1);
				cnt++;
				break;
			case 3:
				GreenLED.setPins(0);
				RedLED.setPins(0);
				OrangeLED.setPins(0);
				BlueLED.setPins(1);
				cnt = 0;
				break;
			default:
				break;
			}
			//			PRINTF("cnt is %d \r\n",cnt);
			//			PRINTF("Hello Rodos, the time now is %f \r\n",SECONDS_NOW());

			suspendCallerUntil(NOW()+100*MILLISECONDS);
		} else{
			blinkAll(100,1);
			wahr = false;
		}
		if(!wahr){
			//			while(1){
			//				PRINTF("should have initialized IMU now\n\r");
			//				suspendCallerUntil(NOW()+10000*MILLISECONDS);
			//			}
			PRINTF("suspending LEDs...\n\r");
			suspendCallerUntil(END_OF_TIME);
		}
	}
}
/**
 *	on = 1 -> turns on the LED, on=0 turns the led off
 */
void GPIO_LED::switchLED(HAL_GPIO led,int on){
	led.setPins(on);
}

/**
 * blinks all LEDs with given speed and stays on if stayOn = 1,
 */
void GPIO_LED::blinkAll(int speedInMsec, int stayOn){
	for(int i=0;i<11;i++){
		GreenLED.setPins(i%2);
		RedLED.setPins(i%2);
		BlueLED.setPins(i%2);
		OrangeLED.setPins(i%2);
		suspendCallerUntil(NOW()+speedInMsec*MILLISECONDS);
	}
	switchLED(GreenLED,stayOn);
	switchLED(RedLED,stayOn);
	switchLED(BlueLED,stayOn);
	switchLED(OrangeLED,stayOn);
}

void GPIO_LED::crossblink(int speedInMsec, int stayOn){
	for(int i=0;i<11;i++){
		GreenLED.setPins(i%2);
		RedLED.setPins(i%2);
		BlueLED.setPins(i%2+1);
		OrangeLED.setPins(i%2+1);
		suspendCallerUntil(NOW()+speedInMsec*MILLISECONDS);
	}
	switchLED(GreenLED,stayOn);
	switchLED(RedLED,stayOn);
	switchLED(BlueLED,stayOn);
	switchLED(OrangeLED,stayOn);
}
