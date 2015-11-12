/*
 * Template.cpp
 *
 * Created on: 25.06.2014
 * Author: Atheel Redah
 *
 */

#include "GPIO_LED.h"

static Application receiverName("GPIO_LED", 2001);

//static Fifo<LED_SWITCH,100> buffer;

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




GPIO_LED::GPIO_LED(const char* name) : Thread(name) {
	//		PRINTF("huh, my name is, huh, my name is");
	//		PRINTF(name);
}


void GPIO_LED::init() {
	GreenLED.init(true, 1, 0);
	RedLED.init(true,1,0);
	BlueLED.init(true,1,0);
	OrangeLED.init(true,1,0);
	this->ledMode.COMMAND = -1;
	this->ledMode.BLUE = 0;
	this->ledMode.ORANGE = 0;
	this->ledMode.RED = 0;
	this->ledMode.GREEN = 0;
}

void GPIO_LED::run() {

	while (1) {
			PRINTF("suspending gpio thread...\n");
			suspendCallerUntil();
			PRINTF("received a message!\n");
			switch (this->ledMode.COMMAND) {
				case 0:
					runAround(100,200,0);
					break;
				case 1:
					blinkAll(100,0);
					break;
				case 2:
					switchLED(RedLED,this->ledMode.RED);
					switchLED(GreenLED,this->ledMode.GREEN);
					switchLED(BlueLED,this->ledMode.BLUE);
					switchLED(OrangeLED,this->ledMode.ORANGE);
					break;
				default:
					break;
			}
//		}
	}
}

void GPIO_LED::setNextMode(LED_SWITCH led){
	this->ledMode = led;
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

void GPIO_LED::runAround(int speedInMsec,int maxduration,int stayOn){
	int time = NOW();
	while(time < maxduration){
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

			suspendCallerUntil(NOW()+speedInMsec*MILLISECONDS);
	}
	cnt = 0;
	switchLED(GreenLED,stayOn);
	switchLED(RedLED,stayOn);
	switchLED(BlueLED,stayOn);
	switchLED(OrangeLED,stayOn);
}
