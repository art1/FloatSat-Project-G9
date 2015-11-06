/*
 * Template.cpp
 *
 * Created on: 25.06.2014
 * Author: Atheel Redah
 *
 */

#include "Template.h"


static Application module01("Template", 2001);

#define LED_GREEN GPIO_060
#define LED_ORANGE GPIO_061
#define LED_RED GPIO_062
#define LED_BLUE GPIO_063

#define BT_UART UART_IDX2
#define USB_UART UART_IDX3

#define IMU_I2C I2C_IDX2

#define RUNNING_SECS	3

HAL_GPIO GreenLED(LED_GREEN);
HAL_GPIO RedLED(LED_RED);
HAL_GPIO BlueLED(LED_BLUE);
HAL_GPIO Orange(LED_ORANGE);

int cnt = 0;
bool wahr = true;

struct SensorData{

};

Topic<SensorData> SensorDataTopic(-1, "Sensor Data");
CommBuffer<SensorData> SensorDataBuffer;
Subscriber SensorDataSubscriber(SensorDataTopic, SensorDataBuffer);






//class Telemetry: public Thread {
////
//public:

	Telemetry::Telemetry(const char* name) : Thread(name) {
//		PRINTF("huh, my name is, huh, my name is");
//		PRINTF(name);
	}

	void Telemetry::init() {
		GreenLED.init(true, 1, 0);
		RedLED.init(true,1,0);
		BlueLED.init(true,1,0);
		Orange.init(true,1,0);
		PRINTF("should now init imu\n\r");

	}

	void Telemetry::run() {

		while (1) {
			if(SECONDS_NOW() < RUNNING_SECS){
			switch (cnt) {
				case 0:
					BlueLED.setPins(0);
					RedLED.setPins(0);
					Orange.setPins(0);
					GreenLED.setPins(1);
					cnt++;
					break;
				case 1:
					BlueLED.setPins(0);
					RedLED.setPins(0);
					GreenLED.setPins(0);
					Orange.setPins(1);
					cnt++;
					break;
				case 2:
					BlueLED.setPins(0);
					GreenLED.setPins(0);
					Orange.setPins(0);
					RedLED.setPins(1);
					cnt++;
					break;
				case 3:
					GreenLED.setPins(0);
					RedLED.setPins(0);
					Orange.setPins(0);
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
				wahr = false;
				for(int i=0;i<11;i++){
					GreenLED.setPins(i%2);
					RedLED.setPins(i%2);
					BlueLED.setPins(i%2);
					Orange.setPins(i%2);
					suspendCallerUntil(NOW()+100*MILLISECONDS);
				}
			}
			if(!wahr){
			GreenLED.setPins(1);
			RedLED.setPins(1);
			Orange.setPins(1);
			BlueLED.setPins(1);
//			while(1){
//				PRINTF("should have initialized IMU now\n\r");
//				suspendCallerUntil(NOW()+10000*MILLISECONDS);
//			}
			PRINTF("suspending LEDs...\n\r");
			suspendCallerUntil(END_OF_TIME);
			}
		}
	}
//};
//Telemetry Telemetry("Telemetry");

/***********************************************************************/
