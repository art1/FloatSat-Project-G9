/*
 * motor.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: arthur
 */

#include "Motor.h"
HAL_GPIO hbridgeA_inA(GPIO_036); /* declare HAL_GPIO for GPIO_036 = PC4 (HBRIDGE-A INA pin) */
HAL_GPIO hbridgeA_inB(GPIO_017); /* declare HAL_GPIO for GPIO_017 = PB1 (HBRIDGE-B INA pin) */
HAL_GPIO hbridge_enable(GPIO_066); /* declare HAL_GPIO for GPIO_066 = PE2 (HBRIDGE Power Enable pin) */

HAL_GPIO dcdc_read(GPIO_067);

HAL_PWM pwm(PWM_IDX12); /* declare HAL_PWM for PWM_IDX12 = TIM4-CH1 (HBRIDGE-A), please refer to hal_pwm.h for correct PWM mapping; PB6*/


Motor::Motor() {
	// TODO Auto-generated constructor stub
	dutyCycle = 0;
}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

void Motor::init(){
	hbridgeA_inA.init(true,1,1);
	hbridgeA_inB.init(true,1,0);
	hbridge_enable.init(true,1,0);
	dcdc_read.init(false,1,0);
	pwm.init(5000,1000); // 84Hz
	pwm.write(250);

}

void Motor::run(){
	PRINTF("Starting Motor\n");
	startMotor();
	pwm.write(250);
//	int cnt = 0;
//	while(1){
//		setspeed(cnt++);
////		suspendCallerUntil(NOW()+100*MILLISECONDS);
//		PRINTF("speed is now %d\n",cnt);
//		ORANGE_TOGGLE;
////		cnt++;
//		if(cnt > 999){
//			cnt = -1000;
//		}
//	}
}


int Motor::setspeed(int16_t duty){
	if(hbridge_enable.readPins() == 0) {
		PRINTF("motor wasnt enabled, starting\n");
		startMotor();
	}
	int k = dcdc_read.readPins();
	PRINTF("power is: %d\n",k);
	k = hbridge_enable.readPins();
	PRINTF("should be: %d\n",k);
	switchDirection(duty);
	pwm.write(abs(duty));
	dutyCycle = duty;
}

int Motor::startMotor(){
	hbridge_enable.setPins(1);
	return 0;
}

int Motor::stopMotor(){
	hbridge_enable.setPins(0);
}

int Motor::switchDirection(int16_t dir){
//	PRINTF("switching directions\n");
	if( dir < 0){
		hbridgeA_inA.setPins(0);
		hbridgeA_inB.setPins(1);
	} else {
		hbridgeA_inA.setPins(1);
		hbridgeA_inB.setPins(0);
	}
}

