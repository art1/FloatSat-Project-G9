/*
 * motor.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: arthur
 */

#include "Motor.h"
//HAL_GPIO hbridgeA_inA(GPIO_036); /* declare HAL_GPIO for GPIO_036 = PC4 (HBRIDGE-A INA pin) */
//HAL_GPIO hbridgeA_inB(GPIO_017); /* declare HAL_GPIO for GPIO_017 = PB1 (HBRIDGE-B INA pin) */
HAL_GPIO hbridge_enable(GPIO_066); /* declare HAL_GPIO for GPIO_066 = PE2 (HBRIDGE Power Enable pin) */

HAL_GPIO dcdc_read(GPIO_067);




HAL_GPIO HBRIDGE_A_INA (GPIO_036);
HAL_GPIO HBRIDGE_A_INB (GPIO_017);

HAL_PWM MotorPWM(PWM_IDX12);



//HAL_PWM pwm(PWM_IDX12); /* declare HAL_PWM for PWM_IDX12 = TIM4-CH1 (HBRIDGE-A), please refer to hal_pwm.h for correct PWM mapping; PB6*/


Motor::Motor() {
	lastDutyCycle = 1;
	dutyCycle = 1; // start motor always in CW directino
	clockwise = true;
	ramp = false;
}

Motor::~Motor() {

}

void Motor::init(){
	hbridge_enable.init(true,1,0);
	MotorPWM.init(5000, 1000);
	HBRIDGE_A_INA.init(true, 1, 0);
	HBRIDGE_A_INB.init(true, 1, 1);
	run();
}

void Motor::run(){
	PRINTF("Starting Motor\n");
	startMotor();
//	suspendCallerUntil(END_OF_TIME);
	//	pwm.write(250);
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
	if(ramp) return -2;
	if(hbridge_enable.readPins() == 0) {
		PRINTF("motor wasnt enabled, starting\n");
		startMotor();
	}

	if((duty < 0) && (lastDutyCycle > 0)){
		switchDirection(duty);
		clockwise = false;
	} else if( (duty > 0 )&&(lastDutyCycle < 0)){
		switchDirection(duty);
		clockwise = true;
	} else MotorPWM.write(duty);


	// rampe fahren falls duty cycle zu stark springt (-> big wheel draws too much current if sudden hcnage in duty cycle)
	if(abs((duty - lastDutyCycle)) >= MOTOR_RAMP_THRESHOLD){
		ramp = true;
		PRINTF("too sudden change in control! current %d controlVal %d\n",duty,lastDutyCycle);
		if(duty < lastDutyCycle){
			spinDownTo(lastDutyCycle,duty);
		} else if(duty > lastDutyCycle){
			spinUpTo(lastDutyCycle,duty);
		}
	}

	lastDutyCycle = duty;
//	if(duty != 0){
//		if((duty < 0) && clockwise){
//			MotorPWM.write(duty);
//
//		} else if ((duty > 0) && (clockwise == false)){
//
//			MotorPWM.write(duty);
//		} else MotorPWM.write(duty);
//	} else stopMotor();

	//	int k = dcdc_read.readPins();
	//	PRINTF("power is: %d\n",k);
	//	k = hbridge_enable.readPins();
	//	PRINTF("should be: %d\n",k);
	//	switchDirection(duty);
	//	pwm.write(abs(duty));
	//	dutyCycle = duty;
	//	MotorPWM.write(duty);
}

int Motor::startMotor(){
	hbridge_enable.setPins(1);
	return 0;
}

int Motor::stopMotor(){
	hbridge_enable.setPins(0);
}

int Motor::switchDirection(int currentSpeed){
	bool tmp = false;

	if(currentSpeed > 100){
		spinDownTo(currentSpeed,100);
		tmp = true;
	}

	//	PRINTF("switching directions\n");
	HBRIDGE_A_INA.setPins(~HBRIDGE_A_INA.readPins());
	HBRIDGE_A_INB.setPins(~HBRIDGE_A_INB.readPins());


	if(currentSpeed > 100) spinUpTo(100,currentSpeed);
	if(tmp) MotorPWM.write(currentSpeed);
}

void Motor::spinDownTo(int _currentSpeed,int _finalVal){
	for(int i=_currentSpeed;i>_finalVal;i--){
		MotorPWM.write(i);
		delayx(1);
	}
	if(ramp)ramp = false;

}

void Motor::spinUpTo(int _currentSpeed,int _finalVal){
	for(int i=_currentSpeed;i<_finalVal;i++){
		MotorPWM.write(i);
		delayx(1);
	}
	if(ramp)ramp = false;
}

