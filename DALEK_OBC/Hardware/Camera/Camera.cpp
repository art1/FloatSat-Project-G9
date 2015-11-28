/*
 * camera.cpp
 *
 *  Created on: Nov 28, 2015
 *      Author: arthur
 */

#include "Camera.h"


SCCB sccb;

Camera::Camera() {
	// TODO Auto-generated constructor stub

}

Camera::~Camera() {
	// TODO Auto-generated destructor stub
}

void Camera::init(){
	// init clock -> PA8
	 GPIO_InitTypeDef GPIO_InitStructure;

	 RCC_ClockSecuritySystemCmd(ENABLE);

	 /* Enable GPIOs clocks */
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	 GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	 /* Configure PA8 */
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);

	 RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);

	 sccb.init();
}

void Camera::run(){

}
