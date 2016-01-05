/*
 * RotationControl.cpp
 *
 *  Created on: Dec 31, 2015
 *      Author: arthur
 */

#include "RotationControl.h"

RotationControl::RotationControl() : Thread("Rotation Control",98,1000){
	active = false;

}

RotationControl::~RotationControl() {
}


void RotationControl::init(){

}

void RotationControl::run(){
	while(1){
		if(!isActive()) suspendCallerUntil(END_OF_TIME);

	}
}

void RotationControl::setRotSpeed(float _speed){
	this->desSpeed = _speed;
}
bool RotationControl::isActive(){
	return active;
}
void RotationControl::setActive(bool _val){
	this->active = _val;
}
