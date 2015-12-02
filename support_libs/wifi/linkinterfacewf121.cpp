/*
 * linkinterfacewf121.cpp
 *
 *  Created on: 31.08.2015
 *      Author: tmikschl
 */

#include "linkinterfacewf121.h"
UDPMsg txMsg;
UDPMsg rxMsg;

LinkinterfaceWF121::LinkinterfaceWF121(WF121 *_wf121) : Linkinterface(-1){
	wf121 = _wf121;
}

void LinkinterfaceWF121::init() {
	wf121->setIoEventReceiver(this);
}

void LinkinterfaceWF121::onWriteFinished() {
    if(threadToResume) threadToResume->resume();
}

void LinkinterfaceWF121::onDataReady() {
    if(threadToResume) threadToResume->resume();
}

bool LinkinterfaceWF121::sendNetworkMsg(NetworkMessage &outMsg)	{
	txMsg.length=outMsg.numberOfBytesToSend();

	memcpy(txMsg.data,&outMsg,txMsg.length);
	wf121->write(&txMsg);

    return true;
}




bool LinkinterfaceWF121::getNetworkMsg(NetworkMessage &inMsg,int32_t &numberOfReceivedBytes) {
	if(wf121->read(&rxMsg)) {
		memcpy(&inMsg,rxMsg.data,rxMsg.length);
		numberOfReceivedBytes = rxMsg.length;
		return true;
	}
	return false;
}
