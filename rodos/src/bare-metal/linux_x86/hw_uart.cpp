

/*********************************************************** Copyright 
 **
 ** Copyright (c) 2008, German Aerospace Center (DLR)
 ** All rights reserved.
 ** 
 ** Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 ** 
 ** 1 Redistributions of source code must retain the above copyright
 **   notice, this list of conditions and the following disclaimer.
 ** 
 ** 2 Redistributions in binary form must reproduce the above copyright
 **   notice, this list of conditions and the following disclaimer in the
 **   documentation and/or other materials provided with the
 **   distribution.
 ** 
 ** 3 Neither the name of the German Aerospace Center nor the names of
 **   its contributors may be used to endorse or promote products derived
 **   from this software without specific prior written permission.
 ** 
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **  
 ****************************************************************************/


/**
 * @file hw_uart.cc
 * @date 2008/12/03 10:03
 * @author Lutz Dittrich, Sergio Montenegro
 *
 * Copyright 2008 DLR
 *
 * @brief UART communication
 *
 */


#include "rodos.h"
#include "hw_uart.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>// for interrupt
#include "gateway/linkinterfaceuart.h"

extern LinkinterfaceUART linkinterfaceUart;
extern int fd;
namespace RODOS {



const char *uartDeviceNames[3] = {"/dev/ttyUSB0","/dev/ttyUSB1","/dev/rfcomm0"};



UART::UART(int uart_num) {
	struct termios t; ///< control structure for a general asynchronous interface

	charReady = false;
	lastChar  = 0;

	const char* devname = uartDeviceNames[uart_num];

	/* prepare termios */
	t.c_iflag = IGNBRK | IGNPAR;
	t.c_oflag = 0;
	t.c_cflag = CS8 | CREAD;// | CLOCAL;
	t.c_lflag = 0;
	t.c_cc[VMIN] = 1; //TW:/* wait for 1 byte */
	t.c_cc[VTIME] = 0; //TW:/* turn off timer */
	cfsetispeed(&t,B115200); /* normal shall be: B115200 Baud */
	cfsetospeed(&t,B115200); /* TODO Shouldn't the speed not be read from a configuration file? */

	fd = open(devname,O_RDWR  | O_NDELAY);
	if (fd==-1) {
		xprintf("UART: cannot open file: %s\n",devname);
		exit(1);
	}


	tcsetattr(fd,TCSANOW, &t);

}



UART::~UART() {
	close(fd);
}


//static int notUsed;
void UART::writechar(const char c) {
	int cnt=0;
	while((write(fd,&c,1) < 1) && (cnt < 1000000)){cnt++;}
}


/**
 * Check if a new value is available. Since this requires a read operation
 * under Linux, the character read during the operation needs to be
 * buffered for the next call.
 */
bool UART::isCharReady() {

	if(charReady) return true;

	charReady = getcharNoWait(lastChar);
	return charReady;
}

/**
 * Returns the latest available character.
 * In case a check for new data has been made, the character is buffered in lastChar
 * and can be used directly.
 */
bool UART::getcharNoWait(char &c) {
	if(charReady) {
		charReady = false;
		c = lastChar;
		return true;
	}

	char buf[1];
	if (read(fd,buf,1)!=1) return false;
	c = buf[0];
	return true;
}

bool UART::putGeneric(const unsigned int len, const void* msg) {
	unsigned int ctr;
	for (ctr = 0; ctr<len; ctr++) {
		writechar(((const char*)msg)[ctr]);
	}
	return true;
}




} //namespace rodos

