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
 * @file hw_specific.cc
 * @date 2008/04/23 7:33
 * @author Johanes Freitag, Sergio Montenegro
 *
 * Uni Wuerzburg
 *
 * @brief all hardware specific stuff I have no better place for ...
 *
 */

#include "hw_specific.h"
#include "rodos.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sched.h>

#include "hal/hal_uart.h"

#include <stdio.h>
#include <ucontext.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <signal.h>

namespace RODOS {

extern "C" {
void abort();
void exit(int c);
}

volatile long taskRunning = 0;

ucontext_t * volatile contextT;

#define STACKSIZE 4096              /* stack size */
ucontext_t signal_context;          /* the interrupt context */
void *signal_stack;                 /* global interrupt stack */

/**
 * initialize hardware specific stuff
 */

extern void sigtermHandler(int sig);
extern long myNodeNr;

void hwInit() {
	taskRunning = 0;

	/* External interrupts simulator */
	signal(SIGTERM, sigtermHandler);

	/** best effort to determine the node noumber (not perfect or even ok!).
	 This number may be changed by the gateway link interface
	 according to other chriteria **/
	// myNodeNr = (getpid() % (MAX_NUMBER_OF_NODES -1) ) + 1; ORIGINAL
	myNodeNr = getpid(); // a default node number, may be replaced by the link interface (gateway)

}

/**
 *  abort execution
 */
void hwResetAndReboot() {
	xprintf("hw_resetAndReboot() -> exit");
	abort();
}


/**
  *  global interrupt disable
  */

sigset_t oldset;

void hwDisableInterrupts(){
	sigset_t newset;
	sigfillset(&newset);

	sigprocmask(SIG_SETMASK,&newset,&oldset);
}

/**
  *  global interrupt enable
  */
void hwEnableInterrupts(){
	sigprocmask(SIG_SETMASK,&oldset,0);
}
static long long timeOfLastTriggerWatchdog = 0;
void hwInitWatchdog(long intervalMilliseconds) {
}

void hwTriggerWatchdog() {
	timeOfLastTriggerWatchdog = NOW();
}

long* hwInitContext(long* stack, void* object) {

	contextT = (ucontext_t*) xmalloc(sizeof(ucontext_t));

	getcontext(contextT);
	contextT->uc_stack.ss_sp = stack;
	contextT->uc_stack.ss_size = sizeof stack;
	contextT->uc_stack.ss_flags = 0;
	if (sigemptyset(&contextT->uc_sigmask) < 0) {
		perror("sigemptyset");
		exit(1);
	}
	makecontext(contextT, (void (*)())threadStartupWrapper, 1, object);

	return (long*) contextT;
}

/******************************************/
// extern "C" int sched_yield();
void sp_partition_yield() { /*sched_yield();*/
}

void startIdleThread() {
}

void __asmSaveContextAndCallScheduler() {

	   /* Create new scheduler context */
	   getcontext(&signal_context);
	   signal_context.uc_stack.ss_sp = signal_stack;
	   signal_context.uc_stack.ss_size = STACKSIZE;
	   signal_context.uc_stack.ss_flags = 0;
	   sigemptyset(&signal_context.uc_sigmask);
	   makecontext(&signal_context, (void (*)())schedulerWrapper, 1,contextT);

	   /* save running thread, jump to scheduler */
	   swapcontext(contextT,&signal_context);

}

void FFLUSH() {
	fflush(stdout);
}

void __asmSwitchToContext(long* context) {
	contextT = (ucontext_t*) context;
	setcontext(contextT); /* go */
}

}

