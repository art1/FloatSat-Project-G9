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
 * @file params.h
 * @date 2012/08/20 16:13
 * @author Michael Ruffer
 *
 * Copyright 2012 University Wuerzburg
 *
 * @brief configuration parameters ...
 *
 */

#ifndef __PARAMS_H__
#define __PARAMS_H__

#ifdef __cplusplus
#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif
#endif

/** Version Number */
#define OSVERSION "STM32F4 Cortex-M4 V1.0"

/*************** System Configuration *********/

/** system clocks
 * - APB1_CLK is fixed to CPU_CLK/4 (TIM2/3/4/5/6/7/12/13/14, USART2/3, UART4/5, SPI2/3, I2C1/2/3, CAN1/2, DAC1/2, WWDG)
 * - APB2_CLK is fixed to CPU_CLK/2 (TIM1/8/9/10/11, USART1/6, SPI1, ADC1/2/3, MMC)
 * - for more information see system_stm32f4xx.c
 */
/** cpu clock */
#ifdef STM32F40_41xxx
#define CPU_CLK							168000000
#endif
#ifdef STM32F429_439xx
#define CPU_CLK							180000000
#endif

/** the following define sets the UART used for debug outputs with xprintf
 *
 */
#define UART_DEBUG						UART_IDX2

/** Memory for allocation (xmalloc) e.g. for all thread stacks ***/
#define XMALLOC_SIZE					40*1024

/** default stack size (in bytes) for threads */
#define DEFAULT_STACKSIZE				2000

/** stack size (in bytes) for scheduler
 * - ISRs and Scheduler are using the main stack (MSP)
 * - The array "char __schedulerStack__[SCHEDULER_STACKSIZE];" in scheduler.cpp is not used!
 * - So we must provide enough stack in the linker script (see "_Min_Stack_Size" in stm32_flash.ld)!
 */
#define SCHEDULER_STACKSIZE 			0

/** time interval between timer interrupts in microseconds - max. 798000us (if CPU_CLK==168MHz)!!! */
#define PARAM_TIMER_INTERVAL			10000

/*** time for time slice to swtich between threads with same priority ***/
#define  TIME_SLICE_FOR_SAME_PRIORITY	(100*MILLISECONDS)

/** default priority for newly created threads */
#define DEFAULT_THREAD_PRIORITY			100

/** user threads shall not have a priority higher than this */
#define MAX_THREAD_PRIORITY				1000

/** high priority levels for priority ceiling  */
#define NETWORKREADER_PRIORITY			(MAX_THREAD_PRIORITY + 2)
/** high priority levels for priority ceiling  */
#define CEILING_PRIORITY				(NETWORKREADER_PRIORITY + 1)

/** using a network, the maximal number of nodes attached */
#define MAX_NUMBER_OF_NODES				10
/** if using network it may have a limit for pakets, eg udp 1500 */
#define MAX_NETWORK_MESSAGE_LENGTH		1300

/*** If using a network: Maximal number of subscribers per node */
#define MAX_SUBSCRIBERS					60


// Declare global variables with this to put them into the 64k-CoreCoupled Memory Block
// No DMA is possible. No initialization is possible,all gets set to zero.
#define PUT_INTO_CCRAM __attribute__ ((section (".bss_ccram")))

#ifdef __cplusplus
#ifndef NO_RODOS_NAMESPACE
}
#endif
#endif

#endif /* __PARAMS_H__ */
