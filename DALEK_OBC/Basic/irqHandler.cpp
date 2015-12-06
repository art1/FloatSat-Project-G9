/*
 * irqHandler.cpp
 *
 *  Created on: Dec 6, 2015
 *      Author: arthur
 */
// überschreibt die abgefangenen Interrupts aus RODOS
// warum werden die überhaupt in rodos an den defaulthandler weitergeleitet????!!!!

#include "basic.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dcmi.h"


extern "C" void DMA2_Stream1_IRQHandler(){
	/** TODO do some stuff */
	// and clear the flags!
}

extern "C" void DCMI_IRQHandler(){

}
