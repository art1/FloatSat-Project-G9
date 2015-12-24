/*
 * irqHandler.cpp
 *
 *  Created on: Dec 6, 2015
 *      Author: arthur
 */
// überschreibt die abgefangenen Interrupts aus RODOS
// warum werden die überhaupt in rodos an den defaulthandler weitergeleitet????!!!!

#include "basic.h"
#include "../Hardware/Camera/Supps/ov7670.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dcmi.h"


//extern "C" void DMA2_Stream1_IRQHandler(){
//	/** TODO do some stuff */
//	// and clear the flags!
//}
//
//extern "C" void DCMI_IRQHandler(){
//
//}


///**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @retval None
//  */
//extern "C" void SysTick_Handler(void)
//{
//  TimingDelay_Decrement();
//} -> defined in hw_timer.cpp

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : EXTI0_IRQHandler
* Description    : This function handles External lines 0 interrupt request.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
extern "C" void EXTI9_5_IRQHandler(void)
{
//	PRINTF("interrupt handler called!\n");
//	test++;
  if ( EXTI_GetITStatus(EXTI_Line7) != RESET )
  {
     if( VSync == 0 )
     {

        FIFO_WE_H();

        VSync = 1;
        FIFO_WE_H();
     }
     else if( VSync == 1 )
     {
          FIFO_WE_L();
          VSync = 2;
     }

	 EXTI_ClearITPendingBit(EXTI_Line7);
	}
}
