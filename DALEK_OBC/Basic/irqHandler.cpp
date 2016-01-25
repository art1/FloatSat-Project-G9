/*
 * irqHandler.cpp
 *
 *  Created on: Dec 6, 2015
 *      Author: arthur
 */
// überschreibt die abgefangenen Interrupts aus RODOS
// warum werden die überhaupt in rodos an den defaulthandler weitergeleitet????!!!!

#include "irqHandler.h"

extern "C" Camera camera;

int count = 0;
int cnt = 0;
int start = 1;


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
	test++;
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


//extern "C" void DMA1_Stream6_IRQHandler(void){
//	xprintf("Picture IT!\n");
//	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET){
//		//xprintf("Frame send! -> Continue with Payload\n");
//		DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF6);
//	}
//}

extern "C" void DMA2_Stream1_IRQHandler(void) {
	static int K;
	//Test on DMA2 Channel1 Transfer Complete interrupt
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) == SET) {
		//xprintf("Frame Complete, detecting Target...\n");
		camera.sendImage();
		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);
	}
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TEIF1) == SET) {
		xprintf("DMA Error\n");
		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TEIF1);
	}

}

extern "C" void DCMI_IRQHandler(void) {
	if(DCMI_GetITStatus(DCMI_IT_VSYNC) == SET)
	{
		if(start == 0)
		{
			start = 1;
		}
		else
		{
			start = 0;
		}
		//xprintf("VSYNC");
		DCMI_ClearFlag(DCMI_IT_VSYNC);
	}
	else if(DCMI_GetITStatus(DCMI_IT_LINE) == SET)
	{
		if(start == 1)
		{
			count++;
		}
		else
		{
			if(count != 0)
			{
				//xprintf("count: %d \n\n", count); //just for counting the number of line
			}
			count = 0;
		}
		DCMI_ClearFlag(DCMI_IT_LINE);
	}
	else if(DCMI_GetITStatus(DCMI_IT_FRAME) == SET){
		//xprintf("FRAME\n");
		//sendPic = 1;

		DCMI_ClearFlag(DCMI_IT_FRAME);
	}
	else if(DCMI_GetITStatus(DCMI_IT_ERR)== SET){
		xprintf("DCMI FLAG ERROR\n");
		DCMI_ClearFlag(DCMI_IT_ERR);
	}
	else if(DCMI_GetITStatus(DCMI_IT_OVF) == SET){
		xprintf("OVERFLOW\n");
		DCMI_ClearFlag(DCMI_IT_OVF);
	}
}

