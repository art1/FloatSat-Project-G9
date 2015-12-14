/*
 * DCMI.cpp
 *
 *  Created on: Dec 6, 2015
 *      Author: arthur
 */

#include "myDCMI.h"

myDCMI::myDCMI(uint32_t imageSize, uint32_t dmaMemoryAddress, uint16_t captureRate, uint16_t captureMode) {
	imgSize = imageSize;
	dmaMemAddr = dmaMemoryAddress;
	captRate = captureRate;
	captMode = captureMode;
}

void myDCMI::InitGPIO() {
		GPIO_InitTypeDef GPIO_InitStructure;

		/*** Configures the DCMI GPIOs to interface with the OV/670 camera module ***/
		/* Enable DCMI GPIOs clocks */
		RCC_AHB1PeriphClockCmd(
				RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
						| RCC_AHB1Periph_GPIOE, ENABLE);

		/* Connect DCMI pins to AF13 -------------------------------------------*/
		/* DCMI_PIXCLK -> PA6*/
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI );
		/* DCMI_HSYNC -> PA4*/
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI );
		/* DCMI_VSYNC -> PB7*/
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI );
		/* DCMI_D0 -> PC6*/
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI );
		/* DCMI_D1 -> PC7*/
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI );
		/* DCMI_D2 -> PE0*/
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI );
		/* DCMI_D3 -> PE1*/
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI );
		/* DCMI_D4 -> PE4*/
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI );
		/* DCMI_D5 -> PB6*/
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI );
		/* DCMI_D6 -> PE5*/
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI );
		/* DCMI_D7 -> PE6*/
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI );

		/* DCMI GPIO configuration ---------------------------------------------*/
		/* PCLK(PA6),  HSYNC(PA4) ----------------------------------------------*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* D5(PB6), VSYNC(PB7) --------------------------------------------------*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* D0..D1(PC6/7) -------------------------------------------------------*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		/* D2..D4(PE0/1/4) D6..D7(PE5/6) ---------------------------------------*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
				| GPIO_Pin_5 | GPIO_Pin_6;
		GPIO_Init(GPIOE, &GPIO_InitStructure);

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO );
		/* Output clock on MCO pin ---------------------------------------------*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);
	//	RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_2);
	//	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);
	}


void myDCMI::InitDCMI() {

	DCMI_InitTypeDef DCMI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DCMI_CROPInitTypeDef DCMI_CROPInitStructure;

	/*** Configures the DCMI to interface with the OV7670 camera module ***/
	/* Enable DCMI clock */
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	DCMI_DeInit();
	/* DCMI configuration */
	DCMI_InitStructure.DCMI_CaptureMode = captMode; //snapshot
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_High;
	DCMI_InitStructure.DCMI_CaptureRate = captRate;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;

	/* Configures the DMA2 to transfer Data from DCMI */
	/* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	/* DMA2 Stream1 Configuration */
	DMA_DeInit(DMA2_Stream1);

	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&DCMI->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) dmaMemAddr;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = imgSize / 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	/* DCMI configuration */
	DCMI_Init(&DCMI_InitStructure);

	/* DMA2 IRQ channel Configuration */
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA2_Stream1, DMA_IT_TE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);




}

void myDCMI::EnableDCMI(){
    DCMI_ITConfig(DCMI_IT_VSYNC, ENABLE);
    DCMI_ITConfig(DCMI_IT_LINE, ENABLE);
    DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
    DCMI_ITConfig(DCMI_IT_OVF, ENABLE);
    DCMI_ITConfig(DCMI_IT_ERR, ENABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
}



void myDCMI::DisableDCMI(){
	DCMI_CaptureCmd(DISABLE);
	DCMI_Cmd(DISABLE);
	DMA_Cmd(DMA2_Stream1, DISABLE);
	DCMI_CROPCmd(DISABLE);
	DCMI_ITConfig(DCMI_IT_FRAME, DISABLE);
}
