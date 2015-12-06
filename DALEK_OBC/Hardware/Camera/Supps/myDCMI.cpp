/*
 * DCMI.cpp
 *
 *  Created on: Dec 6, 2015
 *      Author: arthur
 */

#include "myDCMI.h"

myDCMI::myDCMI() {
	// TODO Auto-generated constructor stub

}

myDCMI::~myDCMI() {
	// TODO Auto-generated destructor stub
}

void myDCMI::init(){
	DCMI_InitTypeDef DCMI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;




	// init all DCMI GPIO stuff and alternative functions -> neable GPIO clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);

	//PA4 -> HREF ; DCMI alternative function!
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource4,GPIO_AF_DCMI);
	// PA6 -> PCLK
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_DCMI);

	// PB7 -> VSYNC
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_DCMI);
	// PB6 -> D5
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_DCMI);

	// PC6 -> D0
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_DCMI);
	// PC7 -> D1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_DCMI);

	// PE0 -> D2
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource0,GPIO_AF_DCMI);
	// PE1 -> D3
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource1,GPIO_AF_DCMI);
	// PE4 -> D4
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource4,GPIO_AF_DCMI);
	// PE5 -> D6
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_DCMI);
	// PE6 -> D7
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_DCMI);

	// init the gpiostructs
	//channel A
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// channel B
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// channel C
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// channel E
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);



	// GPIO init fertig, jetzt DCMI structure
	// init clock to DCMI
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	DCMI_DeInit();
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CAPTUREMODE;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_High;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CAPTURERATE;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_Init(&DCMI_InitStructure);


	// init DMA2 clock and Stream from dcmi
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream1); // needs to be done because multiple use!

	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&DCMI->DR); //RNG data register, Address offset: 0x08
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(IM_WIDTH * IM_HEIGHT);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = (uint32_t)(IM_WIDTH*IM_HEIGHT)/2; // *2/4
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1,&DMA_InitStructure);


	// interrupts setzen -> bild erst übertragen wenn ausgelesen! (DMA/DCMI unabhängig von CPU!)
	// enable TCIF Interrupt for Channel1 DMA2
	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
	/** TODO DMA Interrupt für übertragungsfehler! */
//	DMA_ITConfig(DMA2_Stream1,DMA_IT_TE,ENABLE);


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    // set global interrupt for DMA2 stream
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // set global interrupt for dcmi (see p.280 in ref manual)
    NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /** TODO PRIORITIES ?! *****/

    startClock();


}

void myDCMI::startClock(){
	GPIO_InitTypeDef GPIO_InitStructure;
	/** **************** Init MC01 clck on Pin A8 *****************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // SPEED ?!
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // pullups nicht nötig?
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);
}

void myDCMI::stopClock(){

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // SPEED ?!
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * starts capturing
 */
void myDCMI::enable(){
    DCMI_ITConfig(DCMI_IT_LINE, ENABLE);
    DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
    DCMI_ITConfig(DCMI_IT_ERR, ENABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
}

void myDCMI::disable(){
	DCMI_CaptureCmd(DISABLE);
	DCMI_Cmd(DISABLE);
	DMA_Cmd(DMA2_Stream1,DISABLE);
    DCMI_ITConfig(DCMI_IT_LINE, DISABLE);
    DCMI_ITConfig(DCMI_IT_FRAME, DISABLE);
    DCMI_ITConfig(DCMI_IT_ERR, DISABLE);

}
