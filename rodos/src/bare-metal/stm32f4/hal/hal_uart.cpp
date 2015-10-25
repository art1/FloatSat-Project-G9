/*
 * Copyright (c) 2011 Uni Wuerzburg, Wuerzburg

 * All rights reserved.
 *
 * @author Michael Ruffer
 */

/* By driver enabled pins, "()"->also possible
 * - pins can be changed in HAL_UART::HAL_UART()
   UART1 - Rx:  PB7  (PA10)             Tx: PB6     (PA9)
           RTS: PA12                    CTS:PA11

   UART2 - Rx:  PD6  (PA3)              Tx: PD5     (PA2)
           RTS: PD4  (PA1)              CTS:PD3     (PA0)

   UART3 - Rx:  PD9  (PB11, PC11)       Tx: PD8     (PB10 , PC10)
           RTS: PB14 (PD12)             CTS:PD11    (PB13)

   UART4 - Rx:  PC11 (PA1)              Tx:PC10     (PA0)
           // no hardware flow control

   UART5 - Rx:  PD2                     Tx: PC12
           // no hardware flow control

   UART6 - Rx:  PC7                     Tx: PC6
           RTS: PG8 (PG12)              CTS: PG15 (PG13)

   // STM32F42x/STM32F43x only
   UART7 - Rx:  PF6 (PE7)               Tx: PF7 (PE8)
           // no hardware flow control

   UART8 - Rx:  PE0                     Tx: PE1
           // no hardware flow control
*/

#include "rodos.h"
#include "hal/hal_uart.h"
#include "../STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h"
#include "../STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h"
#include "../STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h"
#include "../STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h"


#define UART_IDX_MIN        UART_IDX1
#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
#define UART_IDX_MAX        UART_IDX8
#else
#define UART_IDX_MAX        UART_IDX6
#endif

#define UART_IDX_MAX_DMA    UART_IDX3


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

extern "C"{
	void USART1_IRQHandler();
	void USART2_IRQHandler();
	void USART3_IRQHandler();
	void UART4_IRQHandler();
	void UART5_IRQHandler();
	void USART6_IRQHandler();
	void UART7_IRQHandler();
	void UART8_IRQHandler();

	void DMA2_Stream7_IRQHandler();
	void DMA1_Stream6_IRQHandler();
	void DMA1_Stream3_IRQHandler();

	void DMA2_Stream5_IRQHandler();
	void DMA1_Stream5_IRQHandler();
	void DMA1_Stream1_IRQHandler();

}


#define DMA_TIMEOUT 10*MILLISECONDS

void EnableDMAClock();


class ReceiveTrigger : Thread {
public:
	ReceiveTrigger():Thread("UARTRecvTrigger"){}
private:
	void run();
};


class HW_HAL_UART {

	friend class ReceiveTrigger;
	friend class HAL_UART;

	friend void USART1_IRQHandler();
	friend void USART2_IRQHandler();
	friend void USART3_IRQHandler();
	friend void UART4_IRQHandler();
	friend void UART5_IRQHandler();
	friend void USART6_IRQHandler();
	friend void UART7_IRQHandler();
	friend void UART8_IRQHandler();


	friend void DMA2_Stream7_IRQHandler();
	friend void DMA1_Stream6_IRQHandler();
	friend void DMA1_Stream3_IRQHandler();

	friend void DMA2_Stream5_IRQHandler();
	friend void DMA1_Stream5_IRQHandler();
	friend void DMA1_Stream1_IRQHandler();

	UART_IDX idx;
	int hwFlowCtrl;
	int baudrate;
	HAL_UART* hal_uart;
	int uartRxError;


	USART_TypeDef *UARTx;

    GPIO_TypeDef *UART_TX_GPIO_PORT;
    uint16_t UART_TX_GPIO;

    GPIO_TypeDef *UART_RX_GPIO_PORT;
    uint16_t UART_RX_GPIO;

    GPIO_TypeDef *UART_RTS_GPIO_PORT;
    uint16_t UART_RTS_GPIO;

    GPIO_TypeDef *UART_CTS_GPIO_PORT;
    uint16_t UART_CTS_GPIO;


	bool isDMAEnabeld;
	volatile bool DMATransmitRunning;
	volatile int DMATransmitRunningSize;
	volatile bool DMAReceiveRunning;
	volatile int DMAReceiveRunningSize;

	int DMAMaxReceiveSize;

	BlockFifo<uint8_t,UART_BUF_SIZE> receiveBuffer;
	BlockFifo<uint8_t,UART_BUF_SIZE> transmittBuffer;

	void DMATransmitFinishedHandler();
	void DMAReceiveFinishedHandler();


	void DMAConfigure();
	void DMAStartTransfer(void* memoryBuffer,int len);
	void DMAStartReceive(void* memoryBuffer,int len);


	void SendTxBufWithDMA();
	void RceiveIntoRxBufWithDMA();

	void UARTIRQHandler();

	int init(unsigned int baudrate);

	uint32_t getGPIO_PinSource(uint32_t GPIO_Pin);
	uint32_t getRCC_APB1Periph_GPIOx(GPIO_TypeDef *port);
	uint32_t getRCC_APBxPeriph_UARTx();
	uint8_t getGPIO_AF_UARTx();
	IRQn getUARTx_IRQn();
};


HW_HAL_UART UART_contextArray[UART_IDX_MAX+1];
DMA_Stream_TypeDef*  UART_DMA_TxStreams[3] = {DMA2_Stream7,DMA1_Stream6,DMA1_Stream3};
DMA_Stream_TypeDef*  UART_DMA_RxStreams[3] = {DMA2_Stream5,DMA1_Stream5,DMA1_Stream1};


volatile bool enableDMATriggerThread=false;
void ReceiveTrigger::run()
{

	while(!enableDMATriggerThread){
		suspendCallerUntil(NOW()+ 10 * SECONDS);
	}

    TIME_LOOP(1*MILLISECONDS, DMA_TIMEOUT) {
    	for(int i=1;i<=3;i++){
    		if(UART_contextArray[i].isDMAEnabeld){
    			DMA_Cmd(UART_DMA_RxStreams[i-1],DISABLE);
    		}
    	}
    }

}

static ReceiveTrigger triggerthread;



HAL_UART::HAL_UART(UART_IDX uartIdx){

    if ((uartIdx < UART_IDX_MIN) || (uartIdx > UART_IDX_MAX)) {
        context = &UART_contextArray[UART_IDX0]; // UART_IDX0 is not used in this implementation
                                                 // -> so we can use this contextArray to save wrong idx
                                                 // -> with this saved idx all HAL_UART-methods will return correctly with -1
        context->idx = uartIdx;
        return;
    }

	context = &UART_contextArray[uartIdx];
	context->idx = uartIdx;
	context->baudrate = 115200;
	context->hwFlowCtrl = UART_PARAMETER_HW_FLOW_CONTROL;
	context->hal_uart = this;
	context->isDMAEnabeld = false;
	context->DMATransmitRunning = false;
	context->DMAReceiveRunning = false;

	switch (uartIdx){
	case UART_IDX1:
	    context->UARTx              = USART1;
	    // pins
	    context->UART_TX_GPIO_PORT  = GPIOB;        // \ PB6
	    context->UART_TX_GPIO       = GPIO_Pin_6;   // /
	    context->UART_RX_GPIO_PORT  = GPIOB;        // \ PB7
	    context->UART_RX_GPIO       = GPIO_Pin_7;   // /
        context->UART_RTS_GPIO_PORT = GPIOA;        // \ PA12
        context->UART_RTS_GPIO      = GPIO_Pin_12;  // /
        context->UART_CTS_GPIO_PORT = GPIOA;        // \ PA11
        context->UART_CTS_GPIO      = GPIO_Pin_11;  // /
	    break;

    case UART_IDX2:
        context->UARTx              = USART2;
        // pins
        context->UART_TX_GPIO_PORT  = GPIOD;        // \ PD5
        context->UART_TX_GPIO       = GPIO_Pin_5;   // /
        context->UART_RX_GPIO_PORT  = GPIOD;        // \ PD6
        context->UART_RX_GPIO       = GPIO_Pin_6;   // /
        context->UART_RTS_GPIO_PORT = GPIOD;        // \ PD4
        context->UART_RTS_GPIO      = GPIO_Pin_4;  // /
        context->UART_CTS_GPIO_PORT = GPIOD;        // \ PD3
        context->UART_CTS_GPIO      = GPIO_Pin_3;  // /
        break;

    case UART_IDX3:
        context->UARTx              = USART3;
        // pins
        context->UART_TX_GPIO_PORT  = GPIOD;        // \ PD8
        context->UART_TX_GPIO       = GPIO_Pin_8;   // /
        context->UART_RX_GPIO_PORT  = GPIOD;        // \ PD9
        context->UART_RX_GPIO       = GPIO_Pin_9;   // /
        context->UART_RTS_GPIO_PORT = GPIOB;        // \ PB14
        context->UART_RTS_GPIO      = GPIO_Pin_14;  // /
        context->UART_CTS_GPIO_PORT = GPIOD;        // \ PD11
        context->UART_CTS_GPIO      = GPIO_Pin_11;  // /
        break;

    case UART_IDX4:
        context->UARTx              = UART4;
        // pins
        context->UART_TX_GPIO_PORT  = GPIOC;        // \ PC10
        context->UART_TX_GPIO       = GPIO_Pin_10;  // /
        context->UART_RX_GPIO_PORT  = GPIOC;        // \ PC11
        context->UART_RX_GPIO       = GPIO_Pin_11;  // /
        break;

    case UART_IDX5:
        context->UARTx              = UART5;
        // pins
        context->UART_TX_GPIO_PORT  = GPIOC;        // \ PC12
        context->UART_TX_GPIO       = GPIO_Pin_12;  // /
        context->UART_RX_GPIO_PORT  = GPIOD;        // \ PD2
        context->UART_RX_GPIO       = GPIO_Pin_2;   // /
        break;

    case UART_IDX6:
        context->UARTx              = USART6;
        // pins
        context->UART_TX_GPIO_PORT  = GPIOC;        // \ PC6
        context->UART_TX_GPIO       = GPIO_Pin_6;   // /
        context->UART_RX_GPIO_PORT  = GPIOC;        // \ PC7
        context->UART_RX_GPIO       = GPIO_Pin_7;   // /
        context->UART_RTS_GPIO_PORT = GPIOG;        // \ PG8
        context->UART_RTS_GPIO      = GPIO_Pin_8;   // /
        context->UART_CTS_GPIO_PORT = GPIOG;        // \ PG15
        context->UART_CTS_GPIO      = GPIO_Pin_15;  // /
        break;

    case UART_IDX7:
        context->UARTx              = UART7;
        // pins
        context->UART_TX_GPIO_PORT  = GPIOF;        // \ PF7
        context->UART_TX_GPIO       = GPIO_Pin_7;   // /
        context->UART_RX_GPIO_PORT  = GPIOF;        // \ PF6
        context->UART_RX_GPIO       = GPIO_Pin_6;   // /
        break;

    case UART_IDX8:
        context->UARTx              = UART8;
        // pins
        context->UART_TX_GPIO_PORT  = GPIOE;        // \ PE1
        context->UART_TX_GPIO       = GPIO_Pin_1;   // /
        context->UART_RX_GPIO_PORT  = GPIOE;        // \ PE0
        context->UART_RX_GPIO       = GPIO_Pin_0;   // /
        break;

    default: break;
	}
};




extern "C"
{

void USART1_IRQHandler() {
	UART_contextArray[UART_IDX1].UARTIRQHandler();
	NVIC_ClearPendingIRQ(USART1_IRQn);
}


void USART2_IRQHandler() {
	UART_contextArray[UART_IDX2].UARTIRQHandler();
	NVIC_ClearPendingIRQ(USART2_IRQn);
}

void USART3_IRQHandler() {
	UART_contextArray[UART_IDX3].UARTIRQHandler();
	NVIC_ClearPendingIRQ(USART3_IRQn);
}


void UART4_IRQHandler() {
	UART_contextArray[UART_IDX4].UARTIRQHandler();
	NVIC_ClearPendingIRQ(UART4_IRQn);
}


void UART5_IRQHandler() {
	UART_contextArray[UART_IDX5].UARTIRQHandler();
	NVIC_ClearPendingIRQ(UART5_IRQn);
}

void USART6_IRQHandler() {
    UART_contextArray[UART_IDX6].UARTIRQHandler();
    NVIC_ClearPendingIRQ(USART6_IRQn);
}

#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
void UART7_IRQHandler() {
    UART_contextArray[UART_IDX7].UARTIRQHandler();
    NVIC_ClearPendingIRQ(UART7_IRQn);
}

void UART8_IRQHandler() {
    UART_contextArray[UART_IDX8].UARTIRQHandler();
    NVIC_ClearPendingIRQ(UART8_IRQn);
}
#endif



//DMA tx for UART1
void DMA2_Stream7_IRQHandler(){
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7))
	{
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);
		NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn); // Clear this before calling the next function because this may start a new DMA TRansfer, which can already generate a new Interrupt
		UART_contextArray[UART_IDX1].DMATransmitFinishedHandler();
	}else{
		NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);
	}
}

//DMA tx for UART2
void DMA1_Stream6_IRQHandler(){
	if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6))
	{
		DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);
		NVIC_ClearPendingIRQ(DMA1_Stream6_IRQn); // Clear this before calling the next function because this may start a new DMA TRansfer, which can already generate a new Interrupt
		UART_contextArray[UART_IDX2].DMATransmitFinishedHandler();
	}else{
		NVIC_ClearPendingIRQ(DMA1_Stream6_IRQn);
	}

}

//DMA tx for UART3
void DMA1_Stream3_IRQHandler(){
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3))
	{
		DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);
		NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn); // Clear this before calling the next function because this may start a new DMA TRansfer, which can already generate a new Interrupt
		UART_contextArray[UART_IDX3].DMATransmitFinishedHandler();
	}else{
		NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
	}

}

//DMA rx for UART1
void DMA2_Stream5_IRQHandler(){
	if(DMA_GetITStatus(DMA2_Stream5,DMA_IT_TCIF5))
	{
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn); // Clear this before calling the next function because this may start a new DMA TRansfer, which can already generate a new Interrupt
		UART_contextArray[UART_IDX1].DMAReceiveFinishedHandler();
	}else{
		NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
	}
}

//DMA rx for UART2
void DMA1_Stream5_IRQHandler(){
	if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5))
	{
		DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5);
		NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn); // Clear this before calling the next function because this may start a new DMA TRansfer, which can already generate a new Interrupt
		UART_contextArray[UART_IDX2].DMAReceiveFinishedHandler();
	}else{
		NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn);
	}

}

//DMA rx for UART3
void DMA1_Stream1_IRQHandler(){
	if(DMA_GetITStatus(DMA1_Stream1,DMA_IT_TCIF1))
	{
		DMA_ClearITPendingBit(DMA1_Stream1,DMA_IT_TCIF1);
		NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn); // Clear this before calling the next function because this may start a new DMA TRansfer, which can already generate a new Interrupt
		UART_contextArray[UART_IDX3].DMAReceiveFinishedHandler();
	}else{
		NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);
	}

}


} // end extern "C"





/*
 * USART
 * - all USART will be initialized in 8N1 mode
 */
int HAL_UART::init(unsigned int iBaudrate) {

	if ((context->idx < UART_IDX_MIN) || (context->idx > UART_IDX_MAX)) {return -1;}

	return context->init(iBaudrate);
}


int HAL_UART::config(UART_PARAMETER_TYPE type, int paramVal) {

	UART_IDX idx = context->idx;

	if ((idx < UART_IDX_MIN) || (idx > UART_IDX_MAX)) {return -1;}

	USART_TypeDef* usart = context->UARTx;
	USART_InitTypeDef Uis;
	GPIO_InitTypeDef gpioRTS;
	GPIO_InitTypeDef gpioCTS;

	USART_StructInit(&Uis);
	Uis.USART_BaudRate = context->baudrate;
	Uis.USART_HardwareFlowControl = context->hwFlowCtrl;

	switch (type) {
		case UART_PARAMETER_BAUDRATE:
			if (paramVal > 0)
			{
				Uis.USART_BaudRate = paramVal;
				context->baudrate = paramVal;
			}
			else {return -1;}
			break;

		case UART_PARAMETER_HW_FLOW_CONTROL:
		    // HW flow control is only supported by USART1/2/3/6
			if (idx == UART_IDX4 || idx == UART_IDX5 || idx == UART_IDX7 || idx == UART_IDX8) {return -1;}

			GPIO_StructInit(&gpioRTS);
			GPIO_StructInit(&gpioCTS);

			if (paramVal > 0){ // enable HW Flow Control
				Uis.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
				context->hwFlowCtrl = USART_HardwareFlowControl_RTS_CTS;
				gpioRTS.GPIO_Mode = GPIO_Mode_AF;
				gpioCTS.GPIO_Mode = GPIO_Mode_AF;
			}
			else {  // disable HW Flow Control
				Uis.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
				context->hwFlowCtrl = USART_HardwareFlowControl_None;
				gpioRTS.GPIO_Mode = GPIO_Mode_IN; // set gpio to default state
				gpioCTS.GPIO_Mode = GPIO_Mode_IN; // set gpio to default state
			}

			GPIO_PinAFConfig(context->UART_CTS_GPIO_PORT,context->getGPIO_PinSource(context->UART_CTS_GPIO),context->getGPIO_AF_UARTx());
			GPIO_PinAFConfig(context->UART_RTS_GPIO_PORT,context->getGPIO_PinSource(context->UART_RTS_GPIO),context->getGPIO_AF_UARTx());
			gpioCTS.GPIO_Pin = context->UART_CTS_GPIO;
			GPIO_Init(context->UART_CTS_GPIO_PORT, &gpioCTS);
			gpioRTS.GPIO_Pin = context->UART_RTS_GPIO;
			GPIO_Init(context->UART_RTS_GPIO_PORT, &gpioRTS);

			break;

		case UART_PARAMETER_ENABLE_DMA:
		    if(context->idx > UART_IDX_MAX_DMA){ //Only UART 1-3 support DMA
		        return -1;
		    }
		    context->DMAConfigure();
		    if(paramVal > UART_BUF_SIZE){
		        context->DMAMaxReceiveSize=UART_BUF_SIZE;
		    }else{
		        context->DMAMaxReceiveSize=paramVal;
		    }
		    context->isDMAEnabeld=true;
		    context->RceiveIntoRxBufWithDMA();
		    enableDMATriggerThread=true;
		    return 0;

		    break; // end case UART_PARAMETER_HW_FLOW_CONTROL

		default: return -1;
	}

	USART_Cmd(usart, DISABLE);
	USART_Init(usart, &Uis);
	USART_Cmd(usart, ENABLE);

	return 0;
	
}


void HAL_UART::reset(){
	if ((context->idx < UART_IDX_MIN) || (context->idx > UART_IDX_MAX)) {return;}

	USART_TypeDef *usart = context->UARTx;

	// Interrupt UART disable
	NVIC_DisableIRQ(context->getUARTx_IRQn());

	// reset interface
	USART_DeInit(usart);

	// reset pins
	// - reset value of all pins: FUNCTION A, GPIO, INPUT
	GPIO_InitTypeDef Gis;
	GPIO_StructInit(&Gis);
	Gis.GPIO_Mode = GPIO_Mode_IN;

	Gis.GPIO_Pin = context->UART_RX_GPIO;
	GPIO_Init(context->UART_RX_GPIO_PORT, &Gis);
	Gis.GPIO_Pin = context->UART_TX_GPIO;
	GPIO_Init(context->UART_TX_GPIO_PORT, &Gis);
}


int HAL_UART::read(char* buf, int size) {

    if ((context->idx < UART_IDX_MIN) || (context->idx > UART_IDX_MAX)) {return -1;}
    if(size <=0) return 0;

    int readCnt = 0;
    int i = 0;

    uint8_t* p = context->receiveBuffer.getBufferToRead(readCnt);

    if (p) {
        if (readCnt > size) {readCnt = size;}

        for (i = 0; i < readCnt; i++ ) {
            buf[i] = p[i];
        }
        context->receiveBuffer.readConcluded(readCnt);

        if(context->isDMAEnabeld && !context->DMAReceiveRunning){
            context->RceiveIntoRxBufWithDMA();
        }

        return readCnt;
    } else {
        return 0;
    }
}


int HAL_UART::write(const char* buf, int size) {

    if ((context->idx < UART_IDX_MIN) || (context->idx > UART_IDX_MAX)) {return -1;}
    if(size <=0) return 0;

    int i = 0;

    int spaceinbuffer;

    uint8_t* p = context->transmittBuffer.getBufferToWrite(spaceinbuffer);

    if(p){
        if(spaceinbuffer < size){
            size=spaceinbuffer;
        }

        for (i=0;i<size;i++){
            p[i]=buf[i];
        }
        context->transmittBuffer.writeConcluded(size);

        USART_ITConfig(context->UARTx,USART_IT_TC,DISABLE);
        if(context->isDMAEnabeld){
            if(!context->DMATransmitRunning){
                context->SendTxBufWithDMA();
            }
        }else{
            USART_ITConfig(context->UARTx,USART_IT_TXE,ENABLE);
        }


        return size;
    }else{
        return 0;
    }
}


int HAL_UART::getcharNoWait() {
    if ((context->idx < UART_IDX_MIN) || (context->idx > UART_IDX_MAX)) {return -1;}

	uint8_t c = 0;
	bool dataAvailible = context->receiveBuffer.get(c);

	if(context->isDMAEnabeld && !context->DMAReceiveRunning){
		context->RceiveIntoRxBufWithDMA();
	}

	if (dataAvailible)
	{
		return (int)c;
	}else	{
		return -1;
	}
}


int HAL_UART::putcharNoWait(char c) {
    if ((context->idx < UART_IDX_MIN) || (context->idx > UART_IDX_MAX)) {return -1;}

	if(context->transmittBuffer.put(c)){

		 USART_ITConfig(context->UARTx,USART_IT_TC,DISABLE);
		if(context->isDMAEnabeld){
			if(!context->DMATransmitRunning){
				context->SendTxBufWithDMA();
			}
		}else{
			USART_ITConfig(context->UARTx,USART_IT_TXE,ENABLE);
		}
		return c;
	}else{
		return -1;
	}

}


int HAL_UART::status(UART_STATUS_TYPE type) {
    if ((context->idx < UART_IDX_MIN) || (context->idx > UART_IDX_MAX)) {return -1;}

	switch (type)
	{
		case UART_STATUS_RX_BUF_LEVEL:
			return context->receiveBuffer.getElementCount();

		case UART_STATUS_RX_ERROR:
			int temp;
			temp=context->uartRxError;
			context->uartRxError=0;
			return temp;

//		case UART_STATUS_TX_COMPLETE:
//		    return (USART_GetFlagStatus(context->UARTx,USART_FLAG_TC) == SET);

		//...
		default: return -1;
	}
}


bool HAL_UART::isWriteFinished() {
    if ((context->idx < UART_IDX_MIN) || (context->idx > UART_IDX_MAX)) {return false;}
	return context->transmittBuffer.isEmpty() && (USART_GetFlagStatus(context->UARTx,USART_FLAG_TC) == SET);
}


bool HAL_UART::isDataReady() {
    if ((context->idx < UART_IDX_MIN) || (context->idx > UART_IDX_MAX)) {return false;}
	return !context->receiveBuffer.isEmpty();
}










/****************** HW_HAL_UART *******************/


void HW_HAL_UART::DMAConfigure() {
	if ((idx < UART_IDX_MIN) || (idx > UART_IDX_MAX_DMA)) {return;}

	EnableDMAClock();

	DMA_InitTypeDef DMA_InitStruct;

//Configure tx DMA

      DMA_Stream_TypeDef* dma=UART_DMA_TxStreams[idx-1];

	  DMA_StructInit(&DMA_InitStruct);
	  DMA_InitStruct.DMA_Channel = DMA_Channel_4; //All UARTS have Channel 4
	  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UARTx->DR);
	  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)0; //Changed later
	  DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	  DMA_InitStruct.DMA_BufferSize = 16; //Changed later
	  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	  DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_Init(dma, &DMA_InitStruct);

	  DMA_ITConfig(dma,DMA_IT_TC,ENABLE);

	  USART_DMACmd(UARTx,USART_DMAReq_Tx,ENABLE);

	  //Configure Rx DMA

	  dma=UART_DMA_RxStreams[idx-1];

	  DMA_StructInit(&DMA_InitStruct);
	  DMA_InitStruct.DMA_Channel = DMA_Channel_4; //All UARTS have Channel 4
	  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UARTx->DR);
	  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)0; //Changed later
	  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	  DMA_InitStruct.DMA_BufferSize = 16; //Changed later
	  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	  DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_Init(dma, &DMA_InitStruct);

	  DMA_ITConfig(dma,DMA_IT_TC,ENABLE);

	  USART_ITConfig(UARTx,USART_IT_RXNE,DISABLE);
	  USART_ITConfig(UARTx,USART_IT_ERR,ENABLE);
	  USART_DMACmd(UARTx,USART_DMAReq_Rx,ENABLE);

	  //Enable DMA IRQs
	  switch(idx){
	  case UART_IDX1:
	      NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	      NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	      break;
	  case UART_IDX2:
	      NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	      NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	      break;
	  case UART_IDX3:
	      NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	      NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	      break;
	  default:
	      break;
	  }


}

void HW_HAL_UART::DMAStartTransfer(void* memoryBuffer,int len) {

	if ((idx < UART_IDX_MIN) || (idx > UART_IDX_MAX_DMA)) {return;}

	DMA_Stream_TypeDef* dma=UART_DMA_TxStreams[idx-1];

	DMA_Cmd(dma,DISABLE);

	//USART_ClearFlag(uart_getRegPointer(idx),USART_FLAG_TC);

	dma->M0AR=(uint32_t)memoryBuffer;
	dma->NDTR= len;

	//PRINTF("Start with UART %d DMA @ %d len=%d",idx,(int)memoryBuffer,len);

	DMA_Cmd(dma,ENABLE);

}

void HW_HAL_UART::DMAStartReceive(void* memoryBuffer,int len) {

	if ((idx < UART_IDX_MIN) || (idx > UART_IDX_MAX_DMA)) {return;}

	DMA_Stream_TypeDef* dma=UART_DMA_RxStreams[idx-1];

	DMA_Cmd(dma,DISABLE);
	while(DMA_GetCmdStatus(dma)==ENABLE){}


	dma->M0AR=(uint32_t)memoryBuffer;
	dma->NDTR= len;


	DMA_Cmd(dma,ENABLE);


	//USART_ClearFlag(uart_getRegPointer(idx),USART_FLAG_ORE);
	//USART_DMACmd(uart_getRegPointer(idx),USART_DMAReq_Rx,ENABLE);
}

void HW_HAL_UART::DMATransmitFinishedHandler(){
	//PRINTF("enter general dma irq handler ");

	if ((idx < UART_IDX_MIN) || (idx > UART_IDX_MAX_DMA)) {return;}

	//while(USART_GetFlagStatus(uart_getRegPointer(idx),USART_FLAG_TC)==SET){}

	DMA_Stream_TypeDef* dma=UART_DMA_TxStreams[idx-1];
	int bytesNotTransfered = DMA_GetCurrDataCounter(dma); //Should be zero
	int bytesTransfered = DMATransmitRunningSize-bytesNotTransfered;

	transmittBuffer.readConcluded(bytesTransfered);

	if(transmittBuffer.isEmpty()){ //Buffer now emtpy
		DMATransmitRunning=false;
		//hal_uart->upCallWriteFinished();
		 USART_ITConfig(UARTx,USART_IT_TC,ENABLE);
	}else{
		SendTxBufWithDMA();
	}

}

void HW_HAL_UART::DMAReceiveFinishedHandler() {

	if ((idx < UART_IDX_MIN) || (idx > UART_IDX_MAX_DMA)) {return;}

	//while(USART_GetFlagStatus(uart_getRegPointer(idx),USART_FLAG_TC)==SET){}

	DMA_Stream_TypeDef* dma=UART_DMA_RxStreams[idx-1];
	int bytesTransfered = DMAReceiveRunningSize-DMA_GetCurrDataCounter(dma);

	receiveBuffer.writeConcluded(bytesTransfered);

	if(bytesTransfered > 0){
		hal_uart->upCallDataReady();
	}

	RceiveIntoRxBufWithDMA();

}


void HW_HAL_UART::SendTxBufWithDMA() {

	int len;
	uint8_t*  p = transmittBuffer.getBufferToRead(len);

	if(p){
		DMATransmitRunningSize=len;
		DMATransmitRunning=true;
		DMAStartTransfer(p,len);
	}
}

void HW_HAL_UART::RceiveIntoRxBufWithDMA() {

	int len;
	uint8_t* p;

	p = receiveBuffer.getBufferToWrite(len);

	if(p){

		if(len > DMAMaxReceiveSize) len = DMAMaxReceiveSize;

		DMAReceiveRunningSize=len;
		DMAReceiveRunning=true;
		DMAStartReceive(p,len);
	}else{
		DMAReceiveRunning=false; //Buffer Overflow, disable Receive until next read
		uartRxError++;
	}

}


void EnableDMAClock() {

	static bool init=false;
	if(!init){
		init=true;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	}
}


void HW_HAL_UART::UARTIRQHandler() {

    uint8_t c;

    if (USART_GetFlagStatus(UARTx,USART_FLAG_ORE) || (isDMAEnabeld && USART_GetITStatus(UARTx,USART_IT_ERR)))
    {
        USART_ReceiveData(UARTx);
        uartRxError++;
    }

    else if (USART_GetITStatus(UARTx,USART_IT_RXNE))
    {
        c = USART_ReceiveData(UARTx) & 0xFF;
        if(!receiveBuffer.put(c)){
            uartRxError++; //FIFO OVerflow
        }

        hal_uart->upCallDataReady();

    }else if(USART_GetITStatus(UARTx,USART_IT_TXE)){
        if(transmittBuffer.get(c)){
            UARTx->DR=c;
        }else{
            // no more data, disable TXE
            USART_ITConfig(UARTx,USART_IT_TXE,DISABLE);
            USART_ITConfig(UARTx,USART_IT_TC,ENABLE);
            //hal_uart->upCallWriteFinished();
        }
    }else if(USART_GetITStatus(UARTx,USART_IT_TC)){
        USART_ITConfig(UARTx,USART_IT_TC,DISABLE);
        hal_uart->upCallWriteFinished();
    }

}


uint32_t HW_HAL_UART::getGPIO_PinSource(uint32_t GPIO_Pin) {

    uint32_t GPIO_PinSource = 0;

    while (GPIO_Pin >>= 1) GPIO_PinSource++;

    return GPIO_PinSource;
}


uint32_t HW_HAL_UART::getRCC_APB1Periph_GPIOx(GPIO_TypeDef *port) {

    switch((uint32_t)port){
    case GPIOA_BASE: return RCC_AHB1Periph_GPIOA;
    case GPIOB_BASE: return RCC_AHB1Periph_GPIOB;
    case GPIOC_BASE: return RCC_AHB1Periph_GPIOC;
    case GPIOD_BASE: return RCC_AHB1Periph_GPIOD;
    case GPIOE_BASE: return RCC_AHB1Periph_GPIOE;
    case GPIOF_BASE: return RCC_AHB1Periph_GPIOF;
    case GPIOG_BASE: return RCC_AHB1Periph_GPIOG;
    case GPIOH_BASE: return RCC_AHB1Periph_GPIOH;
    case GPIOI_BASE: return RCC_AHB1Periph_GPIOI;
    case GPIOJ_BASE: return RCC_AHB1Periph_GPIOJ;
    case GPIOK_BASE: return RCC_AHB1Periph_GPIOK;
    default: return 0;
    }
}

uint32_t HW_HAL_UART::getRCC_APBxPeriph_UARTx() {

    switch(idx){
    case UART_IDX1: return RCC_APB2Periph_USART1;
    case UART_IDX2: return RCC_APB1Periph_USART2;
    case UART_IDX3: return RCC_APB1Periph_USART3;
    case UART_IDX4: return RCC_APB1Periph_UART4;
    case UART_IDX5: return RCC_APB1Periph_UART5;
    case UART_IDX6: return RCC_APB2Periph_USART6;
    case UART_IDX7: return RCC_APB1Periph_UART7;
    case UART_IDX8: return RCC_APB1Periph_UART8;
    default: return 0;
    }
}


uint8_t HW_HAL_UART::getGPIO_AF_UARTx() {

    switch(idx){
    case UART_IDX1: return GPIO_AF_USART1;
    case UART_IDX2: return GPIO_AF_USART2;
    case UART_IDX3: return GPIO_AF_USART3;
    case UART_IDX4: return GPIO_AF_UART4;
    case UART_IDX5: return GPIO_AF_UART5;
    case UART_IDX6: return GPIO_AF_USART6;
    case UART_IDX7: return GPIO_AF_UART7;
    case UART_IDX8: return GPIO_AF_UART8;
    default: return 0;
    }
}


IRQn HW_HAL_UART::getUARTx_IRQn() {

    switch(idx){
    case UART_IDX1: return USART1_IRQn;
    case UART_IDX2: return USART2_IRQn;
    case UART_IDX3: return USART3_IRQn;
    case UART_IDX4: return UART4_IRQn;
    case UART_IDX5: return UART5_IRQn;
    case UART_IDX6: return USART6_IRQn;
#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
    case UART_IDX7: return UART7_IRQn;
    case UART_IDX8: return UART8_IRQn;
#endif
    default: return USART1_IRQn;
    }
}


int HW_HAL_UART::init(unsigned int baudrate) {

    if ((idx < UART_IDX_MIN) || (idx > UART_IDX_MAX)) {return -1;}

    this->baudrate = baudrate;

    GPIO_InitTypeDef Gis;
    USART_InitTypeDef Uis;

    // Release reset and enable clock
    USART_DeInit(UARTx); // reset

    if (idx == UART_IDX1 || idx == UART_IDX6){
        RCC_APB2PeriphClockCmd(getRCC_APBxPeriph_UARTx(), ENABLE); // only USART1/6 are connected to APB2
    //    RCC_AHB2PeriphResetCmd(UART_CLK, DISABLE); // it's done in USART_DeInit()
    }else{
        RCC_APB1PeriphClockCmd(getRCC_APBxPeriph_UARTx(), ENABLE); // USART2/3 & UART4/5/7/8 are connected to APB1
    //    RCC_AHB1PeriphResetCmd(UART_CLK, DISABLE); // it's done in USART_DeInit()
    }

    // Enable GPIO clock and release reset
    RCC_AHB1PeriphClockCmd(getRCC_APB1Periph_GPIOx(UART_TX_GPIO_PORT) | getRCC_APB1Periph_GPIOx(UART_RX_GPIO_PORT), ENABLE);
    RCC_AHB1PeriphResetCmd(getRCC_APB1Periph_GPIOx(UART_TX_GPIO_PORT) | getRCC_APB1Periph_GPIOx(UART_RX_GPIO_PORT), DISABLE);

    // configure pin multiplexer -> choose alternate function (AF) UART
    GPIO_PinAFConfig(UART_TX_GPIO_PORT,getGPIO_PinSource(UART_TX_GPIO),getGPIO_AF_UARTx());
    GPIO_PinAFConfig(UART_RX_GPIO_PORT,getGPIO_PinSource(UART_RX_GPIO),getGPIO_AF_UARTx());

    // configure TX pin
    GPIO_StructInit(&Gis);
    Gis.GPIO_Mode = GPIO_Mode_AF;
    Gis.GPIO_Pin = UART_TX_GPIO;
    GPIO_Init(UART_TX_GPIO_PORT, &Gis);

    // configure RX pin
    GPIO_StructInit(&Gis);
    Gis.GPIO_Mode = GPIO_Mode_AF;
    Gis.GPIO_Pin = UART_RX_GPIO;
    GPIO_Init(UART_RX_GPIO_PORT, &Gis);

    USART_StructInit(&Uis);         // init struct to 8N1
    Uis.USART_BaudRate = baudrate;  // set baudrate
    USART_Init(UARTx, &Uis);        // init USART/UART
    USART_Cmd(UARTx, ENABLE);       // enable USART/UART

    // Interrupt USART1 enable
    USART_ITConfig(UARTx,USART_IT_RXNE,ENABLE);
    USART_ITConfig(UARTx,USART_IT_TXE,ENABLE);
    NVIC_EnableIRQ(getUARTx_IRQn());

    return 0;
}


#ifndef NO_RODOS_NAMESPACE
}
#endif

