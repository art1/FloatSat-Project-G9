/*
 * camera.cpp
 *
 *  Created on: 08.12.2015
 *      Author: akynos
 */


#include "Camera.h"
#include "Supps/initRegister.h"

#define ONE_BYTE_REG_ADDR 0x01
#define TWO_BYTE_REG_ADDR 0x02



Camera::Camera() : Thread("Camera",50){
	reset = HAL_GPIO(GPIO_010); //PA10
	power = HAL_GPIO(GPIO_033); //PC01
	imFin = false;
	isActive = false;
	captureImage = false;
	//	//	processData = false;
	//	//	sendPic = true;
	initDone = false;
	imFin = false;
	//	VSync = -1;
	startTransmissionLength = 0;
	endTransmissionLength = 0;
}





void Camera::initPeripherals(){
	PRINTF("init GPIOs\n");
	// initialize GPIOs
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); // PCLK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); // VSYNC
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); // HREF
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); // D0
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI); // D1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI); // D2
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI); // D3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI); // D4
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); // D5
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI); // D6
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI); // D7

	//Struct for Port-A (HREF, PCLK)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Struct for Port-B (VSYNC, D5)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Struct for Port-C (D0, D1)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//Struct for Port-E (D2,D3,D5,D6,D7)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//Struct for PA10 (RESET)
	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource10;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//Struct for PC1(PowerDown)
	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource1;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	// EXTCLK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);

	/** initialize DCMI Interface ********************/
	PRINTF("init DCMI\n");

	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	DCMI_InitTypeDef DCMI_InitStructure;
	DCMI_DeInit();
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;

	DCMI_Init(&DCMI_InitStructure);

	DCMI_CROPInitTypeDef DCMI_CROPInitStructure;

	DCMI_CROPInitStructure.DCMI_HorizontalOffsetCount = 0; //
	DCMI_CROPInitStructure.DCMI_CaptureCount = 2 * (WIDTH) - 1; //
	DCMI_CROPInitStructure.DCMI_VerticalStartLine = 0; //
	DCMI_CROPInitStructure.DCMI_VerticalLineCount = HEIGHT - 1; //

	DCMI_CROPConfig(&DCMI_CROPInitStructure);

	/** initialize Direct Memory Access Interface **********/
	PRINTF("init DMA\n");
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_BufferSize = IMAGESIZE / 4;
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &DCMI_Buffer;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x50050028;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;

	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	/** initialize Interrup Controller *******************/
	PRINTF("init NVIC\n");
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;

	NVIC_Init(&NVIC_InitStructure);

}




void Camera::init() {
	//	initCamera();

	i2c1.init();


	initDone = true;

}


void Camera::setNewData(CAM_DATA _data){
	this->daten = _data;
	this->isActive = this->daten.activateCamera;
	this->captureImage = this->daten.capture;
}


bool Camera::initFinished(){
	return initDone;
}


void Camera::initCamera(){
	//	suspendCallerUntil(NOW()+2*SECONDS);
	PRINTF("starting cam init\n");

	//	memset(picture,0,sizeof(picture));


	//	retVal = sccb.ov7670_set(0x12,0x80);
	//	retVal = sccb.ov7670_set(0x12,0x00);
	uint8_t r[2] = {0x12,0x80};
	int k = i2c1.write(CAM_ADDRESS,r,2);
	PRINTF("retCode %d\n",k);
	suspendCallerUntil(NOW()+10*MILLISECONDS);
	k = i2c1.write(CAM_ADDRESS,init_registers[0],2);
	PRINTF("retCode %d\n",k);
	if(k < 0) i2c1.init();
	suspendCallerUntil(NOW()+10*MILLISECONDS);

	int totalReg = sizeof(init_registers) / 2;
	int result;
	for (int i = 0; i < totalReg; i++) {
		result = i2c1.write(CAM_ADDRESS, init_registers[i], 2);
		AT(NOW() + 2*MILLISECONDS);
		if (result < 0) {
			PRINTF("Couldnt write register:%d result: %d\n", i, result);
		}
	}

	//	uint16_t x = 0;
	//	while (init_registers[x][0] != 0xFF && init_registers[x][1] != 0xFF) {
	//		xprintf("init register: status x=%d\n", x);
	//
	//		//		retVal = sccb.ov7670_set((unsigned char) init_registers[x][0],
	//		//				(unsigned char) init_registers[x][1]);
	//
	//		retVal = i2c1.write(CAM_ADDRESS,init_registers[x],1);
	//		if(retVal < 0) i2c1.init();
	//		//		uint8_t read = sccb.ov7670_get((unsigned char) init_registers[x][0]);
	//
	//		i2c1.writeRead(CAM_ADDRESS,init_registers[x],1,read,1);
	//		xprintf("cam Init Init %d: reg 0x%x = 0x%x = 0x%x \n", x,
	//				init_registers[x][0], init_registers[x][1], read[0]);
	//		if (retVal) {
	//			xprintf("ERROR I2C %d\n", retVal);
	//		}
	//		x++;
	//
	//	}
	xprintf("done with InitOV7670 init\n");


	initDone = true;
}

void Camera::run(){
	PRINTF("cam thread started!\n");
	// build start and stop frames

	forLoop(j,3)startTransmission[startTransmissionLength++] = FRAME_START;
	startTransmission[startTransmissionLength++] = PL;
	forLoop(j,4) startTransmission[startTransmissionLength++] = 0;
	forLoop(j,4) startTransmission[startTransmissionLength++] = 0;
	startTransmission[startTransmissionLength++] = 160;

	forLoop(j,8) endTransmission[endTransmissionLength++] = 0;
	forLoop(j,3) endTransmission[endTransmissionLength++] = FRAME_END;

	initPeripherals();
	initCamera();


	//	initCamera();
	suspendCallerUntil(NOW()+10*MILLISECONDS);
	reset.init(true);
	power.init(true);

	reset.setPins(1);
	power.setPins(0);
	//	sccb.I2CInit();



	uint8_t data[8];
	uint8_t tmp;

	while(!isActive){
		suspendCallerUntil(END_OF_TIME);
	}


	while(1){
		//		suspendCallerUntil(END_OF_TIME);
		//		suspendCallerUntil(NOW() + 1500*MILLISECONDS);
		if(captureImage){
			ORANGE_ON;
			PRINTF("capturing imageo!\n");

			//capturing image
			DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
			DCMI_ITConfig(DCMI_IT_OVF, ENABLE);
			DCMI_ITConfig(DCMI_IT_ERR, ENABLE);
			DMA_Cmd(DMA2_Stream1, ENABLE);
			DCMI_Cmd(ENABLE);
			DCMI_CaptureCmd(ENABLE);
			AT(NOW() + 10*MILLISECONDS);


			while(!captureDone){
				suspendCallerUntil(NOW()+10*MILLISECONDS);
			}

			PRINTF("Capturing Done, sending Image:\n");
			//			for(int i= 1;i< IMAGESIZE-1;i+=2){
			//				PRINTF("%d,",DCMI_Buffer[i]);
			//			}

			consFrame = 0;
			toSend = 0;
			transmitPicture();
			//			for( int i=0;i<IMAGESIZE-1;i++){
			//				picture[toSend++] = DCMI_Buffer[i];
			//				if(toSend == 200){
			//					PRINTF("Sending Frame %d\n",consFrame);
			//					suspendCallerUntil(NOW()+5*MILLISECONDS);
			//				}
			//			}


			ORANGE_OFF;
			imFin = false;
			suspendCallerUntil(END_OF_TIME);
		}


		//			captureImage = false;
	}

}

void Camera::sendImage(){
	PRINTF("Sending image\n");
	captureDone = true;
}

void Camera::transmitPicture(){
	// disable telemetry first
	INTERCOMM comm;
	comm.camData.activateCamera = false;
	comm.camData.capture = false;
	comm.camData.sendImage = true;
	comm.changedVal = CAM_CHANGED;
	interThreadComm.publish(comm);

	// write all the shit on Bluetooth Uart
	int16_t k = 0;
	PRINTF("sending image....\n");

	// put 160 bytes (one line) in one package
	// send all the shit to the GS
	uint32_t consecNumber = 0;

	for(int i=0;i<IMAGESIZE;i++){
		picture[i%160] = DCMI_Buffer[i];
		if(!(i%160) && (i !=0)){
			longToChar(tmp,consecNumber);
			forLoop(j,4){
				startTransmission[8+j] = tmp[j];
			}
			forLoop(i,13){
				k = bt_uart.write((const char*)&startTransmission[i],1);
				if(k<0)i--;
			}
			forLoop(i,160){
				k = bt_uart.write((const char*)&picture[i],1);
				if(k<0)i--;
			}
			forLoop(i,11){
				k = bt_uart.write((const char*)&endTransmission[i],1);
				if(k<0)i--;
			}
			consecNumber++;
			suspendCallerUntil(NOW()+1*MILLISECONDS);
		}
	}

//	forLoop(i,3){
//		k = bt_uart.write(&startTransmission[i],1);
//		if(k<0)i--;
//	}
//	for( int i=0;i<IMAGESIZE-1;i++){
//		//		DCMI_Buffer[i];
//
//
//		k = bt_uart.write((const char*) &DCMI_Buffer[i],1);
//		/*if((i < IMAGESIZE-1) && (!((i+1)%2)))*/PRINTF("%d,",DCMI_Buffer[i]);
//		if(k <=0 ) i--;
//		//		suspendCallerUntil(NOW()+1*MILLISECONDS);
//	}
//	forLoop(i,4){
//		k = bt_uart.write(&endTransmission[i],1);
//		if(k<0)i--;
//	}



	PRINTF("Sending Image done!\n");

	// re-enable telemetry
	comm.camData.sendImage = false;
	interThreadComm.publish(comm);

}

