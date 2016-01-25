/*
 * Dcmi.h
 *
 *  Created on: Jan 25, 2016
 *      Author: arthur
 */

#ifndef HARDWARE_CAMERA_SUPPS_DCMI_H_
#define HARDWARE_CAMERA_SUPPS_DCMI_H_

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dcmi.h"

#include "stm_misc.h"

class Dcmi {
private:
	uint32_t imgSize;
	uint32_t dmaMemAddr;
	uint16_t captRate;
	uint16_t captMode;
public:
	Dcmi(uint32_t imageSize, uint32_t dmaMemoryAddress, uint16_t captureRate, uint16_t captureMode);
	void InitDCMI();
	void InitGPIO();
	void EnableDCMI();
	void DisableDCMI();
};


#endif /* HARDWARE_CAMERA_SUPPS_DCMI_H_ */
