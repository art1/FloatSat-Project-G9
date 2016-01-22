/*
 * ov7670.h
 *
 *  Created on: Dec 22, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_CAMERA_SUPPS_OV7670_H_
#define HARDWARE_CAMERA_SUPPS_OV7670_H_
#include "../../../Basic/basic.h"
#include "../../../Basic/sysDelay/systick.h"
#include "mySCCB.h"
#include "SCCB.h"
#include "ov7670Register.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"


#define CAMERA_FORMAT	0	// RAW 0, YUV 0, BAW 0, RGB 1
#define IMAGESIZE		(WIDTH*HEIGHT)



#define FIFO_CS_PIN     GPIO_Pin_12  	// PC12
#define FIFO_RRST_PIN   GPIO_Pin_8		// PC08
#define FIFO_RCLK_PIN   GPIO_Pin_8		// PA08
#define FIFO_WE_PIN     GPIO_Pin_11		// PC11 -> is WR Pin on the 22pin version

#define FIFO_CS_H()     GPIOC->BSRRH =FIFO_CS_PIN	  /* GPIO_SetBits(GPIOD , FIFO_CS_PIN)   */
#define FIFO_CS_L()     GPIOC->BSRRL =FIFO_CS_PIN	  /* GPIO_ResetBits(GPIOD , FIFO_CS_PIN) */

#define FIFO_RRST_H()   GPIOC->BSRRH =FIFO_RRST_PIN	  /* GPIO_SetBits(GPIOE , FIFO_RRST_PIN)   */
#define FIFO_RRST_L()   GPIOC->BSRRL =FIFO_RRST_PIN	  /* GPIO_ResetBits(GPIOE , FIFO_RRST_PIN) */

#define FIFO_RCLK_H()   GPIOA->BSRRH =FIFO_RCLK_PIN	  /* GPIO_SetBits(GPIOE , FIFO_RCLK_PIN)   */
#define FIFO_RCLK_L()   GPIOA->BSRRL =FIFO_RCLK_PIN	  /* GPIO_ResetBits(GPIOE , FIFO_RCLK_PIN) */

#define FIFO_WE_H()     GPIOC->BSRRH =FIFO_WE_PIN	  /* GPIO_SetBits(GPIOD , FIFO_WE_PIN)   */
#define FIFO_WE_L()     GPIOC->BSRRL =FIFO_WE_PIN	  /* GPIO_ResetBits(GPIOD , FIFO_WE_PIN) */

#define OV7670							   0x73
#define OV7670_REG_NUM                     114

#define PORT_VSYNC_CMOS                    GPIOB
#define RCC_AHB1Periph_PORT_VSYNC_CMOS     RCC_AHB1Periph_GPIOB
#define PIN_VSYNC_CMOS                     GPIO_Pin_7 // PB7
#define EXTI_LINE_VSYNC_CMOS               EXTI_Line7
#define PORT_SOURCE_VSYNC_CMOS             EXTI_PortSourceGPIOB
#define PIN_SOURCE_VSYNC_CMOS              GPIO_PinSource7
//
//#define OV7670							   0x73
//#define OV7670_REG_NUM                     114
//#define PORT_VSYNC_CMOS                    GPIOA
//#define RCC_APB2Periph_PORT_VSYNC_CMOS     RCC_APB2Periph_GPIOA
//#define PIN_VSYNC_CMOS                     GPIO_Pin_0
//#define EXTI_LINE_VSYNC_CMOS               EXTI_Line0
//#define PORT_SOURCE_VSYNC_CMOS             GPIO_PortSourceGPIOA
//#define PIN_SOURCE_VSYNC_CMOS              GPIO_PinSource0


class ov7670 {
public:
	ov7670();
	virtual ~ov7670();
	int Sensor_Init(void);
	void FIFO_GPIO_Configuration(void);
	void OV7670_PB7_Configuration(void);
//	void OV7670_EXTI_Configuration(void);

//	static const uint8_t OV7670_Reg[OV7670_REG_NUM][2];
private:
	mySCCB sccb;
	Sccb sccb2;
	uint8_t Vsync;	 /* ÷°Õ¨≤Ω–≈∫≈ */
	uint8_t  ReadReg(uint8_t reg);
	int  WriteReg(uint8_t reg,uint8_t regValue);



};

#endif /* HARDWARE_CAMERA_SUPPS_OV7670_H_ */
