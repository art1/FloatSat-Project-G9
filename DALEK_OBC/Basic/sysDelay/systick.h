/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               systick.h
** Descriptions:            None
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-10-30
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef BASIC_SYSDELAY_SYSTICK_H
#define BASIC_SYSDELAY_SYSTICK_H

/* Includes ------------------------------------------------------------------*/	   
#include "stm32f4xx.h"
//#include "stm32f4xx_it.h"
#include "stm32f4xx_conf.h"

/* Private function prototypes -----------------------------------------------*/
#ifdef __cplusplus
extern "C"
#endif

void Delay_millis(__IO uint32_t nTime);
#ifdef __cplusplus
extern "C"
#endif
void TimingDelay_Decrement(void);


#endif
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/






























