/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 06/06/2014 16:45:05
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  MX_GPIO_Init();  /* PA0_GPIO_EXTI0_B1 [Blue PushButton]
                      PD12_LD4 [Green Led]
                      PD13_LD3 [Orange Led]
                      PD14_LD5 [Red Led]
                      PD15_LD6 [Blue Led]
                      PB1_MotorDirection
                      PB2_TogglePin
                      PA15_SPI1_CS 2 PGA */

  /* MX_USART2_UART_Init();  /* USART2_2_USB:
  	  	  	  	  	  	  	    PD8_UART2_TX 2 USB
 	 	 	 	 	 	 	    PD9_UART2_RX 2 USB */

  /* MX_USART3_UART_Init();  /* USART3_2_BT:
  	  	  	  	  	  	  	    PD5_UART2_TX 2 BT
  	  	  	  	  	  	        PD6_UART2_RX 2 BT */

  /* MX_DMA_Init();  /* DMA1_Stream1_USART3_RX
					 	DMA1_Stream5_USART2_RX
					 	DMA2_Stream0_ADC1 */

  /* MX_TIM1_Init();  /* TIM1_PWM:
   	  	  	  	  	     PE9_TIM1_CH1_PWM1
   	  	  	  	  	     PE11_TIM1_CH2_PWM2
   	  	  	  	  	     PE13_TIM1_CH3_PWM3
   	  	  	  	  	     PE14_TIM1_CH4_PWM4 */

  /* MX_TIM2_Init();  /* TIM2_InputCapture:
   	  	  	  	  	     PA5_TIM2_CH1_InputCapture */

  /* MX_TIM3_Init();  /* TIM3_OutputCompare */

  /* MX_I2C2_Init();  /* I2C2_2_IMU:
  	  	  	  	  	     PB10_I2C2_SCL 2 IMU
  	  	  	  	  	     PB11_I2C2_SDA 2 IMU */

  /* MX_ADC1_Init();  /* ADC1:
  	  	  	  	  	     PA1_ADC1_IN1 2 V_Bat
  	  	  	  	  	     PA2_ADC1_IN2 2 I_Motor */

  /* MX_SPI1_Init();  /* SPI1_2_PGA:
  	  	  	  	  	  	 PB3_SPI1_SCK
  	  	  	  	  	     PB4_SPI1_MISO
  	  	  	  	  	     PB5_SPI1_MOSI */

  /*----------------------------------------------------------------------------*/

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {
	  HAL_Delay(1000);
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
static void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
