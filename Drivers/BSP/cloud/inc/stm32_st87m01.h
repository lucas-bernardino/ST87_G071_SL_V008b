  /**
  ******************************************************************************
  * @file    stm32_quectel_m95.h
  * @author  IoT AME 
  * @version V2.0.1
  * @date    29-April-2017
  * @brief   Header file for HAL related functionality of X-CUBE-WIFI1
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_ST87M01_H
#define __STM32_ST87M01_H


/* Includes ------------------------------------------------------------------*/

//#include "stm32_hal.h"
#if defined (STM32L4)
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#endif

#if defined (STM32G0)
#include "stm32g0xx.h"
#endif
//
//#else
//#warning "Define STM32 familiy!!!!"
//#endif

///* Exported macro ------------------------------------------------------------*/
#define DEBUG_MODEM     1 


#if defined (STM32L4)

#define USART_MODEM             USART2 
#define USART_MODEM_BAUDRATE    115200


#define USART_MODEM_CLK_ENABLE()                __HAL_RCC_USART2_CLK_ENABLE()
#define USART_MODEM_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE() 
#define USART_MODEM_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART_MODEM_TX_PIN                      GPIO_PIN_2
#define USART_MODEM_TX_GPIO_PORT                GPIOA
#define USART_MODEM_RX_PIN                      GPIO_PIN_3
#define USART_MODEM_RX_GPIO_PORT                GPIOA
#define USART_MODEM_TX_AF                       GPIO_AF7_USART2
#define USART_MODEM_RX_AF                       GPIO_AF7_USART2

#define USART_MODEM_IRQn                        USART2_IRQn
#define USART_MODEM_IRQHandler                  USART2_IRQHandler

#define USART_MODEM_FORCE_RESET()               __HAL_RCC_USART2_FORCE_RESET()
#define USART_MODEM_RELEASE_RESET()             __HAL_RCC_USART2_RELEASE_RESET()

#define DMA_MODEM_CLK_ENABLE()                  __HAL_RCC_DMA1_CLK_ENABLE()

/* Definition for USARTx's DMA */
#define USART_MODEM_TX_DMA_CHANNEL              DMA1_Channel7
#define USART_MODEM_RX_DMA_CHANNEL              DMA1_Channel6

/* Definition for USARTx's DMA Request */
#define USART_MODEM_TX_DMA_REQUEST              DMA_REQUEST_2
#define USART_MODEM_RX_DMA_REQUEST              DMA_REQUEST_2

/* Definition for USARTx's NVIC */
#define USART_MODEM_DMA_TX_IRQn                 DMA1_Channel7_IRQn
#define USART_MODEM_DMA_RX_IRQn                 DMA1_Channel6_IRQn
#define USART_MODEM_DMA_TX_IRQHandler           DMA1_Channel7_IRQHandler
#define USART_MODEM_DMA_RX_IRQHandler           DMA1_Channel6_IRQHandler

#define TIM_MODEM                               TIM15
#define TIM_MODEM_CLK_ENABLE()                  __HAL_RCC_TIM15_CLK_ENABLE()

#define TIM_MODEMp                              TIM16
#define TIM_MODEMp_CLK_ENABLE()                 __HAL_RCC_TIM16_CLK_ENABLE()

/* Definition for TIMx's NVIC */
#define TIM_Modem_IRQn                          TIM1_BRK_TIM15_IRQn
#define TIM_Modem_IRQHandler                    TIM1_BRK_TIM15_IRQHandler

#define TIM_Modemp_IRQn                         TIM1_UP_TIM16_IRQn
#define TIM_Modemp_IRQHandler                   TIM1_UP_TIM16_IRQHandler

/* Definitions for PWRKEY */
#define PWRKEY_CLK_ENABLE()                     __HAL_RCC_GPIOA_CLK_ENABLE()
#define PWRKEY_CLK_DISABLE()                    __HAL_RCC_GPIOA_CLK_DISABLE()
#define PWRKEY_PIN                              GPIO_PIN_4
#define PWRKEY_GPIO_PORT                        GPIOA        

//jrf#else
//jrf    #error "MCU not defined!!"
#endif

#if defined (STM32G0)

#define USART_MODEM             USART1 
#define USART_MODEM_BAUDRATE    115200


#define USART_MODEM_CLK_ENABLE()                __HAL_RCC_USART1_CLK_ENABLE()
#define USART_MODEM_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE() 
#define USART_MODEM_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART_MODEM_TX_PIN                      GPIO_PIN_6
#define USART_MODEM_TX_GPIO_PORT                GPIOB
#define USART_MODEM_RX_PIN                      GPIO_PIN_7
#define USART_MODEM_RX_GPIO_PORT                GPIOB
#define USART_MODEM_TX_AF                       GPIO_AF0_USART1
#define USART_MODEM_RX_AF                       GPIO_AF0_USART1

#define USART_MODEM_IRQn                        USART1_IRQn
#define USART_MODEM_IRQHandler                  USART1_IRQHandler

#define USART_MODEM_FORCE_RESET()               __HAL_RCC_USART1_FORCE_RESET()
#define USART_MODEM_RELEASE_RESET()             __HAL_RCC_USART1_RELEASE_RESET()

#define DMA_MODEM_CLK_ENABLE()                  __HAL_RCC_DMA1_CLK_ENABLE()

/* Definition for USARTx's DMA */
#define USART_MODEM_TX_DMA_CHANNEL              DMA1_Channel1 //DMA1_Channel7
#define USART_MODEM_RX_DMA_CHANNEL              DMA1_Channel2 // DMA1_Channel6

/* Definition for USARTx's DMA Request */
#define USART_MODEM_TX_DMA_REQUEST              DMA_REQUEST_USART1_TX  //jrf??? DMA_REQUEST_2  //jrf ???
#define USART_MODEM_RX_DMA_REQUEST              DMA_REQUEST_USART1_RX  //jrf ???DMA_REQUEST_2

/* Definition for USARTx's NVIC */
#define USART_MODEM_DMA_TX_IRQn                 DMA1_Channel1_IRQn
#define USART_MODEM_DMA_RX_IRQn                 DMA1_Channel2_IRQn
#define USART_MODEM_DMA_TX_IRQHandler           DMA1_Channel1_IRQHandler
#define USART_MODEM_DMA_RX_IRQHandler           DMA1_Channel2_IRQHandler

#define TIM_MODEM                               TIM15
#define TIM_MODEM_CLK_ENABLE()                  __HAL_RCC_TIM15_CLK_ENABLE()

#define TIM_MODEMp                              TIM16
#define TIM_MODEMp_CLK_ENABLE()                 __HAL_RCC_TIM16_CLK_ENABLE()

/* Definition for TIMx's NVIC */
#define TIM_Modem_IRQn                          TIM15_IRQn  //jrf TIM1_BRK_TIM15_IRQn
#define TIM_Modem_IRQHandler                    TIM15_IRQHandler

#define TIM_Modemp_IRQn                         TIM16_IRQn  //jrf TIM1_UP_TIM16_IRQn
#define TIM_Modemp_IRQHandler                   TIM16_IRQHandler

/* Definitions for PWRKEY */
//#define PWRKEY_CLK_ENABLE()                     __HAL_RCC_GPIOA_CLK_ENABLE()
//#define PWRKEY_CLK_DISABLE()                    __HAL_RCC_GPIOA_CLK_DISABLE()
//#define PWRKEY_PIN                              GPIO_PIN_4
//#define PWRKEY_GPIO_PORT                        GPIOA        

//jrf#else
//jrf    #error "MCU not defined!!"
#endif

void Modem_Timer_Config(void);
void Modem_UART_Configuration(uint32_t baud_rate);
void Modem_UART_DeInit (void);
void Modem_Push_Timer_Config(void);
void Modem_PWRKEY_Configuration(void);

#endif