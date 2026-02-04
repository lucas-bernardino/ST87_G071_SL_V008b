/** \file
 *  \brief Definitions for Hardware Dependent Part of Crypto Device Physical
 *         Layer Using I2C for Communication
 *  \author Atmel Crypto Products
 *  \date January 14, 2013
 * \copyright Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * \atmel_crypto_device_library_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel integrated circuit.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
* \atmel_crypto_device_library_license_stop
 */

#ifndef I2C_PHYS_H_
#define I2C_PHYS_H_

#include <stdint.h>                       // data type definitions
#include "atca_i2c.h"

/** \defgroup crypto_device_i2c_hardware Module 18: I2C Interface
 * Definitions are supplied for various I2C configuration values
 * such as clock, timeouts, and error codes.
*/

//! I2C clock
//#define I2C_CLOCK                         (100000.0)

//! Use pull-up resistors.
#define I2C_PULLUP

/** \brief number of polling iterations for TWINT bit in TWSR after
 *         creating a Start condition in #i2c_send_start()
 *
 * Adjust this value considering how long it takes to check a status bit
 * in the TWI status register, decrement the timeout counter,
 * compare its value with 0, and branch.
 */
#define I2C_START_TIMEOUT                ((uint8_t) 250)

/** \brief number of polling iterations for TWINT bit in TWSR after sending
 *         or receiving a byte.
 *
 * Adjust this value considering how long it takes to check a status bit
 * in the TWI status register, decrement the timeout counter,
 * compare its value with 0, branch, and to send or receive one byte.
 */
#define I2C_BYTE_TIMEOUT                 ((uint8_t) 200)

/** \brief number of polling iterations for TWSTO bit in TWSR after
 *         creating a Stop condition in #i2c_send_stop().
 *
 * Adjust this value considering how long it takes to check a status bit
 * in the TWI control register, decrement the timeout counter,
 * compare its value with 0, and branch.
 */
#define I2C_STOP_TIMEOUT                 ((uint8_t) 250)

/* Definition for I2Cx clock resources */
#define I2Cx                            I2C1
#define RCC_PERIPHCLK_I2Cx              RCC_PERIPHCLK_I2C1
#define RCC_I2CxCLKSOURCE_SYSCLK        RCC_I2C1CLKSOURCE_SYSCLK
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 
#define I2Cx_DMA_CLK_ENABLE()           __HAL_RCC_DMA2_CLK_ENABLE()   
   
#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()
   
/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_6
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_7
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1

/* Definition for I2Cx's NVIC */
#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn
#define I2Cx_EV_IRQHandler              I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHandler              I2C1_ER_IRQHandler   
   
/* Definition for I2Cx's DMA */
#define I2Cx_DMA                        DMA2   
#define I2Cx_DMA_INSTANCE_TX            DMA2_Channel7
#define I2Cx_DMA_INSTANCE_RX            DMA2_Channel6
#define I2Cx_DMA_REQUEST_TX             DMA_REQUEST_5 
#define I2Cx_DMA_REQUEST_RX             DMA_REQUEST_5

/* Definition for I2Cx's DMA NVIC */
#define I2Cx_DMA_TX_IRQn                DMA2_Channel7_IRQn
#define I2Cx_DMA_RX_IRQn                DMA2_Channel6_IRQn
#define I2Cx_DMA_TX_IRQHandler          DMA2_Channel7_IRQHandler
#define I2Cx_DMA_RX_IRQHandler          DMA2_Channel6_IRQHandler

/* Global var */
extern I2C_HandleTypeDef hi2c1;

/** @} */

#endif /*I2C_PHYS_H_*/
