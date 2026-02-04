  /**
  ******************************************************************************
  * @file    modem_ring_buffer.h
  * @author  IoT AME
  * @version V2.1.0
  * @date    29-April-2017
  * @brief   Header File for Circular Buffer management of the Wi-Fi module
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
#ifndef __MODEM_RING_BUFFER_H
#define __MODEM_RING_BUFFER_H
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>



/** @addtogroup BSP
* @{
*/ 


/** @defgroup  NUCLEO_WIFI_BUFFER_MGMT 
  * @brief Wi-Fi_driver modules
  * @{
  */ 


/** @defgroup NUCLEO_WIFI_BUFFER_MGMT_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup NUCLEO_WIFI_BUFFER_MGMT_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup NUCLEO_WIFI_BUFFER_MGMT_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup NUCLEO_WIFI_BUFFER_MGMT_Private_Variables
  * @{
  */

/**
  * @brief  Ring Buffer structure definiton
  */
struct modem_buffer {
    volatile int start; // position of first data from USART
    volatile int end;   // position of last data from USART
    volatile int size;  // Max size in terms of number of data packets (Total Bytes/size of each packet (8 bytes))
    volatile int count; // number of currently filled data packets (=size if full & =0 if empty)
    
    /*unsigned main buffer pointer*/
    uint8_t *element;
};
 
typedef struct modem_buffer modem_buffer_t;

int modem_buffer_full(modem_buffer_t *buffer);
int modem_buffer_empty(modem_buffer_t *buffer);
int modem_buffer_is_half_full(modem_buffer_t *buffer);
int modem_buffer_is_half_empty(modem_buffer_t *buffer);
uint8_t * modem_popstack(modem_buffer_t *buffer);
uint8_t * modem_pop_buffer_queue(modem_buffer_t *buffer);
void modem_buffer_init(modem_buffer_t *buffer, int size);
void modem_flush_buffer_queue(modem_buffer_t *buffer);
void modem_push_buffer_queue(modem_buffer_t *buffer, uint8_t *data);
void modem_rewind_buffer_queue(modem_buffer_t *buffer , int count);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
  
/**
  * @}
  */



#endif
