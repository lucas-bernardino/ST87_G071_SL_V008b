  /**
  ******************************************************************************
  * @file    modem_event_buffer.h
  * @author  IoT AME
  * @version V2.1.0
  * @date    29-Abril-2017
  * @brief   Header File for Event Buffer management of the Modem module
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
#ifndef __MODEM_EVENT_BUFFER_H
#define __MODEM_EVENT_BUFFER_H
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "modem_interface.h"


/** @addtogroup BSP
* @{
*/ 

/** @defgroup  NUCLEO_MODEM_BUFFER_MGMT 
  * @brief Modem_driver modules
  * @{
  */ 

/** @defgroup NUCLEO_MODEM_BUFFER_MGMT_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup NUCLEO_MODEM_BUFFER_MGMT_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup NUCLEO_MODEM_BUFFER_MGMT_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup NUCLEO_MODEM_BUFFER_MGMT_Private_Variables
  * @{
  */


/**
  * @brief  Modem Events enumeration declaration
  */    
typedef enum
{
  MODEM_NO_EVENT  = 0,
  MODEM_IND_EVENT = 0x0001,
  MODEM_SOCK_ID_EVENT,
  MODEM_ALREADY_CONNECT_EVENT,

  //MODEM_GPIO_EVENT,
  MODEM_ATTACHED,
  MODEM_OK_EVENT,
  MODEM_CONNECT_FAIL_EVENT,
  MODEM_STANDBY_CONFIG_EVENT,
  MODEM_RESUME_CONFIG_EVENT,
  MODEM_HTTP_EVENT,
  MODEM_CLIENT_SOCKET_WRITE_EVENT,
  MODEM_FILE_EVENT,
  MODEM_CLIENT_SOCKET_OPEN_EVENT,
  MODEM_CLIENT_SOCKET_CLOSE_EVENT,
  MODEM_CLIENT_SOCKET_STATUS_EVENT,
  MODEM_FW_UPDATE_EVENT,
  MODEM_IP_GPRSATC,
  MODEM_CONNECT_EVENT,
  MODEM_CLIENT_SSL_SOCKET_OPEN_EVENT,
  MODEM_CLIENT_SSL_SOCKET_CLOSE_EVENT,
  MODEM_CLIENT_SOCKET_WRITE_DATA,
  MODEM_CLIENT_SOCKET_WRITE_OK,
  MODEM_CLIENT_SOCKET_READ_DATA,
  MODEM_URC_RDY,
  MODEM_URC_STANDBY,
  MODEM_URC_CALL_READY,
  MODEM_URC_TIME_SYNC,
  MODEM_LATEST_NETWORK_TIME_SYNC,
  MODEM_GET_CLOCK_TIME,
  MODEM_TCPIP_TASK_EVENT,
  MODEM_DNS_GET_IP,
  MODEM_DEACT_EVENT,
  MODEM_MQTT_CLIENT_SOCKET_OPEN_EVENT,
  MODEM_MQTT_CLIENT_PUBLISH_MESSAGE,
  MODEM_MQTT_CLIENT_SUBSCRIBE_MESSAGE,
  MODEM_SIM_STATUS,
  MODEM_TCP_CONNECTION_EVENT

} Modem_Events;

///* Status of the M95 TCPIP Task */
//typedef enum
//{
//  IP_INITIAL=0,
//  IP_START,
//  IP_CONFIG,
//  IP_IND,
//  IP_GPRSACT,
//  IP_STATUS,
//  TCP_CONNECTING,
//  UDP_CONNECTING,
//  IP_CLOSE, 
//  CONNECT_OK, 
//  PDP_DEACT,
//} Tcpip_task_status;



/**
  * @brief  Event structure definiton
  */
typedef struct
{
  Modem_Events event;
  /*uint32_t wind;*/
  uint32_t data_length;
  uint16_t socket_id;
  uint16_t socket_sc;
  uint16_t socket_sid;

  modem_bool enc;
  modem_bool ok_eval;
  modem_bool event_pop;
  /*  Modem_Events event; */
} modem_event_TypeDef;


/**
  * @brief  Event Buffer structure definiton
  */
struct m_event_buffer //JRF TBC
{
    volatile int start;  // position of first data from USART
    volatile int end;    // position of last data from USART
    volatile int size;   // Max size in terms of number of data packets (Total Bytes/size of each packet (8 bytes))
    volatile int count;  // number of currently filled data packets (=size if full & =0 if empty)

    /*unsigned main buffer pointer*/
    modem_event_TypeDef *element;
};

typedef struct m_event_buffer modem_event_buffer;

int modem_event_full(modem_event_buffer *buffer);
int modem_event_empty(modem_event_buffer *buffer);
void modem_reset_event(modem_event_TypeDef *r_event);
void modem_event_init(modem_event_buffer *buffer, int size);
void modem_push_eventbuffer_queue(modem_event_buffer *buffer, modem_event_TypeDef data);
modem_event_TypeDef * modem_pop_eventbuffer_queue(modem_event_buffer *buffer);

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
