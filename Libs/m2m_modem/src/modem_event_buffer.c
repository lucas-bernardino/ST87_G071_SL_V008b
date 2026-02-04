/**
 ******************************************************************************
 * @file    modem_event_buffer.c
 * @author  AME IoT
 * @version V2.1.0
 * @date    29-April-2017
 * @brief   Implements the Event Buffer management of the Modem module
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

#include <stdio.h>
#include <stdlib.h>
#include "modem_event_buffer.h"
//#include "modem_module.h"
#include "modem_globals.h"


/** @addtogroup BSP
* @{
*/ 

/** @defgroup  NUCLEO_MODEM_BUFFER_MGMT
  * @brief Modem_driver modules
  * @{
  */ 

/** @defgroup NUCLEO_MODEM_BUFFER_MGMT_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup NUCLEO_MODEM_BUFFER_MGMT_Private_Variables
  * @{
  */

#define ELEMENT_SIZE sizeof(modem_event_TypeDef)   //1

#ifdef USE_STM32L0XX_NUCLEO
static modem_event_TypeDef event_buffer[10];  /* Used to store various events. Its size varies in case of L0 and other Boards. */
#else
static modem_event_TypeDef event_buffer[50];
#endif

/**
  * @}
  */

/** @defgroup NUCLEO_MODEM_BUFFER_MGMT_Private_Functions
  * @{
  */

/**
  * @brief  init
  *         Initialize a circular buffer of type modem_event_buffer
  * @param  *buffer : pointer to event buffer
  * @param  size    : Number of events in Event_Buffer
  * @retval None
  */
void modem_event_init(modem_event_buffer *buffer, int size) 
{
    uint32_t element_size;
    element_size = sizeof(modem_event_TypeDef);
    buffer->size = element_size*size;
    buffer->start = 0;
    buffer->count = 0;
    buffer->end = 0;
    buffer->element = event_buffer;
}

/**
  * @brief  full
  *         indicates if the given buffer is full or not
  * @param  *buffer : pointer to event buffer
  * @retval status of event_buffer (=1 if full =0 if otherwise)
  */
int modem_event_full(modem_event_buffer *buffer) 
{
  int bufsize = buffer->size;
  if (buffer->count == (bufsize/ELEMENT_SIZE)) 
    { 
      return 1;
    } 
  else 
    {
      return 0;
    }
}

/**
  * @brief  empty
  *         indicates if the given buffer is empty or not
  * @param  *buffer : pointer to event buffer
  * @retval status of event_buffer (=1 if empty =0 if otherwise)
  */
int modem_event_empty(modem_event_buffer *buffer) {
    if (buffer->count == 0) 
      {
        return 1;
      } 
    else 
      {
        return 0;
      }
}

/**
  * @brief  push_buffer
  *         pushes the data structure onto the circular buffer (queues it)
  * @param  *buffer : pointer to event buffer
  * @param  data : data to be Queued in event_buffer
  * @retval None
  */
void modem_push_eventbuffer_queue(modem_event_buffer *buffer, modem_event_TypeDef data) 
{
  int bufsize;
  uint32_t index;
  
  if (modem_event_full(buffer))
    {
      return;
    } 
  else 
    {
      index=buffer->end/ELEMENT_SIZE;
      buffer->element[index].data_length = data.data_length;
      buffer->element[index].enc = data.enc;
      buffer->element[index].ok_eval = data.ok_eval;
      buffer->element[index].socket_id = data.socket_id;
      /*buffer->element[index].wind = data.wind;*/
      buffer->element[index].event = data.event;
      buffer->element[index].event_pop = data.event_pop;
      buffer->count++;
      buffer->end = buffer->end + ELEMENT_SIZE;
      
      //wrap around if max size is reached
      bufsize = (buffer->size);
      if (buffer->end >= bufsize) 
        {
          buffer->end = 0;
        }
    }
}

/**
  * @brief  pop_buffer_queue
  *         dequeues the circular buffer
  * @param  *buffer : pointer to event buffer
  * @retval None
  */ 
modem_event_TypeDef * modem_pop_eventbuffer_queue(modem_event_buffer *buffer) 
{
  int bufsize;
  uint32_t index;

  if (modem_event_empty(buffer)) 
    {
      #if MODEM_PRINT_DEBUG
          //printf("\r\Event Buffer Empty!!\r\n");
      #endif
      return NULL;
    } 
  else
    {
      /* First in First Out*/
      index=buffer->start/ELEMENT_SIZE;
      m_element.data_length = buffer->element[index].data_length;
      m_element.enc = buffer->element[index].enc;
      m_element.ok_eval = buffer->element[index].ok_eval;
      m_element.socket_id = buffer->element[index].socket_id;
      m_element.event = buffer->element[index].event;
      m_element.event_pop = buffer->element[index].event_pop;
      buffer->start = buffer->start + ELEMENT_SIZE;
      buffer->count--;

      bufsize = (buffer->size);
      if (buffer->start >= bufsize)
        {
          buffer->start = 0;
        }
      return &m_element;
    }
}

/**
  * @brief  reset_event
  *         sets an event to default values
  * @param  *r_event : event to reset
  * @retval None
  */ 
void modem_reset_event(modem_event_TypeDef *r_event)
{
  r_event->data_length = 0;
  r_event->enc         = MODEM_FALSE;
  r_event->event       = MODEM_NO_EVENT; 
  r_event->ok_eval     = MODEM_FALSE;
  r_event->socket_id   = 5;              //Socket ID 0-4
  r_event->event_pop   = MODEM_TRUE;
  /*r_event->wind        = 1;*/
}

/**
  * @}
  */ 

/**
  * @}
  */ 


/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
