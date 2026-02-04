/**
 ******************************************************************************
 * @file    modem_ring_buffer.c
 * @author  IoT AME
 * @version V2.1.0
 * @date    29-April-2017
 * @brief   Implements the Circular Buffer management of the Modem module
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
#include "modem_ring_buffer.h"
//#include "modem_module.h"
#include "modem_globals.h"

/** @addtogroup MIDDLEWARES
* @{
*/ 

/** @defgroup  NUCLEO_MODEM_UTILS
  * @brief Modem buffer utility
  * @{
  */ 

/** @defgroup NUCLEO_MODEM_UTILS_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup NUCLEO_MODEM_UTILS_Private_Variables
  * @{
  */

static uint8_t modem_ring_buffer[MODEM_RINGBUF_SIZE];       /* Default size for ring buffer */

#define ELEMENT_SIZE 1

/**
  * @}
  */

/** @defgroup NUCLEO_MODEM_UTILS_Private_Functions
  * @{
  */

/**
  * @brief  modem_buffer_init
  *         Initialize a circular buffer of type modem_buffer_t
  * @param  *buffer : pointer to ring buffer
  * @param  size    : size of the ring buffer
  * @retval None
  */
void modem_buffer_init(modem_buffer_t *buffer, int size) 
{
    buffer->size = size;
    buffer->start = 0;
    buffer->count = 0;
    buffer->end = 0;
    buffer->element = modem_ring_buffer;
}

/**
  * @brief  modem_flush_buffer_queue
  *         flushes the buffer
  * @param  *buffer : pointer to ring buffer
  * @retval None
  */ 
void modem_flush_buffer_queue(modem_buffer_t *buffer) 
{
  buffer->start = buffer->end;  //the tail goes up to the head and buffer becomes empty
  buffer->count = 0;
}

/**
  * @brief  modem_buffer_is_half_full
  *         checks if the buffer is half full (empty)
  * @param  *buffer : pointer to ring buffer
  * @retval status of ring buffer (=1 if half full =0 otherwise)
  */ 
int modem_buffer_is_half_full(modem_buffer_t *buffer)
{
  int bufsize = buffer->size;
  if (buffer->count >= bufsize - 100)
    {
      return 1;
    } 
  else 
    {
      return 0;
    }
}

/**
  * @brief  modem_buffer_is_half_empty
  *         checks if the buffer is less than half
  * @param  *buffer : pointer to ring buffer
  * @retval status of ring buffer (=1 if half empty =0 otherwise)
  */
int modem_buffer_is_half_empty(modem_buffer_t *buffer)
{
  //int bufsize = buffer->size;
  if (buffer->count <= 100)
    {
      return 1;
    }
  else 
    {
      return 0;
    }
}

/**
  * @brief  modem_buffer_full
  *         indicates if the given buffer is full or not
  * @param  *buffer : pointer to ring buffer
  * @retval status of ring buffer (=1 if full =0 otherwise)
  */
int modem_buffer_full(modem_buffer_t *buffer) 
{
  int bufsize = buffer->size;
  if (buffer->count == bufsize)
    return 1;
  else
    return 0;
}

/**
  * @brief  modem_buffer_empty
  *         indicates if the given buffer is empty or not
  * @param  *buffer : pointer to ring buffer
  * @retval status of ring buffer (=1 if empty =0 otherwise)
  */
int modem_buffer_empty(modem_buffer_t *buffer) 
{
  if (buffer->count == 0) 
    return 1;
  else 
    return 0;
}

/**
  * @brief  modem_push_buffer_queue
  *         pushes a new item onto the circular buffer (queues it)
  * @param  *buffer : pointer to ring buffer
  * @param  data : value to be Q'ed in ring buffer.
  * @retval None
  */
void modem_push_buffer_queue(modem_buffer_t *buffer, uint8_t *data) 
{
  int bufsize;

  if (modem_buffer_full(buffer)) 
    {
      return;
    } 
  else 
    {
      buffer->count++;    
      memcpy(&buffer->element[buffer->end], data, ELEMENT_SIZE);
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
  * @brief  modem_pop_buffer_queue
  *         dequeues the circular buffer
  * @param  *buffer : pointer to ring buffer
  * @retval element : value of popped buffer
  */ 
uint8_t * modem_pop_buffer_queue(modem_buffer_t *buffer) 
{
  uint8_t * element;
  int bufsize;

  element = &modem_pop_buffer[0];
  if (modem_buffer_empty(buffer)) 
    {

      return NULL;
    } 
  else 
    {
        //JRF TBC  
        memset(modem_pop_buffer, 0x00 , MODEM_MAX_BUFFER_GLOBAL);
        if(Modem_Control_Variables.enable_receive_data_chunk)
          { //JRF TBC
//            int buf_end = buffer->end;
//            if(buffer->count < MODEM_MAX_BUFFER_GLOBAL)
//              {
//                Modem_Counter_Variables.modem_pop_buffer_size = buffer->count;
//                if(buf_end >= buffer->start)
//                  {
//                        memcpy(element, &buffer->element[buffer->start], Modem_Counter_Variables.modem_pop_buffer_size);
//                  }
//                else
//                  {
//                        int buf_start = buffer->start;
//                        memcpy(element, &buffer->element[buffer->start], RINGBUF_SIZE - buf_start);
//                        memcpy(element+(RINGBUF_SIZE-buffer->start), &buffer->element[0], buf_end);
//                  }
//                buffer->start = buffer->end;
//                buffer->count = 0;
//              }
//            else
//              {
//                if(buf_end >= buffer->start)
//                  {
//                        memcpy(element, &buffer->element[buffer->start], (MODEM_MAX_BUFFER_GLOBAL-1));
//                        buffer->start = buffer->start + (MODEM_MAX_BUFFER_GLOBAL-1);
//                        buffer->count = buf_end - buffer->start;
//                  }
//                else
//                  {
//                        if(buffer->start + (MODEM_MAX_BUFFER_GLOBAL-1) < RINGBUF_SIZE)
//                          {
//                              memcpy(element, &buffer->element[buffer->start], (MODEM_MAX_BUFFER_GLOBAL-1));
//                              buffer->start = buffer->start + (MODEM_MAX_BUFFER_GLOBAL-1);
//                              buffer->count = (RINGBUF_SIZE - buffer->start)+ buf_end;
//                          }
//                        else
//                          {
//                              int buf_start = buffer->start;
//                              memcpy(element, &buffer->element[buffer->start], RINGBUF_SIZE-buf_start);
//                              memcpy(element+(RINGBUF_SIZE-buffer->start), &buffer->element[0], buf_start-(MODEM_MAX_BUFFER_GLOBAL+1));
//                              buffer->start = (buffer->start - (MODEM_MAX_BUFFER_GLOBAL+1));
//                              buffer->count = buf_end - buffer->start;
//                          }
//                  }
//                Modem_Counter_Variables.modem_pop_buffer_size = (MODEM_MAX_BUFFER_GLOBAL-1);
//              }
        }
      else
        {
            /* First in First Out*/
            memcpy(element, &buffer->element[buffer->start], ELEMENT_SIZE);
            buffer->start = buffer->start + ELEMENT_SIZE;
            buffer->count--;
            Modem_Counter_Variables.pop_buffer_size = 1;
            bufsize = (buffer->size);
            if (buffer->start >= bufsize)
                buffer->start = 0;
        }
      return element;
    }
}

/**
  * @brief  modem_rewinds_buffer_queue
  *         rewinds the circular buffer
  * @param  *buffer : pointer to ring buffer
  * @param  count : number of bytes to rewind 
  * @retval None
  */ 
void modem_rewind_buffer_queue(modem_buffer_t *buffer , int count)
{
    int buf_end = buffer->end;
    if(buffer->start - count >= 0)
      {
          buffer->start = buffer->start - count;
          if(buf_end > buffer->start) 
              buffer->count = buf_end - buffer->start;
          else 
              buffer->count = (MODEM_RINGBUF_SIZE-buffer->start)+buf_end;
      }
    else
      {
          buffer->start = MODEM_RINGBUF_SIZE - (count - buffer->start);
          buffer->count = (MODEM_RINGBUF_SIZE - buffer->start)+ buf_end;
      }
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
