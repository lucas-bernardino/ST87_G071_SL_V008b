  /**
  ******************************************************************************
  * @file    modem_globals.h
  * @author  IoT AME
  * @version V2.1.0
  * @date    29-April-2017
  * @brief   Header File for storing all the global variables of Modem module
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
#ifndef __MODEM_GLOBALS_H
#define __MODEM_GLOBALS_H
    
/* Includes ------------------------------------------------------------------*/
#include "modem_const.h"
#include "modem_event_buffer.h"
#include "modem_ring_buffer.h"
#include "modem_module.h"
//jrf #include "diili_time.h"


/* Private variables ---------------------------------------------------------*/

/***********All Buffers**************/
extern char modem_UserDataBuff[USER_DATA_BUFFER_SIZE];   /* Used to store data that is to be send in callback to user */
extern uint8_t modem_pop_buffer[MODEM_MAX_BUFFER_GLOBAL];
extern char modem_network_time_buff[MAX_NETWORK_TIME_SIZE];
extern modem_event_TypeDef m_element;
extern modem_scan *modem_scanned_list;          // [MAX_WIFI_SCAN_NETWORK]

extern UART_HandleTypeDef UartModemHandle,UartMsgHandle;
extern Modem_Config_HandleTypeDef Modem_Config_Variables;
extern Modem_Counter_Variables_t Modem_Counter_Variables;
extern Modem_Control_Variables_t Modem_Control_Variables;
//JRF TBC extern __IO IO_modem_status_flag_typedef IO_modem_status_flag;
extern IO_modem_status_flag_typedef IO_modem_status_flag;

extern volatile uint8_t modem_connected;            //Set once if modem is connected for first time
extern volatile uint8_t modem_client_connected;     //Set once if client is connected
extern volatile uint8_t modem_client_disconnected;  //Set once if client is dis-connected

extern modem_bool modem_open_sockets[3];            //Max open sockets allowed is 3. Each array element depicts one socket (true=open, false=closed)
extern uint32_t modemTick,modemTimeout; 

extern uint8_t Modem_AT_Cmd_Buff[MODEM_AT_COMMAND_BUFF_SIZE];

extern TIM_HandleTypeDef modemTimHandle, modemPushTimHandle; //JRF TBC ,PushTimHandle;


//var teste
extern uint8_t contador;

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __WIFI_GLOBALS_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

