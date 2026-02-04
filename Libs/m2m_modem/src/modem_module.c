/**
 ******************************************************************************
 * @file    modem_module.c
 * @author  AME IoT 
 * @version V2.1.0
 * @date    06-Nov-2024
 * @brief   Enable Modem functionality using AT cmd set
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
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "modem_module.h"
#include "modem_ring_buffer.h"
#include "modem_event_buffer.h"
#include "modem_globals.h"
#include "std_types.h"
#include "utilities.h"
#include "main.h"
#include "cmsis_os.h"
#include "metro.h"

/** @addtogroup MIDDLEWARES
* @{
*/ 
extern UART_HandleTypeDef huart4;
extern osMessageQueueId_t LedStatusQueueHandle;
extern osMessageQueueId_t LedStatusQueueCloudHandle;
extern uint32_t server_port;

//JRF TO REMOVE THIS

extern char clientKey[];
extern char clientCRT[];
extern char rootCA[];
extern char broker_ip[20];
extern uint32_t conta_ok;
extern uint32_t urc_counter;
extern uint8_t getNTP_time;
extern uint8_t get_nb_sent_status;

extern uint32_t test_mes;
extern char server_ip[];
extern uint8_t sckt_type;

led_status_t led_cloud;
uint8_t ip_size;

uint16_t ipread_cnt=0;
uint16_t iprecv_cnt=0;

#ifdef COMM_DEBUG
uint8_t SPY_AT_Buff[MODEM_AT_COMMAND_BUFF_SIZE];
uint16_t spy_buff_size=0;

#endif

/**
  * @brief Structure definition containing instances of other structures
  */ 
typedef struct
{
  modem_buffer_t big_buff;
  modem_event_buffer event_buff;
  modem_event_TypeDef modem_event;
  modem_event_TypeDef * DeQed_modem_event;
  HAL_StatusTypeDef receive_status;
}modem_instances_t;

modem_instances_t modem_instances;

/***********All Buffers**************/

#if defined (__CC_ARM)
size_t strnlen (const char* s, size_t maxlen);

size_t strnlen (const char* s, size_t maxlen)
  {
    size_t len = 0;
    while ((len <= maxlen) && (*s))
      {
          s++;
          len++;
      }
    return len;
  }
#endif


/**
  * @}
  */
  

/**
  * @brief  Modem_Module_and_Timer_Init
  *         Initialize modem module
  * @param  None
  * @retval None
  */
void Modem_Module_and_Timer_Init(void)
{
  memset(modem_UserDataBuff, 0x00, MODEM_MAX_BUFFER_GLOBAL);//Flush the buffer;
  memset(modem_pop_buffer, 0x00 , MODEM_MAX_BUFFER_GLOBAL);
  memset(modem_network_time_buff, 0x00 , MAX_NETWORK_TIME_SIZE);
  modem_buffer_init(&modem_instances.big_buff, MODEM_RINGBUF_SIZE);//Init the ring buffer  
  //IO_modem_status_flag.modem_ready = 0; //reset to get user callback on HW started
  modem_connected = 0; //reset to get user callback on Modem UP
  Modem_Receive_Data();
  Set_Modem_Counter_Variables( );
  Set_Modem_Control_Variables( );
  
  modem_event_init(&modem_instances.event_buff, 50); //max 50 events can be Q'ed (Event Buffer is of size 50)
  
  Modem_Start_Timer();  
  memset(modem_open_sockets,0x00, 5); //init the open socket array
  
  if(HAL_TIM_Base_Start_IT(&modemTimHandle) != HAL_OK)//Start the TIM timer
    {
      #if MODEM_PRINT_DEBUG
      printf("Error");
      #endif
      Error_Handler();
    }
}

/**
  * @brief  Modem_Module_Init
  *         Initialize modem module
  * @param  None
  * @retval None
  */
void Modem_Module_Init(void)
{
  memset(modem_UserDataBuff, 0x00, MODEM_MAX_BUFFER_GLOBAL);//Flush the buffer;
  memset(modem_pop_buffer, 0x00 , MODEM_MAX_BUFFER_GLOBAL);
  memset(modem_network_time_buff, 0x00 , MAX_NETWORK_TIME_SIZE);
  modem_buffer_init(&modem_instances.big_buff, MODEM_RINGBUF_SIZE);//Init the ring buffer  
  //IO_modem_status_flag.modem_ready = 0; //reset to get user callback on HW started
  modem_connected = 0; //reset to get user callback on Modem UP
  Modem_Receive_Data();
  Set_Modem_Counter_Variables( );
  Set_Modem_Control_Variables( );
  
  modem_event_init(&modem_instances.event_buff, 50); //max 50 events can be Q'ed (Event Buffer is of size 50)
   
  memset(modem_open_sockets,0x00, 5); //init the open socket array

}

/**
* @brief  Set_Modem_Control_Variables
*         Sets the default value of some control Variables
* @param  None
* @retval None
*/
void Set_Modem_Control_Variables(void)
{
  IO_modem_status_flag.enable_dequeue                      = MODEM_TRUE;  
  IO_modem_status_flag.Modem_State                         = Modem_Undef_State; 
  IO_modem_status_flag.Modem_Sync_State                    = MODEM_FALSE;
  IO_modem_status_flag.UartReady                           = RESET;
  IO_modem_status_flag.TxUartReady                         = RESET;
  IO_modem_status_flag.AT_Response_Received                = MODEM_FALSE;
  IO_modem_status_flag.CONNECT_Response_Received           = MODEM_FALSE;     
  IO_modem_status_flag.QSSLOPEN_Response_Received          = MODEM_FALSE;
  IO_modem_status_flag.QSSLSEND_Response_Received          = MODEM_FALSE;     
  IO_modem_status_flag.QISEND_Response_Received            = MODEM_FALSE;     
  
  IO_modem_status_flag.simcard                             = MODEM_FALSE;
  
  IO_modem_status_flag.IPSEND_Response_Received            = MODEM_FALSE;
  IO_modem_status_flag.IPSEND_DONE_Response_Received       = MODEM_FALSE;
  
  IO_modem_status_flag.sock_read_ongoing                   = MODEM_FALSE;
  IO_modem_status_flag.prevent_push_OK_event               = MODEM_FALSE;
  IO_modem_status_flag.Timer_Running                       = MODEM_FALSE;
  IO_modem_status_flag.Modem_Enabled                       = MODEM_FALSE;
  IO_modem_status_flag.data_mode                           = MODEM_FALSE;
  IO_modem_status_flag.tickcount                           = 0;
  IO_modem_status_flag.client_socket_close_ongoing         = MODEM_FALSE;
    
  Modem_Control_Variables.Modem_Configuration_Done               =  MODEM_FALSE;
  Modem_Control_Variables.stop_event_dequeue                     =  MODEM_FALSE;
  Modem_Control_Variables.AT_Cmd_Ongoing                         =  MODEM_FALSE;
  Modem_Control_Variables.AT_Cmd_Processing                      =  MODEM_FALSE;
  Modem_Control_Variables.Uartx_Rx_Processing                    =  MODEM_FALSE;
  Modem_Control_Variables.Client_Connected                       =  MODEM_FALSE;
  Modem_Control_Variables.Client_Disconnected                    =  MODEM_FALSE;
  Modem_Control_Variables.start_sock_read                        =  MODEM_FALSE;
  Modem_Control_Variables.enable_receive_data_chunk              =  MODEM_FALSE;
  Modem_Control_Variables.data_pending_sockD                     =  MODEM_FALSE;
  Modem_Control_Variables.enable_sock_read                       =  MODEM_FALSE;
  Modem_Control_Variables.enable_query                           =  MODEM_FALSE;
  Modem_Control_Variables.prevent_push_MODEM_event               =  MODEM_FALSE;
  Modem_Control_Variables.message_pending                        =  MODEM_FALSE;
  Modem_Control_Variables.Pending_SockON_Callback                =  MODEM_FALSE;
  Modem_Control_Variables.Pending_SockD_Callback                 =  MODEM_FALSE;
  Modem_Control_Variables.SockON_Server_Closed_Callback          =  MODEM_FALSE;
  Modem_Control_Variables.Client_Socket_Close_Cmd                =  MODEM_FALSE;
  Modem_Control_Variables.resume_receive_data                    =  MODEM_FALSE;
  Modem_Control_Variables.enable_timeout_timer                   =  MODEM_FALSE;
  Modem_Control_Variables.enable_sock_data                       =  MODEM_FALSE;
  //enable SockOn_Server_Closed_Callback when wind:58 received, not when user requests for socket close.
  Modem_Control_Variables.enable_SockON_Server_Closed_Callback   =  MODEM_TRUE;
  Modem_Control_Variables.AT_RESPONSE                            =  MODEM_MODULE_SUCCESS;
  
  
}

/**
* @brief  Set_Modem_Counter_Variables
*         Sets the default value of some counter Variables
* @param  None
* @retval None
*/
void Set_Modem_Counter_Variables(void)
{
  Modem_Counter_Variables.no_of_open_client_sockets = 0;
  Modem_Counter_Variables.Socket_Data_Length        = 0;
  Modem_Counter_Variables.interim_number_of_bytes   = 0;
  Modem_Counter_Variables.pop_buffer_size           = 0;
  Modem_Counter_Variables.last_process_buffer_index = 5;
  Modem_Counter_Variables.epoch_time                = 0;
  Modem_Counter_Variables.sleep_count               = 0;
  Modem_Counter_Variables.standby_time              = 0;
  Modem_Counter_Variables.timeout_tick              = 0;
  
}


/**
* @brief  Modem_Receive_Data
*         Receive data from UART port
* @param  uint8_t number of bytes to be received
* @retval None
*/
void Modem_Receive_Data(void)
{
  modem_instances.receive_status = HAL_UART_Receive_IT(&UartModemHandle, (uint8_t *)Modem_Counter_Variables.uart_byte, 1);
  if(modem_instances.receive_status!=HAL_OK)
    {
      #if MODEM_PRINT_DEBUG
      printf("HAL_UARTx_Receive_IT Error");
      #endif
    }
  else 
    {
      Modem_Control_Variables.Uartx_Rx_Processing = MODEM_TRUE;
    }
}

extern uint8_t toby;

/**
* @brief  Period elapsed callback in non blocking mode
*         This timer is used for calling back User registered functions with information
*         interrupt every 10ms - processing time ~2.5usec
* @param  htim : TIM handle
* @retval None
*/
void Modem_TIM_Handler(TIM_HandleTypeDef *htim)
{
  /**********************************************************************
  *                                                                     *
  *       Be careful not to make a blocking                             *
  *       call from this function, see                                  *
  *       example Socket_Read() and Socket_Close()                      *
  *                                                                     *
  **********************************************************************/
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
    
  if(Modem_Control_Variables.stop_event_dequeue == MODEM_FALSE)
  {
    __disable_irq();
     modem_instances.DeQed_modem_event = modem_pop_eventbuffer_queue(&modem_instances.event_buff);
    __enable_irq();

    if(modem_instances.DeQed_modem_event!=NULL && modem_instances.DeQed_modem_event->event_pop == MODEM_TRUE)
    {
      switch(modem_instances.DeQed_modem_event->event)
      {
        case MODEM_OK_EVENT:
          IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
          Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS; 
          break;
          
      case MODEM_SIM_STATUS:
          //IO_modem_status_flag.simcard =  MODEM_TRUE;
          break;
          
        case MODEM_ATTACHED:  
          IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
          Modem_Control_Variables.AT_RESPONSE = MODEM_ATTACHED_OK;
          break;
          
        case MODEM_LATEST_NETWORK_TIME_SYNC:      
          IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
          /*if response is "" then is still waiting */
          if ((strstr((const char *)modem_network_time_buff,"\"\"")) != NULL)
          {
            Modem_Control_Variables.AT_RESPONSE = MODEM_WAITING_GET_TIME;
          }
          else
          {
            Modem_Control_Variables.AT_RESPONSE = MODEM_TIME_UPDATED;                          
          }
          break;  

        case MODEM_GET_CLOCK_TIME:
          IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
          /*if response is "" then is still waiting */
          if (((strstr((const char *)modem_network_time_buff,"\"\"")) != NULL) || (strlen(modem_network_time_buff)==0))
          {
            Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_ERROR;
          }
          else
          {
            Modem_Control_Variables.AT_RESPONSE = MODEM_CLOCK_RESPONSE_OK;  
          }
          break;  
                      
        case MODEM_TCPIP_TASK_EVENT:
          IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
          break; 
          
      case MODEM_CLIENT_SOCKET_STATUS_EVENT:
          /* AT#SOCKETCREATE? */  
          sprintf((char*)Modem_AT_Cmd_Buff,"AT#SOCKETCREATE?\r");
          status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));          
          if(status != MODEM_MODULE_SUCCESS)
          {
            IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
            Modem_Control_Variables.AT_RESPONSE = MODEM_AT_CMD_RESP_ERROR;
          }
          else
          {
            Modem_Counter_Variables.timeout_tick = 0;
            Modem_Control_Variables.enable_timeout_timer = MODEM_TRUE;
          }                              
          break;
         
        case MODEM_DNS_GET_IP:
          /* AT#DNS */
          sprintf((char*)Modem_AT_Cmd_Buff,"at#dns=5,0,%s\r",Modem_Counter_Variables.curr_hostname);
          status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));          
          if(status != MODEM_MODULE_SUCCESS)
          {
            IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
            Modem_Control_Variables.AT_RESPONSE = MODEM_AT_CMD_RESP_ERROR;
          }
          else
          {
            Modem_Counter_Variables.timeout_tick = 0;
            Modem_Control_Variables.enable_timeout_timer = MODEM_TRUE;
          }          
          break;
          
        case MODEM_CLIENT_SOCKET_OPEN_EVENT:  
          Modem_Reset_AT_CMD_Buffer();
          /* AT#SOCKETCREATE */  
          if (strchr(Modem_Counter_Variables.curr_protocol,'T')) //T of TCP
          {
            sprintf((char*)Modem_AT_Cmd_Buff,"AT#SOCKETCREATE=5,0,%s,%d,100,100,1\r",Modem_Counter_Variables.curr_protocol,(int)Modem_Counter_Variables.curr_port_number);
          }
          else if (strchr(Modem_Counter_Variables.curr_protocol,'U')) //U of UDP 
          {
            sprintf((char*)Modem_AT_Cmd_Buff,"AT#SOCKETCREATE=5,0,UDP,%d,1,1,1\r", (int)Modem_Counter_Variables.curr_port_number);             
          }
          status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));          
          if(status != MODEM_MODULE_SUCCESS)
          {
            IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
            Modem_Control_Variables.AT_RESPONSE = MODEM_AT_CMD_RESP_ERROR;
          }
          else
          {
            Modem_Counter_Variables.timeout_tick = 0;
            Modem_Control_Variables.enable_timeout_timer = MODEM_TRUE;
          }                    
          break;       
          
        case MODEM_MQTT_CLIENT_SOCKET_OPEN_EVENT:  
            /* AT#MQTTCONNECT */  
            //sprintf((char*)Modem_AT_Cmd_Buff,"AT#MQTTCONNECT=5,0,%s,%d\r", Modem_Counter_Variables.curr_hostname, (int)Modem_Counter_Variables.curr_port_number); 
            sprintf((char*)Modem_AT_Cmd_Buff,"AT#MQTTCONNECT=5,0,162.248.102.207,1883\r"); //,nano,stmicro123\r");  //5,0,%s,%d\r", Modem_Counter_Variables.curr_hostname, (int)Modem_Counter_Variables.curr_port_number); 
            status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));          
            if(status != MODEM_MODULE_SUCCESS)
            {
              IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
              Modem_Control_Variables.AT_RESPONSE = MODEM_AT_CMD_RESP_ERROR;
            }
            else
            {
              Modem_Counter_Variables.timeout_tick = 0;
              Modem_Control_Variables.enable_timeout_timer = MODEM_TRUE;
              IO_modem_status_flag.MQTTOPEN_Send = MODEM_TRUE;
              
            }            
            break;
            
        case MODEM_MQTT_CLIENT_PUBLISH_MESSAGE:
            /* AT#MQTTPUB */  
            sprintf((char*)Modem_AT_Cmd_Buff,"AT#MQTTPUB=%s,%d,3,0\r",Modem_Counter_Variables.topic,Modem_Counter_Variables.topic_message);//Modem_Counter_Variables.curr_hostname, (int)Modem_Counter_Variables.curr_port_number); 
            status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));          
            if(status != MODEM_MODULE_SUCCESS)
            {
              IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
              Modem_Control_Variables.AT_RESPONSE = MODEM_AT_CMD_RESP_ERROR;
            }
            else
            {
              Modem_Counter_Variables.timeout_tick = 0;
              Modem_Control_Variables.enable_timeout_timer = MODEM_TRUE;
              IO_modem_status_flag.MQTTPUB_Send = MODEM_TRUE;
            }           
            break;
            
        case MODEM_MQTT_CLIENT_SUBSCRIBE_MESSAGE:    
            /* AT#MQTTSUB */  
            sprintf((char*)Modem_AT_Cmd_Buff,"AT#MQTTSUB=light_status,0,dimmer,0\r",Modem_Counter_Variables.topic,Modem_Counter_Variables.topic_message);//Modem_Counter_Variables.curr_hostname, (int)Modem_Counter_Variables.curr_port_number); 
            status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));          
            if(status != MODEM_MODULE_SUCCESS)
            {
              IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
              Modem_Control_Variables.AT_RESPONSE = MODEM_AT_CMD_RESP_ERROR;
            }
            else
            {
              Modem_Counter_Variables.timeout_tick = 0;
              Modem_Control_Variables.enable_timeout_timer = MODEM_TRUE;
              IO_modem_status_flag.MQTTSUB_Send = MODEM_TRUE;
            }           
          
            break;
            
        case MODEM_SOCK_ID_EVENT:
          /*check ID and update SocketID array*/
          Modem_Counter_Variables.no_of_open_client_sockets++;
          Modem_Control_Variables.enable_timeout_timer = MODEM_FALSE;
          Modem_Counter_Variables.timeout_tick = 0;
          if(Modem_Counter_Variables.no_of_open_client_sockets > 3)  //Max number of clients is 3
          {
            IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
            Modem_Control_Variables.AT_RESPONSE = MODEM_NOT_SUPPORTED;     
            break;
          }
          Modem_Counter_Variables.Socket_Open_ID = modem_instances.DeQed_modem_event->socket_sid;
          modem_open_sockets[modem_instances.DeQed_modem_event->socket_sid++]  = MODEM_TRUE;  //to bo be fixed
          //Modem_Counter_Variables.Socket_Open_ID = modem_instances.DeQed_modem_event->socket_sid;
          IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
          Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
          //IO_modem_status_flag.QIOPEN_Response_Received = MODEM_TRUE;
          break;
          
        case MODEM_CONNECT_FAIL_EVENT:
          IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
          Modem_Control_Variables.AT_RESPONSE = MODEM_CONNECT_SOCKET_FAIL;
          IO_modem_status_flag.QIOPEN_Response_Received = MODEM_FALSE;
          break; 
        
        case MODEM_ALREADY_CONNECT_EVENT:
          while(1)
          {
            //JRF to be implemented
          }
          break;
          
        case MODEM_CLIENT_SOCKET_READ_DATA:
          Modem_Reset_AT_CMD_Buffer();                          
                          
          /* AT#IPREAD = Retrieve the Received TCP/IP Data */
          sprintf((char*)Modem_AT_Cmd_Buff,"AT#IPREAD=5,0\r");//, 0,1,0 /*Modem_Counter_Variables.curr_sockID*/,MODEM_MAX_RECEIVE_BYTES_SIZE); //MODEM_MAX_RECEIVE_BYTES_SIZE);
          status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
          
          if(status != MODEM_MODULE_SUCCESS)
          {
            #if MODEM_PRINT_DEBUG
            printf("\r\n ERROR In Socket Write\r\n");
            #endif
            IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
            Modem_Control_Variables.AT_RESPONSE = MODEM_AT_CMD_RESP_ERROR;
          }
          IO_modem_status_flag.sock_read_ongoing = MODEM_TRUE;
          break;          
        
        case MODEM_CLIENT_SOCKET_WRITE_EVENT:
          Modem_Reset_AT_CMD_Buffer();     
          if (sckt_type == UDP_TYPE)
          {
            sprintf((char*)Modem_AT_Cmd_Buff,"AT#IPSENDUDP=5,0,%s,%d,0,1,%d\r\n",Modem_Counter_Variables.curr_hostname,(int)Modem_Counter_Variables.curr_port_number,Modem_Counter_Variables.curr_DataLength);          
          }
          else
          {
            sprintf((char*)Modem_AT_Cmd_Buff,"AT#IPSENDTCP=5,0,1,%d\r\n",Modem_Counter_Variables.curr_DataLength); 
          }
          IO_modem_status_flag.IPSEND_Response_Received = MODEM_TRUE;   
          status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));           
          
          if(status != MODEM_MODULE_SUCCESS)
          {
            #if MODEM_PRINT_DEBUG
              printf("\r\n ERROR In Socket Write\r\n");
            #endif
            IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
            Modem_Control_Variables.AT_RESPONSE = MODEM_AT_CMD_RESP_ERROR;
            IO_modem_status_flag.IPSEND_Response_Received = MODEM_FALSE;
          }           
          break;

        case MODEM_CLIENT_SOCKET_WRITE_DATA:

            Modem_Reset_AT_CMD_Buffer();
            memcpy((char*)Modem_AT_Cmd_Buff, (char*)Modem_Counter_Variables.curr_data, Modem_Counter_Variables.curr_DataLength);   
            Modem_AT_Cmd_Buff [Modem_Counter_Variables.curr_DataLength] = '\r'; 
            //Modem_Counter_Variables.curr_DataLength++;
            status = Modem_Transmit_AT_Cmd( Modem_Counter_Variables.curr_DataLength);  
            IO_modem_status_flag.IPSEND_DONE_Response_Received  = MODEM_TRUE;

          break;          
          
        case MODEM_CLIENT_SOCKET_WRITE_OK:
            IO_modem_status_flag.QISEND_Response_Received = MODEM_TRUE;
            break;    
            
        case MODEM_TCP_CONNECTION_EVENT:
            Modem_Reset_AT_CMD_Buffer();
            sprintf((char*)Modem_AT_Cmd_Buff,"AT#TCPCONNECT=5,%d,%s,%d\r",modem_instances.modem_event.socket_sid,server_ip,server_port); //SERVER_PORT);                  
            status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
            if(status == MODEM_MODULE_SUCCESS)
            {
              Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;            
            }
            else
            {
              #if MODEM_PRINT_DEBUG
                printf("\r\n ERROR During Socket Close \r\n");
              #endif
            }
            IO_modem_status_flag.TCP_CONNECT_Send = MODEM_TRUE;
            break; 
            
        case MODEM_CLIENT_SOCKET_CLOSE_EVENT:
          //modem_instances.DeQed_modem_event->socket_id = 0;  //JRF to be fixed
          if(modem_open_sockets[modem_instances.DeQed_modem_event->socket_id])
          {
            modem_instances.DeQed_modem_event->socket_sid--;
            Modem_Reset_AT_CMD_Buffer();

            /* AT+QICLOSE<cr>  - Close a TCP/UDP Connection */
            //sprintf((char*)Modem_AT_Cmd_Buff,"AT+QICLOSE=%d\r"  , modem_instances.DeQed_modem_event->socket_id);
            
            sprintf((char*)Modem_AT_Cmd_Buff,"AT#SOCKETCLOSE=5,%d\r",modem_instances.modem_event.socket_sid);//Modem_Counter_Variables.curr_sockID);//+QICLOSE=%d\r"  , modem_instances.DeQed_modem_event->socket_id);
                                
            status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
            if(status == MODEM_MODULE_SUCCESS)
            {
              Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
              Modem_Control_Variables.stop_event_dequeue = MODEM_TRUE;
              Modem_Counter_Variables.remote_socket_closed_id = modem_instances.DeQed_modem_event->socket_id;
                                    
              //for making changes in the value of open_sockets[sock_id] if no error is returned
              IO_modem_status_flag.client_socket_close_ongoing = MODEM_TRUE;
                                    
              //prevent the OK received after socket close command to be Q'ed
              IO_modem_status_flag.prevent_push_OK_event       = MODEM_TRUE;

              Modem_Control_Variables.stop_event_dequeue = MODEM_FALSE;
              Modem_Control_Variables.data_pending_sockD = MODEM_FALSE;
              Modem_Control_Variables.enable_sock_read   = MODEM_FALSE;
              Modem_Control_Variables.enable_receive_data_chunk = MODEM_FALSE;
              //Modem_Control_Variables.chunk_size         = 0;
              //Modem_Control_Variables.message_size       = 0;                
            }
            else
            {
              #if MODEM_PRINT_DEBUG
                printf("\r\n ERROR During Socket Close \r\n");
              #endif
            }
          }
          else
            #if MODEM_PRINT_DEBUG
              printf("\r\n Socket already close");
            #endif
                    
          break;
          
        case MODEM_DEACT_EVENT:
          IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
          Modem_Control_Variables.AT_RESPONSE = MODEM_DEACT_OK;
          break;
        
        case MODEM_NO_EVENT:
          break;
          
        //default: 
          
      }
    }

    /* If data is pending on client socket SOCKON, make read requests*/
    if(Modem_Control_Variables.start_sock_read == MODEM_TRUE)
    {
      if (getNTP_time)
      {
        Modem_Counter_Variables.Socket_Data_Length = 4;
        getNTP_time = 0;
      }
      Modem_Socket_Read(Modem_Counter_Variables.Socket_Data_Length);
      Modem_Control_Variables.start_sock_read = MODEM_FALSE;
    }
    /* Call Query, after notification for TLS is received */
    else if(Modem_Control_Variables.enable_query == MODEM_TRUE && IO_modem_status_flag.enable_dequeue == MODEM_TRUE)
    {
      //@TBD: Flushing the buffer may be detrimental if we have genuine follow on WIND55?
      Modem_Socket_Pending_Data(); //JRF to check
      Modem_Control_Variables.enable_query = MODEM_FALSE;
    }
    else if(Modem_Control_Variables.Pending_SockON_Callback==MODEM_TRUE)//for client socket
    {
      //Now callback to user with user_data pointer <UserDataBuff>    
      ind_modem_socket_data_received(Modem_Counter_Variables.sockon_id_user, (uint8_t *)modem_UserDataBuff, Modem_Counter_Variables.message_size, Modem_Counter_Variables.message_size); //Modem_Counter_Variables.chunk_size);
      /* check if the message has the expected size - 3 bytes are expected from the webserver */
      if (Modem_Counter_Variables.message_size <=3)
      {
        led_cloud.update_state = 1;
        led_cloud.state        =  modem_UserDataBuff[0] - 0x30;    // lamp status
        led_cloud.dimmer       =  modem_UserDataBuff[1];           //JRF dimmering value
        led_cloud.tti          =  modem_UserDataBuff[2];           //time between transmissions 
        osMessageQueuePut(LedStatusQueueHandle,&led_cloud , 0, 0); //to be read at metro task
      }
      else
      {  /* if the size is bigger then 3 bytes flush the buffer */
         memset(modem_UserDataBuff, 0x00, MODEM_MAX_BUFFER_GLOBAL);//Flush the buffer
      }
      
      
      HAL_GPIO_WritePin(LED_COMM_SENSOR_GPIO_Port, LED_COMM_SENSOR_Pin, GPIO_PIN_SET);
      //memset(modem_UserDataBuff, 0x00, MODEM_MAX_BUFFER_GLOBAL);//Flush the buffer
      Modem_Counter_Variables.chunk_size = 0; 
      Modem_Counter_Variables.message_size = 0; 
      Modem_Resume_Dequeue();
      Modem_Control_Variables.Pending_SockON_Callback=MODEM_FALSE;
    }
    else if(Modem_Control_Variables.Pending_SockD_Callback == MODEM_TRUE)//for server socket
    {
      //Now callback to user with user_data pointer <UserDataBuff>
      ind_modem_socket_data_received(9, (uint8_t *)modem_UserDataBuff, Modem_Counter_Variables.message_size, Modem_Counter_Variables.message_size);//Modem_Counter_Variables.chunk_size);
      memset(modem_UserDataBuff, 0x00, MODEM_MAX_BUFFER_GLOBAL); //Flush the buffer
      Modem_Counter_Variables.chunk_size = 0; 
      Modem_Counter_Variables.message_size = 0; 
      Modem_Resume_Dequeue();
      Modem_Control_Variables.Pending_SockD_Callback=MODEM_FALSE; 
      Modem_Control_Variables.stop_event_dequeue = MODEM_FALSE; //JRF precisa achar melhor lugar
    }
    else if(Modem_Control_Variables.Client_Socket_Close_Cmd == MODEM_TRUE)//for client socket
    {
      // Q the close socket event
      if(modem_open_sockets[Modem_Counter_Variables.client_socket_close_id])
      {
        Modem_Queue_Client_Close_Event(Modem_Counter_Variables.client_socket_close_id);//,0);
      }
      Modem_Control_Variables.Client_Socket_Close_Cmd = MODEM_FALSE;
    }
    else if(Modem_Control_Variables.SockON_Server_Closed_Callback==MODEM_TRUE)//for client socket
    {
      //callback the user
      ind_modem_socket_client_remote_server_closed(&Modem_Counter_Variables.closed_socket_id);
      Modem_Control_Variables.SockON_Server_Closed_Callback = MODEM_FALSE;
    }
  }
}

/**
* @brief  Modem_Start_Timer
*         Start Timer 
* @param  None
* @retval None
*/
void Modem_Start_Timer()
{
  IO_modem_status_flag.tickcount = MODEM_FALSE;
  IO_modem_status_flag.Timer_Running = MODEM_TRUE;
}

/**
* @brief  Modem_Stop_Timer
*         Stop Timer request
* @param  None
* @retval None
*/
void Modem_Stop_Timer()
{  
  IO_modem_status_flag.tickcount      = MODEM_FALSE;  
  IO_modem_status_flag.Timer_Running  = MODEM_FALSE;    
  IO_modem_status_flag.UartReady      = SET;
}

/**
* @brief  Modem_Stop_Dequeue
*         Stop dequeuing data from the ring buffer
* @param  None
* @retval None
*/
void Modem_Stop_Dequeue()
{
  IO_modem_status_flag.enable_dequeue = MODEM_FALSE;
}

/**
* @brief  Modem_Resume_Dequeue
*         Resume dequeuing data from the ring buffer
* @param  None
* @retval None
*/
void Modem_Resume_Dequeue()
{
  IO_modem_status_flag.enable_dequeue = MODEM_TRUE;
}

/**
* @brief  Modem_SysTick_Isr
*         Function called every SysTick to process buffer
* @param  None
* @retval None
*/
void Modem_SysTick_Isr()
{
    //HAL_GPIO_TogglePin(LED_COMM_MET_GPIO_Port, LED_COMM_MET_Pin);
    //Check if Data is Paused
    if((IO_modem_status_flag.Timer_Running) && (IO_modem_status_flag.enable_dequeue==MODEM_TRUE) /*&& ((tickcount++) >= PROCESS_MODEM_TIMER)*/)
      {
         Process_Modem();
      }

    if(Modem_Control_Variables.resume_receive_data == MODEM_TRUE)
      {
          if(modem_buffer_is_half_empty(&modem_instances.big_buff))
            {
                Modem_Control_Variables.resume_receive_data = MODEM_FALSE;
                Modem_Receive_Data();
            }
      }

    if(Modem_Control_Variables.enable_timeout_timer)     // module will timeout when no response from server received
      {
          Modem_Counter_Variables.timeout_tick++;
          //wait for 20 seconds before timeout
          if(Modem_Counter_Variables.timeout_tick > SERVER_CONNECTION_TIMEOUT)       //wait for 20s before timeout
            {
                #if MODEM_PRINT_DEBUG
                  printf("\r\n Timeout! No response received.\r\n");
                #endif
                Modem_Counter_Variables.timeout_tick         = 0;
                Modem_Control_Variables.enable_timeout_timer = MODEM_FALSE;
                Modem_Control_Variables.AT_RESPONSE          = MODEM_AT_CMD_RESP_ERROR; // Timeout if no response received.
                IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
                 //re-enable event Q after 200ms
                Modem_Control_Variables.stop_event_dequeue = MODEM_FALSE;
            }
      }
      

    
}

//extern UART_HandleTypeDef huart1;
//static uint8_t dummy_byte;
/**
* @brief  Modem_HAL_UART_TxCpltCallback
*         Tx Transfer completed callback
* @param  UsartHandle: UART handle 
* @retval None
*/
void Modem_HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{
#ifdef MODEM_USE_VCOM
  if (UartHandleArg==&UartModemMsgHandle)
    console_echo_ready = SET;
#else
  /* Set transmission flag: transfer complete */
  //JRF to be fixed workaround
  IO_modem_status_flag.TxUartReady = SET; 
  //dummy_byte = USART1->RDR;
  //__HAL_UART_ENABLE_IT(huart1., UART_IT_RXNE);
#endif
}

/**
* @brief  Modem_HAL_UART_RxCpltCallback
*         Rx Transfer completed callback
* @param  UsartHandle: UART handle 
* @retval None
*/
void Modem_HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{
  Modem_Control_Variables.Uartx_Rx_Processing = MODEM_FALSE;
  Modem_Stop_Timer();
  
  __disable_irq();
  modem_push_buffer_queue(&modem_instances.big_buff, Modem_Counter_Variables.uart_byte);
  __enable_irq();

#ifdef COMM_DEBUG
  USART4->TDR = USART1->RDR;
#endif  
  
  Modem_Start_Timer();
  
  if(modem_buffer_is_half_full(&modem_instances.big_buff))
  {
    Modem_Control_Variables.resume_receive_data = MODEM_TRUE;
    //HAL_GPIO_WritePin(Modem_USART_RTS_GPIO_PORT, Modem_USART_RTS_PIN, GPIO_PIN_SET);//De-assert RTS
  } 
  else
  {
    if(Modem_Control_Variables.AT_Cmd_Processing == MODEM_FALSE)
    {
      //call Rx only if TX is not under processing (AT command)
      modem_instances.receive_status = HAL_UART_Receive_IT(&UartModemHandle, (uint8_t *)Modem_Counter_Variables.uart_byte, 1);
      if(modem_instances.receive_status!=HAL_OK)
      {
        #if MODEM_PRINT_DEBUG 
        printf("HAL_UARTx_Receive_IT Error");
        #endif
      }
      else 
      {
        Modem_Control_Variables.Uartx_Rx_Processing = MODEM_TRUE;
        //HAL_GPIO_TogglePin(LED_COMM_SENSOR_GPIO_Port, LED_COMM_SENSOR_Pin); //JRF
      }
    }
  }
}  

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA channel
  *         used for USART data reception
  */
void USART_MODEM_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartModemHandle.hdmatx); //JRF
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA channel
  *         used for USART data reception
  */
void USART_MODEM_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartModemHandle.hdmarx);
}

 //static uint32_t start_time,result=0;   
/**
* @brief  USART_Receive_AT_Resp
*         Receive and check AT cmd response
* @param  None
* @retval Modem_Status_t : Response of AT cmd  
*/

Modem_Status_t Modem_Receive_AT_Resp(uint32_t wait_ms )
{
  modemTimeout = wait_ms;

  while ((modemTimeout!=0)&&(IO_modem_status_flag.AT_Response_Received != MODEM_TRUE))
  {
    __NOP();
  }
  if (IO_modem_status_flag.AT_Response_Received == MODEM_FALSE)
  {
    Modem_Control_Variables.AT_RESPONSE = MODEM_TIME_OUT_ERROR;    
  }
  IO_modem_status_flag.AT_Response_Received = MODEM_FALSE;

  return Modem_Control_Variables.AT_RESPONSE;
}



/**
  * @brief  UART error callbacks
  * @param  UsartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void Modem_HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}

/**
* @brief  Process_Modem
*         Pop a byte from the circular buffer and send the byte for processing
*         This function should be called from main or should be run with a periodic timer
* @param  None
* @retval None
*/
void Process_Modem(void)
{
  __disable_irq();
  Modem_Counter_Variables.temp = modem_pop_buffer_queue(&modem_instances.big_buff);   //contents of temp(pop_buffer) will not change till another de-queue is made
  __enable_irq();

  if(Modem_Counter_Variables.temp!=NULL) 
  {
    Process_Modem_Buffer(Modem_Counter_Variables.temp);    
  }
}

/**
* @brief  Process_Modem_Buffer
*         Process and construct a Line buffer
* @param  ptr: pointer to one single byte
* @retval None
*/

void Process_Modem_Buffer(uint8_t * ptr)
{ 
  static uint32_t Fillptr=0;
  static char databytes_No[5];
  unsigned char rxdata = 0;
  static char * ptr_offset;
  static char * pStr;
  static uint8_t process_buffer[MODEM_MAX_BUFFER_GLOBAL];
  

  rxdata =  *(ptr+0);  
//  Fillptr++;
//  if (Fillptr>=MODEM_MAX_BUFFER_GLOBAL)
//  {
//    Fillptr=0;
//  }
  process_buffer[Fillptr++] = rxdata;   
  //process_buffer[Fillptr] = rxdata;     
  //modem_reset_event(&modem_instances.modem_event); //JRF to be FIXED

  if((process_buffer[Fillptr-2]=='\r') && (process_buffer[Fillptr-1]=='\n') && !IO_modem_status_flag.sock_read_ongoing)
  {//end of msg received. Will not receive any other msg till we process this.

    #if MODEM_PRINT_DEBUG
      printf ("\r\n<%s>",process_buffer);
    #endif
      
    //new implementation  
    /* looking for <CR><LF>OK<CR><LF> - it is an answer to a AT command */  
    if((process_buffer[Fillptr-4]=='O')&&(process_buffer[Fillptr-3]=='K')&&(process_buffer[Fillptr-2]=='\r') && (process_buffer[Fillptr-1]=='\n'))
    { /* command response messages */
      if ((((pStr=strstr((char *)process_buffer,"+CGATT:"))) != NULL))
      { /* Response to AT+CGATT */
        databytes_No[0] = *(pStr + 8);      
        if (databytes_No[0]=='1')           // = '1' = modem connected
        {
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event   = MODEM_ATTACHED; 
          modem_instances.modem_event.event_pop = MODEM_TRUE;
        }
        __disable_irq();
        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
        __enable_irq();
        modem_reset_event(&modem_instances.modem_event);
        /*clear buffer*/
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);                  
      }
      else if ((((pStr=strstr((char *)process_buffer,"#SOCKETCREATE:"))) != NULL))  
      { /* Response to AT#SOCKETCREATE */
        ptr_offset = pStr;
        
        //And now find socket ID
        databytes_No[0] = *(pStr + 15);      //points to the number the ':'
        databytes_No[1] = *(pStr + 16) ;     //find '\r' 
              
        if (databytes_No[0] == '\r')        //no digit found no socket opened
        {
          Modem_Counter_Variables.curr_sockID  =  0xFF;
        }
        else if (databytes_No[1] == '\r')        //then is its a 1 digit number
        {
          databytes_No[0] =  *(pStr + 15);   
          Modem_Counter_Variables.curr_sockID  =  databytes_No[0] - 0x30;
        }      
          
        modem_instances.modem_event.ok_eval = MODEM_TRUE;
        modem_instances.modem_event.event = MODEM_SOCK_ID_EVENT;
        IO_modem_status_flag.QIOPEN_Response_Received = MODEM_TRUE;
        modem_instances.modem_event.event_pop = MODEM_TRUE;
        
        __disable_irq();
        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
        __enable_irq();
        modem_reset_event(&modem_instances.modem_event);        
        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);    
      }          
      else if ((((pStr=strstr((char *)process_buffer,"#IPCFG: "))) != NULL))  
      { /* Response to AT#IPCFG? command */
        
       //pStr = strstr((char *)process_buffer,","); 
        //ptr_offset = pStr;
              
        //And now find the data length
        //databytes_No[0] = *(pStr + 13);      //points to the number just after the second colon      
  //      if ( *(pStr + 13) != NULL)  //we have an IP address
  //      {
  //        ptr_offset = strstr((char *)process_buffer,"\r"); 
  //        
  //        
  //      }
  //      
  //      if ((pStr = strstr((char *)process_buffer,",")) != NULL)
  //      {        
  //        ptr_offset = pStr;
  //        ptr_offset++; 
  //        ptr_offset++;
  //      }
        
        //JRF to checked side efect
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);     
      
        __disable_irq();
        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
        __enable_irq();
        modem_reset_event(&modem_instances.modem_event);        
        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);        
        
      } 
      else if ((((pStr=strstr((char *)process_buffer,"#DNS: "))) != NULL))  
      {
        pStr+=6; //position of the first IP digit
        for (ip_size=0;ip_size<20;ip_size++) //20 is the max size of an ipv6 address
        { 
          if (*pStr != '\r') //search the end of the IP address
          {
            broker_ip[ip_size] = *pStr;
            
          }
          else
          {
            broker_ip[ip_size] = 0; //add null character
            break;
          }
          pStr++;
        }
        IO_modem_status_flag.DNS_Response_Received = MODEM_TRUE;
        //JRF to checked side efect
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);     
      
        __disable_irq();
        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
        __enable_irq();
        modem_reset_event(&modem_instances.modem_event);        
        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);                
      }
      else
      {  
        if (IO_modem_status_flag.IPSEND_Response_Received == MODEM_TRUE)
        {
          IO_modem_status_flag.IPSEND_Response_Received = MODEM_FALSE;
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event = MODEM_CLIENT_SOCKET_WRITE_DATA;
          modem_instances.modem_event.event_pop = MODEM_TRUE;
          __disable_irq();
          modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
          __enable_irq();   
          modem_reset_event(&modem_instances.modem_event);
          //IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
          /*clear buffer*/
          Fillptr=0;
          memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);         
        }
        else if (IO_modem_status_flag.IPSEND_DONE_Response_Received == MODEM_TRUE)
        {
          IO_modem_status_flag.IPSEND_DONE_Response_Received = MODEM_FALSE;
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event = MODEM_CLIENT_SOCKET_WRITE_OK;
          modem_instances.modem_event.event_pop = MODEM_TRUE;
          __disable_irq();
          modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
          __enable_irq();   
          modem_reset_event(&modem_instances.modem_event);
          //IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
          /*clear buffer*/
          Fillptr=0;
          memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);             
        }
        else if (IO_modem_status_flag.TCP_CONNECT_Send == MODEM_TRUE)
        {
          IO_modem_status_flag.TCP_CONNECT_Send = MODEM_FALSE;
          IO_modem_status_flag.TCP_CONNECT_Response_Received = MODEM_TRUE;
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event = MODEM_OK_EVENT;
          modem_instances.modem_event.event_pop = MODEM_TRUE;
          __disable_irq();
          modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
          __enable_irq();   
          modem_reset_event(&modem_instances.modem_event);
          /*clear buffer*/
          Fillptr=0;
          memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);                      
        }
        else if (IO_modem_status_flag.MQTTOPEN_Send == MODEM_TRUE)
        {
          IO_modem_status_flag.MQTTOPEN_Send = MODEM_FALSE;
          IO_modem_status_flag.MQTTOPEN_Response_Received = MODEM_TRUE;
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event = MODEM_OK_EVENT;
          modem_instances.modem_event.event_pop = MODEM_TRUE;
          __disable_irq();
          modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
          __enable_irq();   
          modem_reset_event(&modem_instances.modem_event);
          //IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
          /*clear buffer*/
          Fillptr=0;
          memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);             
        }    
        else if (IO_modem_status_flag.MQTTPUB_Send == MODEM_TRUE)
        {
          IO_modem_status_flag.MQTTPUB_Send = MODEM_FALSE;
          IO_modem_status_flag.MQTTPUB_Response_Received = MODEM_TRUE;
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event = MODEM_OK_EVENT;
          modem_instances.modem_event.event_pop = MODEM_TRUE;
          __disable_irq();
          modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
          __enable_irq();   
          modem_reset_event(&modem_instances.modem_event);
          //IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
          /*clear buffer*/
          Fillptr=0;
          memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);             
        }     
        else if (IO_modem_status_flag.MQTTSUB_Send == MODEM_TRUE)
        {
          IO_modem_status_flag.MQTTSUB_Send = MODEM_FALSE;
          IO_modem_status_flag.MQTTSUB_Response_Received = MODEM_TRUE;
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event = MODEM_OK_EVENT;
          modem_instances.modem_event.event_pop = MODEM_TRUE;
          __disable_irq();
          modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
          __enable_irq();   
          modem_reset_event(&modem_instances.modem_event);
          //IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
          /*clear buffer*/
          Fillptr=0;
          memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);             
        }         
        else        
        { //This is a standalone OK
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event = MODEM_OK_EVENT;
          modem_instances.modem_event.event_pop = MODEM_TRUE;
          __disable_irq();
          modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
          __enable_irq();   
          modem_reset_event(&modem_instances.modem_event);
          IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
          /*clear buffer*/
          Fillptr=0;
          memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);            
        }
      }
    }    
    /* URC messages - OK is not expected */
    else if ((((pStr=strstr((char *)process_buffer,"#IPRECV:"))) != NULL))
    { /* UDP data indication */
      iprecv_cnt++;
      HAL_GPIO_WritePin(LED_COMM_SENSOR_GPIO_Port, LED_COMM_SENSOR_Pin, GPIO_PIN_RESET);
      modem_instances.modem_event.ok_eval = MODEM_TRUE;
      modem_instances.modem_event.event = MODEM_CLIENT_SOCKET_READ_DATA;
      modem_instances.modem_event.event_pop = MODEM_TRUE;
      __disable_irq();
      modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
      __enable_irq();
      modem_reset_event(&modem_instances.modem_event);        
      IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
      Fillptr=0;
      memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);         
    }
    else if ((((pStr=strstr((char *)process_buffer,"NB_SENT"))) != NULL))
    { /* UDP message transmitted by eNode */
      modem_instances.modem_event.ok_eval = MODEM_TRUE;
      modem_instances.modem_event.event = MODEM_OK_EVENT;
      modem_instances.modem_event.event_pop = MODEM_TRUE;
      __disable_irq();
      modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
      __enable_irq();
      modem_reset_event(&modem_instances.modem_event);        
      IO_modem_status_flag.nb_sent = true;
      get_nb_sent_status=1;
      IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
      Fillptr=0;
      memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);       
    }
    else if ((((pStr=strstr((char *)process_buffer,"ERROR"))) != NULL))
    {
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event = MODEM_OK_EVENT;
          modem_instances.modem_event.event_pop = MODEM_TRUE;
          __disable_irq();
          modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
          __enable_irq();   
          modem_reset_event(&modem_instances.modem_event);
          IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
          /*clear buffer*/
          Fillptr=0;
          memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);        
    }
    else if ((((pStr=strstr((char *)process_buffer,"+CME ERROR:"))) != NULL))
    {
          if ((((pStr=strstr((char *)process_buffer,"2171"))) != NULL)) 
          { /* address not found */
            Modem_Control_Variables.AT_RESPONSE = MODEM_DNS_REQUEST_FAIL;
          }
          if ((((pStr=strstr((char *)process_buffer,"2152"))) != NULL)) 
          { /* tcpip socket error while connecting */
            Modem_Control_Variables.AT_RESPONSE = MODEM_MQTT_CONNECT_SOCKET_FAIL;
          }   
          if ((((pStr=strstr((char *)process_buffer,"2150"))) != NULL)) 
          { /* tcpip socket creation error */
            Modem_Control_Variables.AT_RESPONSE = MODEM_MQTT_CONNECT_SOCKET_FAIL;
          }
          if ((((pStr=strstr((char *)process_buffer,"20"))) != NULL)) 
          { /* memory full */
           // Modem_Control_Variables.AT_RESPONSE = MODEM_DNS_REQUEST_FAIL
          }
          if ((((pStr=strstr((char *)process_buffer,"1120"))) != NULL)) 
          { /* APP ERROR */
            Modem_Control_Variables.AT_RESPONSE = MODEM_DNS_REQUEST_FAIL; //to fix
          }          
          if ((((pStr=strstr((char *)process_buffer,"2154"))) != NULL)) 
          { /* socket issue during data transfer */
            Modem_Control_Variables.AT_RESPONSE = MODEM_SOCKET_FAIL;
          }          
          
          modem_instances.modem_event.ok_eval = MODEM_TRUE;
          modem_instances.modem_event.event = MODEM_OK_EVENT;
          modem_instances.modem_event.event_pop = MODEM_TRUE;
          __disable_irq();
          modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
          __enable_irq();   
          modem_reset_event(&modem_instances.modem_event);
          IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
          /*clear buffer*/
          Fillptr=0;
          memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);        
    }
    else if ((((pStr=strstr((char *)process_buffer,"#SIMST:"))) != NULL))
    {
        ptr_offset = pStr;    
        //And now find the data length
        IO_modem_status_flag.simcard = databytes_No[0] = *(pStr + 8) - 0x30;   //points to the number just after : mark
        modem_instances.modem_event.ok_eval = MODEM_TRUE;
        modem_instances.modem_event.event = MODEM_SIM_STATUS;    
        modem_instances.modem_event.event_pop = MODEM_TRUE;
        __disable_irq();
        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
        __enable_irq();   
        modem_reset_event(&modem_instances.modem_event);
        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
        /*clear buffer*/
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);        
    }
    else if ((((pStr=strstr((char *)process_buffer,"#IPCFG:"))) != NULL))
    {
        
    }
    else if ((((pStr=strstr((char *)process_buffer,"#MQTTRECV:"))) != NULL))
    {
        
    }
    else if ((((pStr=strstr((char *)process_buffer,"#SYSSTART"))) != NULL))
    {
        
    }
    else if ((((pStr=strstr((char *)process_buffer,"#REBOOT_RESET"))) != NULL))
    {
        modem_instances.modem_event.ok_eval = MODEM_TRUE;
        modem_instances.modem_event.event = MODEM_OK_EVENT;
        modem_instances.modem_event.event_pop = MODEM_TRUE;
        __disable_irq();
        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
        __enable_irq();   
        modem_reset_event(&modem_instances.modem_event);
        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
        /*clear buffer*/
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL); 
    }
    else if ((((pStr=strstr((char *)process_buffer,"+CEREG:"))) != NULL))
    {
        modem_instances.modem_event.ok_eval = MODEM_TRUE;
        modem_instances.modem_event.event = MODEM_OK_EVENT;
        modem_instances.modem_event.event_pop = MODEM_TRUE;
        __disable_irq();
        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
        __enable_irq();   
        modem_reset_event(&modem_instances.modem_event);
        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
        /*clear buffer*/
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);       
    }    
    else if ((((pStr=strstr((char *)process_buffer,"+CGEV:"))) != NULL))
    {
      
    }
    else if ((((pStr=strstr((char *)process_buffer,"#SOCKETCLOSED:"))) != NULL))
    {/* socket closed by the server */
        modem_instances.modem_event.ok_eval = MODEM_TRUE;
        modem_instances.modem_event.event = MODEM_OK_EVENT;    
        modem_instances.modem_event.event_pop = MODEM_TRUE;
        __disable_irq();
        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
        __enable_irq();   
        modem_reset_event(&modem_instances.modem_event);
        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
        IO_modem_status_flag.SCKT_CLOSED_Received = MODEM_TRUE;
        /*clear buffer*/
        Fillptr=0;
        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);        
    }
    else
    { /* URC Messages handling */
//        modem_instances.modem_event.ok_eval = MODEM_TRUE;
//        modem_instances.modem_event.event = MODEM_OK_EVENT;
//        modem_instances.modem_event.event_pop = MODEM_TRUE;
//        __disable_irq();
//        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
//        __enable_irq();   
//        modem_reset_event(&modem_instances.modem_event);
//        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
//        /*clear buffer*/
//        Fillptr=0;
//        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);                  
    }

      
      
      
//     
//    //end of the new implementation
    
//    if(((strstr((const char *)process_buffer,"#SLEEP"))) != NULL)
//    { /* Modem is in sleep  */
//      /*unsolicitated response. Just clear buffer and ignore it */
//      Fillptr=0;
//      memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);
//      urc_counter++;
//    }    
//    else if(((strstr((const char *)process_buffer,"#WAKEUP"))) != NULL)
//    { 
//      /*unsolicitated response. Just clear buffer and ignore it */
//      Fillptr=0;
//      memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);
//      urc_counter++;
//    }    
//    else if ((((pStr=strstr((char *)process_buffer,"+CSCON: "))) != NULL))
//    {
//      /*unsolicitated response, just clear buffer eand ignore it */
//      Fillptr=0;
//      memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);  
//      urc_counter++;                  
//    }
     
//    /* get modem clock Time */
//    else if ((strstr((const char *)process_buffer,"+CCLK: \"")) != NULL)
//    {
//      pStr = strstr((char *)process_buffer,"+CCLK");
//      if ((copy_str_between_strs(pStr, "+CCLK:", "OK", modem_network_time_buff, sizeof(modem_network_time_buff))) == E_OK)
//      {
//        modem_instances.modem_event.ok_eval = MODEM_TRUE;
//        modem_instances.modem_event.event   = MODEM_GET_CLOCK_TIME;
//        modem_instances.modem_event.event_pop = MODEM_TRUE;
//        __disable_irq();
//        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
//        __enable_irq();  
//        modem_reset_event(&modem_instances.modem_event);
//        /*clear buffer*/
//        Fillptr=0;
//        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);         
//      }            
//    }       
//
//    else if((((strstr((const char *)process_buffer,"ERROR"))) != NULL))
//    {/* ERROR Message */
//      Fillptr=0;
//      memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);
//    }    
//    else if((strstr((const char *)process_buffer,"OK")) != NULL)
//    {
//      if (IO_modem_status_flag.client_socket_close_ongoing == MODEM_TRUE)
//      { //Response to AT#SOCKETCLOSE
//        modem_reset_event(&modem_instances.modem_event);
//        Modem_Control_Variables.SockON_Server_Closed_Callback = MODEM_TRUE;
//        modem_instances.modem_event.event_pop = MODEM_TRUE; //jrfFALSE;
//      
//        if(Modem_Counter_Variables.no_of_open_client_sockets > 0)
//        {
//          Modem_Counter_Variables.no_of_open_client_sockets--;
//        }
//        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
//        modem_open_sockets[Modem_Counter_Variables.remote_socket_closed_id] = MODEM_FALSE;
//        //socket ID for which OK is received.
//        Modem_Counter_Variables.closed_socket_id = Modem_Counter_Variables.remote_socket_closed_id;
//        IO_modem_status_flag.client_socket_close_ongoing = MODEM_FALSE;
//
//        Modem_Control_Variables.stop_event_dequeue = MODEM_FALSE;
//        Modem_Control_Variables.data_pending_sockD = MODEM_FALSE;
//        Modem_Control_Variables.enable_sock_read   = MODEM_FALSE;
//        Modem_Control_Variables.enable_receive_data_chunk = MODEM_FALSE;
//        //Modem_Control_Variables.chunk_size         = 0;
//        //Modem_Control_Variables.message_size       = 0;      
//       
//        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
//        Fillptr=0;
//        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);                  
//        
//      }
//      else if (IO_modem_status_flag.IPSEND_Response_Received == MODEM_TRUE)
//      {
//        IO_modem_status_flag.IPSEND_Response_Received = MODEM_FALSE;
//        modem_instances.modem_event.ok_eval = MODEM_TRUE;
//        modem_instances.modem_event.event = MODEM_CLIENT_SOCKET_WRITE_DATA;
//        modem_instances.modem_event.event_pop = MODEM_TRUE;
//        __disable_irq();
//        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
//        __enable_irq();   
//        modem_reset_event(&modem_instances.modem_event);
//        //IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
//        /*clear buffer*/
//        Fillptr=0;
//        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);         
//      }

////      else if (IO_modem_status_flag.QIOPEN_Response_Received == MODEM_TRUE)
////      {
////        modem_instances.modem_event.ok_eval = MODEM_TRUE;
////        //modem_instances.modem_event.event = MODEM_SOCK_ID_EVENT;
////        //IO_modem_status_flag.QIOPEN_Response_Received = MODEM_TRUE;
////        //IO_modem_status_flag.QIOPEN_Response_Received = MODEM_FALSE;
////        modem_instances.modem_event.event_pop = MODEM_TRUE;
////        __disable_irq();
////        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
////        __enable_irq();
////        modem_reset_event(&modem_instances.modem_event);        
////        IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
////        Fillptr=0;
////        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);           
////      }
//      else
//      {
//        
//        //This is a standalone OK
//        modem_instances.modem_event.ok_eval = MODEM_TRUE;
//        modem_instances.modem_event.event = MODEM_OK_EVENT;
//        modem_instances.modem_event.event_pop = MODEM_TRUE;
//        __disable_irq();
//        modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
//        __enable_irq();   
//        modem_reset_event(&modem_instances.modem_event);
//        //IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
//        /*clear buffer*/
//        Fillptr=0;
//        memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);            
//      }
//    }  
//    else
//    { /* No Valid Frame was found before "\r\n" */
//      // do nothing
//    }       
  }
  if ((IO_modem_status_flag.sock_read_ongoing == MODEM_TRUE)
    &&((process_buffer[Fillptr-4]=='O') && (process_buffer[Fillptr-3]=='K'))    
    &&((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA)))
  {
    if ((((pStr=strstr((char *)process_buffer,"#IPREAD: "))) != NULL))  
    {  /* Response to AT#IPREAD -> UDP data to be read */
      ipread_cnt++; 
      
      modem_instances.modem_event.ok_eval = MODEM_TRUE;
      //JRF???modem_instances.modem_event.event = MODEM_OK_EVENT; //jrf to be fixed
      modem_instances.modem_event.event_pop = MODEM_TRUE;      
      
      ptr_offset = pStr;
/*
#IPREAD: 5,0,3
0x35 0x3b dim tti
*/            
      //And now find the data length
      databytes_No[0] = *(pStr + 13);      //points to the number just after the second colon
      databytes_No[1] = *(pStr + 14) ;     //Max size = 512 bytes
      databytes_No[2] = *(pStr + 15) ;
      databytes_No[3] = *(pStr + 16) ;
      
      /* finding the number of bytes to be read - can be 1 to 1500bytes */
      if (databytes_No[1] == '\r')        //then is its a 1 digit number
      {
        databytes_No[0] =  *(pStr + 13);     
        ptr_offset+= 4+1+2;  
      }
      else if (databytes_No[2] == '\r')  //then is its a 2 digit number
      {
        databytes_No[0] =  *(pStr + 13);               
        databytes_No[1] =  *(pStr + 14);               
        ptr_offset+= 4+2+2;  
      }
      else if (databytes_No[3] == '\r')  //then is its a 3 digit number
      {
        databytes_No[0] =  *(pStr + 13);               
        databytes_No[1] =  *(pStr + 14);               
        databytes_No[2] =  *(pStr + 15);   
        ptr_offset+= 4+3+2;        
      }            
      if( databytes_No[1] == '\r')
      {
        Modem_Counter_Variables.interim_number_of_bytes = databytes_No[0] - '0'; 
      }
      else if( databytes_No[2] == '\r')
      {
        Modem_Counter_Variables.interim_number_of_bytes = (((databytes_No[0] - '0') * 10 ) + (databytes_No[1] - '0'));
      }
      else if( databytes_No[3] == '\r')
      {
        Modem_Counter_Variables.interim_number_of_bytes = (((databytes_No[0] - '0') * 100 ) + ((databytes_No[1] - '0') * 10 ) + (databytes_No[2] - '0'));
      }
      else //it's a 4-digit number
      {
        Modem_Counter_Variables.interim_number_of_bytes = (((databytes_No[0] - '0') * 1000 ) + ((databytes_No[1] - '0') * 100 ) + ((databytes_No[2] - '0') * 10 ) + (databytes_No[3] - '0'));
      }              
            
      modem_instances.modem_event.data_length = Modem_Counter_Variables.interim_number_of_bytes; 
      Modem_Counter_Variables.interim_number_of_bytes = 0;      
    
////    
    
    /*check if all data was already stored on buffer before reading it*/
    if (Fillptr >= (modem_instances.modem_event.data_length+8)) /*8 chars after data == '\r\n\r\nOK\r\n' */
    { 
      if (modem_instances.modem_event.data_length < MODEM_MAX_RECEIVE_BYTES_SIZE) 
      {//all data has been retrieved from the modem 
        Modem_Control_Variables.Pending_SockON_Callback=MODEM_TRUE;
      }
      
      pStr=strstr((char *)process_buffer,"#IPREAD: ");
      pStr +=16; //point to the start of the data => #IPREAD: 5,0,1\r\n

      Modem_Counter_Variables.chunk_size = modem_instances.modem_event.data_length ;
      memcpy(&modem_UserDataBuff[Modem_Counter_Variables.message_size], /*process_buffer*/ pStr, Modem_Counter_Variables.chunk_size);//Fillptr);

      modem_instances.modem_event.ok_eval = MODEM_TRUE;
      //JRF???modem_instances.modem_event.event = MODEM_OK_EVENT; //JRF MODEM_CLIENT_SOCKET_READ_DATA;
      modem_instances.modem_event.event_pop = MODEM_TRUE;

      Modem_Counter_Variables.message_size+=Modem_Counter_Variables.chunk_size; //modem_instances.modem_event.data_length;
      //printf("Chunk Size = %d - Message Size = %d\n\r",Modem_Counter_Variables.chunk_size,Modem_Counter_Variables.message_size);      
      
//      __disable_irq();
//      modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
//      __enable_irq();
//      modem_reset_event(&modem_instances.modem_event); 
//      
//      IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
//      Fillptr=0;
//      memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);         
//      IO_modem_status_flag.sock_read_ongoing = MODEM_FALSE;   
     }
    }
    __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();
    modem_reset_event(&modem_instances.modem_event); 
      
    IO_modem_status_flag.prevent_push_OK_event = MODEM_FALSE;
    Fillptr=0;
    memset(process_buffer, 0x00, MODEM_MAX_BUFFER_GLOBAL);         
    IO_modem_status_flag.sock_read_ongoing = MODEM_FALSE; 
  } 
}

/**
* @brief  Queue_Client_Write_Event
*         Queues a Client Socket write event.
* @param  sock_id socket ID to write to
* @param  DataLength length of the data to be written
* @param  pData pointer to data
* @retval None
*/
void Modem_Queue_Client_Write_Event(uint8_t sock_id, uint16_t DataLength, char * pData)
{
    //JRFWait_For_Sock_Read_To_Complete();
    Modem_Counter_Variables.curr_DataLength = DataLength;
    Modem_Counter_Variables.curr_data = pData;
    Modem_Counter_Variables.curr_sockID = sock_id;

    modem_instances.modem_event.event = MODEM_CLIENT_SOCKET_WRITE_EVENT;
    __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();

    modem_reset_event(&modem_instances.modem_event);
}


/**
* @brief Modem_Queue_Client_Open_Event
*        Queue a Client Open Event
* @param hostname hostname
* @param port_number port number to connect to
* @param protocol protocol required to connect to server (t for TCP socket, u for UDP socket, s for secure socket)
* @retval void
*/
void Modem_Queue_Client_Open_Event(uint8_t * hostname, uint32_t port_number,uint8_t * protocol,uint8_t scktID) //,uint8_t ssl_enable)
{
    Modem_Wait_For_Sock_Read_To_Complete();
    Modem_Counter_Variables.curr_hostname = hostname;
    Modem_Counter_Variables.curr_port_number = port_number;
    Modem_Counter_Variables.curr_protocol = protocol;
    Modem_Counter_Variables.curr_sockID = scktID;
    
    modem_instances.modem_event.event = MODEM_CLIENT_SOCKET_OPEN_EVENT;
    
    __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();

    modem_reset_event(&modem_instances.modem_event);
}


/**
* @brief
*        Queue a Client Open Event
* @param hostname hostname
* @param port_number port number to connect to
* @param protocol protocol required to connect to server (t for TCP socket, u for UDP socket, s for secure socket)
* @retval void
*/
void Modem_Queue_Mqtt_Client_Open_Event(uint8_t * serverIP, uint32_t port_number)
{
    Modem_Wait_For_Sock_Read_To_Complete();
    Modem_Counter_Variables.curr_hostname = serverIP;
    Modem_Counter_Variables.curr_port_number = port_number;
    //Modem_Counter_Variables.curr_protocol = protocol;
    //Modem_Counter_Variables.curr_sockID = scktID;
    
    modem_instances.modem_event.event = MODEM_MQTT_CLIENT_SOCKET_OPEN_EVENT;
    
    __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();

    modem_reset_event(&modem_instances.modem_event);
}



void Modem_Queue_Mqtt_Publish_Event(char *topic, char *mes)
{
    Modem_Wait_For_Sock_Read_To_Complete();
    //Modem_Counter_Variables.curr_hostname = serverIP;
    //Modem_Counter_Variables.curr_port_number = port_number;
    //Modem_Counter_Variables.curr_protocol = protocol;
    //Modem_Counter_Variables.curr_sockID = scktID;
    Modem_Counter_Variables.topic = topic;
    Modem_Counter_Variables.topic_message = mes;
    modem_instances.modem_event.event = MODEM_MQTT_CLIENT_PUBLISH_MESSAGE;
    
    __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();

    modem_reset_event(&modem_instances.modem_event);
}


void Modem_Queue_Mqtt_Subscribe_Event(void) //char *topic) 
{
    Modem_Wait_For_Sock_Read_To_Complete();
    //Modem_Counter_Variables.curr_hostname = serverIP;
    //Modem_Counter_Variables.curr_port_number = port_number;
    //Modem_Counter_Variables.curr_protocol = protocol;
    //Modem_Counter_Variables.curr_sockID = scktID;
    //Modem_Counter_Variables.topic = topic;
    //Modem_Counter_Variables.topic_message = mes;
    modem_instances.modem_event.event = MODEM_MQTT_CLIENT_SUBSCRIBE_MESSAGE;
    
    __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();

    modem_reset_event(&modem_instances.modem_event);
}

/**
* @brief Modem_Client_Check_Socket_Status
*        Checking if a socket ID is already opened
* @param socket Id
* @param ST87 context
 * @retval void
*/
void Modem_Client_Check_Socket_Status (uint8_t scktID)
{
    Modem_Wait_For_Sock_Read_To_Complete();
    Modem_Counter_Variables.curr_sockID = scktID;
    
    modem_instances.modem_event.event = MODEM_CLIENT_SOCKET_STATUS_EVENT;
    
    __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();

    modem_reset_event(&modem_instances.modem_event);
}

void Modem_get_server_ip(char *server_name)
{
    Modem_Wait_For_Sock_Read_To_Complete();
    Modem_Counter_Variables.curr_hostname = server_name;
    
    modem_instances.modem_event.event = MODEM_DNS_GET_IP;
    
    __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();

    modem_reset_event(&modem_instances.modem_event);
} 

uint8_t Modem_Get_Sckt_Closed_Status (void)
{
  return(IO_modem_status_flag.SCKT_CLOSED_Received);
}

void Modem_Clear_Sckt_Closed_Status (void)
{
  IO_modem_status_flag.SCKT_CLOSED_Received = MODEM_FALSE;
}

/**
* @brief Queue_Client_Close_Event
*        Queue a Client Close Event
* @param sock_id socket ID to close
* @retval void
*/
void Modem_Queue_Client_Close_Event(uint8_t sock_id) //,uint8_t ssl_enable)
{
    Modem_Wait_For_Sock_Read_To_Complete();
    
    modem_instances.modem_event.event = MODEM_CLIENT_SOCKET_CLOSE_EVENT;
    modem_instances.modem_event.socket_sid = sock_id;
     __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();

    modem_reset_event(&modem_instances.modem_event);
}

/**
* @brief Modem_Wait_For_Sock_Read_To_Complete
*        Wait till sock read is over and the OK of read arrives
* @param None
* @retval None
*/
void Modem_Wait_For_Sock_Read_To_Complete(void)
{
  //wait if read is ongoing or read OK is yet to arrive
  while(IO_modem_status_flag.sock_read_ongoing == MODEM_TRUE || 
       (IO_modem_status_flag.prevent_push_OK_event == MODEM_TRUE && IO_modem_status_flag.client_socket_close_ongoing != MODEM_TRUE)) // to make sure the prevent_push_OK_event is of socket read and not of socket close.
    {
        __NOP(); //nothing to do
    }
}

/**
* @brief  Reset_AT_CMD_Buffer
*         Clear USART2 Rx buffer and Wi-Fi AT cmd buffer
* @param  None
* @retval None
*/
void Modem_Reset_AT_CMD_Buffer()
{
  memset(Modem_AT_Cmd_Buff, 0x00, sizeof Modem_AT_Cmd_Buff); 
}

/**
 * @brief      reset buffer that receive network time
 */
void Modem_Reset_Network_Time_Buffer()
{
  __disable_irq();
  memset(modem_network_time_buff, 0x00, sizeof modem_network_time_buff);
  __enable_irq();
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
    {
      __NOP(); //nothing to do
    }
}

#endif

//////
void Modem_Tcp_Connection_Event(uint8_t * hostname, uint32_t port_number,uint8_t * protocol,uint8_t scktID) //,uint8_t ssl_enable)
{
    Modem_Wait_For_Sock_Read_To_Complete();
    Modem_Counter_Variables.curr_hostname = hostname;
    Modem_Counter_Variables.curr_port_number = port_number;
    //Modem_Counter_Variables.curr_protocol = protocol;
    Modem_Counter_Variables.curr_sockID = scktID;
    
    modem_instances.modem_event.event = MODEM_TCP_CONNECTION_EVENT;
    
    __disable_irq();
    modem_push_eventbuffer_queue(&modem_instances.event_buff, modem_instances.modem_event);
    __enable_irq();

    modem_reset_event(&modem_instances.modem_event);
}



//////
/**
* @brief  Read_Modem_Mode
*         Read Wi-Fi mode 0: idle,1 =STA,2 =IBSS,3 =MiniAP
* @param  string : return modem mode type
* @retval return status of AT cmd request
*/
Modem_Status_t Read_Modem_Mode(char *string)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//  char *mode = "modem_mode";
//  char *pStr;
//  
//    /* AT+S.GCFG=modem_mode */
//  Reset_AT_CMD_Buffer();
//  
//  /* AT : send AT command */  
//  sprintf((char*)Modem_AT_Cmd_Buff,AT_GET_CONFIGURATION_VALUE,mode);  
//  
//  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  if(status == MODEM_MODULE_SUCCESS)
//    {
//      status = USART_Receive_AT_Resp( );
//    }
//  
//  pStr = (char *) strstr((const char *)&Modem_Counter_Variables.get_cfg_value,"modem_mode = ");
//  if(pStr != NULL)
//    {
//      string[0] = *(pStr + 12) ;
//    }
//
  return status ;
}

///**
//* @brief  Write_Modem_SSID
//*         Store SSID in flash memory of Modem module
//* @param  string : pointer of SSID
//* @retval return status of AT cmd request
//*/
//Modem_Status_t Write_Modem_SSID(char *string)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;  
////  Reset_AT_CMD_Buffer(); 
////  
////  /* AT+S.SSIDTXT=abcd <ExampleSSID> //set SSID */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_SET_SSID,string);  
////  
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
////
////  /* AT&W :Save the settings on the flash memory */
////  Reset_AT_CMD_Buffer();
////  Save_Current_Setting();
////
//  return status;
//}
//
///**
//* @brief  Write_Modem_SecKey
//*         Store security key in flash memory of Modem module
//* @param  string : pointer of security key
//* @retval return status of AT cmd request
//*/
//Modem_Status_t Write_Modem_SecKey(char *string)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;  
////  Reset_AT_CMD_Buffer(); 
////  
////  /* AT+S.SCFG=modem_wpa_psk_text,helloworld : set password */
////  sprintf((char*)Modem_AT_Cmd_Buff,"AT+S.SCFG=modem_wpa_psk_text,%s\r",string);
////  
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
////  
////  /* AT&W :Save the settings on the flash memory */
////  Reset_AT_CMD_Buffer();
////  Save_Current_Setting();
////  
//  return status;     
//}
//
///**
//* @brief  PrintErrorMsg
//*         Print error message on UART terminal
//* @param  None
//* @retval None
//*/
//void PrintErrorMsg (void)
//{
//  Print_Msg("error in AT cmd",sizeof("error in AT cmd"));
//}
//
///**
//  * @brief  Print_Msg
//  *         Print messages on UART terminal
//  * @param  msgBuff : Contains data that need to be print
//  * @param  length  : leghth of the data
//  * @retval None
//  */
//void Print_Msg(char * msgBuff,uint8_t length)
//{
//
//}

///**
//* @brief  Error_Handler
//*         This function is executed in case of error occurrence.
//* @param  None
//* @retval None
//*/
//void Error_Handler(void)
//{
//  //The following while(1) is commented as it prevents standby functionality
//  while(1)
//  { 
//    HAL_Delay(1000); 
//  }
//}

/**
* @brief  Modem_Transmit_AT_Cmd
*         send AT cmd on UART port of modem module.
* @param  size size of the AT command
* @retval Modem_Status_t : status of AT cmd
*/
Modem_Status_t Modem_Transmit_AT_Cmd(uint16_t size)
{
  Modem_Status_t output = MODEM_MODULE_SUCCESS;
  //Check for Hardware Started
  if(IO_modem_status_flag.Modem_Enabled == MODEM_FALSE)
  {
      output = MODEM_NOT_READY;
      goto EXIT_AT_TRANSMIT_LB;
  }
  
  Modem_Control_Variables.AT_Cmd_Processing = MODEM_TRUE;//Stop Any Rx between the TX call
  
  if (size == 0)
    {
        #if MODEM_PRINT_DEBUG
            printf("ERROR in Modem_Transmit_AT_Cmd!");
        #endif
        output =  MODEM_UNHANDLED_IND_ERROR;
        goto EXIT_AT_TRANSMIT_LB;
    }
  
  #if MODEM_PRINT_DEBUG  
    printf ("\r\n%s",Modem_AT_Cmd_Buff);
  #endif

  /* Start DMA transmission data through "Modem_AT_Cmd_Buff" buffer */
  if ((HAL_UART_Transmit_DMA(&UartModemHandle, (uint8_t *)Modem_AT_Cmd_Buff, size)) != HAL_OK)
  {
    /* Transfer error in transmission process */
    //Error_Handler();
    #if MODEM_PRINT_DEBUG
      printf("HAL_UART_Transmit Error");
    #endif
    output = MODEM_HAL_UART_ERROR;
    goto EXIT_AT_TRANSMIT_LB;
  }
  
#ifdef COMM_DEBUG
  spy_buff_size = size;
  //memset((uint8_t *)SPY_AT_Buff,0x00,MODEM_AT_COMMAND_BUFF_SIZE);
  memcpy((uint8_t *)SPY_AT_Buff,(uint8_t *)Modem_AT_Cmd_Buff,spy_buff_size);
  SPY_AT_Buff[spy_buff_size]=0x00;
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)SPY_AT_Buff, spy_buff_size);
#endif  


EXIT_AT_TRANSMIT_LB:  
  Modem_Control_Variables.AT_Cmd_Processing = MODEM_FALSE;//Re-enable Rx for UART
  
  if(Modem_Control_Variables.Uartx_Rx_Processing == MODEM_FALSE)
    Modem_Receive_Data();//Start receiving Rx from the UART again, if and only if it was stopped in the previous Uartx_Rx_Handler

  return output;
}

/**
* @brief  Start_DeepSleep_Timer
*         start the deep sleep timer.
* @param  None
* @retval void
*/
//void Start_DeepSleep_Timer(void)
//{
////  Modem_Control_Variables.Deep_Sleep_Timer = MODEM_TRUE;
////  Modem_Counter_Variables.sleep_count = 0;
//}
//
///**
//* @brief  Stop_DeepSleep_Timer
//*         stop the deep sleep timer.
//* @param  None
//* @retval void
//*/
//void Stop_DeepSleep_Timer()
//{
////  Modem_Control_Variables.Deep_Sleep_Timer = MODEM_FALSE;
////  Modem_Counter_Variables.sleep_count = 0;
//}

/**
* @brief  Modem_switch_to_command_mode
*         switch to command mode from data mode
* @param  None
* @retval None
*/
void Modem_switch_to_command_mode(void)
{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//
//  /* AT+S.*/  
//  Reset_AT_CMD_Buffer();  
//
//  sprintf((char*)Modem_AT_Cmd_Buff,AT_DATA_TO_CMD_MODE);   //Notice the lower case
//
//  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  if(status == MODEM_MODULE_SUCCESS)
//  {
//    //nothing to do
//  }
}

/**
* @brief  Modem_switch_to_data_mode
*         switch to data mode from command mode
* @param  None
* @retval None
*/
void Modem_switch_to_data_mode(void)
{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//
//  /* AT+S.*/  
//  Reset_AT_CMD_Buffer();    
//
//  sprintf((char*)Modem_AT_Cmd_Buff,AT_CMD_TO_DATA_MODE);   //Notice the upper case
//
//  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//
//  if(status == MODEM_MODULE_SUCCESS)
//    {
//      //nothing to do
//    }
}

/**
* @brief  Attention_Cmd
*         Attention command
* @param  None
* @retval Modem_Status_t : status of AT cmd Request
*/
//Modem_Status_t Attention_Cmd()
//{
////  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////
////  Reset_AT_CMD_Buffer(); 
////
////  /* AT : send AT command */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_ATTENTION);  
////
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
////return status; 
//}

///**
//* @brief  SET_Power_State
//*         SET power mode of modem module
//* @param  state : power mode of wi-fi module i.e active,sleep,standby,powersave
//* @retval Modem_Status_t : status of AT cmd Request
//*/
//Modem_Status_t SET_Power_State(Modem_Power_State_t state)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////
////#if DEBUG_PRINT  
////  printf("\r\n >>Soft Reset Wi-Fi module\r\n");
////#endif
////
////  Reset_AT_CMD_Buffer();
////
////  /* AT : send AT command */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_SET_POWER_STATE,state);  
//////  Modem_WIND_State.ModemReset = MODEM_FALSE;
////  IO_modem_status_flag.Modem_WIND_State.ModemHWStarted = MODEM_FALSE;
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status != MODEM_MODULE_SUCCESS) 
////    return status;
////  memset((void*)&IO_modem_status_flag.Modem_WIND_State,0x00,sizeof(IO_modem_status_flag.Modem_WIND_State)); /*reset the WIND State?*/
////  /* AT+CFUN=1 //Soft reset */
////  while(IO_modem_status_flag.Modem_WIND_State.ModemHWStarted != MODEM_TRUE)
////  {
////    __NOP(); //nothing to do
////  }
//  return status;
//}
//
///**
//* @brief  Display_Help_Text
//*         this function will print a list of all commands supported with a brief help text for each cmd
//* @param  None
//* @retval Modem_Status_t : status of AT cmd Request
//*/
//Modem_Status_t Display_Help_Text()
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////  Reset_AT_CMD_Buffer();
////
////  /* AT : send AT command */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_HELP_TEXT);  
////
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
//  return status; 
//}
//
///**
//* @brief  GET_Configuration_Value
//*         Get a modem configuration value from the module
//* @param  sVar_name : Name of the config variable
//*         aValue    : value of config variable to be returned to user
//* @retval Modem_Status_t : status of AT cmd Request
//*/
//Modem_Status_t GET_Configuration_Value(char* sVar_name,uint32_t *aValue)
//{
////  int cfg_value_length;
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////
////  Reset_AT_CMD_Buffer(); 
////
////  /* AT : send AT command */  
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_GET_CONFIGURATION_VALUE,sVar_name);   
////
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////      cfg_value_length = strlen((const char*)Modem_Counter_Variables.get_cfg_value);
////      memcpy(aValue,Modem_Counter_Variables.get_cfg_value,cfg_value_length);   //copy user pointer to get_cfg_value
////      memset(Modem_Counter_Variables.get_cfg_value, 0x00,cfg_value_length);
////    }
//  return status; 
//}
//
///**
//* @brief  SET_Configuration_Addr
//*         Get a modem configuration address from the module
//* @param  sVar_name : Name of the config variable
//*         addr    : value of config address to be returned to user
//* @retval Modem_Status_t : status of AT cmd Request
//*/
//Modem_Status_t SET_Configuration_Addr(char* sVar_name,char* addr)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////
////  Reset_AT_CMD_Buffer(); 
////
////  /* AT : send AT command */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_SET_CONFIGURATION_ADDRESS,sVar_name,addr);  
////
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
//  return status;
//}
//
///**
//* @brief  SET_Configuration_Value
//*         SET the value of configuration variable
//* @param  sVar_name : Name of the config variable
//*         aValue    : value of config variable
//* @retval Modem_Status_t : status of AT cmd Request
//*/
//Modem_Status_t SET_Configuration_Value(char* sVar_name,uint32_t aValue)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////
////  Reset_AT_CMD_Buffer(); 
////
////  /* AT : send AT command */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_SET_CONFIGURATION_VALUE,sVar_name,(int)aValue);  
////
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
//  return status; 
//}
//
///**
//* @brief  SET_SSID
//*         SET SSID in flash memory of Wi-Fi module
//* @param  ssid : pointer of SSID
//* @retval Modem_Status_t : status of AT cmd Request
//*/
//Modem_Status_t SET_SSID(char* ssid)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////  
////  Reset_AT_CMD_Buffer(); 
////  
////  /* AT+S.SSIDTXT=abcd <ExampleSSID>  */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_SET_SSID,ssid);  
////
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
//  return status; 
//}
//
//
///**
//* @brief  SET_Modem_SecKey
//*         SET modem security key
//* @param  seckey : pointer of security key
//* @retval Modem_Status_t : status of AT cmd Request
//*/
//Modem_Status_t SET_Modem_SecKey(char* seckey)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////  
////  Reset_AT_CMD_Buffer(); 
////  
////  /* AT+S.SCFG=modem_wpa_psk_text,helloworld : set password */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_SET_SEC_KEY,seckey);  
////
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
//  return status;    
//}
//
//
///**
//* @brief  Restore_Default_Setting
//*         Restore the factory default values of the configuration variables 
//*         and writes them to non volatile storage
//* @param  None
//* @retval Modem_Status_t : status of AT cmd Request
//*/
//Modem_Status_t Restore_Default_Setting()
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////
////  //Reset_AT_CMD_Buffer(); 
////
////  /* AT&F: restore default setting */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_RESTORE_DEFAULT_SETTING);  
////
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
//  return status;
//}
//
///**
//* @brief  Save_Current_Setting
//*         Store the current RAM-based setting to non-volatile storage
//* @param  None
//* @retval Modem_Status_t : status of AT cmd Request
//*/
//Modem_Status_t Save_Current_Setting()
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////  
////  Reset_AT_CMD_Buffer(); 
////  
////  /* AT&W :Save the settings on the flash memory */
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_SAVE_CURRENT_SETTING);  
////
////  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
////  if(status == MODEM_MODULE_SUCCESS)
////    {
////      status = USART_Receive_AT_Resp( );
////    }
//  return status; 
//}
//
///**
//* @brief  ResetBuffer
//*         Reset receive data/indication msg buffer
//* @param  None
//* @retval None
//*/
//void ResetBuffer()
//{  
//  
//}
//
///**
//* @brief  config_init_value
//*         initalize config values before reset
//* @param  sVar_name : Name of the config variable
//*         aValue    : value of config variable
//* @retval None
//*/
//Modem_Status_t config_init_value(char* sVar_name,uint8_t aValue)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////  Reset_AT_CMD_Buffer();   
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_SET_CONFIGURATION_VALUE,sVar_name,aValue);
////  if(HAL_UART_Transmit(&UartModemHandle, (uint8_t *)Modem_AT_Cmd_Buff, strlen((char*)Modem_AT_Cmd_Buff),1000)!= HAL_OK)
////    {
////      Error_Handler();    
////      return Modem_HAL_UART_ERROR;
////    }
////  
////  status = WaitForResponse(AT_RESP_LEN_OK);
//  return status;
//}
//
///**
//* @brief  config_init_addr
//*         initalize config strings/addresses before reset
//* @param  sVar_name : Name of the config variable
//*         addr    : value of config address to be returned to user
//* @retval None
//*/
//Modem_Status_t config_init_addr(char* sVar_name,char* addr)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////  Reset_AT_CMD_Buffer();   
////  sprintf((char*)Modem_AT_Cmd_Buff,AT_SET_CONFIGURATION_ADDRESS,sVar_name,addr);
////  if(HAL_UART_Transmit(&UartModemHandle, (uint8_t *)Modem_AT_Cmd_Buff, strlen((char*)Modem_AT_Cmd_Buff),1000)!= HAL_OK)
////    {
////      Error_Handler();    
////      return Modem_HAL_UART_ERROR;
////    }
////  
////  status = WaitForResponse(AT_RESP_LEN_OK);
//  return status;
//
//}
//
//
///**
//* @brief  WaitForResponse
//*         Wait for OK response
//* @param  alength length of the data to be received
//* @retval None
//*/
//Modem_Status_t WaitForResponse(uint16_t alength)
//{
////  uint8_t USART_RxBuffer[64];    //This buffer is only used in the Init phase (to receive "\r\nOK\r\n")
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
////  
////  if(alength <= RxBufferSize)
////  {
////    if(HAL_UART_Receive(&UartModemHandle, (uint8_t *)USART_RxBuffer, alength,5000)!= HAL_OK)
////      {
////        Error_Handler();
////        return Modem_HAL_UART_ERROR;
////      }
////    if(((strstr((const char *)&USART_RxBuffer,"OK"))) == NULL)
////      {
////        return Modem_AT_CMD_RESP_ERROR;
////      }
////  }
//  return status;  
//}
/**** Wi-Fi indication call back *************/
__weak void ind_modem_warning(Modem_Status_t warning_code)
{
}
	
__weak void ind_modem_error(Modem_Status_t error_code)
{
}

__weak void ind_modem_connection_error(Modem_Status_t status_code)
{
}

__weak void ind_modem_connected(void)
{
}
	
__weak void ind_modem_ap_ready(void)
{
}

__weak void ind_modem_ap_client_joined(uint8_t * client_mac_address)
{
}

__weak void ind_modem_ap_client_left(uint8_t * client_mac_address)
{
}

__weak void ind_modem_on(void)
{
}

__weak void ind_modem_packet_lost(Modem_Status_t status_code)
{
}

__weak void ind_modem_gpio_changed(void)
{
}

__weak void ind_modem_socket_data_received(uint8_t socket_id, uint8_t * data_ptr, uint32_t message_size, uint32_t chunk_size)
{
}

__weak void ind_modem_socket_client_remote_server_closed(uint8_t * socketID)
{
}

__weak void ind_modem_socket_server_data_lost(void)
{
}

__weak void ind_socket_server_client_joined(void)
{
}

__weak void ind_socket_server_client_left(void)
{
}

__weak void ind_modem_http_data_available(uint8_t * data_ptr,uint32_t message_size)
{
}

__weak void ind_modem_file_data_available(uint8_t * data_ptr)
{
}
__weak void ind_modem_resuming(void)
{
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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/

