  /**
  ******************************************************************************
  * @file    modem_module.h
  * @author  IoT AME 
  * @version V2.1.0
  * @date    29-Abril-2017
  * @brief   Header file for Modem-Fi module
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
#ifndef __MODEM_MODULE_H
#define __MODEM_MODULE_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
//#include "modem_ring_buffer.h"
//#include "modem_event_buffer.h"
#include "stm32_st87m01.h"  
#include "modem_const.h"   
#include "modem_interface.h"


/** @addtogroup MIDDLEWARES
* @{
*/ 


/** @addtogroup  NUCLEO_WIFI_MODULE 
  * @brief Wi-Fi_driver modules
  * @{
  */ 


/** @addtogroup NUCLEO_WIFI_MODULE_Private_Macros
  * @{
  */
   
//#define USART3_INT_MODE
#define USART3_POLLING_MODE //JRF TBC
 /**
  * @}
  */


/** @addtogroup NUCLEO_WIFI_MODULE_Private_Variables
  * @{
  */
/* Private variables ---------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/******* Wi-Fi Configuration Setting Parameters *****************/
    
/**
  * @brief  Modem config values structure definition
  */    
typedef struct
{
  uint8_t blink_led;
  char* console1_speed;
  char* modem_ssid;
  char* Modem_Sec_key;
  uint8_t modem_mode;
  uint8_t sleep_enabled;
  uint8_t standby_enabled;
  uint8_t standby_time;
  uint8_t modem_powersave;
  uint8_t modem_operational_mode;
  uint8_t modem_listen_interval;
  uint8_t modem_beacon_wakeup;
  uint8_t modem_priv_mode;
  uint8_t ip_use_dhcp;
  uint8_t ip_use_httpd;
  char* ip_hostname;
  uint8_t* IBSS_IP_Addr;
  uint8_t* IBSS_DefaultGateway;
  uint8_t* IBSS_IP_DNS_Addr;
  uint8_t* IBSS_IP_Mask;  
  char* ip_ipaddr;
}Modem_Config_HandleTypeDef;


/**
  * @brief  Modem control variables structure definition
  */ 
#pragma pack(1)
typedef struct
{
  modem_bool Modem_Configuration_Done;
  modem_bool stop_event_dequeue;
  modem_bool AT_Cmd_Ongoing;
  modem_bool AT_Cmd_Processing;
  modem_bool Uartx_Rx_Processing;
  modem_bool Client_Connected;
  modem_bool Client_Disconnected;
  modem_bool start_sock_read;
  modem_bool enable_receive_data_chunk;
  modem_bool data_pending_sockD;
  modem_bool enable_sock_read;
  modem_bool enable_query;
  modem_bool prevent_push_MODEM_event;
  modem_bool message_pending;
  modem_bool Pending_SockON_Callback;
  modem_bool Pending_SockD_Callback;
  modem_bool SockON_Server_Closed_Callback;
  modem_bool Client_Socket_Close_Cmd;
  modem_bool resume_receive_data;
  modem_bool enable_timeout_timer;
  modem_bool enable_sock_data;
  modem_bool enable_SockON_Server_Closed_Callback; //enable SockON_Server_Closed_Callback in case of WIND:58, not when user requests for socket close.
  Modem_Status_t  AT_RESPONSE;

}Modem_Control_Variables_t;
#pragma pack()

/**
  * @brief  Modem mode enumeration definition
  */ 
typedef enum 
{
  Modem_IDLE_MODE =0,
  Modem_ON_MODE,
  Modem_STANDBY_MODE,
} Modem_Mode_TypeDef;

/********** Wi-Fi Indications*************/


/**
  * @brief  Modem wind state structure definition
  */ 
#pragma pack(1) 
typedef struct 
{
  modem_bool ModemHWFailure;
  modem_bool HardFault;
  modem_bool StackOverflow;
  modem_bool MallocFailed;
  modem_bool InitFailure;
  modem_bool StartFailed;
  modem_bool PS_Mode_Failure;
  modem_bool HeapTooSmall;
  modem_bool ModemSignalLOW;
  modem_bool ModemUp;
  modem_bool ModemStarted_MiniAPMode;
  modem_bool ModemAPClientJoined;
  modem_bool ModemAPClientLeft;
  modem_bool ModemException;  
  modem_bool ModemHWStarted;
  modem_bool ModemPowerDown;
  modem_bool ModemDeauthentication;
  
  /*Modem Connection Errors*/
  modem_bool ModemJoinFailed;
  modem_bool ModemScanBlewUp;
  modem_bool ModemScanFailed;
  modem_bool ModemDeAuth;
  modem_bool ModemDisAssociation;
  
  /*Modem packet lost INDs*/
  modem_bool ModemUnHandledInd;
  modem_bool ModemRXMgmt;
  modem_bool ModemRXData;
  modem_bool ModemRxUnk;   
  modem_bool ModemSockdDataLost;
  
} Modem_WIND_State_TypeDef;
#pragma pack()


/**
  * @brief  Modem WIND indication enumeration definition
  */ 
typedef enum {
  Modem_Console_Active          = 0,
  Modem_Poweron                 = 1,
  Modem_Reset,
  Modem_Watchdog_Running,
  Modem_Heap_Too_Small,
  Modem_Hardware_Dead      = 5,
  Modem_Watchdog_Terminating,
  Modem_SysTickConfigure,
  Modem_Hard_Fault              = 8,   
  Modem_StackOverflow,
  Modem_MallocFailed,
  Modem_Error,
  Modem_PS_Mode_Failure    = 12,
  Modem_CopyrightInfo,
  Modem_BSS_Regained       = 14,
  Modem_Signal_LOW         = 15,
  Modem_Signal_OK          = 16,
  Modem_FW_update               = 17,
  Modem_Encryption_key_Not_Recognized,
  Modem_Join               = 19,
  Modem_JOINFAILED              = 20,
  Modem_Scanning           = 21,
  Modem_SCANBLEWUP,
  Modem_SCANFAILED,
  Modem_Up                 = 24,
  Modem_Association_Successful   = 25,
  Modem_Started_MiniAP_Mode      = 26,
  Modem_Start_Failed                  = 27,
  Modem__MiniAP_Associated       = 28,
  Modem_BSS_LOST                 = 30,
  Modem_EXCEPTION          = 31,    
  Modem_Hardware_Started   = 32,
  Modem_NETWORK_LOST,
  Modem_Unhandled_Event,
  Modem_Scan_Complete           = 35,
  Modem_UNHANDLED_IND,
  Modem_UNHANDLED,
  Modem_Powered_Down,
  Modem_MiniAP_Mode        = 39,
  Modem_Deauthentication   = 40,     
  Modem_Disassociation,
  //Modem_RX_MGMT,
  //Modem_RX_DATA,
  //Modem_RX_UNK,
  Modem_DOT11_AUTHILLEGAL,
  Modem_Creating_PSK            = 46,  
  Modem_WPA_Terminated          = 49,  
  Modem_WPA_Supplicant_Failed,
  Modem_WPA_Handshake_Complete  = 51,
  Modem_GPIO_line,
  Modem_Wakeup,
  Modem_Factory_debug,
  QIRDI_DATA_PENDING           = 55,
  Modem_Remote_Configuration          = 57,
  Modem_SockON_Server_Socket_Closed   = 58,
  Modem_In_Command_Mode         = 59,
  Modem_In_Data_Mode            = 60,
  Modem_Incoming_socket_client  = 61,
  Modem_Outgoing_socket_client  = 62,
  Modem_SockD_Dropping_Data     = 63,
  Modem_SockD_Pending_Data      = 64,
  Modem_Low_Power_Mode_Enabled  = 66,
  Modem_Going_Into_Standby      = 67,
  Modem_Resuming_From_Standby   = 68,
  Modem_Going_Into_DeepSleep    = 69,
  Modem_Resuming_From_DeepSleep = 70,
  Modem_MiniAP_Disassociated     = 72,
  Modem_Rejected_Found_Network  = 74,
  Modem_Undefine_Indication     = 0xFF
} Modem_Indication_t;


/**
  * @brief  Modem power state enumeration definition
  */ 
typedef enum
{ 
  Modem_Undef_State = 0,
  Modem_On_State,
  Modem_Ready_State,
  Modem_PowerSave_State,    
  Modem_Sleep_State=3,
  Modem_StandBy_State=4
} Modem_Power_State_t;




/**
  * @brief  Modem control variables structure definition
  */ 
typedef struct
{
  uint8_t  Socket_Open_ID;
  uint8_t  sockon_query_id;
  uint8_t  sockon_id_user;
  uint8_t  curr_sockID;
  uint8_t  closed_socket_id;
  uint8_t  remote_socket_closed_id;
  uint8_t  client_socket_close_id;
  uint8_t  no_of_open_client_sockets;
  uint8_t * curr_hostname;
  uint8_t * curr_path;
  uint8_t * curr_pURL;
  uint8_t * curr_protocol;
  uint8_t * curr_filename;
  uint8_t * temp;       //pop buffer temporary
  char *topic;
  char *topic_message;
  
  char * curr_data;
  
  uint16_t curr_DataLength;
  

  uint32_t UserDataBuff_index;
  uint32_t Socket_Data_Length;
  uint32_t SockON_Data_Length;
  uint32_t number_of_bytes;
  uint32_t interim_number_of_bytes;
  uint32_t sock_total_count;
  uint32_t chunk_size;
  uint32_t message_size;
  uint32_t pop_queue_length;
  uint32_t pop_buffer_size;
  uint32_t last_process_buffer_index;
  uint32_t epoch_time;
  uint32_t curr_port_number;
  uint32_t sleep_count;
  uint32_t standby_time;
  uint32_t timeout_tick;

  uint8_t uart_byte[1];              /* Used to store one byte that is popped from the ring buffer */

  //Modem_Status_t AT_RESPONSE;
  //Modem_Status_t CONNECT_RESPONSE;
  

  /* Keeps a track of the socket ID that needs to be closed.
     Each array element depicts one socket(true = socket needs to be closed) */
  modem_bool socket_close_pending[5];
}Modem_Counter_Variables_t;

/**
  * @brief  All __IO type variable structure definition
  */ 
typedef struct
{
  __IO modem_bool sock_read_ongoing;
  __IO modem_bool prevent_push_OK_event;
  __IO modem_bool Timer_Running;
  __IO modem_bool enable_dequeue;
  __IO modem_bool Modem_Enabled;
  __IO modem_bool data_mode;
  __IO modem_bool simcard;
  __IO modem_bool nb_sent;

  __IO uint32_t tickcount;
  //__IO uint8_t modem_ready;   
  //__IO uint8_t modem_standby; 
  
  __IO Modem_Power_State_t Modem_State;
  __IO modem_bool Modem_Sync_State;
  
  __IO modem_bool client_socket_close_ongoing;

  __IO modem_bool AT_Response_Received;
  __IO modem_bool CONNECT_Response_Received;
  __IO modem_bool QSSLOPEN_Response_Received;
  __IO modem_bool QSSLSEND_Response_Received;
  
  __IO modem_bool QISEND_Response_Received;
  
  __IO modem_bool QIOPEN_Response_Received;
  
  __IO modem_bool IPSEND_Response_Received;
  
  __IO modem_bool IPSEND_DONE_Response_Received;
  
  __IO modem_bool DNS_Response_Received;
  
  __IO modem_bool MQTTOPEN_Send;
  
  __IO modem_bool MQTTOPEN_Response_Received;
  
  __IO modem_bool MQTTPUB_Send;
  
  __IO modem_bool MQTTPUB_Response_Received;  
  
  __IO modem_bool MQTTSUB_Send;
  
  __IO modem_bool MQTTSUB_Response_Received;
  
  __IO modem_bool TCP_CONNECT_Send;  
  __IO modem_bool TCP_CONNECT_Response_Received;
  
  __IO modem_bool SCKT_CLOSED_Received;

  __IO ITStatus UartReady;

  __IO ITStatus TxUartReady;

} IO_modem_status_flag_typedef;



void PowerUp_Modem_Module(void);
void Modem_Module_and_Timer_Init(void);
void Modem_Module_Init(void);
void Modem_Application(void);

//Modem_Status_t Check_Modem_Ready (void);

/******* Wi-Fi AT CMD SET ****************/
//Modem_Status_t Attention_Cmd(void);
Modem_Status_t Modem_Transmit_AT_Cmd(uint16_t size);
Modem_Status_t Modem_Receive_AT_Resp(uint32_t wait_ms);
//Modem_Status_t Modem_Receive_CONNECT_Resp(uint32_t wait_ms );

Modem_Status_t Modem_WaitForResponse(uint16_t alength);
Modem_Status_t Modem_config_init_value(char* sVar_name,uint8_t aValue);
Modem_Status_t Modem_config_init_addr(char* sVar_name,char* addr);
//void Process_Modem_Qirdi_Indication(uint8_t *process_buff_ptr);

//void Process_Modem_Qird_Indication(uint8_t *process_buff_ptr);


void Modem_Reset_AT_CMD_Buffer(void);
void USART2_SendBuffer(USART_TypeDef* USARTx, uint8_t *pData, uint8_t length);
void Modem_Reset_Network_Time_Buffer();

char* Delete_Colon(char* );
Modem_Status_t Read_Modem_Mode(char *string);
Modem_Status_t Read_Modem_SecKey(char *string);

Modem_Status_t Write_Modem_SSID(char *string);
Modem_Status_t Write_Modem_SecKey(char *string);
Modem_Status_t SET_Modem_SecKey(char* seckey);
void PrintErrorMsg (void);
void Print_Msg(char * msgBuff,uint8_t length);
void Error_Handler(void);
void Modem_Receive_Indication_Msg(void);
char *search_buffer(char *pSourceBuff, uint16_t sourceBuffLen, char *pSearchStringBuff, uint16_t seartchStringLen);

void Modem_ResetBuffer(void);
void Modem_Start_Timer(void);
void Modem_Stop_Timer(void);
void Modem_Request_Time_Out(void);
void Modem_Start_DeepSleep_Timer(void);
void Modem_Stop_DeepSleep_Timer(void);

void Modem_HTTP_Read_Data(void);
//Modem_Status_t Modem_Socket_Read(uint16_t DataLength);
Modem_Status_t Modem_Socket_Read(uint8_t sckID);
void Modem_Read_Socket_Data(void);
void Modem_Socket_Pending_Data(void);
void Modem_switch_to_command_mode(void);
void Modem_switch_to_data_mode(void);
void Modem_Configuration(void);
void Set_Modem_Counter_Variables(void);
void Set_Modem_Control_Variables(void);
void Modem_Variables_Init(void);

void Modem_Receive_Data(void);
void Process_Modem_Buffer(uint8_t * ptr);
void Process_Modem(void);
void Modem_Stop_Dequeue(void);
void Modem_Resume_Dequeue(void);
void Modem_wait_for_command_mode(void);
void Modem_SysTick_Isr(void);
void Modem_RX_EXTI_Isr(uint16_t GPIO_Pin);
void Modem_TIM_Handler(TIM_HandleTypeDef *htim);
void Modem_Queue_Http_Event(uint8_t * hostname, uint8_t * path, uint32_t port_number,uint8_t * pURL_path);
void Modem_Queue_Client_Write_Event(uint8_t sock_id, uint16_t DataLength, char * pData);
void Modem_Queue_Modem_FW_Update_Event(uint8_t * hostname, uint8_t * filename_path, uint32_t port_number);
void Modem_Queue_Modem_File_Event(uint8_t * pHostName, uint8_t * pFileName, uint32_t port_number);
//void Modem_Queue_Client_Open_Event(uint8_t * hostname, uint32_t port_number, uint8_t * protocol); // ,uint8_t ssl_enable);
void Modem_Queue_Client_Open_Event(uint8_t * hostname, uint32_t port_number,uint8_t * protocol,uint8_t scktID); //,uint8_t ssl_enable)
void Modem_Queue_Client_Close_Event(uint8_t sock_id); //,uint8_t ssl_enable);
void Modem_Client_Check_Socket_Status (uint8_t scktID);
void Modem_get_server_ip(char *server_name);
void Modem_Wait_For_Sock_Read_To_Complete(void);
void Modem_HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandleArg);
void Modem_HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandleArg);
void Modem_HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle);

uint8_t Modem_Get_Sckt_Closed_Status (void);
void Modem_Clear_Sckt_Closed_Status (void);


/*----------------------------- Private variables -------------------------------------*/

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
  
#ifdef __cplusplus
  }
#endif
#endif  /* __WIFI_MODULE_H */
