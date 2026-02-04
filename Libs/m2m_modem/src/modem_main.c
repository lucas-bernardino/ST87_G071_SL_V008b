/**
******************************************************************************
* @file    modem_main.c
* @author  IoT AME
* @version V2.0.0
* @date    29-April-2017
* @brief   Modem Main program body. 
*          This file provides entry routine related to Modem Initialization..
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
#include <stdio.h>
#include <string.h>
#include "modem_main.h"
#include "main.h"
//#include "stm32_hal.h"
#if defined (STM32L4)
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
//#else
#endif

#if defined (STM32G0)
//#include "stm32l4xx_hal.h"
#include "stm32g0xx.h"
#endif

//#warning "Define STM32 familiy!!!!"
//#endif
#include "modem_interface.h"
#include "stm32_st87m01.h"
#include "modem_module.h"
//jrf #include "debug_conf.h"
#include <time32.h>

/* APNs availables */
#define  M2M_APN_CFG (0)    /*Vivo m2m apn config*/
#define  ZAP_APN_CFG (1)    /*Vivo regular fone line apn*/

/* Choose APN*/
#define MY_APN_CFG      M2M_APN_CFG

///* configuring apn*/
char * apn = APN_APP;
char * apn_login = "";
char * apn_pass = "";        
char * sec_key = "";

extern Modem_Status_t MODEM_Transmit_AT_Cmd(uint16_t size);

/**
* Private typedef -----------------------------------------------------------
*/

typedef enum {
    modem_state_reset = 0,
    modem_state_ready,
    modem_state_idle,
    modem_state_connected,
    modem_state_connecting,
    modem_state_disconnected,
    modem_state_time_socket,
    modem_open_tls,
    modem_send_json,
    modem_state_socket,
    modem_state_error,
    modem_undefine_state       = 0xFF,  
} modem_state_t;

/**
* Private define ------------------------------------------------------------
*/
#define APPLICATION_DEBUG_MSG 1


/**
* Private macro -------------------------------------------------------------
*/

/**
* Private function prototypes -----------------------------------------------
*/
//uint8_t modem_time_get_time_from_time_server(void);
uint8_t modem_time_get_time_from_time_server(uint8_t *slist);

void (*p_ind_modem_socket_data_received)(uint8_t * data_ptr, uint32_t message_size, uint32_t chunck_size);
void (*p_ind_modem_socket_client_remote_server_closed)(uint8_t * socket_closed_id);

void time_ind_modem_socket_data_received(uint8_t * data_ptr, uint32_t message_size, uint32_t chunck_size);
void time_ind_modem_socket_client_remote_server_closed(uint8_t * socket_closed_id);


/**
* Global Variables ---------------------------------------------------------
*/
static uint8_t TimeSocket;
static uint8_t TimeSocketClosed = 0;


//static uint8_t TimeSocketClosed = 0;

uint32_t last_ticks=0;

struct tm curr_time_date;
uint32_t gSecSince1900 = 0;

modem_state_t modem_state;

#define SERVER_LIST_MAX 6

uint8_t server_list=0;
char * modem_ntp_server[SERVER_LIST_MAX] = {
"time-a-g.nist.gov",
"time-c-g.nist.gov",
"time-d-g.nist.gov",
"time-a-wwv.nist.gov",
"time-a-b.nist.gov",
"utcnist.colorado.edu"
};

char * modem_ntp_server_IP[SERVER_LIST_MAX] = {
"129.6.15.28",            /* time-a-g.nist.gov */
"129.6.15.30",            /* time-c-g.nist.gov */ 
"129.6.15.27",            /* time-d-g.nist.gov */
"132.163.97.1",           /* time-a-wwv.nist.gov */
"132.163.96.6",           /* time-e-b.nist.gov */
"128.138.140.44"          /* utcnist.colorado.edu */
};

extern uint8_t getNTP_time;

/**
* Private functions ---------------------------------------------------------
*/

/**
* @brief  Modem Peripherals initialization
* @param  None
* @retval Modem_Status_t value
*/
int modem_peripherals_init(void)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS; 

  Modem_UART_Configuration(USART_MODEM_BAUDRATE);
    
  //Modem_PWRKEY_Configuration();
//  HAL_GPIO_WritePin(ST87_nRST_GPIO_Port, ST87_nRST_Pin, GPIO_PIN_SET); 
//  HAL_Delay(10); 
//  HAL_GPIO_WritePin(ST87_nRST_GPIO_Port, ST87_nRST_Pin, GPIO_PIN_RESET); 
//  HAL_Delay(10); 
//  HAL_GPIO_WritePin(ST87_nRST_GPIO_Port, ST87_nRST_Pin, GPIO_PIN_SET);   
   
  /* configure the timers  */
  Modem_Timer_Config();
  
  /*Init module and start timer*/
  Modem_Module_and_Timer_Init();
  
  return status;
}

extern uint32_t urc_counter;
extern uint32_t conta_ok;

/**
* @brief  Modem Main program
* @param  None
* @retval Modem_Status_t value
*/
int modem_main(void)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS; 
  
  p_ind_modem_socket_data_received = time_ind_modem_socket_data_received;
  p_ind_modem_socket_client_remote_server_closed = time_ind_modem_socket_client_remote_server_closed;
  
  modem_state = modem_state_idle;
  
  gSecSince1900 = NULL;
  TimeSocketClosed = MODEM_FALSE;

  /* Init the modem ST87 module */  
  status = modem_init(); 

  if(status!=MODEM_MODULE_SUCCESS)
  {
    #if MODEM_PRINT_DEBUG
        printf("\r\rError in Modem M95 Initialization or Not Present\r\n");
    #endif
    return status;
  }

  #if MODEM_PRINT_DEBUG
    printf("\r\nModem M95 is initialized.\r\n");
  #endif

//jrf  
//  /* date/time in unix secs past 1-Jan-70 */
//  while (modem_time_get_time_from_time_server(&server_list) != MODEM_MODULE_SUCCESS);
//  #if MODEM_PRINT_DEBUG  
//    printf("\r\nWait for NTP server Connection to be closed\r\n");
//  #endif
//    
//  getNTP_time = 1;  
//    //Modem_Control_Variables.start_sock_read = MODEM_TRUE;
//    
//  uint32_t temp = 0;
//                            
//  while ((TimeSocketClosed == MODEM_FALSE)&& (temp++ < 50))
//  {
//    HAL_Delay(500); 
//  }
//    
//  if(TimeSocketClosed == MODEM_FALSE)
//  {
//    status = modem_socket_client_close(TimeSocket,0);
//        
//    if(status!=MODEM_MODULE_SUCCESS)
//    {
//      #if MODEM_PRINT_DEBUG  
//      printf("\r\rError in Closer of NTP Server Socket\r\n");
//      #endif
//      return status;
//    }
//  }
//  else
//  {/* confirm if the time was acquired */
//    if (gSecSince1900 == NULL) 
//    {
//      status = MODEM_MODULE_ERROR;
//    }
//  }
  return status;
}



/**
 * @brief   Set local time and date came from the NTP server.
 * @author  JRF
 * @date    2018-04-16
 * @param   secsSince1900[in] - Unix time - time in epoch format second since 1900
 * @return  rawtime: time in epoch format second since 1900         
 */
int net_set_current_time(uint32_t secsSince1900)
{
    #define YEAR0          1900
    #define EPOCH_YEAR     1970
    #define SECS_DAY       (24L * 60L * 60L)
    #define LEAPYEAR(year) (!((year) % 4) && (((year) % 100) || !((year) %400)))
    #define YEARSIZE(year) (LEAPYEAR(year) ? 366 : 365)

    int ret = 0;
    /*time_t*/ uint32_t secs = secsSince1900;
    unsigned long dayclock, dayno;
    int year = EPOCH_YEAR;
    static const int _ytab[2][12] =
    {
        {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
        {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
    };

    gSecSince1900 = secs;
    dayclock = (unsigned long)secs % SECS_DAY;
    dayno    = (unsigned long)secs / SECS_DAY;

    curr_time_date.tm_sec  = (int) dayclock % 60;
    curr_time_date.tm_min  = (int)(dayclock % 3600) / 60;
    curr_time_date.tm_hour = (int) dayclock / 3600;
    curr_time_date.tm_wday = (int) (dayno + 4) % 7;        /* day 0 a Thursday */
    
    while(dayno >= (unsigned long)YEARSIZE(year)) {
      dayno -= YEARSIZE(year);
      year++;
    }
    
    curr_time_date.tm_year = year - YEAR0;
    curr_time_date.tm_yday = (int)dayno;
    curr_time_date.tm_mon  = 0;
    
    while(dayno >= (unsigned long)_ytab[LEAPYEAR(year)][curr_time_date.tm_mon]) {
      dayno -= _ytab[LEAPYEAR(year)][curr_time_date.tm_mon];
      curr_time_date.tm_mon++;
    }
    
    curr_time_date.tm_mday  = (int)++dayno;
    curr_time_date.tm_isdst = 0;
    
    curr_time_date.tm_year += 1900;
    curr_time_date.tm_mon += 1;

    return ret;
}


/**
 * @brief   Assign received time and date to out param.
 * @author  JRF
 * @date    2018-04-16
 * @param   t_time_date[out] - used by wolfssl to check the validity in certificate
 * @return  zero
 */
int net_get_current_time(struct tm* tm1)
{
  int ret = 0;
  
  if (tm1 == NULL) 
  {
    return -1;
  } 

  tm1->tm_year = curr_time_date.tm_year;
  tm1->tm_mon = curr_time_date.tm_mon;
  tm1->tm_mday = curr_time_date.tm_mday;
  tm1->tm_hour = curr_time_date.tm_hour;
  tm1->tm_min = curr_time_date.tm_min;
  tm1->tm_sec = curr_time_date.tm_sec;

  return ret;
}

/**
 * @brief   Return current seconds received from NTP server.
 * @author  JRF
 * @date    2018-04-16
 * @param   none
 * @return  rawtime: time in epoch format second since 1900      
 */
int32_t net_get_ntp_seconds(void)
{
  return last_ticks;
}

/**
* @brief Opens a socket to NTP server to read time
*
* @param no parameter
* @return Modem_Status_t value
*
* The Time Protocol may be implemented over the Transmission Control Protocol (TCP) or 
* the User Datagram Protocol (UDP). A host connects to a server that supports the Time 
* Protocol on port 37. The server then sends the time as a 32-bit unsigned integer in 
* binary format and in network byte order, representing the number of seconds since 00:00 
* (midnight) 1 January, 1900 GMT, and closes the connection. Operation over UDP requires 
* the sending of any datagram to the server port, as there is no connection setup for UDP.

* The fixed 32-bit data format means that the timestamp rolls over approximately every 136 years, 
* with the first such occurrence on 7 February 2036. Programs that use the Time Protocol must be 
* carefully designed to use context-dependent information to distinguish these dates from those in 1900.
* 
*/

uint8_t modem_time_get_time_from_time_server(uint8_t *slist)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint8_t server;
  
  server = *slist;
  
  //jrf status = modem_socket_client_open((uint8_t *)modem_ntp_server[server], 37, "TCP", &TimeSocket,0);
  
  //jrf status = modem_socket_client_open((uint8_t *)modem_ntp_server_IP[server], 37, "TCP", &TimeSocket,0);
  
  if (server<SERVER_LIST_MAX)
  {
    server++;
  }
  else
  {
    server = 0;
  }
  *slist = server;

  return status;
}




/**
****** Modem Indication User Callback: AWS Host ********
*/

/**
* @brief Called asychronously by modem driver when data arrives on modem interface
*
* This function calls a function pointer to read data. According to socket requirement NTP read or AWS read is called
* @param unsigned pointer to char - pointer to data buffer to be read.
* @param signed int - total message data size to be received
* @param signed int - partial message data size received of total message size
* @return no return
*/

void ind_modem_socket_data_received(uint8_t socket_id,uint8_t * data_ptr, uint32_t message_size, uint32_t chunck_size)
{
    (*p_ind_modem_socket_data_received)(data_ptr,message_size,chunck_size);
}

/**
* @brief Called asychronously by modem driver when client socket is closed by NTP server
*
* 
* @param unsigned pointer to char - pointer to socket id
*/
void ind_modem_socket_client_remote_server_closed(uint8_t * socket_closed_id)
{
    (*p_ind_modem_socket_client_remote_server_closed)(socket_closed_id);
}


/**
* @brief Called asychronously by modem driver when modem device is connected to GPRS 
*
* 
* @param no parameter
* @return no return
*/

void ind_modem_connected()
{
    #if MODEM_PRINT_DEBUG
        printf("\r\nModem connected to GPRS\r\n");
    #endif
    modem_state = modem_state_connected;
}


/**
* @brief Called asychronously by modem driver when data arrives on NTP Server socket
*
* 
* @param data_ptr : pointer to data buffer to be read.
* @param message_size : total message data size to be received
* @param chunck_size : partial message data size received of total message size
* @return no return
*/

void time_ind_modem_socket_data_received(uint8_t * data_ptr, uint32_t message_size, uint32_t chunck_size)
{
#if MODEM_PRINT_DEBUG
    printf("\r\nTime Received from NTP Server\r\n");
#endif
    ///	
    /// Time Protocol provides the time as a binary number of seconds since 1900,
    /// 
    /// 2,208,988,800 corresponds to 00:00  1 Jan 1970 GMT from 12:00:01 am on 1 January 1900 GMT
    ///
    last_ticks = ((data_ptr[0]<<24 )|(data_ptr[1]<<16)|(data_ptr[2]<<8)| data_ptr[3]) - 2208988800ul;	
    
    /* Now convert NTP time into everyday time.
     * Unix time starts on Jan 1 1970. In seconds, that's 2208988800.
     * Subtract seventy years.
     */
    //const uint32_t seventyYears = 2208988800UL;
    //uint32_t epoch = last_ticks - seventyYears;
    /* Print the hour, minute and second.
     * GMT is the time at Greenwich Meridian.
     */
    net_set_current_time(last_ticks); //epoch);    
}

/**
* @brief Called asychronously by modem driver when client socket is closed by NTP server
*
* 
* @param socket_closed_id : pointer to socket id
*/
void time_ind_modem_socket_client_remote_server_closed(uint8_t * socket_closed_id)
{
#if MODEM_PRINT_DEBUG
    printf("\r\nNTP server socket closed\r\n");
#endif    
    
    __disable_irq();

    TimeSocketClosed= MODEM_TRUE;
    
    __enable_irq();

    // Call here 
    modem_state = modem_state_idle;
}

//jrf 
/* Street lighting demo server info */
//uint16_t bytes2send=0;
//char server_ip[16]="162.248.102.207";
//uint32_t server_port =1337;
/*************************************************************************
 *
 * This function Creates a socket 
 * @param char *server - server IP address
 * @param uint32_t port - port
 * @param uint8_t * protocol - 0-UDP , 1-TCP
 * @param uint8_t Sckt_Id - socket Id
 * @return status 
 */

//modem_socket_client_open(uint8_t * hostname, uint32_t port_number, uint8_t * protocol, uint8_t *sock_id, uint8_t ssl_enable) 
uint8_t Create_Server_Socket (char *server, uint32_t port,uint8_t * protocol, uint8_t Sckt_Id)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  
  //if (protocol)
  //{
    status = modem_socket_client_open(server, port, protocol, &Sckt_Id,0); 
  //}
  //else
  //{
  //  status = modem_socket_client_open(server, port, "UDP", &Sckt_Id,0);
  //}  
  //modem_socket_client_open(uint8_t * hostname, uint32_t port_number, uint8_t * protocol, uint8_t * sock_id, uint8_t ssl_enable);
  return status;
}


uint8_t Create_Tcp_Connection (char *server, uint32_t port,uint8_t Sckt_Id)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
 
  status = modem_tcp_connection(server, port, &Sckt_Id); 
  return status;
}


uint8_t Check_Socket_Status (uint8_t Sckt_Id)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  Modem_Client_Check_Socket_Status (Sckt_Id); 
  
  //modem_socket_client_open(uint8_t * hostname, uint32_t port_number, uint8_t * protocol, uint8_t * sock_id, uint8_t ssl_enable);
  return status;
}

uint8_t Get_Socket_Closed_Status (void)
{
  return (Modem_Get_Sckt_Closed_Status());
}

uint8_t Get_Server_Ip (char *servername)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  status = modem_get_server_ip_address(servername);
  return status;
}

uint8_t Mqtt_Connect_Socket (char *server, uint32_t port)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  status = modem_socket_mqtt_open(server,port);
  return status;  
}


uint8_t Mqtt_Publish (char *topic, char *mes) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  status = modem_mqtt_publish(topic,mes);
  return status;
}

uint8_t Mqtt_Subscribe (void) //char *topic) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  status = modem_mqtt_subscribe();//topic); 
  return status;
}


uint8_t UDP_Server_send (uint8_t *pBuf,uint16_t size) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  
  status = modem_socket_client_write(0, size, pBuf);
  return status;
}

uint8_t TCP_Server_send (uint8_t *pBuf,uint16_t size)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;

  status = modem_socket_client_write(0, size, pBuf);
  return status;  
}

//uint8_t Get_NB_SENT (void)
//{
//  return(IO_modem_status_flag.nb_sent);
//}

uint8_t UDP_Server_read(uint8_t sckt_id) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  
  status = Modem_Socket_Read(sckt_id);
  return status;
}

uint8_t Close_UDP_Server_Socket (uint8_t sckt_id) 
{
    
    return (modem_socket_client_close(sckt_id,1)); //jrf to be fixed
}

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
