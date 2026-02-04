/**
 ******************************************************************************
 * @file    modem_const.h
 * @author  Central LAB
 * @version V2.1.0
 * @date    17-May-2016
 * @brief   Describes the constants and defines in X-CUBE-MODEM1
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
 
#ifndef _MODEM_CONST_H
#define _MODEM_CONST_H

/** @defgroup NUCLEO_MODEM_INTERFACE_Private_Defines
  * @{
  */
    

#define MODEM_MAX_BUFFER_GLOBAL                       1200  
#define USER_DATA_BUFFER_SIZE                         3000
#define MODEM_RINGBUF_SIZE                            MODEM_MAX_BUFFER_GLOBAL
#define MODEM_AT_COMMAND_BUFF_SIZE                    1750 /*This because of certificate size*/
#define MODEM_MAX_RECEIVE_BYTES_SIZE                  1024 //JRF was 1200 /*max is 1500*/
#define MODEM_MAX_SEND_BYTES_SIZE                     1024 //JRF was 1200 /*max is 1460*/

#if (MODEM_MAX_RECEIVE_BYTES_SIZE > 1500)
    #error "QUECTEL MODEM SUPPORT 1500 BYTES MAX RECEPTION"
#endif

#if (MODEM_MAX_SEND_BYTES_SIZE > 1460)
    #error "QUECTEL MODEM SUPPORT 1460 BYTES MAX RECEPTION"
#endif

#if (MODEM_AT_COMMAND_BUFF_SIZE < 1750)
    #error "MODEM_AT_COMMAND_BUFF_SIZE MUST BE > 1750 TO SEND CERTICATE TO MODEM!"
#endif

#define MAX_NETWORK_TIME_SIZE                         64

#define AT_SHORT_TIMEOUT 100    //multiple pf 10ms  (1sec) (ST87 spec) 30     // multiple of 10ms (300ms) according to modem spec
#define AT_SHORT2_TIMEOUT 4000  // multiple of 10ms (40s) according to modem spec
#define AT_MID_TIMEOUT 200      // multiple of 10ms (2s)
#define AT_MID2_TIMEOUT 1500    // multiple of 10ms (15s)
#define AT_BIG_TIMEOUT 6000     // multiple of 10ms (60s) according to modem spec
#define N_TENTATIVES 3

#define DATA_MODE_TIMEOUT     15 // in seconds see M95 AT Commands Manual should be >=3

    
#define TIMEOUT_DNS_REQUEST  60000  /* 60sec */    
    
#define TIMEOUT_GET_NETWORK_TIME   60000 /*ms. wait time from network*/
#define SERVER_CONNECTION_TIMEOUT  120000 /*ms. Wait this timeout for server response */
#define TIMEOUT_SOCKET_WRITE_EVENT 10000 /*ms*/
#define TIMEOUT_SOCKET_OPEN_EVENT  2000 /*ms*/
#define TIMEOUT_TO_ATTACH_TIME     (5*60*1000) /*ms. wait time to attach the network*/
#define TIMEOUT_TCP_CONNECT        (20000)  /* see commands AT user manual */


/*Modem AT COMMANDS DEFINITIONS*/
#define MODEM_AT_ATTENTION                            "AT\r"
#define MODEM_AT_ECHO_OFF                             "ATE0\r"
#define MODEM_AT_PRODUCT_ID                           "ATI\r"
#define MODEM_AT_SOFTWARE_ID                          "AT+CGMR\r"
#define MODEM_AT_REPORT_ERROR                         "AT+CMEE=2\r"
#define MODEM_AT_SIM_SLOT1                            "AT+QDSIM=1,1\r"
    
#define MODEM_AT_QINDI                                "AT+QINDI=1\r"
#define MODEM_AT_TIME_SYNC                            "AT+QNITZ=1\r"

#define MODEM_AT_SOCKET_OPEN                          "AT+S.SOCKON=%s,%d,%s,ind\r"
#define MODEM_AT_SOCKET_WRITE                         "AT+S.SOCKW=%d,%d\r"
#define MODEM_AT_SOCKET_READ                          "AT+S.SOCKR=%d,%d\r"
#define MODEM_AT_SOCKET_CLOSE                         "AT+S.SOCKC=%d\r"
#define MODEM_AT_SERVER_SOCKET_OPEN                   "AT+S.SOCKD=%d,%s,ind\r" //with indication option
#define MODEM_AT_SERVER_SOCKET_CLOSE                  "AT+S.SOCKD=0\r"
#define MODEM_AT_QUERY_PENDING_DATA                   "AT+S.SOCKQ=%d\r"

    

/**
  * @}
  */

#endif /*_MODEM_CONST_H*/