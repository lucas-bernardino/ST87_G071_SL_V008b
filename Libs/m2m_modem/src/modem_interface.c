/**
 ******************************************************************************
 * @file    modem_interface.c
 * @author  IoT AME
 * @version V2.1.0
 * @date    29-April-2017
 * @brief   User APIs implementation for X-CUBE-MODEM1
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
#include "modem_module.h"
#include "modem_globals.h"
#include "main.h"
//jrf#include "debug_conf.h"
//jrf #include "dio.h"
//jrf #include "atecc508cb.h"
//jrf #include "aws_iot_config.h"

//JRF 
//jrf #include "crypto_module.h"

//jrf extern t_aws_kit awsKit;

extern char clientKey[];
extern char clientCRT[];
extern char rootCA[];
extern char * apn;
extern char * apn_login;
extern char * apn_pass;

char Bands[] = BANDS_APP;

//jrf int aws_client_tls_receive(WOLFSSL* ssl, char *buf, int sz, void *ptr); //JRF
 
/** @addtogroup MIDDLEWARES
* @{
*/ 

/** @defgroup  NUCLEO_MODEM_INTERFACE
  * @brief Modem User API modules
  * @{
  */

/** @defgroup NUCLEO_MODEM_INTERFACE_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @addtogroup NUCLEO_MODEM_INTERFACE_Private_Variables
  * @{
  */
/* Private variables ---------------------------------------------------------*/

/**
  * @}
  */
  
/** @defgroup NUCLEO_MODEM_INTERFACE_Private_Functions
  * @{
  */



/**
  * @brief  modem_init
  *         User API for modem init
  * @param  None
  * @retval None
  */
Modem_Status_t modem_init(void) //modem_config* config)
{
  
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint32_t tick_ref;

#if MODEM_PRINT_DEBUG
  printf("\r\nInitializing ST87 Interface..\r\n");
#endif  

  Modem_Module_Init();
      
  IO_modem_status_flag.Modem_Enabled = MODEM_TRUE;  

  /* Send AT command */
  Modem_Reset_AT_CMD_Buffer();
  sprintf((char*)Modem_AT_Cmd_Buff,MODEM_AT_ATTENTION);
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
    if(status != MODEM_MODULE_SUCCESS) return status;
  }

#ifdef EU_DEMO  
  /* Send RESET command */
  Modem_Reset_AT_CMD_Buffer();
  sprintf((char*)Modem_AT_Cmd_Buff,"at#reset=0\r");
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
    if(status != MODEM_MODULE_SUCCESS) return status;
  } 
#endif
  
#ifdef AME_DEMO
  /* Send RESET command */
  Modem_Reset_AT_CMD_Buffer();
  sprintf((char*)Modem_AT_Cmd_Buff,"at#reset=1,1,0xc44f5b7f\r");
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
    if(status != MODEM_MODULE_SUCCESS) return status;
  }   
#endif  
  
  HAL_Delay(1000);
  
  //no simcard stay here forever
  do
  {
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
    HAL_Delay(25);
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
    HAL_Delay(25);
  }while(IO_modem_status_flag.simcard == 0);  

  
  /* Send INFO command */
  Modem_Reset_AT_CMD_Buffer();
  sprintf((char*)Modem_AT_Cmd_Buff,"at#info\r");
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
    if(status != MODEM_MODULE_SUCCESS) return status;
  }     
  
  /* Send ATE0 = echo off - Max Response Time 1000ms */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,MODEM_AT_ECHO_OFF); 
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
    if(status != MODEM_MODULE_SUCCESS) return status;
  }
      
  /*check connection status operators disabled */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+CSCON=0\r"); 
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
  }  
    
  /* AT+CFUN=0 */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+CFUN=0\r"); 
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
  }    
    
  /* AT+COPS force */
  Modem_Reset_AT_CMD_Buffer(); 
  //sprintf((char*)Modem_AT_Cmd_Buff,"AT+COPS=1,2,\"72404\",9\r");  //for TIM
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+COPS=0\r");  //automatic PLMN selection
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
  }
       
  /* AT#BANDSEL - set bands */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT#BANDSEL=%s\r",BANDS_APP); 
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
  }     
    
  /* AT+CGDCONT - set APN */

  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+CGDCONT=5,\"IPV4V6\",\"%s\"\r",apn); 
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
  }                
  
  /* AT+CFUN=1 */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+CFUN=1\r"); 
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
  } 
   
  tick_ref = HAL_GetTick();
  while (1)
  {
    HAL_Delay(1000);
       
//    /*check network operators*/
//    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
//    Modem_Reset_AT_CMD_Buffer(); 
//    sprintf((char*)Modem_AT_Cmd_Buff,"AT+COPS?\r"); 
//    status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//    if(status == MODEM_MODULE_SUCCESS)
//    {
//      status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
//      HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
//    }
//    HAL_Delay(200);
//    /*check network registration*/       
//    Modem_Reset_AT_CMD_Buffer(); 
//    sprintf((char*)Modem_AT_Cmd_Buff,"AT+CEREG?\r"); 
//    status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
//    if(status == MODEM_MODULE_SUCCESS)
//    {
//      status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
//      HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
//    }
//    HAL_Delay(200);    
    
    /*check connection status operators*/
    Modem_Reset_AT_CMD_Buffer(); 
    sprintf((char*)Modem_AT_Cmd_Buff,"AT+CSCON?\r"); 
    status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
    if(status == MODEM_MODULE_SUCCESS)
    {
      status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
      HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
    }   

    /* check attach */      
    Modem_Reset_AT_CMD_Buffer(); 
    sprintf((char*)Modem_AT_Cmd_Buff,"AT+CGATT?\r"); 
    status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
    if(status == MODEM_ATTACHED_OK)
    {
      status = MODEM_MODULE_SUCCESS;
      break;
    } 
        
    /*check timeout*/
    if ((HAL_GetTick() - tick_ref) > TIMEOUT_TO_ATTACH_TIME)
    {
        status = MODEM_TIME_OUT_ERROR;
        break;
    }
  }  
  
//  /*check network registration TO BE FIXED */       
//  Modem_Reset_AT_CMD_Buffer(); 
//  sprintf((char*)Modem_AT_Cmd_Buff,"AT+CEREG?\r"); 
//  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
//  if(status == MODEM_MODULE_SUCCESS)
//  {
//    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
//    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
//  }
//  HAL_Delay(500);
//  
  /*check network operators*/
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+COPS?\r"); 
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
  }
  HAL_Delay(500);
         
  /* Get IP address */   
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT#IPCFG?\r"); 
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
  }
//
//  /* Set DNS service */  
//  Modem_Reset_AT_CMD_Buffer(); 
//  //google DNS service
//  sprintf((char*)Modem_AT_Cmd_Buff,"at#ipparams=1,0,65535,60,0,8.8.8.8,2001:4860:4860::8888,1\r"); 
//  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
//  if(status == MODEM_MODULE_SUCCESS)
//  {
//    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
//    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
//  } 
  
  /* AT#IPPARAMS - Enable NB_SENT UR */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT#IPPARAMS=1,0,65535,60,1\r"); 
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
  } 
  
  //HAL_Delay (1000);
 
//  /* Get Server IP */  
//  Modem_Reset_AT_CMD_Buffer(); 
//  sprintf((char*)Modem_AT_Cmd_Buff,"at#dns=5,0,broker.hivemq.com\r"); 
//  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
//  if(status == MODEM_MODULE_SUCCESS)
//  {
//    status = Modem_Receive_AT_Resp(AT_BIG_TIMEOUT);  
//    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
//  } 
//  //HAL_Delay (1000);  
          
//  /* MQTT configuration */  
//  Modem_Reset_AT_CMD_Buffer(); 
//  //sprintf((char*)Modem_AT_Cmd_Buff,"at#mqttcfg= mqtt-explorer-9bd770bc_1,20,20,20,20\r"); 
//  sprintf((char*)Modem_AT_Cmd_Buff,"at#mqttcfg= mqtt-st8700-client,20,20,20,20\r"); 
//  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_RESET);
//  if(status == MODEM_MODULE_SUCCESS)
//  {
//    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);  
//    HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port, LED_NETWORK_Pin, GPIO_PIN_SET);
//  }
  //HAL_Delay (1000);
  
  //Modem attached 
  HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port,  LED_NETWORK_Pin, GPIO_PIN_RESET);
  return status;
}

Modem_Status_t modem_socket_client_status (uint8_t *sock_id) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  
  Modem_Client_Check_Socket_Status ((uint8_t)*sock_id);
  return status; 
}

/**
* @brief  modem_socket_client_open
*         Open a network TLS socket
* @param  Hostname hostname to connect to
*         portnumber portnumber of the Host to connect to
*         protocol tcp or udp protocol
*         sock_id socket id of the opened socket returned to the user
          use TLS or not
* @retval Modem_Status_t : return status of socket open request
*/
Modem_Status_t modem_socket_client_open(uint8_t * hostname, uint32_t port_number, uint8_t * protocol, uint8_t *sock_id, uint8_t ssl_enable) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint32_t tick_ref;
  int ret = 0;
  
  HAL_Delay (1000);
  //just to clean some previous message not cleared
  //status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    
  Modem_Queue_Client_Open_Event(hostname,port_number,protocol,*sock_id);//,ssl_enable);
  tick_ref = HAL_GetTick();
    
  //Modem_Control_Variables.AT_RESPONSE = MODEM_CONNECT_SOCKET_FAIL
  while (( IO_modem_status_flag.QIOPEN_Response_Received != MODEM_TRUE)&&(Modem_Control_Variables.AT_RESPONSE != MODEM_CONNECT_SOCKET_FAIL))
  {
    /*check timeout*/
    if ((HAL_GetTick() - tick_ref) > TIMEOUT_SOCKET_OPEN_EVENT)
    {
        status = MODEM_TIME_OUT_ERROR;
        break;
    }  
  }

  if (IO_modem_status_flag.QIOPEN_Response_Received == MODEM_TRUE)
  {
    IO_modem_status_flag.QIOPEN_Response_Received = MODEM_FALSE;
    status = MODEM_MODULE_SUCCESS;
  }
  else
  {
    if (Modem_Control_Variables.AT_RESPONSE == MODEM_CONNECT_SOCKET_FAIL)
    {
      status = MODEM_CONNECT_SOCKET_FAIL;
      Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
    }
    else
    {
      status = MODEM_TIME_OUT_ERROR;
    }
    if(status != MODEM_MODULE_SUCCESS) return status;
  }
 
#if 0  //jrf 1  
  if (ssl_enable)
  {
  
    /* Connect to AWS IoT over TLS handshaking with ECDHE-ECDSA-AES128-GCM-SHA256 cipher suite. 
       If TLS connection fails by AWS IoT during JITR, then return normal failure for the retry. */     
    
    /* Setup the WolfSSL library */
    wolfSSL_Init();
    
    ret = SSL_SUCCESS;
    ret = aws_client_net_tls_cb(&awsKit);

    if (ret == SSL_SUCCESS) 
    {
      if (awsKit.tls.context == NULL) 
      {
        awsKit.tls.context = wolfSSL_CTX_new(wolfTLSv1_2_client_method());
        if (awsKit.tls.context)
        {
          wolfSSL_CTX_set_verify(awsKit.tls.context, SSL_VERIFY_NONE, 0);
        }
      }

      if (awsKit.tls.context) 
      {
        wolfSSL_SetIORecv(awsKit.tls.context, aws_client_tls_receive);
        //wolfSSL_SetIORecv(awsKit.tls.context, (CallbackIORecv)iot_tls_read);
        
        wolfSSL_SetIOSend(awsKit.tls.context, aws_client_tls_send);
        //wolfSSL_SetIOSend(awsKit.tls.context, (CallbackIOSend)iot_tls_write);
          
        awsKit.tls.ssl = wolfSSL_new(awsKit.tls.context);
        if (awsKit.tls.ssl) 
        {
          wolfSSL_SetIOReadCtx(awsKit.tls.ssl, (void*)&awsKit.client);
          
          printf("JRF %x\n\r"),wolfSSL_GetIOReadCtx (awsKit.tls.ssl);
          
          wolfSSL_SetIOWriteCtx(awsKit.tls.ssl, (void*)&awsKit.client);
          
          printf("JRF %x\n\r"),wolfSSL_GetIOWriteCtx (awsKit.tls.ssl);
                          
          ret = wolfSSL_connect(awsKit.tls.ssl);
          printf("JRF CONNECTED!\n\r");
          if (ret != SSL_SUCCESS) 
          {
            ret = 8;//AWS_E_NET_TLS_FAILURE;
            printf("Error(%d) : Failed to TLS connect!", ret);
          }
        } 
        else
        {
          ret = 8;//AWS_E_NET_TLS_FAILURE;
          printf("Error(%d) : Failed to TLS init!", ret);
        }
      }
      else
      {
        ret = 8;//AWS_E_NET_TLS_FAILURE;
        printf("Error(%d) : Failed to TLS context init!", ret);
      }
    }
    
    /* Cleanup the WolfSSL library */
    if (ret == SSL_SUCCESS) 
    {
      ret = 0; //SUCCESS;
    }
    else 
    {
      if (awsKit.tls.ssl)
        wolfSSL_free(awsKit.tls.ssl);
                  
      if (awsKit.tls.context)
        wolfSSL_CTX_free(awsKit.tls.context);
                  
      wolfSSL_Cleanup();
                  
      //network_socket_disconnect(awsKit.socket);
    }
     return ret; 
  }

#endif	  
  *sock_id = Modem_Counter_Variables.Socket_Open_ID; //return the socket id to the user
  return status; 
}

/****************************
*****************************/

Modem_Status_t modem_socket_mqtt_open(uint8_t * server_ip, uint32_t port_number) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint32_t tick_ref;
  //int ret = 0;
 
  Modem_Queue_Mqtt_Client_Open_Event(server_ip,port_number);
     
  tick_ref = HAL_GetTick();
      
  IO_modem_status_flag.MQTTOPEN_Response_Received = MODEM_FALSE;
  Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
  while ((IO_modem_status_flag.MQTTOPEN_Response_Received != MODEM_TRUE)&&(Modem_Control_Variables.AT_RESPONSE != MODEM_MQTT_CONNECT_SOCKET_FAIL))
  {
    /*check timeout*/
    if ((HAL_GetTick() - tick_ref) > 20000) //JRF TBF  TIMEOUT_SOCKET_OPEN_EVENT*2)
    {
        status = MODEM_TIME_OUT_ERROR;
        break;
    }  
  }

  if (IO_modem_status_flag.MQTTOPEN_Response_Received == MODEM_TRUE)
  {
    IO_modem_status_flag.MQTTOPEN_Response_Received = MODEM_FALSE;
    status = MODEM_MODULE_SUCCESS;
  }
  else
  {
    if (Modem_Control_Variables.AT_RESPONSE == MODEM_MQTT_CONNECT_SOCKET_FAIL)
    {
      status = MODEM_MQTT_CONNECT_SOCKET_FAIL;
      Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
    }
    else
    {
      status = MODEM_TIME_OUT_ERROR;
    }
    if(status != MODEM_MODULE_SUCCESS) return status;
  }
 
  //*sock_id = Modem_Counter_Variables.Socket_Open_ID; //return the socket id to the user
  return status; 
}


Modem_Status_t modem_mqtt_publish(char *Topic, char *Message)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint32_t tick_ref;
 
  Modem_Queue_Mqtt_Publish_Event(Topic,Message);
     
  tick_ref = HAL_GetTick();
      
  IO_modem_status_flag.MQTTPUB_Response_Received = MODEM_FALSE;
  Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
  while ((IO_modem_status_flag.MQTTPUB_Response_Received != MODEM_TRUE)&&(Modem_Control_Variables.AT_RESPONSE != MODEM_MQTT_CONNECT_SOCKET_FAIL))
  {
    /*check timeout*/
    if ((HAL_GetTick() - tick_ref) > TIMEOUT_SOCKET_OPEN_EVENT*2)
    {
        status = MODEM_TIME_OUT_ERROR;
        break;
    }  
  }

  if (IO_modem_status_flag.MQTTPUB_Response_Received == MODEM_TRUE)
  {
    IO_modem_status_flag.MQTTPUB_Response_Received = MODEM_FALSE;
    status = MODEM_MODULE_SUCCESS;
  }
  else
  {
    if (Modem_Control_Variables.AT_RESPONSE == MODEM_MQTT_CONNECT_SOCKET_FAIL)
    {
      status = MODEM_MQTT_CONNECT_SOCKET_FAIL;
      Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
    }
    else
    {
      status = MODEM_TIME_OUT_ERROR;
    }
    if(status != MODEM_MODULE_SUCCESS) return status;
  }
 
  //*sock_id = Modem_Counter_Variables.Socket_Open_ID; //return the socket id to the user
  return status; 
}

Modem_Status_t modem_mqtt_subscribe(void) //char *Topic) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint32_t tick_ref;
 
  Modem_Queue_Mqtt_Subscribe_Event();//Topic);
     
  tick_ref = HAL_GetTick();
      
  IO_modem_status_flag.MQTTSUB_Response_Received = MODEM_FALSE;
  Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
  while ((IO_modem_status_flag.MQTTSUB_Response_Received != MODEM_TRUE)&&(Modem_Control_Variables.AT_RESPONSE != MODEM_MQTT_CONNECT_SOCKET_FAIL))
  {
    /*check timeout*/
    if ((HAL_GetTick() - tick_ref) > 10000) //jrf TBF TIMEOUT_SOCKET_OPEN_EVENT*2)
    {
        status = MODEM_TIME_OUT_ERROR;
        break;
    }  
  }

  if (IO_modem_status_flag.MQTTSUB_Response_Received == MODEM_TRUE)
  {
    IO_modem_status_flag.MQTTSUB_Response_Received = MODEM_FALSE;
    status = MODEM_MODULE_SUCCESS;
  }
  else
  {
    if (Modem_Control_Variables.AT_RESPONSE == MODEM_MQTT_CONNECT_SOCKET_FAIL)
    {
      status = MODEM_MQTT_CONNECT_SOCKET_FAIL;
      Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
    }
    else
    {
      status = MODEM_TIME_OUT_ERROR;
    }
    if(status != MODEM_MODULE_SUCCESS) return status;
  }
 
  //*sock_id = Modem_Counter_Variables.Socket_Open_ID; //return the socket id to the user
  return status; 
}

//

/**
* @brief  modem_socket_client_open
*         Open a network TLS socket
* @param  Hostname hostname to connect to
*         portnumber portnumber of the Host to connect to
*         protocol tcp or udp protocol
*         sock_id socket id of the opened socket returned to the user
          use TLS or not
* @retval Modem_Status_t : return status of socket open request
*/
Modem_Status_t modem_get_server_ip_address(char *servername) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint32_t tick_ref;
  int ret = 0;
   
  Modem_get_server_ip(servername);    
  tick_ref = HAL_GetTick();
    
  Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
  while (( IO_modem_status_flag.DNS_Response_Received != MODEM_TRUE)&&(Modem_Control_Variables.AT_RESPONSE != MODEM_DNS_REQUEST_FAIL))
  {
    /*check timeout*/
    if ((HAL_GetTick() - tick_ref) > TIMEOUT_DNS_REQUEST)
    {
        status = MODEM_TIME_OUT_ERROR;
        break;
    }  
  }

  if (IO_modem_status_flag.DNS_Response_Received == MODEM_TRUE)
  {
    IO_modem_status_flag.DNS_Response_Received = MODEM_FALSE;
    status = MODEM_MODULE_SUCCESS;
  }
  else
  {
    if (Modem_Control_Variables.AT_RESPONSE == MODEM_DNS_REQUEST_FAIL)
    {
      status = MODEM_DNS_REQUEST_FAIL;
      Modem_Control_Variables.AT_RESPONSE = MODEM_MODULE_SUCCESS;
    }
    else
    {
      status = MODEM_TIME_OUT_ERROR;
    }
    if(status != MODEM_MODULE_SUCCESS) return status;
  }
  return status; 
}



uint32_t tempo=0;

/**
* @brief  modem_socket_client_write
*         Write len bytes of data to socket
* @param  sock_id socket ID of the socket to write to
*         DataLength: data length to send
*         pData : pointer of data buffer to be written
* @retval Modem_Status_t : return status of socket write request
*/
Modem_Status_t modem_socket_client_write(uint8_t sock_id, uint16_t DataLength, char * pData)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint32_t tick_ref;

  //Check if sock_id is open
  if(!modem_open_sockets[sock_id])
  //JRF ????  return MODEM_NOT_READY;

  if(DataLength>=MODEM_MAX_SEND_BYTES_SIZE || DataLength<=0)
    return MODEM_NOT_SUPPORTED;

  Modem_Queue_Client_Write_Event(sock_id,DataLength,pData);
  tick_ref = HAL_GetTick();
  
  while (IO_modem_status_flag.QISEND_Response_Received != MODEM_TRUE)
  {
    /*check timeout*/
    //if ((HAL_GetTick() - tick_ref) > TIMEOUT_SOCKET_WRITE_EVENT*2)
    tempo = HAL_GetTick() - tick_ref;
    if (tempo > 10000)  //TIMEOUT_SOCKET_WRITE_EVENT*2)
    {
        status = MODEM_TIME_OUT_ERROR;
        break;
    }
  }

  //IO_modem_status_flag.QSSLSEND_Response_Received = MODEM_FALSE;
  IO_modem_status_flag.QISEND_Response_Received = MODEM_FALSE;
  //status = Modem_Receive_AT_Resp( AT_SHORT_TIMEOUT);
  return status;
}

//JRF new
Modem_Status_t modem_tcp_socket_client_write(uint8_t sock_id, uint16_t DataLength, char * pData)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint32_t tick_ref;

  //Check if sock_id is open
  if(!modem_open_sockets[sock_id])
  //JRF ????  return MODEM_NOT_READY;

  if(DataLength>=MODEM_MAX_SEND_BYTES_SIZE || DataLength<=0)
    return MODEM_NOT_SUPPORTED;

  Modem_Queue_Client_Write_Event(sock_id,DataLength,pData);
  tick_ref = HAL_GetTick();
  
  while (IO_modem_status_flag.QISEND_Response_Received != MODEM_TRUE)
  {
    /*check timeout*/
    //if ((HAL_GetTick() - tick_ref) > TIMEOUT_SOCKET_WRITE_EVENT*2)
    tempo = HAL_GetTick() - tick_ref;
    if (tempo > 10000)  //TIMEOUT_SOCKET_WRITE_EVENT*2)
    {
        status = MODEM_TIME_OUT_ERROR;
        break;
    }
  }

  //IO_modem_status_flag.QSSLSEND_Response_Received = MODEM_FALSE;
  IO_modem_status_flag.QISEND_Response_Received = MODEM_FALSE;
  //status = Modem_Receive_AT_Resp( AT_SHORT_TIMEOUT);
  return status;  
}


///**
//* @brief  Socket_Read
//*         Return len bytes of data from socket
//* @param  DataLength: data length to read
//* @retval Modem_Status_t : return status of socket read request
//*/
//Modem_Status_t Modem_Socket_Read(uint16_t DataLength)
//{
//  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//  
//  if (DataLength > MODEM_MAX_RECEIVE_BYTES_SIZE)
//    return 0;
//
//  Modem_Reset_AT_CMD_Buffer();                          
//                          
//  /* AT+QIRD = Retrieve the Received TCP/IP Data */
//  sprintf((char*)Modem_AT_Cmd_Buff,"AT+QIRD=%d,%d,%d,%d\r", 0,1,Modem_Counter_Variables.curr_sockID,DataLength);
//  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//
//  if(status != MODEM_MODULE_SUCCESS)
//  {
//    #if MODEM_PRINT_DEBUG
//      printf("\r\n ERROR In Socket Read\r\n");
//    #endif
//    IO_modem_status_flag.AT_Response_Received = MODEM_TRUE;
//    Modem_Control_Variables.AT_RESPONSE = MODEM_AT_CMD_RESP_ERROR;
//  }                      
//  return status;
//}

/**
* @brief  Socket_Read
*         Return len bytes of data from socket
* @param  DataLength: data length to read
* @retval Modem_Status_t : return status of socket read request
*/
Modem_Status_t Modem_Socket_Read(uint8_t sckID) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  
  Modem_Reset_AT_CMD_Buffer();                          
                          
  /* AT#IPREAD = Retrieve the Received TCP/IP Data */
  sprintf((char*)Modem_AT_Cmd_Buff,"AT#IPREAD=5,0\r");
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
  Modem_Reset_AT_CMD_Buffer();                          
  return status;
}




/**
* @brief  modem_socket_client_close
*         The SOCKC command allows to close socket
* @param  sock_close_id the socket ID of the socket which needs to be closed.
* @retval Modem_Status_t : return status of socket close request
*/
Modem_Status_t modem_socket_client_close(uint8_t sock_close_id, uint8_t ssl_enable)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  Modem_Control_Variables.enable_SockON_Server_Closed_Callback = MODEM_FALSE;
  if(modem_open_sockets[sock_close_id])
  {
    Modem_Queue_Client_Close_Event(sock_close_id);//,ssl_enable);
    return status;
  }
  else
    return MODEM_MODULE_ERROR;
}

/////
Modem_Status_t modem_tcp_connection(uint8_t * hostname, uint32_t port_number, uint8_t *sock_id)//, uint8_t ssl_enable) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  uint32_t tick_ref;
  int ret = 0;
    
  Modem_Tcp_Connection_Event(hostname,port_number,*sock_id);
  tick_ref = HAL_GetTick();
  HAL_Delay(1000);
   
  while (IO_modem_status_flag.TCP_CONNECT_Response_Received != MODEM_TRUE)
  {
    /*check timeout*/
    if ((HAL_GetTick() - tick_ref) > TIMEOUT_TCP_CONNECT)
    {
        status = MODEM_TIME_OUT_ERROR;
        IO_modem_status_flag.TCP_CONNECT_Send = MODEM_FALSE; //OK for at#connect didn´t arrive clear ok flag
        break;
    }  
  }
  
  if (IO_modem_status_flag.TCP_CONNECT_Response_Received == MODEM_TRUE)
  {
    IO_modem_status_flag.TCP_CONNECT_Response_Received = MODEM_FALSE;
    status = MODEM_MODULE_SUCCESS;
  }
  return status; 
}


/////

/**
* @brief  Socket_Pending_Data
*         Query pending data.It will returns the number of bytes of data waiting on socket
* @param None
* @retval uint8_t :number of bytes of data waiting on socket
*/
void Modem_Socket_Pending_Data()
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;

  /* AT+QIRD=<id>,<sc>,<sid>,<len><cr> */
  Modem_Reset_AT_CMD_Buffer();
//  modem_wait_for_command_mode();
  if(modem_open_sockets[Modem_Counter_Variables.sockon_query_id])
    {
      if(Modem_Control_Variables.stop_event_dequeue == MODEM_FALSE)
        //JRFModem_Control_Variables.stop_event_dequeue = MODEM_TRUE;
      
      //JRF to be fixed the params
      sprintf((char*)Modem_AT_Cmd_Buff,"AT+QIRD=0,1,%x,100\r",Modem_Counter_Variables.sockon_query_id);        
      status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
      if(status == MODEM_MODULE_SUCCESS) { }  
    }
}

/**
* @brief  modem_socket_server_open
*         Open a Server socket
* @param  None
* @retval Modem_Status_t : return status of server socket request
*/
Modem_Status_t modem_socket_server_open(uint32_t port_number, uint8_t * protocol) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  Modem_Reset_AT_CMD_Buffer();

  /* AT+S.SOCKD=portNo,t<cr> */  
//  sprintf((char*)Modem_AT_Cmd_Buff,AT_SERVER_SOCKET_OPEN,(int)port_number,protocol);        
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
    {
      status = Modem_Receive_AT_Resp( AT_SHORT_TIMEOUT);
    }
  return status; 
}

/**
* @brief  modem_socket_server_write
*         Write to a Server socket
* @param  None
* @retval Modem_Status_t : return status of server socket request
*/
Modem_Status_t modem_socket_server_write(uint16_t DataLength,char * pData) 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//  /*Can only write if there is a client connected*/
//  if(!modem_client_connected)
//    {
//      return Modem_NOT_READY;
//    }
//     __disable_irq();
//
////  Modem_Control_Variables.do_not_reset_push_MODEM_event = MODEM_TRUE;
////  Modem_Control_Variables.prevent_push_MODEM_event = MODEM_TRUE;
//  __enable_irq();
//
//  while(IO_status_flag.sock_read_ongoing || IO_status_flag.WIND64_count!= 0)//wait till any pending data is read
//    {
//      __NOP(); //nothing to do
//    }
//
//  wait_for_command_mode();
//
//  /*to make sure that by default the mode is not switched to command mode from data mode*/
//  Modem_Control_Variables.switch_by_default_to_command_mode = MODEM_FALSE;
//
//  /*Switch to Data Mode first*/
//  if(!IO_status_flag.data_mode)
//    {
//      Modem_switch_to_data_mode();//switch by default
//      while(!IO_status_flag.data_mode)
//        {
//          //Wait till data_mode is active
//          __NOP(); //nothing to do
//        }
//    }
//
//  /*Write the data on the uart*/
//  if(HAL_UART_Transmit(&UartModemHandle, (uint8_t *)pData, DataLength,1000)!= HAL_OK)
//    {
//      Error_Handler();
//      return Modem_HAL_UART_ERROR;
//    }
//  //HAL_Delay(100);//Wait for tx before switching back to command mode
//
//  /*Switch back to Command Mode*/
//  if(!IO_status_flag.command_mode)
//    {
//      Modem_switch_to_command_mode();//switch by default
//      while(!IO_status_flag.command_mode)
//        {
//          //Wait till command_mode is active
//          __NOP(); //nothing to do
//        }
//    }
//
//  Modem_Control_Variables.switch_by_default_to_command_mode = MODEM_TRUE;  /*back to default behaviour*/
//
//  __disable_irq();
//  Modem_Control_Variables.prevent_push_MODEM_event = MODEM_FALSE;
//  Modem_Control_Variables.do_not_reset_push_MODEM_event = MODEM_FALSE;
//  __enable_irq();
  
  return status;
}

/**
* @brief  Server Socket Close
*         Close a Server socket
* @param  None
* @retval Modem_Status_t : return status of server socket request
*/
Modem_Status_t modem_socket_server_close() 
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//  Reset_AT_CMD_Buffer();
//
//  /* AT+S.SOCKD=portNo,t<cr> */  
//  sprintf((char*)Modem_AT_Cmd_Buff,AT_SERVER_SOCKET_CLOSE);        
//  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  if(status == MODEM_MODULE_SUCCESS)
//    {
//      status = USART_Receive_AT_Resp( );
//    }

  return status; 
}

/**
* @brief  wait_for_command_mode
*         Waits till we are in command mode
* @param  None
* @retval None
*/
void modem_wait_for_command_mode(void)
{
//    while(!IO_status_flag.command_mode)
//      {
//        //Make sure we are in command mode, ideally we should do this in every User API?
//        __NOP(); //nothing to do
//      }
}

/**
* @brief  modem_file_delete
*         Delete a file
* @param  pFileName : File Name to be deleted
* @retval Modem_Status_t : return status of delete file request
*/
Modem_Status_t modem_file_delete(char * pFileName)
{
  /* AT+S.FSD: delete an existing file */  
  Modem_Status_t status = MODEM_MODULE_SUCCESS;  
//  Reset_AT_CMD_Buffer(); 
//  /* AT+S.FSL */
//  sprintf((char*)Modem_AT_Cmd_Buff,AT_DELETE_FILE,pFileName);  
//  
//  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  if(status == MODEM_MODULE_SUCCESS)
//    {
//      status = USART_Receive_AT_Resp( );
//    }
  return status;
}

/**
* @brief  modem_file_list
*         List existing filename
* @param  None
* @retval Modem_Status_t : return status of AT cmd request
*/

Modem_Status_t modem_file_list()
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;  
//  Queue_Wifi_File_Event(NULL,NULL,0);
//  status = USART_Receive_AT_Resp( );
  return status;
}

/**
* @brief  modem_file_show
*         Print the contents of an existing file
* @param  pFileName : pinter of file name
* @retval Modem_Status_t : return status of AT cmd request
*/

Modem_Status_t modem_file_show(uint8_t * pFileName)
{
//  if(pFileName==NULL)
//      return Modem_MODULE_ERROR;
//
  Modem_Status_t status = MODEM_MODULE_SUCCESS;  
//  Queue_Wifi_File_Event(NULL,pFileName,0);
//  status = USART_Receive_AT_Resp( );
  return status; 
}

/**
* @brief  modem_file_create
*         Create file for HTTP server
* @param  pFileName : pointer of file name to be created
*         alength   : length of file
* @retval Modem_Status_t : return status of AT cmd request
*/

Modem_Status_t modem_file_create(char *pFileName, uint16_t alength, char * pUserFileBuff)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//
//  if(alength >1024)
//    return Modem_AT_FILE_LENGTH_ERROR;
//
//  Reset_AT_CMD_Buffer();
//
//  /* AT+S.FSC=/index.html  */
//  sprintf((char*)Modem_AT_Cmd_Buff, AT_CREATE_NEW_HTML_FILE, pFileName, alength);
//
//  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  if(status == MODEM_MODULE_SUCCESS)
//  {
//    status = USART_Receive_AT_Resp( );
//    int len = strlen(pUserFileBuff);
//
//    if(len >= 1024)
//       return Modem_AT_FILE_LENGTH_ERROR;
//
//    /* AT+S.FSA=/index.html  */
//    sprintf((char*)Modem_AT_Cmd_Buff,AT_APPEND_FILE,pFileName,len);
//
//    status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//    if(status == MODEM_MODULE_SUCCESS)
//    {
//      memset(Modem_AT_Cmd_Buff, 0x00, sizeof Modem_AT_Cmd_Buff);
//      memcpy((char*)Modem_AT_Cmd_Buff, (char*) pUserFileBuff,len);
//      Modem_AT_Cmd_Buff[len+1]='\r';
//      status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//      if(status == MODEM_MODULE_SUCCESS)
//      {
//        status = USART_Receive_AT_Resp( );
//      }
//    }
//  }
  return status; 
}

/**
* @brief  modem_http_get
*         Issue an HTTP GET of the given path to the specified host
* @param  None
* @retval Modem_Status_t : return status of AT cmd response
*/

Modem_Status_t modem_http_get(uint8_t * hostname, uint8_t * path, uint32_t port_number)
{
//  if(hostname == NULL || path == NULL)
//    return Modem_MODULE_ERROR;
//  
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//
//  while(IO_status_flag.sock_read_ongoing || IO_status_flag.WIND64_count!= 0) //wait till any pending data is read
//    {
//      __NOP(); //nothing to do
//    }
//
//  // AT+S.HTTPGET=host.example.com,/index.html, port_number<cr>
//  //Queue the http-get command
//  Queue_Http_Event(hostname, path, port_number,NULL);
//
//  //Make the user wait anyway
//  status = USART_Receive_AT_Resp( );

  return status; 
}

/**
* @brief  modem_http_post
*         Issue an HTTP GET of the given path to the specified host
* @param  None
* @retval Modem_Status_t : status of Http Post Request
*/

Modem_Status_t modem_http_post(uint8_t * pURL_path)
{
//  if(pURL_path == NULL)
//    return Modem_MODULE_ERROR;
//  
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//  
//  while(IO_status_flag.sock_read_ongoing || IO_status_flag.WIND64_count!= 0)//wait till any pending data is read
//    {
//      __NOP(); //nothing to do
//    }
//  
//  // AT+S.HTTPPOST = posttestserver.com,/post.php,name=demo&email=mymail&subject=subj&body=message<cr>
//  Queue_Http_Event(NULL,NULL,0,pURL_path);
//  
//  //Make the user wait anyway
//  status = USART_Receive_AT_Resp( );
  return status;
}

/**
* @brief  modem_file_image_create
*         Downloads an updated file system via a single HTTP GET request to the
*         named host and path.
* @param  None
* @retval Modem_Status_t
*/
Modem_Status_t modem_file_image_create(uint8_t * pHostName, uint8_t * pFileName, uint32_t port_number)
{
//  if(pHostName == NULL || pFileName == NULL ||  port_number ==0)
//    return Modem_MODULE_ERROR;
//  
    Modem_Status_t status = MODEM_MODULE_SUCCESS;
//    Queue_Wifi_File_Event(pHostName,pFileName,port_number);
//    status = USART_Receive_AT_Resp( );
//    
//    /* Soft reset the module */
//    SET_Power_State(PowerSave_State);
    return status;
}

/**
* @brief  modem_file_erase_external_flash
*         This API allows to erase the content of the external flash
* @param  None
* @retval Modem_Status_t
*/
Modem_Status_t modem_file_erase_external_flash()
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;  

//  Reset_AT_CMD_Buffer();
//  ResetBuffer();
//
//  /* AT+S.HTTPDFSERASE */
//  sprintf((char*)Modem_AT_Cmd_Buff,AT_ERASE_FLASH_MEMORY);  
//  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//
//  /* Soft reset the module */
//  SET_Power_State(PowerSave_State);

   return status;
}

/**
* @brief  modem_fw_update
*         Issue an HTTP GET of the given path to the specified host and get the firmware updated
* @param  None
* @retval None
*/
Modem_Status_t modem_fw_update(uint8_t * hostname, uint8_t * filename_path, uint32_t port_number)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//  Queue_Wifi_FW_Update_Event(hostname,filename_path,port_number);
//  status = USART_Receive_AT_Resp( );
//  /* Soft reset the module */
//  SET_Power_State(PowerSave_State);
  return status;
}

/**
* @brief  modem_network_scan
*         Performs an immediate scan for available network
* @param  None
* @retval Modem_Status_t : Modem status error
*/
Modem_Status_t modem_network_scan(modem_scan *scan_result, uint16_t max_scan_number)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
//  modem_scanned_list = scan_result;
//  if(max_scan_number>MAX_MODEM_SCAN_NETWORK)
//    return Modem_NOT_SUPPORTED;
//  Modem_Counter_Variables.user_scan_number = max_scan_number;
//  
//  if(Modem_Control_Variables.Scan_Ongoing)
//  {
//    return Modem_AT_CMD_BUSY;
//  }
//  
//  Modem_Control_Variables.Scan_Ongoing = MODEM_TRUE;
//
//  /* AT+S.SCAN: performs an immediate scan for available networks */
//  Reset_AT_CMD_Buffer();
//  sprintf((char*)Modem_AT_Cmd_Buff,AT_Modem_SCAN);  
//  status = USART_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
//  if(status == MODEM_MODULE_SUCCESS)
//  {
//    status = USART_Receive_AT_Resp( );
//  }
//  
  /*At this point we have Modem_Scan_Buffer filled with RSSI and SSID values*/
  return status;
}

/**
* @brief  Set_MiniAP_Mode
*         Configure Modem module in AP mode.
          MiniAP is always configured in open mode (WEP not supported)
* @param  None
* @retval Modem_Status_t : status of AT cmd 
*/
Modem_Status_t modem_ap_start(uint8_t * ssid, uint8_t channel_num)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;  
  
//  /* Set the SSID : AT+S.SSIDTXT=<SSID>*/    
//  if(ssid)
//      status = SET_SSID((char*)ssid);
//  else 
//    /* default SSID : AT+S.SSIDTXT=SPWF_AP*/
//    status = SET_SSID(Modem_Config_Variables.modem_ssid);
//  
//  if(status != MODEM_MODULE_SUCCESS)
//    return Modem_SSID_ERROR;
//
//  /* Set the network privacy mode : AT+S.SCFG=modem_priv_mode,0*/ 
//   status = SET_Configuration_Value(MODEM_PRIV_MODE, None);
//  if(status != MODEM_MODULE_SUCCESS)
//    return Modem_CONFIG_ERROR;
//  
//  /* Set the network mode (1 = STA, 2 = IBSS, 3 = MiniAP) :: AT+S.SCFG=modem_mode,3*/  
//   status = SET_Configuration_Value(MODEM_MODE, Modem_MiniAP_MODE);
//   if(status != MODEM_MODULE_SUCCESS)
//    return Modem_CONFIG_ERROR;
//   
//  /* Set the channel number */  
//   status = SET_Configuration_Value(MODEM_CHANNEL_NUMBER, channel_num);
//   if(status != MODEM_MODULE_SUCCESS)
//    return Modem_CONFIG_ERROR; 
// 
//  /* Save the settings on the flash memory : AT&W*/ 
//  Save_Current_Setting();
//  
//  Modem_Control_Variables.Modem_Configuration_Done = MODEM_TRUE;
//
//  /* Soft reset the module */
//  SET_Power_State(PowerSave_State);
  
  return status; 
}

/**
* @brief  modem_connect
*         Connect to the GPRS 
* @param  APN       : access point name of the GPRS 
* @param  pass_key  : password 
* @retval Modem_Status_t : status of AT cmd 
*/
Modem_Status_t modem_connect(char * apn, char * pass_key)
{
  //uint32_t start_time;
  
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  
  if(Modem_Control_Variables.AT_Cmd_Ongoing == MODEM_FALSE)
    Modem_Control_Variables.AT_Cmd_Ongoing = MODEM_TRUE;
  else 
  {
    return MODEM_AT_CMD_BUSY;
  }
  
  /* Send AT+QICSGP = Select GPRS as the Bearer */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+QICSGP=1,\"%s\",\"%s\"\r",apn,pass_key); 
#if MODEM_PRINT_DEBUG  
  printf ("\r\n%s",Modem_AT_Cmd_Buff); //JRF TBC is not printing
#endif  
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    if(status != MODEM_MODULE_SUCCESS) return status;
  }  
  
  /* Send AT+QISTAT = Get Current connection status */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+QISTAT\r"); 
#if MODEM_PRINT_DEBUG  
  printf ("\r\n%s",Modem_AT_Cmd_Buff); //JRF TBC is not printing
#endif  
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    if(status != MODEM_MODULE_SUCCESS) return status;
  }   
  
  /* Send AT+QIREGAPP = Start TCPIP Task */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+QIREGAPP\r"); 
#if MODEM_PRINT_DEBUG  
  printf ("\r\n%s",Modem_AT_Cmd_Buff); //JRF TBC is not printing
#endif  
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    if(status != MODEM_MODULE_SUCCESS) return status;
  }   
  
  /* Send AT+QIACT = Activate GPRS Context
     takes up to 150s, it depends of the GSM network */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+QIACT\r"); 
#if MODEM_PRINT_DEBUG  
  printf ("\r\n%s",Modem_AT_Cmd_Buff); //JRF TBC is not printing
#endif  
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_BIG_TIMEOUT);
    if(status != MODEM_MODULE_SUCCESS) return status;
  }    
  
  /* Send AT+QISTAT = Get Current connection status 
     TO CHECK IF THE MODEM IS CONNECTED */
  Modem_Reset_AT_CMD_Buffer(); 
  sprintf((char*)Modem_AT_Cmd_Buff,"AT+QISTAT\r"); 
#if MODEM_PRINT_DEBUG  
  printf ("\r\n%s",Modem_AT_Cmd_Buff); //JRF TBC is not printing
#endif  
  status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
  if(status == MODEM_MODULE_SUCCESS)
  {
    status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
    if(status != MODEM_MODULE_SUCCESS) return status;
  }   
  
  Modem_Control_Variables.Modem_Configuration_Done = MODEM_TRUE;

  /* Soft reset the module */
  //SET_Power_State(PowerSave_State);
  
  Modem_Control_Variables.AT_Cmd_Ongoing = MODEM_FALSE;

  return status; 
}

/**
* @brief  get current time from modem clock
*         Connect to the GPRS 
* @param  out_timestamp: pointer to output timestamp 
* @retval Modem_Status_t : status of AT cmd 
*/
char * modem_get_time(void)
{
    Modem_Status_t status = MODEM_MODULE_ERROR;
    
    HAL_Delay(1000);
    Modem_Reset_Network_Time_Buffer();
    Modem_Reset_AT_CMD_Buffer(); 
    sprintf((char*)Modem_AT_Cmd_Buff,"AT+CCLK?\r"); 
    status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
    if(status == MODEM_MODULE_SUCCESS)
    {
      do
      {
        status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
        printf ("status = %d\n\r",status);
      }
      while ((status !=  MODEM_CLOCK_RESPONSE_OK) && (status!=MODEM_TIME_OUT_ERROR));
        
      if (status == MODEM_CLOCK_RESPONSE_OK)
      {
        /*if buffer lenght == 0 than wait resp again because modem process is out of sync*/
        if (strlen(modem_network_time_buff) == 0)
        {
          status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT); 
        }

        if (status == MODEM_CLOCK_RESPONSE_OK)
        {
          return modem_network_time_buff;              
        }
      }      
    }
    return NULL;
}

/**
* @brief  get current time from modem clock
*         Connect to the GPRS 
* @param  out_timestamp: pointer to output timestamp 
* @retval Modem_Status_t : status of AT cmd 
*/
Modem_Status_t modem_get_cell_towers_info(void)
{
    Modem_Status_t status = MODEM_MODULE_SUCCESS;
    
    HAL_Delay(1000);
    /*enable engineering mode (1) and Display serving cell and 1-6 neighboring 
      cells information */
    Modem_Reset_AT_CMD_Buffer(); 
    sprintf((char*)Modem_AT_Cmd_Buff,"AT+QENG=1,1\r"); 
    status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
    if(status == MODEM_MODULE_SUCCESS)
    {
        status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
        if(status != MODEM_MODULE_SUCCESS) return status;
    }
    
    HAL_Delay(1000);
    /*read cells informations configured in last command*/
    Modem_Reset_AT_CMD_Buffer(); 
    sprintf((char*)Modem_AT_Cmd_Buff,"AT+QENG?\r"); 
    status = Modem_Transmit_AT_Cmd(strlen((char*)Modem_AT_Cmd_Buff));
    if(status == MODEM_MODULE_SUCCESS)
    {
        status = Modem_Receive_AT_Resp(AT_SHORT_TIMEOUT);
        if(status != MODEM_MODULE_SUCCESS) return status;
    }    
    
    return status;
    
}

/**
* @brief  modem_standby
*         Configured Modem module to enter standby
* @param  arg_standby_time: standby time
* @retval Modem_Status_t : status of AT cmd
*/
Modem_Status_t modem_standby(uint8_t arg_standby_time)
{
  /*
  For Standby, the presence of Jumpers on JP4 and JP3 has the following behaviour:
  JP3 (middle and bottom): prevents standby and immediately wakes-up module
  JP3 (middle and top): no effect on standby
  JP4 (middle and right): prevents wakeup and standby runs forever
  JP4 (middle and left): no effect on standby
  */
  
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  
 
  return status;
}

/**
* @brief  modem_wakeup
*         wakeup the module from sleep by setting the GPIO6 through PC13
*         or allow it to go to sleep
*         Jumper needed on JP4
* @param  wakeup wakeup (MODEM_TRUE) or allow sleep(MODEM_FALSE)
* @retval Modem_Status_t : status of function call
*/
Modem_Status_t modem_wakeup(modem_bool wakeup)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;    
  
 
  return status;
}

/**
* @brief  modem_disconnect
*         disconnect the module from any AP
* @param  None
* @retval Modem_Status_t : status of AT cmd
*/
Modem_Status_t modem_disconnect(void)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  
       
  return status;
}

/**
* @brief  modem_enable
*         Enable/Disable the Modem interface
* @param  enable enable Modem (MODEM_TRUE) disable Modem (MODEM_FALSE)
* @retval Modem_Status_t : status of AT cmd
*/
Modem_Status_t modem_enable(modem_bool enable)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  
 
  return status;
}

/**
* @brief  modem_restore
*         Restore the Modem with default values.
* @param  None
* @retval Modem_Status_t : status of AT cmd
*/
Modem_Status_t modem_restore()
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;
  

       
  return status;
}



/**
* @brief  Modem_get_IP_address
*         Get the ip address
* @param  ip_addr : pointer to ip address
* @retval status  : status of AT cmd request
*/
Modem_Status_t Modem_get_IP_address(uint8_t *ip_addr)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;

  return status;
}

/**
* @brief  Modem_get_MAC_address
*         Get the MAC address
* @param  ip_addr : pointer to MAC address
* @retval status  : status of AT cmd request
*/
Modem_Status_t Modem_get_MAC_address(uint8_t *mac_addr)
{
  Modem_Status_t status = MODEM_MODULE_SUCCESS;

  return status;
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

