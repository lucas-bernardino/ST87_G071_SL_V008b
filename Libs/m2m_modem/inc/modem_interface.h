/**
  ******************************************************************************
  * @file    modem_interface.h
  * @author  IoT AME
  * @version V2.1.0
  * @date    29-April-2017
  * @brief   Header file for X-CUBE-MODEM1 API
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
#ifndef __MODEM_INTERFACE_H
#define __MODEM_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/   
#include<stdint.h>
	 
   /** @addtogroup MIDDLEWARES
* @{
*/ 


/** @addtogroup  NUCLEO_MODEM_API 
  * @brief Modem_interface API
  * @{
  */ 


/** @addtogroup NUCLEO_MODEM_API_Private_Macros
  * @{
  */   
   
   
   /**
  * @}
  */


/** @addtogroup NUCLEO_MODEM_API_Private_Variables
  * @{
  */
/* Private variables ---------------------------------------------------------*/
   
   
/* Exported macro ------------------------------------------------------------*/
#define _ARG6(_0, _1, _2, _3, _4, _5, _6, ...) _6
#define NARG6(...) _ARG6(__VA_ARGS__, 6, 5, 4, 3, 2, 1, 0)
#define _FIVE_OR_SIX_ARGS_5(NAME, a, b, c, d, e) a, b, c, d, e, 1453727657
#define _FIVE_OR_SIX_ARGS_6(NAME, a, b, c, d, e, f) a, b, c, d, e, f
#define __FIVE_OR_SIX_ARGS(NAME, N, ...) _FIVE_OR_SIX_ARGS_ ## N (NAME, __VA_ARGS__)
#define _FIVE_OR_SIX_ARGS(NAME, N, ...) __FIVE_OR_SIX_ARGS(NAME, N, __VA_ARGS__)
#define FIVE_OR_SIX_ARGS(NAME, ...) NAME(_FIVE_OR_SIX_ARGS(NAME, NARG6(__VA_ARGS__), __VA_ARGS__))

#define modem_socket_client_security(...) FIVE_OR_SIX_ARGS(modem_socket_client_security, __VA_ARGS__)

/* Exported constants --------------------------------------------------------*/
//#define GPIO_IN                 "in"
//#define GPIO_OUT                "out"
//
//#define  GPIO_Off               '0'
//#define  GPIO_Rising            'R'
//#define  GPIO_Falling           'F'
//#define  GPIO_Both              'B'      
//   
//typedef enum
//{ 
//  GPIO_OFF      = 0,
//  GPIO_ON,
//} GpioWriteValue;
//
//typedef enum
//{ 
//  GPIO_PIN0     = 0,
//  GPIO_PIN1,
//  GPIO_PIN2,
//  GPIO_PIN3,
//  GPIO_PIN4,
//  GPIO_PIN5,
//  GPIO_PIN6,
//  GPIO_PIN7,
//  GPIO_PIN8,
//  GPIO_PIN9,
//  GPIO_PIN10,
//  GPIO_PIN11,
//  GPIO_PIN12,
//  GPIO_PIN13,
//  GPIO_PIN14,
//  GPIO_PIN15
//} GpioPin;

typedef enum
{
  MODEM_FALSE         = 0,
  MODEM_TRUE          = 1,
  UNDEF               = 0xFF
} modem_bool;

//typedef enum
//{
//  None          = 0, 
//  WEP           = 1,
//  WPA_Personal  = 2,
//} Modem_Priv_Mode;

/********** Modem Error *************/
typedef enum
{ 
  MODEM_MODULE_SUCCESS           = 0,
  MODEM_TIME_OUT_ERROR           = 1,  
  MODEM_MODULE_ERROR,
  MODEM_HAL_OK,
  MODEM_NOT_SUPPORTED,
  MODEM_NOT_READY,
  MODEM_NO_FILE,
  MODEM_TIME_SYNC,
  MODEM_SCAN_FAILED,
  MODEM_AT_CMD_BUSY,
  MODEM_SSID_ERROR,
  MODEM_SecKey_ERROR,
  MODEM_CONFIG_ERROR,
  MODEM_STA_MODE_ERROR,
  MODEM_AP_MODE_ERROR,
  MODEM_AT_CMD_RESP_ERROR,
  MODEM_AT_FILE_LENGTH_ERROR,
  MODEM_HAL_UART_ERROR,
  MODEM_IN_LOW_POWER_ERROR,
  MODEM_HW_FAILURE_ERROR,
  MODEM_HEAP_TOO_SMALL_WARNING,
  MODEM_STACK_OVERFLOW_ERROR,
  MODEM_HARD_FAULT_ERROR,
  MODEM_MALLOC_FAILED_ERROR,
  MODEM_INIT_ERROR,
  MODEM_POWER_SAVE_WARNING,
  MODEM_SIGNAL_LOW_WARNING,
  MODEM_JOIN_FAILED,
  MODEM_SCAN_BLEWUP,
  MODEM_START_FAILED_ERROR,
  MODEM_EXCEPTION_ERROR,
  MODEM_DE_AUTH,
  MODEM_DISASSOCIATION,
  MODEM_UNHANDLED_IND_ERROR,
  MODEM_RX_MGMT,
  MODEM_RX_DATA,
  MODEM_RX_UNK,
  MODEM_IP_INITIAL,
  MODEM_IP_START,
  MODEM_IP_CONFIG,
  MODEM_IP_IND,
  MODEM_IP_GPRSACT,
  MODEM_IP_STATUS,
  MODEM_TCP_CONNECTING,
  MODEM_UDP_CONNECTING,
  MODEM_IP_CLOSE, 
  MODEM_CONNECT_OK, 
  MODEM_PDP_DEACT,
  MODEM_WAITING_GET_TIME,
  MODEM_TIME_UPDATED,
  MODEM_CLOCK_RESPONSE_OK,
  MODEM_CONNECT_SOCKET_FAIL,
  MODEM_SOCKET_FAIL,
  MODEM_SOCKET_CLOSED,
  MODEM_MQTT_CONNECT_SOCKET_FAIL,
  MODEM_DNS_REQUEST_FAIL,
  MODEM_DEACT_OK,
  MODEM_ATTACHED_OK   //jrf
} Modem_Status_t;

typedef enum
{
  modem_active        = 0,
  modem_reactive      = 1,
  modem_sleep         = 2,
} modem_power_mode;

typedef enum
{
  modem_low           = 0,
  modem_medium        = 1,
  modem_high          = 2,
  modem_max           = 3,
} modem_tx_power_level;

typedef enum
{
  modem_off           = 0,
  modem_on            = 1,
  modem_custom        = 2,
} modem_dhcp_mode;

typedef struct
{  
  modem_bool     wpa;
  modem_bool     wpa2;
  modem_bool     wps;
} modem_security;

typedef struct
{  
  uint8_t       channel_num;
  int           rssi;                     
  char          ssid[30];  
  modem_security sec_type;
} modem_scan;

typedef struct
{  
  modem_bool             ht_mode;
  modem_power_mode            power;
  modem_tx_power_level        power_level;
  modem_dhcp_mode             dhcp;
  char*                 ip_addr;
  char*                 netmask_addr;
  char*                 gateway_addr;
  char*                 dns_addr;
  char*                 host_name;
  modem_bool             web_server;
  char*                 ap_domain_name;
  char*                 ap_config_page_name;
  uint32_t              http_timeout;
  uint32_t              dhcp_timeout;
  uint8_t               modem_region;     
  uint32_t              modem_baud_rate;
} modem_config;

/* Exported functions ------------------------------------------------------- */

#ifdef MODEM_USE_VCOM
void modem_vcom(void);
#endif

Modem_Status_t   modem_init(void); //modem_config* config);
Modem_Status_t   modem_restore(void);
Modem_Status_t   modem_enable(modem_bool enable);
Modem_Status_t   modem_disconnect(void);
Modem_Status_t modem_connect(char * apn, char * pass_key);
Modem_Status_t   modem_ap_start(uint8_t * ssid, uint8_t channel_num);
//Modem_Status_t   modem_adhoc_create(uint8_t * ssid, Modem_Priv_Mode priv_mode);
Modem_Status_t   modem_network_scan(modem_scan *scan_result, uint16_t max_scan_number);
void            modem_reset(void);

/******** Modem Clock Function **********/
char * modem_get_time(void);

/******** Modem geolocation Function **********/
Modem_Status_t modem_get_cell_towers_info(void);

/******** Modem Socket Function **********/
//Modem_Status_t   modem_socket_client_open(uint8_t * hostname, uint32_t port_number, uint8_t * protocol, uint8_t * sock_id);
Modem_Status_t   modem_socket_client_open(uint8_t * hostname, uint32_t port_number, uint8_t * protocol, uint8_t  *sock_id, uint8_t ssl_enable);

Modem_Status_t   modem_socket_client_write(uint8_t sock_id, uint16_t DataLength,char * pData);
Modem_Status_t   modem_socket_client_close(uint8_t sock_close_id,uint8_t ssl_enable);
Modem_Status_t   modem_socket_client_security(uint8_t* tls_mode, uint8_t* root_ca_server, uint8_t* client_cert, uint8_t* client_key, uint8_t* client_domain, uint32_t tls_epoch_time);
Modem_Status_t modem_socket_client_status (uint8_t *sock_id);

//JRF new
Modem_Status_t modem_TLS_socket_client_open(uint8_t * hostname, uint32_t port_number, uint8_t * protocol, uint8_t * sock_id, uint8_t ssl_enable);

/********* Modem Socket Server ********/
Modem_Status_t   modem_socket_server_open(uint32_t port_number, uint8_t * protocol);
Modem_Status_t   modem_socket_server_write(uint16_t DataLength,char * pData);
Modem_Status_t   modem_socket_server_close(void);

/*** FileSystem Request ***********/
Modem_Status_t   modem_file_create(char *pFileName,uint16_t alength,char * databuff);
Modem_Status_t   modem_file_delete(char * pFileName);
Modem_Status_t   modem_file_list(void);
Modem_Status_t   modem_file_show(uint8_t * pFileName);
Modem_Status_t   modem_file_image_create(uint8_t * pHostName,uint8_t * pFileName, uint32_t port_number);
Modem_Status_t   modem_file_erase_external_flash(void);

/*** HTTP File Request ***********/
Modem_Status_t   modem_http_get(uint8_t * hostname, uint8_t * path, uint32_t port_number);
Modem_Status_t   modem_http_post(uint8_t * url_path);

Modem_Status_t   modem_fw_update(uint8_t * hostname, uint8_t * filename_path, uint32_t port_number);

/*** Power Configuration **********/
Modem_Status_t   modem_standby(uint8_t arg_standby_time);
Modem_Status_t   modem_wakeup(modem_bool enable);

/*** GPIO Configuration **********/
//uint8_t         modem_gpio_init(GpioPin pin, char* dir, char irq);
//uint8_t         modem_gpio_read(GpioPin pin, uint8_t *val, uint8_t *dir);
//uint8_t         modem_gpio_write(GpioPin pin, GpioWriteValue value);

void            UART_Configuration(uint32_t baud_rate);
void            GPIO_Configuration(void);
void            Timer_Config(void);
void            UART_Msg_Gpio_Init(void);
void            USART_PRINT_MSG_Configuration(uint32_t baud_rate);

/****** Modem Status Variable Information *******/
Modem_Status_t   Modem_get_IP_address(uint8_t *ip_addr);
Modem_Status_t   Modem_get_MAC_address(uint8_t *mac_addr);

/******** Modem Indication User Callback: For User to implement *********/
void            ind_modem_warning(Modem_Status_t warning_code);
void            ind_modem_error(Modem_Status_t error_code);
void            ind_modem_connection_error(Modem_Status_t status_code);
void            ind_modem_connected(void);
void            ind_modem_ap_ready(void);
void            ind_modem_ap_client_joined(uint8_t * client_mac_address);
void            ind_modem_ap_client_left(uint8_t * client_mac_address);
void            ind_modem_on(void);
void            ind_modem_packet_lost(Modem_Status_t status_code);
void            ind_modem_gpio_changed(void);
void            ind_modem_socket_data_received(uint8_t socket_id, uint8_t * data_ptr, uint32_t message_size, uint32_t chunk_size);
void            ind_modem_socket_client_remote_server_closed(uint8_t * socketID);
void            ind_modem_socket_server_data_lost(void);
void            ind_socket_server_client_joined(void);
void            ind_socket_server_client_left(void);
void            ind_modem_http_data_available(uint8_t * data_ptr,uint32_t message_size);
void            ind_modem_file_data_available(uint8_t * data_ptr);
void            ind_modem_resuming(void);


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
	
#endif /* __MODEM_INTERFACE_H */
