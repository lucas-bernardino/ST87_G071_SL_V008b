/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    crypto_module.c
 * @brief   function related to cryptography chip
 *
 * @author  Roberto Andrich
 * @date    2017-10-06
 *
 * @addtogroup CRYPTOGRAPHY
 * @{
 */


#ifndef __CRYPTO_MODULE_H
#define __CRYPTO_MODULE_H
#ifdef __cplusplus
 extern "C" {
#endif

//JRF
#include <wolfssl/ssl.h>
#include <wolfssl/internal.h>
#include "aws_iot_mqtt_client.h"

/******************************************************************************
*  DEFINES
******************************************************************************/
      
/** \name Max buffer size definition for certificates, MQTT message, WIFI credential, and so on.
   @{ */
#define AWS_ROOT_CERT_MAX				(2048)
#define AWS_CERT_LENGH_MAX				(1024)
#define AWS_WIFI_SSID_MAX				(32)
#define AWS_WIFI_PSK_MAX				(32)
#define AWS_HOST_ADDR_MAX				(64)
#define AWS_THING_NAME_MAX				(32)
#define AWS_CLIENT_NAME_MAX				(12)
#define AWS_ROOT_PUBKEY_MAX				(64)
#define AWS_MQTT_BUF_SIZE_MAX				(1024)
#define AWS_MQTT_TOPIC_MAX				(128)
/** @} */

/** \name Offset address definition for user data
   @{ */
//JRF need to confirm this offset   

//#warning "CRYPTO_PRINT_DEBUG ENABLED! TO BE CONFIRMED THE OFFSET +4 IT RESULTS IN WRONG EXTRACTION OF THE DATA...."

#define AWS_USER_DATA_OFFSET_SSID_LEN		(0+4)
#define AWS_USER_DATA_OFFSET_SSID		(4+4)
#define AWS_USER_DATA_OFFSET_PSK_LEN		(36+4)
#define AWS_USER_DATA_OFFSET_PSK		(40+4)
#define AWS_USER_DATA_OFFSET_HOST_LEN		(104+4)
#define AWS_USER_DATA_OFFSET_HOST		(108+4)
#define AWS_USER_DATA_OFFSET_THING_LEN		(140+4)
#define AWS_USER_DATA_OFFSET_THING		(144+4)
#define AWS_USER_DATA_BUTTON_STATE		(176+4)
#define AWS_USER_DATA_OFFSET_MAX		(180+4)
/** @} */   

/** \name Offset address definition for user data
   @{ */
//#define AWS_USER_DATA_OFFSET_SSID_LEN		(0)
//#define AWS_USER_DATA_OFFSET_SSID		(4)
//#define AWS_USER_DATA_OFFSET_PSK_LEN		(36)
//#define AWS_USER_DATA_OFFSET_PSK		(40)
//#define AWS_USER_DATA_OFFSET_HOST_LEN		(72)
//#define AWS_USER_DATA_OFFSET_HOST		(76)
//#define AWS_USER_DATA_OFFSET_THING_LEN	(140)
//#define AWS_USER_DATA_OFFSET_THING		(144)
//#define AWS_USER_DATA_BUTTON_STATE		(176)
//#define AWS_USER_DATA_OFFSET_MAX		(180)     

#define AWS_GET_USER_DATA_LEN(data, idx) 		((data[idx + 3] << 24) | (data[idx + 2] << 16) | (data[idx + 1] << 8) | data[idx])
#define AWS_CHECK_USER_DATA_LEN(len, max) 		((len == 0) || (len >= max))
#define AWS_CHECK_USER_DATA(data, idx) 			((data[idx] == 0) || (data[idx] == 0xFF))
     
/**
 * Types of current state for the Main task.
 */
typedef enum { 
	MAIN_STATE_INVALID,
	MAIN_STATE_INIT_KIT,
	MAIN_STATE_CHECK_KIT,
	MAIN_STATE_PROVISIONING,
	MAIN_STATE_RUN_KIT,
	MAIN_STATE_MAX
} KIT_MAIN_STATE;

/**
 * Types of buttons.
 */
enum { 
	AWS_KIT_BUTTON_1,
	AWS_KIT_BUTTON_2,
	AWS_KIT_BUTTON_3,
	AWS_KIT_BUTTON_MAX,
};

/**
 * Defines button state.
 */
typedef struct AWS_BUTTON_STATE {
	bool isPressed[AWS_KIT_BUTTON_MAX];
	bool state[AWS_KIT_BUTTON_MAX];
} t_awsButtonState;

/**
 * Defines the WolfSSL context
 */
typedef struct AWS_TLS {
	WOLFSSL_CTX *context;
	WOLFSSL *ssl;	
} MQTTTls;


#if !defined(MAX_MESSAGE_HANDLERS)
#define MAX_MESSAGE_HANDLERS 5 /* redefinable - how many subscriptions do you want? */
#endif

//enum QoS { QOS0, QOS1, QOS2 };
//
typedef struct
{
	int len;
	char* data;
} MQTTLenString;


typedef struct
{
  char* cstring;
  MQTTLenString lenstring;
} MQTTString;

typedef struct MQTTMessage
{
    enum QoS qos;
    unsigned char retained;
    unsigned char dup;
    unsigned short id;
    void *payload;
    size_t payloadlen;
} MQTTMessage;

typedef struct MessageData
{
    MQTTMessage* message;
    MQTTString* topicName;
} MessageData;

typedef struct MQTTClient
{
    unsigned int next_packetid,command_timeout_ms;
    size_t buf_size, readbuf_size;
    unsigned char *buf,*readbuf;
    unsigned int keepAliveInterval;
    char ping_outstanding;
    int isconnected;

    struct MessageHandlers
    {
      const char* topicFilter;
      void (*fp) (MessageData*);
    } messageHandlers[MAX_MESSAGE_HANDLERS];      /* Message handlers are indexed by subscription topic */

    void (*defaultMessageHandler) (MessageData*);

    Network* ipstack;
    Timer ping_timer;
#if defined(MQTT_TASK)
	Mutex mutex;
	Thread thread;
#endif 
} MQTTClient;

/**
 * Defines all data to be stored in slot 8 of ATECC508A.
 */
typedef struct AWS_USER_DATA {
	uint32_t ssidLen;
	uint8_t ssid[AWS_WIFI_SSID_MAX];
	uint32_t pskLen;
	uint8_t psk[AWS_WIFI_PSK_MAX];
	uint32_t hostLen;
	uint8_t host[AWS_HOST_ADDR_MAX];
	uint32_t thingLen;
	uint8_t thing[AWS_THING_NAME_MAX];
	uint32_t port;
	uint32_t clientIDLen;
	uint8_t clientID[AWS_CLIENT_NAME_MAX];
	uint32_t rootPubKeyLen;
	uint8_t rootPubKey[AWS_ROOT_PUBKEY_MAX];
} t_awsUserData;   



/**
 * Defines PEM certificates structure.
 */
typedef struct AWS_CERT {
	uint32_t signerCertLen;
	uint8_t signerCert[AWS_CERT_LENGH_MAX];
	uint32_t devCertLen;
	uint8_t devCert[AWS_CERT_LENGH_MAX];
} t_awsCert;

typedef struct AWS_KIT {
	bool quitMQTT;					//!< determines MQTT disconnection with AWS IoT.
	bool blocking;					//!< determines blocking mode, when receiving packets.
	bool nonBlocking;				//!< determines non-blocking mode, when sending packets.
	bool pushButtonState;			//!< Indicates state of SW0 button.
//	xQueueHandle notiQueue;			//!< Notification queue for communication between Provisioning and Main task.
//	xQueueHandle buttonQueue;		//!< Queue to deal with button event from ISR.
//	KIT_NOTI_STATE noti;			//!< Value of notiQueue above.
//	KIT_CLIENT_STATE clientState;	//!< State of MQTT client task.
	KIT_MAIN_STATE mainState;		//!< State of Main task.
//	KIT_ERROR_STATE	errState;		//!< State of exception.
	t_awsCert cert;					//!< Storage of both Signer and device certificates.
	t_awsUserData user;				//!< Storage of user data containing WIFI credential and so on.
//	t_awsMqttBuffer buffer;			//!< Storage for Paho MQTT to use this buffer.
//	t_awsMqttTopic topic;			//!< Storage for MQTT topics.
//	t_awsLedState led;				//!< Indicates state of LEDS in OLED1 board.
	t_awsButtonState button;		//!< Indicates state of buttons in OLED1 board.
//	Timer keepAlive;				//!< Timer to send PING packet.
//	Timer buttonISR;				//!< Timer to solve debounce issue.
//	Timer resetISR;					//!< Reset timer to measure 3 seconds.
//	Timer exceptionTimer;			//!< Timer to notify exception.
	MQTTClient client;              //!< Indicates client of MQTT
	MQTTTls tls;                    //!< Indicates the WolfSSL TLS context
//	SOCKET *socket;                 //!< Indicates the network socket
} t_aws_kit;

/******************************************************************************
* Public function prototypes
******************************************************************************/
std_return_t crypto_module_init(void);
std_return_t crypto_module_main(void);

//JRF
extern t_aws_kit awsKit;
void crypto_main (void);
int aws_client_net_tls_cb(t_aws_kit* kit);
//int aws_client_tls_receive(WOLFSSL* ssl, char *buf, int sz, void *ptr);
int aws_client_tls_send(WOLFSSL* ssl, char *buf, int sz, void *ptr);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CRYPTO_MODULE_H */

/** @} */
