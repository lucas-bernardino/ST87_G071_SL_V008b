/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    cloud_module.h
 * @brief   function related to cloud connectivity
 *
 * @author  Roberto Andrich
 * @date    2017-11-20
 *
 * @addtogroup CLOUD
 * @{
 */


/******************************************************************************
* Include statements
******************************************************************************/
#include "main.h"
#include "ctype.h"
#include "string.h"
#include "std_types.h"          /* fixed-width types, max values, std return */
#include "utilities.h"          /* module header */
#include "modem_const.h"
#include "m2m_modem.h"
#include "cloud_module.h"
#include "metro.h"
#include "sensor_module.h"
#include "cmsis_os.h"
#include "modem_module.h"


#define N_TRY   2

//#define TTI_DEFAULT 10
extern osMessageQueueId_t MetroQueueHandle;
extern osMessageQueueId_t SensorQueueHandle;
extern osMessageQueueId_t MetroCommStatusHandle;
extern osMessageQueueId_t ST87CommStatusHandle;
extern osMessageQueueId_t LedStatusQueueCloudHandle;
extern uint16_t ipread_cnt;
extern uint16_t iprecv_cnt;

typedef struct
{
  lamp_info_t Metro_info;
  sensor_data_t XL_info;
  uint16_t tx_cnt;
  uint8_t tti;
}cloud_data_t;

cloud_data_t cloud_data;


/**
 * @brief   cloud state machine.
 */
typedef enum
{
    DEMO_INIT,                    /**< Initializing */
    DEMO_MODEM_INIT,              /**< Initializing modem connection */
    DEMO_PARAMS_INIT,             /**< Initializing cloud parameters */
    DEMO_CONNECTING,              /**< connecting to mqtt server */
    DEMO_ENABLE_AUTO_RECONNECT,   /**< enable auto reconnecting */
    DEMO_SUBSCRIBING,             /**< subscribing to topic */
    DEMO_CONFIG_PUBLISH_QOS,      /**< configuring qos options for publish */
    DEMO_CONNECTED,               /**< sending and receiving data packets */
    DEMO_MODEM_ERROR,             /**< modem connection error */
    DEMO_ERROR,                   /**< cloud connection error */
    DEMO_WAIT_TIME,               /**< wait time after errors */
    DEMO_CREATE_SOCKET,
    DEMO_TCP_CONNECT,
    //DEMO_CREATE_UDP_SOCKET,
    //DEMO_CREATE_TCP_SOCKET,
    /*DEMO_CREATE_UDP_SOCKET_TO_WRITE,
    DEMO_CREATE_UDP_SOCKET_TO_READ,*/
    DEMO_CHECK_SOCKET_STATUS,
    DEMO_SEND_UDP_DATA,
    DEMO_WAIT_NB_SENT,
    DEMO_READ_UDP_DATA,
    DEMO_CLOSE_UDP_SOCKET,
    DEMO_WAITING_TO_SEND_DATA,
    DEMO_DNS_CFG,
    DEMO_CREATE_MQTT_SOCKET,
    DEMO_CREATE_MQTT_CONNECT,
    DEMO_MQTT_SUBSCRIBE,
    DEMO_MQTT_PUBLISH_ENERGY,
    DEMO_MQTT_PUBLISH_VRMS,
    DEMO_MQTT_PUBLISH_IRMS,
    DEMO_MQTT_PUBLISH_BOARD_ID,
    DEMO_MQTT_PUBLISH_LAMP_STATUS,
    DEMO_MQTT_PUBLISH_LAMP_DIMM,
    DEMO_MQTT_PUBLISH_TEMP,
    DEMO_MQTT_PUBLISH_ACC_X,
    DEMO_MQTT_PUBLISH_ACC_Y,
    DEMO_MQTT_PUBLISH_ACC_Z,
    DEMO_MQTT_CLOSE,
    DEMO_GET_BROKER_IP,
    DEMO_TEST_MODE
} demo_state_t;

demo_state_t demo_state = DEMO_INIT;
uint8_t send_data=0;
uint32_t udp_timer=0;
lamp_info_t Load_metrology_data;

sensor_data_t Load_Sensor_data;

led_status_t led_state;

uint32_t energy_test=0;
uint16_t temp_test=0;

uint16_t bytes2send=0;


/* joinstlab.net */
char server_ip[]=SERVER_IP;  //"162.248.102.96";
uint32_t server_port;

//uint32_t server_port_write = 1337;
//static uint8_t SocketID_Write=1;
//uint32_t server_port_read  = 1337; //1338;
//static uint8_t SocketID_Read=0;
static uint8_t SocketID=0;
uint32_t conn_number=0;
uint8_t try_again=N_TRY;
char Protocol[3];
//char broker_name[] = "broker.hivemq.com";
char broker_ip[20] = "162.248.102.207";
//uint16_t broker_port = 1883;
//char topic[] = "ST";
//char topic_message [] = "1234";
//uint32_t test_mes = 280000;

uint8_t sizeTest=0;
uint8_t getOsMes[32];
uint32_t st87_rst_counter=0;
float teste1=0;
uint8_t IsST87_Comm_ongoing=false;
uint8_t STPM32_Comm_ongoing=false;
uint8_t get_nb_sent_status=0;



uint8_t Create_Server_Socket (char *server, uint32_t port,uint8_t *protocol, uint8_t Sckt_Id);
uint8_t Create_Tcp_Connection (char *server, uint32_t port,uint8_t Sckt_Id);
uint8_t Check_Socket_Status (uint8_t Sckt_Id);
uint8_t Close_UDP_Server_Socket (uint8_t sckt_id);

//extern uint8_t led_state;
/**
 * @brief   Returns the number of milliseconds since the board began running
 *          the current program.
 * @author  JRF
 * @date    2024-08-03
 * @return  Number of milliseconds since the program started.
 */
uint32_t millis (void)
{
    return HAL_GetTick();
}

//jrf
/**
* @brief main entry function to AWS IoT code
*
* @param no parameter
* @return IoT_Error_t status return
*/
void demo_controller(void) 
{
    std_return_t status;
    uint32_t time_ref;
    uint32_t clock_update_time_ref;
    
    if (sckt_type == UDP_TYPE)
    {
      memcpy(&Protocol,"UDP",3); 
#ifdef AME_DEMO      
      server_port = US_UDP_SERVER_PORT;
#endif
#ifdef EU_DEMO     
      if (port_number == 0)
      {
        server_port = EU_UDP_SERVER_PORT;
      }
      else
      {
        server_port = EU_UDP_SERVER_PORT2;
      }
#endif      
    }
    
    if (sckt_type == TCP_TYPE)
    {
      memcpy(&Protocol,"TCP",3);
#ifdef AME_DEMO      
      server_port = US_TCP_SERVER_PORT;
#endif
#ifdef EU_DEMO    
      if (port_number == 0)
      {      
        server_port = EU_TCP_SERVER_PORT;
      }
      else
      {
        server_port = EU_TCP_SERVER_PORT2;
      }
#endif      
    }

    /*check if a reset has happened and set appropriated cloud apps status 
      Obs.: it is called again in datalogger only to clear the reset flags*/
    //check_cloud_reset();
    
    /**************************************************************************
    * Cloud State Machine
    **************************************************************************/
    while (true)
    {
        //check_periodic_reset();

        osMessageQueueGet(MetroQueueHandle,&Load_metrology_data , 0, 0);      //get message from metro task
        osMessageQueueGet(SensorQueueHandle,&Load_Sensor_data , 0, 0);        //get message from sensor task
        osMessageQueueGet(MetroCommStatusHandle,&STPM32_Comm_ongoing , 0, 0); //get message from metro communication task    
        osMessageQueueGet(LedStatusQueueCloudHandle,&led_state , 0, 0);       //get message from cloud task
        
        if ((led_state.update_state == true)&&(demo_state==DEMO_WAITING_TO_SEND_DATA))
        {
          led_state.update_state = false; 
          Load_metrology_data.status.state = led_state.state;
          Load_metrology_data.status.dimmer = led_state.dimmer;
          osMessageQueueReset(LedStatusQueueCloudHandle);
          udp_timer=1000;
        }  
            
        /* connection closed -> reopen */
        if (Modem_Get_Sckt_Closed_Status())
        {
          Modem_Clear_Sckt_Closed_Status();
          demo_state = DEMO_CREATE_SOCKET;
          IsST87_Comm_ongoing = true;
          try_again = N_TRY;
        }        
        
        switch (demo_state)
        {
          case DEMO_INIT:
            demo_state = DEMO_MODEM_INIT;
            IsST87_Comm_ongoing = true;    
            st87_rst_counter++;
            try_again=N_TRY;
            led_state.tti = cloud_data.tti = TTI_DEFAULT;
            ipread_cnt = 0;
            iprecv_cnt = 0;            
            break;
                
          case DEMO_MODEM_INIT:
            /*reset modem, start it up and get network time*/
            status = m2m_modem_startup();
            if (status == E_OK)
            {
              demo_state = DEMO_CREATE_SOCKET;
              udp_timer = 0;
              /*reinit ref time for periodic update time in Demo CONNECTED state*/
              clock_update_time_ref = millis();
            }
            else
            {
              demo_state = DEMO_MODEM_ERROR;
            }
            break;
                
          case DEMO_CREATE_SOCKET:
            status =  Create_Server_Socket (server_ip,server_port,Protocol,SocketID);
                              
            if (status == E_OK)
            {
              if (sckt_type == UDP_TYPE)
              {
                demo_state = DEMO_WAITING_TO_SEND_DATA;
              }
              else
              {
                demo_state = DEMO_TCP_CONNECT;
                try_again=N_TRY-1;
              }
              udp_timer = 0;
            }
            else
            {
              demo_state = DEMO_MODEM_ERROR;    
            }                
            break;
            
          case DEMO_TCP_CONNECT:  
            status = Create_Tcp_Connection (server_ip,server_port,SocketID);
            if (status == E_OK)
            {
              demo_state = DEMO_SEND_UDP_DATA;
              try_again=N_TRY;
            }
            else
            {
              if (try_again)
              {
                try_again--;            
              }
              else
              {
                demo_state = DEMO_MODEM_ERROR;
                try_again = N_TRY;
              }
            }
            break;
          
          case DEMO_WAITING_TO_SEND_DATA:
            IsST87_Comm_ongoing = false;            
            if (udp_timer==0)
            {
              demo_state = DEMO_SEND_UDP_DATA;
              IsST87_Comm_ongoing = true;
              try_again = N_TRY;
            }
            /* to implement */
            if (ipread_cnt<iprecv_cnt)
            {
              //demo_state = DEMO_READ_UDP_DATA;
            }
            break;
          
          case DEMO_READ_UDP_DATA:             
            
              status = Modem_Socket_Read(SocketID) ;
                
              if (status == E_OK)
              {
                udp_timer = 1000;
                demo_state = DEMO_WAITING_TO_SEND_DATA;
                try_again=N_TRY;
                //udp_timer = cloud_data.tti * 1000;
              }
              else
              {
                if (try_again)
                {
                  try_again--;
                }
                else
                {
                  demo_state = DEMO_MODEM_ERROR;    
                  try_again = N_TRY;
                }
              }               
              break;
             
          case DEMO_SEND_UDP_DATA:
            conn_number++;
            sizeTest = sizeof(cloud_data);
#ifdef PCB_1_0
            cloud_data.Metro_info.energyActive   = 0xFFFFFFFF - Load_metrology_data.energyActive;        /* 4 bytes - int32_t - mwh*/
            cloud_data.Metro_info.energyReactive = 0xFFFFFFFF - Load_metrology_data.energyReactive+1;    /* 4 bytes - int32_t - mwh*/                  
#else
            cloud_data.Metro_info.energyActive   = Load_metrology_data.energyActive;        /* 4 bytes - int32_t - mwh*/
            cloud_data.Metro_info.energyReactive = Load_metrology_data.energyReactive+1;    /* 4 bytes - int32_t - mwh*/                  
#endif            
            cloud_data.Metro_info.rmsvoltage     = Load_metrology_data.rmsvoltage;          /* 4 bytes - int32_t - mVrms */
            cloud_data.Metro_info.rmscurrent     = Load_metrology_data.rmscurrent;          /* 4 bytes - int32_t - mArms */
            cloud_data.Metro_info.status.state   = Load_metrology_data.status.state;        /* 2 bytes - uint16 - 0x0001 = ON 0x0000 = OFF */
            cloud_data.Metro_info.status.dimmer  = Load_metrology_data.status.dimmer;       /* 2 bytes - uint16 */
            cloud_data.XL_info.Ax                = Load_Sensor_data.Ax;                     /* 4 bytes - float - mg*/
            cloud_data.XL_info.Ay                = Load_Sensor_data.Ay;                     /* 4 bytes - float - mg*/  
            cloud_data.XL_info.Az                = Load_Sensor_data.Az;                     /* 4 bytes - float - mg*/
            cloud_data.XL_info.temp              = Load_Sensor_data.temp;                   /* 4 bytes - float  - dC*/ 
            cloud_data.tx_cnt                    = conn_number;                             /* 2 bytes - uint16 */
            cloud_data.tti                       = led_state.tti;                           /* 1 byte - seconds between transmissions */

                                                                                            /* TOTAL = 40 bytes to transmitt */  
            if (sckt_type == UDP_TYPE)
            {
              status = UDP_Server_send(&cloud_data,sizeTest);
            }
            else
            {
              status = TCP_Server_send(&cloud_data,sizeTest);
            }
            time_ref = millis(); 
            //conn_number++;
            
            if (status == E_OK)
            {
              if (sckt_type == UDP_TYPE)
              {
                //demo_state = DEMO_WAIT_NB_SENT; //needed for Brazil
                demo_state = DEMO_WAITING_TO_SEND_DATA; //only for Europe
              }
              else
              {
                demo_state = DEMO_WAITING_TO_SEND_DATA;
              }       
              try_again=N_TRY;
              udp_timer = cloud_data.tti * 1000;
            }
            else
            {
              if (try_again)
              {
                try_again--;
              }
              else
              {
                demo_state = DEMO_MODEM_ERROR;    
                try_again = N_TRY;
              }
            }                    
            break;   
            
          case DEMO_WAIT_NB_SENT:
            if((get_nb_sent_status)||(((millis() - time_ref) > 30000)))
            {
              get_nb_sent_status=0;
              demo_state = DEMO_WAITING_TO_SEND_DATA;
            }
            break;
            
          case DEMO_CHECK_SOCKET_STATUS:
            status = Check_Socket_Status (0);
            demo_state = DEMO_SEND_UDP_DATA;
            //time_ref = millis(); 
            break;
            
          case DEMO_TEST_MODE:
            break;            
                
          case DEMO_CLOSE_UDP_SOCKET:
            /*wait time before close socket */
            if ((millis() - time_ref) > 1000)
            {
              status = Close_UDP_Server_Socket(SocketID);//_Read);
              if (status == E_OK)
              {
                //demo_state = DEMO_CREATE_UDP_SOCKET;
                demo_state = DEMO_CREATE_SOCKET;
              }
              else
              {
                demo_state = DEMO_MODEM_ERROR;    
              }                  
            }
            break;                 
               
          case DEMO_MODEM_ERROR:
            /*wait to init state machine*/
            IsST87_Comm_ongoing = false;
            demo_state = DEMO_WAIT_TIME;
            time_ref = millis();                
            break;

          case DEMO_WAIT_TIME: /* recovering the FSM */
            /*wait time before reinit*/
            if ((millis() - time_ref) > 1000) //CLOUD_WAIT_TIMEOUT)
            {
              demo_state = DEMO_INIT;//DEMO_WAITING_TO_SEND_DATA; //DEMO_SEND_UDP_DATA; //DEMO_INIT;
            }
            break;                              

                
          default:
            demo_state = DEMO_INIT;
            break;
        }
        osMessageQueuePut(ST87CommStatusHandle,&IsST87_Comm_ongoing , 0, 0);
    }

}


/** @} */
