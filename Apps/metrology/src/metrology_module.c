#include "main.h"
#include "metro_hif.h"
#include "metro.h"
#include <string.h>

#define POOLLING_TIME   2000
#define CNF_TIMEOUT     2000 //for test come back to 100

typedef enum {
    MET_INIT = 0,
    MET_PROT
} met_msg_ids_t;

#define NO_WAIT         0U
#define WAIT_FOREVER    osWaitForever

#define DEFAULT_MSG_PRIO	0U


extern osMessageQueueId_t LedStatusQueueHandle;
extern osMessageQueueId_t MetroCommStatusHandle;
extern osMessageQueueId_t ST87CommStatusHandle;
extern osMessageQueueId_t LedStatusQueueCloudHandle;
extern rx_msg_t  msg_Rx;

/**
 * @brief   cloud state machine.
 */
typedef enum
{
    DEMO_INIT =0x00,                    /**< Initializing */
    DEMO_LED_STATE,
    DEMO_IDLE,
    DEMO_SET_REQ_LED_STATE,
    DEMO_CNF_LED_STATE,
    DEMO_REQ_READING,
    DEMO_CNF_READING,
    DEMO_WAIT_TIME,
    DEMO_ERROR,
} metro_state_t;


typedef struct
{
  uint8_t Id;      /**< @brief The ID+Index */
}get_data_req_t;

get_data_req_t Get_Data_Req;

extern osMessageQueueId_t MetroQueueHandle;
extern osMessageQueueId_t LedStatusQueueHandle;

led_status_t led_action;

metro_state_t Metro_State = DEMO_INIT;
metro_state_t last_Metro_State = DEMO_INIT;
//uint8_t led_state;
uint8_t led_dimm=10;

uint32_t metro_timer=0;
uint32_t timeout_error_counter=0;
uint8_t retry=0;
uint8_t txBuff[4];
uint32_t setreq_error=0;
uint32_t readmetreq_error=0;

//Load_status_t Load_Data;
lamp_info_t Load_Data;
msg_t msg;
uint8_t IsSTPM32_Comm_ongoing=false;
uint8_t ST87_Comm_ongoing=false;

extern uint8_t update_state;


 // uint32_t val=0;
 // uint32_t ax1,ax2,ax3,ax4;
  
/* Convert a 4 bytes buffer to a float
 * @return float */
uint32_t Buffer_to_float (uint8_t *b)
{
  uint32_t val=0;
  val = ((uint32_t)*(b+3)<<24) | ((uint32_t)*(b+2)<<16) | ((uint32_t)*(b+1)<<8)  |  (uint32_t)*b;  
  return (val);
}

uint32_t myTest=0;

uint16_t hi_metrology_req_data(void *msg_, uint32_t id, uint16_t idx)
{
    get_data_req_t *msg = msg_;

    msg->Id = id;
    //msg->Attribute.Index = idx;

    return sizeof(*msg);
}

static task_comm_info_t g3_task_comm_info;

void Metrology_controller_init (void)
{
  g3_hif_uart_init(&g3_task_comm_info);
}

static task_comm_info_t g3_task_comm_info;

/**
 * @file    cloud_module.h
 * @brief   function related to cloud connectivity
 *
 * @author  JRF
 * @date    2024-08-06
 *
 * @addtogroup CLOUD
 * @{
 */
uint32_t m_size;
void Metrology_controler (void)
{
    uint16_t len;
    
    osMessageQueueGet(LedStatusQueueHandle,&led_action , 0, 0);  //get message from cloud with the new lamp setting 
        
    switch (Metro_State)
    {
      case DEMO_INIT:
        led_action.state = 1;
        Metro_State = DEMO_LED_STATE;
        metro_timer = HAL_GetTick();        
        Load_Data.energyActive   = 3000;
        Load_Data.energyReactive = 250;
        Load_Data.rmsvoltage     = 221555;
        Load_Data.rmscurrent     = 2000;
        Load_Data.status.state   = 1;
        Load_Data.status.dimmer  = led_action.dimmer = 15;
        Load_Data.status.tti     = led_action.tti = TTI_DEFAULT;    
        break;
        
      case DEMO_LED_STATE:
        osMessageQueuePut(LedStatusQueueCloudHandle,&led_action , 0, 0);  //to be read by cloud task
        led_action.update_state  = 0;         
        Load_Data.status.state   = led_action.state;
        Load_Data.status.dimmer  = led_action.dimmer;
        Load_Data.status.tti     = led_action.tti;
        if (Load_Data.status.dimmer==0)
        {
          Load_Data.status.state = 0;
        }
        osMessageQueueReset (LedStatusQueueHandle); 
        Metro_State = DEMO_SET_REQ_LED_STATE;
        break;
        
      case DEMO_SET_REQ_LED_STATE:
        len = 0x02;
        txBuff[0]=Load_Data.status.state;
        txBuff[1]=Load_Data.status.dimmer;        
        hif_uart_rx_handler_init();
        hif_uart_send_message(HIF_LED_SET_REQ, &txBuff, len);
        Metro_State = DEMO_CNF_LED_STATE;
        //metro_timer = HAL_GetTick();        
        break;
        
      case DEMO_CNF_LED_STATE:

        if ((HAL_GetTick() - metro_timer) > CNF_TIMEOUT) 
        {
          setreq_error++;
          timeout_error_counter++;
          Metro_State = DEMO_WAIT_TIME;
        }        
        break;
               
      case DEMO_IDLE:
        
        IsSTPM32_Comm_ongoing = false;

        if ((led_action.update_state)) 
        {
          Metro_State = DEMO_LED_STATE;
        }        
        
        osMessageQueueGet(ST87CommStatusHandle,&ST87_Comm_ongoing , 0, 0);

        if (((HAL_GetTick() - metro_timer) > POOLLING_TIME)&&(ST87_Comm_ongoing==false))
        {
          Metro_State = DEMO_REQ_READING;
          IsSTPM32_Comm_ongoing = true;
        }
        
        break;
        
      case DEMO_REQ_READING:
        len = 0x01;
        txBuff[0]=0x0F;
        txBuff[1]=0x00;   
        HAL_GPIO_WritePin(LED_COMM_MET_GPIO_Port, LED_COMM_MET_Pin, GPIO_PIN_RESET);
        hif_uart_rx_handler_init();
        hif_uart_send_message(HIF_METRO_DATA_REQ, &txBuff, len);
        Metro_State = DEMO_CNF_READING;
        metro_timer = HAL_GetTick();
        break;    
      
      case DEMO_CNF_READING:
        if (msg_Rx.cmd_id == HIF_METRO_DATA_CNF)
        {
          HAL_Delay(100);
          HAL_GPIO_WritePin(LED_COMM_MET_GPIO_Port, LED_COMM_MET_Pin, GPIO_PIN_SET);
          Metro_State = DEMO_IDLE;
          
          myTest = Buffer_to_float (&msg_Rx.lBuff[1]);
          Load_Data.energyActive = myTest;
          
          myTest = Buffer_to_float (&msg_Rx.lBuff[5]);
          Load_Data.energyReactive = myTest;
          
          myTest = Buffer_to_float (&msg_Rx.lBuff[9]);
          Load_Data.rmsvoltage = myTest;
          
          myTest = Buffer_to_float (&msg_Rx.lBuff[13]);
          Load_Data.rmscurrent = myTest;
          
          memset(&msg_Rx, 0, sizeof(msg_Rx));
          
        }
        if ((HAL_GetTick() - metro_timer) > CNF_TIMEOUT) 
        {
          readmetreq_error++;
          timeout_error_counter++;
          Metro_State = DEMO_WAIT_TIME;
        }
        break;
        
      case DEMO_WAIT_TIME:
        IsSTPM32_Comm_ongoing = false;
        if ((HAL_GetTick() - metro_timer) > POOLLING_TIME)
        {
           Metro_State = DEMO_IDLE;
           metro_timer = HAL_GetTick();
        }
        break;    
        
      case DEMO_ERROR:
        break;   
        
      default: Metro_State = DEMO_INIT;
    }
    osMessageQueuePut(MetroCommStatusHandle,&IsSTPM32_Comm_ongoing , 0, 0);
    osMessageQueuePut(MetroQueueHandle,&Load_Data , 0, 0); 
}