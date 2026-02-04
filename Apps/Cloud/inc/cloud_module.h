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


#ifndef __CLOUD_MODULE_H
#define __CLOUD_MODULE_H
#ifdef __cplusplus
 extern "C" {
#endif


/******************************************************************************
*  DEFINES
******************************************************************************/
//#include "aws_iot_error.h"
//#include "rte_map.h"


#define NUCLEO_mqttCommandTimeout_ms 	   60000 //JRF //2000 //STM Modified this value
#define NUCLEO_tlsHandshakeTimeout_ms      60000 //JRF //2000 //STM Modified this value
#define NUCLEO_keepAliveIntervalInSec      120 		//10 //STM Modified this value
#define NUCLEO_YIELD_MAX_WAIT_MSEC_READ	   100
#define NUCLEO_YIELD_SLEEP_MSEC		   10000
#define TLS_READ_TIMEOUT_MSEC              100




//// US server port 1337
//#define US_SERVER_PORT         1337
//// EU server port 1338
//#define EU_SERVER_PORT         1338
//#define SERVER_PORT (US_SERVER_PORT)

//#define APN "Default"

/******************************************************************************
* Public function prototypes
******************************************************************************/
//void cloud_controller(void);	
//int create_global_timers( void );
//std_return_t cloud_node_update_ack_cb(rte_node_t node);
//apps_status_t get_cloud_apps_status(void);
void demo_controller(void) ;
void mqtt_demo_controller(void) ;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CLOUD_MODULE_H */

/** @} */
