/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    diili_time.h
 * @brief   Library of time functions.
 *
 * @author  Roberto Andrich
 * @date    2017-11-27
 *
 * @addtogroup Time
 * @{
 */


#ifndef __DIILI_TIME_H
#define __DIILI_TIME_H
#ifdef __cplusplus
 extern "C" {
#endif


/******************************************************************************
* Include statements
******************************************************************************/
#include <time.h>

/******************************************************************************
*  DEFINES
******************************************************************************/

/**
 * @brief   Time type
 */
typedef int32_t app_time_t;
typedef int32_t app_timezone_t;

typedef struct
{
    app_time_t        current_time;                     /**<  modem epoch time*/
    app_timezone_t    current_timezone;
} app_date_time_t;

/******************************************************************************
* Public function prototypes
******************************************************************************/
void           app_system_time_controller(void);
void           update_date_time(void);
app_time_t     get_utc_time(void);
app_time_t     get_local_time(void);
app_timezone_t get_timezone(void);
std_return_t   get_time_str(time_t *rawtime, char *out_str, uint16_t size, app_timezone_t time_zone);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DIILI_TIME_H */

/** @} */
