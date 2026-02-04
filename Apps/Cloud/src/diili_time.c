/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    diili_time.c
 * @brief   Library of time functions.
 *
 * @author  Roberto Andrich
 * @date    2017-11-27
 *
 * @addtogroup Time
 * @{
 */


/******************************************************************************
* Include statements
******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "stm32_hal.h"
#include "std_types.h"          /* Standard types */
#include "timer.h"
#include "diili_time.h"
#include "m2m_modem.h"

/******************************************************************************
* Data types, constants and macro definitions
******************************************************************************/
#define DATE_TIME_BASE   1000  /*ms*/
#define TIMEZONE_TO_SEC  3600  /*convert from timezone hour to second*/
#define TIMEZONE_STR_SIZE  6

/******************************************************************************
* Static data declarations
******************************************************************************/
app_date_time_t volatile app_time;          /*must be updated by network*/

app_date_time_t network_time;               /*time read from network */
app_date_time_t network_time_old;           /*time read from network - previous value*/

/******************************************************************************
* Private function prototypes
******************************************************************************/

/******************************************************************************
* Public function bodies
******************************************************************************/

/**
 * @brief   count time with DATE_TIME_BASE precision
 *          this routine is called inside SysTick_Handler()
 * @author  Roberto Andrich
 * @date    2017-11-30
 * @param   none          
 * @param   none          
 * @return  none
 */
void app_system_time_controller(void)
{
    static uint32_t time_ref=0;
    /*increment time variable every 1sec. This variable is updated through AT command*/
    if ((millis() - time_ref) > DATE_TIME_BASE)
    {
        time_ref = millis();
        app_time.current_time++;
    }
    
}

/**
 * @brief   get epoch time and timezone from modem
 * @author  Roberto Andrich
 * @date    2017-11-30
 * @param   none          
 * @param   none          
 * @return  none
 */
void update_date_time(void)
{
    std_return_t status;
    
    /*get time from network (today only by m2m modem)*/
    status = m2m_get_time(&network_time.current_time, &network_time.current_timezone);
    
    if (status == E_OK)
    {
        /*ensure network_time really was updated*/
        if (network_time.current_time != network_time_old.current_time)
        {
            /*update previous network time value*/
            network_time_old.current_time = network_time.current_time;
            
            /*write atomically time variable because if modified in isr*/
            __disable_irq();                
                app_time.current_time = network_time.current_time;
                app_time.current_timezone = network_time.current_timezone;  
            __enable_irq();                         
        }
        
        
    }

    
}

/**
 * @brief   count time with DATE_TIME_BASE precision
 * @author  Roberto Andrich
 * @date    2017-11-30
 * @param   none          
 * @param   none          
 * @return  utc time in epoch seconds (since 1900)
 */
app_time_t get_utc_time(void) 
{
    app_time_t out_time;
    
    /*read atomically time variable*/
    __disable_irq();
    out_time = app_time.current_time;
    __enable_irq();
    
    return out_time;
    
}

/**
 * @brief   count time with DATE_TIME_BASE precision
 * @author  Roberto Andrich
 * @date    2017-11-30
 * @param   none          
 * @param   none          
 * @return  local time in epoch seconds (since 1900)
 */
app_time_t get_local_time(void)
{
    app_time_t out_time;
    
    /*read atomically time variable*/
    __disable_irq();
    out_time = app_time.current_time;
    __enable_irq();
    
    /*calculate local time. In two lines to avoid volatine operation warning*/
    out_time = app_time.current_time;
    out_time += (app_time_t)(app_time.current_timezone * TIMEZONE_TO_SEC );
    
    
    return out_time;
    
}

/**
 * @brief   count time with DATE_TIME_BASE precision
 * @author  Roberto Andrich
 * @date    2017-11-30
 * @param   none          
 * @param   none          
 * @return  timezone in hour unit
 */
app_timezone_t get_timezone(void) 
{

    return app_time.current_timezone;
    
}

/**
 * @brief   format date to %Y-%m-%d %H:%M:%S pattern
 * @author  Roberto Andrich
 * @date    2017-12-12
 * @param   rawtime: time in epoch format          
 * @param   out_str: pointer to out str that will receive formatted date     
 * @param   size:    size of out_str to avoid overflow 
 * @param   time_zone: time zone value
 * @return  std_return_t
 */
std_return_t get_time_str(time_t *rawtime, char *out_str, uint16_t size, app_timezone_t time_zone)
{
    std_return_t status = E_OK;
    struct tm *info;
    
    /*convert rawtime to struct tm*/
    info = gmtime(rawtime);
    
    /*break rawtime data into struct tm*/
    memset(out_str, 0x0, size);             /*clear str to avoid trash*/
    strftime(out_str, size, "\"%Y-%m-%d %H:%M:%S\"", info);

    return status;
    
}
/** @} */
