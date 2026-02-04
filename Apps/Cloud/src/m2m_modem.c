/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    m2m_modem.c
 * @brief   Function to drive the M2M modem
 *
 * @author  Roberto Andrich
 * @date    2017-09-29
 *
 * @addtogroup m2m_modem
 * @{
 */


/******************************************************************************
* Include statements
******************************************************************************/
#include "main.h"
#include <time.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include "std_types.h"          /* fixed-width types, max values, std return */
#include "utilities.h"          /* module header */
#include "m2m_modem.h"
#include "modem_main.h"         /* main modem library */
#include "modem_interface.h"



/******************************************************************************
* Data types, constants and macro definitions
******************************************************************************/
/*Modem clock time definitions:
  Indexes to extract: " "yy/mm/dd,hh:mm:ss+zz""
                       12345678901234567890123*/

#define MODEM_YEAR_IDX     2
#define MODEM_MONTH_IDX    (MODEM_YEAR_IDX + 3)
#define MODEM_DAY_IDX      (MODEM_MONTH_IDX + 3)
#define MODEM_HOUR_IDX     (MODEM_DAY_IDX +3)
#define MODEM_MINUTE_IDX   (MODEM_HOUR_IDX + 3)
#define MODEM_SECOND_IDX   (MODEM_MINUTE_IDX + 3)
#define MODEM_TIMEZONE_IDX (MODEM_SECOND_IDX + 2)

#define MODEM_DATE_DIGITS     2
#define MODEM_TIMEZONE_DIGITS 3
#define MODEM_TIMEZONE_DIV    4  /*timezone comes in quarter from hour*/
#define MODEM_STR_DIGIT_SIZE  4
#define MODEM_YEAR_OFFSET     2000

/*time.h library definitions*/
#define TIME_LIB_YEAR_BEGIN   1900
#define TIME_LIB_MONTH_OFFSET -1


/******************************************************************************
* Static data declarations
******************************************************************************/

/******************************************************************************
* Private function prototypes
******************************************************************************/


/******************************************************************************
* Public function bodies
******************************************************************************/

/**
 * @brief   initialize modem
 * @author  Roberto Andrich
 * @date    2017-mm-dd
 * @param   
 * @return  
 */
std_return_t m2m_modem_init(void)
{
    std_return_t output = E_OK;
    
    if (modem_peripherals_init() !=MODEM_MODULE_SUCCESS)
    {
        output = E_NOT_OK;
    }
    
    return output;
}

/**
 * @brief   reset modem, start it up and get network time
 * @author  JRF
 * @date    2024-10-25
 * @param   
 * @return  
 */
std_return_t m2m_modem_startup(void)
{
    std_return_t output = E_OK;
    
    /* ST87 HW reset */
    HAL_GPIO_WritePin(ST87_nRST_GPIO_Port, ST87_nRST_Pin, GPIO_PIN_SET); 
    HAL_Delay(10); 
    HAL_GPIO_WritePin(ST87_nRST_GPIO_Port, ST87_nRST_Pin, GPIO_PIN_RESET); 
    HAL_Delay(500); 
    HAL_GPIO_WritePin(ST87_nRST_GPIO_Port, ST87_nRST_Pin, GPIO_PIN_SET); 
    HAL_Delay(1000);
  
    if (modem_main() !=MODEM_MODULE_SUCCESS)
    {
        output = E_NOT_OK;
    }
    
    return output;
}

/**
 * @brief   get modem clock time 
 * @author  Roberto Andrich
 * @date    2017-11-28
 * @param   out_time: output in epoch integer format
 * @return  E_OK if sucessfull
 */
std_return_t m2m_get_time(app_time_t *out_time, app_timezone_t *out_timezone)
{
    char *timestamp;
    struct tm time_obj;
    char number_str[MODEM_STR_DIGIT_SIZE]; /*time digit will be temporary stored here*/
    int32_t number = 0;
    int32_t timezone = 0;
    time_t  epoch_time = 0;
    
    /*init day light var. Not used here*/
    time_obj.tm_isdst = -1;
    
    /*extracting time object from timestamp*/
    timestamp = modem_get_time();
    if (timestamp != NULL)
    {
        /*get year*/
        memset(number_str,0,MODEM_STR_DIGIT_SIZE);                         /*clear destination str*/
        strncpy(number_str, timestamp+MODEM_YEAR_IDX, MODEM_DATE_DIGITS);  /*copy digits*/ 
        if (1 != sscanf(number_str, "%"SCNd32, &number))
        {           
            return E_NOT_OK;
        }
        time_obj.tm_year = MODEM_YEAR_OFFSET + number - TIME_LIB_YEAR_BEGIN;
        
        /*get month*/
        memset(number_str,0,MODEM_STR_DIGIT_SIZE);                         /*clear destination str*/
        strncpy(number_str, timestamp+MODEM_MONTH_IDX, MODEM_DATE_DIGITS);  /*copy digits*/ 
        if (1 != sscanf(number_str, "%"SCNd32, &number))
        {           
            return E_NOT_OK;
        }
        time_obj.tm_mon = number + TIME_LIB_MONTH_OFFSET;     
        
        /*get day*/
        memset(number_str,0,MODEM_STR_DIGIT_SIZE);                         /*clear destination str*/
        strncpy(number_str, timestamp+MODEM_DAY_IDX, MODEM_DATE_DIGITS);  /*copy digits*/ 
        if (1 != sscanf(number_str, "%"SCNd32, &number))
        {           
            return E_NOT_OK;
        }
        time_obj.tm_mday = number;

        /*get hour*/
        memset(number_str,0,MODEM_STR_DIGIT_SIZE);                         /*clear destination str*/
        strncpy(number_str, timestamp+MODEM_HOUR_IDX, MODEM_DATE_DIGITS);  /*copy digits*/ 
        if (1 != sscanf(number_str, "%"SCNd32, &number))
        {           
            return E_NOT_OK;
        }
        time_obj.tm_hour = number;        
        
        /*get minute*/
        memset(number_str,0,MODEM_STR_DIGIT_SIZE);                         /*clear destination str*/
        strncpy(number_str, timestamp+MODEM_MINUTE_IDX, MODEM_DATE_DIGITS);  /*copy digits*/ 
        if (1 != sscanf(number_str, "%"SCNd32, &number))
        {           
            return E_NOT_OK;
        }
        time_obj.tm_min = number;
        
        /*get second*/
        memset(number_str,0,MODEM_STR_DIGIT_SIZE);                         /*clear destination str*/
        strncpy(number_str, timestamp+MODEM_SECOND_IDX, MODEM_DATE_DIGITS);  /*copy digits*/ 
        if (1 != sscanf(number_str, "%"SCNd32, &number))
        {           
            return E_NOT_OK;
        }
        time_obj.tm_sec = number;     
        
        /*get timezone*/
        memset(number_str,0,MODEM_STR_DIGIT_SIZE);                         /*clear destination str*/
        strncpy(number_str, timestamp+MODEM_TIMEZONE_IDX, MODEM_TIMEZONE_DIGITS);  /*copy digits*/ 
        if (1 != sscanf(number_str, "%"SCNd32, &number))
        {           
            return E_NOT_OK;
        }
        timezone = number / MODEM_TIMEZONE_DIV;  

        /*final result*/
        epoch_time =  mktime(&time_obj);
        if (-1 == epoch_time)
        {
            return E_NOT_OK;
        }
        else
        {
            *out_time       = (app_time_t)epoch_time;
            *out_timezone   = (app_timezone_t)timezone;
        }
        
        
    }
    else
    {
        return E_NOT_OK;
    }
    
    return E_OK;
}

/******************************************************************************
* Private function bodies
******************************************************************************/
 


/** @} */
