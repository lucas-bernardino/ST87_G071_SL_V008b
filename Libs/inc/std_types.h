/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    std_types.h
 * @brief   Definitions of general types.
 *
 * @author  Cristiano Valerio
 * @date    2017-03-29
 *
 * @addtogroup General
 * @{
 */


#ifndef __STD_TYPES_H
#define __STD_TYPES_H


/******************************************************************************
* Include Section
******************************************************************************/
#include <stdint.h>
#include <stddef.h>
#ifndef __cplusplus
#include <stdbool.h>
#endif


/******************************************************************************
*  DEFINES
******************************************************************************/

/**
 * @brief   This type shall be used to request the version of a software module
 *          using the <module name>_get_version_info() function.
 */
typedef struct {
    uint16_t vendorID;
    uint16_t moduleID;
    uint8_t  instanceID;

    uint8_t sw_major_version;   /**< Vendor numbers */
    uint8_t sw_minor_version;   /**< Vendor numbers */
    uint8_t sw_patch_version;   /**< Vendor numbers */

} std_version_info_t;

/** make compare number... #if version > 10203  ( 1.2.3 ) */
#define STD_GET_VERSION (_major,_minor,_patch) (_major * 10000 + _minor * 100 + _patch)

/** Create std_version_info_t */
/* PC-Lint Exception MISRA rule 19.12 */
/* lint -save -esym(960,19.12) */
#define STD_GET_VERSION_INFO(_vi,_module) \
    if(_vi != NULL) {\
        ((_vi)->vendorID =  _module ## _VENDOR_ID);\
        ((_vi)->moduleID = _module ## _MODULE_ID);\
        ((_vi)->sw_major_version = _module ## _SW_MAJOR_VERSION);\
        ((_vi)->sw_minor_version =  _module ## _SW_MINOR_VERSION);\
        ((_vi)->sw_patch_version =  _module ## _SW_PATCH_VERSION);\
        ((_vi)->ar_major_version =  _module ## _AR_MAJOR_VERSION);\
        ((_vi)->ar_minor_version =  _module ## _AR_MINOR_VERSION);\
        ((_vi)->ar_patch_version =  _module ## _AR_PATCH_VERSION);\
    }
/* lint -restore */

#if !defined(MIN)
#define MIN(_x,_y) (((_x) < (_y)) ? (_x) : (_y))
#endif
#if !defined(MAX)
#define MAX(_x,_y) (((_x) > (_y)) ? (_x) : (_y))
#endif

/**
 * @brief   This type can be used as standard API return type.
 */
typedef enum
{
    E_OK,                       /**< Return OK */
    E_NOT_OK,                   /**< Return not OK */
    E_TIMEOUT,                  /**< Return timeout */
    E_BUSY,                     /**< Return resource busy */
    E_NOT_INITIALIZED           /**< Component not initialized */
} std_return_t;

/**
 * @brief   Physical pin state.
 */
typedef enum
{
    STD_LOW,                    /**< Physical state 0V */
    STD_HIGH                    /**< Physical state 5V or 3.3V */
} std_phy_state_t;

/**
 * @brief   Logic state.
 */
typedef enum
{
    STD_OFF,                    /**< Logic state OFF */
    STD_ON                      /**< Logic state ON */
} std_logic_state_t;

/**
 * @brief   Process identification.
 */
typedef int pid_t;

#define Q0      1L              /* 16385 */
#define Q1      2L              /* 8192.5 */
#define Q2      4L              /* 4096.75 */
#define Q3      8L              /* 2048.875 */
#define Q4      16L             /* 1024.9375 */
#define Q5      32L             /* 512.96875 */
#define Q6      64L             /* 256.984375 */
#define Q7      128L            /* 128.992188 */
#define Q8      256L            /* 64.996094 */
#define Q9      512L            /* 32.998047 */
#define Q10     1024L           /* 16.999023 */
#define Q11     2048L           /* 8.999512 */
#define Q12     4096L           /* 4.999756 */
#define Q13     8192L           /* 2.999878 */
#define Q14     16384L          /* 1.999939 */
#define Q15     32768L          /* 0.999969 */
#define UQ16    65536UL         /* 0.999969 unsigned */

/**
 * @brief  definition of status register for applications (firmware)
 */
typedef union
{
    struct 
    {
        uint32_t location_api_error              : 1;   /*bit0    #1,      */
        uint32_t display_comm_fail               : 1;   /*bit1    #2,      */
        uint32_t loads_comm_fail                 : 1;   /*bit2    #4,      */
        uint32_t hub_comm_fail                   : 1;   /*bit3    #8,      */
        uint32_t blewf_comm_fail                 : 1;   /*bit4    #16,     */
        uint32_t thermostat_comm_fail            : 1;   /*bit5    #32,     */
        uint32_t location_accuracy_error         : 1;   /*bit6    #64,     */
        uint32_t laser_error                     : 1;   /*bit7    #128,    */
        uint32_t ram_fail                        : 1;   /*bit8    #256,    */
        uint32_t reset_power_hub                 : 1;   /*bit9    #512,    */
        uint32_t reset_watchdog_hub              : 1;   /*bit10   #1024,   */
        uint32_t reset_power_cloud               : 1;   /*bit11   #2048,   */
        uint32_t reset_watchdog_cloud            : 1;   /*bit12   #4096,   */
        uint32_t reset_power_blewf               : 1;   /*bit13   #8192,   */
        uint32_t reset_watchdog_blewf            : 1;   /*bit14   #16384,  */
        uint32_t reset_power_thermostat          : 1;   /*bit15   #32768,  */
        uint32_t reset_watchdog_thermostat       : 1;   /*bit16   #65536,  */
        uint32_t reset_power_loads               : 1;   /*bit17   #131072, */
        uint32_t reset_watchdog_loads            : 1;   /*bit18   #262144, */
    } bit;
    uint32_t reg;
} apps_status_t;

/**
 * @brief  Definition of status register for diili system
 */
typedef union
{
    struct 
    {
        uint32_t compressor_fail          : 1;   /*bit0    #1,      */
        uint32_t condenser_fan_fail       : 1;   /*bit1    #2,      */
        uint32_t evaporator_fan_fail      : 1;   /*bit2    #4,      */
        uint32_t defrost_resistance_fail  : 1;   /*bit3    #8,      */
        uint32_t lights_fail              : 1;   /*bit4    #16,     */
        uint32_t door_resistance_fail     : 1;   /*bit5    #32,     */
        uint32_t voltage_fail             : 1;   /*bit6    #64,     */
        uint32_t wrong_location_fail      : 1;   /*bit7    #128,    */
        uint32_t voltage_half_fase        : 1;   /*bit8    #256,    */
        uint32_t internal_temperature_fail: 1;   /*bit9    #512,    */
    } bit;
    uint32_t reg;
} sys_status_t;


#endif

/** @} */
