/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    checksum.h
 * @brief   Library of checksum functions.
 *
 * @author  Cristiano Valerio
 * @date    2017-05-17
 *
 * @addtogroup Checksum
 * @{
 */


/******************************************************************************
* Include statements
******************************************************************************/
#include "std_types.h"          /* Standard types */
#include "checksum.h"           /* Module header */


/******************************************************************************
* Public function bodies
******************************************************************************/

/**
 * @brief  Calculate the diili serial protocol checksum for the given buffer
 * @param  p_data buffer to calculate checksum
 * @param  length buffer length
 * @return        checksum
 */
uint8_t checksum_spd (uint8_t const *p_data, uint16_t length)
{
    uint8_t checksum = 0U;
    uint16_t index;

    for (index = 0U; index < length; index++)
    {
        checksum -= *p_data;
        p_data++;
    }

    return checksum;
}


/** @} */
