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


#ifndef __CHECKSUM_H
#define __CHECKSUM_H
#ifdef __cplusplus
 extern "C" {
#endif


/******************************************************************************
* FUNCTIONS
******************************************************************************/

/** @brief Calculate the diili serial protocol checksum for the given buffer */
uint8_t checksum_spd (uint8_t const *p_data, uint16_t length);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CHECKSUM_H */

/** @} */
