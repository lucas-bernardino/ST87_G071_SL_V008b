/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    m2m_modem.H
 * @brief   Function to drive the modem M2M
 *
 * @author  Roberto Andrich
 * @date    2017-09-29
 *
 * @addtogroup MODEM
 * @{
 */


#ifndef __M2M_MODEM_H
#define __M2M_MODEM_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "diili_time.h"

/******************************************************************************
*  DEFINES
******************************************************************************/

/******************************************************************************
* Public function prototypes
******************************************************************************/
std_return_t m2m_modem_init(void);
std_return_t m2m_modem_startup(void);
std_return_t m2m_get_time(app_time_t *out_time, app_timezone_t *out_timezone);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __M2M_MODEM_H */

/** @} */
