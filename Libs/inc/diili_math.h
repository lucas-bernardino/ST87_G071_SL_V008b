/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    diili_math.h
 * @brief   Library of math functions.
 *
 * @author  Cristiano Valerio
 * @date    2017-09-27
 *
 * @addtogroup Math
 * @{
 */


#ifndef __DIILI_MATH_H
#define __DIILI_MATH_H
#ifdef __cplusplus
 extern "C" {
#endif


/******************************************************************************
* Include statements
******************************************************************************/
#include "arm_math.h"           /* DSP number types */


/******************************************************************************
* Public function prototypes
******************************************************************************/

/** @brief Calculate square root of a float 32 */
std_return_t diili_sqrt_f32 (float32_t in, float32_t *p_out);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DIILI_MATH_H */

/** @} */
