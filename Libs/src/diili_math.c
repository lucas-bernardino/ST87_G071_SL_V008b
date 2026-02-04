/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    diili_math.c
 * @brief   Library of math functions.
 *
 * @author  Cristiano Valerio
 * @date    2017-09-27
 *
 * @addtogroup Math
 * @{
 */


/******************************************************************************
* Include statements
******************************************************************************/
#include "std_types.h"          /* Standard types */
#include "diili_math.h"         /* Module header */


/******************************************************************************
* Public function bodies
******************************************************************************/

/**
 * @brief   Computes the square root of a floating-point number.
 * @author  Cristiano Valerio
 * @date    2017-09-27
 * @param   in           input value
 * @param   p_out        square root of input value
 * @return               OK or not.
 */
std_return_t diili_sqrt_f32 (float32_t in, float32_t *p_out)
{
    std_return_t std_status;
    arm_status astatus;

    astatus = arm_sqrt_f32(in, p_out);

    if (astatus == ARM_MATH_SUCCESS)
    {
        std_status = E_OK;
    }
    else
    {
        std_status = E_NOT_OK;
    }

    return std_status;
}


/** @} */
