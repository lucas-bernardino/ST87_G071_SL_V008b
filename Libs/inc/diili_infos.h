/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    diili_infos.h
 * @brief   Functional unit hardware and software informations.
 *
 * @author  Cristiano Valerio
 * @date    2017-05-17
 *
 * @addtogroup Infos
 * @{
 */


#ifndef __DIILI_INFOS_H
#define __DIILI_INFOS_H
#ifdef __cplusplus
 extern "C" {
#endif


/******************************************************************************
*  DEFINES
******************************************************************************/

/* Index of the information to be read */
typedef enum
{
    INFO_MANUFACTURER_ID,       /**< Embraco identification assigned by the customer */
    INFO_HW_VERSION,            /**< Engineering code */
    INFO_SW_PART_NUMBER,        /**< Software part number including minor version */
    INFO_FINAL_PART_NUMBER,     /**< final part number */
    INFO_CUSTOMER_PART_NUMBER,  /**< Final part number assigned by the customer */
    INFO_NUMBER_OF_INFORMATIONS /**< Shall not be used; used only internally */
} info_index_t;


/******************************************************************************
* FUNCTIONS
******************************************************************************/

/** @brief Returns the length of the requested information */
uint8_t infos_length_get (info_index_t info);

/** @brief Returns the character of the requested information */
char infos_info_get (info_index_t info, uint8_t char_index);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DIILI_INFOS_H */

/** @} */
