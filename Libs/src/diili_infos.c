/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    diili_infos.c
 * @brief   Functional unit hardware and software informations.
 *
 * @author  Cristiano Valerio
 * @date    2017-05-17
 *
 * @addtogroup Infos
 * @{
 */


/******************************************************************************
* Include section
******************************************************************************/
#include "std_types.h"                  /* Standard types */
#include "diili_infos.h"                /* Module header */
#include "diili_infos_conf.h"           /* Module configuration */
#include "eST_flash_upgrade_memory.h"   /* FW code start */
#include "eST_flash_upgrade_lib.h"      /* Number of status long words */


/******************************************************************************
* Types section
******************************************************************************/

/**
 * @brief   Information type.
 */
typedef struct {
    const char *information;    /**< String */
    const uint8_t length;       /**< Length */
} info_t;


/******************************************************************************
* Local Variables section
******************************************************************************/

/**
 * @brief   HW and SW informations.
 */
static const info_t infos[INFO_NUMBER_OF_INFORMATIONS] = {
    {&manufacturer_id, 1},
    {hw_version, (sizeof(hw_version)/sizeof(char) - 1)},
    {sw_part_number, (sizeof(sw_part_number)/sizeof(char) - 1)},
    {final_part_number, (sizeof(final_part_number)/sizeof(char) - 1)},
    {customer_part_number, (sizeof(customer_part_number)/sizeof(char) - 1)}};

/**
 * @brief   Informations used by the bootloader.
 */
#pragma location=".z_end_marker"
__root const uint32_t marker = 0xEAEAEBEB;
#pragma location=".version"
__root const uint32_t code_version = CODE_VERSION;
#pragma section=".z_end_marker"
#pragma section=".intvec"
#pragma location=".size"
__root const uint32_t code_size = ((uint32_t) __section_end(".z_end_marker")) - FW_CODE_START;
#pragma location=".empty"
__root const uint32_t empty = 0xEEEEEE00;
#pragma location=".flash_status"
__root const uint64_t flash_status[N_STATUS_LW] = {STATUS_LW_PROG, STATUS_LW_PROG, STATUS_LW_ERASED, STATUS_LW_ERASED};  //write_finished

#define BUFFER_SIZE    4
#pragma location=".buffer"
__root static const uint32_t aDataBuffer[BUFFER_SIZE] =
{
  0x00001021, 0x20423063, 0x408450a5, 0x60c670e7
};


/******************************************************************************
* Public Function Implementation Section
******************************************************************************/

/**
 * @brief   Return the length of the string excluding the string termination
 *          character.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   info        Information index
 * @return              Information length
 */
uint8_t infos_length_get (info_index_t info)
{
    uint8_t info_length;

    if (info < INFO_NUMBER_OF_INFORMATIONS)
    {
        info_length = infos[info].length;
    }
    else
    {
        info_length = 0;
    }

    return info_length;
}

/**
 * @brief   Return the character corresponding to char_index of the string.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   info        Information index
 * @param   char_index  Index of the requested character
 * @return              Requested character
 */
char infos_info_get (info_index_t info, uint8_t char_index)
{
    char character = '\0';

    if (info < INFO_NUMBER_OF_INFORMATIONS)
    {
        if (char_index < infos[info].length)
        {
            character = infos[info].information[char_index];
        }
    }

    return character;
}


/** @} */
