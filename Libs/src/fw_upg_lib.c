/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    fw_upg_lib.c
 * @brief   Firmware upgrade helper lib.
 *
 * @author  Cristiano Valerio
 * @date    2017-12-14
 *
 * @addtogroup FW UPG
 * @{
 */


/******************************************************************************
* Include statements
******************************************************************************/
#include "std_types.h"          /* Standard types */
#include "fw_upg_lib.h"         /* Module header */
#include "utilities.h"          /* 8 / 16 / 32 bits type conversion */
#include "spd_defs.h"           /* Node and address sizes in serial protocol */


/******************************************************************************
* Public function bodies
******************************************************************************/

/**
 * @brief   Check if the given version is valid.
 * @author  Cristiano Valerio
 * @date    2017-12-15
 * @param   version      image version number
 * @return               Valid (E_OK) or not (E_NOT_OK).
 */
std_return_t fw_upg_version_plausibility_check (uint32_t version)
{
    std_return_t status;

    if ((version > 0U) && (version < UINT32_MAX))
    {
        status = E_OK;
    }
    else
    {
        status = E_NOT_OK;
    }

    return status;
}

/**
 * @brief   Check if a given upg version is newer than the fw version.
 * @author  Cristiano Valerio
 * @date    2017-12-15
 * @param   fw_version   FW version
 * @param   upg_version  UPG version
 * @return               UPG newer than FW (true) or not (false).
 */
bool fw_upg_is_new_version (uint32_t fw_version, uint32_t upg_version)
{
    bool b_is_new;

    if (fw_upg_version_plausibility_check(upg_version) == E_OK)
    {
        if (fw_upg_version_plausibility_check(fw_version) == E_OK)
        {
            if (upg_version > fw_version)
            {
                b_is_new = true;
            }
            else
            {
                b_is_new = false;
            }
        }
        else
        {
            b_is_new = true;
        }
    }
    else
    {
        b_is_new = false;
    }

    return b_is_new;
}

/**
 * @brief   Convert from serial protocol - diili to fw upg packet.
 * @author  Cristiano Valerio
 * @date    2017-12-15
 * @param   spd_data     pointer to spd data buffer
 * @param   n_bytes      spd data buffer number of bytes
 * @param   node         fw image destination
 * @param   start_addr   packet start address
 * @param   img_packet   image packet
 * @param   img_size     image packet size
 * @return               OK or not.
 */
std_return_t fw_upg_spd_to_fw_upg (uint8_t *spd_data, uint16_t n_bytes, uint8_t *node, uint32_t *start_addr, uint32_t *img_packet, uint16_t *img_size)
{
    uint16_t addr_low;
    uint16_t addr_high;
    uint16_t img_low;
    uint16_t img_high;
    uint16_t img_start_idx;

    *node = spd_data[0];

    util_uint8_to_uint16(spd_data[1], spd_data[2], &addr_low);
    util_uint8_to_uint16(spd_data[3], spd_data[4], &addr_high);
    util_uint16_to_uint32(addr_low, addr_high, start_addr);

    *img_size = n_bytes - (SPD_FW_UPG_DEST_BYTES+SPD_FW_UPG_ADDR_BYTES);
    *img_size = *img_size / 4;

    img_start_idx = SPD_FW_UPG_DEST_BYTES+SPD_FW_UPG_ADDR_BYTES;

    for (uint16_t widx=0; widx < *img_size; widx++)
    {
        uint16_t img_byte_idx;

        img_byte_idx = img_start_idx + widx*4;
        util_uint8_to_uint16(spd_data[img_byte_idx], spd_data[img_byte_idx+1], &img_low);
        util_uint8_to_uint16(spd_data[img_byte_idx+2], spd_data[img_byte_idx+3], &img_high);
        util_uint16_to_uint32(img_low, img_high, &img_packet[widx]);
    }

    return E_OK;
}

/**
 * @brief   Convert from fw upg to serial protocol - diili packet.
 * @author  Cristiano Valerio
 * @date    2017-12-15
 * @param   node         fw image destination
 * @param   start_addr   packet start address
 * @param   img_packet   image packet
 * @param   img_size     image packet size
 * @param   spd_data     pointer to spd data buffer
 * @param   n_bytes      spd data buffer number of bytes
 * @return               OK or not.
 */
std_return_t fw_upg_fw_upg_to_spd (uint8_t node, uint32_t start_addr, uint32_t *img_packet, uint16_t img_size, uint8_t *spd_data, uint16_t *n_bytes)
{
    uint16_t addr_low;
    uint16_t addr_high;
    uint16_t img_start_idx;

    spd_data[0] = node;

    util_uint32_to_uint16(start_addr, &addr_low, &addr_high);
    util_uint16_to_uint8(addr_low, &spd_data[1], &spd_data[2]);
    util_uint16_to_uint8(addr_high, &spd_data[3], &spd_data[4]);

    img_start_idx = SPD_FW_UPG_DEST_BYTES+SPD_FW_UPG_ADDR_BYTES;
    for (uint16_t widx=0; widx < img_size; widx++)
    {
        uint16_t img_low;
        uint16_t img_high;
        uint16_t byte_idx = img_start_idx + widx*4;

        util_uint32_to_uint16(img_packet[widx], &img_low, &img_high);
        util_uint16_to_uint8(img_low, &spd_data[byte_idx], &spd_data[byte_idx+1]);
        util_uint16_to_uint8(img_high, &spd_data[byte_idx+2], &spd_data[byte_idx+3]);
    }

    *n_bytes = SPD_FW_UPG_DEST_BYTES + SPD_FW_UPG_ADDR_BYTES + (img_size*4);

    return E_OK;
}


/** @} */
