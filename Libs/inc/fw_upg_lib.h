/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    fw_upg_lib.h
 * @brief   Firmware upgrade helper lib.
 *
 * @author  Cristiano Valerio
 * @date    2017-12-14
 *
 * @addtogroup FW UPG
 * @{
 */


#ifndef __FW_UPG_LIB_H
#define __FW_UPG_LIB_H
#ifdef __cplusplus
 extern "C" {
#endif


/******************************************************************************
* Public function prototypes
******************************************************************************/

/** @brief Check if the given version is valid */
std_return_t fw_upg_version_plausibility_check (uint32_t version);

/** @brief Check if a given upg version is newer than the fw version */
bool fw_upg_is_new_version (uint32_t fw_version, uint32_t upg_version);

/** @brief Convert from serial protocol - diili to fw upg packet */
std_return_t fw_upg_spd_to_fw_upg (uint8_t *spd_data, uint16_t n_bytes, uint8_t *node, uint32_t *start_addr, uint32_t *img_packet, uint16_t *img_size);

/** @brief Convert from fw upg to serial protocol - diili packet */
std_return_t fw_upg_fw_upg_to_spd (uint8_t node, uint32_t start_addr, uint32_t *img_packet, uint16_t img_size, uint8_t *spd_data, uint16_t *n_bytes);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __FW_UPG_LIB_H */

/** @} */
