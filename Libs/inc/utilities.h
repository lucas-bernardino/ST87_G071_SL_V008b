/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    utilities.h
 * @brief   Perform type safe conversions between common data types.
 *
 * @author  Cristiano Valerio
 * @date    2017-05-17
 *
 * @addtogroup Util
 * @{
 */


#ifndef __UTILITIES_H
#define __UTILITIES_H
#ifdef __cplusplus
 extern "C" {
#endif


/******************************************************************************
* Public function prototypes
******************************************************************************/

/** @brief Combines two uint8_t in a uint16_t */
void util_uint8_to_uint16 (uint8_t in_low, uint8_t in_high, uint16_t* const out);

/** @brief Splits an uint16_t in two uint8_t */
void util_uint16_to_uint8 (uint16_t in, uint8_t* const out_low,
        uint8_t* const out_high);

/** @brief Combines two uint16_t in a uint32_t */
void util_uint16_to_uint32 (uint16_t in_low, uint16_t in_high, uint32_t* const out);

/** @brief Splits an uint32_t in two uint16_t */
void util_uint32_to_uint16 (uint32_t in, uint16_t* const out_low,
        uint16_t* const out_high);

/** @brief Splits an uint64_t in two uint32_t */
void util_uint64_to_uint32 (uint64_t in, uint32_t * const out_low,
        uint32_t * const out_high);

/** @brief Converts an int16_t to uint16_t by saturating negative values at zero */
uint16_t util_int16_to_uint16 (int16_t in);

/** @brief Converts an int16_t to uint16_t by saturating values above INT16_MAX to INT16_MAX */
int16_t util_uint16_to_int16 (uint16_t in);

/** @brief Convert a string to an uint32_t */
std_return_t util_string_to_uint32 (uint8_t *in_str, uint32_t *p_out_num);

/** @brief Convert an uint32_t to a string */
std_return_t util_uint32_to_string (uint32_t in_num, uint8_t *out_str);

/** @brief Return the input variable with the bit given by bit_mask set / cleared */
uint8_t util_uint8_bit_set (uint8_t in, uint8_t bit_mask, bool value);

/** @brief Return whether the bit given by bit_mask is set or cleared */
bool util_uint8_bit_get (uint8_t in, uint8_t bit_mask);

/** @brief get from origin str a substring (out str) between STR1 and STR2 */
std_return_t copy_str_between_strs(char *origin_str, const char *STR1, const char *STR2, char *out_str, uint16_t out_size);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __UTILITIES_H */

/** @} */
