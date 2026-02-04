/******************************************************************************
* The information contained herein is confidential property of Embraco. The
* user, copying, transfer or disclosure of such information is prohibited
* except by express written agreement with Embraco.
******************************************************************************/

/**
 * @file    utilities.c
 * @brief   Perform type safe conversions between common data types.
 *
 * @author  Cristiano Valerio
 * @date    2017-05-17
 *
 * @addtogroup Util
 * @{
 */


/******************************************************************************
* Include statements
******************************************************************************/
#include <string.h>
#include "std_types.h"          /* fixed-width types, max values, std return */
#include "utilities.h"          /* module header */


/******************************************************************************
* Data types, constants and macro definitions
******************************************************************************/

static const uint8_t lsb_mask = 0xFFU;          /* 1's mask for the least */
                                                /* significant byte */

static const uint16_t lsb_mask_16b = 0xFFFFU;   /* 1's mask for the least 16b */

static const uint32_t lsw_mask = 0xFFFFFFFFU;   /* 1's mask for the least */
                                                /* significant word */

#define IS_CAP_LETTER(c)    (((c) >= 'A') && ((c) <= 'F'))
#define IS_LC_LETTER(c)     (((c) >= 'a') && ((c) <= 'f'))
#define IS_09(c)            (((c) >= '0') && ((c) <= '9'))
#define ISVALIDHEX(c)       (IS_CAP_LETTER(c) || IS_LC_LETTER(c) || IS_09(c))
#define ISVALIDDEC(c)       IS_09(c)
#define CONVERTDEC(c)       (c - '0')
#define CONVERTHEX_ALPHA(c) (IS_CAP_LETTER(c) ? ((c) - 'A'+10) : ((c) - 'a'+10))
#define CONVERTHEX(c)       (IS_09(c) ? ((c) - '0') : CONVERTHEX_ALPHA(c))


/******************************************************************************
* Public function bodies
******************************************************************************/

/**
 * @brief   Combines two uint8_t in one uint16_t.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   in_low      low byte
 * @param   in_high     high byte
 * @param   out         output, combined two bytes
 */
void util_uint8_to_uint16 (uint8_t in_low, uint8_t in_high, uint16_t* const out)
{
    (*out) = (((uint16_t) in_high) << 8) + in_low;
}

/**
 * @brief   Splits the uint16_t in two uint8_t.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   in          word to be split
 * @param   out_low     output, low byte
 * @param   out_high    output, high byte
 */
void util_uint16_to_uint8 (uint16_t in, uint8_t* const out_low,
        uint8_t* const out_high)
{
    (*out_low) = (uint8_t)(in & lsb_mask);
    (*out_high) = (uint8_t)(in >> 8);
}

/**
 * @brief   Combines two uint16_t in one uint32_t.
 * @author  Cristiano Valerio
 * @date    2017-08-17
 * @param   in_low      low 16b
 * @param   in_high     high 16b
 * @param   out         output, combined two 16b
 */
void util_uint16_to_uint32 (uint16_t in_low, uint16_t in_high, uint32_t* const out)
{
    (*out) = (((uint32_t) in_high) << 16) + in_low;
}

/**
 * @brief   Splits the uint32_t in two uint16_t.
 * @author  Cristiano Valerio
 * @date    2017-08-17
 * @param   in          32b to be split
 * @param   out_low     output, low 16b
 * @param   out_high    output, high 16b
 */
void util_uint32_to_uint16 (uint32_t in, uint16_t* const out_low,
        uint16_t* const out_high)
{
    (*out_low) = (uint16_t)(in & lsb_mask_16b);
    (*out_high) = (uint16_t)(in >> 16);
}

/**
 * @brief   Splits the uint64_t in two uint32_t.
 * @author  Cristiano Valerio
 * @date    2017-07-21
 * @param   in          64b to be split
 * @param   out_low     output, low 32b
 * @param   out_high    output, high 32b
 */
void util_uint64_to_uint32 (uint64_t in, uint32_t * const out_low,
        uint32_t * const out_high)
{
    (*out_low) = (uint32_t)(in & lsw_mask);
    (*out_high) = (uint32_t)(in >> 32);
}

/**
 * @brief   Convert from int16_t to uint16_t.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   in          data to be converted
 * @return              converted data
 */
uint16_t util_int16_to_uint16 (int16_t in)
{
    uint16_t out;

    if (0 < in)
    {
        out = in;
    }
    else
    {
        out = 0;
    }

    return (out);
}

/**
 * @brief   Convert from uint16_t to int16_t.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   in          data to be converted
 * @return              converted data
 */
int16_t util_uint16_to_int16 (uint16_t in)
{
    int16_t out;

    if (INT16_MAX >= in)
    {
        out = in;
    }
    else
    {
        out = INT16_MAX;
    }

    return (out);
}

/**
 * @brief   Convert from string to uint32_t.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   in_str      input string
 * @param   p_out_num   output number
 * @return              Conversion OK or not
 */
std_return_t util_string_to_uint32 (uint8_t *in_str, uint32_t *p_out_num)
{
    uint32_t i = 0;
    uint32_t val = 0;
    std_return_t res = E_OK;

    if ((in_str[0] == '0') && ((in_str[1] == 'x') || (in_str[1] == 'X')))
    {
        i = 2;
        while ((i < 11) && (in_str[i] != '\0'))
        {
            if (ISVALIDHEX(in_str[i]))
            {
                val = (val << 4) + CONVERTHEX(in_str[i]);
            }
            else
            {
                /* Invalid input */
                res = E_NOT_OK;
                break;
            }
            i++;
        }

        /* valid result */
        if (in_str[i] == '\0')
        {
            *p_out_num = val;
            res = E_OK;
        }
    }
    else /* max 10-digit decimal input */
    {
        while ((i < 11) && (res != 1))
        {
            if (in_str[i] == '\0')
            {
                *p_out_num = val;
                /* return OK */
                res = E_OK;
            }
            else if (((in_str[i] == 'k') || (in_str[i] == 'K')) && (i > 0))
            {
                val = val << 10;
                *p_out_num = val;
                res = E_OK;
            }
            else if (((in_str[i] == 'm') || (in_str[i] == 'M')) && (i > 0))
            {
                val = val << 20;
                *p_out_num = val;
                res = E_OK;
            }
            else if (ISVALIDDEC(in_str[i]))
            {
                val = val * 10 + CONVERTDEC(in_str[i]);
            }
            else
            {
                /* Invalid input */
                res = E_NOT_OK;
                break;
            }
            i++;
        }
    }

    return res;
}

/**
 * @brief   Convert from uint32_t to string.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   in_num      input number
 * @param   out_num     output string
 * @return              Conversion OK or not
 */
std_return_t util_uint32_to_string (uint32_t in_num, uint8_t *out_str)
{
    uint32_t i;
    uint32_t divider = 1000000000;
    uint32_t pos = 0;
    uint32_t status = 0;

    for (i = 0; i < 10; i++)
    {
        out_str[pos++] = (in_num / divider) + 48;

        in_num = in_num % divider;
        divider /= 10;
        if ((out_str[pos-1] == '0') & (status == 0))
        {
            pos = 0;
        }
        else
        {
            status++;
        }
    }

    return E_OK;
}

/**
 * @brief   Set / clear the bit given by bit_mask in input variable in and
 *          returns the corresponding result.
 * @note    It does not modify the input variable in, the result with the modified
 *          bit is the return of the function.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   in          data to set the bit
 * @param   bit_mask    bit to be set / cleared
 * @param   value       if it is to set (true) or clear (false)
 * @return              same as in with bits set / cleared
 */
uint8_t util_uint8_bit_set (uint8_t in, uint8_t bit_mask, bool value)
{
    uint8_t out;

    if (value)
    {
        out = in | bit_mask;
    }
    else
    {
        out = in & (~bit_mask);
    }

    return (out);
}

/**
 * @brief   Check if the bits given by bit_mask are set in the input variable.
 * @author  Cristiano Valerio
 * @date    2017-05-17
 * @param   in          data to check the bits
 * @param   bit_mask    bits to be checked
 * @return              if the bits are set (true) or not (false)
 */
bool util_uint8_bit_get (uint8_t in, uint8_t bit_mask)
{
    bool b_is_bit_set;

    if (in & bit_mask)
    {
        b_is_bit_set = true;
    }
    else
    {
        b_is_bit_set = false;
    }

    return (b_is_bit_set);
}

/**
 * @brief   get the content between strings str1 and str2. str1 must be different from str2
 * @author  Roberto Andrich
 * @date    2017-11-17
 * @param   *origin_str     pointer to the string where content will be searched
 * @param   *str1           str after which content will be copied
 * @param   *str2           str before which content will be copied
 * @param   out_str         output string
 * @param   out_size        size of output array that will hold the string
 * @return  E_OK if string was copied properly            
 * @remarks Example: if   origin_str =  "abcdefg", str1="bc", str2="fg"
 *                   then out_str    =  "de"
 *          if the content do not fit in out_str then E_NOT_OK will be output
 */
std_return_t copy_str_between_strs(char *origin_str, const char *STR1, const char *STR2, char *out_str, uint16_t out_size)
{
    char *start, *end;
    uint16_t size_match_str;
    std_return_t status = E_NOT_OK;
    
    /*check if STR1 exists inside origin str*/
    if (start = strstr(origin_str, STR1))
    {
        /*point to position just after STR1*/
        start += strlen( STR1 );
        
        /*check if STR2 exists inside origin str*/
        if (end = strstr(origin_str, STR2))
        {
            /*size of the found substring*/
            size_match_str = end - start + 1;
            
            /*if smaller the out str then copy */
            if (size_match_str < out_size)
            {
                /*clear out string and after that update it*/
                memset(out_str, 0x00 , out_size);
                memcpy(out_str, start, end - start);
                out_str[end - start] = '\0'; /*include end of str char*/
                status = E_OK;
            }
            
        }
        
        
    }
    
    return status;
    
}


/** @} */
