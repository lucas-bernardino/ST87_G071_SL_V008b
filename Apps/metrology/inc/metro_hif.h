/**
  ******************************************************************************
  * @file    hif_generic_drv.h
  * @author  AMG/IPC Application Team
  * @brief   Header file for the low-level msg handling code
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifndef HIF_GENERIC_DRV_H
#define HIF_GENERIC_DRV_H

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <task_comm.h>

#define MAX_PAYLOAD  64

typedef struct {
    uint8_t cmd_id;
    uint8_t size;
    uint8_t lBuff[MAX_PAYLOAD];
} rx_msg_t;


typedef enum {

  HIF_HI_HWRESET_REQ        = 0x00,
  HIF_HI_HWRESET_CNF        = 0x01,
  HIF_METRO_DATA_REQ        = 0x02,
  HIF_METRO_DATA_CNF        = 0x03,
  HIF_LED_SET_REQ           = 0x04,
  HIF_LED_SET_CNF           = 0x05

} hif_cmdIdHostif_t;


void g3_hif_uart_init(task_comm_info_t *g3_task_comm_info_);

void hif_uart_rx_free_resources(void);

const void *hif_rx_get_msg_payload(const void *buff, uint16_t *len);

bool g3_hif_uart_tx_handler(uint8_t *tx_data);
void g3_hif_uart_rx_handler(uint8_t rx_data);

bool hif_uart_send_message(uint8_t cmd_id, void *buff, uint16_t len);

bool hif_uart_tx_is_ongoing(void);

bool hif_uart_is_free(void);

/*static*/ void hif_uart_rx_handler_init(void);

#endif /* HIF_GENERIC_DRV_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
