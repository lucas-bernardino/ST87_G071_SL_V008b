/** \file
 *  \brief Functions of Hardware Dependent Part of Crypto Device Physical
 *         Layer Using I2C For Communication
 *  \author Atmel Crypto Products
 *  \date  June 24, 2013
 * \copyright Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * \atmel_crypto_device_library_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel integrated circuit.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
* \atmel_crypto_device_library_license_stop
 */


//#include <asf.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32_hal.h"
#include "status_codes.h"
#include "crypto_phy.h"		// definitions and declarations for the hardware dependent I2C module
#include "atca_config.h"
#include "stm32_i2c.h"
#include "debug_conf.h"

#define  I2C_CRYPTO_DEFAULT_TIMEOUT  (200000)

I2C_HandleTypeDef hi2c1;

struct i2c_master_packet 
{
  /** Address to slave device  */
  uint16_t address;
  /** Length of data array */
  uint16_t data_length;
  /** Data array containing all data to be transferred */
  uint8_t *data;
  /** Use 10-bit addressing. Set to false if the feature is not supported by the device  */
  bool ten_bit_address;
  /** Use high speed transfer. Set to false if the feature is not supported by the device */
  bool high_speed;
  /** High speed mode master code (0000 1XXX), valid when high_speed is true */
  uint8_t hs_master_code;
};

volatile uint8_t slave_address = (ATCA_I2C_DEFAULT_ADDRESS >> 1);

/** \brief This function sets the address of the I2C peripheral.
			NOTE: Shifts bits right by 1 (addr >> 1) since the driver shifts left when sending address
 * */
void i2c_set_address(uint8_t addr)
{
  //slave_address = (addr >> 1);
  slave_address = (addr);
}

/**\brief This function initializes and enables the I2C peripheral.
 * \param[in] speed freq in Hz 
 * \return none
 * \modified by JRF 7-18/-8
 */
void i2c_enable(uint32_t speed)
{
  uint32_t speed_timing;
  GPIO_InitTypeDef  GPIO_InitStruct;
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
  //RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  
  //speed_timing = speed;
  speed_timing = 0xF010F8FE; //10KHz  //@100KHz0x10909cec; 
  
//  /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
//  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx;
//  RCC_PeriphCLKInitStruct.I2c3ClockSelection =  RCC_I2C1CLKSOURCE_PCLK1; //RCC_I2CxCLKSOURCE_SYSCLK; RCC_I2C1CLKSOURCE_PCLK1
//  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

  /*##-2- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SCL_GPIO_CLK_ENABLE();
  I2Cx_SDA_GPIO_CLK_ENABLE();
  /* Enable I2Cx clock */
  I2Cx_CLK_ENABLE();
  
  /*##-1- Configure the I2C peripheral ######################################*/
  hi2c1.Instance              = I2Cx;
  hi2c1.Init.Timing           = speed_timing;
  hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
  hi2c1.Init.OwnAddress1      = 0x00;
  hi2c1.Init.OwnAddress2      = 0xFF;
  
  if(HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    /* Initialization Error */
    //Error_Handler(); //JRF to be implemented
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1,I2C_ANALOGFILTER_ENABLE);  

  /* Enable DMAx clock */
  I2Cx_DMA_CLK_ENABLE();
  
  /*##-3- Configure peripheral GPIO ##########################################*/  
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);
    
  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);
    
  /*##-4- Configure the DMA Channels #########################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = I2Cx_DMA_INSTANCE_TX;
  hdma_tx.Init.Request             = I2Cx_DMA_REQUEST_TX;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

  HAL_DMA_Init(&hdma_tx);   
  
  /* Associate the initialized DMA handle to the the I2C handle */
  __HAL_LINKDMA(&hi2c1, hdmatx, hdma_tx);
    
  /* Configure the DMA handler for Transmission process */
  hdma_rx.Instance                 = I2Cx_DMA_INSTANCE_RX;
  hdma_rx.Init.Request             = I2Cx_DMA_REQUEST_RX;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

  HAL_DMA_Init(&hdma_rx);
    
  /* Associate the initialized DMA handle to the the I2C handle */
  __HAL_LINKDMA(&hi2c1, hdmarx, hdma_rx);
    
  /*##-5- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (I2Cx_TX) */
  HAL_NVIC_SetPriority(I2Cx_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(I2Cx_DMA_TX_IRQn);
  
  /* NVIC configuration for DMA transfer complete interrupt (I2Cx_RX) */
  HAL_NVIC_SetPriority(I2Cx_DMA_RX_IRQn, 0, 0);   
  HAL_NVIC_EnableIRQ(I2Cx_DMA_RX_IRQn);
  
  /*##-6- Configure the NVIC for I2C ########################################*/   
  /* NVIC for I2Cx */
  HAL_NVIC_SetPriority(I2Cx_ER_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(I2Cx_ER_IRQn);
  HAL_NVIC_SetPriority(I2Cx_EV_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn);
}


/** \brief This function disables the I2C peripheral. */
void i2c_disable(void)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  /*##-1- Reset peripherals ##################################################*/
  I2Cx_FORCE_RESET();
  I2Cx_RELEASE_RESET();
  
  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);  
  
  //HAL_I2C_DeInit(&hi2c1);

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
   
  /*##-3- Disable the DMA Channels ###########################################*/
  /* De-Initialize the DMA Channel associated to transmission process */
  HAL_DMA_DeInit(&hdma_tx); 
  /* De-Initialize the DMA Channel associated to reception process */
  HAL_DMA_DeInit(&hdma_rx);
  
  /*##-4- Disable the NVIC for DMA ###########################################*/
  HAL_NVIC_DisableIRQ(I2Cx_DMA_TX_IRQn);
  HAL_NVIC_DisableIRQ(I2Cx_DMA_RX_IRQn);

  /*##-5- Disable the NVIC for I2C ##########################################*/
  HAL_NVIC_DisableIRQ(I2Cx_ER_IRQn);
  HAL_NVIC_DisableIRQ(I2Cx_EV_IRQn);	
}


/** \brief This function creates a Start condition (SDA low, then SCL low).
 * \return status of the operation
 */
uint8_t i2c_send_start(void)
{
  // Do nothing, return success
  return I2C_SUCCESS;
}


/** \brief This function creates a Stop condition (SCL high, then SDA high).
 * \return status of the operation
 */
uint8_t i2c_send_stop(void)
{
  //__HAL_I2C_SET_FLAG(&hi2c1, I2C_FLAG_STOPF);
  return I2C_SUCCESS;
}


/**\brief This function sends bytes to an I2C device.
 * \param[in] count number of bytes to send
 * \param[in] data pointer to tx buffer
 * \return status of the operation
 * \modified by JRF 7/18/18
 */
uint8_t i2c_send_bytes(uint8_t count, uint8_t *data)
{
  struct i2c_master_packet packet = {
    .address     = slave_address,
    .data_length = count,
    .data        = data,
    .ten_bit_address = false,
    .high_speed      = false,
    .hs_master_code  = 0x0,
  };

    uint8_t statusCode;
    HAL_StatusTypeDef hal_status;    
    
    do
    {
      //hal_status = HAL_I2C_Master_Transmit(&hi2c1,ATCA_I2C_DEFAULT_ADDRESS, (uint8_t*)packet.data,packet.data_length, I2C_CRYPTO_DEFAULT_TIMEOUT);
      hal_status = HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)ATCA_I2C_DEFAULT_ADDRESS, (uint8_t*)packet.data, packet.data_length);      
      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
      {
      }
    }
    while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

    if (hal_status == HAL_OK)
    {
        statusCode = I2C_SUCCESS;
    }
    else
    {
        statusCode = I2C_COMM_FAIL;
    }
	
    return statusCode;
}

/** \brief This function receives bytes from an I2C device using 
 *         DMA transfer and sends a Stop.
 *
 * \param[in] count number of bytes to receive
 * \param[out] data pointer to rx buffer
 * \return status of the operation
 * \changed by JRF 7-18-18
 */
uint8_t i2c_receive_bytes(uint8_t count, uint8_t *data)
{
  struct i2c_master_packet packet = 
  {
    .address     = slave_address,
    .data_length = count,
    .data        = data,
    .ten_bit_address = false,
    .high_speed      = false,
    .hs_master_code  = 0x0,
  };
  uint8_t statusCode;
  HAL_StatusTypeDef hal_status; 

  /*## Put I2C peripheral in reception process ###########################*/  
  do
  {
    hal_status = HAL_I2C_Master_Receive_DMA(&hi2c1,(uint16_t)ATCA_I2C_DEFAULT_ADDRESS, (uint8_t*)packet.data, packet.data_length); 
 
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
    } 
  }
  while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

  if (hal_status == HAL_OK)
  {
    statusCode = I2C_SUCCESS;
  }
  else
  {
    statusCode = I2C_COMM_FAIL;
  }        
  return statusCode;
}

/**\wakeup crypto IC
 * \param[in] none 
 * \param[out] none
 * \return none
 * \changed by JRF 7-18-18
 */
uint8_t i2c_send_wake()
{
  // Send the wake by writing to an address of 0x00
  struct i2c_master_packet packet = 
  {
    .address     = 0x00,
    .data_length = 0,
    .data        = NULL,
    .ten_bit_address = false,
    .high_speed      = false,
    .hs_master_code  = 0x0,
  }; 
  uint8_t statusCode;
  HAL_StatusTypeDef hal_status;
    
  // Send the 00 address as the wake pulse
  // A NACK of the address is a successful wake

  //do
  //{
    hal_status = HAL_I2C_Master_Transmit_DMA(&hi2c1, packet.address,(uint8_t*)packet.data, packet.data_length);      
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
    }
  //}
  //while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);  

  if ((hal_status == HAL_OK)) 
  {
    statusCode = I2C_SUCCESS;
  }
  else
  {
    statusCode = I2C_COMM_FAIL;
  }	  
  return statusCode;
}

///////////////////////////////////////////////////////////////////////////////
// Master implementation - wrapper functions
void phy_i2c_master_enable(i2c_bus_t bus, uint32_t speed)
{
	i2c_enable(I2C_SPEED_100KHZ);
}

void phy_i2c_master_disable(i2c_bus_t bus)
{
	i2c_disable();
}

uint8_t phy_i2c_master_send_start(i2c_bus_t bus)
{
	return i2c_send_start();
}

uint8_t phy_i2c_master_send_stop(i2c_bus_t bus)
{
	return i2c_send_stop();
}

uint8_t i2c_master_send_bytes(i2c_bus_t bus, uint8_t count, uint8_t *data)
{
	return i2c_send_bytes(count, data);
    
}

//JRF
//uint8_t i2c_master_receive_byte(i2c_bus_t bus, uint8_t *data)
//{
//	return i2c_receive_byte(data);
//}

uint8_t i2c_master_receive_bytes(i2c_bus_t bus, uint8_t count, uint8_t *data)
{
      return i2c_receive_bytes(count, data);
}

void i2c_master_set_slave_address(i2c_bus_t bus, uint8_t addr)
{
	i2c_set_address(addr);
}

uint8_t i2c_master_send_wake(i2c_bus_t bus)
{
	return i2c_send_wake();
}

