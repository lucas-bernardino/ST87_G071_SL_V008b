/**
 ******************************************************************************
 * @file    stm32_st87m01.c
 * @author  JRF
 * @version V1.0.0
 * @date    2024-08-05
 * @brief   HAL related functionality of ST87M01
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "modem_module.h"
#include "modem_globals.h"
#include "stm32_st87m01.h"

//#if defined (STM32L4)
//#include "stm32l4xx_hal.h"
//#include "stm32l4xx.h"
//#include "stm32l4xx_it.h"
////#else
//#endif

#if defined (STM32G0)
#include "stm32g0xx.h"
#include "stm32g0xx_it.h"
#endif
//#warning "Define STM32 familiy!!!!"
//#endif


  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
    since APB1 prescaler is set to 4 (0x100).
       TIM3CLK = PCLK1*2
       PCLK1   = HCLK/2
    => TIM3CLK = PCLK1*2 = (HCLK/2)*2 = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f1xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */
void Modem_Timer_Config(void)
{
  
  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;
  
  /* Set TIMx instance */
  modemTimHandle.Instance = TIM_MODEM;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
#if defined (STM32L4)
  modemTimHandle.Init.Period            = 100 - 1;
#endif
#if defined (STM32G0)  
modemTimHandle.Init.Period            = 100 - 1;
#endif
//    #error "STM32L4 not defined!!"
//#endif 
  modemTimHandle.Init.Prescaler         = uwPrescalerValue;
  modemTimHandle.Init.ClockDivision     = 0;
  modemTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
#ifdef USE_STM32F1xx_NUCLEO
  modemTimHandle.Init.RepetitionCounter = 0;
#endif 

  if (HAL_TIM_Base_Init(&modemTimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();   //JRF TO BE IMPLEMENTED
  }
}

/**
  * @brief Push_Timer_Config
  *        This function configures the Push Timer
  * @param None
  * @retval None
  */
void Modem_Push_Timer_Config(void)
{
  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;
  
  /* Set TIMx instance */
  modemPushTimHandle.Instance = TIM_MODEMp;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  modemPushTimHandle.Init.Period            = 10 - 1;//10000
  modemPushTimHandle.Init.Prescaler         = uwPrescalerValue;
  modemPushTimHandle.Init.ClockDivision     = 0;
  modemPushTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
#ifdef USE_STM32F1xx_NUCLEO
  modemPushTimHandle.Init.RepetitionCounter = 0;
#endif 

  if (HAL_TIM_Base_Init(&modemPushTimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); //JRF TBC
  }

}

/**
* @brief  Modem_USART_Configuration
* USART_MODEM configured as follow:
*      - BaudRate = 115200 baud  
*      - Word Length = 8 Bits
*      - One Stop Bit
*      - No parity
*      - Hardware flow control enabled (RTS and CTS signals)
*      - Receive and transmit enabled
*
* @param  None
* @retval None
*/
void Modem_UART_Configuration(uint32_t baud_rate)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
  
  //JRF
  /* Enable DMA clock */
  DMA_MODEM_CLK_ENABLE();
  
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = USART_MODEM_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.Request             = USART_MODEM_TX_DMA_REQUEST;

  HAL_DMA_Init(&hdma_tx);

  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA(&UartModemHandle, hdmatx, hdma_tx);  
  
  /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
  HAL_NVIC_SetPriority(USART_MODEM_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART_MODEM_DMA_TX_IRQn);  
  
//  UartModemHandle.Instance             = USART_MODEM;
//  UartModemHandle.Init.BaudRate        = baud_rate;
//  UartModemHandle.Init.WordLength      = UART_WORDLENGTH_8B;
//  UartModemHandle.Init.StopBits        = UART_STOPBITS_1;
//  UartModemHandle.Init.Parity          = UART_PARITY_NONE ;
//  UartModemHandle.Init.HwFlowCtl       = UART_HWCONTROL_NONE;
//  UartModemHandle.Init.Mode            = UART_MODE_TX_RX;
//  UartModemHandle.Init.OverSampling    = UART_OVERSAMPLING_8;//JRF UART_OVERSAMPLING_16;
//  UartModemHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  UartModemHandle.Instance = USART_MODEM;
  UartModemHandle.Init.BaudRate = baud_rate;
  UartModemHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartModemHandle.Init.StopBits = UART_STOPBITS_1;
  UartModemHandle.Init.Parity = UART_PARITY_NONE;
  UartModemHandle.Init.Mode = UART_MODE_TX_RX;
  UartModemHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartModemHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  UartModemHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  UartModemHandle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
#ifdef PCB_1_0  
  // needed to do the swap of the USART1 pins
  UartModemHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  UartModemHandle.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
#else
  //HW fixed no need of swapping USART1 pins
  UartModemHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif  
    
  if(HAL_UART_DeInit(&UartModemHandle) != HAL_OK)
  {
    Error_Handler(); //JRF TBC
  }
  
  if(HAL_UART_Init(&UartModemHandle) != HAL_OK)
  {
    Error_Handler(); //JRF TBC
  }
  else
  {
    /*Enabling ISR */
    HAL_NVIC_SetPriority(USART_MODEM_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART_MODEM_IRQn);
  }
 }

void Modem_UART_DeInit (void)
{
  HAL_UART_DeInit(&UartModemHandle);
  while (1)
  {
    Error_Handler(); //JRF TBC
  } 
}
    


/**
* @brief  Modem_PWRKEY_Configuration
* Configure PWRKEY pin used to 
* power-on/power-off the modem
*
* @param  None
* @retval None
*/
void Modem_PWRKEY_Configuration(void)
{
//  GPIO_InitTypeDef  GPIO_InitStruct;
// 
//  PWRKEY_CLK_ENABLE();
//  GPIO_InitStruct.Mode   = GPIO_MODE_OUTPUT_OD;
//  GPIO_InitStruct.Pull   = GPIO_PULLUP; 
//  GPIO_InitStruct.Speed  = GPIO_SPEED_HIGH;
//  GPIO_InitStruct.Pin    = PWRKEY_PIN;
//  
//  HAL_GPIO_Init(PWRKEY_GPIO_PORT, &GPIO_InitStruct);
//  HAL_GPIO_WritePin(PWRKEY_GPIO_PORT, PWRKEY_PIN , GPIO_PIN_SET);
  
}

/** JRF ***************************************
* @brief  This function handles USARTx Handler.
* @param  None
* @retval None
*/
void USART_MODEM_IRQHandler(void)													//USART1_IRQHandler
{
    HAL_UART_IRQHandler(&UartModemHandle);
}

/**
* @brief  This function handles TIM interrupt request.
* @param  None
* @retval None
    TIMx counter clock equal to 10000 Hz . With a period of 1K. so it is running at 10Hz. 100ms/.1s
*/
void TIM_Modem_IRQHandler(void)	//TIM5_IRQHandler
{
    HAL_TIM_IRQHandler(&modemTimHandle);
}

/**
* @brief  This function handles TIM interrupt request.
* @param  None
* @retval None
*/
void TIM_Modemp_IRQHandler(void)	     //TIM2_IRQHandler : NOT used
{
    HAL_TIM_IRQHandler(&modemPushTimHandle); //JRF TBC
}

///**
//* @brief  Period elapsed callback in non blocking mode
//*         This timer is used for calling back User registered functions with information
//* @param  htim : TIM handle
//* @retval None
//*/
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{ 
//  if (htim==&modemTimHandle)
//  {
//  
//    modemTick++;
//    if (modemTimeout)
//    {
//      modemTimeout--;
//    }
//    Modem_TIM_Handler (htim);
//
//  }  
//}

/**
* @brief TIM MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
* @param htim: TIM handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{    
  
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */
    //HAL_TIM_Base_Start_IT(htim);
  /* USER CODE END TIM1_MspInit 1 */
  }  
    if ( htim == &modemPushTimHandle)
    {    
        /* TIMx Peripheral clock enable */
        TIM_MODEMp_CLK_ENABLE();
        
        /*##-2- Configure the NVIC for TIMx ########################################*/
        /* Set the TIMx priority */
        HAL_NVIC_SetPriority(TIM_Modemp_IRQn, 3, 0);
        
        /* Enable the TIMx global Interrupt */
        HAL_NVIC_EnableIRQ(TIM_Modemp_IRQn);
    }
    
    if ( htim == &modemTimHandle)
    {    
        /* TIMx Peripheral clock enable */
        TIM_MODEM_CLK_ENABLE();
        
        /*##-2- Configure the NVIC for TIMx ########################################*/
        /* Set the TIMx priority */
        HAL_NVIC_SetPriority(TIM_Modem_IRQn, 3, 0);
        
        /* Enable the TIMx global Interrupt */
        HAL_NVIC_EnableIRQ(TIM_Modem_IRQn);
    }   
}

/**
 * @brief   Tx Transfer completed callback.
 * @author  JRF
 * @date    2018-08-05
 * @param   huart       STM32 UART instance
*/
void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
    /*Modem Tx callback*/  //JRF
    if (huart->Instance == USART_MODEM)
    {
      Modem_HAL_UART_TxCpltCallback(huart);
    }    
#ifdef COMM_DEBUG    
    /*COMM_DEBUG Tx callback*/ 
    if (huart->Instance == USART4)
    {
    }
#endif
}

/**
 * @brief   Rx Transfer completed callback.
 * @author  JRF
 * @date    2018-08-05
 * @param   huart       STM32 UART instance
*/
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
    /*Modem Rx callback*/
    if (huart->Instance == USART_MODEM)
    {
        Modem_HAL_UART_RxCpltCallback(huart);
    }
}


/* USER CODE END 1 */

#ifdef USART_PRINT_MSG
void USART_PRINT_MSG_Configuration(uint32_t baud_rate)
{
  UartMsgHandle.Instance             = UART_MSG;
  UartMsgHandle.Init.BaudRate        = baud_rate;
  UartMsgHandle.Init.WordLength      = UART_WORDLENGTH_8B;
  UartMsgHandle.Init.StopBits        = UART_STOPBITS_1;
  UartMsgHandle.Init.Parity          = UART_PARITY_NONE ;
  UartMsgHandle.Init.HwFlowCtl       = UART_HWCONTROL_NONE;// USART_HardwareFlowControl_RTS_CTS;
  UartMsgHandle.Init.Mode            = UART_MODE_TX_RX;

  if(HAL_UART_DeInit(&UartMsgHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartMsgHandle) != HAL_OK)
  {
    Error_Handler();
  }
      
}


void UART_Msg_Gpio_Init()
{ 
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_PRINT_TX_GPIO_CLK_ENABLE();
  USARTx_PRINT_RX_GPIO_CLK_ENABLE();


  /* Enable USARTx clock */
  USARTx_PRINT_CLK_ENABLE(); 
    __SYSCFG_CLK_ENABLE();
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USART_PRINT_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
  GPIO_InitStruct.Alternate = PRINTMSG_USARTx_TX_AF;
#endif  
  HAL_GPIO_Init(USART_PRINT_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USART_PRINT_RX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = PRINTMSG_USARTx_RX_AF;
#endif 
  
  HAL_GPIO_Init(USART_PRINT_RX_GPIO_PORT, &GPIO_InitStruct);
  
#ifdef WIFI_USE_VCOM
  /*##-3- Configure the NVIC for UART ########################################*/
  /* NVIC for USART */
  HAL_NVIC_SetPriority(USARTx_PRINT_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USARTx_PRINT_IRQn);
#endif
}
#endif



/**
  * @}
  */ 

/**
  * @}
  */ 


/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/

