/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_TIM1_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENSOR_INT_Pin LL_GPIO_PIN_7
#define SENSOR_INT_GPIO_Port GPIOA
#define SENSOR_SCL_Pin LL_GPIO_PIN_11
#define SENSOR_SCL_GPIO_Port GPIOA
#define SENSOR_SDA_Pin LL_GPIO_PIN_12
#define SENSOR_SDA_GPIO_Port GPIOA
#define PROT_SEL_Pin LL_GPIO_PIN_13
#define PROT_SEL_GPIO_Port GPIOA
#define PORT_SEL_Pin LL_GPIO_PIN_14
#define PORT_SEL_GPIO_Port GPIOA
#define ST87_nRST_Pin LL_GPIO_PIN_15
#define ST87_nRST_GPIO_Port GPIOA
#define LED_COMM_SENSOR_Pin LL_GPIO_PIN_3
#define LED_COMM_SENSOR_GPIO_Port GPIOB
#define LED_COMM_MET_Pin LL_GPIO_PIN_4
#define LED_COMM_MET_GPIO_Port GPIOB
#define LED_NETWORK_Pin LL_GPIO_PIN_5
#define LED_NETWORK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define COMM_DEBUG
//#define DEBUG_MODE

//#define AME_DEMO
#define EU_DEMO

#define UDP_ECHO_SERVER            1777               /* Echo Server for UDP messages */
#define TCP_ECHO_SERVER            1778               /* Echo Server for UDP messages */

#define US_UDP_SERVER_PORT         1337               /* US udp server port */
#define EU_UDP_SERVER_PORT         1338               /* EU udp server port */
#define EU_UDP_SERVER_PORT2        1339               /* EU udp server port 2*/

#define US_TCP_SERVER_PORT         1900               /* US tcp server port */
#define EU_TCP_SERVER_PORT         1901               /* EU tcp server port */
#define EU_TCP_SERVER_PORT2        1902               /* EU tcp server port */

#define SERVER_IP              "162.248.102.96"       /* joinstlab.net */

#define APN_TIM                "nbiot.gsim"           /* apn for TIM */
#define APN_AMARI              "Default"              /* apn for Amari callbox */
#define APN_1NCE               "iot.1nce.net"         /* apn for 1NCE used in Europe */
#define APN_VIVO               "kiteiot.vivo.com.br"  /* apn for Vivo-Telefonica */
#define APN_ONOMONDO           "onomondo"             /* apn for Onomondo -> T-Mobile */
#define APN_APP                (APN_1NCE)

#define AMARI_BANDS            "8"
#define EU_BANDS               "20,8" 
#define US_BANDS               "4,12"  
#define BRA_BANDS              "28,3" 
#define BANDS_APP              (EU_BANDS)

#define TCP_TYPE               0
#define UDP_TYPE               1

#define TTI_DEFAULT 10     /* interval between transmissions */  

extern uint8_t sckt_type; 
extern uint8_t port_number;

//#define MEMS_TYPE               (LIS2DUX12)
//#define MEMS_TYPE               (LIS2DU12) 
//#define MEMS_TYPE               (LIS2DUX12) 


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
