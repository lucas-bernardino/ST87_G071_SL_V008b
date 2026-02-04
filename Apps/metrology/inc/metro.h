#if defined (STM32L4)
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#endif

#if defined (STM32G0)
#include "stm32g0xx.h"
#else
#warning "Define STM32 familiy!!!!"
#endif

typedef struct
{
  uint8_t update_state;
  uint8_t state;
  uint8_t dimmer;
  uint8_t tti;
}
led_status_t;

typedef struct
{
  int32_t      energyActive;
  int32_t      energyReactive;
  int32_t      rmsvoltage;
  int32_t      rmscurrent;  
  led_status_t status;
}
lamp_info_t;



//typedef struct
//{
//  int32_t energyActive;
//  int32_t energyReactive;
//  int32_t rmsvoltage;
//  int32_t rmscurrent;  
//  uint16_t state;
//  uint16_t dimm;
//}
//Load_status_t;
//
//typedef struct
//{
//  uint8_t update_state;
//  uint8_t led_state;
//  uint8_t dimmer;
//  uint8_t tti;
//}
//led_status_t;