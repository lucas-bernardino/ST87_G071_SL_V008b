#include "main.h"
#include "sensor_module.h"
//#include "metro.h"

#ifdef LIS2DUX12
#include "lis2dux12_reg.h"
#endif

#ifdef LIS2DW12
#include "lis2dw12_reg.h"
#endif

#ifdef LIS2DU12
#include "lis2du12_reg.h"
#endif

#include "metro.h"
#include "cmsis_os.h"

#ifdef LIS2DU12
#define SA0 0
#define WRITE_ADDRESS (0x32) /* see datasheet 6.1.1 */
#define READ_ADDRESS  (0x33) 
#endif

#ifdef LIS2DUX12
#define SA0 0
#define WRITE_ADDRESS (0x30)   /* see datasheet 6.1.1 */
#define READ_ADDRESS  (0x31)   
#endif

#ifdef LIS2DW12
#define SA0 0
#define WRITE_ADDRESS (0x30)   /* see datasheet 6.1.1 */
#define READ_ADDRESS  (0x31)   
#endif

#define BUS_I2C_POLL_TIMEOUT                0x1000U
#define SENSOR_BUS hi2c2

/* BSP Common Error codes */
#define BSP_ERROR_NONE                    0
#define BSP_ERROR_NO_INIT                -1
#define BSP_ERROR_WRONG_PARAM            -2
#define BSP_ERROR_BUSY                   -3
#define BSP_ERROR_PERIPH_FAILURE         -4
#define BSP_ERROR_COMPONENT_FAILURE      -5
#define BSP_ERROR_UNKNOWN_FAILURE        -6
#define BSP_ERROR_UNKNOWN_COMPONENT      -7
#define BSP_ERROR_BUS_FAILURE            -8
#define BSP_ERROR_CLOCK_FAILURE          -9
#define BSP_ERROR_MSP_FAILURE            -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED      -11

/* BSP BUS error codes */

#define BSP_ERROR_BUS_TRANSACTION_FAILURE    -100
#define BSP_ERROR_BUS_ARBITRATION_LOSS       -101
#define BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE    -102
#define BSP_ERROR_BUS_PROTOCOL_FAILURE       -103

#define BSP_ERROR_BUS_MODE_FAULT             -104
#define BSP_ERROR_BUS_FRAME_ERROR            -105
#define BSP_ERROR_BUS_CRC_ERROR              -106
#define BSP_ERROR_BUS_DMA_FAILURE            -107

#define BOOT_TIME         10 //ms

#define NSAMPLES  10

#define TEMP_CAL_FACTOR 3

extern osMessageQueueId_t SensorQueueHandle;
extern osMessageQueueId_t LedStatusQueueHandle;
extern osMessageQueueId_t ST87CommStatusHandle;

extern I2C_HandleTypeDef hi2c2;
extern lamp_info_t Load_data;

static int32_t platform_write(void *handle, uint8_t reg,const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);

uint8_t pBuffer[6];
uint8_t statusreg;
static stmdev_ctx_t dev_ctx;
static uint8_t reg_tmp;
uint8_t drdy=0;
sensor_data_t Sensor_data;
float temp_avg[NSAMPLES];
uint8_t idx=0;
led_status_t led_mems_action;
uint8_t ST87_Comm_ON=0;

#ifdef LIS2DW12
static int16_t data_raw_acceleration[3];
static float_t acceleration_mg[3];
uint16_t data_temp;
static uint8_t whoamI, rst;
//static uint8_t tx_buffer[1000];
static lis2dw12_ctrl4_int1_pad_ctrl_t  ctrl4_int1_pad;
lis2dw12_all_sources_t all_int_status;
#endif

#ifdef LIS2DU12
lis2du12_status_register_t status_reg;
static lis2du12_status_t status;
static lis2du12_md_t md;
static lis2du12_data_t data;
//lis2du12_pin_int_route_t int1_route;
//static lis2du12_pin_int_route_t int_route;
lis2du12_id_t id;
//static lis2du12_int_mode_t int_mode;
static uint8_t tap_1_event = 0;
static uint8_t tap_2_event = 0;
lis2du12_tap_md_t tap_mode;
lis2du12_all_sources_t status_all;

static int32_t platform_write(void *handle, uint8_t reg,const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LIS2DU12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, BUS_I2C_POLL_TIMEOUT);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LIS2DU12_I2C_ADD_H, reg,I2C_MEMADD_SIZE_8BIT, bufp, len, BUS_I2C_POLL_TIMEOUT);
  return 0;
}

void lis2du12_read_drdy_handler(void)
{
  lis2du12_data_get(&dev_ctx, &md, &data);
}

/* irq handler ---------------------------------------------------------------*/
void lis2du12_tap_irq_handler(void)
{
  lis2du12_all_sources_t status;

  lis2du12_all_sources_get(&dev_ctx, &status);

  if (status.single_tap) {
    tap_1_event = 1;
  }
  if (status.double_tap) {
    tap_2_event = 1;
  }

}
#endif

#ifdef LIS2DUX12
lis2dux12_status_register_t status_reg;
static lis2dux12_md_t md;
lis2dux12_pin_int_route_t int_route;
lis2dux12_status_t status;
uint8_t id;
static lis2dux12_xl_data_t data_xl;
static lis2dux12_outt_data_t data_temp;
static uint8_t fifo_1_tap_event = 0;
static uint8_t fifo_2_tap_event = 0;
lis2dux12_tap_config_t val;
lis2dux12_int_config_t int_mode;
lis2dux12_all_sources_t all_int_status;

///JRF
static int32_t platform_write(void *handle, uint8_t reg,const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LIS2DUX12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, BUS_I2C_POLL_TIMEOUT);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LIS2DUX12_I2C_ADD_L, reg,I2C_MEMADD_SIZE_8BIT, bufp, len, BUS_I2C_POLL_TIMEOUT);
  return 0;
}

void lis2dux12_tap_handler(void)
{
  lis2dux12_all_sources_t status;

  lis2dux12_all_sources_get(&dev_ctx, &status);

  if (status.single_tap) {
    fifo_1_tap_event = 1;
  }
  if (status.double_tap) {
    fifo_2_tap_event = 1;
  }
}

#endif

#ifdef LIS2DW12
static int32_t platform_write(void *handle, uint8_t reg,const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, BUS_I2C_POLL_TIMEOUT);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_L, reg,I2C_MEMADD_SIZE_8BIT, bufp, len, BUS_I2C_POLL_TIMEOUT);
  return 0;
}
#endif

float Temperature_avg (float *temp, uint8_t samples)
{
  uint8_t i;
  float aux=0;
  for (i=0;i<samples;i++)
  {
    aux+=*(temp+i);
  }
  return(aux/samples);
}

void Sensor_Init (void)
{
  dev_ctx.read_reg  = (stmdev_read_ptr)platform_read;
  dev_ctx.write_reg = (stmdev_write_ptr)platform_write;
  dev_ctx.mdelay    = HAL_Delay;
  dev_ctx.handle    = &SENSOR_BUS;
  
  HAL_Delay (BOOT_TIME);
  
#ifdef LIS2DU12
  HAL_I2C_Mem_Read(&hi2c2, READ_ADDRESS, LIS2DU12_WHO_AM_I, 1, pBuffer, 1,100);
  if(pBuffer[0] != LIS2DU12_ID)
  {
    Error_Handler();
  }

  /* Check device ID */
  lis2du12_id_get(&dev_ctx, &id);
  if (id.whoami != LIS2DU12_ID)
  {
    Error_Handler();
  }
  
 /* Restore default configuration */
  lis2du12_init_set(&dev_ctx, LIS2DU12_RESET);
  do {
    lis2du12_status_get(&dev_ctx, &status);
  } while (status.sw_reset);

  /* Set bdu and if_inc recommended for driver usage */
  lis2du12_init_set(&dev_ctx, LIS2DU12_DRV_RDY);  
  
//  int_mode.enable = PROPERTY_ENABLE;
//  int_mode.active_low = PROPERTY_DISABLE;
//  int_mode.base_sig = LIS2DU12_INT_LATCHED;
//  lis2du12_interrupt_mode_set(&dev_ctx, &int_mode);  
//  
//  tap_mode.z_en = 1;
//  tap_mode.threshold.z = 3; /* LSB is FS/32 */
//  tap_mode.priority = LIS2DU12_ZYX;
//  tap_mode.shock = 0; /* LSB is 4*ODR */
//  tap_mode.quiet = 3; /* LSB is 8*ODR */
//  tap_mode.tap_double.en = 1;
//  tap_mode.tap_double.latency = 5;
//  lis2du12_tap_mode_set(&dev_ctx, &tap_mode);
//
//  /* Configure interrupt pins */
//  lis2du12_pin_int1_route_get(&dev_ctx, &int_route);
//  int_route.single_tap = PROPERTY_ENABLE;
//  int_route.double_tap = PROPERTY_ENABLE;
//  lis2du12_pin_int1_route_set(&dev_ctx, &int_route);  
  
  /* Set Output Data Rate */
  md.fs =  LIS2DU12_4g;
  md.odr = LIS2DU12_25Hz;
  lis2du12_mode_set(&dev_ctx, &md);
  
  // sanity check
  lis2du12_read_reg(&dev_ctx, LIS2DU12_CTRL2, &reg_tmp, 1);
  reg_tmp |= 0x08;
  lis2du12_write_reg(&dev_ctx, LIS2DU12_CTRL2, &reg_tmp, 1);
  reg_tmp=0;
  lis2du12_read_reg(&dev_ctx, LIS2DU12_CTRL2, &reg_tmp, 1);
  lis2du12_read_reg(&dev_ctx, LIS2DU12_INTERRUPT_CFG, &reg_tmp, 1);
  lis2du12_read_reg(&dev_ctx, LIS2DU12_MD1_CFG, &reg_tmp, 1);
//  
  //first XL data reading
  lis2du12_data_get(&dev_ctx, &md, &data);
  Sensor_data.Ax = data.xl.mg[0];
  Sensor_data.Ay = data.xl.mg[1];
  Sensor_data.Az = data.xl.mg[2];
  Sensor_data.temp = data.heat.deg_c;
#endif
  
#ifdef LIS2DUX12
  lis2dux12_exit_deep_power_down(&dev_ctx);

  /* Check device ID */
  lis2dux12_device_id_get(&dev_ctx, &id);
  if (id != LIS2DUX12_ID)
    while(1);

  /* Restore default configuration */
  lis2dux12_init_set(&dev_ctx, LIS2DUX12_RESET);
  do {
    lis2dux12_status_get(&dev_ctx, &status);
  } while (status.sw_reset);

  /* Set bdu and if_inc recommended for driver usage */
  lis2dux12_init_set(&dev_ctx, LIS2DUX12_SENSOR_ONLY_ON);
  int_mode.int_cfg = LIS2DUX12_INT_LATCHED;
  lis2dux12_int_config_set(&dev_ctx, &int_mode);  
  
  
  val.axis = LIS2DUX12_TAP_ON_Z;
  val.pre_still_ths = 125.0f / 62.5f;
  val.post_still_ths = 500.0f / 62.5f;
  val.post_still_time = 32 / 4 /2;
  val.peak_ths = 500.0f / 62.5f;
  val.pre_still_start = 0;
  val.pre_still_n = 10;
  val.inverted_peak_time = 4;
  val.shock_wait_time = 6 / 2;
  val.rebound = 0;
  val.latency = 128 / 32 /2;
  val.single_tap_on = 1;
  val.double_tap_on = 1;
  val.triple_tap_on = 0;
  val.wait_end_latency = 1;  
  lis2dux12_tap_config_set(&dev_ctx, val);  

  /* Configure interrupt pins */
  int_route.drdy   = PROPERTY_ENABLE;
  int_route.tap    = PROPERTY_ENABLE;
  lis2dux12_pin_int1_route_set(&dev_ctx, &int_route);
  
  int_mode.int_cfg = LIS2DUX12_INT_LEVEL;
  lis2dux12_int_config_set(&dev_ctx, &int_mode);  
  
  /* Set Output Data Rate */
  md.fs  = LIS2DUX12_8g;
  //md.bw  = LIS2DUX12_ODR_div_4;
  md.odr = LIS2DUX12_25Hz_HP; //LIS2DUX12_25Hz_LP;
  lis2dux12_mode_set(&dev_ctx, &md);  
  
  //first XL data reading
  lis2dux12_xl_data_get(&dev_ctx, &md, &data_xl);
  lis2dux12_outt_data_get(&dev_ctx, &data_temp);
  Sensor_data.Ax = data_xl.mg[0];
  Sensor_data.Ay = data_xl.mg[1];
  Sensor_data.Az = data_xl.mg[2];
  Sensor_data.temp = data_temp.heat.deg_c - TEMP_CAL_FACTOR;
#endif
#ifdef LIS2DW12
  /* Check device ID */
  
  lis2dw12_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS2DW12_ID)
    while (1) {
      /* manage here device not found */
    }

  /* Restore default configuration */
  lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lis2dw12_reset_get(&dev_ctx, &rst);
  } while (rst);
  
  lis2dw12_int_notification_set(&dev_ctx, LIS2DW12_INT_LATCHED);
  lis2dw12_pin_polarity_set(&dev_ctx,LIS2DW12_ACTIVE_HIGH); 
  lis2dw12_pin_int1_route_get(&dev_ctx, &ctrl4_int1_pad);
  ctrl4_int1_pad.int1_drdy = PROPERTY_ENABLE;
  lis2dw12_pin_int1_route_set(&dev_ctx, &ctrl4_int1_pad);
  
  /* Set full scale */
  lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_8g);
  lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_25Hz);  
  
  
  /* Configure filtering chain accelerometer */
  //lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
  //lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_10);
  /* Configure power mode and Output Data Rate */
   //LIS2DW12_CONT_LOW_PWR_12bit);
  
  
  //first XL data reading
  data_raw_acceleration[0] = data_raw_acceleration [1] = data_raw_acceleration [2] = 0;
  lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  lis2dw12_temperature_raw_get(&dev_ctx, &data_temp );
  Sensor_data.Ax = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[0]);
  Sensor_data.Ay = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[1]);
  Sensor_data.Az = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[2]);  
  Sensor_data.temp = lis2dw12_from_lsb_to_celsius(data_temp);
#endif
  
  uint8_t j;
  for (j=0;j<=NSAMPLES;j++)
  {
    temp_avg[j] = Sensor_data.temp;
  }
  idx = 0;
  
#ifdef LIS2DW12
#endif
   
  //enable sensor Int ***must be here to avoid hard fault
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 1);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
  
}

void Sensor_Interrupt (void)
{
  //osMessageQueueGet(ST87CommStatusHandle,&ST87_Comm_ON , 0, 0);
  
 //if (ST87_Comm_ON==0)
 {
#ifdef LIS2DU12
  lis2du12_read_reg(&dev_ctx,LIS2DU12_STATUS ,(uint8_t*)&status_reg,1); 
  lis2du12_data_get(&dev_ctx, &md, &data);
  Sensor_data.Ax = data.xl.mg[0];
  Sensor_data.Ay = data.xl.mg[1];
  Sensor_data.Az = data.xl.mg[2];
  Sensor_data.temp = data.heat.deg_c;   
  temp_avg [idx] = data.heat.deg_c; 
  idx++;
  if (idx >=NSAMPLES)
  {
    idx = 0;
  }  
#endif

#ifdef LIS2DUX12
  lis2dux12_all_sources_get(&dev_ctx, &all_int_status);
  lis2dux12_xl_data_get(&dev_ctx, &md, &data_xl);
  lis2dux12_outt_data_get(&dev_ctx, &data_temp);
  Sensor_data.Ax = data_xl.mg[0];
  Sensor_data.Ay = data_xl.mg[1];
  Sensor_data.Az = data_xl.mg[2];
  lis2dux12_all_sources_get(&dev_ctx, &all_int_status);
  Sensor_data.temp = data_temp.heat.deg_c;
  
  lis2dux12_tap_handler();

  if (all_int_status.single_tap) {
    all_int_status.single_tap = 0;
    led_mems_action.state = 1;
    led_mems_action.update_state = 1;
    led_mems_action.dimmer = 15;
  }
  if (all_int_status.double_tap) {
    all_int_status.double_tap = 0;
    led_mems_action.state = 0;
    led_mems_action.update_state = 1;    
    led_mems_action.dimmer = 0;
  }
  temp_avg [idx] = data_temp.heat.deg_c; 
  idx++;
  if (idx >=NSAMPLES)
  {
    idx = 0;
  }  
#endif  
  
#ifdef LIS2DW12
  data_raw_acceleration[0] = data_raw_acceleration [1] = data_raw_acceleration [2] = 0;
  lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  lis2dw12_temperature_raw_get(&dev_ctx, &data_temp );
  Sensor_data.Ax = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[0]);
  Sensor_data.Ay = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[1]);
  Sensor_data.Az = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[2]);
  temp_avg [idx] = lis2dw12_from_lsb_to_celsius(data_temp);  
  idx++;
  if (idx >=NSAMPLES)
  {
    idx = 0;
  }     
#endif  

  Sensor_data.temp = Temperature_avg(&temp_avg[0],NSAMPLES) - TEMP_CAL_FACTOR; 
  HAL_GPIO_TogglePin(LED_COMM_SENSOR_GPIO_Port, LED_COMM_SENSOR_Pin);
 }
}

void Sensor_Controller (void)
{
  //led_mems_action.update_state = 0;
//  lis2dw12_all_sources_get(&dev_ctx, &all_int_status);
//  if (all_int_status.
//      .drdy)
//  {
//    drdy = 1;
//  }
  if (drdy)
  {
    //osMessageQueueGet(LedStatusQueueHandle,&led_action , 0, 0);
    drdy = 0;
    Sensor_Interrupt();
    osMessageQueuePut(SensorQueueHandle,&Sensor_data , 0, 0); 
    //osMessageQueuePut(LedStatusQueueHandle,&led_mems_action,0,0);
  }
  //osDelay(1000);
}