/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at: opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "com.h"
#include <math.h>
#include <stdlib.h>
#include "C:\IKS0LAB_SensorHub-modified\IKS0LAB_SensorHub\Drivers\lps22hh\lps22hh.h"
#include "C:\IKS0LAB_SensorHub-modified\IKS0LAB_SensorHub\Drivers\lps22hh\lps22hh_reg.h"
#include "C:\IKS0LAB_SensorHub-modified\IKS0LAB_SensorHub\Drivers\lsm6dsox\lsm6dsox.h"
#include "C:\IKS0LAB_SensorHub-modified\IKS0LAB_SensorHub\Drivers\lsm6dsox\lsm6dsox_reg.h"

/* Private typedef -----------------------------------------------------------*/
extern  I2C_HandleTypeDef I2C_EXPBD_Handle;

/* Private define ------------------------------------------------------------*/
#define NUM_RECORDS              10
#define MAX_BUF_SIZE             256
#define UART_TRANSMIT_TIMEOUT    5000
#define TX_BUF_DIM               1000
#define USE_STM32F4XX_NUCLEO
#define BSP_ERROR_NONE           0
#define BSP_ERROR_WRONG_PARAM    -2
#define BSP_ERROR_PERIPH_FAILURE -4
#define KEY_BUTTON_PIN          GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT    GPIOC
#define USER_BUTTON_EXTI_IRQn    EXTI15_10_IRQn
#define KEY_BUTTON_GPIO_PORT     USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_EXTI_IRQn     USER_BUTTON_EXTI_IRQn
#define INT2_master_Pin          GPIO_PIN_1
#define BUTTONn                  1
#define NUM_CALIB                4
#define NUM_FEATURES             16

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT};
const uint16_t BUTTON_PIN[BUTTONn] = {KEY_BUTTON_PIN};
const uint8_t BUTTON_IRQn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn};

typedef enum
{
  BUTTON_USER = 0U,
}Button_TypeDef;

typedef enum
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

typedef enum
{
  COM1 = 0U,
  COMn
}COM_TypeDef;

#define COMn                             1U
#define COM1_UART                        USART2

#define COM_POLL_TIMEOUT                 1000
extern UART_HandleTypeDef hcom_uart[COMn];
#define  huart2 hcom_uart[COM1]
USART_TypeDef* COM_USART[COMn] = {COM1_UART};
UART_HandleTypeDef hcom_uart[COMn];

#define USARTx_TX_AF                            GPIO_AF7_USART2
#define USARTx_RX_AF                            GPIO_AF7_USART2
#define USER_BUTTON_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()   
#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)      USER_BUTTON_GPIO_CLK_ENABLE()

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* union to manage LPS22HH data */
typedef union{
  struct {
    uint32_t u32bit; /* pressure plus status register */
    int16_t  i16bit; /* temperature */
  } p_and_t;
  uint8_t u8bit[6];
} p_and_t_byte_t;

typedef struct record {
  float acc[3];
  float press;
} record_t;

typedef struct fifo_record {
  record_t fifo[NUM_RECORDS];
  record_t mean;
} fifo_record_t;

typedef struct features_t
{
  float data[NUM_FEATURES];
} features_t;

typedef struct labeled_record {
  features_t features;
  int8_t label; // 0 means moving down horizontally, 1 down vertically, 2 means up horizontally, 3 means up vertically 
} labeled_record;

typedef struct indexer
{
  uint8_t hint[((int)NUM_CALIB + 7) / 8];
} indexer;

void set_index(indexer * idx, int id)
{
  //  printf("array id: %d, bit id: %d, %d\n", (id / 8), (id % 8), idx->hint[id / 8]);
  // id / 8 is the index in the hint array
  // id mod 8 is the index of the bit of the uint8_t
  idx->hint[id / 8] |= (1 << (id % 8));
//  printf("after %d\n", idx->hint[id / 8]);
}

void unset_index(indexer * idx, int id)
{
  // id / 8 is the index in the hint array
  // id mod 8 is the index of the bit of the uint8_t
  // clears the appropriate bit
  idx->hint[id / 8] &= ~(1 << (id % 8));
}

uint8_t is_used(const indexer * idx, int id)
{
  return (idx->hint[id / 8] & (1 << (id % 8))) ? 1 : 0;
}

typedef struct node_t {
  float split;
  uint8_t dim; // column where the data set is split, feature ID
  uint8_t label;
  float error; // fraction of opposite labeled data points
  int cnt[4];
  
  indexer idx; // stores which data points are for this node
  labeled_record * data; // p0inter to all data
  
  struct node_t * son; // left
  struct node_t * daughter; // right
} node_t;

/* Private variables ---------------------------------------------------------*/
static char dataOut[MAX_BUF_SIZE];
static stmdev_ctx_t ag_ctx;
//static stmdev_ctx_t press_ctx;

/* Volatile variables */
static volatile uint8_t button_pressed = 0;
static volatile uint8_t MLC_interrupt = 0;
volatile uint32_t Int_Current_Time1 = 0; /*!< Int_Current_Time1 Value */
volatile uint32_t Int_Current_Time2 = 0; /*!< Int_Current_Time2 Value */

/* Private function prototypes -----------------------------------------------*/
static void Sleep_Mode(void);
void SystemClock_Config(void);
static int32_t lsm6dsox_write_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len);
static int32_t lsm6dsox_read_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len);
int32_t BSP_COM_Init(COM_TypeDef COM);
static void USART2_MspInit(UART_HandleTypeDef *huart);
void  BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void USARTConfig(void);
void ErrorHandler(void);
fifo_record_t read_it_my_boy(void);
void compute_mean(const node_t * node, features_t * features, char label);
void compute_std(const node_t * node, const features_t * mean, features_t * features, char label);
float compute_gini(const node_t * node, float split_point, int dim);
float dim_data(const record_t * data, int dim);
static float square(float val);
node_t * dec_tree_generator(labeled_record data[]);

#ifndef M_PI
#define M_PI (3.141592653589793)
#endif
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
      snprintf(dataOut, MAX_BUF_SIZE, "\r\nError.\r\n");
      HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
}

double getInverseCDFValue(float p)
{
double a1 = -39.69683028665376;
double a2 = 220.9460984245205;
double a3 = -275.9285104469687;
double a4 = 138.3577518672690;
double a5 =-30.66479806614716;
double a6 = 2.506628277459239;

double b1 = -54.47609879822406;
double b2 = 161.5858368580409;
double b3 = -155.6989798598866;
double b4 = 66.80131188771972;
double b5 = -13.28068155288572;

double c1 = -0.007784894002430293;
double c2 = -0.3223964580411365;
double c3 = -2.400758277161838;
double c4 = -2.549732539343734;
double c5 = 4.374664141464968;
double c6 = 2.938163982698783;

double d1 = 0.007784695709041462;
double d2 = 0.3224671290700398;
double d3 = 2.445134137142996;
double d4 = 3.754408661907416;

//Define break-points.

double p_low =  0.02425;
double p_high = 1 - p_low;
long double  q, r, e, u;
long double x = 0.0;


//Rational approximation for lower region.

if (0 < p && p < p_low) {
    q = sqrt(-2*log(p));
    x = (((((c1*q+c2)*q+c3)*q+c4)*q+c5)*q+c6) / ((((d1*q+d2)*q+d3)*q+d4)*q+1);
}

//Rational approximation for central region.

if (p_low <= p && p <= p_high) {
    q = p - 0.5;
    r = q*q;
    x = (((((a1*r+a2)*r+a3)*r+a4)*r+a5)*r+a6)*q / (((((b1*r+b2)*r+b3)*r+b4)*r+b5)*r+1);
}

//Rational approximation for upper region.

if (p_high < p && p < 1) {
    q = sqrt(-2*log(1-p));
    x = -(((((c1*q+c2)*q+c3)*q+c4)*q+c5)*q+c6) / ((((d1*q+d2)*q+d3)*q+d4)*q+1);
}


//Pseudo-code algorithm for refinement

if(( 0 < p)&&(p < 1)){
    e = 0.5 * erfc(-x/sqrt(2)) - p;
    u = e * sqrt(2*M_PI) * exp(x*x/2);
    x = x - u/(1 + x*u/2);
}

return x;
}

/* Configure low level function to access to external device
 * Check if LPS22HH connected to Sensor Hub
 * Configure lps22hh for data acquisition
 * Configure Sensor Hub to read one slave with XL trigger
 * Set FIFO watermark
 * Set FIFO mode to Stream mode
 * Enable FIFO batching of Slave0 + ACC + Gyro samples
 * Poll for FIFO watermark interrupt and read samples
 */
void lsm6dsox_hub_fifo_lps22hh(void)
{
  uint8_t rst, whoamI1 = 255, whoamI2 = 255;

  /* Initialize lsm6dsox driver interface */
  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &hi2c1;

  /* Initialize lps22hh driver interface */
  stmdev_ctx_t * press_ctx = (stmdev_ctx_t *)malloc(sizeof(stmdev_ctx_t));
  (*press_ctx).read_reg = lsm6dsox_read_lps22hh_cx;
  (*press_ctx).write_reg = lsm6dsox_write_lps22hh_cx;
  (*press_ctx).handle = &hi2c1;
 
  /* Check if LPS22HH connected to Sensor Hub. */
  lps22hh_device_id_get(press_ctx, &whoamI2);
  if ( whoamI2 != LPS22HH_ID )
    while(1); /*manage here device not found */
 
  /* Check if LSM6DSOX connected to Sensor Hub. */
  lsm6dsox_device_id_get(&ag_ctx, &whoamI1);
  if (whoamI1 != LSM6DSOX_ID)
    while(1); /*manage here device not found */

  /* Restore default configuration. */
  lsm6dsox_reset_set(&ag_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&ag_ctx, &rst);
  } while (rst);

  /* Disable I3C interface.*/
  lsm6dsox_i3c_disable_set(&ag_ctx, LSM6DSOX_I3C_DISABLE);
  
  /* Set data rate.*/
  lps22hh_data_rate_set(press_ctx, LPS22HH_10_Hz_LOW_NOISE);
  return;
}

char getUserInput(UART_HandleTypeDef *huart, char * welcomeMSG, char * validOptions)
{
  uint8_t tmp = 0, res = 0;
  unsigned nOptions = strlen(validOptions), i;
  
  snprintf(dataOut, MAX_BUF_SIZE, "%s\r\n", welcomeMSG);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  while (42) 
  {
    if (HAL_UART_Receive(huart, &tmp, 1, 1000) == HAL_OK)
    {
      if (tmp == 13)
      {
        for (i = 0; i < nOptions; ++i)
        {
          if (res == validOptions[i]) break;
        }
        if (i == nOptions)
        {
          snprintf(dataOut, MAX_BUF_SIZE, "Error: invalid choice: %c\r\n", (char)res);
          HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
          continue;
        }
        return res;
      }
      res = tmp;
    }
  }
}

// use label 255 for computing means of all axes
void compute_mean(const node_t * node, features_t * features, char label)
{
  memset(&features, 0, sizeof(features_t));
  
  int num_rec = 0;
  for (int i = 0; i < NUM_CALIB; ++i)
  { 
    if ((label != 255 && node->data[i].label != label) || !is_used(&(node->idx), i)) 
    { 
      continue; 
    }
      
    const labeled_record * record = &(node->data[i]);
    for (int k = 0; k < NUM_FEATURES; ++k)
    {
       features->data[k] += record->features.data[k];
    }
    ++num_rec; 
  }
  
  if (num_rec)
  {
    for (int k = 0; k < NUM_FEATURES; ++k)
    {
      features->data[k] /= num_rec;
    }
  }
}

// label 0 means down, 1 up
// use label 255 for computing means of all axes
void compute_std(const node_t * node, const features_t * mean, features_t * features, char label)
{
  memset(&features, 0, sizeof(features_t));
  
  int num_rec = 0;
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    if ((label != 255 && node->data[i].label != label) || !is_used(&(node->idx), i)) 
    { 
      continue; 
    }
    
    const labeled_record * record = &(node->data[i]);
    for (int k = 0; k < NUM_FEATURES; ++k)
    {
       features->data[k] += square(record->features.data[k] - mean->data[k]);
    }
    ++num_rec;
  }
  
  if (num_rec > 1)
  {
    for (int k = 0; k < NUM_FEATURES; ++k)
    {
      features->data[k] /= num_rec - 1;
      features->data[k] = sqrt(features->data[k]);
    }
  }
}

record_t compute_min(const record_t data[])
{
  record_t records_min = data[0];
  
  for (int j = 1; j < NUM_RECORDS; ++j)
  { 
    for (int i = 0; i < 3; i++)
    {
      if (data[j].acc[i] < records_min.acc[i])
      {
        records_min.acc[i] = data[j].acc[i];
      }
    }
    if (data[j].press < records_min.press)
    {
      records_min.press = data[j].press;
    }
  }

  return records_min;
}

record_t compute_max(const record_t data[])
{
  record_t records_max = data[0];
  
  for (int j = 1; j < NUM_RECORDS; ++j)
  {
    for (int i = 0; i < 4; ++i)
    {
      if (dim_data(&data[j], i) > dim_data(&records_max, i))
      {
        if (i == 3)
        {
          records_max.press = data[j].press;
        }
        else
        {
          records_max.acc[i] = data[j].acc[i];
        }
      }
    }
  }

  return records_max;
}

record_t compute_peak_to_peak(const record_t * records_max, const record_t * records_min)
{
  record_t peak;
  for (int i = 0; i < 3; i++)
  {
      peak.acc[i] = records_max->acc[i] - records_min->acc[i];
  }
  peak.press = records_max->press - records_min->press;
  return peak;
}

fifo_record_t read_it_my_boy(void)
{
  static const int watermark = 3 * NUM_RECORDS;
  record_t record[NUM_RECORDS]; 
  record_t records_mean;
  memset(&records_mean, 0, sizeof(records_mean));
  record_t fifo_samples;
  memset(&fifo_samples, 0, sizeof(fifo_samples));
  fifo_record_t output;
  memset(&output, 0, sizeof(output));
 
  uint8_t wtm_flag;
  lsm6dsox_pin_int1_route_t int1_route;
  lsm6dsox_sh_cfg_read_t sh_cfg_read;
  p_and_t_byte_t data_raw_press_temp;
  axis3bit16_t data_raw_acceleration;
  axis3bit16_t data_raw_angular_rate;
  axis3bit16_t dummy;
  /*
   * Configure LSM6DSOX FIFO.
   *
   *
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to #watermark samples. (#watermark/3) * (Acc + Gyro + Pressure)
   */
  lsm6dsox_fifo_watermark_set(&ag_ctx, watermark);

  /* Set FIFO mode to FIFO mode. */
  lsm6dsox_fifo_mode_set(&ag_ctx, LSM6DSOX_FIFO_MODE);

  /* Enable latched interrupt notification. */
  lsm6dsox_int_notification_set(&ag_ctx, LSM6DSOX_ALL_INT_LATCHED);

  /*
   * FIFO watermark interrupt routed on INT1 pin.
   * Remember that INT1 pin is used by sensor to switch in I3C mode.
   */
  lsm6dsox_pin_int1_route_get(&ag_ctx, &int1_route);
  int1_route.int1_ctrl.int1_fifo_th = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&ag_ctx, &int1_route);

  /*
   * Enable FIFO batching of Slave0.
   * ODR batching is 13 Hz.
   */
  lsm6dsox_sh_batch_slave_0_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dsox_sh_data_rate_set(&ag_ctx, LSM6DSOX_SH_ODR_13Hz);

  /* Set FIFO batch XL/Gyro ODR to 12.5 [Hz]. */
  lsm6dsox_fifo_xl_batch_set(&ag_ctx, LSM6DSOX_XL_BATCHED_AT_12Hz5);
  lsm6dsox_fifo_gy_batch_set(&ag_ctx, LSM6DSOX_GY_BATCHED_AT_12Hz5);

  /*
   * Prepare sensor hub to read data from external Slave0 continuously
   * in order to store data in FIFO.
   */
  sh_cfg_read.slv_add = (LPS22HH_I2C_ADD_H & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = LPS22HH_STATUS;
  sh_cfg_read.slv_len = 6;
  lsm6dsox_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
  /* Configure Sensor Hub to read one slave. */
  lsm6dsox_sh_slave_connected_set(&ag_ctx, LSM6DSOX_SLV_0);
  /* Enable I2C Master. */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /* Configure LSM6DSOX. */
  lsm6dsox_xl_full_scale_set(&ag_ctx, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&ag_ctx, LSM6DSOX_2000dps);
  lsm6dsox_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_12Hz5);
  lsm6dsox_gy_data_rate_set(&ag_ctx, LSM6DSOX_GY_ODR_12Hz5);

  while(1) {
    uint16_t num = 0, test = 0;
    uint8_t accID = 0, pressID = 0;
    lsm6dsox_fifo_tag_t reg_tag;

    /* Read watermark flag. */
    lsm6dsox_fifo_wtm_flag_get(&ag_ctx, &wtm_flag);
    
    if ( wtm_flag ) {
      /* Read number of samples in FIFO. */
      lsm6dsox_fifo_data_level_get(&ag_ctx, &num);
      
      while(num--) {
        lsm6dsox_fifo_data_level_get(&ag_ctx, &test);
          
        /* Read FIFO tag. */
        lsm6dsox_fifo_sensor_tag_get(&ag_ctx, &reg_tag);
        switch(reg_tag)
        {
          case LSM6DSOX_XL_NC_TAG:
            memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dsox_fifo_out_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
            if (accID >= (watermark/3)) { break; }
            for (uint8_t k = 0; k < 3; ++k) {
              record[accID].acc[k] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[k]);
            }
            ++accID;
            break;

          case LSM6DSOX_GYRO_NC_TAG:
            memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dsox_fifo_out_raw_get(&ag_ctx, data_raw_angular_rate.u8bit);
            break;

          case LSM6DSOX_SENSORHUB_SLAVE0_TAG:
            memset(data_raw_press_temp.u8bit, 0x00, sizeof(p_and_t_byte_t));
            lsm6dsox_fifo_out_raw_get(&ag_ctx, data_raw_press_temp.u8bit);

            data_raw_press_temp.u8bit[0] = 0x00; /* remove status register */
            if (pressID >= (watermark/3)) { break; }
            record[pressID].press = lps22hh_from_lsb_to_hpa( data_raw_press_temp.p_and_t.u32bit);
            ++pressID;
            break;

          default:
          /* Flush unused samples. */
            memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dsox_fifo_out_raw_get(&ag_ctx, dummy.u8bit);
            break;
        }
      }
      uint8_t nRecords = accID > pressID ? pressID : accID;
      for (uint8_t k = 0; k < nRecords; ++k) {
        record_t * cur = &record[k];
        
        output.fifo[k].acc[0] = cur->acc[0];
        output.fifo[k].acc[1] = cur->acc[1];
        output.fifo[k].acc[2] = cur->acc[2];
        output.fifo[k].press = cur->press;

        records_mean.acc[0] += cur->acc[0];
        records_mean.acc[1] += cur->acc[1];
        records_mean.acc[2] += cur->acc[2];
        records_mean.press += cur->press;
      }
        
        records_mean.acc[0] /= nRecords;
        records_mean.acc[1] /= nRecords;
        records_mean.acc[2] /= nRecords;
        records_mean.press /= nRecords;
               
        output.mean = records_mean;
        return output;
    }
  }
}

/**
 * @brief  Enter sleep mode and wait for interrupt
 * @param  None
 * @retval None
 */
static void Sleep_Mode(void)
{
  snprintf(dataOut, MAX_BUF_SIZE, "\r\n \r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; /* Systick IRQ OFF */
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; /* Systick IRQ ON */
}

/**
 * @brief  TIM_ALGO init function
 * @param  None
 * @retval None
 * @details This function intializes the Timer used to synchronize the algorithm
 */
static void MX_TIM_ALGO_Init(void)
{
#if (defined (USE_STM32F4XX_NUCLEO))
#define CPU_CLOCK  84000000U

#elif (defined (USE_STM32L0XX_NUCLEO))
#define CPU_CLOCK  32000000U

#elif (defined (USE_STM32L1XX_NUCLEO))
#define CPU_CLOCK  32000000U

#elif (defined (USE_STM32L4XX_NUCLEO))
#define CPU_CLOCK  80000000U

#else
#error Not supported platform
#endif
}

/**
 * @brief  Initializes USART2 MSP.
 * @param  huart USART2 handle
 * @retval None
 */

static void USART2_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable Peripheral clock */
    __HAL_RCC_USART2_CLK_ENABLE();
 
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief  Configures COM port.
 * @param  COM: COM port to be configured.
 *              This parameter can be COM1
 * @param  UART_Init: Pointer to a UART_HandleTypeDef structure that contains the
 *                    configuration information for the specified USART peripheral.
 * @retval BSP error code
 */
int32_t BSP_COM_Init(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;
 
  if(COM > COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
     hcom_uart[COM].Instance = COM_USART[COM];
#if (USE_HAL_UART_REGISTER_CALLBACKS == 0)
    /* Init the UART Msp */
    USART2_MspInit(&hcom_uart[COM]);
#else
    if(IsUsart2MspCbValid == 0U)
    {
      if(BSP_COM_RegisterDefaultMspCallbacks(COM) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_MSP_FAILURE;
      }
    }
#endif
  }

  return ret;
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_KEY
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability  
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef GPIO_InitStruct;
 
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
 
  if(ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }
 
  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

void init(void)
{
  uint8_t tmp;
  HAL_Init();
 
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize UART */
  USARTConfig();
 
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM_ALGO_Init();
 
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);
    
  snprintf(dataOut, MAX_BUF_SIZE, "\r\n------ LSM6DSOX Sensor Hub DEMO ------\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  lsm6dsox_hub_fifo_lps22hh();

  /* Wait for USER BUTTON push */
  Sleep_Mode();
 
  snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress USER button to start the DEMO ...\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
 
  I2C_read(0x0F, &tmp, 1);
}

/*
 * @brief  Write lps22hh device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dsox_write_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t ret;
  uint8_t drdy;
  lsm6dsox_status_master_t master_status;
  lsm6dsox_sh_cfg_write_t sh_cfg_write;

  /* Configure Sensor Hub to read LPS22HH. */
  sh_cfg_write.slv0_add = (LPS22HH_I2C_ADD_H & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = lsm6dsox_sh_cfg_write(&ag_ctx, &sh_cfg_write);

  /* Disable accelerometer. */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);

  /* Enable I2C Master. */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_104Hz);

  /* Wait Sensor Hub operation flag set. */
  lsm6dsox_acceleration_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
  do
  {
    HAL_Delay(20);
        lsm6dsox_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do
  {
    HAL_Delay(20);
    lsm6dsox_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);

  return ret;
}

/*
 * @brief  Read lps22hh device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dsox_read_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  lsm6dsox_sh_cfg_read_t sh_cfg_read;
  axis3bit16_t data_raw_acceleration;
  int32_t ret;
  uint8_t drdy;
  lsm6dsox_status_master_t master_status;

  /* Disable accelerometer. */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);

  /* Configure Sensor Hub to read LPS22HH. */
  sh_cfg_read.slv_add = (LPS22HH_I2C_ADD_H & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dsox_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
  lsm6dsox_sh_slave_connected_set(&ag_ctx, LSM6DSOX_SLV_0);

  /* Enable I2C Master and I2C master. */
   lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_104Hz);

  /* Wait Sensor Hub operation flag set. */
  lsm6dsox_acceleration_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
  do {
    HAL_Delay(20);
  lsm6dsox_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do {
    //HAL_Delay(20);
    lsm6dsox_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);

  /* Read SensorHub registers. */
  lsm6dsox_sh_read_data_raw_get(&ag_ctx,(lsm6dsox_emb_sh_read_t*)data);

  return ret;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_H, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
      HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_H, Reg,
                       I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  return 0;
}

//float get_split_point(float m1, float m2, float o1, float o2)
//{
//    float tmp;
//    if (m1 > m2) // swap
//    {
//      tmp = m1;
//      m1 = m2;
//      m2 = tmp;
//      
//      tmp = o1; // swap stds
//      o1 = o2;
//      o2 = tmp;
//    }
//    // d = m2 - m1, alfa = o1 / (o1 + o2), split = m1 + alfa * d;
//    
//    float dist = m2 - m1;
//    tmp = o1 + o2;
//    if (tmp < 1e-7) {
//      tmp = 0.5f;
//    }
//    else {
//      tmp = o1 / tmp;
//    }
//    
//    return m1 + tmp * dist;
//}

static int count_valid_pts(const node_t * node)
{
  int res = 0;
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    if (is_used(&(node->idx), i))
    {
      ++res;
    }
  }
  
  return res;
}

typedef struct pointDim
{
  float pt;
  int dim;
} pointDim;

// returns the best split point and dim for given data
pointDim get_best_split(const node_t * node)
{
  int nValid = count_valid_pts(node);
  int k = sqrt(nValid); // floor
  features_t mean, std;
  compute_mean(node,&mean, 255);  
  compute_std(node, &mean, &std, 255);
  
  pointDim best = {0, -1};
  float giniBest = 1.0f, split_point;
  
  for (int i = 0; i < NUM_FEATURES; ++i)
  {
    for (int j = 1; j <= k; ++j)
    {
      split_point = mean.data[i] + std.data[i] * getInverseCDFValue(j /(k + 1));
      float gini = compute_gini(node, split_point, i);
      if (gini < giniBest)
      {
        giniBest = gini;
        best.pt = split_point;
        best.dim = i;
      }
    }
  }
  
  return best; // will return dim == -1 in case of failure
}

float dim_data(const record_t * data, int dim)
{
  switch(dim)
  {
  case 0:
  case 1:
  case 2:
    return data->acc[dim];
  case 3:
    return data->press;
  }
  return -1; // shouldnt happen
}

static float square(float val)
{
  return val * val;
}

static int sum4(const int * array)
{
  int res = 0;
  for (int i = 0; i < 4; ++i)
  {
    res += array[i];
  }
  
  return res;
}

float compute_gini(const node_t * node, float split_point, int dim) // feature dim
{
  int gte[4];
  int lt[4];
  memset(gte, 0, sizeof(gte));
  memset(lt, 0, sizeof(lt));

  for (int i = 0; i < NUM_CALIB; ++i)
  {
    if (!is_used(&(node->idx), i))
    {continue;}
    
    // 0 means moving down horizontally, 1 down vertically, 2 means up horizontally, 3 means up vertically 
    if (node->data[i].features.data[dim] < split_point)
    {
      ++lt[node->data[i].label];
    }
    else
    {
      ++gte[node->data[i].label];
    }
  }
  
  float res = 0;
  for (int i = 0; i < 4; ++i)
  {
    float n_in_class = lt[i] + gte[i];
    if (!n_in_class) { continue; }
    float gini = 1 - square(lt[i] / n_in_class) - square(gte[i] / n_in_class);
    res += gini * n_in_class;
  }
  
  int nValid = sum4(lt) + sum4(gte);
  if (nValid == 0)
  {  
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nError 0 samples in giniIndex...\r\n");
    HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  }
 
  return res / nValid;
}

void set_label(node_t * node)
{
  // 0 means moving down horizontally, 1 down vertically, 2 means up horizontally, 3 means up vertically 
  int array[4] = {0, 0, 0, 0};
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    if (is_used(&(node->idx), i))
    {
      ++array[node->data[i].label];
    }
  }
  // set label
  int max_val = 0;
  float sum = 0;
  for (int i = 0; i < 4; i++)
  {
    sum += array[i];
    if (array[i] > max_val)
    {
      max_val = array[i];
      node->label = i;
    }
    node->cnt[i] = array[i];
  }
  
  node->error = (sum ? (sum - max_val) / sum : -1);
}

void split_node(node_t * node)
{  
  pointDim ptDim = get_best_split(node); // split point and dim of the best split
  
  node->dim = ptDim.dim;
  node->split = ptDim.pt;
  node->son = node->daughter = NULL;
  
  // left one
  node_t * son = (node_t *)malloc(sizeof(node_t));
  son->daughter = son->son = NULL;
  son->split = 0;
  son->dim = -1;
  son->data = node->data;
  memcpy((void *) (&son->idx), (const void *) (&node->idx), sizeof(node->idx));
  
  uint8_t cnt = 0;
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    if (son->data[i].features.data[node->dim] >= node->split)
    {
      unset_index(&(son->idx), i);
    }
    else if(is_used((&son->idx), i))
    {
      ++cnt;
    }
  }
  
  if (cnt > 0)
  {
    node->son = son;
    set_label(son);
  }
  else 
  {
    free(son);
    son = NULL;
  }
  
  node_t * daughter = (node_t *)malloc(sizeof(node_t));
  daughter->daughter = daughter->son = NULL;
  daughter->split = 0;
  daughter->dim = 0;
  daughter->data = node->data;
  memcpy((void *)(&daughter->idx), (const void *)(&node->idx), sizeof(node->idx));
  
  cnt = 0;
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    if (daughter->data[i].features.data[node->dim] < node->split)
    {
      unset_index(&(daughter->idx), i);
    }
    else if(is_used((&daughter->idx), i))
    {
      ++cnt;
    }
  }
  if (cnt > 0)
  {
    node->daughter = daughter;
    set_label(daughter);
  }
  else 
  {
    free(daughter);
    daughter = NULL;
  }
    
   if (node->son != NULL && node->son->error > 0)
   {
     split_node(node->son);
   }
   
   if (node->daughter != NULL && node->daughter->error > 0)
   {
     split_node(node->daughter);
   }
}

node_t * dec_tree_generator(labeled_record data[] /* nRecords is NUM_CALIB */)
{
  node_t * head = (node_t *) malloc(sizeof(node_t));
  memset(head, 0, sizeof(node_t));
  head->data = data;
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    set_index(&(head->idx), i);
  }
  
  set_label(head);
  
  split_node(head);
  
  return head;
}

const char * dim2text[4] = { "ACC_X",
                             "ACC_Y",
                             "ACC_Z",
                             "PRESSURE" };

int sumator(const node_t * node)
{
  int sum = 0;
  for (int i = 0; i < 4; i++)
      sum += node->cnt[i];
  return sum;
}

void print_node_WEKA_J48(const node_t * node, int depth)
{  
  uint8_t isLeaf = (node->son == node->daughter); //(node->son == NULL) && (node->daughter == NULL);
  int length = 0;
  memset(dataOut, 0, sizeof(dataOut));
  
  if (isLeaf)
  {    
      int nRecords = sumator(node);
      static const char *classified[] = {"down horizontally","down vertically","up horizontally","up vertically" };
      char text[20];
      
      for (int i = 0; i < 4; i++)
      {
        if (node->label == i)
          strcpy(text, classified[i]);
      }
      
      // 0 means moving down horizontally, 1 down vertically, 2 means up horizontally, 3 means up vertically 
      snprintf(dataOut, MAX_BUF_SIZE, ": %s (%d.0", text, nRecords);
      length = strlen(dataOut);
      
      if (node->error != 0)
      {
        snprintf(dataOut + length, MAX_BUF_SIZE - length, "/%d.0)", (int)node->error * nRecords);
      }
      else
      {
        snprintf(dataOut + length, MAX_BUF_SIZE - length, ")");
      }
      HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
      return;
  }
  
  // process left son first
  if (node->son != NULL)
  {
    dataOut[length++] = '\r';
    dataOut[length++] = '\n';
    // indent first
    for (int i = 0; i < depth; ++i)
    {
      dataOut[length++] = '|';
      dataOut[length++] = '\t';
    }
    snprintf(dataOut + length, MAX_BUF_SIZE - length, "%s < %.03f", dim2text[node->dim], node->split);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
    
    print_node_WEKA_J48(node->son, depth + 1);
  }
  
  // process the right one
  if (node->daughter != NULL)
  {
    length = 0;
    memset(dataOut, 0, sizeof(dataOut));
    dataOut[length++] = '\r';
    dataOut[length++] = '\n';
    // indent first
    for (int i = 0; i < depth; ++i)
    {
      dataOut[length++] = '|';
      dataOut[length++] = '\t';
    }
    snprintf(dataOut + length, MAX_BUF_SIZE - length, "%s >= %.03f", dim2text[node->dim], node->split);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
    
    print_node_WEKA_J48(node->daughter, depth + 1);
  }
}

void calculateFeatures(const fifo_record_t * data, features_t * features)
{
  record_t data_mean = data->mean;
  record_t data_min = compute_min(data->fifo);
  record_t data_max = compute_max(data->fifo);
  record_t data_peak = compute_peak_to_peak(&data_max, &data_min);
  
  int numFeatPerAxes = 4;
  for (int k = 0; k < numFeatPerAxes; ++k) // feature iteration
  {
    record_t * cur = NULL;
    switch(k)
    {
    case 0:
      cur = &data_mean;
      break;
    case 1:
      cur = &data_min;
      break;
    case 2:
      cur = &data_max;
      break;
    case 3:
      cur = &data_peak;
      break;
    }
    for (int i = 0; i < 4; ++i) // axis iteration
    {
      features->data[k * numFeatPerAxes + i] = dim_data(cur, i);
    }
  }
}

static const char * classified[] = {"down horizontally","down vertically","up horizontally","up vertically" };

int main(void)
{
  init();
  labeled_record data[NUM_CALIB];
  memset(data, 0, sizeof(data)); // set default labels to down (0)
  
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    /* _NOTE_: Pushing button creates interrupt/event and wakes up MCU from sleep mode */
    fifo_record_t rec = read_it_my_boy();
    data[i].label = getUserInput(&UartHandle, "For down horizontally type '0', for down vertically type '1'. For up horizontally type '2', for up vertically type '3'.", "0123");
    calculateFeatures(&rec, &data[i].features);
    
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nClassified: %s \r\n", classified[data[i].label]);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
    Sleep_Mode();
  }
  
  node_t * head = dec_tree_generator(data);      
  print_node_WEKA_J48(head, 0);
  
  Sleep_Mode();
  
  // enable MLC & set ODR to 52 [Hz]
  lsm6dsox_mlc_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dsox_mlc_data_rate_set(&ag_ctx, LSM6DSOX_ODR_PRGS_12Hz5);
  
  fifo_record_t data_out;
  int reg2 = 0;
  
  // set MLC status interrupt on pin2
  lsm6dsox_pin_int2_route_t int2_route;
  lsm6dsox_pin_int2_route_get(&ag_ctx, &int2_route);
  int2_route.mlc_int2.int2_mlc1 = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&ag_ctx, &int2_route);
      
  /* Infinite loop */
  while (1)
  {
    // get MLC status register
    lsm6dsox_mlc_status_mainpage_t status;
    int reg1 = lsm6dsox_mlc_status_get(&ag_ctx,&status);
    reg1 = status.is_mlc1;
      
    if (reg2 != reg1 && reg1 != 0)
    {
      snprintf(dataOut, MAX_BUF_SIZE, "\r\n MLC status register: %d", status.is_mlc1);
      HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
    }
    
    if (MLC_interrupt)
    {
      snprintf(dataOut, MAX_BUF_SIZE, "\r\n Change of position.");
      HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
      MLC_interrupt = 0;
    }
    
    reg2 = reg1;
    
    if (button_pressed)
    {        
      data_out = read_it_my_boy();
              
      for (int i = 0; i < NUM_RECORDS; i++)
      {
        snprintf(dataOut, MAX_BUF_SIZE, "\r\n%.3f, %.3f, %.3f, %.3f", data_out.fifo[i].acc[0], data_out.fifo[i].acc[1], data_out.fifo[i].acc[2], data_out.fifo[i].press);
        HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
      }
      
      snprintf(dataOut, MAX_BUF_SIZE, "\r\n \r\n");
      HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
      
      /* Reset FIFO by setting FIFO mode to Bypass */
      lsm6dsox_fifo_mode_set(&ag_ctx, LSM6DSOX_BYPASS_MODE);
      
      button_pressed = 0;
      }
   }
 }

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  Get the current tick value in millisecond
 * @param  None
 * @retval The tick value
 */
uint32_t user_currentTimeGetTick(void)
{
  return HAL_GetTick();
}

/**
 * @brief  Get the delta tick value in millisecond from Tick1 to the current tick
 * @param  Tick1 the reference tick used to compute the delta
 * @retval The delta tick value
 */
uint32_t user_currentTimeGetElapsedMS(uint32_t Tick1)
{
  volatile uint32_t Delta, Tick2;

  Tick2 = HAL_GetTick();

  /* Capture computation */
  Delta = Tick2 - Tick1;
  return Delta;
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin the pin connected to EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT2_master_Pin)
  {
    MLC_interrupt = 1;
  }
  
  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    /* Manage software debouncing*/
    int doOperation = 0;

    if (Int_Current_Time1 == 0 && Int_Current_Time2 == 0)
    {
      Int_Current_Time1 = user_currentTimeGetTick();
      doOperation = 1;
    }
    else
    {
      int i2;
      Int_Current_Time2 = user_currentTimeGetTick();
      i2 = Int_Current_Time2;

      /* If button interrupt after more than 300 ms is received -> get it, otherwise -> discard */
      if ((i2 - Int_Current_Time1)  > 300)
      {
        Int_Current_Time1 = Int_Current_Time2;
        doOperation = 1;
      }
    }

    if (doOperation)
    {
      button_pressed = 1;
    }
  }
  else
  {
    Error_Handler();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
