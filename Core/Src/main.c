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
#include <stdio.h>
#include <string.h>
#include "C:\IKS0LAB_SensorHub\Drivers\lps22hh\lps22hh.h"
#include "C:\IKS0LAB_SensorHub\Drivers\lps22hh\lps22hh_reg.h"
#include "C:\IKS0LAB_SensorHub\Drivers\lsm6dsox\lsm6dsox.h"
#include "C:\IKS0LAB_SensorHub\Drivers\lsm6dsox\lsm6dsox_reg.h"

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
#define NUM_CALIB                30
#define NUM_FEATURES             16
#define TESTING                  1

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
  int8_t label;
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
void compute_mean(const node_t * node, features_t * features, int8_t label);
void compute_std(const node_t * node, const features_t * mean, features_t * features, int8_t label);
float compute_gini(const node_t * node, float split_point, int dim);
float dim_data(const record_t * data, int dim);
float compute_entropy(const node_t * node, float split_point, int dim);
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

// use label -1 for computing means of all axes
void compute_mean(const node_t * node, features_t * features, int8_t label)
{
  memset(features, 0, sizeof(features_t));

  int num_rec = 0;
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    if ((label != -1 && node->data[i].label != label) || !is_used(&(node->idx), i))
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

// use label -! for computing means of all axes
void compute_std(const node_t * node, const features_t * mean, features_t * features, int8_t label)
{
  memset(features, 0, sizeof(features_t));

  int num_rec = 0;
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    if ((label != -1 && node->data[i].label != label) || !is_used(&(node->idx), i))
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
  return;
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
  compute_mean(node,&mean, -1);
  compute_std(node, &mean, &std, -1);

  pointDim best = {0, -1};
  //float  = 1.0f, split_point = 0;
// max entropy of a system should be log_2(num_classes);
  float entropyLowest = 200, split_point = 0;

  for (int i = 0; i < NUM_FEATURES; ++i)
  {
    for (int j = 1; j <= k; ++j)
    {
      split_point = mean.data[i] + std.data[i] * getInverseCDFValue((float) j / (k + 1));
    float error = compute_gini(node, split_point, i);
     float entropy = compute_entropy(node, split_point, i);
      if (error < entropyLowest)//entropy < entropyLowest)
      {
        entropyLowest = error;
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

float compute_entropy(const node_t * node, float split_point, int dim) // feature dim
{
    int labels[4];
    memset(labels, 0, sizeof(labels));
    
    for (int i = 0; i < NUM_CALIB; ++i)
    {
      if (!is_used(&(node->idx), i))
      {continue;}
        
      ++labels[node->data[i].label];
    }
    
    // https://towardsdatascience.com/connections-log-likelihood-cross-entropy-kl-divergence-logistic-regression-and-neural-networks-40043dfb6200
    float res = 0;
    float nRecords = sum4(labels);
    for (int i = 0; i < 4; ++i)
    {
        if (labels[i])
        {
            float px = labels[i] / nRecords;
            res -= px * log2f(px);
        }
    }
    
    return res;
}

int argMax4(int * array)
{
    int id = 0, val = array[0];
    for (int i = 1; i < 4; ++i)
    {
        if (array[i] > val)
        {
            id = i;
            val = array[i];
        }
    }
    return id;
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
  
  int ltLabel = argMax4(lt), gteLabel = argMax4(gte);
  float ltSum = sum4(lt), gteSum = sum4(gte);
  float res = 0;
  if (ltSum)
  {
      res += (ltSum - lt[ltLabel]) / ltSum * ltSum / (ltSum + gteSum);
  }
  if (gteSum)
  {
    res += (gteSum - gte[gteLabel]) / gteSum * gteSum / (ltSum + gteSum);
  }
  
  return res;
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
  
  int cnt2 = 0;
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    if (daughter->data[i].features.data[node->dim] < node->split)
    {
      unset_index(&(daughter->idx), i);
    }
    else if(is_used((&daughter->idx), i))
    {
      ++cnt2;
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
    
   if (node->son != NULL && node->son->error > 0 && cnt > 2)
   {
     split_node(node->son);
   }
   
   if (node->daughter != NULL && node->daughter->error > 0 && cnt2 > 2)
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

const char * dim2text[16] = {
    "MEAN_X", "MEAN_Y", "MEAN_Z", "MEAN_PRESSURE",
    "MAX_X", "MAX_Y", "MAX_Z", "MAX_PRESSURE",
    "MIN_X", "MIN_Y", "MIN_Z", "MIN_PRESSURE",
    "PEAK_X", "PEAK_Y", "PEAK_Z", "PEAK_PRESSURE"
};

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
      
      // 0 means moving down horizontally, 1 down vertically, 2 means up horizontally, 3 means up vertically 
      snprintf(dataOut, MAX_BUF_SIZE, ": %s (%d.0", classified[node->label], nRecords);
      length = strlen(dataOut);
      
      if (node->error != 0)
      {
        snprintf(dataOut + length, MAX_BUF_SIZE - length, "/%d.0)", (int)(node->error * nRecords));
      }
      else
      {
        snprintf(dataOut + length, MAX_BUF_SIZE - length, ")");
      }
      printf("%s", dataOut);
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
      printf("%s", dataOut);
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
    printf("%s", dataOut);
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

    static const float test_data[48][16] = {269.010, 202.874, 977.952, 988.102, 896.700, 382.592, 1165.954, 988.148, -218.014, 19.215, 831.735, 988.072, 1114.714, 363.377, 334.219, 0.077,
    179.407, 585.460, 887.391, 988.172, 642.452, 854.793, 959.835, 988.201, -320.372, 168.360, 671.732, 988.124, 962.824, 686.433, 288.103, 0.077,
    261.171, 427.708, 936.289, 988.120, 630.557, 640.805, 1164.734, 988.144, -141.276, 259.311, 701.134, 988.095, 771.833, 381.494, 463.600, 0.050,
    175.631, 454.969, 930.153, 988.149, 655.933, 735.660, 1079.273, 988.185, -257.542, 174.826, 734.562, 988.111, 913.475, 560.834, 344.711, 0.075,
    293.544, 366.244, 926.102, 988.071, 862.540, 598.105, 1093.303, 988.090, -25.925, 168.055, 716.506, 988.047, 888.465, 430.050, 376.797, 0.042,
    288.384, 382.391, 905.783, 988.089, 716.811, 551.074, 1027.301, 988.134, -128.832, 68.686, 708.210, 988.054, 845.643, 482.388, 319.091, 0.079,
    397.903, 796.440, -506.306, 987.875, 875.594, 1043.588, -231.373, 987.933, 96.807, 561.017, -874.862, 987.846, 778.787, 482.571, 643.489, 0.087,
    557.796, 525.338, -769.448, 987.811, 689.117, 724.558, -524.722, 987.902, 233.386, 278.160, -1071.648, 987.711, 455.731, 446.398, 546.926, 0.191,
    349.518, 238.791, -961.098, 987.481, 996.801, 592.798, -787.937, 987.510, -83.204, -61.915, -1084.580, 987.453, 1080.005, 654.713, 296.643, 0.057,
    62.861, 1043.466, -71.693, 987.371, 189.649, 1219.329, 191.296, 987.391, -129.015, 842.593, -214.354, 987.358, 318.664, 376.736, 405.650, 0.033,
    255.120, 1049.468, -31.293, 987.449, 517.402, 1180.228, 74.359, 987.470, 2.928, 851.438, -158.173, 987.402, 514.474, 328.790, 232.532, 0.067,
    363.475, 206.210, -950.020, 987.292, 774.761, 488.732, -787.266, 987.315, -36.661, -160.430, -1081.835, 987.266, 811.422, 649.162, 294.569, 0.049,
    201.032, 399.550, 810.287, 987.368, 396.805, 935.191, 1221.342, 987.405, -107.604, -287.981, 214.964, 987.321, 504.409, 1223.172, 1006.378, 0.084,
    241.054, 530.279, 826.837, 987.368, 589.443, 996.862, 1128.500, 987.438, -12.444, -13.481, 579.317, 987.311, 601.887, 1010.343, 549.183, 0.127,
    855.183, 289.128, -203.557, 987.305, 1255.929, 923.174, 201.910, 987.379, 380.457, -213.683, -508.679, 987.230, 875.472, 1136.857, 710.589, 0.148,
    -660.002, 724.125, 184.781, 987.189, -436.882, 1297.348, 481.595, 987.220, -844.301, 107.482, 13.847, 987.150, 407.419, 1189.866, 467.748, 0.071,
    129.204, 688.007, 544.754, 987.313, 342.881, 1388.848, 931.958, 987.364, -87.047, -61.610, 247.782, 987.260, 429.928, 1450.458, 684.176, 0.104,
    314.113, 255.474, -1005.884, 987.202, 451.278, 508.130, -794.525, 987.242, 215.391, -149.633, -1283.440, 987.153, 235.887, 657.763, 488.915, 0.089,
    215.110, -332.017, -761.908, 987.141, 398.574, 71.431, -573.156, 987.184, 111.142, -540.094, -1003.877, 987.106, 287.432, 611.525, 430.721, 0.078,
    241.578, 290.982, -1002.328, 987.069, 374.540, 444.385, -842.715, 987.095, 128.161, -65.331, -1230.736, 987.044, 246.379, 509.716, 388.021, 0.051,
    919.709, 155.885, 78.989, 987.119, 1410.991, 691.130, 318.664, 987.150, 410.835, -298.839, -67.344, 987.075, 1000.156, 989.969, 386.008, 0.075,
    43.456, 555.387, 842.678, 987.062, 238.022, 952.698, 1030.229, 987.113, -153.842, -62.647, 581.025, 987.016, 391.864, 1015.345, 449.204, 0.097,
    68.985, 419.290, 827.422, 987.156, 250.039, 1120.021, 1166.015, 987.205, -134.322, -195.261, 406.565, 987.118, 384.361, 1315.282, 759.450, 0.087,
    125.623, 596.153, 810.958, 987.094, 315.797, 988.200, 918.355, 987.163, -19.459, 34.770, 634.461, 987.044, 335.256, 953.430, 283.894, 0.119,
    -326.381, -36.472, 899.799, 987.156, 102.907, 131.394, 1094.889, 987.169, -714.432, -226.615, 676.246, 987.141, 817.339, 358.009, 418.643, 0.028,
    51.472, -172.636, 988.676, 987.160, 311.527, 204.228, 1214.998, 987.183, -480.009, -360.937, 864.797, 987.143, 791.536, 565.165, 350.201, 0.040,
    -317.920, 28.017, 966.289, 987.279, 295.972, 290.055, 1086.959, 987.307, -631.533, -444.080, 840.336, 987.247, 927.505, 734.135, 246.623, 0.060,
    -55.254, -204.082, 963.983, 987.267, 197.884, 126.575, 1046.089, 987.291, -323.239, -447.252, 803.614, 987.238, 521.123, 573.827, 242.475, 0.053,
    1.488, -38.259, 1015.327, 987.059, 4.209, -36.051, 1015.833, 987.071, 0.061, -40.748, 1014.796, 987.047, 4.148, 4.697, 1.037, 0.024,
    -92.988, 209.212, 921.484, 987.104, 628.971, 453.657, 1091.412, 987.148, -738.039, -195.749, 805.993, 987.064, 1367.010, 649.406, 285.419, 0.084,
    -251.174, 434.113, -626.580, 987.455, 257.420, 1131.672, -84.241, 987.521, -768.051, -112.301, -1183.278, 987.381, 1025.471, 1243.973, 1099.037, 0.140,
    133.773, -402.289, 843.380, 987.263, 699.426, -114.802, 1090.741, 987.285, -504.592, -630.801, 656.665, 987.238, 1204.018, 515.999, 434.076, 0.047,
    -927.999, -16.135, -345.004, 987.446, -785.985, 13.664, 59.597, 987.507, -1010.038, -75.030, -640.988, 987.405, 224.053, 88.694, 700.585, 0.101,
    963.526, 31.049, 172.581, 987.376, 1078.236, 155.367, 614.514, 987.420, 839.299, -83.692, -130.540, 987.351, 238.937, 239.059, 745.054, 0.069,
    824.537, 70.095, 569.221, 987.447, 986.248, 135.481, 759.084, 987.515, 732.244, -0.976, 391.986, 987.375, 254.004, 136.457, 367.098, 0.139,
    -879.144, -220.466, 206.717, 987.353, -690.215, 38.674, 454.572, 987.413, -973.194, -370.697, -112.179, 987.302, 282.979, 409.371, 566.751, 0.111,
    -213.189, -246.885, 748.415, 987.689, -105.408, -28.731, 1094.889, 987.762, -298.534, -477.630, 493.429, 987.632, 193.126, 448.899, 601.460, 0.130,
    -6.692, 181.707, 1199.730, 987.684, 56.730, 308.233, 1394.765, 987.729, -119.987, 105.713, 849.669, 987.642, 176.717, 202.520, 545.096, 0.088,
    -244.787, -201.233, 783.905, 987.604, -151.402, 24.583, 1093.791, 987.651, -319.518, -487.512, 466.955, 987.571, 168.116, 512.095, 626.836, 0.079,
    -160.137, 173.826, 1070.465, 987.655, 62.342, 502.762, 1653.649, 987.708, -370.392, -68.137, 621.712, 987.609, 432.734, 570.899, 1031.937, 0.098,
    636.053, -330.711, 81.917, 987.620, 1200.602, -48.190, 522.221, 987.648, 263.764, -614.148, -752.801, 987.579, 936.838, 565.958, 1275.022, 0.069,
    -932.519, 145.162, -343.351, 987.583, -376.675, 402.661, -153.903, 987.598, -1441.613, -330.010, -504.043, 987.554, 1064.938, 732.671, 350.140, 0.044,
    521.184, -555.710, 44.573, 987.595, 977.586, -252.296, 518.866, 987.628, 279.990, -769.820, -591.517, 987.544, 697.596, 517.524, 1110.383, 0.083,
    -0.494, -357.936, -1011.209, 987.600, 157.868, -213.866, -610.305, 987.632, -136.823, -561.871, -1285.453, 987.563, 294.691, 348.005, 675.148, 0.070,
    -590.242, -203.526, 671.244, 987.683, -520.269, 52.948, 751.947, 987.710, -660.142, -296.277, 625.616, 987.644, 139.873, 349.225, 126.331, 0.066,
    -33.818, -73.279, -1115.775, 987.685, 140.361, 42.151, -803.126, 987.734, -183.732, -284.809, -1448.933, 987.635, 324.093, 326.960, 645.807, 0.099,
    784.582, -241.035, 240.907, 987.625, 1457.107, 170.434, 389.546, 987.650, 447.679, -623.298, 99.125, 987.587, 1009.428, 793.732, 290.421, 0.063,
    -1013.930, 65.270, 12.273, 987.782, -508.191, 230.763, 124.257, 987.803, -1510.116, -256.627, -120.170, 987.764, 1001.925, 487.390, 244.427, 0.039};
//first 12 down_hor
//second 12 down_ver
//third 12 up_hor
//fourth 12 up_ver

void load_features(labeled_record * data)
{
    for (int i = 0; i < NUM_CALIB; ++i)
    {
        float * feat_data = data[i].features.data;
        memcpy(feat_data, test_data[i], NUM_FEATURES * sizeof(float));
        data[i].label = i / 12;
    }
}

int main(void)
{
  init();
  labeled_record data[NUM_CALIB];// = malloc(NUM_CALIB * sizeof(labeled_record));
  memset(data, 0, sizeof(data)); // set default labels to down (0)
  
#ifdef TESTING
  load_features(data);
#else
  for (int i = 0; i < NUM_CALIB; ++i)
  {
    fifo_record_t output = read_it_my_boy();
    calculateFeatures(&output, &(data[i].features));
    /* _NOTE_: Pushing button creates interrupt/event and wakes up MCU from sleep mode */
    char tmp = getUserInput(&UartHandle, "\r\nFor down horizontally type '0', for down vertically type '1'. For up horizontally type '2', for up vertically type '3'.", "0123");
    data[i].label = tmp - '0';
    
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nClassified: %s \r\n", classified[data[i].label]);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
    Sleep_Mode();
  }
#endif // testing
  
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
        snprintf(dataOut, MAX_BUF_SIZE, "\r\nacc x: %.3f, acc y: %.3f, acc z: %.3f, press: %.3f", data_out.fifo[i].acc[0], data_out.fifo[i].acc[1], data_out.fifo[i].acc[2], data_out.fifo[i].press);
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
