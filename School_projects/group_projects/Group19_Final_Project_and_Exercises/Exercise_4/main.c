/**
 * Copyright (c) 2009 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>


#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf_drv_timer.h"
#include "nrf_temp.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "lsm6dso_reg.h"
#include "lsm6dso_platform.h"
#include "nrfx_twi.h"
#include "app_uart.h"
#include "nrf_uart.h"
#include "arm_math.h"
#include "DSP/fdacoefs.h"
#include "DSP/tmwtypes.h"


#define BUTTON_PIN 13
#define TWI_INSTANCE_ID 0
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256
#define NUM_TAPS 50

#define BOOT_TIME 10


// Defining variables

// Exercise 1
int32_t volatile temp;
const nrf_drv_timer_t TIMER_TEMP = NRF_DRV_TIMER_INSTANCE(0);

// Exercise 2
static uint8_t whoamI, rst;
const nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);
stmdev_ctx_t dev_ctx;

// Exercise 3
int16_t data_raw_acceleration[3];
static float_t acceleration_mg[3];

// Exercise 4
arm_fir_instance_q15 fir_lpf;
static q15_t firStateQ15[NUM_TAPS + 1];
q15_t data_raw_filtered;
float_t filtered_mg;


// Function for measuring temperature
void meas_temp()
{
  // Start the temperature measurement
  NRF_TEMP->TASKS_START = 1; 

  // Wait while temperature measurement is not finished
  while (NRF_TEMP->EVENTS_DATARDY == 0)
    {
    // Do nothing.
    }

  NRF_TEMP->EVENTS_DATARDY = 0;

  // The value in the register is a multiple of 4
  temp = (nrf_temp_read() / 4);

  // Stop the temperature measurement
  NRF_TEMP->TASKS_STOP = 1; 
}


void temp_timeout_handler(nrf_timer_event_t event_type, void* p_context)
{
  meas_temp();
}


void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  meas_temp();
}


uint32_t config_gpio()
{
  uint32_t err_code = NRF_SUCCESS;

  if(!nrf_drv_gpiote_is_init())
    {
      err_code = nrf_drv_gpiote_init();
    }

  // Set which clock edge triggers the interrupt
  nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  
  // Configure the internal pull up resistor
  config.pull = NRF_GPIO_PIN_PULLUP;

  // Configure the pin as input
  err_code = nrf_drv_gpiote_in_init(BUTTON_PIN, &config, button_handler);
  
  if (err_code != NRF_SUCCESS)
    {
      // Handle error condition
    }

  // Enable events
  nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
  
  return err_code;
}


uint32_t init_timers()
{
  // Time(in milliseconds) between consecutive compare events.
  uint32_t time_ms = 5000; 
  uint32_t time_ticks;
  uint32_t err_code = NRF_SUCCESS; 
  
  // Configure TIMER_TEMP for measuring the temperature
  nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
  err_code = nrf_drv_timer_init(&TIMER_TEMP, &timer_cfg, temp_timeout_handler);
  APP_ERROR_CHECK(err_code);

  time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_TEMP, time_ms);

  nrf_drv_timer_extended_compare(&TIMER_TEMP, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  nrf_drv_timer_enable(&TIMER_TEMP);

  return err_code;
}


// Initializing the sensor
void twi_init(void)
{
  nrfx_twi_config_t twi_config = NRFX_TWI_DEFAULT_CONFIG;
  twi_config.scl = 27;
  twi_config.sda = 26;
  twi_config.frequency = NRF_TWI_FREQ_400K;
  nrfx_twi_init(&m_twi, &twi_config, twi_handler, NULL);
  nrfx_twi_enable(&m_twi);
}

// Configuring the sensor
void lsm6dso_setup(void)
{
  
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay_ms;
  dev_ctx.handle = (void*)&m_twi; // pass TWI instance handle

   /* Check device ID */
  lsm6dso_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSO_ID)
  {
    while (1);
  }
  printf("Sensor ID: %#02x\n", whoamI);

   /* Disable I3C interface */
  lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz);
  /* Set full scale */
  lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
}


// Initializing uart error handler
void uart_error_handle(app_uart_evt_t * p_event)
{
  if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
  {
    APP_ERROR_HANDLER(p_event->data.error_communication);
  }
  else if (p_event->evt_type == APP_UART_FIFO_ERROR)
  {
    APP_ERROR_HANDLER(p_event->data.error_code);
  }
}


// Function for initializing uart
void uart_init(void)
{
  uint32_t err_code;
  const app_uart_comm_params_t comm_params =
  {
    .rx_pin_no = 8,
    .tx_pin_no = 6,
    .rts_pin_no = RTS_PIN_NUMBER,
    .cts_pin_no = CTS_PIN_NUMBER,
    .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
    .use_parity = false,
    .baud_rate = NRF_UART_BAUDRATE_115200
  };
  APP_UART_FIFO_INIT(&comm_params,
    UART_RX_BUF_SIZE,
    UART_TX_BUF_SIZE,
    uart_error_handle,
    APP_IRQ_PRIORITY_LOWEST,
    err_code);
  APP_ERROR_CHECK(err_code);
}


// Initializing the FIR filter instance
void dsp_init() 
{
  arm_fir_init_q15(&fir_lpf, NUM_TAPS, B, firStateQ15, 1);
}

// Function for filtering
q15_t fir_process_sample(q15_t in)
    {
      q15_t out;
      arm_fir_q15(&fir_lpf, &in, &out, 1);
      return out;
    }


// Extract acceleration data from sensor
void sensor_data_polling(void)
{
    uint8_t reg;
    /* Read output only if new xl value is available */
    lsm6dso_xl_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {

      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

      acceleration_mg[0] = lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] = lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] = lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);

      /* Convert uint16_t data to q15_t*/
      q15_t z_axis_acceleration = (q15_t)(acceleration_mg[2] / 1000.0f * 32767.0f);

      /* Filter data */
      data_raw_filtered = fir_process_sample(z_axis_acceleration);

      /* Convert q15_t data to uint16_t*/
      filtered_mg = (data_raw_filtered / 32768.0f) * 1000.0f;
     
      /* Print both unfiltered and filtered z-axis data*/
      printf("%.2f\t%.2f\r\n", acceleration_mg[2], filtered_mg); 
      
    }
  
}


int main(void)
{
  init_timers();
  config_gpio();
  twi_init();
  uart_init();
  dsp_init();
  lsm6dso_setup();
      while (true)
      {
        sensor_data_polling();
      }
}