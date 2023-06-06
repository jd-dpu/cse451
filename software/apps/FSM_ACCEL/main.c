// Finite State Machine for 
// Blinking LED at the press of a button

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "mpu9250.h"

#include "gpio.h"
#include "states.h"
#include "lsm9ds1.h"
#include "display.h"

// intialize statechart variables
  // if needed


state current_state;
// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

void print_state(){

  printf("CURRENT STATE: %d \n", current_state);
}
void initialize_hardware(){
  ret_code_t error_code = NRF_SUCCESS;
  gpio_config(BUCKLER_LED0, true);
  gpio_config(BUCKLER_LED1, true);
  gpio_config(BUCKLER_LED2, true);


  gpio_set(BUCKLER_LED0);
  gpio_set(BUCKLER_LED1);
  gpio_set(BUCKLER_LED2);

  gpio_config(BUCKLER_BUTTON0, false);

  // any additional robot related initialization can be placed here as well

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);

  // initialize LSM9DS1 driver
  lsm9ds1_init(&twi_mngr_instance);
  printf("lsm9ds1 initialized\n");
  lsm9ds1_start_gyro_integration();

  //Display initialization
  // initialize spi master(controller)
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = { 
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };  
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);

  // initialize display driver
  display_init(&spi_instance);
  printf("Display initialized\n");
  nrf_delay_ms(1000);
}

void display_accel_data()
{
    lsm9ds1_measurement_t acc_measurement = lsm9ds1_read_accelerometer();

    printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
    printf("                  ----------\t----------\t----------\n");
    printf("Acceleration (g): %10.3f\t%10.3f\t%10.3f\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
    
    return;
    char buf[16] = {0};
    snprintf(buf, 16, "accel (x,y,z)");
    display_write(buf, 0);
    snprintf(buf, 16, "%2.1f,%2.1f,%2.1f", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
    display_write(buf, 1);
}

void display_gyro_data()
{
    lsm9ds1_measurement_t gyr_measurement = lsm9ds1_read_gyro_integration();

    printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
    printf("                  ----------\t----------\t----------\n");
    printf("Angle  (degrees): %10.3f\t%10.3f\t%10.3f\n", gyr_measurement.x_axis, gyr_measurement.y_axis, gyr_measurement.z_axis);

    return;
    char buf[16] = {0};
    snprintf(buf, 16, "gyro (x,y,z)");
    display_write(buf, 0);
    snprintf(buf, 16, "%2.1f,%2.1f,%2.1f", gyr_measurement.x_axis, gyr_measurement.y_axis, gyr_measurement.z_axis);
    display_write(buf, 1);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs

  initialize_hardware();
  
  // initialize the state variable

  current_state = INIT; 
  
  // loop forever, running state machine
  while (1) {
    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(50);
    print_state();

    // iterate statechart
    switch(current_state){


      case INIT:
        // move on to testing the button status
        current_state = OFF;
        break;
      case ON:
        gpio_set(BUCKLER_LED0);
        display_accel_data();        
        if (!gpio_read(BUCKLER_BUTTON0))
          {
            current_state = OFF;            
          }
        break;
      case OFF:
        gpio_clear(BUCKLER_LED0);
        if (!gpio_read(BUCKLER_BUTTON0))
          {
            current_state = ON;          
          }
        break;
      default:
        current_state = OFF;

    }
    
  }
}
