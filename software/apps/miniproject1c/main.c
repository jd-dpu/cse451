// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

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
#include "lsm9ds1.h"
#include "helper_funcs.h"

extern KobukiSensors_t sensors;


// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);


// preferred function to use for distance measurement

float dist_current=0.0f;

float distance_measure(uint16_t encoder)
{
  const float CONVERSION = 0.000677; // 2*pi/65535 * Wheel_dia
  uint16_t cur_encoder = sensors.leftWheelEncoder;

  float updated_dist = cur_encoder - encoder;

  // printf("Delta Encoder -- %d \n",update_dist);

  return (float) updated_dist * CONVERSION; 
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");
  
  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  states state = OFF;
  

  start_gyro();
  float referenceAngle = read_gyro();
  float current_angle = 0;
  uint16_t prev_encoder = sensors.leftWheelEncoder;
   
  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);

    dist_current = distance_measure(prev_encoder);
    current_angle = referenceAngle - read_gyro();

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(50);


    // handle states
    switch(state) {
      case OFF: {
        // transition logic
        if (is_button_press()) {
          state = DRIVING;      
          // initializing the distance calculations from here.
          prev_encoder = read_encoder();
          referenceAngle = read_gyro();
        } else {
          // perform state-specific actions here
          drive_kobuki(0, 0);
          printf("OFF: %f\n", dist_current);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVING: {
        // transition logic
        if (is_button_press()) {
          // initializing the distance calculations from here.
          prev_encoder = read_encoder();          
          referenceAngle = read_gyro();
          state = OFF; 
        } else {
          drive_kobuki(50,50);
        }

        if(is_center_bumper())
        {
          prev_encoder = read_encoder();
          referenceAngle = read_gyro();
          state = REORIENT;
        }
        else if(is_left_bumper())
          state = RIGHT;
        else if (is_right_bumper())
          state = LEFT;
        else
          state = DRIVING;
 
        printf("DRIVING: %f, \t%f\n", dist_current, referenceAngle - read_gyro());
        
        break; // each case needs to end with break!
      }

      case REORIENT: {
        // transition logic
        if (is_button_press()) {
          // initializing the distance calculations from here.
          state = OFF; 
        }else if(dist_current > -.05 && current_angle < 170)
            drive_kobuki(-30,-90);
          else
            state = DRIVING;

        printf("REORIENTING: %f, \t%f\n", dist_current, referenceAngle - read_gyro());
       
        break; // each case needs to end with break!
      }

      case LEFT: {
        // transition logic
        if (is_button_press()) {
          // initializing the distance calculations from here.
          state = OFF; 
        } else if(!is_right_bumper())
        {
          state = DRIVING;
        }
        else {
          drive_kobuki(-80,50);
        }
        printf("LEFT: %f, \t%f\n", dist_current, referenceAngle - read_gyro());
        
        break; // each case needs to end with break!
      }      

      case RIGHT: {
        // transition logic
        if (is_button_press()) {
          // initializing the distance calculations from here.
          state = OFF; 
        }else if(!is_left_bumper())
        {
          state = DRIVING;
        }
        else {
          drive_kobuki(50,-80);
        }
        printf("RIGHT: %f, \t%f\n", dist_current, referenceAngle - read_gyro());

        
        break; // each case needs to end with break!
      }      

    }
  }
}


