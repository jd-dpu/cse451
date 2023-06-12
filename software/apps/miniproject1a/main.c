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
  
  triangle_path_states path_state = FIRST_LEG;

  // encoder values & distance measurement variables

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
    int bias = 0;

    // handle states
    switch(state) {
      case OFF: {
        // transition logic
        if (is_button_press()) {
          state = DRIVING;      
          path_state = FIRST_LEG;    
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
          switch(path_state)
          {
            case FIRST_LEG:{
            if (current_angle < -3)
              bias = 5;
            else if (current_angle > 3)
              bias = -5;
            else
              bias = 0; 
            if(dist_current <= 0.5)
              drive_kobuki(50+bias, 50-bias);
            else
            {
              drive_kobuki(0,0);
              nrf_delay_ms(500);
              referenceAngle = read_gyro();
              prev_encoder = read_encoder();
              path_state = RIGHT_ANGLE;
            }
            break;

            }
            case RIGHT_ANGLE:{
            if(current_angle < 90)
            {
              drive_kobuki(50, -50);
            }else
            {
              drive_kobuki(0, 0);
              nrf_delay_ms(500);
              referenceAngle = read_gyro();
              prev_encoder = read_encoder();
              path_state = BASE_LEG;
            }

            break;
            }

            case BASE_LEG:{
            if (current_angle < -3)
              bias = 5;
            else if (current_angle > 3 )
              bias = -5;
            else
              bias = 0; 
            if(dist_current < 0.5)
              drive_kobuki(50+bias, 50-bias);
            else
            {
              drive_kobuki(0,0);
              nrf_delay_ms(500);
              referenceAngle = read_gyro();
              prev_encoder = read_encoder();
              path_state = OBTUSE_ANGLE;
            }

            break;
            }
            case OBTUSE_ANGLE:{
            if((referenceAngle - read_gyro()) < 135)
            {
              drive_kobuki(50, -50);
            }else
            {
              drive_kobuki(0, 0);
              nrf_delay_ms(500);
              referenceAngle = read_gyro();
              prev_encoder = read_encoder();
              path_state = HYPOTNUSE;
            }

            break;
            }
            case HYPOTNUSE:{
            if (current_angle < -3)
              bias = 5;
            else if (current_angle > 3 )
              bias = -5;
            else
              bias = 0; 
              
            if(dist_current < 0.7)
              drive_kobuki(50+bias, 50-bias);
            else
            {
              drive_kobuki(0,0);
              nrf_delay_ms(500);
              referenceAngle = read_gyro();
              prev_encoder = read_encoder();
              path_state = DONE;
            }

            break;
            }
          }

          state = DRIVING;
 
          printf("DRIVING: %f, \t%f\n", dist_current, referenceAngle - read_gyro());
        }
        break; // each case needs to end with break!
      }
      
      
      // add other cases here

    }
  }
}

