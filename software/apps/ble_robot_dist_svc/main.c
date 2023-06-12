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
#include "app_util.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"
#include "helper_funcs.h"
#include "simple_ble.h"

extern KobukiSensors_t sensors;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// preferred function to use for distance measurement

static float dist_current = 0.0f;
static float current_angle = 0;
simple_ble_app_t* simple_ble_app;

static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0000, 
        .adv_name          = "Robot Sensor Service", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

//c05899c4-457c-4c75-93ab-e55018bb3073
static simple_ble_service_t sensor_service = {{.uuid128 = {0x73, 0x30, 0xbb, 0x18, 0x50, 0xe5, 0xab, 0x93,
                                                          0x75, 0x4c, 0x7c, 0x45, 0xc4, 0x99, 0x58, 0xc0}}};

static simple_ble_char_t dist_char = {.uuid16 = 0x99c5};
static simple_ble_char_t gyro_char = {.uuid16 = 0x99c6};

float distance_measure(uint16_t encoder)
{
  const float CONVERSION = 0.000677; // 2*pi/65535 * Wheel_dia
  uint16_t cur_encoder = sensors.leftWheelEncoder;

  float updated_dist = cur_encoder - encoder;

  // printf("Delta Encoder -- %d \n",update_dist);

  return (float)updated_dist * CONVERSION;
}

int main(void)
{
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

  // encoder values & distance measurement variables

  start_gyro();
  
  uint16_t prev_encoder = sensors.leftWheelEncoder;

  //ble setup

  simple_ble_app = simple_ble_init(&ble_config);

  // Display Service 
  simple_ble_add_service(&sensor_service);

  //distance
  simple_ble_add_characteristic(1, 0, 1, 0,
      sizeof(dist_current), (uint8_t*)&dist_current,
      &sensor_service, &dist_char);

  //gyro
  simple_ble_add_characteristic(1, 0, 1, 0,
      sizeof(current_angle), (uint8_t*)&current_angle,
      &sensor_service, &gyro_char);

  // Start Advertising
  simple_ble_adv_only_name();

  // loop forever, running state machine
  while (1)
  {
    // read sensors from robot
    kobukiSensorPoll(&sensors);

    dist_current = distance_measure(prev_encoder);
    current_angle = read_gyro();

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(50);
    
    // handle states
    switch (state)
    {
      case OFF:
      {
        // transition logic
        if (is_button_press())
        {
          state = DRIVING;
          // initializing the distance calculations from here.
          prev_encoder = read_encoder();
        }
        else
        {
          // perform state-specific actions here
          drive_kobuki(0, 0);
          printf("OFF: %f\n", dist_current);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVING:
      {
        if (is_button_press())
        {
          state = OFF;
          // initializing the distance calculations from here.
          prev_encoder = read_encoder();
        }
        else // forward about 0.5m
          drive_kobuki(50, 50);
        printf("\nCurrent Dist: \t%.2f\tReference Angle:%.2f", dist_current, current_angle);
        break; // each case needs to end with break!
      }
    }
  }
}
