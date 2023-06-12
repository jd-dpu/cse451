// Robot Template app
//
// Framework for creating applications that control the Kobuki robot using BLE

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

#include "simple_ble.h"
#include "states.h"

#include "helper_funcs.h"
extern KobukiSensors_t sensors;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// KobukiSensors_t sensors = {0};

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
    // c0:98:e5:49:aa:bb  -- Make sure to match this with YOUR_ADDRESS in robot_control.py
    .platform_id = 0x49,  // used as 4th octect in device BLE address
    .device_id = 0xaabb,  //
    .adv_name = "KOBUKI", // used in advertisements if there is room
    .adv_interval = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
    .min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS),
};

// c05899c4-457c-4c75-93ab-e55018bb3073 -- Make sure to match with SERVICE_UUID

// TODO: define a service for driving with UUID c05899c4-457c-4c75-93ab-e55018bb3073
static simple_ble_service_t drive_service = {{.uuid128 = {0x73, 0x30, 0xbb, 0x18, 0x50, 0xe5, 0xab, 0x93,
                                                          0x75, 0x4c, 0x7c, 0x45, 0xc4, 0x99, 0x58, 0xc0}}};

// TODO: defining characteristics related to driving

// characteristic for forward move: forward_char  -- {.uuid16 = 0x99c5}
// connected variable		  : drive_forward
static simple_ble_char_t forward_char = {.uuid16 = 0x99c5};
static bool drive_forward;

// characteristic for backward move: backward_char -- {.uuid16 = 0x99c6}
// connected variable		   : drive_backward
static simple_ble_char_t backward_char = {.uuid16 = 0x99c6};
static bool drive_backward;

// characteristic for left move   : left_char -- {.uuid16 = 0x99c8}
// connected variable		  : drive_left
static simple_ble_char_t left_char = {.uuid16 = 0x99c8};
static bool drive_left;

// characteristic for right move  : right_char -- {.uuid16 = 0x99c7}
// connected variable		  : drive_right
static simple_ble_char_t right_char = {.uuid16 = 0x99c7};
static bool drive_right;

// characteristic for right move  : right_char -- {.uuid16 = 0x99c9}
// connected variable		  : drive_right
static simple_ble_char_t stop_char = {.uuid16 = 0x99c9};
static bool drive_stop;

static bool prev_df, prev_db, prev_dr, prev_dl = 0;

simple_ble_app_t *simple_ble_app;

// robot drive variables

// left and right wheel speeds -- 0 - 100
static int16_t leftdrive, rightdrive = 0;

// speed increment at each button press from BLE controller
static int16_t speed = 75;

// turn speed increment at each button press from BLE controller
static int16_t turning_speed = 40;

void ble_evt_write(ble_evt_t const *p_ble_evt)
{
  if (simple_ble_is_char_event(p_ble_evt, &forward_char))
  {
    printf("Got write to forward characteristic!\n");
    simple_ble_notify_char(&forward_char);
  }
  if (simple_ble_is_char_event(p_ble_evt, &backward_char))
  {
    printf("Got write to backward characteristic!\n");
    simple_ble_notify_char(&backward_char);
  }
  if (simple_ble_is_char_event(p_ble_evt, &right_char))
  {
    printf("Got write to right characteristic! %b\n", drive_right);
    simple_ble_notify_char(&right_char);
  }
  if (simple_ble_is_char_event(p_ble_evt, &left_char))
  {
    printf("Got write to left characteristic! %b\n", drive_left);
    simple_ble_notify_char(&left_char);
  }
  // stop
  if (simple_ble_is_char_event(p_ble_evt, &stop_char))
  {
    printf("Got write to stop characteristic! %b\n", drive_stop);
  }
  // a small delay to allow the action for some time
  printf("Driving with Left: %d \t Right: %d \n", leftdrive, rightdrive);

  nrf_delay_ms(100);
}

int main(void)
{
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&drive_service);

  // TODO: adding the characteristics for each directional movement

  // characteristic for forward move: forward_char
  // connected variable		  : drive_forward
  simple_ble_add_characteristic(0, 1, 1, 0,
                                sizeof(drive_forward), (uint8_t *)&drive_forward,
                                &drive_service, &forward_char);

  // characteristic for backward move: backward_char
  // connected variable		   : drive_backward
  simple_ble_add_characteristic(0, 1, 1, 0,
                                sizeof(drive_backward), (uint8_t *)&drive_backward,
                                &drive_service, &backward_char);

  // characteristic for left move   : left_char
  // connected variable		  : drive_left
  simple_ble_add_characteristic(0, 1, 1, 0,
                                sizeof(drive_left), (uint8_t *)&drive_left,
                                &drive_service, &left_char);

  // characteristic for right move  : right_char
  // connected variable		  : drive_right
  simple_ble_add_characteristic(0, 1, 1, 0,
                                sizeof(drive_right), (uint8_t *)&drive_right,
                                &drive_service, &right_char);

  simple_ble_add_characteristic(0, 1, 1, 0,
                                sizeof(drive_stop), (uint8_t *)&drive_stop,
                                &drive_service, &stop_char);

  // Start Advertising
  simple_ble_adv_only_name();

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);

  // initialize all the sensors in LSM9DS1
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // loop forever, running state machine
  while (1)
  {
    // even if we are not using the sensors, this command is needed to update the robot status
    kobukiSensorPoll(&sensors);
    nrf_delay_ms(5);

    // reset speed
    leftdrive = rightdrive = 0;

    // base power for wheels
    if (drive_forward)
    {
      leftdrive = rightdrive = speed;
    }
    else if (drive_backward)
      leftdrive = rightdrive = -speed;

    if (drive_left)
    {
      if (leftdrive == 0 && rightdrive == 0)
      {
        leftdrive -= turning_speed;
        rightdrive += turning_speed;
      }else{
        leftdrive = leftdrive - leftdrive * .2;
        rightdrive = rightdrive + rightdrive * .2;
      }
    }

    if (drive_right)
    {
      if (leftdrive == 0 && rightdrive == 0)
      {
        leftdrive += turning_speed;
        rightdrive -= turning_speed;
      }
      else
      {
        leftdrive = leftdrive + leftdrive * .2;
        rightdrive = rightdrive - rightdrive * .2;
      }
    }

    // stop overrides all and resets everything
    if (drive_stop)
    {
      leftdrive = rightdrive = 0;
      drive_left = drive_right = drive_backward = drive_forward = false;
    }

    // an ever present function to operate the robot. As a result of drive actions in BLE application, the drive speeds for left and right wheels are modified
    kobukiDriveDirect(leftdrive, rightdrive);
    // power_manage();
  }
}
