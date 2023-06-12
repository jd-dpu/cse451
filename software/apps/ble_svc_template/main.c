// BLE Service Template
//
// Creates a service for changing LED state over BLE

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_gpio.h"

#include "simple_ble.h"
#include "buckler.h"

#include "max44009.h"
#include "nrf_delay.h"

#include "opt3004.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0000, 
        .adv_name          = "CSE352 Light Service", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c70
static simple_ble_service_t light_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x89,0x10,0xE6,0x32}
}};

float lux = 0;

/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state
simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
}

int main(void) {

  // Initialize
  ret_code_t error_code = NRF_SUCCESS;
  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);

  opt3004_config_t config = {
    .range_number = OPT3004_AUTORANGE,
    .conversion_time = OPT3004_CONVERSION_100MS,
    .latch_interrupt = 1,
    .interrupt_polarity = OPT3004_INTERRUPT_ACTIVE_LO,
    .fault_count = OPT3004_FAULT_COUNT_1,
  };

  // initialize opt3004 driver
  opt3004_init(&twi_mngr_instance);
  error_code = opt3004_config(config);

  printf("opt3004 initialized: %ld\n", error_code);

  opt3004_continuous();
  // Setup LED GPIO
  nrf_gpio_cfg_output(BUCKLER_LED0);

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&light_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(lux), (uint8_t*)&lux,
      &light_service, NULL);
  // Start Advertising
  simple_ble_adv_only_name();

  while(1) {
    
    //power_manage();
    
    nrf_delay_ms(2000);
    lux = (float)opt3004_read_result();
    
  }
}

