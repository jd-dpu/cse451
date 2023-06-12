

// BLE App for Display Access and Light Detector Values

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_gpio.h"
#include "display.h"

#include "simple_ble.h"
#include "buckler.h"

#include "opt3004.h"
#include "nrf_delay.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"
#include "helper_funcs.h"

extern KobukiSensors_t sensors;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);


// Create a timer
APP_TIMER_DEF(light_timer);

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

float dist_current=0.0f;

float distance_measure(uint16_t encoder)
{
  const float CONVERSION = 0.000677; // 2*pi/65535 * Wheel_dia
  uint16_t cur_encoder = sensors.leftWheelEncoder;

  float updated_dist = cur_encoder - encoder;

  // printf("Delta Encoder -- %d \n",update_dist);

  return (float) updated_dist * CONVERSION; 
}

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0000, 
        .adv_name          = "LightDetect", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

static simple_ble_service_t accel_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x10,0x89,0xE6,0x32}
}};

static simple_ble_char_t accel_char = {.uuid16 = 0x8911};
static float accel_value = 0;

//de97aeee-0e7e-4720-8038-4dc47aa9562f
static simple_ble_service_t gyro_service = {{
    .uuid128 = {0x2f,0x56,0xa9,0x7a,0xc4,0x4d,0x38,0x80,
                0x20,0x47,0x7e,0x0e,0xee,0xae,0x97,0xde}
}};

static simple_ble_char_t gyro_char = {.uuid16 = 0xaeef};
static float gyro_value = 0;


/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state
simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
     // Implement the display action
    }




int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  // Initialize

  // initialize display
  // nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  // nrf_drv_spi_config_t spi_config = {
  //   .sck_pin = BUCKLER_LCD_SCLK,
  //   .mosi_pin = BUCKLER_LCD_MOSI,
  //   .miso_pin = BUCKLER_LCD_MISO,
  //   .ss_pin = BUCKLER_LCD_CS,
  //   .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
  //   .orc = 0,
  //   .frequency = NRF_DRV_SPI_FREQ_4M,
  //   .mode = NRF_DRV_SPI_MODE_2,
  //   .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  // };
  // error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  // APP_ERROR_CHECK(error_code);
  // display_init(&spi_instance);
  // display_write("Hello, Human!", DISPLAY_LINE_0);
  // printf("Display initialized!\n");

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

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);


  // Display Service 
  simple_ble_add_service(&display_service);

  simple_ble_add_characteristic(0, 1, 1, 0,
     sizeof(display_data), (uint8_t*)&display_data,
     &display_service, &display_char);

  // Light Sensor Service 
  simple_ble_add_service(&light_service);

  simple_ble_add_characteristic(1, 0, 1, 0,
      sizeof(light_value), (uint8_t*)&light_value,
      &light_service, &light_char);

  // Start Advertising
  simple_ble_adv_only_name();

  // Set a timer to read the light sensor and notify readers
  app_timer_init();
  app_timer_create(&light_timer, APP_TIMER_MODE_REPEATED, (app_timer_timeout_handler_t) light_timer_callback);
  app_timer_start(light_timer, APP_TIMER_TICKS(1000), NULL); // 1000 milliseconds

  while(1) {
    if (sample_light) {
       // first, read the sensor value
       // need a simple_ble notify characteristic action here

    }
    nrf_delay_ms(2000);
    light_value = (float)opt3004_read_result();
    //power_manage();
  }
}

