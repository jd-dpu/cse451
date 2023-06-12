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


// intialize statechart variables
  // if needed


state current_state = INIT;

void print_state(){

  printf("CURRENT STATE: %d \n", current_state);
}

void initialize_hardware(){

  gpio_config(BUCKLER_LED0, true);
  gpio_config(BUCKLER_LED1, true);
  gpio_config(BUCKLER_LED2, true);


  gpio_set(BUCKLER_LED0);
  gpio_set(BUCKLER_LED1);
  gpio_set(BUCKLER_LED2);

  gpio_config(BUCKLER_BUTTON0, false);

  // any additional robot related initialization can be placed here as well

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
  bool unpressed = true;
 
  // loop forever, running state machine
  print_state();
  while (1) {
    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(50);

    // iterate statechart
    switch(current_state){
      case INIT:
        // move on to testing the button status
        current_state = OFF;
        break;
      case ON:
        gpio_clear(BUCKLER_LED0);
        
        if (!gpio_read(BUCKLER_BUTTON0)){
          if(unpressed){
            current_state = BLINK;
            unpressed = false;
            print_state();

          }
        }else
          unpressed = true;            
        break;
      case OFF:
        gpio_set(BUCKLER_LED0);
        
        if (!gpio_read(BUCKLER_BUTTON0)){
          if(unpressed){
            current_state = ON;
            unpressed = false;
          print_state();
          }        
        }else
          unpressed = true;
        break;
      case BLINK:
        gpio_set(BUCKLER_LED0);
        nrf_delay_ms(500);
        gpio_clear(BUCKLER_LED0);
        nrf_delay_ms(500);

        if (!gpio_read(BUCKLER_BUTTON0))
        {
          current_state = OFF;
          unpressed = false;
          print_state();
        }else
          unpressed = true;
        break;
      default:
        current_state = OFF;

    }
    
  }
}
