#pragma once
#include "stdint.h"
#include "nrf.h"
#include "stdbool.h"


// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - INPUT =0, OUTPUT =1
void gpio_config(uint8_t gpio_num, bool dir);

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num);

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num);

// Inputs: 
//  gpio_num - gpio number 0-31
// Returns:
//  current state of the specified gpio pin
bool gpio_read(uint8_t gpio_num);

void gpio_toggle(uint8_t pin_numb);
