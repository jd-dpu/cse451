#include "gpio.h"


// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - INPUT =0, OUTPUT =1
void gpio_config(uint8_t pin_numb, bool mode){
    // printf("Requested from Pin #: %d \n", pin_numb);
    uint32_t address = 0x50000000 + 0x700 + 4 * pin_numb;  // loading PIN_CNF[x] register for pin x
    uint32_t *ptr_cnf = (uint32_t *) address;//0x50000000 + 0x00000700;// + 4 * pin_numb;   
    // printf("Address being written at is: %x \n", ptr_cnf);
    *ptr_cnf = mode; // a single digit value automatically goes to DIR bit
}

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t pin_numb)
{
  // printf("Setting Pin #: %d \n", pin_numb);
  uint32_t address = 0x50000000 + 0x508; // loading OUTSET register first
  uint32_t *ptr_OUTSET = (uint32_t *) address;//0x50000000 + 0x00000700;// + 4 * pin_numb; 
  // printf("Address being written at is: %x \n", ptr_OUTSET);
  *ptr_OUTSET = 1<<pin_numb; // a single digit value automatically goes to DIR bit      
}

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t pin_numb){
  // printf("Clearing Pin #: %d \n", pin_numb);
  uint32_t address = 0x50000000 + 0x50C; // loading OUTCLR register first
  uint32_t *ptr_OUTCLR = (uint32_t *) address;//0x50000000 + 0x00000700;// + 4 * pin_numb; 
  // printf("Address being written at is: %x \n", ptr_OUTCLR);
  *ptr_OUTCLR = 1<<pin_numb; // a single digit value automatically goes to DIR bit      
}

// Inputs: 
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
    uint32_t address = 0x50000000 + 0x510; // loading IN register first
    uint32_t *ptr_IN = (uint32_t *) address;//0x50000000 + 0x00000700;// + 4 * pin_numb; 
    
    bool val = (*(ptr_IN)&(1<<gpio_num));
    
     //printf("IN Reg: %x \n", *ptr_IN);
    // should return pin state
    return val;
}

void gpio_toggle(uint8_t pin_numb){    
    uint32_t address = 0x50000000 +0x504; // loading OUT register
    uint32_t *ptr = (uint32_t *) address; 
    int val = (*(ptr));
 
    *ptr ^=val<<pin_numb;  // Read the contents of the register and clear a necessary bit
}
