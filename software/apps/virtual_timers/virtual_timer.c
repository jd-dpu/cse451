// Virtual timer implementation

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "nrf.h"

#include "virtual_timer.h"
#include "virtual_timer_linked_list.h"

// This is the interrupt handler that fires on a compare event
void TIMER4_IRQHandler(void) {
  // This should always be the first line of the interrupt handler!
  // It clears the event so that it doesn't happen again
  NRF_TIMER4->EVENTS_COMPARE[0] = 0;

  // Place your interrupt handler code here
   
  // Update CC[0] register from the remaining timer values
  checkTimers();
  list_print();
}
uint32_t timer_check_duration = 100;

void checkTimers(){
	// update CC[0] value by looking at the linked list
	// node after the current one, and update CC[0] using
	// the timer_value from this node

  /*
  1. Remove the first node and place at the end of the linked list after adding the current timer value. 
  Use read_timer() to get the current timer value. Use the linked list function that both inserts the 
  node and also sorts the list based on the timer value.
  2. Now get the first node of the updated list. This will be the next timer value to trigger the new COMPARE EVENT at.
  3. Check if the timer vallue in this node is already reached. 
  In that case, call the callback function. The callback function can be called using the following command: timer_node->cbFunc();
  */
  uint32_t start_time = read_timer();
  printf("\nTimer interrupt: %d", start_time);//, duration: %d\n",start_time, timer_check_duration);
  node_t * current_node = list_get_first();
  if (current_node == NULL)
    return;

  //make sure we do not miss a virtual timer we will wait until the next timer is in the future
  while(current_node->timer_value < read_timer() + timer_check_duration){ // add timers which may be REALLY close which we may miss
    current_node = list_remove_first();
    if(current_node->repeated){
      current_node->timer_value = read_timer() + current_node->period;
      list_insert_sorted(current_node); 
    }
    current_node->cbFunc();
    current_node = list_get_first();
    if(current_node == NULL)
      return;
  }
  //update our time buffer for processing
  timer_check_duration = read_timer() - start_time;
  NRF_TIMER4->CC[0] = current_node->timer_value; // for overlapping timers
}


// Read the current value of the timer counter
uint32_t read_timer(void) {
  // Same function as the regular timers lab from CSE 351
  // Should return the value of the internal counter for TIMER4
  NRF_TIMER4->TASKS_CAPTURE[0]=1;
  return NRF_TIMER4->CC[0];

}

// Initialize TIMER4 as a free running timer
// 1) Set to be a 32 bit timer
// 2) Set to count at 1MHz
// 3) Enable the timer peripheral interrupt (look carefully at the INTENSET register!)
// 4) Clear the timer
// 5) Start the timer
void virtual_timer_init(void) {
  // Place your timer initialization code here
  NRF_TIMER4->MODE = 0;  // timer mode
  NRF_TIMER4->BITMODE = 3; //32 bit timer
  NRF_TIMER4->PRESCALER  = 4; //prescale divide by 16 (2^4) for 1MHz

  NRF_TIMER4->INTENSET |= (1<<16); //

  NRF_TIMER4->TASKS_CLEAR = 1;
  NRF_TIMER4->TASKS_START = 1;

  NVIC_EnableIRQ(TIMER4_IRQn);
  NVIC_SetPriority(TIMER4_IRQn, 0);

}

// Start a timer. This function is called for both one-shot and repeated timers
// To start a timer:
// 1) Create a linked list node (This requires `malloc()`. Don't forget to free later)
// 2) Setup the linked list node with the correct information
//      - You will need to modify the `node_t` struct in "virtual_timer_linked_list.h"!
// 3) Place the node in the linked list
// 4) Setup the compare register so that the timer fires at the right time
// 5) Return a timer ID
//
// Your implementation will also have to take special precautions to make sure that
//  - You do not miss any timers
//  - You do not cause consistency issues in the linked list (hint: you may need the `__disable_irq()` and `__enable_irq()` functions).
//
// Follow the lab manual and start with simple cases first, building complexity and
// testing it over time.
static uint32_t timer_start(uint32_t microseconds, virtual_timer_callback_t cb, bool repeated) {
  node_t * node = (node_t *) malloc(sizeof(node_t));
  node->cbFunc = cb;
  node->repeated = repeated;
  node->period = microseconds;
  node->timer_value = read_timer() + microseconds;
  node->id = (int)&node;

  __disable_irq();
  list_insert_sorted(node);
  __enable_irq();
  node_t *first = list_get_first();

  //if we are first then update timer
  if (first->id == node->id)
  {
    NRF_TIMER4->CC[0] = first->timer_value;
  }
  // Return a unique timer ID. (hint: What is guaranteed unique about the timer you have created?)
  return node->id;
}

// You do not need to modify this function
// Instead, implement timer_start
uint32_t virtual_timer_start(uint32_t microseconds, virtual_timer_callback_t cb) {
  return timer_start(microseconds, cb, false);
}

// You do not need to modify this function
// Instead, implement timer_start
uint32_t virtual_timer_start_repeated(uint32_t microseconds, virtual_timer_callback_t cb) {
  return timer_start(microseconds, cb, true);
}

// Remove a timer by ID.
// Make sure you don't cause linked list consistency issues!
// Do not forget to free removed timers.
void virtual_timer_cancel(uint32_t timer_id) {
}

