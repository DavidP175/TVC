#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

// Initialize timer0 for timing
void init_timer0(void);

// Get current time in milliseconds
uint32_t get_time_ms(void);

#endif // TIMER_H 