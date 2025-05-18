#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"

volatile uint32_t timer_overflow_count = 0;

// Timer0 overflow interrupt
ISR(TIMER0_OVF_vect) {
    timer_overflow_count++;
}

// Initialize timer0 for timing
void init_timer0(void) {
    TCCR0A = 0; // Normal port operation
    TCCR0B = (1<<CS01) | (1<<CS00); // Prescaler of 64
    TIMSK0 = (1<<TOIE0); // Enable timer overflow interrupt
    sei(); // Enable global interrupts
}

// Get current time in milliseconds
uint32_t get_time_ms(void) {
    uint32_t time;
    uint8_t oldSREG = SREG;
    cli(); // Disable interrupts
    time = (timer_overflow_count << 8) | TCNT0;
    SREG = oldSREG; // Restore interrupt state
    // With 16MHz clock and prescaler of 64:
    // Each tick = 64/16MHz = 4 microseconds
    // To get milliseconds: time * 4 / 1000 = time * 0.004
    return time * 0.004; // Convert to milliseconds
} 