/*
 * blink.c
 *
 * Created: 2024-09-22
 * Author : hjeong
 */ 

#include "config.h"
#include <avr/io.h>
#define F_CPU 16000000UL
#include "util/delay.h"

#ifdef BLINK_ASM
/* address version */
int main_blink_asm(void)
{
	// Set DDRB as output
	asm volatile (
		"ldi r16, 0xFF\n\t"  // Load immediate value 0xFF into register r16
		"out 0x17, r16\n\t"  // Output r16 to DDRB (Port B Data Direction Register, address 0x04)
	);

	while(1)
	{
		// Set PORTB = 0xAA
		asm volatile (
			"ldi r16, 0xAA\n\t"  // Load 0xAA into register r16
			"out 0x18, r16\n\t"  // Output r16 to PORTB (Port B Data Register, address 0x05)
		);

		_delay_ms(2000); // 1-second delay

		// Set PORTB = 0x55
		asm volatile (
			"ldi r16, 0x55\n\t"  // Load 0x55 into register r16
			"out 0x18, r16\n\t"  // Output r16 to PORTB
		);

		_delay_ms(1000); // 1-second delay
	}
}
#endif

#ifdef BLINK_ASM_MACRO
/* macro version  */
int main_blink_asm_macro(void)
{
    // Set DDRB as output using register name
    asm volatile (
        "ldi r16, 0xFF\n\t"   // Load 0xFF into r16
        "out %[ddrb], r16\n\t" // Output r16 to DDRB
        : /* no output */
        : [ddrb] "I" (_SFR_IO_ADDR(DDRB)) // map DDRB to its I/O address
        : "r16" // clobber (modified) list
    );

    while(1)
    {
        // Set PORTB = 0xAA using register name
        asm volatile (
            "ldi r16, 0xAA\n\t"   // Load 0xAA into r16
            "out %[portb], r16\n\t" // Output r16 to PORTB
            : /* no output */
            : [portb] "I" (_SFR_IO_ADDR(PORTB)) // map PORTB to its I/O address
            : "r16" // clobber list
        );

        _delay_ms(1000); // 1-second delay

        // Set PORTB = 0x55 using register name
        asm volatile (
            "ldi r16, 0x55\n\t"   // Load 0x55 into r16
            "out %[portb], r16\n\t" // Output r16 to PORTB
            : /* no output */
            : [portb] "I" (_SFR_IO_ADDR(PORTB)) // map PORTB to its I/O address
            : "r16" // clobber list
        );

        _delay_ms(1000); // 1-second delay
    }
}
#endif

#ifdef BLINK_ASM_RANDOM
/* random version */
// Simple pseudo-random number generator (PRNG) seed
uint16_t seed = 12345; // Initial seed value

// Function to generate a pseudo-random number
uint8_t my_random(void) {
    // Linear Congruential Generator (LCG) parameters
    seed = (seed * 1103515245 + 12345);  // Update seed
    return (uint8_t)(seed & 0xFF);  // Return an 8-bit pseudo-random number
}

int main_blink_asm_random(void)
{
    // Set DDRB as output using inline assembly
    asm volatile (
        "ldi r16, 0xFF\n\t"   // Load 0xFF into r16 (all bits high)
        "out %[ddrb], r16\n\t" // Output r16 to DDRB (set PORTB as output)
        : /* no output */
        : [ddrb] "I" (_SFR_IO_ADDR(DDRB))  // map DDRB to its I/O address
        : "r16"
    );

    while(1)
    {
        // Set PORTB with a random 8-bit value
        asm volatile (
            "out %[portb], %[rand_val]\n\t"  // Output random value directly to PORTB
            : /* no output */
            : [portb] "I" (_SFR_IO_ADDR(PORTB)),  // map PORTB to its I/O address
              [rand_val] "r" (my_random())  // Get a random 8-bit value for PORTB
        );

        // Delay for a random period between 100ms and 1000ms
        _delay_ms(1000);  // Generate a random delay between 100ms and 1000ms
    }
}
#endif

#ifdef BLINK_ASM_RANDOM_DELAY
/* port delay random */
// Simple pseudo-random number generator (PRNG) seed
uint16_t seed = 12345; // Initial seed value

// Function to generate a pseudo-random number
uint8_t my_random(void) {
    // Linear Congruential Generator (LCG) parameters
    seed = (seed * 1103515245 + 12345);  // Update seed
    return (uint8_t)(seed & 0xFF);  // Return an 8-bit pseudo-random number
}

// Custom delay function that delays for approximately the given number of milliseconds
void custom_delay_ms(uint16_t ms) {
    while (ms--) {
        _delay_ms(1);  // Delay 1 millisecond in each loop iteration
    }
}

void main_blink_asm_random_delay(void)
{
    // Set DDRB as output using inline assembly
    asm volatile (
        "ldi r16, 0xFF\n\t"   // Load 0xFF into r16 (all bits high)
        "out %[ddrb], r16\n\t" // Output r16 to DDRB (set PORTB as output)
        : /* no output */
        : [ddrb] "I" (_SFR_IO_ADDR(DDRB))  // map DDRB to its I/O address
        : "r16"
    );

    while(1)
    {
        // Set PORTB with a random 8-bit value
        asm volatile (
            "out %[portb], %[rand_val]\n\t"  // Output random value directly to PORTB
            : /* no output */
            : [portb] "I" (_SFR_IO_ADDR(PORTB)),  // map PORTB to its I/O address
              [rand_val] "r" (my_random())  // Get a random 8-bit value for PORTB
        );

        // Delay for a random period between 100ms and 1000ms using custom delay function
        custom_delay_ms(100 + (my_random() % 900));  // Generate a random delay between 100ms and 1000ms
    }
}
#endif