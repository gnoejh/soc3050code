/*
 * =============================================================================
 * ASSEMBLY INSTRUCTION PROGRAMMING - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Port_Assembly
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of essential AVR assembly instructions for port programming.
 * Students learn direct register manipulation and hardware control fundamentals.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master essential AVR assembly instructions (IN, OUT, SBI, CBI, SBIC, SBIS)
 * 2. Learn direct register manipulation techniques
 * 3. Practice inline assembly programming in C
 * 4. Understand low-level hardware control principles
 * 5. Compare assembly vs C programming approaches
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - LEDs connected to PORTB (PB0-PB7) with current-limiting resistors
 * - Push buttons connected to PORTD for input testing
 * - Status indicators for instruction demonstration
 * - Serial connection for educational feedback (9600 baud)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: OUT instruction for output control
 * - Demo 2: IN instruction for input reading
 * - Demo 3: SBI instruction for single bit set
 * - Demo 4: CBI instruction for single bit clear
 * - Demo 5: SBIC instruction for skip if bit clear
 * - Demo 6: SBIS instruction for skip if bit set
 *
 * =============================================================================
 */

#include "config.h"

#define DELAY_SHORT 100
#define DELAY_MEDIUM 250
#define DELAY_LONG 500

// Demo 1: OUT - Write to I/O
void demo_01_out(void)
{
    asm volatile("ldi r16, 0xFF \n\t"
                 "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(DDRB)) : "r16");
    while (1)
    {
        asm volatile("ldi r16, 0xFF \n\t"
                     "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)) : "r16");
        _delay_ms(DELAY_LONG);
        asm volatile("ldi r16, 0x00 \n\t"
                     "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)) : "r16");
        _delay_ms(DELAY_LONG);
    }
}

// Demo 2: IN - Read from I/O
void demo_02_in(void)
{
    asm volatile("ldi r16, 0xFF \n\t"
                 "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(DDRB)) : "r16");
    asm volatile("ldi r16, 0x00 \n\t"
                 "out %0, r16 \n\t"
                 "ldi r16, 0x80 \n\t"
                 "out %1, r16 \n\t"
                 : : "I"(_SFR_IO_ADDR(DDRD)), "I"(_SFR_IO_ADDR(PORTD)) : "r16");

    uint8_t val;
    while (1)
    {
        asm volatile("in %0, %1 \n\t" : "=r"(val) : "I"(_SFR_IO_ADDR(PIND)));
        if (val & 0x80)
        {
            asm volatile("ldi r16, 0x0F \n\t"
                         "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)) : "r16");
        }
        else
        {
            asm volatile("ldi r16, 0xF0 \n\t"
                         "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)) : "r16");
        }
        _delay_ms(10);
    }
}

// Demo 3: SBI - Set Bit
void demo_03_sbi(void)
{
    asm volatile("ldi r16, 0xFF \n\t"
                 "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(DDRB)) : "r16");
    asm volatile("ldi r16, 0x00 \n\t"
                 "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)) : "r16");

    while (1)
    {
        for (int i = 0; i < 8; i++)
        {
            asm volatile("sbi %0, 0 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
            if (i == 0)
            {
                asm volatile("sbi %0, 0 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
            }
            if (i == 1)
            {
                asm volatile("sbi %0, 1 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
            }
            if (i == 2)
            {
                asm volatile("sbi %0, 2 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
            }
            if (i == 3)
            {
                asm volatile("sbi %0, 3 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
            }
            if (i == 4)
            {
                asm volatile("sbi %0, 4 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
            }
            if (i == 5)
            {
                asm volatile("sbi %0, 5 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
            }
            if (i == 6)
            {
                asm volatile("sbi %0, 6 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
            }
            if (i == 7)
            {
                asm volatile("sbi %0, 7 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
            }
            _delay_ms(DELAY_MEDIUM);
        }
        asm volatile("ldi r16, 0x00 \n\t"
                     "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)) : "r16");
        _delay_ms(DELAY_LONG);
    }
}

// Demo 4: CBI - Clear Bit
void demo_04_cbi(void)
{
    asm volatile("ldi r16, 0xFF \n\t"
                 "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(DDRB)) : "r16");

    while (1)
    {
        asm volatile("ldi r16, 0xFF \n\t"
                     "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)) : "r16");
        _delay_ms(DELAY_LONG);

        asm volatile("cbi %0, 0 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
        _delay_ms(DELAY_MEDIUM);
        asm volatile("cbi %0, 1 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
        _delay_ms(DELAY_MEDIUM);
        asm volatile("cbi %0, 2 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
        _delay_ms(DELAY_MEDIUM);
        asm volatile("cbi %0, 3 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
        _delay_ms(DELAY_MEDIUM);
        asm volatile("cbi %0, 4 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
        _delay_ms(DELAY_MEDIUM);
        asm volatile("cbi %0, 5 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
        _delay_ms(DELAY_MEDIUM);
        asm volatile("cbi %0, 6 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
        _delay_ms(DELAY_MEDIUM);
        asm volatile("cbi %0, 7 \n\t" : : "I"(_SFR_IO_ADDR(PORTB)));
        _delay_ms(DELAY_LONG);
    }
}

// Demo 5: SBIC - Skip if Bit is Clear
void demo_05_sbic(void)
{
    asm volatile("ldi r16, 0xFF \n\t"
                 "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(DDRB)) : "r16");
    asm volatile("cbi %0, 7 \n\t"
                 "sbi %1, 7 \n\t" : : "I"(_SFR_IO_ADDR(DDRD)), "I"(_SFR_IO_ADDR(PORTD)));

    while (1)
    {
        asm volatile(
            "sbic %0, 7 \n\t"
            "rjmp not_pressed5 \n\t"
            "ldi r16, 0xFF \n\t"
            "out %1, r16 \n\t"
            "rjmp end_check5 \n\t"
            "not_pressed5: \n\t"
            "ldi r16, 0x00 \n\t"
            "out %1, r16 \n\t"
            "end_check5: \n\t"
            : : "I"(_SFR_IO_ADDR(PIND)), "I"(_SFR_IO_ADDR(PORTB)) : "r16");
        _delay_ms(10);
    }
}

// Demo 6: SBIS - Skip if Bit is Set
void demo_06_sbis(void)
{
    asm volatile("ldi r16, 0xFF \n\t"
                 "out %0, r16 \n\t" : : "I"(_SFR_IO_ADDR(DDRB)) : "r16");
    asm volatile("cbi %0, 7 \n\t"
                 "sbi %1, 7 \n\t" : : "I"(_SFR_IO_ADDR(DDRD)), "I"(_SFR_IO_ADDR(PORTD)));

    while (1)
    {
        asm volatile(
            "sbis %0, 7 \n\t"
            "rjmp pressed6 \n\t"
            "ldi r16, 0xAA \n\t"
            "out %1, r16 \n\t"
            "rjmp end_check6 \n\t"
            "pressed6: \n\t"
            "ldi r16, 0x55 \n\t"
            "out %1, r16 \n\t"
            "end_check6: \n\t"
            : : "I"(_SFR_IO_ADDR(PIND)), "I"(_SFR_IO_ADDR(PORTB)) : "r16");
        _delay_ms(10);
    }
}

int main(void)
{
    demo_01_out(); // Start here: Learn OUT
    // demo_02_in(); // Learn IN
    //  demo_03_sbi();    // Learn SBI
    //  demo_04_cbi();    // Learn CBI
    //  demo_05_sbic();   // Learn SBIC
    //  demo_06_sbis();   // Learn SBIS

    return 0;
}
