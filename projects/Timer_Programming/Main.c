/*
 * =============================================================================
 * TIMER0 PROGRAMMING - DEMO CODE
 * =============================================================================
 * PROJECT: Timer_Programming
 * See Slide.md for complete lecture content and theory
 *
 * DEMOS:
 * 1. Normal Mode POLLING - Overflow flag polling
 * 2. Normal Mode INTERRUPT - Overflow ISR
 * 3. CTC Mode POLLING - Compare match flag polling (Timer1)
 * 4. CTC Mode INTERRUPT - Compare match ISR (Timer1)
 * 5. Fast PWM Mode - PWM generation (Timer1)
 * 6. Phase Correct PWM - Symmetric PWM (Timer1)
 * 7. Prescaler Comparison - All 5 prescaler settings
 * 8. Multi-tasking with Overflow ISR
 * =============================================================================
 */

#include "config.h"

// Global variables for interrupt-driven demos
volatile uint16_t overflow_count = 0;
volatile uint16_t compare_count = 0;
volatile uint16_t timer_ticks = 0;
volatile uint8_t task1_flag = 0;
volatile uint8_t task2_flag = 0;
volatile uint8_t task3_flag = 0;
volatile uint8_t demo_mode = 0; // ISR mode selector

// ===== MAIN - SELECT YOUR DEMO =====
int main(void)
{
    simple_init();

    // Uncomment ONE demo to run:
    // demo0_led_test();              // Simple LED blink test
    // demo1_normal_polling(); // Timer0 overflow polling
    // demo2_normal_interrupt();      // Timer0 overflow interrupt
    demo3_ctc_polling(); // Timer1 CTC polling
    // demo4_ctc_interrupt();         // Timer1 CTC interrupt
    // demo5_fast_pwm();              // Timer1 Fast PWM
    // demo6_phase_correct_pwm();     // Timer1 Phase Correct PWM
    // demo7_prescaler_comparison();  // Prescaler comparison
    // demo8_multitask_interrupt();   // Multi-task with ISR

    while (1)
    {
    }
    return 0;
}

// ===== DEMO 0: LED TEST =====
// Simple LED blink without timer - verify hardware
void demo0_led_test(void)
{
    uint8_t count = 0;
    while (1)
    {
        PORTB &= ~(1 << 0); // LED ON (active LOW)
        _delay_ms(1000);
        PORTB |= (1 << 0); // LED OFF
        _delay_ms(1000);

        if (++count >= 10)
        {
            PORTB = 0x00; // All ON
            _delay_ms(200);
            PORTB = 0xFF; // All OFF
            _delay_ms(200);
            count = 0;
        }
    }
}

// ===== DEMO 1: NORMAL MODE - POLLING =====
// Timer0 overflow polling: 16MHz/1024/256 = 61 Hz
// Toggle LED every 30 overflows (~0.5 sec)
void demo1_normal_polling(void)
{
    uint16_t count = 0;

    TCCR0 = 0x00;       // Stop timer
    TCNT0 = 0;          // Clear counter
    TIFR = (1 << TOV0); // Clear overflow flag
    PORTB &= ~(1 << 0); // LED ON

    // Start Timer0: Normal mode, prescaler 1024
    TCCR0 = (1 << CS02) | (1 << CS00);

    while (1)
    {
        if (TIFR & (1 << TOV0)) // Poll overflow flag
        {
            count++;
            TIFR = (1 << TOV0); // Clear flag

            if (count >= 30)
            {
                PORTB ^= (1 << 0); // Toggle LED
                count = 0;
            }
        }
    }
}

// ===== TIMER ISRs =====
// Timer0 Overflow ISR - shared by demo2 and demo8
ISR(TIMER0_OVF_vect)
{
    if (demo_mode == 2)
    {
        overflow_count++;
        if (overflow_count >= 30) // Toggle every 30 overflows
        {
            PORTB ^= (1 << 0);
            overflow_count = 0;
        }
    }
    else if (demo_mode == 8)
    {
        TCNT0 = 6; // Reload for ~1ms timing
        timer_ticks++;
        if (timer_ticks % 1 == 0)
            task1_flag = 1;
        if (timer_ticks % 100 == 0)
            task2_flag = 1;
        if (timer_ticks % 500 == 0)
            task3_flag = 1;
    }
}

// Timer1 Compare Match A ISR
ISR(TIMER1_COMPA_vect)
{
    compare_count++;
    if (compare_count >= 1000) // Toggle every 1000 matches (~1 sec)
    {
        PORTB ^= (1 << 0);
        compare_count = 0;
    }
}

// ===== DEMO 2: NORMAL MODE - INTERRUPT =====
// Timer0 overflow interrupt: Same timing as demo1, but CPU is free
void demo2_normal_interrupt(void)
{
    overflow_count = 0;
    demo_mode = 2;

    TCCR0 = 0x00;
    TCNT0 = 0;
    TIFR = (1 << TOV0);
    PORTB &= ~(1 << 0); // LED ON

    TIMSK = (1 << TOIE0);              // Enable overflow interrupt
    TCCR0 = (1 << CS02) | (1 << CS00); // Prescaler 1024
    sei();

    while (1)
    {
        // CPU is free - ISR handles LED toggling
        for (volatile uint16_t i = 0; i < 1000; i++)
            asm volatile("nop");
    }
}

// ===== DEMO 3: CTC MODE - POLLING (Timer1) =====
// Timer1 CTC polling: 16MHz/64/250 = 1kHz, toggle every 1000 matches
void demo3_ctc_polling(void)
{
    uint16_t count = 0;

    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1H = 0;
    TCNT1L = 0;
    OCR1AH = 0;
    OCR1AL = 249; // TOP = 249, 1ms period
    TIFR = (1 << OCF1A);
    PORTB &= ~(1 << 0);

    // CTC mode (WGM12=1), prescaler 64
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    while (1)
    {
        if (TIFR & (1 << OCF1A))
        {
            count++;
            TIFR = (1 << OCF1A);
            if (count >= 1000)
            {
                PORTB ^= (1 << 0);
                count = 0;
            }
        }
    }
}

// ===== DEMO 4: CTC MODE - INTERRUPT (Timer1) =====
// Timer1 CTC interrupt: Same as demo3 but using ISR
void demo4_ctc_interrupt(void)
{
    compare_count = 0;

    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1H = 0;
    TCNT1L = 0;
    OCR1AH = 0;
    OCR1AL = 249;
    TIFR = (1 << OCF1A);
    PORTB &= ~(1 << 0);

    TIMSK = (1 << OCIE1A); // Enable compare match interrupt
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    sei();

    while (1)
    {
        for (volatile uint16_t i = 0; i < 1000; i++)
            asm volatile("nop");
    }
}

// ===== DEMO 5: FAST PWM MODE (Timer1) =====
// Timer1 Fast PWM 8-bit: 16MHz/64/256 = 976.5 Hz
// Duty cycle sweep 0-100% on PB5 (OC1A)
void demo5_fast_pwm(void)
{
    uint8_t duty = 0;

    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1H = 0;
    TCNT1L = 0;
    OCR1AH = 0;
    OCR1AL = 0;

    DDRB |= (1 << 5); // PB5 (OC1A) as output
    DDRB = 0xFF;
    PORTB = 0xFF;

    // Fast PWM 8-bit: COM1A1=1, WGM10=1, WGM12=1, prescaler 64
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    while (1)
    {
        for (duty = 0; duty < 255; duty++)
        {
            OCR1AL = duty;
            PORTB = ~duty; // Visual display
            for (volatile uint16_t i = 0; i < 10000; i++)
                asm volatile("nop");
        }
        for (duty = 255; duty > 0; duty--)
        {
            OCR1AL = duty;
            PORTB = ~duty;
            for (volatile uint16_t i = 0; i < 10000; i++)
                asm volatile("nop");
        }
    }
}

// ===== DEMO 6: PHASE CORRECT PWM (Timer1) =====
// Timer1 Phase Correct PWM 8-bit: ~490 Hz
// Symmetric counting (0->255->0) on PB5 (OC1A)
void demo6_phase_correct_pwm(void)
{
    uint8_t duty = 128;

    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1H = 0;
    TCNT1L = 0;
    OCR1AH = 0;
    OCR1AL = duty;

    DDRB |= (1 << 5); // PB5 (OC1A) as output

    // Phase Correct PWM 8-bit: COM1A1=1, WGM10=1, prescaler 64
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << CS11) | (1 << CS10);

    uint16_t count = 0;
    while (1)
    {
        duty = 128 + (count % 128);
        OCR1AL = duty;
        for (volatile uint16_t i = 0; i < 50000; i++)
            asm volatile("nop");
        count++;
    }
}

// ===== DEMO 7: PRESCALER COMPARISON =====
// Cycles through all 5 Timer0 prescaler values
void demo7_prescaler_comparison(void)
{
    const uint8_t values[] = {
        (1 << CS00),                // clk/1
        (1 << CS01),                // clk/8
        (1 << CS01) | (1 << CS00),  // clk/64
        (1 << CS02),                // clk/256
        (1 << CS02) | (1 << CS00)}; // clk/1024

    PORTB &= ~(1 << 0);

    while (1)
    {
        for (uint8_t i = 0; i < 5; i++)
        {
            TCCR0 = 0x00;
            TCNT0 = 0;
            TIFR = (1 << TOV0);
            TCCR0 = values[i];

            for (uint16_t j = 0; j < 50; j++)
            {
                while (!(TIFR & (1 << TOV0)))
                    ;
                TIFR = (1 << TOV0);
                PORTB ^= (1 << 0);
            }

            for (volatile uint32_t k = 0; k < 100000; k++)
                asm volatile("nop");
        }
    }
}

// ===== DEMO 8: MULTI-TASKING WITH ISR =====
// Timer0 overflow interrupt for task scheduling
// Tasks at different rates: 1ms, 100ms, 500ms
void demo8_multitask_interrupt(void)
{
    timer_ticks = 0;
    demo_mode = 8;

    PORTB &= ~(1 << 0);

    TCCR0 = 0x00;
    TCNT0 = 6;
    TIFR = (1 << TOV0);
    TIMSK = (1 << TOIE0);
    TCCR0 = (1 << CS01) | (1 << CS00); // Prescaler 64
    sei();

    while (1)
    {
        if (task1_flag)
            task1_flag = 0;

        if (task2_flag)
        {
            task2_flag = 0;
            PORTB ^= (1 << 0); // Toggle every 100ms
        }

        if (task3_flag)
            task3_flag = 0;
    }
}

// ===== HELPER FUNCTIONS =====
void show_demo_menu(void) {}

void simple_init(void)
{
    DDRB = 0xFF;  // All pins as output
    PORTB = 0xFF; // All LEDs OFF (active low)
    for (volatile uint32_t i = 0; i < 100000; i++)
        asm volatile("nop");
}
