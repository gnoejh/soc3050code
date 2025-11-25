/*
 * =============================================================================
 * EDUCATIONAL ATmega128 GENERAL TIMER LIBRARY - IMPLEMENTATION FILE
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim
 *
 * This file implements comprehensive timer functionality for all ATmega128
 * timer/counters with professional-quality code and educational documentation.
 *
 * =============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>
#include "_timer.h"

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz default
#endif

/*
 * =============================================================================
 * INTERNAL STATE AND CALLBACK STORAGE
 * =============================================================================
 */

/* Callback function pointers for each timer */
static timer_callback_t overflow_callbacks[4] = {NULL, NULL, NULL, NULL};
static timer_callback_t compare_a_callbacks[4] = {NULL, NULL, NULL, NULL};
static timer_callback_t compare_b_callbacks[4] = {NULL, NULL, NULL, NULL};
static timer_callback_t compare_c_callbacks[4] = {NULL, NULL, NULL, NULL};
static timer_capture_callback_t capture_callback = NULL;

/* Millisecond counter for timing system */
volatile uint32_t system_milliseconds = 0;

/* Task scheduler state */
static timer_task_t scheduler_tasks[TIMER_MAX_TASKS];
static uint8_t scheduler_initialized = 0;

/*
 * =============================================================================
 * CORE TIMER FUNCTIONS
 * =============================================================================
 */

void Timer_init(timer_id_t timer, timer_mode_t mode, uint16_t prescaler)
{
    switch (timer)
    {
    case TIMER_0:
        // Stop timer first
        TCCR0 = 0;
        TCNT0 = 0;

        // Configure mode
        switch (mode)
        {
        case TIMER_MODE_NORMAL:
            // WGM01:00 = 00 (Normal mode)
            TCCR0 &= ~((1 << WGM01) | (1 << WGM00));
            break;
        case TIMER_MODE_CTC:
            // WGM01:00 = 10 (CTC mode)
            TCCR0 |= (1 << WGM01);
            TCCR0 &= ~(1 << WGM00);
            break;
        case TIMER_MODE_FAST_PWM:
            // WGM01:00 = 11 (Fast PWM)
            TCCR0 |= (1 << WGM01) | (1 << WGM00);
            break;
        case TIMER_MODE_PHASE_CORRECT_PWM:
            // WGM01:00 = 01 (Phase Correct PWM)
            TCCR0 &= ~(1 << WGM01);
            TCCR0 |= (1 << WGM00);
            break;
        default:
            break;
        }

        // Set prescaler (starts timer)
        TCCR0 = (TCCR0 & 0xF8) | (prescaler & 0x07);
        break;

    case TIMER_1:
        // Stop timer
        TCCR1B = 0;
        TCNT1 = 0;

        // Configure mode
        switch (mode)
        {
        case TIMER_MODE_NORMAL:
            TCCR1A &= ~((1 << WGM11) | (1 << WGM10));
            TCCR1B &= ~((1 << WGM13) | (1 << WGM12));
            break;
        case TIMER_MODE_CTC:
            // WGM13:0 = 0100 (CTC, TOP = OCR1A)
            TCCR1A &= ~((1 << WGM11) | (1 << WGM10));
            TCCR1B &= ~(1 << WGM13);
            TCCR1B |= (1 << WGM12);
            break;
        case TIMER_MODE_FAST_PWM:
            // WGM13:0 = 1110 (Fast PWM, TOP = ICR1)
            TCCR1A |= (1 << WGM11);
            TCCR1A &= ~(1 << WGM10);
            TCCR1B |= (1 << WGM13) | (1 << WGM12);
            break;
        case TIMER_MODE_PHASE_CORRECT_PWM:
            // WGM13:0 = 1010 (Phase Correct PWM, TOP = ICR1)
            TCCR1A |= (1 << WGM11);
            TCCR1A &= ~(1 << WGM10);
            TCCR1B |= (1 << WGM13);
            TCCR1B &= ~(1 << WGM12);
            break;
        default:
            break;
        }

        // Set prescaler
        TCCR1B = (TCCR1B & 0xF8) | (prescaler & 0x07);
        break;

    case TIMER_2:
        // Stop timer
        TCCR2 = 0;
        TCNT2 = 0;

        // Configure mode
        switch (mode)
        {
        case TIMER_MODE_NORMAL:
            TCCR2 &= ~((1 << WGM21) | (1 << WGM20));
            break;
        case TIMER_MODE_CTC:
            TCCR2 |= (1 << WGM21);
            TCCR2 &= ~(1 << WGM20);
            break;
        case TIMER_MODE_FAST_PWM:
            TCCR2 |= (1 << WGM21) | (1 << WGM20);
            break;
        case TIMER_MODE_PHASE_CORRECT_PWM:
            TCCR2 &= ~(1 << WGM21);
            TCCR2 |= (1 << WGM20);
            break;
        default:
            break;
        }

        // Set prescaler
        TCCR2 = (TCCR2 & 0xF8) | (prescaler & 0x07);
        break;

    case TIMER_3:
        // Stop timer
        TCCR3B = 0;
        TCNT3 = 0;

        // Configure mode (same as Timer1)
        switch (mode)
        {
        case TIMER_MODE_NORMAL:
            TCCR3A &= ~((1 << WGM31) | (1 << WGM30));
            TCCR3B &= ~((1 << WGM33) | (1 << WGM32));
            break;
        case TIMER_MODE_CTC:
            TCCR3A &= ~((1 << WGM31) | (1 << WGM30));
            TCCR3B &= ~(1 << WGM33);
            TCCR3B |= (1 << WGM32);
            break;
        case TIMER_MODE_FAST_PWM:
            TCCR3A |= (1 << WGM31);
            TCCR3A &= ~(1 << WGM30);
            TCCR3B |= (1 << WGM33) | (1 << WGM32);
            break;
        case TIMER_MODE_PHASE_CORRECT_PWM:
            TCCR3A |= (1 << WGM31);
            TCCR3A &= ~(1 << WGM30);
            TCCR3B |= (1 << WGM33);
            TCCR3B &= ~(1 << WGM32);
            break;
        default:
            break;
        }

        // Set prescaler
        TCCR3B = (TCCR3B & 0xF8) | (prescaler & 0x07);
        break;
    }
}

void Timer_start(timer_id_t timer, uint16_t prescaler)
{
    switch (timer)
    {
    case TIMER_0:
        TCCR0 = (TCCR0 & 0xF8) | (prescaler & 0x07);
        break;
    case TIMER_1:
        TCCR1B = (TCCR1B & 0xF8) | (prescaler & 0x07);
        break;
    case TIMER_2:
        TCCR2 = (TCCR2 & 0xF8) | (prescaler & 0x07);
        break;
    case TIMER_3:
        TCCR3B = (TCCR3B & 0xF8) | (prescaler & 0x07);
        break;
    }
}

void Timer_stop(timer_id_t timer)
{
    switch (timer)
    {
    case TIMER_0:
        TCCR0 &= 0xF8; // Clear CS02:00
        break;
    case TIMER_1:
        TCCR1B &= 0xF8;
        break;
    case TIMER_2:
        TCCR2 &= 0xF8;
        break;
    case TIMER_3:
        TCCR3B &= 0xF8;
        break;
    }
}

void Timer_reset(timer_id_t timer)
{
    switch (timer)
    {
    case TIMER_0:
        TCNT0 = 0;
        break;
    case TIMER_1:
        TCNT1 = 0;
        break;
    case TIMER_2:
        TCNT2 = 0;
        break;
    case TIMER_3:
        TCNT3 = 0;
        break;
    }
}

uint16_t Timer_get_count(timer_id_t timer)
{
    switch (timer)
    {
    case TIMER_0:
        return TCNT0;
    case TIMER_1:
        return TCNT1;
    case TIMER_2:
        return TCNT2;
    case TIMER_3:
        return TCNT3;
    default:
        return 0;
    }
}

void Timer_set_count(timer_id_t timer, uint16_t value)
{
    switch (timer)
    {
    case TIMER_0:
        TCNT0 = (uint8_t)value;
        break;
    case TIMER_1:
        TCNT1 = value;
        break;
    case TIMER_2:
        TCNT2 = (uint8_t)value;
        break;
    case TIMER_3:
        TCNT3 = value;
        break;
    }
}

uint8_t Timer_is_running(timer_id_t timer)
{
    uint8_t cs_bits;
    switch (timer)
    {
    case TIMER_0:
        cs_bits = TCCR0 & 0x07;
        break;
    case TIMER_1:
        cs_bits = TCCR1B & 0x07;
        break;
    case TIMER_2:
        cs_bits = TCCR2 & 0x07;
        break;
    case TIMER_3:
        cs_bits = TCCR3B & 0x07;
        break;
    default:
        return 0;
    }
    return (cs_bits != 0);
}

/*
 * =============================================================================
 * COMPARE MATCH FUNCTIONS
 * =============================================================================
 */

void Timer_set_compare_value(timer_id_t timer, timer_compare_register_t compare_reg, uint16_t value)
{
    switch (timer)
    {
    case TIMER_0:
        if (compare_reg == TIMER_COMPARE_A)
            OCR0 = (uint8_t)value;
        break;
    case TIMER_1:
        switch (compare_reg)
        {
        case TIMER_COMPARE_A:
            OCR1A = value;
            break;
        case TIMER_COMPARE_B:
            OCR1B = value;
            break;
        case TIMER_COMPARE_C:
            OCR1C = value;
            break;
        }
        break;
    case TIMER_2:
        if (compare_reg == TIMER_COMPARE_A)
            OCR2 = (uint8_t)value;
        break;
    case TIMER_3:
        switch (compare_reg)
        {
        case TIMER_COMPARE_A:
            OCR3A = value;
            break;
        case TIMER_COMPARE_B:
            OCR3B = value;
            break;
        case TIMER_COMPARE_C:
            OCR3C = value;
            break;
        }
        break;
    }
}

uint16_t Timer_get_compare_value(timer_id_t timer, timer_compare_register_t compare_reg)
{
    switch (timer)
    {
    case TIMER_0:
        return OCR0;
    case TIMER_1:
        switch (compare_reg)
        {
        case TIMER_COMPARE_A:
            return OCR1A;
        case TIMER_COMPARE_B:
            return OCR1B;
        case TIMER_COMPARE_C:
            return OCR1C;
        default:
            return 0;
        }
    case TIMER_2:
        return OCR2;
    case TIMER_3:
        switch (compare_reg)
        {
        case TIMER_COMPARE_A:
            return OCR3A;
        case TIMER_COMPARE_B:
            return OCR3B;
        case TIMER_COMPARE_C:
            return OCR3C;
        default:
            return 0;
        }
    default:
        return 0;
    }
}

void Timer_set_compare_output_mode(timer_id_t timer, timer_compare_register_t compare_reg, timer_compare_output_t mode)
{
    switch (timer)
    {
    case TIMER_0:
        // Timer0 only has one compare output
        TCCR0 = (TCCR0 & ~(0x30)) | ((mode & 0x03) << 4);
        break;
    case TIMER_1:
        if (compare_reg == TIMER_COMPARE_A)
        {
            TCCR1A = (TCCR1A & ~(0xC0)) | ((mode & 0x03) << 6);
        }
        else if (compare_reg == TIMER_COMPARE_B)
        {
            TCCR1A = (TCCR1A & ~(0x30)) | ((mode & 0x03) << 4);
        }
        break;
    case TIMER_2:
        TCCR2 = (TCCR2 & ~(0x30)) | ((mode & 0x03) << 4);
        break;
    case TIMER_3:
        if (compare_reg == TIMER_COMPARE_A)
        {
            TCCR3A = (TCCR3A & ~(0xC0)) | ((mode & 0x03) << 6);
        }
        else if (compare_reg == TIMER_COMPARE_B)
        {
            TCCR3A = (TCCR3A & ~(0x30)) | ((mode & 0x03) << 4);
        }
        break;
    }
}

/*
 * =============================================================================
 * INPUT CAPTURE FUNCTIONS (Timer1 only)
 * =============================================================================
 */

void Timer1_input_capture_init(timer_input_capture_edge_t edge, uint8_t noise_cancel)
{
    // Set edge select
    if (edge == TIMER_IC_RISING_EDGE)
    {
        TCCR1B |= (1 << ICES1);
    }
    else
    {
        TCCR1B &= ~(1 << ICES1);
    }

    // Set noise canceler
    if (noise_cancel)
    {
        TCCR1B |= (1 << ICNC1);
    }
    else
    {
        TCCR1B &= ~(1 << ICNC1);
    }
}

uint16_t Timer1_get_capture_value(void)
{
    return ICR1;
}

void Timer1_set_capture_edge(timer_input_capture_edge_t edge)
{
    if (edge == TIMER_IC_RISING_EDGE)
    {
        TCCR1B |= (1 << ICES1);
    }
    else
    {
        TCCR1B &= ~(1 << ICES1);
    }
}

uint32_t Timer1_measure_frequency_Hz(void)
{
    // This is a simplified implementation
    // Real implementation would use interrupt-based capture
    uint16_t capture1, capture2;
    uint32_t period;

    // Wait for first capture
    TIFR |= (1 << ICF1); // Clear flag
    while (!(TIFR & (1 << ICF1)))
        ;
    capture1 = ICR1;

    // Wait for second capture
    TIFR |= (1 << ICF1);
    while (!(TIFR & (1 << ICF1)))
        ;
    capture2 = ICR1;

    // Calculate period in timer ticks
    if (capture2 > capture1)
    {
        period = capture2 - capture1;
    }
    else
    {
        period = (0xFFFF - capture1) + capture2;
    }

    // Convert to frequency (assumes prescaler 1)
    if (period > 0)
    {
        return F_CPU / period;
    }
    return 0;
}

uint32_t Timer1_measure_pulse_width_us(void)
{
    uint16_t capture1, capture2;
    uint32_t width;

    // Capture rising edge
    Timer1_set_capture_edge(TIMER_IC_RISING_EDGE);
    TIFR |= (1 << ICF1);
    while (!(TIFR & (1 << ICF1)))
        ;
    capture1 = ICR1;

    // Capture falling edge
    Timer1_set_capture_edge(TIMER_IC_FALLING_EDGE);
    TIFR |= (1 << ICF1);
    while (!(TIFR & (1 << ICF1)))
        ;
    capture2 = ICR1;

    // Calculate width
    if (capture2 > capture1)
    {
        width = capture2 - capture1;
    }
    else
    {
        width = (0xFFFF - capture1) + capture2;
    }

    // Convert to microseconds (assumes prescaler 1)
    return (width * 1000000UL) / F_CPU;
}

/*
 * =============================================================================
 * INTERRUPT MANAGEMENT
 * =============================================================================
 */

void Timer_enable_overflow_interrupt(timer_id_t timer)
{
    switch (timer)
    {
    case TIMER_0:
        TIMSK |= (1 << TOIE0);
        break;
    case TIMER_1:
        TIMSK |= (1 << TOIE1);
        break;
    case TIMER_2:
        TIMSK |= (1 << TOIE2);
        break;
    case TIMER_3:
        ETIMSK |= (1 << TOIE3);
        break;
    }
}

void Timer_disable_overflow_interrupt(timer_id_t timer)
{
    switch (timer)
    {
    case TIMER_0:
        TIMSK &= ~(1 << TOIE0);
        break;
    case TIMER_1:
        TIMSK &= ~(1 << TOIE1);
        break;
    case TIMER_2:
        TIMSK &= ~(1 << TOIE2);
        break;
    case TIMER_3:
        ETIMSK &= ~(1 << TOIE3);
        break;
    }
}

void Timer_enable_compare_interrupt(timer_id_t timer, timer_compare_register_t compare_reg)
{
    switch (timer)
    {
    case TIMER_0:
        TIMSK |= (1 << OCIE0);
        break;
    case TIMER_1:
        if (compare_reg == TIMER_COMPARE_A)
            TIMSK |= (1 << OCIE1A);
        else if (compare_reg == TIMER_COMPARE_B)
            TIMSK |= (1 << OCIE1B);
        break;
    case TIMER_2:
        TIMSK |= (1 << OCIE2);
        break;
    case TIMER_3:
        if (compare_reg == TIMER_COMPARE_A)
            ETIMSK |= (1 << OCIE3A);
        else if (compare_reg == TIMER_COMPARE_B)
            ETIMSK |= (1 << OCIE3B);
        break;
    }
}

void Timer_disable_compare_interrupt(timer_id_t timer, timer_compare_register_t compare_reg)
{
    switch (timer)
    {
    case TIMER_0:
        TIMSK &= ~(1 << OCIE0);
        break;
    case TIMER_1:
        if (compare_reg == TIMER_COMPARE_A)
            TIMSK &= ~(1 << OCIE1A);
        else if (compare_reg == TIMER_COMPARE_B)
            TIMSK &= ~(1 << OCIE1B);
        break;
    case TIMER_2:
        TIMSK &= ~(1 << OCIE2);
        break;
    case TIMER_3:
        if (compare_reg == TIMER_COMPARE_A)
            ETIMSK &= ~(1 << OCIE3A);
        else if (compare_reg == TIMER_COMPARE_B)
            ETIMSK &= ~(1 << OCIE3B);
        break;
    }
}

void Timer1_enable_capture_interrupt(void)
{
    TIMSK |= (1 << TICIE1);
}

void Timer1_disable_capture_interrupt(void)
{
    TIMSK &= ~(1 << TICIE1);
}

/*
 * =============================================================================
 * CALLBACK REGISTRATION
 * =============================================================================
 */

void Timer_register_overflow_callback(timer_id_t timer, timer_callback_t callback)
{
    if (timer < 4)
    {
        overflow_callbacks[timer] = callback;
    }
}

void Timer_register_compare_callback(timer_id_t timer, timer_compare_register_t compare_reg, timer_callback_t callback)
{
    if (timer < 4)
    {
        switch (compare_reg)
        {
        case TIMER_COMPARE_A:
            compare_a_callbacks[timer] = callback;
            break;
        case TIMER_COMPARE_B:
            compare_b_callbacks[timer] = callback;
            break;
        case TIMER_COMPARE_C:
            compare_c_callbacks[timer] = callback;
            break;
        }
    }
}

void Timer1_register_capture_callback(timer_capture_callback_t callback)
{
    capture_callback = callback;
}

/*
 * =============================================================================
 * INTERRUPT SERVICE ROUTINES
 * =============================================================================
 */

/* Timer0 Overflow */
ISR(TIMER0_OVF_vect)
{
    if (overflow_callbacks[TIMER_0])
    {
        overflow_callbacks[TIMER_0]();
    }
}

/* Timer0 Compare Match */
ISR(TIMER0_COMP_vect)
{
    if (compare_a_callbacks[TIMER_0])
    {
        compare_a_callbacks[TIMER_0]();
    }
}

/* Timer1 Overflow */
ISR(TIMER1_OVF_vect)
{
    if (overflow_callbacks[TIMER_1])
    {
        overflow_callbacks[TIMER_1]();
    }
}

/* Timer1 Compare Match A */
ISR(TIMER1_COMPA_vect)
{
    if (compare_a_callbacks[TIMER_1])
    {
        compare_a_callbacks[TIMER_1]();
    }
}

/* Timer1 Compare Match B */
ISR(TIMER1_COMPB_vect)
{
    if (compare_b_callbacks[TIMER_1])
    {
        compare_b_callbacks[TIMER_1]();
    }
}

/* Timer1 Input Capture */
ISR(TIMER1_CAPT_vect)
{
    if (capture_callback)
    {
        capture_callback(ICR1);
    }
}

/* Timer2 Overflow */
ISR(TIMER2_OVF_vect)
{
    if (overflow_callbacks[TIMER_2])
    {
        overflow_callbacks[TIMER_2]();
    }
}

/* Timer2 Compare Match */
ISR(TIMER2_COMP_vect)
{
    if (compare_a_callbacks[TIMER_2])
    {
        compare_a_callbacks[TIMER_2]();
    }
}

/* Timer3 Overflow */
ISR(TIMER3_OVF_vect)
{
    if (overflow_callbacks[TIMER_3])
    {
        overflow_callbacks[TIMER_3]();
    }
}

/* Timer3 Compare Match A */
ISR(TIMER3_COMPA_vect)
{
    if (compare_a_callbacks[TIMER_3])
    {
        compare_a_callbacks[TIMER_3]();
    }
}

/* Timer3 Compare Match B */
ISR(TIMER3_COMPB_vect)
{
    if (compare_b_callbacks[TIMER_3])
    {
        compare_b_callbacks[TIMER_3]();
    }
}

/*
 * =============================================================================
 * TIMING CALCULATION HELPERS
 * =============================================================================
 */

uint16_t Timer_calculate_prescaler(timer_id_t timer, uint32_t desired_freq_hz)
{
    // Try each prescaler to find best match
    uint16_t prescalers[] = {1, 8, 64, 256, 1024};
    uint16_t best_prescaler = 0;
    uint32_t min_error = 0xFFFFFFFF;

    for (uint8_t i = 0; i < 5; i++)
    {
        uint32_t timer_freq = F_CPU / prescalers[i];
        uint32_t max_freq, error;

        if (timer == TIMER_0 || timer == TIMER_2)
        {
            max_freq = timer_freq / 256; // 8-bit timer
        }
        else
        {
            max_freq = timer_freq / 65536; // 16-bit timer
        }

        if (max_freq < desired_freq_hz)
            continue;

        error = (max_freq > desired_freq_hz) ? (max_freq - desired_freq_hz) : (desired_freq_hz - max_freq);
        if (error < min_error)
        {
            min_error = error;
            best_prescaler = i + 1;
        }
    }

    return best_prescaler;
}

uint16_t Timer_calculate_compare_value(timer_id_t timer, uint16_t prescaler, uint32_t period_ms)
{
    uint16_t prescaler_values[] = {0, 1, 8, 64, 256, 1024};
    if (prescaler > 5)
        return 0;

    uint32_t timer_freq = F_CPU / prescaler_values[prescaler];
    uint32_t ticks_needed = (timer_freq * period_ms) / 1000;

    if (timer == TIMER_0 || timer == TIMER_2)
    {
        if (ticks_needed > 255)
            return 0; // 8-bit overflow
    }
    else
    {
        if (ticks_needed > 65535)
            return 0; // 16-bit overflow
    }

    return (uint16_t)ticks_needed;
}

uint32_t Timer_calculate_actual_frequency(timer_id_t timer, uint16_t prescaler, uint16_t compare_value)
{
    uint16_t prescaler_values[] = {0, 1, 8, 64, 256, 1024};
    if (prescaler > 5 || compare_value == 0)
        return 0;

    uint32_t timer_freq = F_CPU / prescaler_values[prescaler];
    return timer_freq / compare_value;
}

float Timer_get_resolution_us(timer_id_t timer, uint16_t prescaler)
{
    uint16_t prescaler_values[] = {0, 1, 8, 64, 256, 1024};
    if (prescaler > 5)
        return 0.0f;

    float timer_freq = (float)F_CPU / prescaler_values[prescaler];
    return 1000000.0f / timer_freq; // Period in microseconds
}

/*
 * =============================================================================
 * MILLISECOND TIMING SYSTEM
 * =============================================================================
 */

static void millis_overflow_handler(void)
{
    system_milliseconds++;
}

void Timer_millis_init(void)
{
    // Use Timer0 in CTC mode for 1ms interrupts
    // 16MHz / 64 = 250kHz, need 250 ticks for 1ms
    Timer_init(TIMER_0, TIMER_MODE_CTC, TIMER_PRESCALER_64);
    Timer_set_compare_value(TIMER_0, TIMER_COMPARE_A, 249); // 0-249 = 250 ticks
    Timer_register_compare_callback(TIMER_0, TIMER_COMPARE_A, millis_overflow_handler);
    Timer_enable_compare_interrupt(TIMER_0, TIMER_COMPARE_A);
    system_milliseconds = 0;
}

uint32_t Timer_millis(void)
{
    uint32_t ms;
    uint8_t sreg = SREG;
    cli(); // Atomic read
    ms = system_milliseconds;
    SREG = sreg;
    return ms;
}

uint32_t Timer_micros(void)
{
    uint32_t ms, us;
    uint8_t ticks;
    uint8_t sreg = SREG;

    cli();
    ms = system_milliseconds;
    ticks = TCNT0;
    SREG = sreg;

    // Each tick is 4us with prescaler 64 at 16MHz
    us = (ms * 1000UL) + (ticks * 4UL);
    return us;
}

void Timer_delay_ms(uint32_t ms)
{
    uint32_t start = Timer_millis();
    while ((Timer_millis() - start) < ms)
        ;
}

/*
 * =============================================================================
 * TASK SCHEDULER
 * =============================================================================
 */

void Timer_scheduler_init(timer_id_t timer)
{
    // Clear all tasks
    for (uint8_t i = 0; i < TIMER_MAX_TASKS; i++)
    {
        scheduler_tasks[i].callback = NULL;
        scheduler_tasks[i].enabled = 0;
    }
    scheduler_initialized = 1;
}

uint8_t Timer_scheduler_add_task(timer_callback_t callback, uint32_t interval_ms)
{
    if (!scheduler_initialized)
        return 0xFF;

    // Find free slot
    for (uint8_t i = 0; i < TIMER_MAX_TASKS; i++)
    {
        if (scheduler_tasks[i].callback == NULL)
        {
            scheduler_tasks[i].callback = callback;
            scheduler_tasks[i].interval_ms = interval_ms;
            scheduler_tasks[i].last_run_ms = Timer_millis();
            scheduler_tasks[i].enabled = 1;
            return i;
        }
    }
    return 0xFF; // Scheduler full
}

void Timer_scheduler_remove_task(uint8_t task_id)
{
    if (task_id < TIMER_MAX_TASKS)
    {
        scheduler_tasks[task_id].callback = NULL;
        scheduler_tasks[task_id].enabled = 0;
    }
}

void Timer_scheduler_enable_task(uint8_t task_id)
{
    if (task_id < TIMER_MAX_TASKS)
    {
        scheduler_tasks[task_id].enabled = 1;
    }
}

void Timer_scheduler_disable_task(uint8_t task_id)
{
    if (task_id < TIMER_MAX_TASKS)
    {
        scheduler_tasks[task_id].enabled = 0;
    }
}

void Timer_scheduler_update(void)
{
    uint32_t current_ms = Timer_millis();

    for (uint8_t i = 0; i < TIMER_MAX_TASKS; i++)
    {
        if (scheduler_tasks[i].callback && scheduler_tasks[i].enabled)
        {
            if ((current_ms - scheduler_tasks[i].last_run_ms) >= scheduler_tasks[i].interval_ms)
            {
                scheduler_tasks[i].callback();
                scheduler_tasks[i].last_run_ms = current_ms;
            }
        }
    }
}
