/*
 * =============================================================================
 * TIMER0 PROGRAMMING - POLLING VS INTERRUPT COMPARISON
 * =============================================================================
 *
 * PROJECT: Timer_Programming
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of ATmega128 Timer0 operations comparing POLLING
 * and INTERRUPT methods. Students learn timer control registers, various modes,
 * and the advantages of interrupt-driven programming.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master Timer0 configuration and control registers (TCCR0, TCNT0, OCR0)
 * 2. Learn prescaler settings for timing control
 * 3. Understand Normal, CTC, Fast PWM, and Phase Correct PWM modes
 * 4. Compare POLLING vs INTERRUPT programming methods
 * 5. Practice ISR programming with Timer0 overflow and compare match interrupts
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - LEDs on PORTB for visual timing indication
 * - Optional: Oscilloscope on OC0 pin (PB4) for PWM observation
 * - Serial connection for timing measurements (9600 baud)
 *
 * =============================================================================
 * TIMER0 CONTROL REGISTERS - DETAILED REFERENCE FOR STUDENTS
 * =============================================================================
 *
 * REGISTER 1: TCCR0 (Timer/Counter Control Register 0) - 8-BIT TIMER
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: FOC0  WGM00  COM01  COM00  WGM01   CS02   CS01   CS00
 *
 * FOC0 (bit 7): Force Output Compare
 *               Write 1 to force immediate compare match (non-PWM modes only)
 *
 * WGM01:00 (bits 6,3): Waveform Generation Mode - SELECT TIMER MODE
 *                      00 = Normal mode (count 0 to 255, overflow at TOP)
 *                      01 = PWM Phase Correct (count 0 to 255 to 0)
 *                      10 = CTC (Clear Timer on Compare, count 0 to OCR0)
 *                      11 = Fast PWM (count 0 to 255, PWM on OC0)
 *
 * COM01:00 (bits 5-4): Compare Match Output Mode
 *                      In Fast PWM: 10=Clear OC0 on match (non-inverting)
 *                                   11=Set OC0 on match (inverting)
 *                      In CTC: 01=Toggle OC0 on compare match
 *
 * CS02:00 (bits 2-0): Clock Select (Prescaler) - CRITICAL FOR TIMING
 *                     000 = No clock (timer stopped)
 *                     001 = clk/1 (no prescaling) - 16MHz
 *                     010 = clk/8 - 2MHz
 *                     011 = clk/64 - 250kHz
 *                     100 = clk/256 - 62.5kHz
 *                     101 = clk/1024 - 15.625kHz
 *
 * REGISTER 2: TCNT0 (Timer/Counter Register 0)
 * Current timer value (0-255). Read: value=TCNT0, Write: TCNT0=0
 *
 * REGISTER 3: OCR0 (Output Compare Register 0)
 * Compare value (0-255). In CTC: TOP value. In PWM: duty cycle.
 * Example: OCR0=128 gives 50% duty, OCR0=64 gives 25%, OCR0=192 gives 75%
 *
 * REGISTER 4: TIFR (Timer Interrupt Flag Register)
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name:  OCF2   TOV2   ICF1  OCF1A  OCF1B  TOV1   OCF0   TOV0
 *
 * TOV0 (bit 0): Timer0 Overflow Flag - POLLING MODE CHECK
 *               Set by hardware when timer overflows from 255 to 0
 *               Clear by writing 1: TIFR = (1<<TOV0);
 *               Check: test with (TIFR & (1<<TOV0))
 *
 * OCF0 (bit 1): Timer0 Output Compare Match Flag - CTC/PWM MODE
 *               Set when TCNT0 equals OCR0
 *               Clear by writing 1: TIFR = (1<<OCF0);
 *
 * REGISTER 5: TIMSK (Timer Interrupt Mask Register) - INTERRUPT ENABLE
 *
 *    Bit:   7       6       5       4       3       2       1       0
 *    Name: OCIE2  TOIE2  TICIE1  OCIE1A  OCIE1B  TOIE1   OCIE0   TOIE0
 *
 * TOIE0 (bit 0): Timer0 Overflow Interrupt Enable - INTERRUPT MODE
 *                Write 1 to enable overflow interrupt
 *                Usage: TIMSK = (1<<TOIE0); sei();
 *                Then define: ISR(TIMER0_OVF_vect) { code here }
 *
 * OCIE0 (bit 1): Timer0 Compare Match Interrupt Enable
 *                Write 1 to enable compare match interrupt
 *                Usage: TIMSK = (1<<OCIE0); sei();
 *                Then define: ISR(TIMER0_COMP_vect) { code here }
 *
 * CRITICAL: Must call sei() to enable global interrupts after setting TIMSK
 *
 * PRESCALER TIMING REFERENCE @ 16MHz FOR TIMER0:
 * clk/1:    Timer frequency 16MHz,    Overflow every 16 microseconds
 * clk/8:    Timer frequency 2MHz,     Overflow every 128 microseconds
 * clk/64:   Timer frequency 250kHz,   Overflow every 1.024 milliseconds
 * clk/256:  Timer frequency 62.5kHz,  Overflow every 4.096 milliseconds
 * clk/1024: Timer frequency 15.625kHz, Overflow every 16.384 milliseconds
 *
 * =============================================================================
 * DEMO PROGRAMS - POLLING VS INTERRUPT COMPARISON
 * =============================================================================
 * Demo 1: Normal Mode POLLING - Overflow flag polling
 * Demo 2: Normal Mode INTERRUPT - Overflow ISR
 * Demo 3: CTC Mode POLLING - Compare match flag polling
 * Demo 4: CTC Mode INTERRUPT - Compare match ISR
 * Demo 5: Fast PWM Mode - PWM generation
 * Demo 6: Phase Correct PWM - Symmetric PWM
 * Demo 7: Prescaler Comparison - All 5 prescaler settings
 * Demo 8: Multi-tasking with Overflow ISR
 * =============================================================================
 */

#include "config.h"
#include <stdio.h>

// Function prototypes
void demo1_normal_polling(void);
void demo2_normal_interrupt(void);
void demo3_ctc_polling(void);
void demo4_ctc_interrupt(void);
void demo5_fast_pwm(void);
void demo6_phase_correct_pwm(void);
void demo7_prescaler_comparison(void);
void demo8_multitask_interrupt(void);
void show_demo_menu(void);
void simple_init(void);

// Global variables for interrupt-driven demos
volatile uint16_t overflow_count = 0; // Overflow counter for demo2
volatile uint16_t compare_count = 0;  // Compare match counter for demo4
volatile uint16_t timer_ticks = 0;    // Tick counter for demo8
volatile uint8_t task1_flag = 0;      // Task flags for demo8
volatile uint8_t task2_flag = 0;
volatile uint8_t task3_flag = 0;

/*
 * =============================================================================
 * MAIN FUNCTION - DEMO SELECTOR
 * =============================================================================
 */
int main(void)
{
    simple_init();

    printf("\r\n");
    printf("=======================================================\r\n");
    printf("   TIMER0 PROGRAMMING - POLLING VS INTERRUPT\r\n");
    printf("   ATmega128 @ 16MHz - Educational Demonstration\r\n");
    printf("=======================================================\r\n");

    show_demo_menu();

    // Select demo to run (change this number to select different demo)
    int demo_choice = 1;

    printf("\r\nRunning Demo %d...\r\n\r\n", demo_choice);

    switch (demo_choice)
    {
    case 1:
        demo1_normal_polling();
        break;
    case 2:
        demo2_normal_interrupt();
        break;
    case 3:
        demo3_ctc_polling();
        break;
    case 4:
        demo4_ctc_interrupt();
        break;
    case 5:
        demo5_fast_pwm();
        break;
    case 6:
        demo6_phase_correct_pwm();
        break;
    case 7:
        demo7_prescaler_comparison();
        break;
    case 8:
        demo8_multitask_interrupt();
        break;
    default:
        printf("Invalid demo selection. Please choose 1-8.\r\n");
    }

    while (1)
    {
        // Demo runs continuously
    }

    return 0;
}

/*
 * =============================================================================
 * DEMO 1: NORMAL MODE - POLLING METHOD
 * =============================================================================
 * Method: POLLING - CPU actively checks TOV0 flag in loop
 * Demonstrates:
 * - Timer0 in Normal mode (counts 0->255, then overflows to 0)
 * - Polling the TOV0 flag in TIFR register
 * - Manual flag clearing by writing 1
 * - CPU is blocked waiting for overflow
 *
 * Timer Configuration:
 * - Mode: Normal (WGM01:00 = 00)
 * - Prescaler: 256 (CS02:00 = 100)
 * - Overflow frequency: 16MHz/256/256 = 244.14 Hz (4.096ms period)
 */
void demo1_normal_polling(void)
{
    uint16_t count = 0;

    printf("DEMO 1: Normal Mode - POLLING Method\r\n");
    printf("=====================================\r\n");
    printf("Timer0 Normal mode, prescaler 256\r\n");
    printf("POLLING TOV0 flag in TIFR register\r\n");
    printf("CPU BLOCKED waiting for overflow\r\n");
    printf("Overflow period: 4.096ms (244 Hz)\r\n\r\n");

    // Initialize Timer0
    TCCR0 = 0x00;       // Stop timer
    TCNT0 = 0;          // Clear counter
    TIFR = (1 << TOV0); // Clear overflow flag

    // Configure LED on PORTB.0
    DDRB |= (1 << 0);
    PORTB |= (1 << 0); // LED off (active low)

    // Start Timer0: Normal mode, prescaler 256
    TCCR0 = (1 << CS02);

    printf("Timer started. Polling TOV0 flag...\r\n");
    printf("Note: CPU cannot do other work while polling!\r\n\r\n");

    while (1)
    {
        // POLLING: Check overflow flag in loop
        if (TIFR & (1 << TOV0))
        {
            count++;

            // Clear flag by writing 1 to it
            TIFR = (1 << TOV0);

            // Toggle LED
            PORTB ^= (1 << 0);

            if (count % 100 == 0)
            {
                printf("Overflows: %u (Time: ~%.2f seconds) [POLLING]\r\n",
                       count, count * 0.004096);
            }
        }

        // CPU is stuck in this loop - cannot do other tasks!
    }
}

/*
 * =============================================================================
 * DEMO 2: NORMAL MODE - INTERRUPT METHOD
 * =============================================================================
 * Method: INTERRUPT - ISR automatically called on overflow
 * Demonstrates:
 * - Timer0 overflow interrupt (TOIE0)
 * - ISR(TIMER0_OVF_vect) - Interrupt Service Routine
 * - CPU is free to do other work
 * - Volatile variables for ISR communication
 *
 * Timer Configuration:
 * - Mode: Normal (WGM01:00 = 00)
 * - Prescaler: 256
 * - Interrupt: TOIE0 enabled
 * - Same timing as Demo 1, but CPU is not blocked!
 */

// Timer0 Overflow ISR
ISR(TIMER0_OVF_vect)
{
    // This executes automatically every overflow (every 4.096ms)
    overflow_count++;

    // Toggle LED in ISR
    PORTB ^= (1 << 1);
}

void demo2_normal_interrupt(void)
{
    printf("DEMO 2: Normal Mode - INTERRUPT Method\r\n");
    printf("=======================================\r\n");
    printf("Timer0 Normal mode, prescaler 256\r\n");
    printf("INTERRUPT with ISR(TIMER0_OVF_vect)\r\n");
    printf("CPU FREE to do other work!\r\n");
    printf("Overflow period: 4.096ms (244 Hz)\r\n\r\n");

    // Reset global counter
    overflow_count = 0;

    // Initialize Timer0
    TCCR0 = 0x00;
    TCNT0 = 0;
    TIFR = (1 << TOV0);

    // Configure LED on PORTB.1
    DDRB |= (1 << 1);
    PORTB |= (1 << 1);

    // Enable Timer0 overflow interrupt
    TIMSK = (1 << TOIE0);

    // Start Timer0: Normal mode, prescaler 256
    TCCR0 = (1 << CS02);

    // Enable global interrupts
    sei();

    printf("Timer interrupt enabled. ISR running automatically!\r\n");
    printf("Note: Main loop is FREE to do other tasks!\r\n\r\n");

    // Main loop can do other work
    uint32_t other_work_count = 0;
    while (1)
    {
        // CPU is free to do other tasks here!
        other_work_count++;

        // Print status periodically
        if (overflow_count % 100 == 0 && overflow_count > 0)
        {
            static uint16_t last_count = 0;
            if (last_count != overflow_count)
            {
                last_count = overflow_count;
                printf("Overflows: %u (Time: ~%.2f sec) [INTERRUPT] - Main loop work: %lu\r\n",
                       overflow_count, overflow_count * 0.004096, other_work_count);
                other_work_count = 0;
            }
        }

        // Simulate doing other work
        for (volatile uint16_t i = 0; i < 1000; i++)
        {
            asm volatile("nop");
        }
    }
}

/*
 * =============================================================================
 * DEMO 3: CTC MODE - POLLING METHOD
 * =============================================================================
 * Method: POLLING - CPU checks OCF0 flag
 * Demonstrates:
 * - Timer0 in CTC mode (Clear Timer on Compare)
 * - OCR0 register sets TOP value
 * - Polling OCF0 flag for compare match
 * - Precise frequency generation
 *
 * Timer Configuration:
 * - Mode: CTC (WGM01:00 = 10)
 * - Prescaler: 64
 * - OCR0: 249 (timer counts 0-249, then resets)
 * - Compare frequency: 16MHz/64/250 = 1000 Hz (1ms period)
 */
void demo3_ctc_polling(void)
{
    uint16_t count = 0;

    printf("DEMO 3: CTC Mode - POLLING Method\r\n");
    printf("==================================\r\n");
    printf("Timer0 CTC mode, prescaler 64, OCR0=249\r\n");
    printf("POLLING OCF0 flag in TIFR register\r\n");
    printf("Compare frequency: 1000 Hz (1ms period)\r\n\r\n");

    // Initialize Timer0
    TCCR0 = 0x00;
    TCNT0 = 0;
    OCR0 = 249;         // TOP value for 1ms period
    TIFR = (1 << OCF0); // Clear compare flag

    // Configure LED on PORTB.2
    DDRB |= (1 << 2);
    PORTB |= (1 << 2);

    // Start Timer0: CTC mode (WGM01=1), prescaler 64
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);

    printf("Timer started in CTC mode. Polling OCF0 flag...\r\n\r\n");

    while (1)
    {
        // POLLING: Check compare match flag
        if (TIFR & (1 << OCF0))
        {
            count++;

            // Clear flag
            TIFR = (1 << OCF0);

            // Toggle LED
            PORTB ^= (1 << 2);

            if (count % 1000 == 0)
            {
                printf("Compare matches: %u (Time: ~%u seconds) [POLLING]\r\n",
                       count, count / 1000);
            }
        }
    }
}

/*
 * =============================================================================
 * DEMO 4: CTC MODE - INTERRUPT METHOD
 * =============================================================================
 * Method: INTERRUPT - ISR called on compare match
 * Demonstrates:
 * - Timer0 compare match interrupt (OCIE0)
 * - ISR(TIMER0_COMP_vect) - Compare match ISR
 * - Precise timing with interrupts
 * - CPU free for other tasks
 *
 * Timer Configuration:
 * - Mode: CTC (WGM01:00 = 10)
 * - Prescaler: 64
 * - OCR0: 249
 * - Interrupt: OCIE0 enabled
 * - Same timing as Demo 3, but using interrupts!
 */

// Timer0 Compare Match ISR
ISR(TIMER0_COMP_vect)
{
    // Executes automatically on compare match (every 1ms)
    compare_count++;

    // Toggle LED in ISR
    PORTB ^= (1 << 3);
}

void demo4_ctc_interrupt(void)
{
    printf("DEMO 4: CTC Mode - INTERRUPT Method\r\n");
    printf("====================================\r\n");
    printf("Timer0 CTC mode, prescaler 64, OCR0=249\r\n");
    printf("INTERRUPT with ISR(TIMER0_COMP_vect)\r\n");
    printf("Compare frequency: 1000 Hz (1ms period)\r\n\r\n");

    // Reset global counter
    compare_count = 0;

    // Initialize Timer0
    TCCR0 = 0x00;
    TCNT0 = 0;
    OCR0 = 249;
    TIFR = (1 << OCF0);

    // Configure LED on PORTB.3
    DDRB |= (1 << 3);
    PORTB |= (1 << 3);

    // Enable Timer0 compare match interrupt
    TIMSK = (1 << OCIE0);

    // Start Timer0: CTC mode, prescaler 64
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);

    // Enable global interrupts
    sei();

    printf("Timer interrupt enabled. ISR running!\r\n\r\n");

    // Main loop is free
    uint32_t work_count = 0;
    while (1)
    {
        work_count++;

        if (compare_count % 1000 == 0 && compare_count > 0)
        {
            static uint16_t last = 0;
            if (last != compare_count)
            {
                last = compare_count;
                printf("Matches: %u (Time: ~%u sec) [INTERRUPT] - Work: %lu\r\n",
                       compare_count, compare_count / 1000, work_count);
                work_count = 0;
            }
        }

        for (volatile uint16_t i = 0; i < 1000; i++)
        {
            asm volatile("nop");
        }
    }
}

/*
 * =============================================================================
 * DEMO 5: FAST PWM MODE
 * =============================================================================
 * Demonstrates:
 * - Timer0 in Fast PWM mode
 * - Duty cycle control via OCR0
 * - Dynamic PWM adjustment (0% to 100% sweep)
 * - OC0 pin output (PB4)
 *
 * Timer Configuration:
 * - Mode: Fast PWM (WGM01:00 = 11)
 * - Prescaler: 64
 * - PWM frequency: 16MHz/64/256 = 976.5 Hz
 */
void demo5_fast_pwm(void)
{
    uint8_t duty = 0;

    printf("DEMO 5: Fast PWM Mode\r\n");
    printf("======================\r\n");
    printf("Timer0 Fast PWM, prescaler 64\r\n");
    printf("PWM frequency: 976.5 Hz\r\n");
    printf("Duty cycle: 0%% to 100%% sweep\r\n");
    printf("Observe PWM on OC0 pin (PB4)\r\n\r\n");

    // Initialize
    TCCR0 = 0x00;
    TCNT0 = 0;
    OCR0 = 0;

    // Configure PB4 (OC0) as output
    DDRB |= (1 << 4);

    // Configure PORTA for duty cycle display
    DDRA = 0xFF;
    PORTA = 0xFF;

    // Fast PWM: WGM01=1, WGM00=1, COM01=1 (non-inverting), prescaler 64
    TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << CS01) | (1 << CS00);

    printf("PWM started. Sweeping duty cycle...\r\n");

    while (1)
    {
        for (duty = 0; duty < 255; duty++)
        {
            OCR0 = duty;
            PORTA = ~duty;

            if (duty % 32 == 0)
            {
                printf("Duty: %u/255 (%.1f%%)\r\n", duty, (duty * 100.0) / 255.0);
            }

            for (volatile uint16_t i = 0; i < 10000; i++)
            {
                asm volatile("nop");
            }
        }

        for (duty = 255; duty > 0; duty--)
        {
            OCR0 = duty;
            PORTA = ~duty;

            if (duty % 32 == 0)
            {
                printf("Duty: %u/255 (%.1f%%)\r\n", duty, (duty * 100.0) / 255.0);
            }

            for (volatile uint16_t i = 0; i < 10000; i++)
            {
                asm volatile("nop");
            }
        }
    }
}

/*
 * =============================================================================
 * DEMO 6: PHASE CORRECT PWM MODE
 * =============================================================================
 * Demonstrates:
 * - Timer0 in Phase Correct PWM mode
 * - Symmetric counting (0->255->0)
 * - Lower frequency than Fast PWM (counts both directions)
 *
 * Timer Configuration:
 * - Mode: Phase Correct PWM (WGM01:00 = 01)
 * - Prescaler: 64
 * - PWM frequency: ~490 Hz (half of Fast PWM)
 */
void demo6_phase_correct_pwm(void)
{
    uint8_t duty = 128;

    printf("DEMO 6: Phase Correct PWM Mode\r\n");
    printf("===============================\r\n");
    printf("Timer0 Phase Correct PWM, prescaler 64\r\n");
    printf("PWM frequency: ~490 Hz (symmetric)\r\n");
    printf("Observe on OC0 pin (PB4)\r\n\r\n");

    TCCR0 = 0x00;
    TCNT0 = 0;
    OCR0 = duty;

    DDRB |= (1 << 4);

    // Phase Correct PWM: WGM01=0, WGM00=1, COM01=1, prescaler 64
    TCCR0 = (1 << WGM00) | (1 << COM01) | (1 << CS01) | (1 << CS00);

    printf("Phase Correct PWM started at 50%% duty\r\n");

    uint16_t count = 0;
    while (1)
    {
        duty = 128 + (count % 128);
        OCR0 = duty;

        if (count % 10 == 0)
        {
            printf("Duty: %u/255 (%.1f%%), Symmetric waveform\r\n",
                   duty, (duty * 100.0) / 255.0);
        }

        for (volatile uint16_t i = 0; i < 50000; i++)
        {
            asm volatile("nop");
        }

        count++;
    }
}

/*
 * =============================================================================
 * DEMO 7: PRESCALER COMPARISON
 * =============================================================================
 * Demonstrates all 5 prescaler values with Timer0
 */
void demo7_prescaler_comparison(void)
{
    const char *names[] = {"1", "8", "64", "256", "1024"};
    const uint8_t values[] = {
        (1 << CS00),
        (1 << CS01),
        (1 << CS01) | (1 << CS00),
        (1 << CS02),
        (1 << CS02) | (1 << CS00)};
    const float periods[] = {0.016, 0.128, 1.024, 4.096, 16.384};

    printf("DEMO 7: Prescaler Comparison\r\n");
    printf("=============================\r\n");
    printf("Timer0 with all 5 prescaler values\r\n\r\n");

    DDRB |= (1 << 5);
    PORTB |= (1 << 5);

    while (1)
    {
        for (uint8_t i = 0; i < 5; i++)
        {
            printf("Prescaler: 1/%s, Period: %.3f ms, Freq: %.2f Hz\r\n",
                   names[i], periods[i], 1000.0 / periods[i]);

            TCCR0 = 0x00;
            TCNT0 = 0;
            TIFR = (1 << TOV0);
            TCCR0 = values[i];

            for (uint16_t j = 0; j < 50; j++)
            {
                while (!(TIFR & (1 << TOV0)))
                    ;
                TIFR = (1 << TOV0);
                PORTB ^= (1 << 5);
            }

            printf("  50 overflows done (~%.2f sec)\r\n\r\n",
                   50 * periods[i] / 1000.0);
        }
    }
}

/*
 * =============================================================================
 * DEMO 8: MULTI-TASKING WITH TIMER0 OVERFLOW INTERRUPT
 * =============================================================================
 * Demonstrates:
 * - ISR-based task scheduling
 * - Multiple tasks at different rates
 * - Timer0 overflow interrupt for periodic execution
 *
 * Uses Timer0 overflow with prescaler 64 for ~1ms timing
 */

// Timer0 overflow ISR for multi-tasking
ISR(TIMER0_OVF_vect)
{
    // Reload timer for ~1ms timing
    // With prescaler 64: 16MHz/64 = 250kHz
    // For 1ms: need 250 counts
    // Preload: 256-250 = 6
    TCNT0 = 6;

    timer_ticks++;

    if (timer_ticks % 1 == 0)
    {
        task1_flag = 1; // Every 1ms
    }
    if (timer_ticks % 100 == 0)
    {
        task2_flag = 1; // Every 100ms
    }
    if (timer_ticks % 500 == 0)
    {
        task3_flag = 1; // Every 500ms
    }
}

void demo8_multitask_interrupt(void)
{
    uint32_t t1_count = 0, t2_count = 0, t3_count = 0;

    printf("DEMO 8: Multi-tasking with Timer0 Overflow ISR\r\n");
    printf("===============================================\r\n");
    printf("Timer0 overflow ISR at ~1ms interval\r\n");
    printf("Task 1: 1ms rate, Task 2: 100ms rate, Task 3: 500ms rate\r\n\r\n");

    timer_ticks = 0;

    DDRB |= (1 << 6) | (1 << 7);
    PORTB |= (1 << 6) | (1 << 7);
    DDRA |= (1 << 0);
    PORTA |= (1 << 0);

    // Initialize Timer0
    TCCR0 = 0x00;
    TCNT0 = 6;
    TIFR = (1 << TOV0);

    // Enable overflow interrupt
    TIMSK = (1 << TOIE0);

    // Start with prescaler 64
    TCCR0 = (1 << CS01) | (1 << CS00);

    sei();

    printf("Multi-tasking started!\r\n\r\n");

    while (1)
    {
        if (task1_flag)
        {
            task1_flag = 0;
            t1_count++;
            if (t1_count % 500 == 0)
            {
                PORTA ^= (1 << 0);
                printf("Task1 (1ms): %lu\r\n", t1_count);
            }
        }

        if (task2_flag)
        {
            task2_flag = 0;
            t2_count++;
            PORTB ^= (1 << 6);
            printf("Task2 (100ms): %lu\r\n", t2_count);
        }

        if (task3_flag)
        {
            task3_flag = 0;
            t3_count++;
            PORTB ^= (1 << 7);
            printf("Task3 (500ms): %lu - System time: %.3f sec\r\n\r\n",
                   t3_count, timer_ticks / 1000.0);
        }
    }
}

/*
 * =============================================================================
 * HELPER FUNCTIONS
 * =============================================================================
 */

void show_demo_menu(void)
{
    printf("\r\nAvailable Demos (Timer0 Only):\r\n");
    printf("1. Normal Mode - POLLING (flag check in loop)\r\n");
    printf("2. Normal Mode - INTERRUPT (ISR automatic)\r\n");
    printf("3. CTC Mode - POLLING (compare flag check)\r\n");
    printf("4. CTC Mode - INTERRUPT (compare ISR)\r\n");
    printf("5. Fast PWM Mode\r\n");
    printf("6. Phase Correct PWM Mode\r\n");
    printf("7. Prescaler Comparison (all 5 values)\r\n");
    printf("8. Multi-tasking with Overflow ISR\r\n");
    printf("\r\nChange 'demo_choice' in main() to select.\r\n\r\n");
}

void simple_init(void)
{
    Uart1_init();

    DDRB = 0xFF;
    PORTB = 0xFF;

    DDRA = 0xFF;
    PORTA = 0xFF;

    for (volatile uint32_t i = 0; i < 100000; i++)
    {
        asm volatile("nop");
    }
}
