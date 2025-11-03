/*
 * =============================================================================
 * TIMER/COUNTER PROGRAMMING - COMPREHENSIVE EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Timer_Programming
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Comprehensive educational demonstration of ATmega128 timer/counter operations,
 * control registers, and various timer modes. Students learn precise timing
 * generation, PWM, CTC mode, and interrupt-driven timer programming.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master timer/counter configuration and control registers (TCCR, TCNT, etc.)
 * 2. Learn prescaler settings for timing control
 * 3. Understand Normal, CTC, Fast PWM, and Phase Correct PWM modes
 * 4. Practice interrupt-driven timer programming
 * 5. Generate precise timing delays and frequencies
 * 6. Compare Timer0, Timer1 (16-bit), and Timer2 characteristics
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - LEDs on PORTB for visual timing indication
 * - Optional: Oscilloscope on OC0, OC1A, OC2 pins for PWM observation
 * - Serial connection for timing measurements (9600 baud)
 *
 * ATMEGA128 TIMER OVERVIEW:
 * - Timer0: 8-bit timer with PWM capability
 * - Timer1: 16-bit timer with advanced PWM and input capture
 * - Timer2: 8-bit timer with asynchronous operation capability
 * - Timer3: 16-bit timer similar to Timer1
 *
 * =============================================================================
 * TIMER CONTROL REGISTERS - DETAILED REFERENCE FOR STUDENTS
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
 *                     001 = clk/1 (no prescaling)
 *                     010 = clk/8
 *                     011 = clk/64
 *                     100 = clk/256
 *                     101 = clk/1024
 *
 * REGISTER 2: TCNT0 (Timer/Counter Register 0)
 * Current timer value (0-255). Read: value=TCNT0, Write: TCNT0=0
 *
 * REGISTER 3: OCR0 (Output Compare Register 0)
 * Compare value (0-255). In CTC: TOP value. In PWM: duty cycle.
 * Example: OCR0=128 gives 50% duty, OCR0=64 gives 25%, OCR0=192 gives 75%
 *
 * REGISTER 4: TIFR (Timer Interrupt Flag Register) - SHARED BY ALL TIMERS
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
 * TOIE1 (bit 2): Timer1 Overflow Interrupt Enable
 * OCIE1B (bit 3): Timer1 Compare Match B Interrupt Enable
 * OCIE1A (bit 4): Timer1 Compare Match A Interrupt Enable
 * TICIE1 (bit 5): Timer1 Input Capture Interrupt Enable
 * TOIE2 (bit 6): Timer2 Overflow Interrupt Enable - used in demo6
 * OCIE2 (bit 7): Timer2 Compare Match Interrupt Enable
 *
 * CRITICAL: Must call sei() to enable global interrupts after setting TIMSK
 *
 * TIMER1 CONTROL REGISTERS (16-BIT TIMER) - ADVANCED FEATURES
 *
 * REGISTER 6: TCCR1A (Timer/Counter 1 Control Register A)
 *
 *    Bit:   7       6       5       4       3       2       1       0
 *    Name: COM1A1 COM1A0 COM1B1 COM1B0  FOC1A   FOC1B  WGM11   WGM10
 *
 * COM1A1:0: Compare Output Mode for Channel A (OC1A pin)
 * COM1B1:0: Compare Output Mode for Channel B (OC1B pin)
 * WGM11:10: Waveform Generation Mode bits (combine with TCCR1B)
 *
 * REGISTER 7: TCCR1B (Timer/Counter 1 Control Register B)
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: ICNC1  ICES1   -    WGM13  WGM12   CS12   CS11   CS10
 *
 * WGM13:10: Waveform Generation Mode (4 bits total)
 *           0000 = Normal (16-bit, 0 to 65535)
 *           0100 = CTC (OCR1A is TOP)
 *           1110 = Fast PWM (ICR1 is TOP)
 *
 * CS12:10: Clock Select (Prescaler) - Same as Timer0
 *          000=Stop, 001=clk/1, 010=clk/8, 011=clk/64, 100=clk/256, 101=clk/1024
 *
 * ICNC1 (bit 7): Input Capture Noise Canceler
 * ICES1 (bit 6): Input Capture Edge Select
 *
 * REGISTER 8: TCNT1 (Timer/Counter 1 Register - 16-bit)
 * Access: uint16_t value = TCNT1; (compiler handles both bytes)
 * Range: 0 to 65535 (versus 0-255 for 8-bit timers)
 *
 * PRESCALER TIMING REFERENCE @ 16MHz:
 * clk/1: Timer0 overflow every 16us, Timer1 overflow every 4.096ms
 * clk/8: Timer0 overflow every 128us, Timer1 overflow every 32.768ms
 * clk/64: Timer0 overflow every 1.024ms, Timer1 overflow every 262.144ms
 * clk/256: Timer0 overflow every 4.096ms, Timer1 overflow every 1.048s
 * clk/1024: Timer0 overflow every 16.384ms, Timer1 overflow every 4.194s
 *
 * =============================================================================
 * DEMO PROGRAMS - LEARNING PROGRESSION
 * =============================================================================
 * Demo 1: Normal Mode - Basic overflow timing with polling
 * Demo 2: CTC Mode - Clear Timer on Compare Match
 * Demo 3: Fast PWM Mode - High-frequency PWM generation
 * Demo 4: Phase Correct PWM - Symmetric PWM waveforms
 * Demo 5: Prescaler Comparison - All 5 prescaler demonstrations
 * Demo 6: Interrupt Multi-tasking - Timer2 ISR-based scheduling
 * Demo 7: Timer1 Advanced - 16-bit timer features
 * =============================================================================
 */

#include "config.h"
#include <stdio.h>

// Function prototypes for demo programs
void demo1_normal_mode(void);
void demo2_ctc_mode(void);
void demo3_fast_pwm(void);
void demo4_phase_correct_pwm(void);
void demo5_prescaler_comparison(void);
void demo6_interrupt_multitask(void);
void demo7_timer1_advanced(void);
void show_demo_menu(void);
void simple_init(void);

// Global variables for interrupt-driven demos
volatile uint16_t timer2_ticks = 0; // Tick counter for demo6
volatile uint8_t task1_flag = 0;    // Task flags for multitasking demo
volatile uint8_t task2_flag = 0;
volatile uint8_t task3_flag = 0;

/*
 * =============================================================================
 * MAIN FUNCTION - DEMO SELECTOR
 * =============================================================================
 */
int main(void)
{
    simple_init(); // Initialize UART and ports

    printf("\r\n");
    printf("=======================================================\r\n");
    printf("   TIMER PROGRAMMING - COMPREHENSIVE DEMONSTRATION\r\n");
    printf("   ATmega128 @ 16MHz - Timer0, Timer1, Timer2\r\n");
    printf("=======================================================\r\n");

    show_demo_menu();

    // Select demo to run (change this number to select different demo)
    int demo_choice = 1;

    printf("\r\nRunning Demo %d...\r\n\r\n", demo_choice);

    switch (demo_choice)
    {
    case 1:
        demo1_normal_mode();
        break;
    case 2:
        demo2_ctc_mode();
        break;
    case 3:
        demo3_fast_pwm();
        break;
    case 4:
        demo4_phase_correct_pwm();
        break;
    case 5:
        demo5_prescaler_comparison();
        break;
    case 6:
        demo6_interrupt_multitask();
        break;
    case 7:
        demo7_timer1_advanced();
        break;
    default:
        printf("Invalid demo selection. Please choose 1-7.\r\n");
    }

    while (1)
    {
        // Demo runs continuously
    }

    return 0;
}

/*
 * =============================================================================
 * DEMO 1: NORMAL MODE - BASIC OVERFLOW TIMING WITH POLLING
 * =============================================================================
 * Demonstrates:
 * - Timer0 in Normal mode (counts 0->255, then overflows to 0)
 * - Polling the TOV0 flag in TIFR register
 * - Manual flag clearing
 * - LED toggling based on overflow events
 *
 * Timer Configuration:
 * - Mode: Normal (WGM01:00 = 00)
 * - Prescaler: 256 (CS02:00 = 100)
 * - Overflow frequency: 16MHz/256/256 = 244.14 Hz (4.096ms period)
 */
void demo1_normal_mode(void)
{
    uint16_t overflow_count = 0;

    printf("DEMO 1: Normal Mode - Overflow Polling\r\n");
    printf("---------------------------------------\r\n");
    printf("Timer0 in Normal mode, prescaler 256\r\n");
    printf("Polling TOV0 flag, toggling LED on overflow\r\n");
    printf("Overflow period: 4.096ms (244 Hz)\r\n\r\n");

    // Initialize Timer0
    TCCR0 = 0x00;       // Stop timer
    TCNT0 = 0;          // Clear timer counter
    TIFR = (1 << TOV0); // Clear any pending overflow flag

    // Configure PORTB.0 as output for LED
    DDRB |= (1 << 0);
    PORTB |= (1 << 0); // LED off (active low)

    // Start Timer0: Normal mode, prescaler 256
    // TCCR0: FOC0=0, WGM00=0, COM01=0, COM00=0, WGM01=0, CS02=1, CS01=0, CS00=0
    TCCR0 = (1 << CS02); // Start with prescaler 256

    printf("Timer started. Monitoring overflows...\r\n");

    while (1)
    {
        // Poll the Timer0 Overflow Flag (TOV0)
        if (TIFR & (1 << TOV0))
        {
            // Overflow occurred!
            overflow_count++;

            // Clear the flag by WRITING 1 to it (this is ATmega convention)
            TIFR = (1 << TOV0);

            // Toggle LED
            PORTB ^= (1 << 0);

            // Print status every 100 overflows (approximately 0.4 seconds)
            if (overflow_count % 100 == 0)
            {
                printf("Overflows: %u (Time: ~%.2f seconds)\r\n",
                       overflow_count, overflow_count * 0.004096);
            }
        }
    }
}

/*
 * =============================================================================
 * DEMO 2: CTC MODE - CLEAR TIMER ON COMPARE MATCH
 * =============================================================================
 * Demonstrates:
 * - Timer0 in CTC mode (Clear Timer on Compare)
 * - OCR0 register usage to set TOP value
 * - Precise frequency generation
 * - OCF0 flag polling for compare match detection
 *
 * Timer Configuration:
 * - Mode: CTC (WGM01:00 = 10)
 * - Prescaler: 64 (CS02:00 = 011)
 * - OCR0: 249 (timer counts 0 to 249, then resets)
 * - Compare frequency: 16MHz/64/250 = 1000 Hz (1ms period)
 */
void demo2_ctc_mode(void)
{
    uint16_t compare_count = 0;

    printf("DEMO 2: CTC Mode - Clear Timer on Compare\r\n");
    printf("------------------------------------------\r\n");
    printf("Timer0 in CTC mode, prescaler 64, OCR0=249\r\n");
    printf("Polling OCF0 flag, toggling LED on compare match\r\n");
    printf("Compare frequency: 1000 Hz (1ms period)\r\n\r\n");

    // Initialize Timer0
    TCCR0 = 0x00; // Stop timer
    TCNT0 = 0;    // Clear counter
    OCR0 = 249;   // Set compare value (TOP)
                  // Timer will count 0->249 (250 counts)

    TIFR = (1 << OCF0); // Clear compare match flag

    // Configure PORTB.1 as output
    DDRB |= (1 << 1);
    PORTB |= (1 << 1);

    // Start Timer0: CTC mode (WGM01=1), prescaler 64 (CS01=1, CS00=1)
    // TCCR0: FOC0=0, WGM00=0, COM01=0, COM00=0, WGM01=1, CS02=0, CS01=1, CS00=1
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);

    printf("Timer started in CTC mode. Monitoring compare matches...\r\n");

    while (1)
    {
        // Poll the Output Compare Flag (OCF0)
        if (TIFR & (1 << OCF0))
        {
            // Compare match occurred! TCNT0 has been reset to 0
            compare_count++;

            // Clear the flag
            TIFR = (1 << OCF0);

            // Toggle LED
            PORTB ^= (1 << 1);

            // Print every 1000 matches (1 second)
            if (compare_count % 1000 == 0)
            {
                printf("Compare matches: %u (Time: ~%u seconds)\r\n",
                       compare_count, compare_count / 1000);
            }
        }
    }
}

/*
 * =============================================================================
 * DEMO 3: FAST PWM MODE - HIGH-FREQUENCY PWM GENERATION
 * =============================================================================
 * Demonstrates:
 * - Timer0 in Fast PWM mode
 * - Duty cycle control via OCR0 register
 * - Dynamic PWM adjustment (sweep from 0% to 100%)
 * - Non-inverting PWM output on OC0 pin (PB4)
 *
 * Timer Configuration:
 * - Mode: Fast PWM (WGM01:00 = 11)
 * - Prescaler: 64 (CS02:00 = 011)
 * - PWM frequency: 16MHz/64/256 = 976.5 Hz
 * - Duty cycle: Varies from 0% to 100% via OCR0
 */
void demo3_fast_pwm(void)
{
    uint8_t duty = 0;
    uint16_t delay;

    printf("DEMO 3: Fast PWM Mode - PWM Generation\r\n");
    printf("---------------------------------------\r\n");
    printf("Timer0 in Fast PWM mode, prescaler 64\r\n");
    printf("PWM frequency: 976.5 Hz\r\n");
    printf("Duty cycle sweeping from 0%% to 100%%\r\n");
    printf("Observe PWM on OC0 pin (PB4) with oscilloscope\r\n\r\n");

    // Initialize Timer0
    TCCR0 = 0x00; // Stop timer
    TCNT0 = 0;
    OCR0 = 0; // Start with 0% duty cycle

    // Configure PB4 (OC0) as output for PWM
    DDRB |= (1 << 4);

    // Configure PORTA for LED duty cycle indicator
    DDRA = 0xFF;
    PORTA = 0xFF; // All LEDs off

    // Start Timer0: Fast PWM mode, non-inverting, prescaler 64
    // Fast PWM mode: WGM01=1, WGM00=1
    // Non-inverting: COM01=1, COM00=0 (Clear OC0 on compare, set at BOTTOM)
    // Prescaler 64: CS02=0, CS01=1, CS00=1
    TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << CS01) | (1 << CS00);

    printf("PWM started. Sweeping duty cycle...\r\n");

    while (1)
    {
        // Sweep duty cycle from 0% to 100%
        for (duty = 0; duty < 255; duty++)
        {
            OCR0 = duty; // Set PWM duty cycle

            // Display duty cycle on LEDs (8-bit value)
            PORTA = ~duty; // Inverted for active-low LEDs

            // Print duty cycle every 32 steps
            if (duty % 32 == 0)
            {
                printf("Duty cycle: %u/255 (%.1f%%)\r\n",
                       duty, (duty * 100.0) / 255.0);
            }

            // Small delay to make sweep visible
            for (delay = 0; delay < 10000; delay++)
            {
                asm volatile("nop");
            }
        }

        // Sweep back down from 100% to 0%
        for (duty = 255; duty > 0; duty--)
        {
            OCR0 = duty;
            PORTA = ~duty;

            if (duty % 32 == 0)
            {
                printf("Duty cycle: %u/255 (%.1f%%)\r\n",
                       duty, (duty * 100.0) / 255.0);
            }

            for (delay = 0; delay < 10000; delay++)
            {
                asm volatile("nop");
            }
        }
    }
}

/*
 * =============================================================================
 * DEMO 4: PHASE CORRECT PWM MODE - SYMMETRIC PWM WAVEFORMS
 * =============================================================================
 * Demonstrates:
 * - Timer0 in Phase Correct PWM mode
 * - Symmetric counting (0->255->0)
 * - Suitable for motor control (reduced harmonics)
 * - Comparison with Fast PWM
 *
 * Timer Configuration:
 * - Mode: PWM Phase Correct (WGM01:00 = 01)
 * - Prescaler: 64
 * - PWM frequency: 16MHz/64/510 = 490 Hz (half of Fast PWM)
 * - Duty cycle: Controlled via OCR0
 */
void demo4_phase_correct_pwm(void)
{
    uint8_t duty = 128; // Start at 50% duty cycle

    printf("DEMO 4: Phase Correct PWM Mode\r\n");
    printf("-------------------------------\r\n");
    printf("Timer0 in Phase Correct PWM, prescaler 64\r\n");
    printf("PWM frequency: ~490 Hz (symmetric counting)\r\n");
    printf("Compare with Fast PWM: symmetric vs asymmetric\r\n");
    printf("Observe on OC0 pin (PB4)\r\n\r\n");

    // Initialize
    TCCR0 = 0x00;
    TCNT0 = 0;
    OCR0 = duty;

    DDRB |= (1 << 4); // OC0 output

    // Start Timer0: Phase Correct PWM, non-inverting, prescaler 64
    // Phase Correct PWM: WGM01=0, WGM00=1
    // Non-inverting: COM01=1, COM00=0
    // Prescaler 64: CS02=0, CS01=1, CS00=1
    TCCR0 = (1 << WGM00) | (1 << COM01) | (1 << CS01) | (1 << CS00);

    printf("Phase Correct PWM started at 50%% duty\r\n");
    printf("Note: Frequency is half of Fast PWM (counts up AND down)\r\n");

    uint16_t count = 0;
    while (1)
    {
        uint16_t delay;

        // Slowly vary duty cycle to show waveform changes
        duty = 128 + (count % 128); // Vary between 50% and 100%
        OCR0 = duty;

        if (count % 10 == 0)
        {
            printf("Duty: %u/255 (%.1f%%), Symmetric waveform\r\n",
                   duty, (duty * 100.0) / 255.0);
        }

        for (delay = 0; delay < 50000; delay++)
        {
            asm volatile("nop");
        }

        count++;
    }
}

/*
 * =============================================================================
 * DEMO 5: PRESCALER COMPARISON - ALL 5 PRESCALER SETTINGS
 * =============================================================================
 * Demonstrates:
 * - All 5 prescaler values (1, 8, 64, 256, 1024)
 * - Timing differences with different prescalers
 * - Overflow frequency calculation verification
 * - Real-time prescaler switching
 */
void demo5_prescaler_comparison(void)
{
    const char *prescaler_names[] = {"1", "8", "64", "256", "1024"};
    const uint8_t prescaler_values[] = {
        (1 << CS00),               // clk/1
        (1 << CS01),               // clk/8
        (1 << CS01) | (1 << CS00), // clk/64
        (1 << CS02),               // clk/256
        (1 << CS02) | (1 << CS00)  // clk/1024
    };
    const float overflow_periods[] = {0.016, 0.128, 1.024, 4.096, 16.384}; // in milliseconds

    printf("DEMO 5: Prescaler Comparison\r\n");
    printf("-----------------------------\r\n");
    printf("Cycling through all 5 prescaler values\r\n");
    printf("Observe LED blink rate changes\r\n\r\n");

    DDRB |= (1 << 2); // LED output
    PORTB |= (1 << 2);

    while (1)
    {
        for (uint8_t i = 0; i < 5; i++)
        {
            printf("\r\nPrescaler: 1/%s\r\n", prescaler_names[i]);
            printf("Overflow period: %.3f ms\r\n", overflow_periods[i]);
            printf("Overflow frequency: %.2f Hz\r\n", 1000.0 / overflow_periods[i]);

            // Configure timer with current prescaler
            TCCR0 = 0x00;       // Stop
            TCNT0 = 0;          // Reset
            TIFR = (1 << TOV0); // Clear flag

            TCCR0 = prescaler_values[i]; // Start with new prescaler

            // Count 50 overflows with this prescaler
            for (uint16_t overflow = 0; overflow < 50; overflow++)
            {
                while (!(TIFR & (1 << TOV0)))
                {
                    // Wait for overflow
                }
                TIFR = (1 << TOV0); // Clear flag
                PORTB ^= (1 << 2);  // Toggle LED
            }

            printf("Completed 50 overflows (~%.2f seconds)\r\n",
                   50 * overflow_periods[i] / 1000.0);
        }
    }
}

/*
 * =============================================================================
 * DEMO 6: INTERRUPT-DRIVEN MULTI-TASKING WITH TIMER2
 * =============================================================================
 * Demonstrates:
 * - Timer2 overflow interrupt (ISR)
 * - ISR-based task scheduling
 * - Multiple tasks with different execution rates
 * - Volatile variables for ISR communication
 * - sei() for global interrupt enable
 *
 * Timer Configuration:
 * - Timer2 in Normal mode
 * - Prescaler: 64
 * - Overflow interrupt enabled (TOIE2)
 * - Tasks triggered at different rates (1ms, 100ms, 500ms, 1000ms)
 */

// Timer2 overflow ISR - executes every ~1ms
ISR(TIMER2_OVF_vect)
{
    // Reload timer for precise 1ms timing
    // With prescaler 64: 16MHz/64 = 250kHz timer clock
    // For 1ms: need 250 counts
    // Preload with 256-250 = 6
    TCNT2 = 6;

    timer2_ticks++;

    // Schedule tasks at different rates
    if (timer2_ticks % 1 == 0)
    {
        task1_flag = 1; // Execute every 1ms (fast task)
    }
    if (timer2_ticks % 100 == 0)
    {
        task2_flag = 1; // Execute every 100ms (medium task)
    }
    if (timer2_ticks % 500 == 0)
    {
        task3_flag = 1; // Execute every 500ms (slow task)
    }
}

void demo6_interrupt_multitask(void)
{
    uint32_t task1_count = 0;
    uint32_t task2_count = 0;
    uint32_t task3_count = 0;

    printf("DEMO 6: Interrupt-Driven Multi-Tasking\r\n");
    printf("---------------------------------------\r\n");
    printf("Timer2 overflow ISR at ~1ms interval\r\n");
    printf("Task 1: 1ms rate (fast blink)\r\n");
    printf("Task 2: 100ms rate (medium blink)\r\n");
    printf("Task 3: 500ms rate (slow blink)\r\n\r\n");

    // Configure LEDs on PORTB
    DDRB |= (1 << 3) | (1 << 4) | (1 << 5);  // Three LEDs
    PORTB |= (1 << 3) | (1 << 4) | (1 << 5); // All off

    // Initialize Timer2
    TCCR2 = 0x00;       // Stop timer
    TCNT2 = 6;          // Preload for 1ms with prescaler 64
    TIFR = (1 << TOV2); // Clear overflow flag

    // Enable Timer2 overflow interrupt
    TIMSK = (1 << TOIE2);

    // Start Timer2 with prescaler 64
    // CS22=0, CS21=1, CS20=1
    TCCR2 = (1 << CS22); // Actually this is prescaler 64 for Timer2

    // Enable global interrupts
    sei();

    printf("Timer2 interrupt enabled. Tasks running...\r\n\r\n");

    while (1)
    {
        // Task 1: Fast task (1ms rate)
        if (task1_flag)
        {
            task1_flag = 0;
            task1_count++;
            // Fast LED toggle (too fast to see, but executes)
            if (task1_count % 500 == 0)
            { // Toggle every 500ms
                PORTB ^= (1 << 3);
                printf("Task 1 toggle (1ms rate, count: %lu)\r\n", task1_count);
            }
        }

        // Task 2: Medium task (100ms rate)
        if (task2_flag)
        {
            task2_flag = 0;
            task2_count++;
            PORTB ^= (1 << 4); // Toggle LED2
            printf("Task 2 toggle (100ms rate, count: %lu)\r\n", task2_count);
        }

        // Task 3: Slow task (500ms rate)
        if (task3_flag)
        {
            task3_flag = 0;
            task3_count++;
            PORTB ^= (1 << 5); // Toggle LED3
            printf("Task 3 toggle (500ms rate, count: %lu)\r\n", task3_count);
            printf("System time: %.3f seconds\r\n\r\n", timer2_ticks / 1000.0);
        }
    }
}

/*
 * =============================================================================
 * DEMO 7: TIMER1 ADVANCED - 16-BIT TIMER FEATURES
 * =============================================================================
 * Demonstrates:
 * - 16-bit Timer1 (0 to 65535 range)
 * - CTC mode with 16-bit TOP value
 * - Precise frequency generation
 * - OCR1A usage for compare value
 * - Timer1 overflow interrupt
 */

// Timer1 Compare Match A ISR
ISR(TIMER1_COMPA_vect)
{
    // Toggle LED on compare match
    PORTB ^= (1 << 6);
}

void demo7_timer1_advanced(void)
{
    printf("DEMO 7: Timer1 Advanced (16-bit Timer)\r\n");
    printf("--------------------------------------\r\n");
    printf("Timer1 in CTC mode with OCR1A\r\n");
    printf("16-bit resolution: 0-65535\r\n");
    printf("Compare match interrupt enabled\r\n\r\n");

    // Configure LED
    DDRB |= (1 << 6);
    PORTB |= (1 << 6);

    // Initialize Timer1
    TCCR1A = 0x00; // Normal port operation, CTC mode (part 1)
    TCCR1B = 0x00; // Stop timer
    TCNT1 = 0;     // Clear counter

    // Set compare value for 1 Hz toggle (0.5 second period)
    // With prescaler 256: 16MHz/256 = 62500 Hz timer clock
    // For 0.5s: 62500 * 0.5 = 31250 counts
    OCR1A = 31250;

    // Clear compare match flag
    TIFR = (1 << OCF1A);

    // Enable Timer1 Compare Match A interrupt
    TIMSK |= (1 << OCIE1A);

    // Start Timer1: CTC mode (WGM13:10 = 0100), prescaler 256
    // CTC mode with OCR1A: WGM13=0, WGM12=1 (in TCCR1B), WGM11=0, WGM10=0 (in TCCR1A)
    // Prescaler 256: CS12=1, CS11=0, CS10=0
    TCCR1B = (1 << WGM12) | (1 << CS12);

    // Enable global interrupts
    sei();

    printf("Timer1 started. LED toggles every 0.5 seconds (1 Hz)\r\n");
    printf("Using ISR(TIMER1_COMPA_vect) for compare match\r\n\r\n");

    uint32_t seconds = 0;
    while (1)
    {
        // Main loop can do other work
        // LED toggling happens in ISR automatically

        // Print status every few seconds
        for (volatile uint32_t i = 0; i < 1000000; i++)
        {
            asm volatile("nop");
        }

        seconds++;
        printf("Running... %lu seconds elapsed\r\n", seconds);
        printf("Timer1 value: %u / 31250\r\n", TCNT1);
    }
}

/*
 * =============================================================================
 * HELPER FUNCTIONS
 * =============================================================================
 */

void show_demo_menu(void)
{
    printf("\r\nAvailable Demos:\r\n");
    printf("1. Normal Mode - Overflow Polling\r\n");
    printf("2. CTC Mode - Clear Timer on Compare\r\n");
    printf("3. Fast PWM Mode - PWM Generation\r\n");
    printf("4. Phase Correct PWM - Symmetric Waveforms\r\n");
    printf("5. Prescaler Comparison - All 5 Prescalers\r\n");
    printf("6. Interrupt Multi-tasking - Timer2 ISR\r\n");
    printf("7. Timer1 Advanced - 16-bit Timer\r\n");
    printf("\r\nChange 'demo_choice' in main() to select demo.\r\n\r\n");
}

void simple_init(void)
{
    // Initialize UART for printf
    Uart1_init();

    // Initialize all port pins
    DDRB = 0xFF;  // All outputs
    PORTB = 0xFF; // All LEDs off (active low)

    // Initialize PORTA for additional LEDs/indicators
    DDRA = 0xFF;
    PORTA = 0xFF;

    // Small delay for hardware stabilization
    for (volatile uint32_t i = 0; i < 100000; i++)
    {
        asm volatile("nop");
    }
}
