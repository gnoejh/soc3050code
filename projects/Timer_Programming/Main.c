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
 * DOCUMENTATION REFERENCE:
 * ATmega128 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/2467S.pdf
 * - Timer/Counter0 section (pages 77-85)
 * - Timer/Counter1 section (pages 86-107)
 * - Interrupt vectors (pages 44-46)
 * - I/O Port descriptions (pages 62-75)
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
 * TIMER1 CONTROL REGISTERS - 16-BIT TIMER (Used for CTC/PWM Demos 3-6)
 * =============================================================================
 *
 * NOTE: Timer1 is used for CTC and PWM demos due to SimulIDE Timer0 bug
 * Timer1 is a 16-bit timer with more features and precision than Timer0
 *
 * REGISTER 1: TCCR1A (Timer/Counter1 Control Register A)
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: COM1A1 COM1A0 COM1B1 COM1B0 COM1C1 COM1C0 WGM11  WGM10
 *
 * COM1A1:0 (bits 7-6): Compare Output Mode for Channel A (OC1A pin = PB5)
 *                      00 = Normal port operation, OC1A disconnected
 *                      10 = Clear OC1A on compare match (non-inverting PWM)
 *                      11 = Set OC1A on compare match (inverting PWM)
 *
 * WGM11:10 (bits 1-0): Waveform Generation Mode (lower 2 bits)
 *                      Combined with WGM13:12 in TCCR1B for full mode
 *
 * REGISTER 2: TCCR1B (Timer/Counter1 Control Register B)
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: ICNC1  ICES1   -    WGM13  WGM12   CS12   CS11   CS10
 *
 * WGM13:10 (bits 4-3 + TCCR1A bits 1-0): Waveform Generation Mode
 *          0001 = Phase Correct PWM, 8-bit (TOP=0xFF)
 *          0100 = CTC (Clear Timer on Compare, TOP=OCR1A)
 *          0101 = Fast PWM, 8-bit (TOP=0xFF)
 *
 * CS12:10 (bits 2-0): Clock Select (Prescaler)
 *                     000 = No clock (timer stopped)
 *                     001 = clk/1 (no prescaling) - 16MHz
 *                     010 = clk/8 - 2MHz
 *                     011 = clk/64 - 250kHz
 *                     100 = clk/256 - 62.5kHz
 *                     101 = clk/1024 - 15.625kHz
 *
 * REGISTER 3: TCNT1H/TCNT1L (Timer/Counter1 Register - 16-bit)
 * Current timer value (0-65535). MUST access high byte first when writing!
 * Write: TCNT1H = 0; TCNT1L = 0;  (high byte first)
 * Read:  value = TCNT1;             (compiler handles it)
 *
 * REGISTER 4: OCR1AH/OCR1AL (Output Compare Register 1A - 16-bit)
 * Compare value (0-65535 or 0-255 for 8-bit PWM modes)
 * In CTC mode: TOP value that resets the counter
 * In PWM mode: Duty cycle value
 * Write: OCR1AH = high_byte; OCR1AL = low_byte; (high byte first)
 * For 8-bit values: OCR1AH = 0; OCR1AL = duty;
 *
 * REGISTER 5: TIFR (Timer Interrupt Flag Register) - Timer1 Flags
 *
 * OCF1A (bit 4): Timer1 Output Compare A Match Flag
 *                Set when TCNT1 equals OCR1A
 *                Clear by writing 1: TIFR = (1<<OCF1A);
 *
 * TOV1 (bit 2):  Timer1 Overflow Flag
 *                Set when timer overflows from MAX to 0
 *                Clear by writing 1: TIFR = (1<<TOV1);
 *
 * REGISTER 6: TIMSK (Timer Interrupt Mask Register) - Timer1 Interrupts
 *
 * OCIE1A (bit 4): Timer1 Output Compare A Match Interrupt Enable
 *                 Write 1 to enable compare match A interrupt
 *                 Usage: TIMSK = (1<<OCIE1A); sei();
 *                 Then define: ISR(TIMER1_COMPA_vect) { code here }
 *
 * TOIE1 (bit 2):  Timer1 Overflow Interrupt Enable
 *                 Write 1 to enable overflow interrupt
 *                 Usage: TIMSK = (1<<TOIE1); sei();
 *                 Then define: ISR(TIMER1_OVF_vect) { code here }
 *
 * TIMER1 EXAMPLES @ 16MHz:
 *
 * CTC Mode Example (Demo 3, 4):
 *   TCCR1A = 0x00;                              // Normal port operation
 *   TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10); // CTC mode, prescaler 64
 *   OCR1AH = 0; OCR1AL = 249;                   // TOP = 249
 *   Frequency: 16MHz/64/250 = 1000 Hz (1ms period)
 *
 * Fast PWM 8-bit Example (Demo 5):
 *   TCCR1A = (1<<COM1A1) | (1<<WGM10);          // Non-inverting, Fast PWM 8-bit
 *   TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10); // Fast PWM, prescaler 64
 *   OCR1AH = 0; OCR1AL = 128;                   // 50% duty cycle
 *   PWM Frequency: 16MHz/64/256 = 976.5 Hz
 *
 * Phase Correct PWM 8-bit Example (Demo 6):
 *   TCCR1A = (1<<COM1A1) | (1<<WGM10);          // Non-inverting, Phase Correct
 *   TCCR1B = (1<<CS11) | (1<<CS10);             // Phase Correct, prescaler 64
 *   OCR1AH = 0; OCR1AL = 128;                   // 50% duty cycle
 *   PWM Frequency: 16MHz/64/510 = ~490 Hz (half of Fast PWM)
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
// stdio.h removed - causes UART initialization issues in SimulIDE
// #include <stdio.h>

// Function prototypes
void demo1_normal_polling(void);
void demo2_normal_interrupt(void);
void demo3_ctc_polling(void);
void demo4_ctc_interrupt(void);
void demo5_fast_pwm(void);
void demo6_phase_correct_pwm(void);
void demo7_prescaler_comparison(void);
void demo8_multitask_interrupt(void);
void demo0_led_test(void); // Simple LED test without timer
void show_demo_menu(void);
void simple_init(void);

// Global variables for interrupt-driven demos
volatile uint16_t overflow_count = 0; // Overflow counter for demo2
volatile uint16_t compare_count = 0;  // Compare match counter for demo4
volatile uint16_t timer_ticks = 0;    // Tick counter for demo8
volatile uint8_t task1_flag = 0;      // Task flags for demo8
volatile uint8_t task2_flag = 0;
volatile uint8_t task3_flag = 0;
volatile uint8_t demo_mode = 0; // ISR mode selector: 2=demo2, 8=demo8

/*
 * =============================================================================
 * MAIN FUNCTION - DEMO SELECTOR
 * =============================================================================
 * INSTRUCTIONS: Uncomment ONE demo function below to select which demo to run.
 * Comment out all others. Rebuild and upload to run the selected demo.
 * =============================================================================
 */
int main(void)
{
    simple_init();

    // NOTE: printf() disabled - Uart1_init() blocks in SimulIDE
    // Watch LEDs instead of serial output

    // =============================================================================
    // SELECT DEMO: Uncomment ONE function below, comment out all others
    // =============================================================================

    // demo0_led_test();              // Demo 0: Simple LED Test (1 second blink, no timer)
    demo1_normal_polling(); // Demo 1: Normal Mode - POLLING
    // demo2_normal_interrupt(); // Demo 2: Normal Mode - INTERRUPT
    // demo3_ctc_polling(); // Demo 3: CTC Mode - POLLING
    // demo4_ctc_interrupt();         // Demo 4: CTC Mode - INTERRUPT
    // demo5_fast_pwm(); // Demo 5: Fast PWM Mode
    // demo6_phase_correct_pwm();     // Demo 6: Phase Correct PWM Mode
    // demo7_prescaler_comparison();  // Demo 7: Prescaler Comparison
    // demo8_multitask_interrupt();   // Demo 8: Multi-tasking with ISR

    // =============================================================================

    while (1)
    {
        // Demo runs continuously
    }

    return 0;
}

/*
 * =============================================================================
 * DEMO 0: SIMPLE LED TEST (NO TIMER)
 * =============================================================================
 * Purpose: Verify LED hardware works before testing timer functionality
 * Method: Simple _delay_ms() loops to blink LED
 * This confirms:
 * - PORTB configuration is correct
 * - LED polarity is correct (active LOW)
 * - LED circuit is functional
 * - _delay_ms() timing works
 */
void demo0_led_test(void)
{
    uint8_t count = 0;

    // NO PRINTF - Just blink to test hardware
    // PORTB already configured in simple_init()

    while (1)
    {
        PORTB &= ~(1 << 0); // LED ON (active LOW)
        _delay_ms(1000);    // 1 second

        PORTB |= (1 << 0); // LED OFF (active LOW)
        _delay_ms(1000);   // 1 second

        // Blink all LEDs briefly every 10 cycles to show program is running
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
 * - Prescaler: 1024 (CS02:00 = 101) - SLOW for visible blinking
 * - Overflow frequency: 16MHz/1024/256 = 61.04 Hz (16.384ms period)
 */
void demo1_normal_polling(void)
{
    uint16_t count = 0;

    // NO PRINTF - UART blocks in SimulIDE
    // Watch PB0 LED blinking at ~1 Hz (1 second ON/OFF)

    // Initialize Timer0
    TCCR0 = 0x00;       // Stop timer
    TCNT0 = 0;          // Clear counter
    TIFR = (1 << TOV0); // Clear overflow flag

    // Turn PB0 ON (ACTIVE LOW: 0=ON, 1=OFF)
    PORTB &= ~(1 << 0);

    // Start Timer0: Normal mode, prescaler 1024
    // 16MHz/1024/256 = 61 Hz overflow
    // Toggle every 30 overflows = ~1 Hz (1 second period)
    TCCR0 = (1 << CS02) | (1 << CS00);

    while (1)
    {
        // POLLING: Check overflow flag in loop
        if (TIFR & (1 << TOV0))
        {
            count++;

            // Clear flag by writing 1 to it
            TIFR = (1 << TOV0);

            // Toggle LED every 30 overflows (~0.5 seconds, so 1 sec full cycle)
            if (count >= 30)
            {
                PORTB ^= (1 << 0);
                count = 0;
            }
        }
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
 * - Prescaler: 1024 - SLOW for visible blinking
 * - Interrupt: TOIE0 enabled
 * - Same timing as Demo 1, but CPU is not blocked!
 */

/*
 * =============================================================================
 * TIMER0 INTERRUPT SERVICE ROUTINES
 * =============================================================================
 * Note: Only ONE ISR per vector is allowed!
 * ISRs use demo_mode to determine behavior for different demos
 */

// Timer0 Overflow ISR - shared by demo2 and demo8
ISR(TIMER0_OVF_vect)
{
    if (demo_mode == 2)
    {
        // Demo 2: Overflow counter with slower toggle
        overflow_count++;

        // Toggle LED every 30 overflows for slower, visible blinking
        // 61 Hz / 30 = ~2 Hz toggle = ~1 Hz blink (much slower, easier to see)
        if (overflow_count >= 30)
        {
            PORTB ^= (1 << 0); // Toggle PB0
            overflow_count = 0;
        }
    }
    else if (demo_mode == 8)
    {
        // Demo 8: Multi-tasking with timer reload
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

// Timer1 Compare Match A ISR - used by demo4
ISR(TIMER1_COMPA_vect)
{
    compare_count++;

    // Toggle LED every 1000 matches (~1 second)
    if (compare_count >= 1000)
    {
        PORTB ^= (1 << 0);
        compare_count = 0;
    }
}

void demo2_normal_interrupt(void)
{
    // Reset global counter and set mode
    overflow_count = 0;
    demo_mode = 2; // ISR will use demo2 logic

    // Initialize Timer0
    TCCR0 = 0x00;
    TCNT0 = 0;
    TIFR = (1 << TOV0);

    // Turn PB0 ON (ACTIVE LOW: 0=ON, 1=OFF)
    PORTB &= ~(1 << 0);

    // Enable Timer0 overflow interrupt
    TIMSK = (1 << TOIE0);

    // Start Timer0: Normal mode, prescaler 1024 (slower blinking)
    TCCR0 = (1 << CS02) | (1 << CS00);

    // Enable global interrupts
    sei();

    // Main loop can do other work
    while (1)
    {
        // CPU is free to do other tasks here!
        // ISR handles the LED toggling automatically

        // Simulate doing other work
        for (volatile uint16_t i = 0; i < 1000; i++)
        {
            asm volatile("nop");
        }
    }
}

/*
 * =============================================================================
 * DEMO 3: TIMER1 CTC MODE - POLLING METHOD
 * =============================================================================
 * Method: POLLING - CPU checks OCF1A flag
 * Demonstrates:
 * - Timer1 (16-bit) in CTC mode (Clear Timer on Compare)
 * - OCR1A register sets TOP value
 * - Polling OCF1A flag for compare match
 * - Precise frequency generation with 16-bit resolution
 *
 * Timer Configuration:
 * - Timer: TIMER1 (16-bit)
 * - Mode: CTC (WGM13:10 = 0100, TOP = OCR1A)
 * - Prescaler: 64
 * - OCR1A: 249 (timer counts 0-249, then resets)
 * - Compare frequency: 16MHz/64/250 = 1000 Hz (1ms period)
 *
 * NOTE: Switched from Timer0 to Timer1 due to SimulIDE Timer0 CTC bug
 */
void demo3_ctc_polling(void)
{
    uint16_t count = 0;

    // Initialize Timer1 in CTC mode
    TCCR1A = 0x00;       // Normal port operation, CTC mode
    TCCR1B = 0x00;       // Stop timer
    TCNT1H = 0;          // Clear counter (high byte first)
    TCNT1L = 0;          // Clear counter (low byte)
    OCR1AH = 0;          // OCR1A = 249 (TOP value)
    OCR1AL = 249;        // For 1ms period: 16MHz/64/250 = 1kHz
    TIFR = (1 << OCF1A); // Clear compare match flag

    // PB0 already configured in simple_init()
    PORTB &= ~(1 << 0); // Turn LED ON

    // Start Timer1: CTC mode (WGM12=1), prescaler 64 (CS11=1, CS10=1)
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    while (1)
    {
        // POLLING: Check compare match A flag
        if (TIFR & (1 << OCF1A))
        {
            count++;

            // Clear flag by writing 1
            TIFR = (1 << OCF1A);

            // Toggle LED every 1000 matches (~1 second)
            if (count >= 1000)
            {
                PORTB ^= (1 << 0);
                count = 0;
            }
        }
    }
}

/*
 * =============================================================================
 * DEMO 4: TIMER1 CTC MODE - INTERRUPT METHOD
 * =============================================================================
 * Method: INTERRUPT - ISR called on compare match
 * Demonstrates:
 * - Timer1 compare match A interrupt (OCIE1A)
 * - ISR(TIMER1_COMPA_vect) - Compare match ISR
 * - Precise timing with interrupts
 * - CPU free for other tasks
 *
 * Timer Configuration:
 * - Timer: TIMER1 (16-bit)
 * - Mode: CTC (WGM13:10 = 0100, TOP = OCR1A)
 * - Prescaler: 64
 * - OCR1A: 249
 * - Interrupt: OCIE1A enabled
 * - Same timing as Demo 3, but using interrupts!
 *
 * NOTE: Switched from Timer0 to Timer1 due to SimulIDE Timer0 CTC bug
 */

void demo4_ctc_interrupt(void)
{
    // Reset global counter
    compare_count = 0;

    // Initialize Timer1
    TCCR1A = 0x00; // Normal port operation
    TCCR1B = 0x00; // Stop timer
    TCNT1H = 0;    // Clear counter
    TCNT1L = 0;
    OCR1AH = 0; // OCR1A = 249
    OCR1AL = 249;
    TIFR = (1 << OCF1A); // Clear compare flag

    // PB0 already configured in simple_init()
    PORTB &= ~(1 << 0); // Turn LED ON

    // Enable Timer1 compare match A interrupt
    TIMSK = (1 << OCIE1A);

    // Start Timer1: CTC mode, prescaler 64
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    // Enable global interrupts
    sei();

    // Main loop is free
    while (1)
    {
        // CPU free to do other work
        for (volatile uint16_t i = 0; i < 1000; i++)
        {
            asm volatile("nop");
        }
    }
}

/*
 * =============================================================================
 * DEMO 5: TIMER1 FAST PWM MODE
 * =============================================================================
 * Demonstrates:
 * - Timer1 in Fast PWM mode (8-bit for simplicity)
 * - Duty cycle control via OCR1A
 * - Dynamic PWM adjustment (0% to 100% sweep)
 * - OC1A pin output (PB5)
 *
 * Timer Configuration:
 * - Timer: TIMER1 (16-bit, using 8-bit Fast PWM)
 * - Mode: Fast PWM 8-bit (WGM13:10 = 0101, TOP = 0xFF)
 * - Prescaler: 64
 * - PWM frequency: 16MHz/64/256 = 976.5 Hz
 *
 * NOTE: Switched from Timer0 to Timer1 due to SimulIDE Timer0 PWM bug
 */
void demo5_fast_pwm(void)
{
    uint8_t duty = 0;

    // Initialize Timer1
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1H = 0;
    TCNT1L = 0;
    OCR1AH = 0;
    OCR1AL = 0;

    // Configure PB5 (OC1A) as output
    DDRB |= (1 << 5);

    // Configure PORTB for visual duty cycle display (8 LEDs)
    DDRB = 0xFF;
    PORTB = 0xFF;

    // Fast PWM 8-bit: WGM12:10 = 101, COM1A1=1 (non-inverting), prescaler 64
    // TCCR1A: COM1A1=1, WGM11=0, WGM10=1
    // TCCR1B: WGM13=0, WGM12=1, CS11=1, CS10=1
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    while (1)
    {
        for (duty = 0; duty < 255; duty++)
        {
            OCR1AL = duty;
            PORTB = ~duty; // Display on PORTB LEDs

            for (volatile uint16_t i = 0; i < 10000; i++)
            {
                asm volatile("nop");
            }
        }

        for (duty = 255; duty > 0; duty--)
        {
            OCR1AL = duty;
            PORTB = ~duty; // Display on PORTB LEDs

            for (volatile uint16_t i = 0; i < 10000; i++)
            {
                asm volatile("nop");
            }
        }
    }
}

/*
 * =============================================================================
 * DEMO 6: TIMER1 PHASE CORRECT PWM MODE
 * =============================================================================
 * Demonstrates:
 * - Timer1 in Phase Correct PWM mode (8-bit)
 * - Symmetric counting (0->255->0)
 * - Lower frequency than Fast PWM (counts both directions)
 *
 * Timer Configuration:
 * - Timer: TIMER1 (16-bit, using 8-bit Phase Correct PWM)
 * - Mode: Phase Correct PWM 8-bit (WGM13:10 = 0001, TOP = 0xFF)
 * - Prescaler: 64
 * - PWM frequency: ~490 Hz (half of Fast PWM)
 * - Output: OC1A pin (PB5)
 *
 * NOTE: Switched from Timer0 to Timer1 due to SimulIDE Timer0 PWM bug
 */
void demo6_phase_correct_pwm(void)
{
    uint8_t duty = 128;

    // Initialize Timer1
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1H = 0;
    TCNT1L = 0;
    OCR1AH = 0;
    OCR1AL = duty;

    // Configure PB5 (OC1A) as output
    DDRB |= (1 << 5);

    // Phase Correct PWM 8-bit: WGM11:10 = 01, COM1A1=1, prescaler 64
    // TCCR1A: COM1A1=1, WGM11=0, WGM10=1
    // TCCR1B: WGM13=0, WGM12=0, CS11=1, CS10=1
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << CS11) | (1 << CS10);

    uint16_t count = 0;
    while (1)
    {
        duty = 128 + (count % 128);
        OCR1AL = duty;

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
    const uint8_t values[] = {
        (1 << CS00),
        (1 << CS01),
        (1 << CS01) | (1 << CS00),
        (1 << CS02),
        (1 << CS02) | (1 << CS00)};

    // PB0 already configured in simple_init()
    PORTB &= ~(1 << 0); // Turn LED ON

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
                PORTB ^= (1 << 0); // Toggle PB0
            }

            // Delay between prescaler changes
            for (volatile uint32_t k = 0; k < 100000; k++)
            {
                asm volatile("nop");
            }
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
 * ISR shared with demo2 - uses demo_mode to differentiate
 */

void demo8_multitask_interrupt(void)
{
    timer_ticks = 0;
    demo_mode = 8; // ISR will use demo8 logic

    // PB0 already configured in simple_init()
    PORTB &= ~(1 << 0); // Turn LED ON

    // Initialize Timer0
    TCCR0 = 0x00;
    TCNT0 = 6;
    TIFR = (1 << TOV0);

    // Enable overflow interrupt
    TIMSK = (1 << TOIE0);

    // Start with prescaler 64
    TCCR0 = (1 << CS01) | (1 << CS00);

    sei();

    while (1)
    {
        if (task1_flag)
        {
            task1_flag = 0;
        }

        if (task2_flag)
        {
            task2_flag = 0;
            PORTB ^= (1 << 0); // Toggle PB0 every 100ms
        }

        if (task3_flag)
        {
            task3_flag = 0;
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
    // Menu disabled - UART not available in SimulIDE
    // To select a demo: Uncomment ONE demo function call in main()
}

void simple_init(void)
{
    // UART DISABLED FOR TESTING - May be blocking in SimulIDE
    // Uart1_init();

    // Initialize PORTB for LEDs (hardware kit and SimulIDE)
    // Active LOW: 0=ON, 1=OFF
    DDRB = 0xFF;  // All pins as output
    PORTB = 0xFF; // All LEDs OFF initially (active low)

    // Small delay for stability
    for (volatile uint32_t i = 0; i < 100000; i++)
    {
        asm volatile("nop");
    }
}
