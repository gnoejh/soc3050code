/*
 * =============================================================================
 * INTERRUPT PROGRAMMING - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master interrupt handling through practical exercises
 * DURATION: 90 minutes
 * DIFFICULTY: Advanced
 *
 * STUDENTS WILL:
 * - Implement external interrupt handlers
 * - Practice button debouncing techniques
 * - Handle interrupt priority and nesting
 * - Measure ISR execution time
 * - Create event counters
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - 4 buttons on INT0-3 (PD0-3)
 * - LEDs on PORTB
 * - Optional: Oscilloscope for timing verification
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration
#define LED_PORT PORTB
#define LED_DDR DDRB

// Global variables
uint16_t lab_score = 0;
volatile uint32_t int0_count = 0;
volatile uint32_t int1_count = 0;
volatile uint32_t int2_count = 0;
volatile uint32_t int3_count = 0;
volatile uint8_t button_state = 0;
volatile uint32_t last_interrupt_time = 0;
volatile uint32_t isr_entry_count = 0;
volatile uint16_t timer_ticks = 0;

/*
 * =============================================================================
 * LAB EXERCISE 1: EXTERNAL INTERRUPTS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Configure and handle external interrupts
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

// Simple interrupt handlers
ISR(INT0_vect)
{
    int0_count++;
    LED_PORT ^= (1 << 0); // Toggle LED0
}

ISR(INT1_vect)
{
    int1_count++;
    LED_PORT ^= (1 << 1); // Toggle LED1
}

void lab_ex1_simple_interrupt(void)
{
    /*
     * CHALLENGE: Basic interrupt handling
     * TASK: Toggle LED on button press using INT0
     * LEARNING: Interrupt configuration, ISR basics
     */

    puts_USART1("\r\n=== Lab 1.1: Simple External Interrupt ===\r\n");
    puts_USART1("Press button on INT0 (PD0) to toggle LED\r\n");
    puts_USART1("Press 'Q' to exit\r\n\r\n");

    // Configure LED
    LED_DDR |= (1 << 0);
    LED_PORT |= (1 << 0); // Off

    // Configure INT0 (falling edge)
    EICRA = (1 << ISC01); // Falling edge
    EIMSK = (1 << INT0);  // Enable INT0

    sei(); // Enable global interrupts

    int0_count = 0;

    puts_USART1("Interrupt enabled. Waiting for button presses...\r\n");

    while (1)
    {
        // Display count periodically
        static uint32_t last_display = 0;
        static uint32_t tick = 0;
        tick++;

        if (tick - last_display > 100000)
        {
            last_display = tick;
            char msg[50];
            sprintf(msg, "INT0 count: %lu\r", int0_count);
            puts_USART1(msg);
        }

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    // Disable interrupt
    EIMSK &= ~(1 << INT0);
    cli();

    char summary[60];
    sprintf(summary, "\r\n\r\nTotal interrupts: %lu\r\n", int0_count);
    puts_USART1(summary);

    lab_score += 75;
}

void lab_ex1_multi_interrupt(void)
{
    /*
     * CHALLENGE: Handle multiple interrupt sources
     * TASK: Count interrupts from 4 different buttons
     * LEARNING: Multiple interrupt sources, vector table
     */

    puts_USART1("\r\n=== Lab 1.2: Multiple Interrupts ===\r\n");
    puts_USART1("Press buttons on INT0-3 to count\r\n");
    puts_USART1("Press 'Q' to exit\r\n\r\n");

    // Configure LEDs
    LED_DDR = 0xFF;
    LED_PORT = 0xFF; // All off

    // Configure all external interrupts (falling edge)
    EICRA = (1 << ISC01) | (1 << ISC11); // INT0, INT1 falling edge
    EICRB = (1 << ISC41) | (1 << ISC51); // INT4, INT5 falling edge
    EIMSK = (1 << INT0) | (1 << INT1);   // Enable INT0, INT1

    sei();

    int0_count = 0;
    int1_count = 0;

    while (1)
    {
        // Display counts
        char msg[80];
        sprintf(msg, "\rINT0: %5lu | INT1: %5lu | Total: %5lu",
                int0_count, int1_count, int0_count + int1_count);
        puts_USART1(msg);

        _delay_ms(100);

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    // Disable interrupts
    EIMSK = 0;
    cli();

    puts_USART1("\r\n\r\nMulti-interrupt test complete!\r\n");

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: DEBOUNCING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement software debouncing
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

volatile uint8_t debounce_timer = 0;
volatile uint8_t button_pressed = 0;

ISR(TIMER0_OVF_vect)
{
    // Debounce timer tick
    if (debounce_timer > 0)
    {
        debounce_timer--;
    }
}

ISR(INT2_vect)
{
    // Debounced button handler
    if (debounce_timer == 0)
    {
        button_pressed = 1;
        int2_count++;
        debounce_timer = 20; // 20ms debounce period
    }
}

void lab_ex2_software_debounce(void)
{
    /*
     * CHALLENGE: Implement software debouncing
     * TASK: Count clean button presses without bounces
     * LEARNING: Debouncing techniques, timer+interrupt
     */

    puts_USART1("\r\n=== Lab 2.1: Software Debouncing ===\r\n");
    puts_USART1("Press button rapidly - debouncing will filter bounces\r\n");
    puts_USART1("Press 'Q' to exit\r\n\r\n");

    // Configure LED
    LED_DDR |= (1 << 2);
    LED_PORT |= (1 << 2);

    // Configure Timer0 for 1ms tick (prescaler 64)
    TCCR0 = (1 << CS01) | (1 << CS00);
    TIMSK = (1 << TOIE0);

    // Configure INT2 (falling edge)
    EICRB = (1 << ISC21);
    EIMSK = (1 << INT2);

    sei();

    int2_count = 0;
    debounce_timer = 0;
    button_pressed = 0;

    while (1)
    {
        if (button_pressed)
        {
            button_pressed = 0;
            LED_PORT ^= (1 << 2); // Toggle LED

            char msg[50];
            sprintf(msg, "Clean press %lu\r\n", int2_count);
            puts_USART1(msg);
        }

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    // Disable
    EIMSK = 0;
    TIMSK = 0;
    cli();

    char summary[60];
    sprintf(summary, "\r\nTotal clean presses: %lu\r\n", int2_count);
    puts_USART1(summary);

    lab_score += 125;
}

void lab_ex2_state_machine_debounce(void)
{
    /*
     * CHALLENGE: State machine debouncing
     * TASK: Implement 3-state debounce (idle/pressed/released)
     * LEARNING: State machines in ISR
     */

    puts_USART1("\r\n=== Lab 2.2: State Machine Debouncing ===\r\n");
    puts_USART1("Advanced debouncing with state tracking\r\n");
    puts_USART1("Press 'Q' to exit\r\n\r\n");

    enum
    {
        STATE_IDLE,
        STATE_PRESSED,
        STATE_RELEASED
    };

    volatile uint8_t state = STATE_IDLE;
    volatile uint16_t press_count = 0;
    volatile uint16_t release_count = 0;

    LED_DDR |= (1 << 3);
    LED_PORT |= (1 << 3);

    while (1)
    {
        // Poll button state
        uint8_t button = (PIND & (1 << 0)) ? 1 : 0;

        switch (state)
        {
        case STATE_IDLE:
            if (button == 0) // Button pressed
            {
                _delay_ms(5); // Debounce delay
                if ((PIND & (1 << 0)) == 0)
                {
                    state = STATE_PRESSED;
                    press_count++;
                    LED_PORT &= ~(1 << 3); // LED on

                    char msg[40];
                    sprintf(msg, "Pressed: %u\r\n", press_count);
                    puts_USART1(msg);
                }
            }
            break;

        case STATE_PRESSED:
            if (button == 1) // Button released
            {
                _delay_ms(5); // Debounce delay
                if (PIND & (1 << 0))
                {
                    state = STATE_IDLE;
                    release_count++;
                    LED_PORT |= (1 << 3); // LED off

                    char msg[40];
                    sprintf(msg, "Released: %u\r\n", release_count);
                    puts_USART1(msg);
                }
            }
            break;
        }

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }

        _delay_ms(10);
    }

    puts_USART1("\r\nState machine debounce complete!\r\n");

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: ISR PERFORMANCE (20 minutes)
 * =============================================================================
 * OBJECTIVE: Measure and optimize ISR execution
 * DIFFICULTY: ★★★★☆ (Advanced)
 */

volatile uint16_t isr_duration_us = 0;
volatile uint32_t max_isr_duration = 0;

ISR(INT3_vect)
{
    // Set measurement pin high
    PORTB |= (1 << 7);

    isr_entry_count++;

    // Simulate work
    for (volatile uint16_t i = 0; i < 100; i++)
        ;

    // Set measurement pin low
    PORTB &= ~(1 << 7);
}

void lab_ex3_isr_timing(void)
{
    /*
     * CHALLENGE: Measure ISR execution time
     * TASK: Use oscilloscope or counter to measure ISR duration
     * LEARNING: ISR optimization, timing analysis
     */

    puts_USART1("\r\n=== Lab 3.1: ISR Timing Measurement ===\r\n");
    puts_USART1("Measuring INT3 ISR execution time\r\n");
    puts_USART1("PB7 will pulse during ISR (measure with scope)\r\n");
    puts_USART1("Press button on INT3, then 'Q' to exit\r\n\r\n");

    // Configure PB7 as output for timing measurement
    DDRB |= (1 << 7);
    PORTB &= ~(1 << 7);

    // Configure INT3
    EICRB = (1 << ISC31); // Falling edge
    EIMSK = (1 << INT3);

    sei();

    isr_entry_count = 0;

    while (1)
    {
        char msg[60];
        sprintf(msg, "\rISR called: %lu times", isr_entry_count);
        puts_USART1(msg);

        _delay_ms(100);

        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    EIMSK = 0;
    cli();

    puts_USART1("\r\n\r\nISR timing test complete!\r\n");
    puts_USART1("Check oscilloscope for pulse width on PB7\r\n");

    lab_score += 125;
}

void lab_ex3_nested_interrupts(void)
{
    /*
     * CHALLENGE: Understand interrupt priority
     * TASK: Demonstrate interrupt preemption
     * LEARNING: Nested interrupts, priority
     */

    puts_USART1("\r\n=== Lab 3.2: Nested Interrupts ===\r\n");
    puts_USART1("Demonstrating interrupt priority and nesting\r\n");
    puts_USART1("INT0 has higher priority than Timer overflow\r\n\r\n");

    volatile uint16_t timer_isr_count = 0;
    volatile uint16_t int_during_timer = 0;

    // Configure Timer0 for slow overflow (~2ms)
    TCCR0 = (1 << CS02) | (1 << CS00); // Prescaler 1024
    TIMSK = (1 << TOIE0);

    // Configure INT0
    EICRA = (1 << ISC01);
    EIMSK = (1 << INT0);

    sei();

    puts_USART1("Press button during timer ISR to test preemption\r\n");
    puts_USART1("Press 'Q' to exit\r\n\r\n");

    while (1)
    {
        _delay_ms(200);

        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    EIMSK = 0;
    TIMSK = 0;
    cli();

    puts_USART1("\r\nNested interrupt test complete!\r\n");

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: EVENT COUNTING (25 minutes)
 * =============================================================================
 * OBJECTIVE: Build frequency counter and event logger
 * DIFFICULTY: ★★★★★ (Expert)
 */

void lab_ex4_frequency_counter(void)
{
    /*
     * CHALLENGE: Measure external signal frequency
     * TASK: Count pulses over 1 second to determine frequency
     * LEARNING: Frequency measurement, gating
     */

    puts_USART1("\r\n=== Lab 4.1: Frequency Counter ===\r\n");
    puts_USART1("Measuring frequency on INT0 pin\r\n");
    puts_USART1("Apply external signal or press button\r\n");
    puts_USART1("Press 'Q' to exit\r\n\r\n");

    // Configure INT0 for rising edge
    EICRA = (1 << ISC01) | (1 << ISC00); // Rising edge
    EIMSK = (1 << INT0);

    sei();

    for (uint8_t measurement = 0; measurement < 10; measurement++)
    {
        int0_count = 0;

        puts_USART1("Measuring for 1 second...\r\n");

        _delay_ms(1000); // 1 second gate time

        uint32_t frequency = int0_count;

        char msg[60];
        sprintf(msg, "Measurement %u: %lu Hz\r\n", measurement + 1, frequency);
        puts_USART1(msg);

        // Check for early exit
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    EIMSK = 0;
    cli();

    puts_USART1("\r\nFrequency measurement complete!\r\n");

    lab_score += 150;
}

/*
 * =============================================================================
 * LAB MENU SYSTEM
 * =============================================================================
 */

void print_lab_menu(void)
{
    puts_USART1("\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("  INTERRUPT PROGRAMMING - LAB EXERCISES\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 1: External Interrupts\r\n");
    puts_USART1("  1. Simple External Interrupt\r\n");
    puts_USART1("  2. Multiple Interrupts\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 2: Debouncing\r\n");
    puts_USART1("  3. Software Debouncing\r\n");
    puts_USART1("  4. State Machine Debouncing\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 3: ISR Performance\r\n");
    puts_USART1("  5. ISR Timing Measurement\r\n");
    puts_USART1("  6. Nested Interrupts\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 4: Event Counting\r\n");
    puts_USART1("  7. Frequency Counter\r\n");
    puts_USART1("\r\n");
    puts_USART1("  0. Run All Exercises\r\n");
    puts_USART1("  X. Exit Lab\r\n");
    puts_USART1("\r\n");
    char score_str[40];
    sprintf(score_str, "Current Score: %u points\r\n\r\n", lab_score);
    puts_USART1(score_str);
    puts_USART1("Select exercise (1-7, 0, X): ");
}

int main(void)
{
    // Initialize system
    init_devices();
    Uart1_init();

    _delay_ms(100);

    puts_USART1("\r\n\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("*  ATmega128 INTERRUPT PROGRAMMING LAB         *\r\n");
    puts_USART1("*  Hands-On Interrupt Exercises                *\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("\r\n");
    puts_USART1("Welcome to the Interrupt Programming Lab!\r\n");
    puts_USART1("Master interrupts through practical exercises.\r\n");

    while (1)
    {
        print_lab_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\r');
        putch_USART1('\n');

        switch (choice)
        {
        case '1':
            lab_ex1_simple_interrupt();
            break;
        case '2':
            lab_ex1_multi_interrupt();
            break;
        case '3':
            lab_ex2_software_debounce();
            break;
        case '4':
            lab_ex2_state_machine_debounce();
            break;
        case '5':
            lab_ex3_isr_timing();
            break;
        case '6':
            lab_ex3_nested_interrupts();
            break;
        case '7':
            lab_ex4_frequency_counter();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_simple_interrupt();
            lab_ex1_multi_interrupt();
            lab_ex2_software_debounce();
            lab_ex2_state_machine_debounce();
            lab_ex3_isr_timing();
            lab_ex3_nested_interrupts();
            lab_ex4_frequency_counter();

            char final_buffer[80];
            sprintf(final_buffer, "\r\n*** ALL EXERCISES COMPLETE! ***\r\nFinal Score: %u points\r\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\r\nExiting lab. Great work!\r\n");
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\r\n");
        }

        puts_USART1("\r\nPress any key to continue...\r\n");
        getch_USART1();
    }

    return 0;
}
