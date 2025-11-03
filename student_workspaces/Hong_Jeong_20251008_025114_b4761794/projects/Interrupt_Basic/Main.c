/*
 * =============================================================================
 * INTERRUPT PROGRAMMING - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Interrupt_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational comparison between POLLING vs INTERRUPT-based input handling.
 * Students learn both approaches with practical demonstrations and performance analysis.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Compare polling vs interrupt-driven input handling
 * 2. Master real ISR programming (no wrappers!)
 * 3. Learn external interrupt configuration
 * 4. Practice timer-based interrupt handling
 * 5. Understand interrupt priorities and timing
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - Push button on INT0 (PD0) for external interrupts
 * - Additional buttons on PD1-PD3 for polling demos
 * - LEDs on PORTB for status indication
 * - Serial connection for debugging (9600 baud)
 *
 * LEARNING PROGRESSION:
 * POLLING METHODS (Simple, blocking, CPU intensive):
 * - Demo 1: Basic Button Polling
 * - Demo 2: Multiple Button Polling
 * - Demo 3: Polling with Debouncing
 *
 * INTERRUPT METHODS (Efficient, non-blocking, complex):
 * - Demo 4: External Interrupt Basics
 * - Demo 5: Timer Interrupt Operations
 * - Demo 6: Multiple Interrupt Sources
 * - Demo 7: Advanced ISR Techniques
 *
 * =============================================================================
 */

#include "config.h"

// Function prototypes
void demo_polling_basic_button(void);
void demo_polling_multiple_buttons(void);
void demo_polling_with_debounce(void);
void demo_interrupt_external_basic(void);
void demo_interrupt_timer_basic(void);
void demo_interrupt_multiple_sources(void);
void demo_interrupt_advanced(void);

// Global variables for interrupt handling (volatile for ISR access)
volatile uint8_t external_interrupt_count = 0;
volatile uint8_t timer_interrupt_count = 0;
volatile uint8_t button_pressed = 0;
volatile uint8_t int0_triggered = 0;
volatile uint8_t int1_triggered = 0;

/*
 * =============================================================================
 * EDUCATIONAL INTERRUPT SERVICE ROUTINES
 * =============================================================================
 * These are the actual ISRs that students must learn to write.
 * No wrappers or managers - direct hardware programming!
 */

// External Interrupt 0 ISR (INT0 - PD0)
// Students learn: ISR syntax, interrupt vectors, debouncing
ISR(INT0_vect)
{
    // External interrupt triggered on INT0 (falling edge)
    external_interrupt_count++;
    button_pressed = 1;
    int0_triggered = 1;

    // Toggle LED to show interrupt occurred
    PORTB ^= (1 << 0); // Toggle LED 0 immediately

    // Note: Debouncing should be handled in main loop, not ISR
    // ISRs should be fast and minimal!
}

// External Interrupt 1 ISR (INT1 - PD1)
// Students learn: Multiple interrupt sources
ISR(INT1_vect)
{
    // External interrupt triggered on INT1
    int1_triggered = 1;

    // Toggle different LED for INT1
    PORTB ^= (1 << 1); // Toggle LED 1
}

// Timer2 Overflow ISR
// Students learn: Timer interrupts, periodic events
ISR(TIMER2_OVF_vect)
{
    // Timer interrupt triggered (approximately every 1 second @ 16MHz)
    timer_interrupt_count++;

    // Toggle LED to show timer interrupt
    PORTB ^= (1 << 2); // Toggle LED 2 for timer
}

/*
 * =============================================================================
 * POLLING-BASED INPUT HANDLING DEMOS
 * =============================================================================
 * These demos use polling (busy-waiting) to check for button presses.
 * Advantages: Simple to understand and implement
 * Disadvantages: CPU is blocked while waiting, inefficient
 */

/*
 * Demo 1: Basic Button Polling
 * Simple button reading using polling method - CPU waits for button press
 */
void demo_polling_basic_button(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Button Polling ===\r\n");
    puts_USART1("POLLING METHOD: CPU continuously checks button state\r\n");
    puts_USART1("Press button on PD1 to see polling in action\r\n");
    puts_USART1("Press button on PD0 (INT0) to exit demo\r\n\r\n");

    // Configure buttons as inputs with pull-ups
    DDRD &= ~(1 << PD0 | 1 << PD1); // PD0, PD1 as inputs
    PORTD |= (1 << PD0 | 1 << PD1); // Enable pull-ups

    // Configure LEDs
    DDRB = 0xFF;  // All PORTB as outputs
    PORTB = 0xFF; // All LEDs off initially

    uint16_t poll_count = 0;
    uint8_t last_button_state = 1; // Pulled up = 1

    while (1)
    {
        poll_count++;

        // POLLING: CPU continuously checks button state
        uint8_t current_button_state = (PIND & (1 << PD1)) ? 1 : 0;

        // Detect button press (high to low transition)
        if (last_button_state == 1 && current_button_state == 0)
        {
            // Button pressed!
            PORTB ^= (1 << 3); // Toggle LED 3

            puts_USART1("POLLING DETECTED: Button pressed after ");
            char buffer[20];
            sprintf(buffer, "%u", poll_count);
            puts_USART1(buffer);
            puts_USART1(" poll cycles\r\n");

            poll_count = 0; // Reset counter

            // Simple debounce delay (blocks CPU)
            _delay_ms(200);
        }

        last_button_state = current_button_state;

        // Check exit condition (PD0 button)
        if (!(PIND & (1 << PD0)))
        {
            puts_USART1("Exiting polling demo...\r\n");
            _delay_ms(200); // Debounce
            break;
        }

        // Show CPU usage with LED blinking
        if ((poll_count % 10000) == 0)
        {
            PORTB ^= (1 << 7); // Toggle LED 7 to show CPU is busy polling
        }
    }

    puts_USART1("Polling Demo 1 completed.\r\n");
    puts_USART1("Note: CPU was busy polling the entire time!\r\n");
}

/*
 * Demo 2: Multiple Button Polling
 * Polling multiple buttons - even more CPU intensive
 */
void demo_polling_multiple_buttons(void)
{
    puts_USART1("\r\n=== DEMO 2: Multiple Button Polling ===\r\n");
    puts_USART1("POLLING METHOD: CPU checks multiple buttons continuously\r\n");
    puts_USART1("Buttons: PD1(LED3), PD2(LED4), PD3(LED5)\r\n");
    puts_USART1("Press PD0 to exit demo\r\n\r\n");

    // Configure buttons
    DDRD &= ~(1 << PD0 | 1 << PD1 | 1 << PD2 | 1 << PD3);
    PORTD |= (1 << PD0 | 1 << PD1 | 1 << PD2 | 1 << PD3);

    uint32_t total_polls = 0;
    uint8_t button_states[4] = {1, 1, 1, 1}; // All pulled up

    while (1)
    {
        total_polls++;

        // POLLING: Check all buttons every loop iteration
        for (uint8_t i = 1; i <= 3; i++)
        {
            uint8_t current_state = (PIND & (1 << i)) ? 1 : 0;

            // Detect press (high to low)
            if (button_states[i] == 1 && current_state == 0)
            {
                PORTB ^= (1 << (i + 2)); // Toggle corresponding LED

                char msg[50];
                sprintf(msg, "POLL: Button PD%u pressed (polls: %lu)\r\n", i, total_polls);
                puts_USART1(msg);

                _delay_ms(150); // Debounce delay
            }

            button_states[i] = current_state;
        }

        // Check exit button (PD0)
        if (!(PIND & (1 << PD0)))
        {
            puts_USART1("Exiting multiple button polling demo...\r\n");
            _delay_ms(200);
            break;
        }

        // Show CPU load
        if ((total_polls % 5000) == 0)
        {
            PORTB ^= (1 << 6); // Toggle LED 6 to show CPU busy
        }
    }

    char final_msg[60];
    sprintf(final_msg, "Total poll cycles: %lu\r\n", total_polls);
    puts_USART1(final_msg);
    puts_USART1("Polling Demo 2 completed.\r\n");
}

/*
 * Demo 3: Polling with Software Debouncing
 * More sophisticated polling with proper debouncing
 */
void demo_polling_with_debounce(void)
{
    puts_USART1("\r\n=== DEMO 3: Polling with Debouncing ===\r\n");
    puts_USART1("POLLING METHOD: CPU polls with software debouncing\r\n");
    puts_USART1("Press PD1 to test debounced button\r\n");
    puts_USART1("Press PD0 to exit demo\r\n\r\n");

    // Configure buttons
    DDRD &= ~(1 << PD0 | 1 << PD1);
    PORTD |= (1 << PD0 | 1 << PD1);

    uint8_t button_state = 1;
    uint8_t debounce_count = 0;
    const uint8_t DEBOUNCE_THRESHOLD = 5;
    uint16_t press_count = 0;

    while (1)
    {
        // POLLING with debouncing algorithm
        uint8_t raw_button = (PIND & (1 << PD1)) ? 1 : 0;

        if (raw_button != button_state)
        {
            debounce_count++;
            if (debounce_count >= DEBOUNCE_THRESHOLD)
            {
                // State change confirmed after multiple consistent readings
                button_state = raw_button;
                debounce_count = 0;

                if (button_state == 0) // Button pressed (active low)
                {
                    press_count++;
                    PORTB ^= (1 << 4); // Toggle LED 4

                    char msg[40];
                    sprintf(msg, "DEBOUNCED PRESS #%u detected\r\n", press_count);
                    puts_USART1(msg);
                }
            }
        }
        else
        {
            debounce_count = 0; // Reset if state is stable
        }

        // Check exit
        if (!(PIND & (1 << PD0)))
        {
            puts_USART1("Exiting debounced polling demo...\r\n");
            _delay_ms(200);
            break;
        }

        // Small delay for debouncing timing
        _delay_ms(10);
    }

    puts_USART1("Polling Demo 3 completed.\r\n");
    puts_USART1("Note: Even with debouncing, CPU was continuously busy!\r\n");
}

/*
 * =============================================================================
 * INTERRUPT-BASED INPUT HANDLING DEMOS
 * =============================================================================
 * These demos use interrupts - CPU is free to do other work!
 * Advantages: Efficient, responsive, non-blocking
 * Disadvantages: More complex, requires understanding of ISRs
 */

/*
 * Demo 4: External Interrupt Basics
 * Using real ISR(INT0_vect) for button handling
 */
void demo_interrupt_external_basic(void)
{
    puts_USART1("\r\n=== DEMO 4: External Interrupt Basics ===\r\n");
    puts_USART1("INTERRUPT METHOD: ISR(INT0_vect) handles button automatically\r\n");
    puts_USART1("Students observe: CPU is FREE while ISR handles button\r\n");
    puts_USART1("Press button on PD0 (INT0) to trigger interrupt\r\n");
    puts_USART1("Watch LEDs: CPU free to do other work!\r\n");
    puts_USART1("Press 's' to show statistics, 'q' to quit\r\n\r\n");

    // EDUCATIONAL: Configure external interrupt (students learn registers!)
    DDRD &= ~(1 << PD0); // PD0 as input
    PORTD |= (1 << PD0); // Enable pull-up resistor

    // Configure interrupt trigger (falling edge)
    EICRA |= (1 << ISC01);  // Falling edge trigger
    EICRA &= ~(1 << ISC00); // Clear ISC00 for falling edge

    // Enable External Interrupt 0
    EIMSK |= (1 << INT0);

    // Enable global interrupts (CRITICAL!)
    sei();

    // Reset counters
    external_interrupt_count = 0;
    button_pressed = 0;

    uint32_t cpu_work_counter = 0;

    while (1)
    {
        // EDUCATIONAL POINT: CPU can do other work while ISR handles interrupts!
        cpu_work_counter++;

        // Show CPU is free by doing LED animations
        for (uint8_t i = 1; i < 8; i++)
        {
            PORTB = ~(1 << i); // Light up one LED at a time
            _delay_ms(50);

            // Check for button press handled by ISR
            if (button_pressed)
            {
                button_pressed = 0; // Clear flag

                char msg[60];
                sprintf(msg, "ISR HANDLED: External interrupt #%u (CPU work: %lu)\r\n",
                        external_interrupt_count, cpu_work_counter);
                puts_USART1(msg);
                puts_USART1("Notice: CPU was free to animate LEDs while ISR handled button!\r\n");
            }
        }
        PORTB = 0xFF; // All LEDs off
        _delay_ms(100);

        // Check for serial commands (non-interrupt)
        if (UCSR1A & (1 << RXC1))
        {
            char cmd = UDR1;
            if (cmd == 's' || cmd == 'S')
            {
                char stats[100];
                sprintf(stats, "\r\n--- INTERRUPT STATISTICS ---\r\n");
                puts_USART1(stats);
                sprintf(stats, "External interrupts: %u\r\n", external_interrupt_count);
                puts_USART1(stats);
                sprintf(stats, "CPU work cycles: %lu\r\n", cpu_work_counter);
                puts_USART1(stats);
                puts_USART1("Key point: CPU was FREE during button handling!\r\n\r\n");
            }
            else if (cmd == 'q' || cmd == 'Q')
            {
                break;
            }
        }
    }

    // Disable interrupt
    EIMSK &= ~(1 << INT0);

    puts_USART1("External Interrupt Demo 4 completed.\r\n");
    puts_USART1("Learning: ISR handled button while CPU did animations!\r\n");
}

/*
 * Demo 5: Timer Interrupt Operations
 * Using real ISR(TIMER2_OVF_vect) for periodic events
 */
void demo_interrupt_timer_basic(void)
{
    puts_USART1("\r\n=== DEMO 5: Timer Interrupt Basics ===\r\n");
    puts_USART1("INTERRUPT METHOD: ISR(TIMER2_OVF_vect) handles timing automatically\r\n");
    puts_USART1("Students observe: Precise timing without CPU polling!\r\n");
    puts_USART1("Timer ISR triggers every ~1 second\r\n");
    puts_USART1("Press any key to exit\r\n\r\n");

    // EDUCATIONAL: Configure Timer2 for overflow interrupt
    Timer2_init(); // Use shared library for basic setup

    // Enable Timer2 overflow interrupt (students learn this!)
    TIMSK |= (1 << TOIE2);

    // Enable global interrupts
    sei();

    // Reset timer counter
    timer_interrupt_count = 0;

    uint8_t last_timer_count = 0;
    uint32_t main_loop_iterations = 0;

    while (1)
    {
        main_loop_iterations++;

        // EDUCATIONAL: Check if timer ISR has incremented counter
        if (timer_interrupt_count != last_timer_count)
        {
            last_timer_count = timer_interrupt_count;

            char msg[80];
            sprintf(msg, "TIMER ISR: Interrupt #%u triggered (main loops: %lu)\r\n",
                    timer_interrupt_count, main_loop_iterations);
            puts_USART1(msg);

            main_loop_iterations = 0; // Reset counter
        }

        // CPU free to do other work while timer ISR handles timing
        PORTB = ~(main_loop_iterations & 0xFF); // Show CPU activity
        _delay_ms(10);

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            UDR1; // Read and discard character
            puts_USART1("Exiting timer interrupt demo...\r\n");
            break;
        }
    }

    // Disable timer interrupt
    TIMSK &= ~(1 << TOIE2);

    puts_USART1("Timer Interrupt Demo 5 completed.\r\n");
    puts_USART1("Learning: Precise timing without CPU intervention!\r\n");
}

/*
 * Demo 6: Multiple Interrupt Sources
 * Both external and timer interrupts working together
 */
void demo_interrupt_multiple_sources(void)
{
    puts_USART1("\r\n=== DEMO 6: Multiple Interrupt Sources ===\r\n");
    puts_USART1("INTERRUPT METHOD: Multiple ISRs working simultaneously\r\n");
    puts_USART1("INT0 (PD0): External interrupt for button\r\n");
    puts_USART1("INT1 (PD1): Second external interrupt\r\n");
    puts_USART1("TIMER2: Periodic timer interrupt\r\n");
    puts_USART1("Press any key to exit\r\n\r\n");

    // Configure multiple external interrupts
    DDRD &= ~(1 << PD0 | 1 << PD1);
    PORTD |= (1 << PD0 | 1 << PD1);

    // Configure interrupt triggers
    EICRA |= (1 << ISC01) | (1 << ISC11); // Falling edge for both
    EICRA &= ~(1 << ISC00 | 1 << ISC10);

    // Enable both external interrupts
    EIMSK |= (1 << INT0) | (1 << INT1);

    // Configure timer interrupt
    Timer2_init();
    TIMSK |= (1 << TOIE2);

    // Enable global interrupts
    sei();

    // Reset all counters
    external_interrupt_count = 0;
    timer_interrupt_count = 0;
    int0_triggered = 0;
    int1_triggered = 0;

    uint8_t last_timer = 0;

    while (1)
    {
        // Check INT0 interrupt
        if (int0_triggered)
        {
            int0_triggered = 0;
            puts_USART1("ISR: INT0 (PD0) button pressed!\r\n");
        }

        // Check INT1 interrupt
        if (int1_triggered)
        {
            int1_triggered = 0;
            puts_USART1("ISR: INT1 (PD1) button pressed!\r\n");
        }

        // Check timer interrupt
        if (timer_interrupt_count != last_timer)
        {
            last_timer = timer_interrupt_count;
            char msg[50];
            sprintf(msg, "ISR: Timer tick #%u\r\n", timer_interrupt_count);
            puts_USART1(msg);
        }

        // CPU free for other work
        static uint8_t pattern = 0;
        PORTB = ~pattern;
        pattern = (pattern << 1) | (pattern >> 7); // Rotate pattern
        _delay_ms(200);

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            UDR1; // Read and discard
            break;
        }
    }

    // Disable all interrupts
    EIMSK &= ~(1 << INT0 | 1 << INT1);
    TIMSK &= ~(1 << TOIE2);

    char summary[100];
    sprintf(summary, "\r\n--- FINAL STATISTICS ---\r\n");
    puts_USART1(summary);
    sprintf(summary, "INT0 triggers: %u\r\n", external_interrupt_count);
    puts_USART1(summary);
    sprintf(summary, "Timer triggers: %u\r\n", timer_interrupt_count);
    puts_USART1(summary);
    puts_USART1("Multiple interrupts handled simultaneously!\r\n");
}

/*
 * Demo 7: Advanced ISR Techniques
 * Interrupt priority, nesting, and optimization
 */
void demo_interrupt_advanced(void)
{
    puts_USART1("\r\n=== DEMO 7: Advanced ISR Techniques ===\r\n");
    puts_USART1("ADVANCED: Fast ISRs, priority handling, optimization\r\n");
    puts_USART1("Demonstrates proper ISR design principles\r\n");
    puts_USART1("Press PD0 for high-priority interrupt simulation\r\n");
    puts_USART1("Press any serial key to exit\r\n\r\n");

    // Configure for advanced demo
    DDRD &= ~(1 << PD0);
    PORTD |= (1 << PD0);
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);
    EIMSK |= (1 << INT0);

    Timer2_init();
    TIMSK |= (1 << TOIE2);

    sei();

    external_interrupt_count = 0;
    timer_interrupt_count = 0;
    uint32_t performance_counter = 0;

    while (1)
    {
        performance_counter++;

        // Simulate CPU-intensive work
        for (uint16_t i = 0; i < 1000; i++)
        {
            // Busy work - interrupts will preempt this
            volatile uint16_t dummy = i * 2;
        }

        // Check interrupt activity
        if (button_pressed)
        {
            button_pressed = 0;

            // Measure interrupt response time (educational)
            puts_USART1("FAST ISR: Interrupt handled with minimal delay!\r\n");

            char perf_msg[60];
            sprintf(perf_msg, "Performance: %lu work cycles between interrupts\r\n",
                    performance_counter);
            puts_USART1(perf_msg);

            performance_counter = 0;
        }

        // Status display
        if ((performance_counter % 10000) == 0)
        {
            PORTB ^= (1 << 7); // Heartbeat LED
        }

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            UDR1; // Read and discard
            break;
        }
    }

    EIMSK &= ~(1 << INT0);
    TIMSK &= ~(1 << TOIE2);

    puts_USART1("Advanced ISR Demo 7 completed.\r\n");
    puts_USART1("Learning: ISRs should be fast and minimal!\r\n");
}

/*
 * =============================================================================
 * MAIN PROGRAM ENTRY POINT
 * =============================================================================
 */

int main(void)
{
    // Initialize system components
    init_devices();
    Uart1_init();

    puts_USART1("\r\n\r\n");
    puts_USART1("=======================================================\r\n");
    puts_USART1("     ATmega128 Interrupt Programming Methods\r\n");
    puts_USART1("     EDUCATIONAL: Polling vs Interrupt Comparison\r\n");
    puts_USART1("=======================================================\r\n");
    puts_USART1("STUDENTS LEARN:\r\n");
    puts_USART1("✓ Real ISR programming: ISR(INT0_vect), ISR(TIMER2_OVF_vect)\r\n");
    puts_USART1("✓ Direct register access: EIMSK |= (1 << INT0)\r\n");
    puts_USART1("✓ Interrupt configuration: EICRA, TIMSK registers\r\n");
    puts_USART1("✓ Volatile variables for ISR communication\r\n");
    puts_USART1("✓ Performance comparison: blocking vs non-blocking\r\n");
    puts_USART1("=======================================================\r\n\r\n");

    puts_USART1("IMPORTANT: Students edit main() to select ONE demo:\r\n\r\n");

    _delay_ms(2000);

    // ===================================================================
    // EDUCATIONAL SELECTION: Students uncomment ONE demo to learn from
    // ===================================================================

    // =====================================
    // POLLING DEMOS: CPU waits for input
    // =====================================
    // demo_polling_basic_button();        // Demo 1: Basic button polling (CPU blocks)
    // demo_polling_multiple_buttons();    // Demo 2: Multiple button polling (more blocking)
    // demo_polling_with_debounce();       // Demo 3: Polling with debouncing (still blocks)

    // ========================================
    // INTERRUPT DEMOS: CPU continues running
    // ========================================
    demo_interrupt_external_basic(); // Demo 4: Real ISR external interrupt ← ACTIVE
    // demo_interrupt_timer_basic();       // Demo 5: Timer interrupt ISR
    // demo_interrupt_multiple_sources();  // Demo 6: Multiple interrupt sources
    // demo_interrupt_advanced();          // Demo 7: Advanced ISR techniques

    puts_USART1("\r\n=======================================================\r\n");
    puts_USART1("EDUCATIONAL SUMMARY:\r\n");
    puts_USART1("• Polling: Simple but blocks CPU → inefficient for input\r\n");
    puts_USART1("• Interrupts: Complex but frees CPU → efficient and responsive\r\n");
    puts_USART1("• Students must learn ISR syntax and register programming\r\n");
    puts_USART1("• No wrapper functions - direct hardware control only!\r\n");
    puts_USART1("=======================================================\r\n");

    // Keep LED blinking to show program is running
    while (1)
    {
        PORTB ^= (1 << 7); // Toggle LED to show CPU is free
        _delay_ms(1000);
    }

    return 0;
}