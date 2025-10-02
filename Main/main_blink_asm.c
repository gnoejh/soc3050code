/*
 * blink.c
 *
 * Created: 2024-09-22
 * Author : hjeong
 */

#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// Global variables for interrupt-based serial communication
volatile char rx_buffer[64];
volatile uint8_t rx_buffer_index = 0;
volatile uint8_t rx_command_ready = 0;

// Global variables for interrupt-based transmission
volatile char tx_buffer[128];
volatile uint8_t tx_buffer_head = 0;
volatile uint8_t tx_buffer_tail = 0;
volatile uint8_t tx_busy = 0;

// Circular buffer structures for advanced communication
#define CIRC_BUFFER_SIZE 256
typedef struct
{
    volatile char buffer[CIRC_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    volatile uint8_t overflow;
} CircularBuffer;

volatile CircularBuffer circ_rx_buffer = {0};
volatile CircularBuffer circ_tx_buffer = {0};

// Global variables for ADC interrupt processing
volatile unsigned int adc_interrupt_result = 0;
extern volatile uint8_t adc_interrupt_complete; // Defined in _adc.c
volatile uint8_t adc_interrupt_channel = 0;

// Minimal UART function implementations for self-contained assembly example
#ifdef ASSEMBLY_BLINK_BASIC
void puts_USART1(const char *str)
{
    // Simple blocking send for educational purposes
    while (*str)
    {
        while (!(UCSR1A & (1 << UDRE1)))
            ;          // Wait for empty transmit buffer
        UDR1 = *str++; // Send character
    }
}

void putch_USART1(char ch)
{
    while (!(UCSR1A & (1 << UDRE1)))
        ;      // Wait for empty transmit buffer
    UDR1 = ch; // Send character
}

// Minimal function stubs for _init.c compatibility (only when standalone)
void Timer2_init(void) { /* Stub - not needed for basic blink */ }
void Uart1_init(void) { /* Stub - not needed for basic blink */ }
void Adc_init(void) { /* Stub - not needed for basic blink */ }
void lcd_init(void) { /* Stub - not needed for basic blink */ }
#endif

// Interrupt Service Routine for UART1 receive - only needed for serial examples
#if defined(SERIAL_POLLING_SINGLE_CHAR) || defined(SERIAL_POLLING_STRING) || defined(SERIAL_INTERRUPT_CIRCULAR_BUFFER) || defined(SERIAL_INTERRUPT_RX) || defined(SERIAL_INTERRUPT_TX)
ISR(USART1_RX_vect)
{
    char received_char = UDR1;

    if (received_char == '\r' || received_char == '\n')
    {
        if (rx_buffer_index > 0)
        {
            rx_buffer[rx_buffer_index] = '\0'; // Null terminate
            rx_command_ready = 1;              // Signal command ready
        }
    }
    else if (received_char == '\b' || received_char == 127) // Backspace
    {
        if (rx_buffer_index > 0)
        {
            rx_buffer_index--;
            puts_USART1(" \b"); // Erase character on terminal
        }
    }
    else if (rx_buffer_index < sizeof(rx_buffer) - 1)
    {
        rx_buffer[rx_buffer_index++] = received_char;
        putch_USART1(received_char); // Echo character
    }
}

// Interrupt Service Routine for UART1 transmit (TX)
ISR(USART1_UDRE_vect)
{
    if (tx_buffer_head != tx_buffer_tail)
    {
        // Send next character from buffer
        UDR1 = tx_buffer[tx_buffer_tail];
        tx_buffer_tail = (tx_buffer_tail + 1) % sizeof(tx_buffer);
    }
    else
    {
        // Buffer empty, disable TX interrupt
        UCSR1B &= ~(1 << UDRIE1);
        tx_busy = 0;
    }
}

// Function to add string to TX buffer
void interrupt_puts(const char *str)
{
    while (*str)
    {
        // Wait if buffer is full
        uint8_t next_head = (tx_buffer_head + 1) % sizeof(tx_buffer);
        while (next_head == tx_buffer_tail && tx_busy)
            ;

        // Add character to buffer
        tx_buffer[tx_buffer_head] = *str++;
        tx_buffer_head = next_head;
    }

    // Enable TX interrupt if not already running
    if (!tx_busy)
    {
        tx_busy = 1;
        UCSR1B |= (1 << UDRIE1);
    }
}

// Circular buffer utility functions
uint8_t circ_buffer_put(volatile CircularBuffer *cb, char data)
{
    if (cb->count >= CIRC_BUFFER_SIZE)
    {
        cb->overflow = 1;
        return 0; // Buffer full
    }

    cb->buffer[cb->head] = data;
    cb->head = (cb->head + 1) % CIRC_BUFFER_SIZE;
    cb->count++;
    return 1; // Success
}

uint8_t circ_buffer_get(volatile CircularBuffer *cb, char *data)
{
    if (cb->count == 0)
        return 0; // Buffer empty

    *data = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % CIRC_BUFFER_SIZE;
    cb->count--;
    return 1; // Success
}

uint16_t circ_buffer_available(volatile CircularBuffer *cb)
{
    return cb->count;
}

uint16_t circ_buffer_free_space(volatile CircularBuffer *cb)
{
    return CIRC_BUFFER_SIZE - cb->count;
}

void circ_buffer_clear(volatile CircularBuffer *cb)
{
    cb->head = cb->tail = cb->count = 0;
    cb->overflow = 0;
}
#endif // End of serial/UART ISR and functions

// Circular buffer ISRs (used conditionally)
#ifdef SERIAL_INTERRUPT_CIRCULAR_BUFFER
// Override default ISRs for circular buffer operation
ISR(USART1_RX_vect, ISR_ALIASOF(USART1_RX_vect_circ));
ISR(USART1_UDRE_vect, ISR_ALIASOF(USART1_UDRE_vect_circ));

ISR(USART1_RX_vect_circ)
{
    char received_char = UDR1;
    circ_buffer_put(&circ_rx_buffer, received_char);
}

ISR(USART1_UDRE_vect_circ)
{
    char data;
    if (circ_buffer_get(&circ_tx_buffer, &data))
    {
        UDR1 = data;
    }
    else
    {
        // No more data to send, disable interrupt
        UCSR1B &= ~(1 << UDRIE1);
    }
}

// ADC Interrupt Service Routine
ISR(ADC_vect)
{
    adc_interrupt_result = ADC;
    adc_interrupt_complete = 1;
}
#endif

#ifdef ASSEMBLY_BLINK_BASIC
/* address version */
int main_blink_asm(void)
{
    // Set DDRB as output
    asm volatile(
        "ldi r16, 0xFF\n\t" // Load immediate value 0xFF into register r16
        "out 0x17, r16\n\t" // Output r16 to DDRB (Port B Data Direction Register, address 0x04)
    );

    while (1)
    {
        // Set PORTB = 0xAA
        asm volatile(
            "ldi r16, 0xAA\n\t" // Load 0xAA into register r16
            "out 0x18, r16\n\t" // Output r16 to PORTB (Port B Data Register, address 0x05)
        );

        _delay_ms(2000); // 1-second delay

        // Set PORTB = 0x55
        asm volatile(
            "ldi r16, 0x55\n\t" // Load 0x55 into register r16
            "out 0x18, r16\n\t" // Output r16 to PORTB
        );

        _delay_ms(1000); // 1-second delay
    }
}
#endif

#ifdef ASSEMBLY_BLINK_PATTERN
/* macro version  */
int main_blink_asm_macro(void)
{
    // Set DDRB as output using register name
    asm volatile(
        "ldi r16, 0xFF\n\t"              // Load 0xFF into r16
        "out %[ddrb], r16\n\t"           // Output r16 to DDRB
        :                                /* no output */
        : [ddrb] "I"(_SFR_IO_ADDR(DDRB)) // map DDRB to its I/O address
        : "r16"                          // clobber (modified) list
    );

    while (1)
    {
        // Set PORTB = 0xAA using register name
        asm volatile(
            "ldi r16, 0xAA\n\t"                // Load 0xAA into r16
            "out %[portb], r16\n\t"            // Output r16 to PORTB
            :                                  /* no output */
            : [portb] "I"(_SFR_IO_ADDR(PORTB)) // map PORTB to its I/O address
            : "r16"                            // clobber list
        );

        _delay_ms(1000); // 1-second delay

        // Set PORTB = 0x55 using register name
        asm volatile(
            "ldi r16, 0x55\n\t"                // Load 0x55 into r16
            "out %[portb], r16\n\t"            // Output r16 to PORTB
            :                                  /* no output */
            : [portb] "I"(_SFR_IO_ADDR(PORTB)) // map PORTB to its I/O address
            : "r16"                            // clobber list
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
uint8_t my_random(void)
{
    // Linear Congruential Generator (LCG) parameters
    seed = (seed * 1103515245 + 12345); // Update seed
    return (uint8_t)(seed & 0xFF);      // Return an 8-bit pseudo-random number
}

int main_blink_asm_random(void)
{
    // Set DDRB as output using inline assembly
    asm volatile(
        "ldi r16, 0xFF\n\t"              // Load 0xFF into r16 (all bits high)
        "out %[ddrb], r16\n\t"           // Output r16 to DDRB (set PORTB as output)
        :                                /* no output */
        : [ddrb] "I"(_SFR_IO_ADDR(DDRB)) // map DDRB to its I/O address
        : "r16");

    while (1)
    {
        // Set PORTB with a random 8-bit value
        asm volatile(
            "out %[portb], %[rand_val]\n\t"     // Output random value directly to PORTB
            :                                   /* no output */
            : [portb] "I"(_SFR_IO_ADDR(PORTB)), // map PORTB to its I/O address
              [rand_val] "r"(my_random())       // Get a random 8-bit value for PORTB
        );

        // Delay for a random period between 100ms and 1000ms
        _delay_ms(1000); // Generate a random delay between 100ms and 1000ms
    }
}
#endif

#ifdef BLINK_ASM_RANDOM_DELAY
/* port delay random */
// Simple pseudo-random number generator (PRNG) seed
uint16_t seed = 12345; // Initial seed value

// Function to generate a pseudo-random number
uint8_t my_random(void)
{
    // Linear Congruential Generator (LCG) parameters
    seed = (seed * 1103515245 + 12345); // Update seed
    return (uint8_t)(seed & 0xFF);      // Return an 8-bit pseudo-random number
}

// Custom delay function that delays for approximately the given number of milliseconds
void custom_delay_ms(uint16_t ms)
{
    while (ms--)
    {
        _delay_ms(1); // Delay 1 millisecond in each loop iteration
    }
}

void main_blink_asm_random_delay(void)
{
    // Set DDRB as output using inline assembly
    asm volatile(
        "ldi r16, 0xFF\n\t"              // Load 0xFF into r16 (all bits high)
        "out %[ddrb], r16\n\t"           // Output r16 to DDRB (set PORTB as output)
        :                                /* no output */
        : [ddrb] "I"(_SFR_IO_ADDR(DDRB)) // map DDRB to its I/O address
        : "r16");

    while (1)
    {
        // Set PORTB with a random 8-bit value
        asm volatile(
            "out %[portb], %[rand_val]\n\t"     // Output random value directly to PORTB
            :                                   /* no output */
            : [portb] "I"(_SFR_IO_ADDR(PORTB)), // map PORTB to its I/O address
              [rand_val] "r"(my_random())       // Get a random 8-bit value for PORTB
        );

        // Delay for a random period between 100ms and 1000ms using custom delay function
        custom_delay_ms(100 + (my_random() % 900)); // Generate a random delay between 100ms and 1000ms
    }
}
#endif

/*
 * Individual LED Control with Button Direction Control
 * Educational demonstration of button input and LED rotation
 */
#ifdef ASSEMBLY_BLINK_INDIVIDUAL
void main_blink_pin(void)
{
    /*
     * EDUCATIONAL STEP 1: Initialize ports using modernized library
     */
    Port_init(); // Initialize port system

    /*
     * EDUCATIONAL STEP 2: Initialize state machine variables
     */
    unsigned char direction = 0;         // 0 = clockwise, 1 = counterclockwise
    unsigned char led_state = 0x01;      // Start with first LED (binary: 00000001)
    unsigned char last_button_state = 1; // Track button state for edge detection

    /*
     * EDUCATIONAL STEP 3: Main control loop with state machine
     */
    while (1)
    {
        /*
         * EDUCATIONAL STEP 3.1: Button input processing
         */
        unsigned char current_button_state = read_buttons();

        /*
         * EDUCATIONAL STEP 3.2: Edge detection (falling edge)
         */
        if (current_button_state == 0 && last_button_state != 0)
        {
            direction = !direction; // Toggle direction on button press
        }

        /* Update button state for next iteration */
        last_button_state = current_button_state;

        /*
         * EDUCATIONAL STEP 3.3: LED pattern generation
         */
        if (direction == 0) // Clockwise
        {
            led_pattern(led_state);
            led_state <<= 1;       // Shift left (next LED)
            if (led_state == 0x00) // Wrap around check
            {
                led_state = 0x01; // Reset to first LED
            }
        }
        else // Counterclockwise
        {
            led_pattern(led_state); // Output pattern
            led_state >>= 1;        // Shift right (previous LED)
            if (led_state == 0x00)  // Wrap around check
            {
                led_state = 0x80; // Reset to last LED
            }
        }

        _delay_ms(500);
    }
}
#endif

/*
 * Simple Button Reading Example
 * Educational demonstration of basic button input
 */
#ifdef ASSEMBLY_BUTTON_SIMPLE
void main_button_simple(void)
{
    // Initialize port system
    Port_init();

    while (1)
    {
        // Read button state and control LEDs accordingly
        unsigned char button_state = read_buttons();

        if (button_state == 0) // Button pressed (active low)
        {
            led_on(0); // Turn on LED 0 as indicator
        }
        else
        {
            led_off(0); // Turn off LED 0 as indicator
        }

        // Small delay for stability
        _delay_ms(50);
    }
}
#endif

/*
 * Button-Controlled LED Example
 * Educational demonstration of interactive LED control with buttons
 */
#ifdef ASSEMBLY_BUTTON_LED_CONTROL
void main_button_led_control(void)
{
    // Initialize port system
    Port_init();

    unsigned char led_index = 0; // Current LED index (0-7)

    while (1)
    {
        // Read button state
        unsigned char button_state = read_buttons();

        // Clear all LEDs first
        for (int i = 0; i < 8; i++)
        {
            led_off(i);
        }

        // Light up current LED
        led_on(led_index);

        // Change LED on button press
        if (button_state == 0) // Button pressed (active low)
        {
            led_index = (led_index + 1) % 8; // Move to next LED, wrap around
            _delay_ms(200);                  // Debounce delay
        }

        _delay_ms(50); // Main loop delay
    }
}
#endif

/*
 * C LED Basic Control Example
 * Educational demonstration of C-based LED control using library functions
 */
#ifdef C_LED_BASIC
void main_c_led_basic(void)
{
    // Initialize port system using C library function
    Port_init();

    while (1)
    {
        // Turn on LEDs in sequence using C functions
        for (unsigned char i = 0; i < 8; i++)
        {
            led_on(i);
            _delay_ms(200);
        }

        // Turn off LEDs in sequence using C functions
        for (unsigned char i = 0; i < 8; i++)
        {
            led_off(i);
            _delay_ms(200);
        }

        // Demonstrate pattern function
        led_pattern(0xAA); // Alternating pattern
        _delay_ms(500);
        led_pattern(0x55); // Opposite pattern
        _delay_ms(500);
        led_pattern(0x00); // All LEDs off
        _delay_ms(500);
    }
}
#endif

/*
 * C LED Patterns Example
 * Educational demonstration of advanced LED patterns using C library functions
 */
#ifdef C_LED_PATTERNS
void main_c_led_patterns(void)
{
    Port_init(); // Initialize port system

    while (1)
    {
        // Knight Rider effect
        for (unsigned char cycle = 0; cycle < 3; cycle++)
        {
            // Forward sweep
            for (unsigned char i = 0; i < 8; i++)
            {
                led_pattern(1 << i);
                _delay_ms(150);
            }
            // Backward sweep
            for (unsigned char i = 7; i > 0; i--)
            {
                led_pattern(1 << i);
                _delay_ms(150);
            }
        }

        // Chasing pattern
        for (unsigned char cycle = 0; cycle < 5; cycle++)
        {
            led_pattern(0x81); // Ends
            _delay_ms(200);
            led_pattern(0x42); // Moving in
            _delay_ms(200);
            led_pattern(0x24); // Closer
            _delay_ms(200);
            led_pattern(0x18); // Center
            _delay_ms(200);
        }

        // Binary counting pattern
        for (unsigned char count = 0; count < 16; count++)
        {
            led_pattern(count);
            _delay_ms(300);
        }

        // Breathing effect
        unsigned char patterns[] = {0x00, 0x18, 0x3C, 0x7E, 0xFF, 0x7E, 0x3C, 0x18};
        for (unsigned char cycle = 0; cycle < 4; cycle++)
        {
            for (unsigned char i = 0; i < 8; i++)
            {
                led_pattern(patterns[i]);
                _delay_ms(200);
            }
        }

        led_pattern(0x00); // All off
        _delay_ms(1000);   // Pause before repeating
    }
}
#endif

/*
 * C LED Button Interactive Example
 * Educational demonstration of interactive button-LED control using C abstraction
 */
#ifdef C_LED_BUTTON_INTERACTIVE
void main_c_led_button_interactive(void)
{
    Port_init(); // Initialize port system

    unsigned char current_pattern = 0;
    unsigned char last_button_state = 1; // Assume button not pressed initially

    while (1)
    {
        unsigned char button_state = read_buttons();

        // Detect button press (transition from high to low)
        if (last_button_state == 1 && button_state == 0)
        {
            current_pattern = (current_pattern + 1) % 8; // Cycle through 8 patterns

            // Display different patterns based on current_pattern
            switch (current_pattern)
            {
            case 0:
                led_pattern(0x01); // Single LED
                break;
            case 1:
                led_pattern(0x03); // Two LEDs
                break;
            case 2:
                led_pattern(0x0F); // Four LEDs
                break;
            case 3:
                led_pattern(0xFF); // All LEDs
                break;
            case 4:
                led_pattern(0xAA); // Alternating pattern
                break;
            case 5:
                led_pattern(0x55); // Inverse alternating
                break;
            case 6:
                led_pattern(0x18); // Center two LEDs
                break;
            case 7:
                led_pattern(0x81); // Corner LEDs
                break;
            }

            _delay_ms(200); // Debounce delay
        }

        last_button_state = button_state;
        _delay_ms(50); // Main loop delay
    }
}
#endif

/*
 * Serial Communication Example
 * Educational demonstration of UART communication using C abstraction
 */
#ifdef SERIAL_POLLING_SINGLE_CHAR
void main_serial_polling_single_char(void)
{
    Uart1_init(); // Initialize UART1 with default settings

    putch_USART1('A'); // Send initial character
    puts_USART1("\r\nUART Echo Test - Type characters:\r\n");

    while (1)
    {
        unsigned char received_char = getch_USART1(); // Blocking receive
        putch_USART1(received_char);                  // Echo back to terminal

        if (received_char == '\r') // If Enter key pressed
        {
            putch_USART1('\n'); // Add line feed
        }
    }
}
#endif

/*
 * ADC Basic Reading Example
 * Educational demonstration of analog-to-digital conversion using C abstraction
 */
#ifdef ADC_BASIC_READING
void main_adc_basic_reading(void)
{
    Adc_init();   // Initialize ADC system
    Uart1_init(); // Initialize UART for displaying results

    puts_USART1("\r\nADC Basic Reading Test\r\n");
    puts_USART1("Reading ADC Channel 0...\r\n");

    while (1)
    {
        // Read ADC value from channel 0
        unsigned int adc_value = Read_Adc_Data(0);

        // Display raw ADC value
        puts_USART1("ADC Value: ");
        USART1_print_decimal(adc_value);
        puts_USART1(" (0-1023)\r\n");

        // Convert to voltage in millivolts
        unsigned int voltage_mv = Read_Adc_Voltage_mV(0);
        puts_USART1("Voltage: ");
        USART1_print_decimal(voltage_mv);
        puts_USART1(" mV\r\n");

        // Read averaged value for noise reduction
        unsigned int averaged = Read_Adc_Averaged(0, 8);
        puts_USART1("Averaged (8 samples): ");
        USART1_print_decimal(averaged);
        puts_USART1("\r\n\r\n");

        _delay_ms(1000); // Update every second
    }
}
#endif

/*
 * Buzzer Basic Beep Example
 * Educational demonstration of audio output using C abstraction
 */
#ifdef BUZZER_BASIC_BEEP
void main_buzzer_basic_beep(void)
{
    Buzzer_init(); // Initialize buzzer system

    while (1)
    {
        // Basic beep demonstration
        Sound(500, 200); // 500Hz tone for 200ms
        _delay_ms(500);  // Silent pause

        // Different frequency beeps
        Sound(800, 150); // Higher pitch, shorter duration
        _delay_ms(300);

        Sound(300, 300); // Lower pitch, longer duration
        _delay_ms(400);

        // Pre-defined sound effects
        S_Good(); // Success sound
        _delay_ms(800);

        S_Push1(); // Button press sound
        _delay_ms(600);

        S_Start(); // Startup sequence
        _delay_ms(1000);

        // Musical demonstration
        S_Star(); // Twinkle Twinkle opening
        _delay_ms(1500);

        // Frequency sweep demonstration
        for (unsigned int freq = 200; freq <= 1000; freq += 100)
        {
            Sound(freq, 100);
            _delay_ms(50);
        }

        _delay_ms(2000); // Long pause before repeating
    }
}
#endif

/*
 * C Timer Basic Example
 * Educational demonstration of timer functionality using C abstraction
 */
#ifdef C_TIMER_BASIC
void main_timer_basic(void)
{
    Port_init();    // Initialize port system
    Timer2_init();  // Initialize Timer2 for 1ms interrupts
    Timer2_start(); // Start timer operation

    // Timer-based LED blinking demonstration
    unsigned int led_state = 0;
    unsigned long last_time = 0;
    const unsigned int blink_interval = 500; // 500ms

    while (1)
    {
        unsigned long current_time = Timer2_get_milliseconds();

        // Non-blocking timer-based LED toggle
        if (current_time - last_time >= blink_interval)
        {
            if (led_state == 0)
            {
                PORTB = 0xFF; // All LEDs on
                led_state = 1;
            }
            else
            {
                PORTB = 0x00; // All LEDs off
                led_state = 0;
            }
            last_time = current_time;
        }

        // Demonstrate task scheduling with timer
        if (Timer2_check_task1())
        { // Task 1: Every 500ms (default)
            // Toggle single LED to show Task 1 execution
            PORTB ^= 0x01;
        }

        if (Timer2_check_task2())
        { // Task 2: Every 100ms (default)
            // Brief flash to show Task 2 execution
            PORTC = 0xFF;
            _delay_ms(10);
            PORTC = 0x00;
        }

        if (Timer2_check_task3())
        { // Task 3: Every 1000ms (default)
            // Show timer activity with indicator
            PORTD ^= 0x80;

            // Optional: Show uptime (would normally use UART)
            unsigned long uptime = Timer2_get_milliseconds();
            // Real application would transmit uptime via UART
        }
    }
}
#endif

/*
 * C Timer Interrupt Example
 * Educational demonstration of timer interrupts and ISR handling
 */
#ifdef C_TIMER_INTERRUPT
void main_timer_interrupt(void)
{
    Port_init();    // Initialize port system
    Timer2_init();  // Initialize Timer2 for 1ms interrupts
    sei();          // Enable global interrupts
    Timer2_start(); // Start timer operation

    // Variables for interrupt-driven LED control
    static unsigned char led_pattern = 0x01;
    static unsigned char pattern_direction = 0; // 0 = shift left, 1 = shift right

    while (1)
    {
        // Task 1: LED pattern animation (interrupt-driven timing)
        if (Timer2_check_task1())
        { // Every 500ms (default)
            // Animate LED pattern using interrupts
            PORTB = led_pattern;

            // Shift pattern left or right
            if (pattern_direction == 0)
            {
                led_pattern <<= 1;
                if (led_pattern >= 0x80)
                {
                    pattern_direction = 1;
                }
            }
            else
            {
                led_pattern >>= 1;
                if (led_pattern <= 0x01)
                {
                    pattern_direction = 0;
                }
            }
        }

        // Task 2: Fast blink indicator (interrupt-driven)
        if (Timer2_check_task2())
        { // Every 100ms (default)
            // Toggle indicator LED to show interrupt activity
            PORTC ^= 0x01;
        }

        // Task 3: Heartbeat (interrupt-driven)
        if (Timer2_check_task3())
        { // Every 1000ms (default)
            // Double-blink heartbeat pattern
            PORTD = 0xFF;
            _delay_ms(50);
            PORTD = 0x00;
            _delay_ms(50);
            PORTD = 0xFF;
            _delay_ms(50);
            PORTD = 0x00;

            // Show system uptime using timer interrupt system
            unsigned long uptime_seconds = Timer2_get_milliseconds() / 1000;
            // In a real application, this would be transmitted via UART
            // For demonstration, show binary representation on PORTD
            PORTD = (unsigned char)(uptime_seconds & 0xFF);
        }

        // Main loop remains free for other tasks
        // This demonstrates the power of interrupt-driven programming
        // where timing tasks don't block the main execution flow
    }
}
#endif

/*
 * C Timer PWM Example
 * Educational demonstration of PWM (Pulse Width Modulation) using timer-based control
 */
#ifdef C_TIMER_PWM
void main_timer_pwm(void)
{
    Port_init();    // Initialize port system
    Timer2_init();  // Initialize Timer2 for precise timing
    sei();          // Enable global interrupts
    Timer2_start(); // Start timer operation

    // PWM variables
    unsigned char pwm_duty_cycle = 0;     // PWM duty cycle (0-100%)
    unsigned char pwm_counter = 0;        // PWM cycle counter
    unsigned char fade_direction = 0;     // 0 = fade up, 1 = fade down
    const unsigned char PWM_PERIOD = 100; // PWM period in timer ticks

    while (1)
    {
        // Software PWM implementation using timer interrupts
        if (Timer2_check_task2())
        { // Fast PWM update (every 100ms default)
            pwm_counter++;
            if (pwm_counter >= PWM_PERIOD)
            {
                pwm_counter = 0;
            }

            // Generate PWM signal on PORTB
            if (pwm_counter < pwm_duty_cycle)
            {
                PORTB = 0xFF; // PWM ON
            }
            else
            {
                PORTB = 0x00; // PWM OFF
            }
        }

        // Update PWM duty cycle for fading effect
        if (Timer2_check_task1())
        { // Slower update (every 500ms default)
            if (fade_direction == 0)
            {
                pwm_duty_cycle += 5; // Increase brightness
                if (pwm_duty_cycle >= 95)
                {
                    fade_direction = 1;
                }
            }
            else
            {
                pwm_duty_cycle -= 5; // Decrease brightness
                if (pwm_duty_cycle <= 5)
                {
                    fade_direction = 0;
                }
            }

            // Show current duty cycle on PORTC (0-255 scale)
            PORTC = (pwm_duty_cycle * 255) / 100;
        }

        // Multi-channel PWM demonstration
        if (Timer2_check_task3())
        { // Slowest update (every 1000ms default)
            // Channel 2: Different PWM pattern on PORTD
            static unsigned char channel2_duty = 25;
            static unsigned char channel2_dir = 0;

            if (channel2_dir == 0)
            {
                channel2_duty += 25;
                if (channel2_duty >= 75)
                    channel2_dir = 1;
            }
            else
            {
                channel2_duty -= 25;
                if (channel2_duty <= 25)
                    channel2_dir = 0;
            }

            // Simple PWM for second channel
            for (int i = 0; i < 100; i++)
            {
                if (i < channel2_duty)
                {
                    PORTD = 0xFF;
                }
                else
                {
                    PORTD = 0x00;
                }
                _delay_ms(1); // 1ms per PWM step = 100ms period
            }
        }

        // Demonstrate PWM frequency control
        // Real hardware PWM would use timer compare registers
        // This software implementation shows the principle
    }
}
#endif

/*
 * Serial String Communication Example
 * Educational demonstration of string-based UART communication
 */
#ifdef SERIAL_POLLING_STRING
void main_serial_polling_string(void)
{
    Port_init();  // Initialize port system
    Uart1_init(); // Initialize UART for string communication

    // Send welcome message
    puts_USART1("\r\n=== String Communication Test ===\r\n");
    puts_USART1("Enter strings (press Enter to send):\r\n");
    puts_USART1("Type 'led on' to turn on LEDs\r\n");
    puts_USART1("Type 'led off' to turn off LEDs\r\n");
    puts_USART1("Type 'help' for commands\r\n\r\n");

    char input_buffer[32]; // Buffer for input strings
    unsigned char buffer_index = 0;
    unsigned char led_state = 0;

    while (1)
    {
        // Simple polling method: check if data received
        // In a real implementation, you'd use interrupt-based or check UCSR1A register
        // For this educational demo, we'll simulate with basic checks

        // Check UART receive flag (UCSR1A register, RXC1 bit)
        if (UCSR1A & (1 << RXC1)) // Data received
        {
            char received_char = UDR1; // Read data directly from register

            // Echo character back to terminal
            putch_USART1(received_char);

            // Handle different characters
            if (received_char == '\r' || received_char == '\n')
            {
                // End of string - process command
                input_buffer[buffer_index] = '\0'; // Null terminate
                puts_USART1("\r\n");               // New line

                // Process command strings
                if (strcmp(input_buffer, "led on") == 0)
                {
                    PORTB = 0xFF; // Turn on all LEDs
                    led_state = 1;
                    puts_USART1("LEDs turned ON\r\n");
                }
                else if (strcmp(input_buffer, "led off") == 0)
                {
                    PORTB = 0x00; // Turn off all LEDs
                    led_state = 0;
                    puts_USART1("LEDs turned OFF\r\n");
                }
                else if (strcmp(input_buffer, "status") == 0)
                {
                    puts_USART1("LED Status: ");
                    if (led_state)
                        puts_USART1("ON\r\n");
                    else
                        puts_USART1("OFF\r\n");
                }
                else if (strcmp(input_buffer, "help") == 0)
                {
                    puts_USART1("Available commands:\r\n");
                    puts_USART1("  led on  - Turn on LEDs\r\n");
                    puts_USART1("  led off - Turn off LEDs\r\n");
                    puts_USART1("  status  - Show LED status\r\n");
                    puts_USART1("  help    - Show this help\r\n");
                }
                else if (buffer_index > 0)
                {
                    puts_USART1("Unknown command: ");
                    puts_USART1(input_buffer);
                    puts_USART1("\r\nType 'help' for available commands\r\n");
                }

                // Reset buffer for next command
                buffer_index = 0;
                puts_USART1("\r\n> "); // Prompt for next command
            }
            else if (received_char == '\b' || received_char == 127) // Backspace
            {
                if (buffer_index > 0)
                {
                    buffer_index--;
                    puts_USART1(" \b"); // Erase character on terminal
                }
            }
            else if (buffer_index < sizeof(input_buffer) - 1)
            {
                // Add character to buffer
                input_buffer[buffer_index++] = received_char;
            }
        }

        // Visual feedback - blink LED to show system is active
        static unsigned int blink_counter = 0;
        if (++blink_counter > 50000)
        {
            PORTC ^= 0x01; // Toggle heartbeat LED
            blink_counter = 0;
        }
    }
}

// Interrupt-based Serial Communication Example
void main_serial_interrupt_rx(void)
{
    Port_init();
    Uart1_init();
    sei(); // Enable global interrupts

    puts_USART1("Interrupt-Based Serial Receiver Example\n");
    puts_USART1("Commands: 'led on', 'led off', 'status', 'help'\n");
    puts_USART1("System running with interrupt-based input...\n");

    while (1)
    {
        // Check if complete command received
        if (rx_command_ready)
        {
            rx_command_ready = 0;

            // Process command
            if (strcmp(rx_buffer, "led on") == 0)
            {
                PORTC |= 0x01;
                puts_USART1("LED turned ON\n");
            }
            else if (strcmp(rx_buffer, "led off") == 0)
            {
                PORTC &= ~0x01;
                puts_USART1("LED turned OFF\n");
            }
            else if (strcmp(rx_buffer, "status") == 0)
            {
                puts_USART1("LED status: ");
                puts_USART1((PORTC & 0x01) ? "ON\n" : "OFF\n");
            }
            else if (strcmp(rx_buffer, "help") == 0)
            {
                puts_USART1("Available commands:\n");
                puts_USART1("  led on  - Turn LED on\n");
                puts_USART1("  led off - Turn LED off\n");
                puts_USART1("  status  - Show LED status\n");
                puts_USART1("  help    - Show this help\n");
            }
            else if (strlen(rx_buffer) > 0)
            {
                puts_USART1("Unknown command: ");
                puts_USART1(rx_buffer);
                puts_USART1(" (type 'help' for commands)\n");
            }

            // Clear buffer
            rx_buffer_index = 0;
            rx_buffer[0] = '\0';
        }

        // Background task - heartbeat LED
        static unsigned int counter = 0;
        if (++counter > 30000)
        {
            PORTC ^= 0x02; // Toggle heartbeat LED on PC1
            counter = 0;
        }
    }
}

// Interrupt-based Serial Transmission Example
void main_serial_interrupt_tx(void)
{
    Port_init();
    Uart1_init();
    sei(); // Enable global interrupts

    interrupt_puts("Interrupt-Based Serial Transmission Example\n");
    interrupt_puts("This demonstrates interrupt-driven transmission\n");
    interrupt_puts("The system can do other work while transmitting\n\n");

    unsigned int message_count = 0;

    while (1)
    {
        // Generate periodic messages using interrupt transmission
        static unsigned int timer = 0;
        if (++timer > 65000)
        {
            char message[64];
            sprintf(message, "Message #%u - Sent via interrupts\n", ++message_count);
            interrupt_puts(message);

            timer = 0;
        }

        // Background task - LED pattern to show system is active
        static unsigned int led_timer = 0;
        if (++led_timer > 20000)
        {
            PORTC ^= 0x01; // Toggle LED to show non-blocking operation
            led_timer = 0;
        }

        // Additional background work could be done here
        // This demonstrates non-blocking transmission
        static unsigned int work_counter = 0;
        work_counter++; // Simulate background processing

        // Button check (if available)
        if (!(PINB & 0x01)) // If button pressed
        {
            interrupt_puts("Button pressed - instant response!\n");
            _delay_ms(200); // Debounce
        }
    }
}

// Interrupt-based Serial Echo Example
void main_serial_interrupt_echo(void)
{
    Port_init();
    Uart1_init();
    sei(); // Enable global interrupts

    interrupt_puts("Interrupt-Based Serial Echo Example\n");
    interrupt_puts("Type characters - they will be echoed back\n");
    interrupt_puts("Special commands:\n");
    interrupt_puts("  'reset' - Clear counters\n");
    interrupt_puts("  'stats' - Show character statistics\n");
    interrupt_puts("  'help'  - Show this help\n\n");

    unsigned int char_count = 0;
    unsigned int line_count = 0;
    char echo_buffer[32];

    while (1)
    {
        // Check if complete command received via interrupts
        if (rx_command_ready)
        {
            rx_command_ready = 0;
            char_count += strlen((char *)rx_buffer);
            line_count++;

            // Process special commands
            if (strcmp((char *)rx_buffer, "reset") == 0)
            {
                char_count = 0;
                line_count = 0;
                interrupt_puts("Counters reset!\n");
            }
            else if (strcmp((char *)rx_buffer, "stats") == 0)
            {
                sprintf(echo_buffer, "Characters: %u, Lines: %u\n", char_count, line_count);
                interrupt_puts(echo_buffer);
            }
            else if (strcmp((char *)rx_buffer, "help") == 0)
            {
                interrupt_puts("Available commands:\n");
                interrupt_puts("  reset - Clear counters\n");
                interrupt_puts("  stats - Show statistics\n");
                interrupt_puts("  help  - Show this help\n");
            }
            else
            {
                // Echo the received text with formatting
                interrupt_puts("Echo: [");
                interrupt_puts((char *)rx_buffer);
                interrupt_puts("]\n");
            }

            // Clear buffer for next input
            rx_buffer_index = 0;
            rx_buffer[0] = '\0';
        }

        // Background task - status LED blinks to show system activity
        static unsigned int status_timer = 0;
        if (++status_timer > 50000)
        {
            PORTC ^= 0x04; // Toggle status LED on PC2
            status_timer = 0;
        }

        // Periodic heartbeat message
        static unsigned int heartbeat_timer = 0;
        static unsigned int heartbeat_count = 0;
        if (++heartbeat_timer > 1000000) // Very slow heartbeat
        {
            sprintf(echo_buffer, "[Heartbeat #%u]\n", ++heartbeat_count);
            interrupt_puts(echo_buffer);
            heartbeat_timer = 0;
        }
    }
}

// Interrupt-based Serial Sentence Processing Example
void main_serial_interrupt_sentence(void)
{
    Port_init();
    Uart1_init();
    sei(); // Enable global interrupts

    interrupt_puts("Interrupt-Based Sentence Processing Example\n");
    interrupt_puts("Enter sentences for analysis:\n");
    interrupt_puts("Commands:\n");
    interrupt_puts("  'analyze <text>' - Analyze text\n");
    interrupt_puts("  'reverse <text>' - Reverse text\n");
    interrupt_puts("  'upper <text>'   - Convert to uppercase\n");
    interrupt_puts("  'count'         - Show statistics\n");
    interrupt_puts("  'clear'         - Clear statistics\n\n");

    unsigned int total_sentences = 0;
    unsigned int total_words = 0;
    unsigned int total_chars = 0;
    char response[128];

    while (1)
    {
        if (rx_command_ready)
        {
            rx_command_ready = 0;

            char *input = (char *)rx_buffer;
            int len = strlen(input);
            total_chars += len;
            total_sentences++;

            // Process different commands
            if (strncmp(input, "analyze ", 8) == 0)
            {
                char *text = input + 8;
                int words = 1;
                int vowels = 0;
                int consonants = 0;

                // Count words, vowels, consonants
                for (int i = 0; text[i]; i++)
                {
                    if (text[i] == ' ' && text[i + 1] != ' ' && text[i + 1] != '\0')
                        words++;
                    char c = text[i] | 0x20; // Convert to lowercase
                    if (c >= 'a' && c <= 'z')
                    {
                        if (c == 'a' || c == 'e' || c == 'i' || c == 'o' || c == 'u')
                            vowels++;
                        else
                            consonants++;
                    }
                }
                total_words += words;

                sprintf(response, "Analysis: %d words, %d vowels, %d consonants\n", words, vowels, consonants);
                interrupt_puts(response);
            }
            else if (strncmp(input, "reverse ", 8) == 0)
            {
                char *text = input + 8;
                int text_len = strlen(text);

                interrupt_puts("Reversed: ");
                for (int i = text_len - 1; i >= 0; i--)
                {
                    char temp[2] = {text[i], '\0'};
                    interrupt_puts(temp);
                }
                interrupt_puts("\n");
            }
            else if (strncmp(input, "upper ", 6) == 0)
            {
                char *text = input + 6;
                interrupt_puts("Uppercase: ");

                for (int i = 0; text[i]; i++)
                {
                    char c = text[i];
                    if (c >= 'a' && c <= 'z')
                        c -= 32; // Convert to uppercase
                    char temp[2] = {c, '\0'};
                    interrupt_puts(temp);
                }
                interrupt_puts("\n");
            }
            else if (strcmp(input, "count") == 0)
            {
                sprintf(response, "Statistics: %u sentences, %u words, %u characters\n",
                        total_sentences, total_words, total_chars);
                interrupt_puts(response);
            }
            else if (strcmp(input, "clear") == 0)
            {
                total_sentences = total_words = total_chars = 0;
                interrupt_puts("Statistics cleared!\n");
            }
            else
            {
                // Default sentence analysis
                int words = 1;
                for (int i = 0; input[i]; i++)
                {
                    if (input[i] == ' ' && input[i + 1] != ' ' && input[i + 1] != '\0')
                        words++;
                }
                total_words += words;

                sprintf(response, "Received: \"%s\" (%d words)\n", input, words);
                interrupt_puts(response);
            }

            // Clear buffer
            rx_buffer_index = 0;
            rx_buffer[0] = '\0';
        }

        // Background processing indicator
        static unsigned int process_timer = 0;
        if (++process_timer > 40000)
        {
            PORTC ^= 0x08; // Toggle processing LED on PC3
            process_timer = 0;
        }
    }
}

// Interrupt-based Serial with Circular Buffer Example
void main_serial_interrupt_circular_buffer(void)
{
    Port_init();
    Uart1_init();

    // Clear buffers
    circ_buffer_clear(&circ_rx_buffer);
    circ_buffer_clear(&circ_tx_buffer);

    sei(); // Enable global interrupts

    char message[128];
    sprintf(message, "Circular Buffer Communication Example\n");
    for (int i = 0; message[i]; i++)
        circ_buffer_put(&circ_tx_buffer, message[i]);
    UCSR1B |= (1 << UDRIE1); // Enable TX interrupt

    sprintf(message, "Buffer sizes: RX=%d, TX=%d bytes\n", CIRC_BUFFER_SIZE, CIRC_BUFFER_SIZE);
    for (int i = 0; message[i]; i++)
        circ_buffer_put(&circ_tx_buffer, message[i]);

    sprintf(message, "Commands: 'status', 'clear', 'test', 'flood'\n\n");
    for (int i = 0; message[i]; i++)
        circ_buffer_put(&circ_tx_buffer, message[i]);

    char input_line[64];
    uint8_t input_index = 0;
    unsigned int total_received = 0;
    unsigned int total_sent = 0;

    while (1)
    {
        // Process received characters from circular buffer
        char ch;
        if (circ_buffer_get(&circ_rx_buffer, &ch))
        {
            total_received++;

            if (ch == '\r' || ch == '\n')
            {
                if (input_index > 0)
                {
                    input_line[input_index] = '\0';

                    // Process commands
                    if (strcmp(input_line, "status") == 0)
                    {
                        sprintf(message, "Buffer Status:\n");
                        for (int i = 0; message[i]; i++)
                            circ_buffer_put(&circ_tx_buffer, message[i]);

                        sprintf(message, "RX: %u/%u used, %s\n",
                                circ_rx_buffer.count, CIRC_BUFFER_SIZE,
                                circ_rx_buffer.overflow ? "OVERFLOW!" : "OK");
                        for (int i = 0; message[i]; i++)
                            circ_buffer_put(&circ_tx_buffer, message[i]);

                        sprintf(message, "TX: %u/%u used\n",
                                circ_tx_buffer.count, CIRC_BUFFER_SIZE);
                        for (int i = 0; message[i]; i++)
                            circ_buffer_put(&circ_tx_buffer, message[i]);

                        sprintf(message, "Totals: %u received, %u sent\n", total_received, total_sent);
                        for (int i = 0; message[i]; i++)
                            circ_buffer_put(&circ_tx_buffer, message[i]);
                    }
                    else if (strcmp(input_line, "clear") == 0)
                    {
                        circ_buffer_clear(&circ_rx_buffer);
                        circ_buffer_clear(&circ_tx_buffer);
                        total_received = total_sent = 0;
                        sprintf(message, "Buffers cleared!\n");
                        for (int i = 0; message[i]; i++)
                            circ_buffer_put(&circ_tx_buffer, message[i]);
                    }
                    else if (strcmp(input_line, "test") == 0)
                    {
                        sprintf(message, "Testing circular buffer efficiency...\n");
                        for (int i = 0; message[i]; i++)
                            circ_buffer_put(&circ_tx_buffer, message[i]);

                        for (int test = 0; test < 5; test++)
                        {
                            sprintf(message, "Test message #%d\n", test + 1);
                            for (int i = 0; message[i]; i++)
                                circ_buffer_put(&circ_tx_buffer, message[i]);
                        }
                    }
                    else if (strcmp(input_line, "flood") == 0)
                    {
                        sprintf(message, "Flooding buffer with test data...\n");
                        for (int i = 0; message[i]; i++)
                            circ_buffer_put(&circ_tx_buffer, message[i]);

                        for (int flood = 0; flood < 10; flood++)
                        {
                            sprintf(message, "Flood test line %d - demonstrating buffer capacity\n", flood);
                            for (int i = 0; message[i]; i++)
                                circ_buffer_put(&circ_tx_buffer, message[i]);
                        }
                    }
                    else
                    {
                        sprintf(message, "Echo: [%s]\n", input_line);
                        for (int i = 0; message[i]; i++)
                            circ_buffer_put(&circ_tx_buffer, message[i]);
                    }

                    input_index = 0;
                }
            }
            else if (ch == '\b' || ch == 127) // Backspace
            {
                if (input_index > 0)
                {
                    input_index--;
                    char backspace[] = "\b \b";
                    for (int i = 0; backspace[i]; i++)
                        circ_buffer_put(&circ_tx_buffer, backspace[i]);
                }
            }
            else if (input_index < sizeof(input_line) - 1)
            {
                input_line[input_index++] = ch;
                circ_buffer_put(&circ_tx_buffer, ch); // Echo
            }

            // Enable TX interrupt if there's data to send
            if (circ_tx_buffer.count > 0)
            {
                UCSR1B |= (1 << UDRIE1);
            }
        }

        // Background activity - LED indicates buffer activity
        static unsigned int led_timer = 0;
        if (++led_timer > 30000)
        {
            // LED pattern shows buffer usage
            if (circ_rx_buffer.count > CIRC_BUFFER_SIZE / 2 || circ_tx_buffer.count > CIRC_BUFFER_SIZE / 2)
                PORTC |= 0x10; // High usage - solid LED
            else if (circ_rx_buffer.count > 0 || circ_tx_buffer.count > 0)
                PORTC ^= 0x10; // Medium usage - blinking LED
            else
                PORTC &= ~0x10; // Low usage - LED off

            led_timer = 0;
        }

        // Update sent counter when TX buffer drains
        static uint16_t prev_tx_count = 0;
        if (circ_tx_buffer.count < prev_tx_count)
        {
            total_sent += (prev_tx_count - circ_tx_buffer.count);
        }
        prev_tx_count = circ_tx_buffer.count;
    }
}

// ADC Basic Reading Example
void main_adc_basic_reading(void)
{
    Port_init();
    Uart1_init();
    Adc_init();

    puts_USART1("ADC Basic Reading Example\n");
    puts_USART1("Reading from ADC channels 0-7\n");
    puts_USART1("Press any key to start continuous reading...\n");

    // Wait for user input to start
    getchar_USART1();

    puts_USART1("\nStarting ADC readings:\n");
    puts_USART1("Ch0    Ch1    Ch2    Ch3    Ch4    Ch5    Ch6    Ch7\n");
    puts_USART1("----   ----   ----   ----   ----   ----   ----   ----\n");

    unsigned int reading_count = 0;

    while (1)
    {
        char adc_display[128];

        // Read all 8 ADC channels
        unsigned int adc_values[8];
        for (int ch = 0; ch < 8; ch++)
        {
            adc_values[ch] = Read_Adc_Data(ch);
        }

        // Format and display readings
        sprintf(adc_display, "%04u   %04u   %04u   %04u   %04u   %04u   %04u   %04u\n",
                adc_values[0], adc_values[1], adc_values[2], adc_values[3],
                adc_values[4], adc_values[5], adc_values[6], adc_values[7]);
        puts_USART1(adc_display);

        // Every 10 readings, show some statistics
        if (++reading_count % 10 == 0)
        {
            unsigned int min_val = 1023, max_val = 0;
            unsigned int active_channels = 0;

            for (int ch = 0; ch < 8; ch++)
            {
                if (adc_values[ch] > 10)
                    active_channels++; // Count non-zero readings
                if (adc_values[ch] < min_val)
                    min_val = adc_values[ch];
                if (adc_values[ch] > max_val)
                    max_val = adc_values[ch];
            }

            sprintf(adc_display, "Stats: Active channels=%u, Range=%u-%u, Count=%u\n",
                    active_channels, min_val, max_val, reading_count);
            puts_USART1(adc_display);
        }

        // Visual feedback - LED brightness based on ADC0
        if (adc_values[0] > 512)
            PORTC |= 0x01; // LED bright
        else if (adc_values[0] > 256)
            PORTC ^= 0x01; // LED blinking
        else
            PORTC &= ~0x01; // LED off

        // Check for user input to pause/resume
        if (isReadyGetChar_USART1())
        {
            char user_input = getchar_USART1();
            if (user_input == ' ')
            {
                puts_USART1("\nPaused. Press SPACE to resume, 'q' to quit...\n");
                while (1)
                {
                    char cmd = getchar_USART1();
                    if (cmd == ' ')
                    {
                        puts_USART1("Resuming...\n");
                        break;
                    }
                    else if (cmd == 'q' || cmd == 'Q')
                    {
                        puts_USART1("ADC reading stopped.\n");
                        return;
                    }
                }
            }
        }

        _delay_ms(500); // Update every 500ms
    }
}

// ADC Voltage Conversion Example
void main_adc_voltage_conversion(void)
{
    Port_init();
    Uart1_init();
    Adc_init();

    puts_USART1("ADC Voltage Conversion Example\n");
    puts_USART1("Converting ADC readings to voltage values\n");
    puts_USART1("Reference: 5.0V, Resolution: 10-bit (0-1023)\n");
    puts_USART1("Commands: 's' = single reading, 'c' = continuous, 'q' = quit\n\n");

    const float REFERENCE_VOLTAGE = 5.0;
    const int ADC_RESOLUTION = 1024;

    char mode = 'c'; // Default to continuous mode
    unsigned int sample_count = 0;

    puts_USART1("Starting continuous voltage monitoring...\n");
    puts_USART1("Channel   Raw ADC   Voltage   % of Vref\n");
    puts_USART1("-------   -------   -------   ---------\n");

    while (1)
    {
        // Check for user commands
        if (isReadyGetChar_USART1())
        {
            char cmd = getchar_USART1();
            if (cmd == 'q' || cmd == 'Q')
            {
                puts_USART1("\nVoltage monitoring stopped.\n");
                return;
            }
            else if (cmd == 's' || cmd == 'S')
            {
                mode = 's';
                puts_USART1("\nSingle reading mode. Press 's' for reading, 'c' for continuous.\n");
            }
            else if (cmd == 'c' || cmd == 'C')
            {
                mode = 'c';
                puts_USART1("\nContinuous reading mode.\n");
            }
        }

        if (mode == 'c' || mode == 's')
        {
            char voltage_display[128];

            // Read and convert multiple channels
            for (int channel = 0; channel < 4; channel++) // Monitor channels 0-3
            {
                // Get raw ADC reading
                unsigned int raw_adc = Read_Adc_Data(channel);

                // Convert to voltage using library function
                unsigned int voltage_mv = Read_Adc_Voltage_mV(channel);
                float voltage_v = voltage_mv / 1000.0;

                // Calculate percentage of reference voltage
                float percentage = (raw_adc * 100.0) / ADC_RESOLUTION;

                // Format and display
                sprintf(voltage_display, "  CH%d     %4u      %5.3fV     %5.1f%%\n",
                        channel, raw_adc, voltage_v, percentage);
                puts_USART1(voltage_display);
            }

            sample_count++;

            // Show statistics every 20 samples
            if (sample_count % 20 == 0)
            {
                sprintf(voltage_display, "\nSample #%u completed. Commands: s=single, c=continuous, q=quit\n", sample_count);
                puts_USART1(voltage_display);
                puts_USART1("Channel   Raw ADC   Voltage   % of Vref\n");
                puts_USART1("-------   -------   -------   ---------\n");
            }

            // Visual feedback based on CH0 voltage
            unsigned int ch0_raw = Read_Adc_Data(0);
            if (ch0_raw > 750)      // > ~3.7V
                PORTC = 0x0F;       // All LEDs on
            else if (ch0_raw > 500) // > ~2.4V
                PORTC = 0x07;       // 3 LEDs on
            else if (ch0_raw > 250) // > ~1.2V
                PORTC = 0x03;       // 2 LEDs on
            else if (ch0_raw > 50)  // > ~0.2V
                PORTC = 0x01;       // 1 LED on
            else
                PORTC = 0x00; // All LEDs off

            if (mode == 's')
            {
                puts_USART1("\nSingle reading complete. Press 's' for another reading.\n");
                mode = 'w'; // Wait for next command
            }
            else
            {
                _delay_ms(1000); // Update every second in continuous mode
            }
        }
        else
        {
            _delay_ms(100); // Wait for command
        }

        // Background heartbeat
        static unsigned int heartbeat = 0;
        if (++heartbeat > 50000)
        {
            PORTC ^= 0x80; // Toggle heartbeat LED
            heartbeat = 0;
        }
    }
}

// ADC Multiple Channels Example
void main_adc_multiple_channels(void)
{
    Port_init();
    Uart1_init();
    Adc_init();

    puts_USART1("ADC Multiple Channels Sensor Array Example\n");
    puts_USART1("Monitoring 8 sensor channels with analysis\n");
    puts_USART1("Simulated sensors: Light, Temperature, Pressure, etc.\n\n");

    // Sensor configuration
    const char *sensor_names[8] = {
        "Light   ", "Temp    ", "Pressure", "Humidity",
        "Sound   ", "Motion  ", "Gas     ", "Voltage "};

    unsigned int sensor_minimums[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
    unsigned int sensor_maximums[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    unsigned long sensor_totals[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    unsigned int sample_count = 0;

    puts_USART1("Sensor Array Status:\n");
    puts_USART1("Light    Temp     Pressure Humidity Sound    Motion   Gas      Voltage\n");
    puts_USART1("-------- -------- -------- -------- -------- -------- -------- --------\n");

    while (1)
    {
        char display_line[128];
        unsigned int readings[8];
        unsigned int active_sensors = 0;

        // Read all 8 channels with averaging
        for (int ch = 0; ch < 8; ch++)
        {
            // Take multiple samples for noise reduction
            readings[ch] = Read_Adc_Averaged(ch, 4);

            // Update statistics
            if (readings[ch] < sensor_minimums[ch])
                sensor_minimums[ch] = readings[ch];
            if (readings[ch] > sensor_maximums[ch])
                sensor_maximums[ch] = readings[ch];
            sensor_totals[ch] += readings[ch];

            // Count active sensors (readings > threshold)
            if (readings[ch] > 20)
                active_sensors++;
        }

        sample_count++;

        // Display current readings
        sprintf(display_line, "%04u     %04u     %04u     %04u     %04u     %04u     %04u     %04u\n",
                readings[0], readings[1], readings[2], readings[3],
                readings[4], readings[5], readings[6], readings[7]);
        puts_USART1(display_line);

        // Sensor analysis every 10 samples
        if (sample_count % 10 == 0)
        {
            puts_USART1("\n--- Sensor Analysis ---\n");

            for (int ch = 0; ch < 8; ch++)
            {
                unsigned int average = sensor_totals[ch] / sample_count;
                unsigned int range = sensor_maximums[ch] - sensor_minimums[ch];

                sprintf(display_line, "%s: Avg=%4u, Range=%4u, Current=%4u ",
                        sensor_names[ch], average, range, readings[ch]);
                puts_USART1(display_line);

                // Status indicator
                if (readings[ch] > average + 100)
                    puts_USART1("HIGH\n");
                else if (readings[ch] < average - 100)
                    puts_USART1("LOW\n");
                else
                    puts_USART1("NORMAL\n");
            }

            sprintf(display_line, "\nActive sensors: %u/8, Total samples: %u\n\n", active_sensors, sample_count);
            puts_USART1(display_line);

            puts_USART1("Light    Temp     Pressure Humidity Sound    Motion   Gas      Voltage\n");
            puts_USART1("-------- -------- -------- -------- -------- -------- -------- --------\n");
        }

        // Visual feedback - LED pattern based on sensor activity
        PORTC = 0;
        if (readings[0] > 500)
            PORTC |= 0x01; // Light sensor LED
        if (readings[1] > 500)
            PORTC |= 0x02; // Temperature LED
        if (readings[2] > 500)
            PORTC |= 0x04; // Pressure LED
        if (readings[3] > 500)
            PORTC |= 0x08; // Humidity LED

        // Alert conditions
        if (readings[0] > 900) // Very bright light
        {
            puts_USART1("*** ALERT: High light level detected! ***\n");
        }
        if (readings[6] > 800) // Gas sensor
        {
            puts_USART1("*** ALERT: Gas detected! ***\n");
        }

        // Check for user input
        if (isReadyGetChar_USART1())
        {
            char cmd = getchar_USART1();
            if (cmd == 'r' || cmd == 'R')
            {
                // Reset statistics
                for (int i = 0; i < 8; i++)
                {
                    sensor_minimums[i] = 1023;
                    sensor_maximums[i] = 0;
                    sensor_totals[i] = 0;
                }
                sample_count = 0;
                puts_USART1("\n*** Statistics reset ***\n\n");
            }
            else if (cmd == 'q' || cmd == 'Q')
            {
                puts_USART1("\nMulti-channel monitoring stopped.\n");
                return;
            }
        }

        _delay_ms(800); // Update every 800ms
    }
}

// ADC Interrupt Example
void main_adc_interrupt(void)
{
    Port_init();
    Uart1_init();
    Adc_init();
    sei(); // Enable global interrupts

    puts_USART1("ADC Interrupt-Driven Sampling Example\n");
    puts_USART1("Non-blocking ADC conversions with interrupt processing\n");
    puts_USART1("Commands: 'f' = fast mode, 's' = slow mode, 'q' = quit\n\n");

    unsigned int conversion_count = 0;
    unsigned int total_conversions = 0;
    unsigned long sum_values = 0;
    unsigned int current_channel = 0;
    char sampling_mode = 'f'; // f=fast, s=slow

    // Enable ADC interrupt
    ADCSRA |= (1 << ADIE);

    puts_USART1("Starting interrupt-based sampling...\n");
    puts_USART1("Channel  Value   Voltage  Rate(Hz)  Total\n");
    puts_USART1("-------  -----   -------  --------  -----\n");

    // Start first conversion
    Start_Adc_Interrupt(current_channel);

    unsigned long last_time = 0;
    unsigned int last_count = 0;

    while (1)
    {
        // Check if ADC conversion is complete
        if (adc_interrupt_complete)
        {
            adc_interrupt_complete = 0;
            conversion_count++;
            total_conversions++;
            sum_values += adc_interrupt_result;

            // Calculate voltage
            float voltage = (adc_interrupt_result * 5.0) / 1024.0;

            // Calculate sampling rate (approximate)
            unsigned int rate_hz = 0;
            if (conversion_count >= 10)
            {
                rate_hz = (conversion_count * 1000) / (conversion_count * 50); // Rough estimate
                conversion_count = 0;
            }

            // Display result
            char result_line[80];
            sprintf(result_line, "  CH%d    %4u    %5.3fV    %3uHz    %5u\n",
                    current_channel, adc_interrupt_result, voltage, rate_hz, total_conversions);
            puts_USART1(result_line);

            // Visual feedback based on ADC value
            if (adc_interrupt_result > 768)      // > 75%
                PORTC = 0x0F;                    // All LEDs
            else if (adc_interrupt_result > 512) // > 50%
                PORTC = 0x07;                    // 3 LEDs
            else if (adc_interrupt_result > 256) // > 25%
                PORTC = 0x03;                    // 2 LEDs
            else if (adc_interrupt_result > 64)  // > 6%
                PORTC = 0x01;                    // 1 LED
            else
                PORTC = 0x00; // No LEDs

            // Cycle through channels 0-3
            current_channel = (current_channel + 1) % 4;

            // Start next conversion after delay based on mode
            if (sampling_mode == 'f')
                _delay_ms(50); // Fast mode - 20Hz per channel
            else
                _delay_ms(200); // Slow mode - 5Hz per channel

            Start_Adc_Interrupt(current_channel);
        }

        // Check for user commands
        if (isReadyGetChar_USART1())
        {
            char cmd = getchar_USART1();

            if (cmd == 'f' || cmd == 'F')
            {
                sampling_mode = 'f';
                puts_USART1("\n>>> Fast sampling mode (20Hz per channel) <<<\n");
            }
            else if (cmd == 's' || cmd == 'S')
            {
                sampling_mode = 's';
                puts_USART1("\n>>> Slow sampling mode (5Hz per channel) <<<\n");
            }
            else if (cmd == 'q' || cmd == 'Q')
            {
                // Disable ADC interrupt
                ADCSRA &= ~(1 << ADIE);
                puts_USART1("\nADC interrupt sampling stopped.\n");

                // Show final statistics
                char final_stats[80];
                unsigned int average = sum_values / total_conversions;
                sprintf(final_stats, "Final stats: %u conversions, average = %u\n",
                        total_conversions, average);
                puts_USART1(final_stats);
                return;
            }
            else if (cmd == 'r' || cmd == 'R')
            {
                // Reset statistics
                total_conversions = 0;
                sum_values = 0;
                puts_USART1("\n>>> Statistics reset <<<\n");
            }
        }

        // Background processing - show we're not blocking
        static unsigned int background_counter = 0;
        if (++background_counter > 10000)
        {
            background_counter = 0;
            // This demonstrates the CPU is free to do other work
        }
    }
}

// Graphics Basic Shapes Example
void main_graphics_basic_shapes(void)
{
    Port_init();
    Uart1_init();
    lcd_init();

    puts_USART1("Graphics Basic Shapes Example\n");
    puts_USART1("Drawing geometric primitives on 128x64 GLCD\n");
    puts_USART1("Commands: '1-7' = different shapes, 'c' = clear, 'q' = quit\n\n");

    lcd_clear();

    // Initial welcome display
    gotoxy_textLCD(0, 0);
    string_textLCD("Graphics Demo");
    gotoxy_textLCD(0, 1);
    string_textLCD("Press 1-7 for shapes");

    char demo_mode = '1';
    unsigned int animation_step = 0;

    while (1)
    {
        // Check for user input
        if (isReadyGetChar_USART1())
        {
            char cmd = getchar_USART1();

            if (cmd >= '1' && cmd <= '7')
            {
                demo_mode = cmd;
                lcd_clear();
                animation_step = 0;

                char mode_msg[40];
                sprintf(mode_msg, "Mode %c selected\n", cmd);
                puts_USART1(mode_msg);
            }
            else if (cmd == 'c' || cmd == 'C')
            {
                lcd_clear();
                puts_USART1("Screen cleared\n");
            }
            else if (cmd == 'q' || cmd == 'Q')
            {
                lcd_clear();
                puts_USART1("Graphics demo stopped.\n");
                return;
            }
        }

        // Execute current demo mode
        switch (demo_mode)
        {
        case '1': // Basic pixels and dots
            gotoxy_textLCD(0, 0);
            string_textLCD("1: Pixel Patterns");

            for (int i = 0; i < 20; i++)
            {
                GLCD_Dot(i * 6, 20 + (animation_step % 20));
                GLCD_Dot(127 - i * 6, 40 - (animation_step % 20));
            }
            break;

        case '2': // Lines
            gotoxy_textLCD(0, 0);
            string_textLCD("2: Line Drawing");

            // Diagonal lines
            GLCD_Line(0, 20, 127, 63);
            GLCD_Line(127, 20, 0, 63);

            // Animated line
            int line_x = (animation_step % 128);
            GLCD_Line(line_x, 20, 127 - line_x, 35);
            break;

        case '3': // Rectangles
            gotoxy_textLCD(0, 0);
            string_textLCD("3: Rectangles");

            // Static rectangles
            GLCD_Rectangle(10, 20, 50, 50);
            GLCD_Rectangle(70, 25, 120, 55);

            // Animated rectangle
            int rect_size = 10 + (animation_step % 20);
            GLCD_Rectangle(30, 30, 30 + rect_size, 30 + rect_size / 2);
            break;

        case '4': // Circles
            gotoxy_textLCD(0, 0);
            string_textLCD("4: Circles");

            // Static circles
            GLCD_Circle(30, 35, 15);
            GLCD_Circle(90, 35, 20);

            // Animated circle
            int radius = 5 + (animation_step % 15);
            GLCD_Circle(64, 40, radius);
            break;

        case '5': // Mixed shapes
            gotoxy_textLCD(0, 0);
            string_textLCD("5: Mixed Shapes");

            // House shape
            GLCD_Rectangle(40, 35, 80, 60); // Base
            GLCD_Line(40, 35, 60, 20);      // Roof left
            GLCD_Line(60, 20, 80, 35);      // Roof right
            GLCD_Rectangle(50, 45, 60, 60); // Door
            GLCD_Rectangle(65, 40, 75, 50); // Window
            break;

        case '6': // Pattern grid
            gotoxy_textLCD(0, 0);
            string_textLCD("6: Grid Patterns");

            // Grid pattern
            for (int x = 0; x < 128; x += 16)
            {
                GLCD_Line(x, 20, x, 63);
            }
            for (int y = 20; y < 64; y += 8)
            {
                GLCD_Line(0, y, 127, y);
            }

            // Animated dot in grid
            int grid_x = (animation_step % 8) * 16 + 8;
            int grid_y = ((animation_step / 8) % 5) * 8 + 24;
            GLCD_Circle(grid_x, grid_y, 3);
            break;

        case '7': // Full demo
            gotoxy_textLCD(0, 0);
            string_textLCD("7: Full Demo");

            // Multiple elements
            GLCD_Rectangle(5, 20, 35, 45);
            GLCD_Circle(50, 32, 12);
            GLCD_Line(70, 20, 120, 50);
            GLCD_Rectangle(75, 35, 125, 60);

            // Animated elements
            int demo_x = (animation_step % 100);
            GLCD_Dot(demo_x + 10, 55);
            GLCD_Dot(demo_x + 12, 55);
            break;
        }

        // Visual feedback LEDs
        animation_step++;
        PORTC = (animation_step >> 8) & 0x0F; // LED pattern based on animation

        // Status update every 50 steps
        if (animation_step % 50 == 0)
        {
            char status_msg[50];
            sprintf(status_msg, "Mode %c, Step %u\n", demo_mode, animation_step);
            puts_USART1(status_msg);
        }

        _delay_ms(100); // Animation speed
    }
}

// Graphics Animation Example
void main_graphics_animation(void)
{
    Port_init();
    Uart1_init();
    lcd_init();

    puts_USART1("Graphics Animation Example\n");
    puts_USART1("Advanced animations on 128x64 GLCD\n");
    puts_USART1("Commands: '1-5' = animations, 's' = speed, 'q' = quit\n\n");

    lcd_clear();

    // Animation variables
    int ball_x = 20, ball_y = 30;
    int ball_dx = 2, ball_dy = 1;
    int square_x = 0;
    int wave_phase = 0;
    unsigned int frame_count = 0;
    unsigned int delay_ms = 50; // Animation speed
    char current_animation = '1';

    // Initial display
    gotoxy_textLCD(0, 0);
    string_textLCD("Animation Demo");
    gotoxy_textLCD(0, 1);
    string_textLCD("Press 1-5 for modes");

    while (1)
    {
        // Check for user input
        if (isReadyGetChar_USART1())
        {
            char cmd = getchar_USART1();

            if (cmd >= '1' && cmd <= '5')
            {
                current_animation = cmd;
                lcd_clear();
                frame_count = 0;

                // Reset animation variables
                ball_x = 20;
                ball_y = 30;
                ball_dx = 2;
                ball_dy = 1;
                square_x = 0;
                wave_phase = 0;

                char msg[40];
                sprintf(msg, "Animation %c started\n", cmd);
                puts_USART1(msg);
            }
            else if (cmd == 's' || cmd == 'S')
            {
                delay_ms = (delay_ms == 50) ? 20 : (delay_ms == 20) ? 100
                                                                    : 50;
                char msg[40];
                sprintf(msg, "Speed: %ums delay\n", delay_ms);
                puts_USART1(msg);
            }
            else if (cmd == 'q' || cmd == 'Q')
            {
                lcd_clear();
                puts_USART1("Animation demo stopped.\n");
                return;
            }
        }

        // Clear screen for smooth animation
        lcd_clear();

        // Execute current animation
        switch (current_animation)
        {
        case '1': // Bouncing Ball
            gotoxy_textLCD(0, 0);
            string_textLCD("1: Bouncing Ball");

            // Update ball position
            ball_x += ball_dx;
            ball_y += ball_dy;

            // Bounce off walls
            if (ball_x <= 3 || ball_x >= 124)
                ball_dx = -ball_dx;
            if (ball_y <= 18 || ball_y >= 60)
                ball_dy = -ball_dy;

            // Draw ball
            GLCD_Circle(ball_x, ball_y, 3);

            // Draw borders
            GLCD_Rectangle(0, 18, 127, 63);
            break;

        case '2': // Moving Square
            gotoxy_textLCD(0, 0);
            string_textLCD("2: Moving Square");

            // Update square position
            square_x = (square_x + 2) % 128;

            // Draw moving square
            GLCD_Rectangle(square_x, 25, square_x + 15, 40);

            // Draw trail effect
            if (square_x > 20)
            {
                for (int i = 0; i < 3; i++)
                {
                    int trail_x = square_x - (i + 1) * 8;
                    if (trail_x >= 0)
                        GLCD_Rectangle(trail_x, 27 + i, trail_x + 10 - i * 2, 38 - i);
                }
            }
            break;

        case '3': // Sine Wave
            gotoxy_textLCD(0, 0);
            string_textLCD("3: Sine Wave");

            // Draw animated sine wave
            for (int x = 0; x < 128; x++)
            {
                // Calculate sine wave with moving phase
                int y = 40 + (int)(15 * sin((x + wave_phase) * 0.1));
                GLCD_Dot(x, y);

                // Add harmonic
                int y2 = 40 + (int)(8 * sin((x + wave_phase) * 0.2));
                GLCD_Dot(x, y2);
            }

            wave_phase = (wave_phase + 3) % 360;
            break;

        case '4': // Rotating Lines
            gotoxy_textLCD(0, 0);
            string_textLCD("4: Rotating Lines");

            // Draw rotating lines from center
            int center_x = 64, center_y = 40;
            for (int i = 0; i < 8; i++)
            {
                float angle = (frame_count * 0.1) + (i * 45 * 3.14159 / 180);
                int end_x = center_x + (int)(25 * cos(angle));
                int end_y = center_y + (int)(15 * sin(angle));

                GLCD_Line(center_x, center_y, end_x, end_y);
            }

            // Center point
            GLCD_Circle(center_x, center_y, 2);
            break;

        case '5': // Complex Animation
            gotoxy_textLCD(0, 0);
            string_textLCD("5: Complex Demo");

            // Multiple animated elements

            // Bouncing ball
            ball_x += ball_dx;
            ball_y += ball_dy;
            if (ball_x <= 3 || ball_x >= 60)
                ball_dx = -ball_dx;
            if (ball_y <= 18 || ball_y >= 45)
                ball_dy = -ball_dy;
            GLCD_Circle(ball_x, ball_y, 2);

            // Moving square
            int sq_x = 70 + (int)(15 * sin(frame_count * 0.1));
            GLCD_Rectangle(sq_x, 25, sq_x + 10, 35);

            // Rotating line
            float rot_angle = frame_count * 0.2;
            int rot_x = 100 + (int)(20 * cos(rot_angle));
            int rot_y = 50 + (int)(10 * sin(rot_angle));
            GLCD_Line(100, 50, rot_x, rot_y);

            // Boundaries
            GLCD_Rectangle(0, 18, 65, 48);
            GLCD_Rectangle(67, 18, 127, 63);
            break;
        }

        frame_count++;

        // Visual feedback LEDs - show animation activity
        PORTC = (frame_count >> 4) & 0x0F;

        // Status update every 100 frames
        if (frame_count % 100 == 0)
        {
            char status_msg[60];
            sprintf(status_msg, "Animation %c, Frame %u, Speed %ums\n",
                    current_animation, frame_count, delay_ms);
            puts_USART1(status_msg);
        }

        _delay_ms(delay_ms);
    }
}

// Graphics Sensor Display Example
void main_graphics_sensor_display(void)
{
    Port_init();
    Uart1_init();
    Adc_init();
    lcd_init();

    puts_USART1("Graphics Sensor Display Example\n");
    puts_USART1("Real-time sensor data visualization on GLCD\n");
    puts_USART1("Commands: 'm' = mode, 's' = speed, 'r' = reset, 'q' = quit\n\n");

    lcd_clear();

    // Display variables
    char display_mode = '1';            // 1=chart, 2=gauges, 3=oscilloscope, 4=dashboard
    unsigned int sensor_history[4][64]; // History for 4 sensors, 64 samples each
    unsigned int history_index = 0;
    unsigned int sample_count = 0;
    unsigned int update_speed = 200; // ms between updates

    // Initialize history arrays
    for (int ch = 0; ch < 4; ch++)
    {
        for (int i = 0; i < 64; i++)
        {
            sensor_history[ch][i] = 512; // Start at mid-scale
        }
    }

    gotoxy_textLCD(0, 0);
    string_textLCD("Sensor Display");
    gotoxy_textLCD(0, 1);
    string_textLCD("Press 'm' for modes");

    while (1)
    {
        // Read all sensor channels
        unsigned int sensors[4];
        for (int ch = 0; ch < 4; ch++)
        {
            sensors[ch] = Read_Adc_Data(ch);
        }

        // Update history buffers
        for (int ch = 0; ch < 4; ch++)
        {
            sensor_history[ch][history_index] = sensors[ch];
        }
        history_index = (history_index + 1) % 64;
        sample_count++;

        // Check for user commands
        if (isReadyGetChar_USART1())
        {
            char cmd = getchar_USART1();

            if (cmd == 'm' || cmd == 'M')
            {
                display_mode = (display_mode == '4') ? '1' : (display_mode + 1);
                lcd_clear();
                char msg[40];
                sprintf(msg, "Display mode %c\n", display_mode);
                puts_USART1(msg);
            }
            else if (cmd == 's' || cmd == 'S')
            {
                update_speed = (update_speed == 200) ? 100 : (update_speed == 100) ? 500
                                                                                   : 200;
                char msg[40];
                sprintf(msg, "Update speed: %ums\n", update_speed);
                puts_USART1(msg);
            }
            else if (cmd == 'r' || cmd == 'R')
            {
                // Reset history
                for (int ch = 0; ch < 4; ch++)
                {
                    for (int i = 0; i < 64; i++)
                    {
                        sensor_history[ch][i] = 512;
                    }
                }
                history_index = 0;
                sample_count = 0;
                puts_USART1("History reset\n");
            }
            else if (cmd == 'q' || cmd == 'Q')
            {
                lcd_clear();
                puts_USART1("Sensor display stopped.\n");
                return;
            }
        }

        // Clear display area (keep title)
        for (int y = 16; y < 64; y++)
        {
            for (int x = 0; x < 128; x++)
            {
                // Clear pixel by drawing off pixel
                // Note: This is a simplified clear - in practice use lcd_clear() or buffer operations
            }
        }

        // Display based on current mode
        switch (display_mode)
        {
        case '1': // Real-time Chart
            gotoxy_textLCD(0, 0);
            string_textLCD("Chart: 4 Sensors");

            // Draw chart axes
            GLCD_Line(10, 60, 120, 60); // X-axis
            GLCD_Line(10, 20, 10, 60);  // Y-axis

            // Draw sensor data as lines
            for (int ch = 0; ch < 4; ch++)
            {
                for (int x = 0; x < 63; x++)
                {
                    int idx1 = (history_index + x) % 64;
                    int idx2 = (history_index + x + 1) % 64;

                    int y1 = 60 - (sensor_history[ch][idx1] * 35 / 1024);
                    int y2 = 60 - (sensor_history[ch][idx2] * 35 / 1024);

                    GLCD_Line(11 + x, y1, 12 + x, y2);
                }
            }

            // Show current values
            char value_str[20];
            sprintf(value_str, "%03u %03u %03u %03u",
                    sensors[0], sensors[1], sensors[2], sensors[3]);
            gotoxy_textLCD(0, 7);
            string_textLCD(value_str);
            break;

        case '2': // Gauge Display
            gotoxy_textLCD(0, 0);
            string_textLCD("Gauges Mode");

            // Draw 4 circular gauges
            for (int ch = 0; ch < 4; ch++)
            {
                int gauge_x = 30 + (ch % 2) * 70;
                int gauge_y = 35 + (ch / 2) * 25;
                int radius = 12;

                // Gauge circle
                GLCD_Circle(gauge_x, gauge_y, radius);

                // Gauge needle
                float angle = (sensors[ch] * 180.0 / 1024.0) - 90; // -90 to +90 degrees
                int needle_x = gauge_x + (int)(radius * 0.8 * cos(angle * 3.14159 / 180));
                int needle_y = gauge_y + (int)(radius * 0.8 * sin(angle * 3.14159 / 180));
                GLCD_Line(gauge_x, gauge_y, needle_x, needle_y);

                // Center dot
                GLCD_Dot(gauge_x, gauge_y);
            }
            break;

        case '3': // Oscilloscope Mode
            gotoxy_textLCD(0, 0);
            string_textLCD("Oscilloscope");

            // Draw scope grid
            for (int x = 20; x < 120; x += 20)
            {
                for (int y = 25; y < 60; y += 2)
                {
                    GLCD_Dot(x, y);
                }
            }

            // Draw waveform for sensor 0
            for (int x = 0; x < 100; x++)
            {
                int idx = (history_index + x * 64 / 100) % 64;
                int y = 25 + (sensor_history[0][idx] * 30 / 1024);
                GLCD_Dot(20 + x, y);
            }

            // Trigger line
            GLCD_Line(20, 40, 120, 40);
            break;

        case '4': // Dashboard
            gotoxy_textLCD(0, 0);
            string_textLCD("Dashboard");

            // Bar graphs for each sensor
            for (int ch = 0; ch < 4; ch++)
            {
                int bar_x = 10 + ch * 28;
                int bar_height = sensors[ch] * 35 / 1024;

                // Bar outline
                GLCD_Rectangle(bar_x, 25, bar_x + 15, 60);

                // Fill bar
                for (int y = 60 - bar_height; y < 60; y++)
                {
                    GLCD_Line(bar_x + 1, y, bar_x + 14, y);
                }

                // Sensor number
                char sensor_label[2] = {'1' + ch, '\0'};
                gotoxy_textLCD(bar_x / 6, 7);
                string_textLCD(sensor_label);
            }

            // Statistics
            unsigned int avg = (sensors[0] + sensors[1] + sensors[2] + sensors[3]) / 4;
            unsigned int max_val = sensors[0];
            for (int i = 1; i < 4; i++)
            {
                if (sensors[i] > max_val)
                    max_val = sensors[i];
            }

            char stats[30];
            sprintf(stats, "Avg:%03u Max:%03u", avg, max_val);
            gotoxy_textLCD(13, 7);
            string_textLCD(stats);
            break;
        }

        // Visual feedback LEDs based on sensor activity
        PORTC = 0;
        if (sensors[0] > 600)
            PORTC |= 0x01;
        if (sensors[1] > 600)
            PORTC |= 0x02;
        if (sensors[2] > 600)
            PORTC |= 0x04;
        if (sensors[3] > 600)
            PORTC |= 0x08;

        // Status updates
        if (sample_count % 50 == 0)
        {
            char status_msg[80];
            sprintf(status_msg, "Mode %c, Samples: %u, S0-3: %u,%u,%u,%u\n",
                    display_mode, sample_count, sensors[0], sensors[1], sensors[2], sensors[3]);
            puts_USART1(status_msg);
        }

        _delay_ms(update_speed);
    }
}

// DC Motor PWM Control Example
void main_motors_dc_pwm(void)
{
    Port_init();
    Uart1_init();
    Timer2_init();

    puts_USART1("DC Motor PWM Control Example\n");
    puts_USART1("Variable speed control using PWM signals\n");
    puts_USART1("Commands: '+/-' = speed, 'd' = direction, 's' = stop, 'a' = auto, 'q' = quit\n\n");

    // PWM and motor control variables
    unsigned char pwm_duty = 0; // 0-255 PWM duty cycle
    char motor_direction = 1;   // 1 = forward, -1 = reverse
    char auto_mode = 0;         // 0 = manual, 1 = auto sweep
    unsigned int auto_step = 0;

    // Set up PWM pins (simulate on PORTC for demonstration)
    DDRC |= 0xFF; // Set PORTC as output for motor control

    // Motor control using Timer2 for PWM generation
    // Note: In real hardware, this would use hardware PWM on specific pins

    puts_USART1("Motor PWM Control Ready\n");
    puts_USART1("Speed: 0%, Direction: Forward, Mode: Manual\n");
    puts_USART1("Use +/- to adjust speed, 'd' to change direction\n\n");

    while (1)
    {
        // Check for user commands
        if (isReadyGetChar_USART1())
        {
            char cmd = getchar_USART1();

            switch (cmd)
            {
            case '+':
            case '=':
                if (pwm_duty < 250)
                    pwm_duty += 10;
                auto_mode = 0;
                break;

            case '-':
            case '_':
                if (pwm_duty > 10)
                    pwm_duty -= 10;
                auto_mode = 0;
                break;

            case 'd':
            case 'D':
                motor_direction = -motor_direction;
                puts_USART1("Direction changed\n");
                break;

            case 's':
            case 'S':
                pwm_duty = 0;
                auto_mode = 0;
                puts_USART1("Motor stopped\n");
                break;

            case 'a':
            case 'A':
                auto_mode = !auto_mode;
                auto_step = 0;
                if (auto_mode)
                    puts_USART1("Auto sweep mode enabled\n");
                else
                    puts_USART1("Manual control mode\n");
                break;

            case 'q':
            case 'Q':
                pwm_duty = 0;
                PORTC = 0;
                puts_USART1("Motor control stopped.\n");
                return;

            default:
                puts_USART1("Commands: +/- speed, d=direction, s=stop, a=auto, q=quit\n");
                break;
            }
        }

        // Auto mode - automatic speed sweep
        if (auto_mode)
        {
            auto_step++;
            if (auto_step < 100)
            {
                pwm_duty = auto_step * 2; // Ramp up
            }
            else if (auto_step < 200)
            {
                pwm_duty = 200 - (auto_step - 100) * 2; // Ramp down
            }
            else if (auto_step < 250)
            {
                pwm_duty = 0; // Pause
            }
            else
            {
                auto_step = 0;
                motor_direction = -motor_direction; // Change direction
            }
        }

        // Generate PWM signal using software PWM
        // In real hardware, this would use Timer1 or Timer3 hardware PWM
        static unsigned char pwm_counter = 0;
        pwm_counter++;

        // Calculate effective duty cycle based on direction
        unsigned char effective_duty = pwm_duty;
        if (motor_direction < 0)
        {
            // For reverse direction, we could use different pins or H-bridge control
            // Here we simulate with different LED pattern
        }

        // Software PWM generation (8-bit, 256 steps)
        if (pwm_counter < effective_duty)
        {
            // Motor ON - PWM high
            if (motor_direction > 0)
                PORTC = 0x0F; // Forward direction pattern
            else
                PORTC = 0xF0; // Reverse direction pattern
        }
        else
        {
            // Motor OFF - PWM low
            PORTC = 0x00;
        }

        // Visual feedback with LEDs showing PWM level
        static unsigned int led_update_counter = 0;
        if (++led_update_counter > 1000)
        {
            led_update_counter = 0;

            // LED bar graph showing speed
            unsigned char led_pattern = 0;
            unsigned char speed_bars = pwm_duty / 32; // 0-7 bars
            for (int i = 0; i < speed_bars; i++)
            {
                led_pattern |= (1 << i);
            }

            // Different pattern for reverse
            if (motor_direction < 0)
            {
                led_pattern = ~led_pattern;
            }

            // Update status display
            if (led_update_counter == 0) // Only update display occasionally
            {
                char status_msg[80];
                int speed_percent = (pwm_duty * 100) / 255;
                sprintf(status_msg, "Speed: %3d%%, Dir: %s, Mode: %s, PWM: %3u/255\n",
                        speed_percent,
                        (motor_direction > 0) ? "FWD" : "REV",
                        auto_mode ? "AUTO" : "MANUAL",
                        pwm_duty);
                puts_USART1(status_msg);
            }
        }

        // Motor control timing - PWM frequency approximately 1kHz
        _delay_us(50); // Fine timing for PWM

        // Status update every few seconds
        static unsigned int status_counter = 0;
        if (++status_counter > 40000)
        {
            status_counter = 0;

            char detailed_status[100];
            sprintf(detailed_status, "Motor Status - Speed: %d%%, Direction: %s, Auto: %s\n",
                    (pwm_duty * 100) / 255,
                    (motor_direction > 0) ? "Forward" : "Reverse",
                    auto_mode ? "ON" : "OFF");
            puts_USART1(detailed_status);
        }
    }
}

// Servo Motor Control Example
void main_motors_servo_basic(void)
{
    Port_init();
    Uart1_init();
    Timer2_init();

    puts_USART1("Servo Motor Control Example\n");
    puts_USART1("Precision positioning using PWM signals\n");
    puts_USART1("Commands: '0-9' = positions, 'a' = auto sweep, 's' = stop, 'q' = quit\n\n");

    // Servo control variables
    unsigned int servo_position = 90; // Servo angle (0-180 degrees)
    unsigned int pulse_width = 1500;  // Pulse width in microseconds (1000-2000µs)
    char auto_sweep = 0;              // Auto sweep mode
    unsigned int sweep_step = 0;
    char sweep_direction = 1;

    // Servo timing constants
    const unsigned int SERVO_MIN_PULSE = 1000;    // 1ms = 0 degrees
    const unsigned int SERVO_MAX_PULSE = 2000;    // 2ms = 180 degrees
    const unsigned int SERVO_CENTER_PULSE = 1500; // 1.5ms = 90 degrees
    const unsigned int SERVO_PERIOD = 20000;      // 20ms period

    puts_USART1("Servo Control Ready\n");
    puts_USART1("Position: 90°, Pulse: 1500µs\n");
    puts_USART1("Use 0-9 for preset positions, 'a' for auto sweep\n\n");

    // Set up servo output pin (simulate on PORTC)
    DDRC |= 0xFF; // Set PORTC as output

    unsigned long cycle_counter = 0;

    while (1)
    {
        // Check for user commands
        if (isReadyGetChar_USART1())
        {
            char cmd = getchar_USART1();

            switch (cmd)
            {
            case '0': // 0 degrees
                servo_position = 0;
                pulse_width = SERVO_MIN_PULSE;
                auto_sweep = 0;
                puts_USART1("Position: 0° (Full Left)\n");
                break;

            case '1': // 20 degrees
                servo_position = 20;
                pulse_width = SERVO_MIN_PULSE + (servo_position * 1000 / 180);
                auto_sweep = 0;
                break;

            case '2': // 40 degrees
                servo_position = 40;
                pulse_width = SERVO_MIN_PULSE + (servo_position * 1000 / 180);
                auto_sweep = 0;
                break;

            case '3': // 60 degrees
                servo_position = 60;
                pulse_width = SERVO_MIN_PULSE + (servo_position * 1000 / 180);
                auto_sweep = 0;
                break;

            case '4': // 80 degrees
                servo_position = 80;
                pulse_width = SERVO_MIN_PULSE + (servo_position * 1000 / 180);
                auto_sweep = 0;
                break;

            case '5': // 90 degrees (center)
                servo_position = 90;
                pulse_width = SERVO_CENTER_PULSE;
                auto_sweep = 0;
                puts_USART1("Position: 90° (Center)\n");
                break;

            case '6': // 100 degrees
                servo_position = 100;
                pulse_width = SERVO_MIN_PULSE + (servo_position * 1000 / 180);
                auto_sweep = 0;
                break;

            case '7': // 120 degrees
                servo_position = 120;
                pulse_width = SERVO_MIN_PULSE + (servo_position * 1000 / 180);
                auto_sweep = 0;
                break;

            case '8': // 140 degrees
                servo_position = 140;
                pulse_width = SERVO_MIN_PULSE + (servo_position * 1000 / 180);
                auto_sweep = 0;
                break;

            case '9': // 180 degrees
                servo_position = 180;
                pulse_width = SERVO_MAX_PULSE;
                auto_sweep = 0;
                puts_USART1("Position: 180° (Full Right)\n");
                break;

            case 'a':
            case 'A':
                auto_sweep = !auto_sweep;
                sweep_step = 0;
                if (auto_sweep)
                    puts_USART1("Auto sweep mode enabled\n");
                else
                    puts_USART1("Manual position mode\n");
                break;

            case 's':
            case 'S':
                auto_sweep = 0;
                servo_position = 90;
                pulse_width = SERVO_CENTER_PULSE;
                puts_USART1("Servo stopped at center position\n");
                break;

            case 'q':
            case 'Q':
                PORTC = 0;
                puts_USART1("Servo control stopped.\n");
                return;

            default:
                puts_USART1("Commands: 0-9=positions, a=auto, s=stop, q=quit\n");
                break;
            }
        }

        // Auto sweep mode
        if (auto_sweep)
        {
            sweep_step++;
            if (sweep_step > 100) // Change direction every 100 steps
            {
                sweep_step = 0;
                if (sweep_direction > 0)
                {
                    servo_position += 10;
                    if (servo_position >= 180)
                    {
                        servo_position = 180;
                        sweep_direction = -1;
                    }
                }
                else
                {
                    servo_position -= 10;
                    if (servo_position <= 0)
                    {
                        servo_position = 0;
                        sweep_direction = 1;
                    }
                }

                // Update pulse width
                pulse_width = SERVO_MIN_PULSE + (servo_position * 1000 / 180);
            }
        }

        // Generate servo PWM signal (20ms period, 1-2ms pulse)
        cycle_counter++;
        unsigned int cycle_time = cycle_counter % (SERVO_PERIOD / 10); // Scale for timing

        if (cycle_time < (pulse_width / 10))
        {
            // Servo pulse HIGH
            PORTC = 0xFF;
        }
        else
        {
            // Servo pulse LOW
            PORTC = 0x00;
        }

        // Visual feedback - LED pattern shows servo position
        static unsigned int led_update = 0;
        if (++led_update > 2000)
        {
            led_update = 0;

            // LED bar graph representing servo position
            unsigned char led_pattern = 0;
            unsigned char position_bars = servo_position / 23; // 0-7 bars for 0-180°
            for (int i = 0; i <= position_bars && i < 8; i++)
            {
                led_pattern |= (1 << i);
            }

            // Update position display
            char position_msg[80];
            sprintf(position_msg, "Servo: %3u°, Pulse: %4uµs, Mode: %s\n",
                    servo_position, pulse_width, auto_sweep ? "AUTO" : "MANUAL");
            puts_USART1(position_msg);
        }

        // Precise timing for servo control
        _delay_us(10); // 10µs resolution for PWM timing

        // Status update every few seconds
        static unsigned int status_timer = 0;
        if (++status_timer > 200000)
        {
            status_timer = 0;

            char status_msg[100];
            sprintf(status_msg, "Servo Status - Position: %u°, Pulse Width: %uµs, Auto: %s\n",
                    servo_position, pulse_width, auto_sweep ? "ON" : "OFF");
            puts_USART1(status_msg);

            // Show servo specifications
            if (status_timer == 0) // Only once per cycle
            {
                puts_USART1("Servo Specs: 0°=1000µs, 90°=1500µs, 180°=2000µs, Period=20ms\n");
            }
        }
    }
}

// Stepper Motor Control Example
void main_motors_stepper_basic(void)
{
    Port_init();
    Uart1_init();
    Timer2_init();

    puts_USART1("Stepper Motor Control Example\n");
    puts_USART1("Precise step-by-step motor control\n");
    puts_USART1("Commands: 'f/r' = forward/reverse, '+/-' = speed, 's' = steps, 'q' = quit\n\n");

    // Stepper motor variables
    unsigned int current_step = 0; // Current step position (0-3 for 4-step sequence)
    int motor_direction = 1;       // 1 = forward, -1 = reverse
    unsigned int step_delay = 100; // Delay between steps in ms
    unsigned int total_steps = 0;  // Total steps taken
    unsigned int target_steps = 0; // Target steps for programmed movement
    char auto_mode = 0;            // 0 = manual, 1 = auto stepping

    // 4-phase stepper motor sequence (full step)
    // Each row represents one step, each column represents a coil (A, B, C, D)
    const unsigned char step_sequence[4][4] = {
        {1, 0, 1, 0}, // Step 0: Coils A and C
        {0, 1, 1, 0}, // Step 1: Coils B and C
        {0, 1, 0, 1}, // Step 2: Coils B and D
        {1, 0, 0, 1}  // Step 3: Coils A and D
    };

    // Alternative half-step sequence for smoother operation
    const unsigned char half_step_sequence[8][4] = {
        {1, 0, 0, 0}, // Step 0: A only
        {1, 1, 0, 0}, // Step 1: A + B
        {0, 1, 0, 0}, // Step 2: B only
        {0, 1, 1, 0}, // Step 3: B + C
        {0, 0, 1, 0}, // Step 4: C only
        {0, 0, 1, 1}, // Step 5: C + D
        {0, 0, 0, 1}, // Step 6: D only
        {1, 0, 0, 1}  // Step 7: D + A
    };

    char step_mode = 'f'; // 'f' = full step, 'h' = half step
    unsigned int sequence_length = 4;

    puts_USART1("Stepper Motor Ready\n");
    puts_USART1("Mode: Full Step, Speed: 100ms/step, Direction: Forward\n");
    puts_USART1("Use 'f' for forward, 'r' for reverse, '+/-' for speed\n\n");

    // Set up motor control pins (simulate on PORTC)
    DDRC |= 0xFF; // Set PORTC as output

    while (1)
    {
        // Check for user commands
        if (isReadyGetChar_USART1())
        {
            char cmd = getchar_USART1();

            switch (cmd)
            {
            case 'f':
            case 'F':
                motor_direction = 1;
                auto_mode = 1;
                puts_USART1("Forward stepping enabled\n");
                break;

            case 'r':
            case 'R':
                motor_direction = -1;
                auto_mode = 1;
                puts_USART1("Reverse stepping enabled\n");
                break;

            case '+':
            case '=':
                if (step_delay > 20)
                    step_delay -= 10;
                char speed_msg[50];
                sprintf(speed_msg, "Speed increased: %ums/step\n", step_delay);
                puts_USART1(speed_msg);
                break;

            case '-':
            case '_':
                if (step_delay < 500)
                    step_delay += 10;
                sprintf(speed_msg, "Speed decreased: %ums/step\n", step_delay);
                puts_USART1(speed_msg);
                break;

            case 's':
            case 'S':
                auto_mode = 0;
                puts_USART1("Stepping stopped\n");
                PORTC = 0x00; // Turn off all coils
                break;

            case '1':
                target_steps = 50;
                auto_mode = 1;
                puts_USART1("Stepping 50 steps\n");
                break;

            case '2':
                target_steps = 100;
                auto_mode = 1;
                puts_USART1("Stepping 100 steps\n");
                break;

            case '3':
                target_steps = 200;
                auto_mode = 1;
                puts_USART1("Stepping 200 steps (full rotation)\n");
                break;

            case 'm':
            case 'M':
                step_mode = (step_mode == 'f') ? 'h' : 'f';
                sequence_length = (step_mode == 'f') ? 4 : 8;
                current_step = 0; // Reset position
                puts_USART1((step_mode == 'f') ? "Full step mode\n" : "Half step mode\n");
                break;

            case 'q':
            case 'Q':
                PORTC = 0x00; // Turn off all coils
                puts_USART1("Stepper motor control stopped.\n");
                return;

            default:
                puts_USART1("Commands: f/r=direction, +/-=speed, s=stop, 1-3=steps, m=mode, q=quit\n");
                break;
            }
        }

        // Execute stepping if in auto mode
        if (auto_mode && (target_steps == 0 || total_steps < target_steps))
        {
            // Update step position based on direction
            if (motor_direction > 0)
            {
                current_step = (current_step + 1) % sequence_length;
            }
            else
            {
                current_step = (current_step == 0) ? (sequence_length - 1) : (current_step - 1);
            }

            // Apply step pattern to motor coils
            unsigned char coil_pattern = 0;
            if (step_mode == 'f')
            {
                // Full step mode
                for (int coil = 0; coil < 4; coil++)
                {
                    if (step_sequence[current_step][coil])
                    {
                        coil_pattern |= (1 << coil);
                    }
                }
            }
            else
            {
                // Half step mode
                for (int coil = 0; coil < 4; coil++)
                {
                    if (half_step_sequence[current_step][coil])
                    {
                        coil_pattern |= (1 << coil);
                    }
                }
            }

            // Output to motor coils (simulated on PORTC)
            PORTC = coil_pattern;

            total_steps++;

            // Check if target reached
            if (target_steps > 0 && total_steps >= target_steps)
            {
                auto_mode = 0;
                target_steps = 0;
                PORTC = 0x00; // Turn off coils
                puts_USART1("Target steps reached. Motor stopped.\n");
            }

            _delay_ms(step_delay);
        }

        // Status display
        static unsigned int status_counter = 0;
        if (++status_counter > (1000 / (step_delay + 1)))
        {
            status_counter = 0;

            char status_msg[100];
            sprintf(status_msg, "Stepper: Step %u/%u, Total: %u, Mode: %s, Speed: %ums, Dir: %s\n",
                    current_step, sequence_length - 1, total_steps,
                    (step_mode == 'f') ? "FULL" : "HALF",
                    step_delay,
                    (motor_direction > 0) ? "FWD" : "REV");
            puts_USART1(status_msg);
        }

        // Visual feedback - show current step pattern
        static unsigned int led_update = 0;
        if (++led_update > 100)
        {
            led_update = 0;

            // LED pattern shows which coils are active
            unsigned char led_pattern = PORTC & 0x0F;

            // Add direction indicator on upper LEDs
            if (motor_direction > 0)
                led_pattern |= 0x10; // Forward indicator
            else
                led_pattern |= 0x20; // Reverse indicator

            if (auto_mode)
                led_pattern |= 0x80; // Auto mode indicator
        }

        // Background timing
        _delay_ms(10);
    }
}
#endif