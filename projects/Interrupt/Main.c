/*
 * ============================================================================
 * ATMEGA128 POLLING vs INTERRUPT - EDUCATIONAL TEACHING MODULE
 * ============================================================================
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * TOPIC: Polling vs Interrupt-Driven Programming
 * TARGET: ATmega128 @ 16MHz with GLCD display
 *
 * PURPOSE:
 * This module teaches the fundamental difference between polling and interrupt-
 * driven programming through hands-on demonstrations. Students learn when to
 * use each approach and how to implement ATmega128 external interrupts.
 *
 * ============================================================================
 * EDUCATIONAL OBJECTIVES
 * ============================================================================
 * 1. Understand polling: continuous checking of input states in a loop
 * 2. Understand interrupts: hardware-driven event response
 * 3. Compare CPU efficiency: polling (100% busy) vs interrupts (event-driven)
 * 4. Learn ATmega128 external interrupt system (INT0-INT7)
 * 5. Master ISR (Interrupt Service Routine) programming principles
 * 6. Practice volatile variable usage for ISR-main communication
 * 7. Configure interrupt edge detection (rising, falling, change, level)
 * 8. Understand interrupt priorities and vector table
 *
 * ============================================================================
 * HARDWARE CONFIGURATION
 * ============================================================================
 * PORTB (Output): 8 LEDs - Active LOW (0=ON, 1=OFF)
 *   - LED0 (PB0): Interrupt demo - toggles via INT0 ISR
 *   - LED3 (PB3): Polling demo - follows button state
 *
 * DOCUMENTATION REFERENCE:
 * ATmega128 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/2467S.pdf
 * - External Interrupts (pages 76-81)
 * - Interrupt vectors (pages 44-46)
 * - I/O Ports (pages 62-75)
 *
 * =============================================================================
 * EXTERNAL INTERRUPT REGISTERS - DETAILED REFERENCE FOR STUDENTS
 * =============================================================================
 *
 * REGISTER 1: EICRA (External Interrupt Control Register A) - INT0-INT3
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: ISC31  ISC30  ISC21  ISC20  ISC11  ISC10  ISC01  ISC00
 *
 * ISCn1:0: Interrupt Sense Control for INTn
 *          00 = Low level (triggers continuously while LOW)
 *          01 = Any logical change (both edges)
 *          10 = Falling edge (HIGH→LOW, typical for button press)
 *          11 = Rising edge (LOW→HIGH, typical for button release)
 *
 * Example: Configure INT0 falling edge (button press detection):
 *   EICRA = (1<<ISC01);  // ISC01:00 = 10 (falling edge)
 *
 * REGISTER 2: EICRB (External Interrupt Control Register B) - INT4-INT7
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: ISC71  ISC70  ISC61  ISC60  ISC51  ISC50  ISC41  ISC40
 *
 * Same ISCn1:0 modes as EICRA, but for INT4-INT7 (on PORTE)
 *
 * REGISTER 3: EIMSK (External Interrupt Mask Register) - ENABLE INTERRUPTS
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: INT7   INT6   INT5   INT4   INT3   INT2   INT1   INT0
 *
 * INTn (bit n): Interrupt Enable
 *               1 = Enable interrupt n
 *               0 = Disable interrupt n
 *
 * Example: Enable INT0 only:
 *   EIMSK = (1<<INT0);
 *
 * CRITICAL: Must also call sei() to enable global interrupts!
 *
 * REGISTER 4: EIFR (External Interrupt Flag Register) - STATUS FLAGS
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: INTF7  INTF6  INTF5  INTF4  INTF3  INTF2  INTF1  INTF0
 *
 * INTFn (bit n): Interrupt Flag for INTn
 *                Set by hardware when interrupt condition occurs
 *                Cleared automatically when ISR executes
 *                Can manually clear by writing 1: EIFR = (1<<INTFn);
 *
 * REGISTER 5: SREG (Status Register) - GLOBAL INTERRUPT ENABLE
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name:  I      T      H      S      V      N      Z      C
 *
 * I (bit 7): Global Interrupt Enable
 *            1 = Enable all interrupts (set with sei())
 *            0 = Disable all interrupts (set with cli())
 *
 * CRITICAL: Even if EIMSK enables INT0, interrupts won't fire unless I=1
 *
 * TYPICAL INTERRUPT INITIALIZATION:
 *
 *   void int0_init(void) {
 *       // Configure PD0 as input with pull-up
 *       DDRD &= ~(1<<PD0);   // Input
 *       PORTD |= (1<<PD0);   // Pull-up resistor
 *
 *       // Configure INT0 for falling edge
 *       EICRA = (1<<ISC01);  // Falling edge
 *
 *       // Enable INT0
 *       EIMSK = (1<<INT0);
 *
 *       // Enable global interrupts
 *       sei();
 *   }
 *
 *   // Define ISR
 *   ISR(INT0_vect) {
 *       // Interrupt code here
 *       PORTB ^= (1<<PB0);  // Toggle LED
 *   }
 *
 * INTERRUPT VECTOR NAMES (for ISR macro):
 *   INT0_vect, INT1_vect, INT2_vect, INT3_vect
 *   INT4_vect, INT5_vect, INT6_vect, INT7_vect
 *
 * ISR BEST PRACTICES:
 * 1. Keep ISR short and fast (< 50µs if possible)
 * 2. Use volatile for variables shared with main()
 * 3. Avoid floating-point math in ISRs
 * 4. Don't use delay functions in ISRs
 * 5. Set flags in ISR, process in main loop
 *
 * DEBOUNCING BUTTONS IN INTERRUPTS:
 *
 *   volatile uint8_t button_pressed = 0;
 *   volatile uint32_t last_press_time = 0;
 *
 *   ISR(INT0_vect) {
 *       uint32_t now = milliseconds;  // From timer
 *       if(now - last_press_time > 200) {  // 200ms debounce
 *           button_pressed = 1;
 *           last_press_time = now;
 *       }
 *   }
 *
 * EDGE DETECTION EXAMPLES:
 *
 *   Falling Edge (Button press with pull-up):
 *     EICRA = (1<<ISC01);  // ISC01:00 = 10
 *
 *   Rising Edge (Button release with pull-up):
 *     EICRA = (1<<ISC01) | (1<<ISC00);  // ISC01:00 = 11
 *
 *   Any Change (Detect both press and release):
 *     EICRA = (1<<ISC00);  // ISC01:00 = 01
 *
 *   Low Level (Continuous trigger while pressed):
 *     EICRA = 0x00;  // ISC01:00 = 00 (use with caution!)
 *
 * MULTIPLE INTERRUPTS:
 *
 *   void init_multiple_interrupts(void) {
 *       // INT0: Falling edge, INT1: Rising edge
 *       EICRA = (1<<ISC01) | (1<<ISC11) | (1<<ISC10);
 *
 *       // Enable both
 *       EIMSK = (1<<INT0) | (1<<INT1);
 *
 *       sei();
 *   }
 *
 *   ISR(INT0_vect) { /* Handle INT0 */
}
 *   ISR(INT1_vect) { /* Handle INT1 */ }
 *
 * INTERRUPT PRIORITIES:
 * - Lower vector number = Higher priority
 * - INT0 has highest priority (vector 2)
 * - If multiple interrupts occur simultaneously, lowest vector executes first
 * - Interrupts don't nest by default (I-bit cleared during ISR)
 *
 * =============================================================================
 *
 *   - LED7 (PB7): Timing indicator (optional)
 *
 * PORTD (Input): Buttons - Active LOW with pull-ups (0=pressed, 1=released)
 *   - PD0: INT0 interrupt button (falling edge trigger)
 *   - PD1: Polling button (checked in main loop)
 *   - PD2-PD7: Available for expansion
 *
 * GLCD (128×64): Real-time status display
 *   - Shows current mode (Polling or Interrupt)
 *   - Displays PORTB (LED states) in decimal
 *   - Displays PIND (button states) in decimal
 *   - Shows interrupt event messages
 *
 * ============================================================================
 * ATMEGA128 EXTERNAL INTERRUPT SYSTEM OVERVIEW
 * ============================================================================
 * The ATmega128 has 8 external interrupt pins (INT0-INT7) that can trigger
 * interrupts based on edge detection or level sensing.
 *
 * INTERRUPT VECTOR TABLE (Relevant entries):
 * Vector | Name    | Pin | Address | Priority | Description
 * -------|---------|-----|---------|----------|---------------------------
 *   1    | RESET   | -   | 0x0000  | Highest  | System reset
 *   2    | INT0    | PD0 | 0x0004  | 2        | External Interrupt 0
 *   3    | INT1    | PD1 | 0x0006  | 3        | External Interrupt 1
 *   4    | INT2    | PD2 | 0x0008  | 4        | External Interrupt 2
 *   5    | INT3    | PD3 | 0x000A  | 5        | External Interrupt 3
 *   6    | INT4    | PE4 | 0x000C  | 6        | External Interrupt 4
 *   7    | INT5    | PE5 | 0x000E  | 7        | External Interrupt 5
 *   8    | INT6    | PE6 | 0x0010  | 8        | External Interrupt 6
 *   9    | INT7    | PE7 | 0x0012  | 9        | External Interrupt 7
 *
 * CONTROL REGISTERS:
 * - EICRA (0x6A): Controls INT0-INT3 edge/level sensing
 * - EICRB (0x5A): Controls INT4-INT7 edge/level sensing
 * - EIMSK (0x59): Enable/disable individual interrupts
 * - EIFR (0x58): Interrupt flags (set by hardware, cleared by ISR)
 * - SREG: Status register with global interrupt enable bit (I-bit)
 *
 * ============================================================================
 * INTERRUPT SENSE CONTROL (ISC) - EDGE/LEVEL DETECTION
 * ============================================================================
 * Each interrupt (INTn) has 2 control bits: ISCn1 and ISCn0
 *
 * ISCn1 | ISCn0 | Mode              | When Interrupt Triggers
 * ------|-------|-------------------|----------------------------------------
 *   0   |   0   | Low Level         | Continuously while pin is LOW
 *   0   |   1   | Any Change        | On rising edge OR falling edge
 *   1   |   0   | Falling Edge      | When pin goes HIGH → LOW (button press)
 *   1   |   1   | Rising Edge       | When pin goes LOW → HIGH (button release)
 *
 * EXAMPLE: Configure INT0 for falling edge detection (button press)
 *   EICRA |= (1 << ISC01);   // Set bit 1 (ISC01 = 1)
 *EICRA &= ~(1 << ISC00);     // Clear bit 0 (ISC00 = 0)
 *   Result: ISC01=1, ISC00=0 → Falling edge mode
 *
 * ============================================================================
 * POLLING vs INTERRUPT - DETAILED COMPARISON
 * ============================================================================
 *
 * POLLING METHOD:
 * - CPU continuously checks input state in a loop (while(1))
 * - Simple to understand and implement
 * - Response time depends on loop execution time
 * - May miss very short events (faster than loop time)
 * - CPU is 100% busy (cannot sleep or do other tasks efficiently)
 * - Good for: Simple systems, predictable timing, learning basics
 *
 * INTERRUPT METHOD:
 * - Hardware detects event and triggers ISR automatically
 * - Immediate response (2-4 clock cycles latency)
 * - Guaranteed event detection (won't miss short pulses)
 * - CPU can sleep or perform other tasks between events
 * - Requires careful ISR design (keep short and fast)
 * - Good for: Responsive systems, multiple events, power efficiency
 *
 * COMPARISON TABLE:
 * Aspect             | POLLING                  | INTERRUPT
 * -------------------|--------------------------|---------------------------
 * Response Time      | Variable (loop-dependent)| Fixed (~2-4 cycles)
 * CPU Efficiency     | Low (100% busy)          | High (event-driven)
 * Power Consumption  | High (always active)     | Low (can sleep)
 * Code Complexity    | Simple                   | Moderate (ISR management)
 * Event Detection    | May miss short events    | Guaranteed detection
 * Debugging          | Easy (linear flow)       | Complex (asynchronous)
 * Multiple Events    | Sequential checks        | Priority-based handling
 *
 * ============================================================================
 * ISR (INTERRUPT SERVICE ROUTINE) PROGRAMMING RULES
 * ============================================================================
 * 1. KEEP ISRs SHORT AND FAST
 *    - Avoid delays (_delay_ms, _delay_us)
 *    - Avoid heavy calculations or loops
 *    - Avoid calling complex functions
 *    - Set flags for main loop processing instead
 *
 * 2. USE VOLATILE FOR SHARED VARIABLES
 *    - Any variable accessed by both ISR and main must be volatile
 *    - Prevents compiler optimization that could cause bugs
 *    - Example: volatile uint8_t flag;
 *
 * 3. MINIMIZE ISR EXECUTION TIME
 *    - Long ISRs block other interrupts (if same/lower priority)
 *    - Target: ISR should complete in < 10 microseconds
 *    - Do minimal work, defer processing to main loop
 *
 * 4. PROTECT CRITICAL SECTIONS
 *    - Use cli()/sei() to disable/enable interrupts temporarily
 *    - Needed when main code accesses multi-byte shared variables
 *    - Keep critical sections as short as possible
 *
 * 5. DON'T CALL BLOCKING FUNCTIONS
 *    - No printf, lcd_string (unless very fast), Serial prints
 *    - ISRs should be non-blocking
 *
 * ============================================================================
 * MODULE STRUCTURE - 5 PROGRESSIVE DEMOS
 * ============================================================================
 * Demo 1: Polling Basics       - Simple button polling with LED response
 * Demo 2: Polling Limitations   - Show how polling can miss fast events
 * Demo 3: Interrupt Basics      - INT0 interrupt with toggle LED
 * Demo 4: ISR Communication     - Using flags between ISR and main
 * Demo 5: Edge Detection Modes  - Different trigger modes (fall, rise, change)
 *
 * TEACHING PROGRESSION (3 weeks):
 * Week 1: Demos 1-2 (Polling concepts and limitations)
 * Week 2: Demo 3 (Introduction to interrupts)
 * Week 3: Demos 4-5 (Advanced ISR techniques and edge modes)
 *
 * ============================================================================
 * USAGE INSTRUCTIONS FOR INSTRUCTORS
 * ============================================================================
 * 1. Build project: Use "Build Current Project" task or cli-build-project.ps1
 * 2. Simulate: Use SimulIDE or program hardware (COM port configuration)
 * 3. Demo selection: Uncomment ONE demo_XX() call in main() function
 * 4. Student exercises: Modify delays, edge modes, add counters
 * 5. Assessment: Use provided labs in DOCUMENTATION_SUMMARY.md
 *
 * FILES IN THIS PROJECT:
 * - Main.c: This file - 5 teaching demos with comprehensive documentation
 * - config.h: Hardware initialization and utility functions
 * - INTERRUPT_QUICK_REFERENCE.md: Student reference (print for labs)
 * - INTERRUPT_FLOW_DIAGRAMS.md: Visual aids (polling vs interrupt flow)
 * - DOCUMENTATION_SUMMARY.md: Instructor teaching guide with lesson plans
 *
 * ============================================================================
 * MEMORY OPTIMIZATION: PROGMEM USAGE
 * ============================================================================
 * All constant strings are stored in PROGMEM (flash memory) to save SRAM.
 * ATmega128 has only 4KB SRAM - must be used carefully!
 *
 * Pattern used:
 *   const char STR_NAME[] PROGMEM = "Text";  // Define in flash
 *   lcd_string_P(row, col, STR_NAME);        // Read from flash with _P suffix
 *
 * ============================================================================
 */

#include "config.h"

// ============================================================================
// PROGMEM STRING CONSTANTS - Stored in Flash Memory
// ============================================================================
// These strings are stored in flash (128KB) instead of SRAM (4KB) to save memory
const char STR_TITLE[] PROGMEM = "Polling vs Interrupt";
const char STR_DIVIDER[] PROGMEM = "--------------------";
const char STR_MODE_POLLING[] PROGMEM = "Mode: POLLING";
const char STR_MODE_INTERRUPT[] PROGMEM = "Mode: INTERRUPT";
const char STR_MODE_COMPARE[] PROGMEM = "Mode: COMPARISON";
const char STR_MODE_ISR_FLAG[] PROGMEM = "Mode: ISR + FLAG";
const char STR_MODE_EDGE[] PROGMEM = "Mode: EDGE MODES";

const char STR_PD1_POLLING[] PROGMEM = "PD1: Poll (LED3)";
const char STR_PD0_INT0[] PROGMEM = "PD0: INT0 (LED0)";
const char STR_PRESS_PD1[] PROGMEM = "Press PD1 button";
const char STR_PRESS_PD0[] PROGMEM = "Press PD0 button";
const char STR_PRESS_BOTH[] PROGMEM = "Press PD0 or PD1";

const char STR_PORTB[] PROGMEM = "PORTB:";
const char STR_PIND[] PROGMEM = "PIND: ";
const char STR_COUNT[] PROGMEM = "Count:";
const char STR_EVENTS[] PROGMEM = "Events:";

const char STR_INT0_TRIG[] PROGMEM = "INT0: Triggered! ";
const char STR_BUTTON_PRESSED[] PROGMEM = "Button pressed!  ";
const char STR_FALLING_EDGE[] PROGMEM = "Falling edge";
const char STR_RISING_EDGE[] PROGMEM = "Rising edge ";
const char STR_ANY_CHANGE[] PROGMEM = "Any change  ";
const char STR_LOW_LEVEL[] PROGMEM = "Low level   ";

const char STR_POLLING_SLOW[] PROGMEM = "Slow polling...";
const char STR_POLLING_FAST[] PROGMEM = "Fast polling...";
const char STR_MISSED_EVENT[] PROGMEM = "May miss fast!";

// Helper function to print PROGMEM strings to GLCD
static void lcd_string_P(uint8_t row, uint8_t col, const char *str_P)
{
    char buffer[21]; // 20 chars + null terminator
    strncpy_P(buffer, str_P, 20);
    buffer[20] = '\0';
    lcd_string(row, col, buffer);
}

// ============================================================================
// GLOBAL VARIABLES - ISR Communication
// ============================================================================
// VOLATILE keyword is REQUIRED for variables shared between ISR and main
// Without volatile, compiler may optimize away reads, causing bugs!

volatile uint8_t int0_flag = 0;      // Flag: INT0 interrupt occurred
volatile uint16_t int0_count = 0;    // Counter: Number of INT0 interrupts
volatile uint8_t last_edge_type = 0; // 0=falling, 1=rising, 2=change

// ============================================================================
// INTERRUPT SERVICE ROUTINES (ISRs)
// ============================================================================

// INT0 ISR - Responds to button press on PD0
//
// PURPOSE: Demonstrate immediate hardware response to external event
// TRIGGER: Falling edge on PD0 (button press: HIGH → LOW)
// EXECUTION TIME: < 10 microseconds (keep ISRs short!)
//
// ISR PROGRAMMING PRINCIPLES (demonstrated here):
// 1. Keep short and fast - no delays, no complex logic
// 2. Set flags for main loop processing
// 3. Use volatile for shared variables
// 4. Minimize work done in ISR context
// 5. Provide immediate feedback (LED toggle is acceptable)
//
ISR(INT0_vect)
{
    // Set flag for main loop - this is the preferred communication method
    int0_flag = 1;

    // Increment counter - demonstrates counting events
    int0_count++;

    // Immediate visual feedback - LED0 toggles on every interrupt
    // This is acceptable because led_toggle() is a simple bit operation
    led_toggle(0);

    // WHAT NOT TO DO IN ISR:
    // - _delay_ms() or _delay_us() - blocks other interrupts
    // - Complex calculations - defer to main loop
    // - Multiple GLCD operations - too slow
    // - Serial/UART transmissions - may block
    // - Heavy processing - affects system responsiveness

    // NOTE: ISR automatically clears INTF0 flag upon completion
    // NOTE: Global interrupts (I-bit) are automatically disabled during ISR
    //       and re-enabled upon return (unless nested interrupts configured)
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// Initialize hardware and display header
static void setup_io_and_display(void)
{
    // Initialize ATmega128 peripherals
    init_devices(); // Calls Port_init() and lcd_init()

    // Ensure button pins are configured as inputs with pull-ups
    // (Port_init() already does this, but we make it explicit for teaching)
    DDRD &= ~((1 << PD0) | (1 << PD1)); // PD0, PD1 as inputs
    PORTD |= (1 << PD0) | (1 << PD1);   // Enable internal pull-up resistors

    // Clear display and show title
    lcd_clear();
    lcd_string_P(0, 0, STR_TITLE);

    // Turn off all LEDs initially (active low: set bits to 1)
    led_all_off();
}

// Configure INT0 interrupt for falling edge detection
static void setup_ext_interrupt(void)
{
    // ========================================================================
    // INTERRUPT CONFIGURATION - Detailed Step-by-Step
    // ========================================================================

    // STEP 1: Set Interrupt Sense Control bits for INT0
    // We want FALLING EDGE detection (button press: HIGH → LOW)
    // ISC01=1, ISC00=0 selects falling edge mode (see truth table in header)

    EICRA |= (1 << ISC01);  // Set ISC01 bit (bit 1 of EICRA)
    EICRA &= ~(1 << ISC00); // Clear ISC00 bit (bit 0 of EICRA)

    // Resulting EICRA bits [ISC01:ISC00] = [1:0] → Falling edge

    // STEP 2: Enable INT0 interrupt in External Interrupt Mask Register
    // This allows INT0 to trigger ISR when edge is detected

    EIMSK |= (1 << INT0); // Set INT0 bit in EIMSK register

    // STEP 3: Enable global interrupts
    // This is the master switch for ALL interrupts in the system
    // Individual interrupts (like INT0) must also be enabled in EIMSK

    sei(); // Set I-bit in SREG (Status Register)

    // ========================================================================
    // HARDWARE INTERRUPT MECHANISM (What happens when button pressed):
    // ========================================================================
    // 1. PD0 pin transitions from HIGH to LOW (falling edge)
    // 2. INT0 edge detector circuit detects the transition
    // 3. Hardware sets INTF0 flag in EIFR register
    // 4. If INT0 is enabled (EIMSK) AND global interrupts enabled (SREG.I):
    //    a) Current instruction completes
    //    b) CPU saves PC (Program Counter) on stack
    //    c) CPU saves SREG on stack
    //    d) CPU clears global interrupt enable (I-bit = 0)
    //    e) CPU jumps to INT0 vector address (0x0004)
    //    f) ISR(INT0_vect) function executes
    //    g) RETI instruction restores SREG and PC
    //    h) Execution resumes where it left off
    // 5. Total latency: 2-4 clock cycles (~125-250 nanoseconds @ 16MHz)
    // ========================================================================
}

// Configure INT0 for different edge detection modes (Demo 5)
static void setup_ext_interrupt_mode(uint8_t mode)
{
    // Disable INT0 first
    EIMSK &= ~(1 << INT0);

    // Configure edge detection mode
    switch (mode)
    {
    case 0: // Falling edge (button press)
        EICRA |= (1 << ISC01);
        EICRA &= ~(1 << ISC00);
        last_edge_type = 0;
        break;

    case 1: // Rising edge (button release)
        EICRA |= (1 << ISC01) | (1 << ISC00);
        last_edge_type = 1;
        break;

    case 2: // Any change (both edges)
        EICRA &= ~(1 << ISC01);
        EICRA |= (1 << ISC00);
        last_edge_type = 2;
        break;

    case 3: // Low level (continuous while held)
        EICRA &= ~((1 << ISC01) | (1 << ISC00));
        last_edge_type = 3;
        break;
    }

    // Enable INT0 and global interrupts
    EIMSK |= (1 << INT0);
    sei();
}

// ============================================================================
// DEMO 1: POLLING BASICS - Continuous Button Checking
// ============================================================================
//
// PURPOSE:
// Demonstrate the fundamental polling concept where the CPU continuously checks
// input state in a loop. Button on PD1 directly controls LED3.
//
// KEY CONCEPTS:
// - while(1) loop with continuous state checking
// - Direct relationship between input and output
// - CPU is 100% busy checking (even when nothing happens)
// - Response time depends on loop execution time
// - Simple to understand and implement
//
// TEACHING FOCUS:
// - Students see that CPU must actively check button state
// - _delay_ms() affects response time (experiment with different values)
// - Display shows real-time register values (PORTB, PIND)
// - May miss very short button presses (< loop time)
//
// STUDENT EXPERIMENTS:
// 1. Change _delay_ms(50) to _delay_ms(200) - notice slower response
// 2. Change _delay_ms(50) to _delay_ms(5) - notice faster response
// 3. Try very quick button taps - may be missed with long delays
//
static void demo_01_polling_basics(void)
{
    setup_io_and_display();

    // Display demo information
    lcd_string_P(1, 0, STR_MODE_POLLING);
    lcd_string_P(2, 0, STR_PD1_POLLING);
    lcd_string_P(3, 0, STR_PRESS_PD1);

    // Main polling loop - runs forever, checking button continuously
    while (1)
    {
        // READ INPUT: Get current state of all PORTD pins
        uint8_t pind_value = PIND;

        // POLLING LOGIC: Check if PD1 button is pressed
        // Active-low: button pressed = 0, button released = 1
        if (pind_value & (1 << PD1)) // Bit PD1 is HIGH (button NOT pressed)
        {
            led_off(3); // Turn LED3 OFF
        }
        else // Bit PD1 is LOW (button IS pressed)
        {
            led_on(3); // Turn LED3 ON
        }

        // REAL-TIME STATUS DISPLAY (for educational purposes)
        // Shows actual register values so students can correlate with hardware

        lcd_xy(5, 0);
        lcd_string_P(5, 0, STR_PORTB);
        lcd_xy(5, 7);
        GLCD_3DigitDecimal(PORTB); // LED states: 255=all off, 247=LED3 on

        lcd_xy(6, 0);
        lcd_string_P(6, 0, STR_PIND);
        lcd_xy(6, 7);
        GLCD_3DigitDecimal(pind_value); // Button states: 255=none, 253=PD1 pressed

        // POLLING DELAY: This affects how fast we check the button
        // Shorter delay = faster response, but more CPU usage
        // Longer delay = slower response, may miss short button presses
        _delay_ms(50); // 50ms = checking 20 times per second

        // TEACHING NOTE: With 50ms delay, the fastest button press we can
        // reliably detect is ~50ms. Faster presses may be missed!
    }
}

// ============================================================================
// DEMO 2: POLLING LIMITATIONS - Demonstrating Missed Events
// ============================================================================
//
// PURPOSE:
// Show how polling with delays can miss fast events. This demonstrates why
// interrupts are necessary for time-critical applications.
//
// KEY CONCEPTS:
// - Polling with long delays may miss short button presses
// - CPU cannot do other tasks while polling
// - Trade-off between responsiveness and CPU efficiency
// - Visual feedback shows when CPU is "busy" doing other work
//
// TEACHING FOCUS:
// - Simulate "busy" CPU with longer delays
// - Students try to press button during "busy" periods
// - Compare with interrupt method where events are never missed
// - Understand limitations of polling approach
//
// STUDENT EXPERIMENTS:
// 1. Try pressing PD1 very quickly during "Slow polling..." phase
// 2. Notice some presses are missed (LED doesn't respond)
// 3. Compare with interrupt demo where all presses are detected
//
static void demo_02_polling_limitations(void)
{
    setup_io_and_display();

    lcd_string_P(1, 0, STR_MODE_POLLING);
    lcd_string_P(2, 0, STR_PD1_POLLING);
    lcd_string_P(3, 0, STR_PRESS_PD1);

    uint8_t phase = 0; // 0=fast polling, 1=slow polling

    while (1)
    {
        uint8_t pind_value = PIND;

        // Same polling logic as Demo 1
        if (pind_value & (1 << PD1))
        {
            led_off(3);
        }
        else
        {
            led_on(3);
        }

        // Alternate between fast and slow polling to demonstrate difference
        if (phase == 0)
        {
            // FAST POLLING: Quick response
            lcd_string_P(4, 0, STR_POLLING_FAST);
            _delay_ms(20); // 20ms delay - good response

            // After 100 checks (2 seconds), switch to slow
            static uint8_t fast_count = 0;
            if (++fast_count > 100)
            {
                fast_count = 0;
                phase = 1;
            }
        }
        else
        {
            // SLOW POLLING: Simulates "busy" CPU doing other work
            lcd_string_P(4, 0, STR_POLLING_SLOW);
            lcd_string_P(5, 0, STR_MISSED_EVENT);
            _delay_ms(300); // 300ms delay - slow response, may miss presses!

            // After 10 checks (3 seconds), switch back to fast
            static uint8_t slow_count = 0;
            if (++slow_count > 10)
            {
                slow_count = 0;
                phase = 0;
                lcd_string(5, 0, "                    "); // Clear message
            }
        }

        // Display register values
        lcd_xy(6, 0);
        lcd_string_P(6, 0, STR_PORTB);
        lcd_xy(6, 7);
        GLCD_3DigitDecimal(PORTB);

        lcd_xy(7, 0);
        lcd_string_P(7, 0, STR_PIND);
        lcd_xy(7, 7);
        GLCD_3DigitDecimal(pind_value);

        // TEACHING POINT: During slow phase, quick button presses are missed!
        // This shows why interrupts are needed for critical event detection
    }
}

// ============================================================================
// DEMO 3: INTERRUPT BASICS - Hardware-Driven Response
// ============================================================================
//
// PURPOSE:
// Introduce interrupt-driven programming. Button on PD0 triggers INT0 interrupt
// which toggles LED0. Response is immediate regardless of main loop timing.
//
// KEY CONCEPTS:
// - Hardware detects button press automatically (no polling needed)
// - ISR executes immediately when interrupt occurs (2-4 cycles latency)
// - Main loop can be busy doing other work - doesn't affect response
// - CPU can sleep between events (power efficient)
// - Asynchronous event handling
//
// TEACHING FOCUS:
// - Compare with Demo 1 - notice immediate response even with long delays
// - ISR is called automatically by hardware
// - LED0 toggles in ISR, main loop just updates display
// - Button presses are never missed (hardware queues the event)
//
// STUDENT EXPERIMENTS:
// 1. Press PD0 rapidly - all presses are detected
// 2. Compare with polling demo - interrupts are more responsive
// 3. Watch LED0 toggle immediately regardless of _delay_ms in main
//
static void demo_03_interrupt_basics(void)
{
    setup_io_and_display();
    setup_ext_interrupt(); // Configure INT0 for falling edge

    lcd_string_P(1, 0, STR_MODE_INTERRUPT);
    lcd_string_P(2, 0, STR_PD0_INT0);
    lcd_string_P(3, 0, STR_PRESS_PD0);

    // Main loop - NOT checking button! Hardware handles that via INT0
    while (1)
    {
        uint8_t pind_value = PIND;

        // Display register values (for comparison with polling)
        lcd_xy(5, 0);
        lcd_string_P(5, 0, STR_PORTB);
        lcd_xy(5, 7);
        GLCD_3DigitDecimal(PORTB); // LED0 toggles via ISR

        lcd_xy(6, 0);
        lcd_string_P(6, 0, STR_PIND);
        lcd_xy(6, 7);
        GLCD_3DigitDecimal(pind_value);

        // IMPORTANT: Even with this 100ms delay, button response is instant!
        // That's because ISR handles the button, not this main loop
        _delay_ms(100);

        // TEACHING POINT: Main loop can do ANY work here (even long delays)
        // and button will still respond instantly via interrupt mechanism
    }
}

// ============================================================================
// DEMO 4: ISR COMMUNICATION - Using Flags Between ISR and Main
// ============================================================================
//
// PURPOSE:
// Demonstrate proper ISR-to-main communication using flags and counters.
// Shows how to defer processing from ISR to main loop context.
//
// KEY CONCEPTS:
// - ISR sets flags that main loop checks
// - Volatile keyword ensures compiler doesn't optimize away flag checks
// - ISR keeps minimal work (just set flag and toggle LED)
// - Main loop does complex processing (display updates)
// - Counter tracks total number of interrupts
//
// TEACHING FOCUS:
// - Why keep ISRs short: other interrupts may be blocked
// - Volatile keyword is REQUIRED for ISR-shared variables
// - Flag-based communication is standard pattern
// - Count shows every button press is detected
//
// STUDENT EXPERIMENTS:
// 1. Press PD0 multiple times - watch counter increment
// 2. Remove 'volatile' keyword and observe problems (compiler optimization)
// 3. Add more processing in ISR - notice slower response
//
static void demo_04_isr_communication(void)
{
    setup_io_and_display();
    setup_ext_interrupt();

    // Reset counters
    int0_count = 0;
    int0_flag = 0;

    lcd_string_P(1, 0, STR_MODE_ISR_FLAG);
    lcd_string_P(2, 0, STR_PD0_INT0);
    lcd_string_P(3, 0, STR_PRESS_PD0);

    while (1)
    {
        // CHECK ISR FLAG: This is how ISR communicates with main
        if (int0_flag)
        {
            // Acknowledge flag (clear it)
            int0_flag = 0;

            // COMPLEX PROCESSING in main loop context (not in ISR)
            // This is safe to do here - doesn't block other interrupts
            lcd_string_P(4, 0, STR_INT0_TRIG);
            _delay_ms(200);                           // Show message briefly
            lcd_string(4, 0, "                    "); // Clear message
        }

        // Display interrupt counter
        lcd_xy(5, 0);
        lcd_string_P(5, 0, STR_EVENTS);
        lcd_xy(5, 8);
        GLCD_4DigitDecimal(int0_count); // Shows total interrupt count

        // Display register values
        lcd_xy(6, 0);
        lcd_string_P(6, 0, STR_PORTB);
        lcd_xy(6, 7);
        GLCD_3DigitDecimal(PORTB);

        uint8_t pind_value = PIND;
        lcd_xy(7, 0);
        lcd_string_P(7, 0, STR_PIND);
        lcd_xy(7, 7);
        GLCD_3DigitDecimal(pind_value);

        _delay_ms(50); // Moderate refresh rate

        // TEACHING POINT: Flag-based communication allows ISR to be fast
        // while main loop does slow work (display updates, calculations, etc.)
    }
}

// ============================================================================
// DEMO 5: EDGE DETECTION MODES - Different Interrupt Triggers
// ============================================================================
//
// PURPOSE:
// Show different interrupt trigger modes: falling edge, rising edge, any change,
// and low level. Students see how different ISC settings affect behavior.
//
// KEY CONCEPTS:
// - Falling edge: triggers on button press (HIGH → LOW)
// - Rising edge: triggers on button release (LOW → HIGH)
// - Any change: triggers on both press and release
// - Low level: continuously triggers while button held (use carefully!)
//
// TEACHING FOCUS:
// - ISC01/ISC00 bit combinations create different modes
// - Falling edge is most common for buttons (detects press)
// - Rising edge detects button release
// - "Any change" doubles interrupt rate (press + release)
// - Low level can cause interrupt flood (triggers continuously)
//
// STUDENT EXPERIMENTS:
// 1. Try each mode (0-3) by changing call in main()
// 2. Notice when LED toggles (press, release, or both)
// 3. With low level mode, watch rapid interrupt firing while held
//
static void demo_05_edge_detection_modes(uint8_t mode)
{
    setup_io_and_display();
    setup_ext_interrupt_mode(mode);

    // Reset counter
    int0_count = 0;

    lcd_string_P(1, 0, STR_MODE_EDGE);
    lcd_string_P(2, 0, STR_PD0_INT0);
    lcd_string_P(3, 0, STR_PRESS_PD0);

    while (1)
    {
        // Display current mode
        lcd_xy(4, 0);
        lcd_string(4, 0, "Mode: ");
        lcd_xy(4, 6);
        switch (mode)
        {
        case 0:
            lcd_string_P(4, 6, STR_FALLING_EDGE);
            break;
        case 1:
            lcd_string_P(4, 6, STR_RISING_EDGE);
            break;
        case 2:
            lcd_string_P(4, 6, STR_ANY_CHANGE);
            break;
        case 3:
            lcd_string_P(4, 6, STR_LOW_LEVEL);
            break;
        }

        // Display interrupt counter
        lcd_xy(5, 0);
        lcd_string_P(5, 0, STR_EVENTS);
        lcd_xy(5, 8);
        GLCD_4DigitDecimal(int0_count);

        // Display register values
        lcd_xy(6, 0);
        lcd_string_P(6, 0, STR_PORTB);
        lcd_xy(6, 7);
        GLCD_3DigitDecimal(PORTB);

        uint8_t pind_value = PIND;
        lcd_xy(7, 0);
        lcd_string_P(7, 0, STR_PIND);
        lcd_xy(7, 7);
        GLCD_3DigitDecimal(pind_value);

        _delay_ms(50);

        // TEACHING POINTS:
        // - Mode 0 (falling): Interrupt on button press only
        // - Mode 1 (rising): Interrupt on button release only
        // - Mode 2 (any change): Interrupt on press AND release (2× count)
        // - Mode 3 (low level): Interrupts continuously while held (rapid count!)
    }
}

// ============================================================================
// MAIN FUNCTION - Demo Selection
// ============================================================================
int main(void)
{
    // ========================================================================
    // INSTRUCTOR GUIDE: Demo Selection for Teaching
    // ========================================================================
    // This module contains 5 progressive demos teaching polling vs interrupts
    // Uncomment ONE demo function call below based on your lesson plan
    //
    // WEEK 1: POLLING CONCEPTS
    //   Day 1: demo_01_polling_basics() - Introduction to polling
    //   Day 2: demo_02_polling_limitations() - Why polling has limits
    //
    // WEEK 2: INTERRUPT INTRODUCTION
    //   Day 1: demo_03_interrupt_basics() - First interrupt program
    //   Day 2: demo_04_isr_communication() - ISR flags and communication
    //
    // WEEK 3: ADVANCED INTERRUPT TECHNIQUES
    //   Day 1: demo_05_edge_detection_modes(0) - Falling edge (press)
    //   Day 2: demo_05_edge_detection_modes(1) - Rising edge (release)
    //   Day 3: demo_05_edge_detection_modes(2) - Any change (both)
    //   Day 4: demo_05_edge_detection_modes(3) - Low level (continuous)
    // ========================================================================

    // === WEEK 1: POLLING ===
    // demo_01_polling_basics();          // Simple polling with PD1→LED3
    // demo_02_polling_limitations();     // Show missed events with slow polling

    // === WEEK 2: INTERRUPTS ===
    // demo_03_interrupt_basics();        // INT0 interrupt with PD0→LED0
    demo_04_isr_communication(); // ISR flags, counters, main processing

    // === WEEK 3: EDGE MODES ===
    // demo_05_edge_detection_modes(0);   // Falling edge (button press)
    // demo_05_edge_detection_modes(1);   // Rising edge (button release)
    // demo_05_edge_detection_modes(2);   // Any change (press + release)
    // demo_05_edge_detection_modes(3);   // Low level (continuous while held)

    // ========================================================================
    // STUDENT EXERCISES
    // ========================================================================
    /*
     * EXERCISE 1: Polling Response Time
     * - Run demo_01_polling_basics()
     * - Change _delay_ms(50) to different values (10, 100, 500)
     * - Try rapid button presses - do you miss any?
     * - Calculate: With 50ms delay, what's the fastest pulse you can detect?
     *
     * EXERCISE 2: Interrupt Comparison
     * - Run demo_03_interrupt_basics()
     * - Try same rapid button presses
     * - Notice ALL presses are detected
     * - Why? ISR responds in ~250 nanoseconds!
     *
     * EXERCISE 3: Event Counting
     * - Run demo_04_isr_communication()
     * - Press button 10 times
     * - Does counter show exactly 10? (should be accurate)
     * - Now try demo_02_polling_limitations() - same 10 presses
     * - Does it count all 10? (probably misses some)
     *
     * EXERCISE 4: Edge Detection
     * - Run demo_05 with mode 0 (falling edge)
     * - Count interrupts for 5 button presses
     * - Now run with mode 1 (rising edge)
     * - Count interrupts for 5 button presses
     * - Now run with mode 2 (any change)
     * - Count interrupts for 5 button presses - should be 2× previous!
     *
     * EXERCISE 5: Register Analysis
     * - Watch PORTB and PIND values on display
     * - Calculate binary representation:
     *   PORTB=254 (decimal) = 11111110 (binary) = LED0 ON
     *   PIND=254 (decimal) = 11111110 (binary) = PD0 pressed
     * - Practice: If PORTB shows 247, which LED is on? (LED3)
     *
     * EXERCISE 6: Multiple Interrupts (Advanced)
     * - Modify setup_ext_interrupt() to enable INT1 also
     * - Create ISR(INT1_vect) that toggles LED1
     * - Now PD0 toggles LED0, PD1 toggles LED1
     * - Test priority: If both pressed simultaneously, which ISR runs first?
     *   Answer: INT0 (higher priority, lower vector number)
     *
     * EXERCISE 7: Volatile Demonstration (Advanced)
     * - Remove 'volatile' from int0_flag declaration
     * - Compile with optimization (-Os or -O2)
     * - Observe: Flag checking may not work! Compiler optimizes it away
     * - Restore 'volatile' - now it works correctly
     * - Lesson: Always use volatile for ISR-shared variables
     */

    // ========================================================================
    // ASSESSMENT IDEAS FOR INSTRUCTORS
    // ========================================================================
    /*
     * QUIZ QUESTIONS:
     * 1. What is the main difference between polling and interrupts?
     * 2. Which control register enables INT0? (EIMSK)
     * 3. What does ISC01=1, ISC00=0 configure? (falling edge)
     * 4. Why must ISR-shared variables be volatile?
     * 5. What is the INT0 vector address? (0x0004)
     *
     * LAB PRACTICAL:
     * - Write code to detect rising edge on INT2 (PD2)
     * - ISR should increment a counter
     * - Display counter on GLCD
     * - Time limit: 15 minutes
     *
     * PROJECT IDEAS:
     * - Button debouncing with interrupts
     * - Multi-button menu system
     * - Interrupt-driven event logger
     * - Reaction time game (measure button press latency)
     */

    return 0;
}