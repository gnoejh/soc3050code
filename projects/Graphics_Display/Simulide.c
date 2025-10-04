/*
 * =============================================================================
 * GRAPHICS DISPLAY - SIMULIDE OPTIMIZED VERSION
 * =============================================================================
 * ATmega128 Educational Framework - SimulIDE Compatible
 *
 * OBJECTIVE: Demonstrate GLCD functionality optimized for SimulIDE simulation
 * FOCUS: Works with official Simulator.simu circuit configuration
 *
 * IMPORTANT: This version is designed for the official Simulator.simu circuit
 * which has a DIFFERENT pin mapping than the standard GLCD library!
 *
 * OFFICIAL SIMULIDE CIRCUIT PIN MAPPING:
 * Data Bus: PORTA0-2, PORTG0-1, PORTC0-2 (mixed ports!)
 * Control: PORTE4(RS), PORTE5(EN), PORTA4(RW), PORTE6(CS2), PORTE7(CS1)
 *
 * SIMULIDE CONSIDERATIONS:
 * - GLCD simulation is much slower than real hardware
 * - Uses mixed port data bus configuration
 * - Requires careful timing for simulation environment
 *
 * HARDWARE SIMULATION:
 * - ATmega128 microcontroller
 * - 128x64 Graphical LCD (KS0108 compatible)
 * - Official Simulator.simu circuit file
 *
 * =============================================================================
 */

#include "config.h"

// Simple display messages for SimulIDE testing
char welcome_msg[] = "SOC3050 Graphics";
char course_msg[] = "Embedded Systems";
char demo_msg[] = "SimulIDE Demo";
char status_msg[] = "GLCD Working!";
char circuit_msg[] = "Official Circuit";

// Function prototypes for SimulIDE optimized functions
void simulide_init(void);
void simulide_display_header(void);
void simulide_display_course_info(void);
void simulide_display_simple_graphics(void);
void simulide_display_counter(void);
void simulide_clear_and_wait(void);

/*
 * =============================================================================
 * SIMULIDE INITIALIZATION
 * =============================================================================
 */

void simulide_init(void)
{
    // Initialize basic systems first
    init_devices();
    puts_USART1("\\r\\n*** SimulIDE GLCD Debug Version ***\\r\\n");
    puts_USART1("Step 1: Basic initialization complete\\r\\n");

    // Give SimulIDE time to stabilize
    _delay_ms(1000);

    // Debug: Check if we can communicate via serial first
    puts_USART1("Step 2: Testing serial communication - if you see this, UART works\\r\\n");

    // Try to initialize GLCD with debug output
    puts_USART1("Step 3: Attempting GLCD initialization...\\r\\n");

    // Manual GLCD initialization for SimulIDE
    // Set up PORTA as output for data
    DDRA = 0xFF;  // PORTA as output
    PORTA = 0x00; // Clear data lines

    // Set up PORTE control pins as outputs
    DDRE |= (1 << DDE4) | (1 << DDE5) | (1 << DDE6) | (1 << DDE7);             // PE4-PE7 as outputs
    PORTE &= ~((1 << PORTE4) | (1 << PORTE5) | (1 << PORTE6) | (1 << PORTE7)); // Clear all control lines

    puts_USART1("Step 4: Port configuration complete\\r\\n");
    _delay_ms(500);

    // Try the library initialization
    init_GLCD();
    puts_USART1("Step 5: init_GLCD() called\\r\\n");
    _delay_ms(1000);

    // Try to clear display
    puts_USART1("Step 6: Attempting to clear display...\\r\\n");
    lcd_clear();
    puts_USART1("Step 7: lcd_clear() called\\r\\n");
    _delay_ms(1000);

    puts_USART1("Step 8: GLCD initialization sequence complete\\r\\n");
    puts_USART1("If GLCD shows nothing, check SimulIDE connections\\r\\n");
} /*
   * =============================================================================
   * SIMULIDE DISPLAY FUNCTIONS
   * =============================================================================
   */

void simulide_display_header(void)
{
    puts_USART1("=== SimulIDE GLCD Header Display ===\\r\\n");

    // Clear display
    lcd_clear();
    _delay_ms(500); // Give SimulIDE time to process

    // Display course header
    lcd_string(0, 0, welcome_msg);
    _delay_ms(300);

    lcd_string(1, 0, course_msg);
    _delay_ms(300);

    // Add a simple separator line
    lcd_string(2, 0, "================");
    _delay_ms(300);

    lcd_string(3, 0, demo_msg);
    _delay_ms(300);

    puts_USART1("Header display complete\\r\\n");
    _delay_ms(2000);
}

void simulide_display_course_info(void)
{
    puts_USART1("=== SimulIDE Course Information ===\\r\\n");

    lcd_clear();
    _delay_ms(500);

    // Course information display
    lcd_string(0, 0, "Course: SOC3050");
    _delay_ms(400);

    lcd_string(1, 0, "Topic: Graphics");
    _delay_ms(400);

    lcd_string(2, 0, "Target: ATmega128");
    _delay_ms(400);

    lcd_string(3, 0, "Display: 128x64");
    _delay_ms(400);

    lcd_string(4, 0, "Mode: SimulIDE");
    _delay_ms(400);

    lcd_string(5, 0, status_msg);
    _delay_ms(400);

    puts_USART1("Course info display complete\\r\\n");
    _delay_ms(3000);
}

void simulide_display_simple_graphics(void)
{
    puts_USART1("=== SimulIDE Simple Graphics ===\\r\\n");

    lcd_clear();
    _delay_ms(500);

    // Title
    lcd_string(0, 0, "Simple Graphics:");
    _delay_ms(500);

    // Draw some simple patterns using text characters
    lcd_string(2, 0, "Patterns:");
    _delay_ms(300);

    lcd_string(3, 0, "***************");
    _delay_ms(300);

    lcd_string(4, 0, "###############");
    _delay_ms(300);

    lcd_string(5, 0, "===============");
    _delay_ms(300);

    puts_USART1("Simple graphics display complete\\r\\n");
    _delay_ms(2000);
}

void simulide_display_counter(void)
{
    puts_USART1("=== SimulIDE Counter Demo ===\\r\\n");

    lcd_clear();
    _delay_ms(500);

    lcd_string(0, 0, "Counter Demo:");
    _delay_ms(300);

    // Simple counter demonstration
    for (uint8_t count = 0; count < 10; count++)
    {
        char counter_str[20];
        sprintf(counter_str, "Count: %d", count);

        lcd_string(2, 0, counter_str);

        // Display in binary
        char binary_str[20];
        sprintf(binary_str, "Binary: %d%d%d%d",
                (count & 8) ? 1 : 0,
                (count & 4) ? 1 : 0,
                (count & 2) ? 1 : 0,
                (count & 1) ? 1 : 0);
        lcd_string(3, 0, binary_str);

        // Display in hex
        char hex_str[20];
        sprintf(hex_str, "Hex: 0x%X", count);
        lcd_string(4, 0, hex_str);

        // Status indicator
        if (count % 2 == 0)
        {
            lcd_string(5, 0, "Status: Even");
        }
        else
        {
            lcd_string(5, 0, "Status: Odd ");
        }

        char serial_msg[50];
        sprintf(serial_msg, "Counter: %d (0x%X, Binary: %d%d%d%d)\\r\\n",
                count, count,
                (count & 8) ? 1 : 0, (count & 4) ? 1 : 0,
                (count & 2) ? 1 : 0, (count & 1) ? 1 : 0);
        puts_USART1(serial_msg);

        _delay_ms(1500); // Slower for SimulIDE
    }

    puts_USART1("Counter demo complete\\r\\n");
}

void simulide_clear_and_wait(void)
{
    lcd_clear();
    _delay_ms(500);

    lcd_string(2, 0, "Demo Complete!");
    lcd_string(3, 0, "Check Serial");
    lcd_string(4, 0, "for details");

    _delay_ms(2000);
}

/*
 * =============================================================================
 * SIMULIDE MENU SYSTEM
 * =============================================================================
 */

void show_simulide_menu(void)
{
    puts_USART1("\\r\\n");
    puts_USART1("=========================================\\r\\n");
    puts_USART1("   GRAPHICS DISPLAY - SIMULIDE DEMO    \\r\\n");
    puts_USART1("=========================================\\r\\n");
    puts_USART1("1. Display Header Information           \\r\\n");
    puts_USART1("2. Show Course Information              \\r\\n");
    puts_USART1("3. Simple Graphics Demo                 \\r\\n");
    puts_USART1("4. Counter Demonstration               \\r\\n");
    puts_USART1("                                        \\r\\n");
    puts_USART1("0. Run All Demonstrations              \\r\\n");
    puts_USART1("X. Exit Demo                           \\r\\n");
    puts_USART1("=========================================\\r\\n");
    puts_USART1("Note: Optimized for SimulIDE performance\\r\\n");
    puts_USART1("Select option (1-4, 0, X): ");
}

/*
 * =============================================================================
 * MAIN PROGRAM
 * =============================================================================
 */

int main(void)
{
    simulide_init();

    // Very basic initial test - with extensive debugging
    puts_USART1("=== STARTING BASIC GLCD TESTS ===\\r\\n");

    // Test 1: Try to write directly using low-level functions
    puts_USART1("Test 1: Attempting basic display test...\\r\\n");

    // Try to set a pixel or basic pattern
    puts_USART1("Trying to clear display again...\\r\\n");
    lcd_clear();
    _delay_ms(2000); // Give SimulIDE plenty of time

    // Test 2: Try to write a single character at position 0,0
    puts_USART1("Test 2: Attempting to write single character...\\r\\n");
    lcd_string(0, 0, "H");
    puts_USART1("Character 'H' written to position 0,0\\r\\n");
    _delay_ms(3000);

    // Test 3: Try to write "HELLO"
    puts_USART1("Test 3: Attempting to write HELLO...\\r\\n");
    lcd_clear();
    _delay_ms(1000);
    lcd_string(0, 0, "HELLO");
    puts_USART1("Word 'HELLO' written to position 0,0\\r\\n");
    _delay_ms(3000);

    // Test 4: Try multiple lines
    puts_USART1("Test 4: Attempting multiple lines...\\r\\n");
    lcd_clear();
    _delay_ms(1000);

    lcd_string(0, 0, "Line 1");
    puts_USART1("Line 1 written\\r\\n");
    _delay_ms(1000);

    lcd_string(1, 0, "Line 2");
    puts_USART1("Line 2 written\\r\\n");
    _delay_ms(1000);

    lcd_string(2, 0, "Test OK");
    puts_USART1("Line 3 written\\r\\n");
    _delay_ms(3000);

    puts_USART1("=== BASIC TESTS COMPLETE ===\\r\\n");
    puts_USART1("If you see text in serial but nothing on GLCD, there's a connection issue\\r\\n");
    puts_USART1("Check: PORTA->Data, PE4->RS, PE5->EN, PE6->CS2, PE7->CS1\\r\\n"); // Initial display
    lcd_clear();
    _delay_ms(500);
    lcd_string(1, 0, "SimulIDE GLCD");
    _delay_ms(300);
    lcd_string(2, 0, "Demo Ready");
    _delay_ms(300);
    lcd_string(4, 0, "Use Serial Menu");
    _delay_ms(500);

    puts_USART1("SimulIDE Graphics Demo initialized\\r\\n");
    puts_USART1("GLCD display optimized for simulation\\r\\n");

    while (1)
    {
        show_simulide_menu();
        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\\r');
        putch_USART1('\\n');

        switch (choice)
        {
        case '1':
            simulide_display_header();
            break;

        case '2':
            simulide_display_course_info();
            break;

        case '3':
            simulide_display_simple_graphics();
            break;

        case '4':
            simulide_display_counter();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL DEMONSTRATIONS ***\\r\\n");
            simulide_display_header();
            simulide_clear_and_wait();

            simulide_display_course_info();
            simulide_clear_and_wait();

            simulide_display_simple_graphics();
            simulide_clear_and_wait();

            simulide_display_counter();
            simulide_clear_and_wait();

            puts_USART1("\\r\\n*** ALL DEMONSTRATIONS COMPLETE ***\\r\\n");
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting SimulIDE Graphics Demo\\r\\n");
            puts_USART1("Thank you for testing GLCD functionality!\\r\\n");

            lcd_clear();
            lcd_string(2, 0, "Demo Complete");
            lcd_string(3, 0, "Thank you!");

            while (1)
                ; // Stay here

        default:
            puts_USART1("Invalid choice. Please try again.\\r\\n");
        }

        puts_USART1("\\r\\nPress any key to continue...\\r\\n");
        getch_USART1();
    }

    return 0;
}
