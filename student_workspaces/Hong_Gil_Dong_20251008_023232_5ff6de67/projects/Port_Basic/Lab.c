/*
 * =============================================================================
 * PORT I/O PROGRAMMING - HANDS-ON LAB EXERCISES
 * =============================================================================
 *
 * PROJECT: Port_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Interactive laboratory exercises for hands-on experience with ATmega128 port I/O.
 * Students practice digital I/O programming through guided exercises and challenges.
 *
 * LAB OBJECTIVES:
 * 1. Practice LED control with various patterns
 * 2. Implement button-controlled interactions
 * 3. Create bit manipulation challenges
 * 4. Debug common port I/O issues
 * 5. Work on team-based programming exercises
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - 8 LEDs connected to PORTB (PB0-PB7)
 * - 4 push buttons on PORTD (PD4-PD7)
 * - Optional: Buzzer on PC0 for audio feedback
 * - Serial terminal for interaction (9600 baud)
 *
 * LAB STRUCTURE:
 * - Exercise 1: Knight Rider LED Pattern
 * - Exercise 2: Binary Counter Display
 * - Exercise 3: Button-LED Control
 * - Exercise 4: Reaction Time Game
 * - Exercise 5: Advanced Bit Operations
 *
 * DURATION: 90 minutes
 * DIFFICULTY: Beginner
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration
#define LED_PORT PORTB
#define LED_DDR DDRB
#define BUTTON_PORT PORTD
#define BUTTON_DDR DDRD
#define BUTTON_PIN PIND

#define BTN0 4 // PD4
#define BTN1 5 // PD5
#define BTN2 6 // PD6
#define BTN3 7 // PD7

// Global variables for lab exercises
volatile uint8_t button_pressed = 0;
volatile uint16_t score = 0;
volatile uint8_t level = 1;

/*
 * =============================================================================
 * LAB EXERCISE 1: LED PATTERN CHALLENGES (15 minutes)
 * =============================================================================
 * OBJECTIVE: Master LED control with creative patterns
 * DIFFICULTY: ★☆☆☆☆ (Basic)
 */

void lab_ex1_knight_rider(void)
{
    /*
     * CHALLENGE: Create the "Knight Rider" scanning effect
     * TASK: Make a single LED scan left-to-right, then right-to-left
     * HINT: Use bit shifting and delays
     */

    puts_USART1("\r\n=== Lab 1.1: Knight Rider Scanner ===\r\n");
    puts_USART1("Watch the LED scan back and forth!\r\n");

    LED_DDR = 0xFF; // All LEDs as outputs

    for (uint8_t cycle = 0; cycle < 5; cycle++)
    {
        // Scan left to right
        for (uint8_t i = 0; i < 8; i++)
        {
            LED_PORT = ~(1 << i); // Turn on one LED
            _delay_ms(100);
        }

        // Scan right to left
        for (int8_t i = 7; i >= 0; i--)
        {
            LED_PORT = ~(1 << i);
            _delay_ms(100);
        }
    }

    LED_PORT = 0xFF; // Turn off all LEDs
    puts_USART1("Pattern complete!\r\n");
}

void lab_ex1_binary_counter(void)
{
    /*
     * CHALLENGE: Display binary counting on LEDs
     * TASK: Count from 0 to 255 in binary, showing on LEDs
     * LEARNING: Understand binary number representation
     */

    puts_USART1("\r\n=== Lab 1.2: Binary Counter ===\r\n");
    puts_USART1("Counting 0-255 in binary on LEDs...\r\n");

    LED_DDR = 0xFF;

    for (uint16_t count = 0; count < 256; count++)
    {
        LED_PORT = ~count; // Display inverted (LEDs are active LOW)
        _delay_ms(50);

        // Print progress every 32 counts
        if (count % 32 == 0)
        {
            char buffer[40];
            sprintf(buffer, "Count: %u (0x%02X)\r\n", count, count);
            puts_USART1(buffer);
        }
    }

    LED_PORT = 0xFF;
    puts_USART1("Counting complete!\r\n");
}

void lab_ex1_random_sparkle(void)
{
    /*
     * CHALLENGE: Create random LED sparkle effect
     * TASK: Turn random LEDs on/off for 10 seconds
     * LEARNING: Pseudo-random number generation
     */

    puts_USART1("\r\n=== Lab 1.3: Random Sparkle ===\r\n");
    puts_USART1("Random LED sparkle for 10 seconds...\r\n");

    LED_DDR = 0xFF;
    uint16_t seed = 42; // Simple PRNG seed

    for (uint16_t i = 0; i < 200; i++)
    {
        // Simple linear congruential generator
        seed = (seed * 1103515245 + 12345) & 0xFFFF;
        LED_PORT = ~(seed & 0xFF);
        _delay_ms(50);
    }

    LED_PORT = 0xFF;
    puts_USART1("Sparkle complete!\r\n");
}

/*
 * =============================================================================
 * LAB EXERCISE 2: BUTTON-CONTROLLED INTERACTIONS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement responsive button controls
 * DIFFICULTY: ★★☆☆☆ (Easy)
 */

void lab_ex2_button_led_control(void)
{
    /*
     * CHALLENGE: Control individual LEDs with buttons
     * TASK: Button 0 -> LED 0, Button 1 -> LED 1, etc.
     * LEARNING: Reading input pins and controlling outputs
     */

    puts_USART1("\r\n=== Lab 2.1: Button-LED Control ===\r\n");
    puts_USART1("Press buttons to control corresponding LEDs\r\n");
    puts_USART1("Press all 4 buttons together to exit\r\n");

    LED_DDR = 0xFF;     // LEDs as outputs
    BUTTON_DDR = 0x00;  // Buttons as inputs (lower nibble)
    BUTTON_PORT = 0xF0; // Enable pull-ups on PD4-7

    LED_PORT = 0xFF; // Start with all LEDs off

    while (1)
    {
        uint8_t buttons = BUTTON_PIN;

        // Check if all buttons pressed (exit condition)
        if ((buttons & 0xF0) == 0x00)
        {
            puts_USART1("All buttons pressed - exiting!\r\n");
            break;
        }

        // Control LEDs based on buttons (inverted logic)
        if (!(buttons & (1 << BTN0)))
            LED_PORT &= ~(1 << 0);
        else
            LED_PORT |= (1 << 0);
        if (!(buttons & (1 << BTN1)))
            LED_PORT &= ~(1 << 1);
        else
            LED_PORT |= (1 << 1);
        if (!(buttons & (1 << BTN2)))
            LED_PORT &= ~(1 << 2);
        else
            LED_PORT |= (1 << 2);
        if (!(buttons & (1 << BTN3)))
            LED_PORT &= ~(1 << 3);
        else
            LED_PORT |= (1 << 3);

        _delay_ms(10); // Debouncing delay
    }

    LED_PORT = 0xFF;
}

void lab_ex2_reaction_game(void)
{
    /*
     * CHALLENGE: Create a reaction time game
     * TASK: Light random LED, measure how fast student presses corresponding button
     * LEARNING: Timing and human-computer interaction
     */

    puts_USART1("\r\n=== Lab 2.2: Reaction Time Game ===\r\n");
    puts_USART1("Press the button for the lit LED as fast as possible!\r\n");

    LED_DDR = 0xFF;
    BUTTON_DDR = 0x00;
    BUTTON_PORT = 0xF0;

    uint16_t seed = 12345;

    for (uint8_t round = 0; round < 5; round++)
    {
        char buffer[60];
        sprintf(buffer, "\r\nRound %u/5: Get ready...\r\n", round + 1);
        puts_USART1(buffer);

        _delay_ms(1000 + (seed % 1000)); // Random delay

        // Light random LED (0-3 to match buttons)
        seed = (seed * 1103515245 + 12345) & 0xFFFF;
        uint8_t target_led = seed % 4;
        LED_PORT = ~(1 << target_led);

        puts_USART1("GO! Press the button!\r\n");

        // Measure reaction time
        uint16_t reaction_time = 0;
        uint8_t correct = 0;

        while (reaction_time < 3000) // 3 second timeout
        {
            uint8_t buttons = BUTTON_PIN;

            if (!(buttons & (1 << (BTN0 + target_led))))
            {
                correct = 1;
                break;
            }

            _delay_ms(1);
            reaction_time++;
        }

        LED_PORT = 0xFF;

        if (correct)
        {
            sprintf(buffer, "Correct! Reaction time: %u ms\r\n", reaction_time);
            score += (1000 - reaction_time); // Faster = more points
        }
        else
        {
            sprintf(buffer, "Too slow! Timeout.\r\n");
        }
        puts_USART1(buffer);
    }

    char final_score[50];
    sprintf(final_score, "\r\nFinal Score: %u points!\r\n", score);
    puts_USART1(final_score);
}

void lab_ex2_sequence_memory(void)
{
    /*
     * CHALLENGE: Simon Says style memory game
     * TASK: Remember and repeat LED sequence
     * LEARNING: Arrays and sequence processing
     */

    puts_USART1("\r\n=== Lab 2.3: Sequence Memory Game ===\r\n");
    puts_USART1("Watch the sequence, then repeat it!\r\n");

    LED_DDR = 0xFF;
    BUTTON_DDR = 0x00;
    BUTTON_PORT = 0xF0;

    uint8_t sequence[10];
    uint16_t seed = 54321;

    for (level = 1; level <= 5; level++)
    {
        char buffer[50];
        sprintf(buffer, "\r\n--- Level %u: %u steps ---\r\n", level, level + 2);
        puts_USART1(buffer);

        // Generate sequence
        for (uint8_t i = 0; i < level + 2; i++)
        {
            seed = (seed * 1103515245 + 12345) & 0xFFFF;
            sequence[i] = seed % 4;
        }

        // Show sequence
        puts_USART1("Watch carefully...\r\n");
        _delay_ms(1000);

        for (uint8_t i = 0; i < level + 2; i++)
        {
            LED_PORT = ~(1 << sequence[i]);
            _delay_ms(500);
            LED_PORT = 0xFF;
            _delay_ms(300);
        }

        // Get player input
        puts_USART1("Your turn! Repeat the sequence.\r\n");
        uint8_t correct_count = 0;

        for (uint8_t i = 0; i < level + 2; i++)
        {
            // Wait for button press
            while (1)
            {
                uint8_t buttons = BUTTON_PIN;

                for (uint8_t btn = 0; btn < 4; btn++)
                {
                    if (!(buttons & (1 << (BTN0 + btn))))
                    {
                        LED_PORT = ~(1 << btn);
                        _delay_ms(300);
                        LED_PORT = 0xFF;

                        if (btn == sequence[i])
                        {
                            correct_count++;
                        }

                        // Wait for button release
                        while (!(BUTTON_PIN & (1 << (BTN0 + btn))))
                            _delay_ms(10);
                        goto next_step;
                    }
                }
            }
        next_step:;
        }

        if (correct_count == level + 2)
        {
            puts_USART1("Perfect! Moving to next level.\r\n");
            score += level * 100;
        }
        else
        {
            puts_USART1("Oops! Wrong sequence. Try again!\r\n");
            level--; // Retry this level
        }

        _delay_ms(1000);
    }

    puts_USART1("\r\n*** GAME COMPLETE! ***\r\n");
    char final_score[50];
    sprintf(final_score, "Total Score: %u points\r\n", score);
    puts_USART1(final_score);
}

/*
 * =============================================================================
 * LAB EXERCISE 3: BIT MANIPULATION CHALLENGES (15 minutes)
 * =============================================================================
 * OBJECTIVE: Practice advanced bit operations
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex3_bit_rotation(void)
{
    /*
     * CHALLENGE: Implement rotate left and rotate right
     * TASK: Visualize bit rotation on LEDs
     * LEARNING: Bit shifting and masking
     */

    puts_USART1("\r\n=== Lab 3.1: Bit Rotation ===\r\n");
    puts_USART1("Watch bits rotate left and right!\r\n");

    LED_DDR = 0xFF;
    uint8_t pattern = 0x01; // Start with bit 0 set

    puts_USART1("Rotating LEFT...\r\n");
    for (uint8_t i = 0; i < 8; i++)
    {
        LED_PORT = ~pattern;
        _delay_ms(300);

        // Rotate left
        uint8_t carry = (pattern & 0x80) >> 7; // Save MSB
        pattern = (pattern << 1) | carry;      // Shift and insert carry
    }

    puts_USART1("Rotating RIGHT...\r\n");
    for (uint8_t i = 0; i < 8; i++)
    {
        LED_PORT = ~pattern;
        _delay_ms(300);

        // Rotate right
        uint8_t carry = (pattern & 0x01) << 7; // Save LSB
        pattern = (pattern >> 1) | carry;      // Shift and insert carry
    }

    LED_PORT = 0xFF;
    puts_USART1("Rotation complete!\r\n");
}

void lab_ex3_bit_counting(void)
{
    /*
     * CHALLENGE: Count number of set bits (population count)
     * TASK: Display patterns and count '1' bits
     * LEARNING: Bit counting algorithms
     */

    puts_USART1("\r\n=== Lab 3.2: Bit Counting ===\r\n");
    puts_USART1("Counting set bits in various patterns...\r\n");

    LED_DDR = 0xFF;
    uint8_t patterns[] = {0x00, 0xFF, 0xAA, 0x55, 0x0F, 0xF0, 0x3C, 0x81};

    for (uint8_t p = 0; p < 8; p++)
    {
        uint8_t pattern = patterns[p];
        LED_PORT = ~pattern;

        // Count set bits
        uint8_t count = 0;
        uint8_t temp = pattern;
        while (temp)
        {
            count += temp & 1;
            temp >>= 1;
        }

        char buffer[60];
        sprintf(buffer, "Pattern 0x%02X has %u bits set\r\n", pattern, count);
        puts_USART1(buffer);

        _delay_ms(800);
    }

    LED_PORT = 0xFF;
}

void lab_ex3_parity_checker(void)
{
    /*
     * CHALLENGE: Calculate even/odd parity
     * TASK: Determine if number of set bits is even or odd
     * LEARNING: Parity calculation and error detection
     */

    puts_USART1("\r\n=== Lab 3.3: Parity Checker ===\r\n");
    puts_USART1("Checking parity of button inputs...\r\n");
    puts_USART1("Press buttons - LED 7 shows parity (ON=odd, OFF=even)\r\n");
    puts_USART1("Press all buttons to exit\r\n");

    LED_DDR = 0xFF;
    BUTTON_DDR = 0x00;
    BUTTON_PORT = 0xF0;

    while (1)
    {
        uint8_t buttons = ~BUTTON_PIN & 0xF0; // Get button states (inverted)

        // Exit condition
        if ((BUTTON_PIN & 0xF0) == 0x00)
            break;

        // Calculate parity
        uint8_t parity = 0;
        uint8_t temp = buttons;
        while (temp)
        {
            parity ^= (temp & 1);
            temp >>= 1;
        }

        // Show buttons on lower LEDs, parity on LED 7
        LED_PORT = ~(buttons | (parity << 7));

        _delay_ms(50);
    }

    LED_PORT = 0xFF;
    puts_USART1("Parity check complete!\r\n");
}

/*
 * =============================================================================
 * LAB EXERCISE 4: TEAM CHALLENGES (20 minutes)
 * =============================================================================
 * OBJECTIVE: Collaborative problem solving
 * DIFFICULTY: ★★★★☆ (Advanced)
 */

void lab_ex4_traffic_light(void)
{
    /*
     * CHALLENGE: Implement 4-way traffic light controller
     * TASK: Control traffic lights with proper timing and transitions
     * LEARNING: State machines and timing coordination
     */

    puts_USART1("\r\n=== Lab 4.1: Traffic Light Controller ===\r\n");
    puts_USART1("Simulating 4-way intersection...\r\n");
    puts_USART1("LED 0-2: North, 3-5: East, 6-7: Pedestrian\r\n");

    LED_DDR = 0xFF;

    enum
    {
        STATE_NS_GREEN,
        STATE_NS_YELLOW,
        STATE_EW_GREEN,
        STATE_EW_YELLOW,
        STATE_PED
    };
    uint8_t state = STATE_NS_GREEN;

    for (uint8_t cycle = 0; cycle < 3; cycle++) // 3 cycles
    {
        switch (state)
        {
        case STATE_NS_GREEN:
            LED_PORT = 0b11111100; // NS green, others red
            puts_USART1("North-South: GREEN\r\n");
            _delay_ms(3000);
            state = STATE_NS_YELLOW;
            break;

        case STATE_NS_YELLOW:
            LED_PORT = 0b11111101; // NS yellow
            puts_USART1("North-South: YELLOW\r\n");
            _delay_ms(1000);
            state = STATE_EW_GREEN;
            break;

        case STATE_EW_GREEN:
            LED_PORT = 0b11110011; // EW green, NS red
            puts_USART1("East-West: GREEN\r\n");
            _delay_ms(3000);
            state = STATE_EW_YELLOW;
            break;

        case STATE_EW_YELLOW:
            LED_PORT = 0b11110111; // EW yellow
            puts_USART1("East-West: YELLOW\r\n");
            _delay_ms(1000);
            state = STATE_PED;
            break;

        case STATE_PED:
            LED_PORT = 0b00111111; // Pedestrian walk
            puts_USART1("Pedestrian: WALK\r\n");
            _delay_ms(2000);
            // Flash warning
            for (uint8_t i = 0; i < 5; i++)
            {
                LED_PORT ^= 0xC0;
                _delay_ms(300);
            }
            state = STATE_NS_GREEN;
            break;
        }
    }

    LED_PORT = 0xFF;
    puts_USART1("Traffic simulation complete!\r\n");
}

void lab_ex4_morse_code(void)
{
    /*
     * CHALLENGE: Morse code translator
     * TASK: Display text as morse code on LED
     * LEARNING: Character encoding and timing
     */

    puts_USART1("\r\n=== Lab 4.2: Morse Code Translator ===\r\n");
    puts_USART1("Displaying 'SOS' in Morse code...\r\n");

    LED_DDR = 0xFF;

#define DOT_TIME 200
#define DASH_TIME 600
#define SYMBOL_GAP 200
#define LETTER_GAP 600

    // Morse code for "SOS"
    // S = ... (dot dot dot)
    // O = --- (dash dash dash)
    // S = ... (dot dot dot)

    auto void dot(void)
    {
        LED_PORT = 0x00;
        _delay_ms(DOT_TIME);
        LED_PORT = 0xFF;
        _delay_ms(SYMBOL_GAP);
    }

    auto void dash(void)
    {
        LED_PORT = 0x00;
        _delay_ms(DASH_TIME);
        LED_PORT = 0xFF;
        _delay_ms(SYMBOL_GAP);
    }

    for (uint8_t repeat = 0; repeat < 3; repeat++)
    {
        // S
        puts_USART1("S ");
        dot();
        dot();
        dot();
        _delay_ms(LETTER_GAP);

        // O
        puts_USART1("O ");
        dash();
        dash();
        dash();
        _delay_ms(LETTER_GAP);

        // S
        puts_USART1("S\r\n");
        dot();
        dot();
        dot();
        _delay_ms(LETTER_GAP * 2);
    }

    puts_USART1("Morse code complete!\r\n");
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
    puts_USART1("  PORT PROGRAMMING - LAB EXERCISES\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 1: LED Pattern Challenges\r\n");
    puts_USART1("  1. Knight Rider Scanner\r\n");
    puts_USART1("  2. Binary Counter\r\n");
    puts_USART1("  3. Random Sparkle\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 2: Button-Controlled Interactions\r\n");
    puts_USART1("  4. Button-LED Control\r\n");
    puts_USART1("  5. Reaction Time Game\r\n");
    puts_USART1("  6. Sequence Memory Game\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 3: Bit Manipulation\r\n");
    puts_USART1("  7. Bit Rotation\r\n");
    puts_USART1("  8. Bit Counting\r\n");
    puts_USART1("  9. Parity Checker\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 4: Team Challenges\r\n");
    puts_USART1("  A. Traffic Light Controller\r\n");
    puts_USART1("  B. Morse Code Translator\r\n");
    puts_USART1("\r\n");
    puts_USART1("  0. Run All Exercises\r\n");
    puts_USART1("  X. Exit Lab\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select exercise (1-9, A, B, 0, X): ");
}

int main(void)
{
    // Initialize system
    init_devices();
    Uart1_init();

    _delay_ms(100);

    puts_USART1("\r\n\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("*  ATmega128 PORT PROGRAMMING LAB              *\r\n");
    puts_USART1("*  Hands-On Exercises for Students             *\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("\r\n");
    puts_USART1("Welcome to the Port Programming Lab!\r\n");
    puts_USART1("Complete all exercises to master port I/O.\r\n");

    while (1)
    {
        print_lab_menu();

        // Wait for user input
        char choice = getch_USART1();
        putch_USART1(choice); // Echo
        putch_USART1('\r');
        putch_USART1('\n');

        // Reset score for each exercise
        score = 0;

        switch (choice)
        {
        case '1':
            lab_ex1_knight_rider();
            break;
        case '2':
            lab_ex1_binary_counter();
            break;
        case '3':
            lab_ex1_random_sparkle();
            break;
        case '4':
            lab_ex2_button_led_control();
            break;
        case '5':
            lab_ex2_reaction_game();
            break;
        case '6':
            lab_ex2_sequence_memory();
            break;
        case '7':
            lab_ex3_bit_rotation();
            break;
        case '8':
            lab_ex3_bit_counting();
            break;
        case '9':
            lab_ex3_parity_checker();
            break;
        case 'A':
        case 'a':
            lab_ex4_traffic_light();
            break;
        case 'B':
        case 'b':
            lab_ex4_morse_code();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_knight_rider();
            lab_ex1_binary_counter();
            lab_ex1_random_sparkle();
            lab_ex2_button_led_control();
            lab_ex2_reaction_game();
            lab_ex2_sequence_memory();
            lab_ex3_bit_rotation();
            lab_ex3_bit_counting();
            lab_ex3_parity_checker();
            lab_ex4_traffic_light();
            lab_ex4_morse_code();
            puts_USART1("\r\n*** ALL EXERCISES COMPLETE! ***\r\n");
            break;

        case 'X':
        case 'x':
            puts_USART1("\r\nExiting lab. Great work!\r\n");
            LED_PORT = 0xFF;
            while (1)
                ; // Stop

        default:
            puts_USART1("Invalid choice. Please try again.\r\n");
        }

        puts_USART1("\r\nPress any key to continue...\r\n");
        getch_USART1();
    }

    return 0;
}
