/*
 * =============================================================================
 * SPI MASTER COMMUNICATION - HANDS-ON LAB EXERCISES
 * =============================================================================
 *
 * PROJECT: SPI_Master_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Interactive laboratory exercises for hands-on experience with SPI master protocols.
 * Students practice synchronous communication through guided exercises and device control.
 *
 * LAB OBJECTIVES:
 * 1. Configure ATmega128 as SPI master controller
 * 2. Interface with various SPI devices and modes
 * 3. Implement device selection and timing control
 * 4. Debug SPI communication and signal integrity
 * 5. Create multi-device SPI bus management systems
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - SPI slave devices (EEPROM, DAC, shift registers)
 * - SPI bus connections (MOSI, MISO, SCK, SS)
 * - Logic analyzer or oscilloscope for signal verification
 * - Serial terminal for debugging (9600 baud)
 *
 * LAB STRUCTURE:
 * - Exercise 1: SPI Master Configuration and Testing
 * - Exercise 2: EEPROM Read/Write Operations
 * - Exercise 3: DAC Control and Waveform Generation
 * - Exercise 4: Multi-Device SPI Bus Management
 * - Exercise 5: Protocol Debugging and Optimization
 *
 * DURATION: 75 minutes
 * DIFFICULTY: Intermediate
 *
 * =============================================================================
 */
 * - SPI devices: 74HC595 shift register, SPI EEPROM
 * - 8 LEDs connected to shift register outputs
 * - Logic analyzer or oscilloscope (optional)
 * - Pull-up resistors on MISO line
 *
 * SPI PINS (ATmega128):
 * - MOSI (Master Out, Slave In): PB2
 * - MISO (Master In, Slave Out): PB3
 * - SCK (Serial Clock): PB1
 * - SS (Slave Select): PB0 (and additional pins for multiple devices)
 *
 * LAB STRUCTURE:
 * - Exercise 1: SPI initialization and basic data transfer (20 min)
 * - Exercise 2: 74HC595 shift register control (20 min)
 * - Exercise 3: SPI timing and mode configuration (20 min)
 * - Exercise 4: Multi-device SPI bus management (15 min)
 *
 * =============================================================================
 */

#include "config.h"

 // SPI device control pins
#define SPI_SS_595 0    // PB0 - 74HC595 Slave Select
#define SPI_SS_EEPROM 4 // PB4 - EEPROM Slave Select
#define SPI_LATCH_595 5 // PB5 - 74HC595 Latch pin

 // SPI configuration
#define SPI_MODE_0 0 // CPOL=0, CPHA=0
#define SPI_MODE_1 1 // CPOL=0, CPHA=1
#define SPI_MODE_2 2 // CPOL=1, CPHA=0
#define SPI_MODE_3 3 // CPOL=1, CPHA=1

// Lab session variables
uint16_t lab_score = 0;
 uint8_t spi_transactions = 0;
 uint8_t current_led_pattern = 0;

 /*
  * =============================================================================
  * SPI COMMUNICATION FUNCTIONS
  * =============================================================================
  */

 void spi_master_init(uint8_t mode, uint8_t clock_div)
 {
     // Set SPI pins as outputs (MOSI, SCK, SS)
     DDRB |= (1 << PB2) | (1 << PB1) | (1 << PB0) | (1 << SPI_SS_EEPROM) | (1 << SPI_LATCH_595);

     // Set MISO as input
     DDRB &= ~(1 << PB3);

     // Enable pull-up on MISO
     PORTB |= (1 << PB3);

     // Configure SPI Control Register
     SPCR = (1 << SPE) | (1 << MSTR); // Enable SPI, Master mode

     // Set clock polarity and phase based on mode
     switch (mode)
     {
     case SPI_MODE_0:
         SPCR &= ~((1 << CPOL) | (1 << CPHA));
         break;
     case SPI_MODE_1:
         SPCR &= ~(1 << CPOL);
         SPCR |= (1 << CPHA);
         break;
     case SPI_MODE_2:
         SPCR |= (1 << CPOL);
         SPCR &= ~(1 << CPHA);
         break;
     case SPI_MODE_3:
         SPCR |= (1 << CPOL) | (1 << CPHA);
         break;
     }

     // Set clock prescaler
     SPCR &= ~((1 << SPR1) | (1 << SPR0));
     SPSR &= ~(1 << SPI2X);

     switch (clock_div)
     {
     case 4:
         SPSR |= (1 << SPI2X);
         break; // f/4
     case 8:    /* default settings */
         break; // f/8
     case 16:
         SPCR |= (1 << SPR0);
         break; // f/16
     case 32:
         SPSR |= (1 << SPI2X);
         SPCR |= (1 << SPR0);
         break; // f/32
     case 64:
         SPCR |= (1 << SPR1);
         break; // f/64
     case 128:
         SPCR |= (1 << SPR1) | (1 << SPR0);
         break; // f/128
     }

     // Set SS pins high (inactive)
     PORTB |= (1 << SPI_SS_595) | (1 << SPI_SS_EEPROM);
 }

 uint8_t spi_transfer(uint8_t data)
 {
     SPDR = data; // Start transmission
     while (!(SPSR & (1 << SPIF)))
         ;        // Wait for transmission complete
     return SPDR; // Return received data
 }

 void spi_select_device(uint8_t device_pin)
 {
     PORTB &= ~(1 << device_pin); // Pull SS low to select device
 }

 void spi_deselect_device(uint8_t device_pin)
 {
     PORTB |= (1 << device_pin); // Pull SS high to deselect device
 }

 /*
  * =============================================================================
  * DEVICE-SPECIFIC FUNCTIONS
  * =============================================================================
  */

 void shift_register_send(uint8_t data)
 {
     spi_select_device(SPI_SS_595);
     spi_transfer(data);
     spi_deselect_device(SPI_SS_595);

     // Pulse latch pin to transfer data to outputs
     PORTB |= (1 << SPI_LATCH_595);
     _delay_us(1);
     PORTB &= ~(1 << SPI_LATCH_595);
 }

 void shift_register_send_16bit(uint16_t data)
 {
     spi_select_device(SPI_SS_595);
     spi_transfer((data >> 8) & 0xFF); // Send high byte first
     spi_transfer(data & 0xFF);        // Send low byte
     spi_deselect_device(SPI_SS_595);

     // Latch data
     PORTB |= (1 << SPI_LATCH_595);
     _delay_us(1);
     PORTB &= ~(1 << SPI_LATCH_595);
 }

 /*
  * =============================================================================
  * LAB EXERCISE 1: SPI INITIALIZATION AND BASIC TRANSFER (20 minutes)
  * =============================================================================
  * OBJECTIVE: Learn SPI protocol fundamentals and basic data transfer
  * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
  */

 void lab_ex1_spi_basic_init(void)
 {
     /*
      * CHALLENGE: Initialize SPI and test basic communication
      * TASK: Configure SPI master and verify signal generation
      * LEARNING: SPI registers, timing, signal integrity
      */

     puts_USART1("\\r\\n=== Lab 1: SPI Basic Initialization ===\\r\\n");
     puts_USART1("Initializing SPI master with different configurations\\r\\n");

     lcd_clear();
     lcd_string(0, 0, "SPI BASIC INIT");
     lcd_string(1, 0, "Testing modes");

     // Test different SPI modes
     const char *mode_names[] = {"Mode 0", "Mode 1", "Mode 2", "Mode 3"};

     for (uint8_t mode = 0; mode < 4; mode++)
     {
         char test_msg[50];
         sprintf(test_msg, "Testing SPI %s...\\r\\n", mode_names[mode]);
         puts_USART1(test_msg);

         // Initialize SPI with current mode
         spi_master_init(mode, 16); // f/16 clock speed

         // LCD display
         char lcd_msg[20];
         sprintf(lcd_msg, "Mode: %d", mode);
         lcd_string(3, 0, lcd_msg);

         // Send test pattern to shift register
         uint8_t test_pattern = 0xAA; // Alternating bits
         shift_register_send(test_pattern);

         sprintf(test_msg, "Sent test pattern: 0x%02X\\r\\n", test_pattern);
         puts_USART1(test_msg);

         _delay_ms(1000);

         // Send different pattern
         test_pattern = 0x55;
         shift_register_send(test_pattern);

         sprintf(test_msg, "Sent test pattern: 0x%02X\\r\\n", test_pattern);
         puts_USART1(test_msg);

         _delay_ms(1000);

         spi_transactions += 2;
     }

     puts_USART1("SPI initialization test complete!\\r\\n");
     lab_score += 100;
 }

 void lab_ex1_clock_speed_test(void)
 {
     /*
      * CHALLENGE: Test different SPI clock speeds
      * TASK: Configure various clock prescalers and observe timing
      * LEARNING: Clock generation, timing requirements, speed limitations
      */

     puts_USART1("\\r\\n=== Lab 1.2: Clock Speed Test ===\\r\\n");
     puts_USART1("Testing different SPI clock speeds\\r\\n");

     lcd_clear();
     lcd_string(0, 0, "SPI CLOCK TEST");
     lcd_string(1, 0, "Speed variations");

     uint8_t clock_dividers[] = {4, 8, 16, 32, 64, 128};
     const char *speed_names[] = {"f/4", "f/8", "f/16", "f/32", "f/64", "f/128"};
     uint8_t num_speeds = sizeof(clock_dividers) / sizeof(clock_dividers[0]);

     for (uint8_t i = 0; i < num_speeds; i++)
     {
         char speed_msg[50];
         sprintf(speed_msg, "Testing clock speed %s...\\r\\n", speed_names[i]);
         puts_USART1(speed_msg);

         // Initialize with new clock speed
         spi_master_init(SPI_MODE_0, clock_dividers[i]);

         // LCD display
         lcd_string(3, 0, speed_names[i]);

         // Send rotating pattern
         uint8_t pattern = 0x01;
         for (uint8_t j = 0; j < 8; j++)
         {
             shift_register_send(pattern);
             pattern = (pattern << 1) | (pattern >> 7); // Rotate left
             _delay_ms(200);
         }

         shift_register_send(0x00); // Turn off all LEDs
         spi_transactions += 9;
     }

     puts_USART1("Clock speed test complete!\\r\\n");
     lab_score += 100;
 }

 /*
  * =============================================================================
  * LAB EXERCISE 2: 74HC595 SHIFT REGISTER CONTROL (20 minutes)
  * =============================================================================
  * OBJECTIVE: Master shift register operation and LED control
  * DIFFICULTY: ★★★☆☆ (Medium)
  */

 void lab_ex2_led_patterns(void)
 {
     /*
      * CHALLENGE: Create various LED patterns using shift register
      * TASK: Implement different visual effects and animations
      * LEARNING: Serial-to-parallel conversion, pattern generation
      */

     puts_USART1("\\r\\n=== Lab 2: LED Pattern Control ===\\r\\n");
     puts_USART1("Creating LED patterns with 74HC595\\r\\n");

     lcd_clear();
     lcd_string(0, 0, "LED PATTERNS");
     lcd_string(1, 0, "74HC595 Control");

     spi_master_init(SPI_MODE_0, 16);

     // Pattern 1: Sequential LEDs
     puts_USART1("Pattern 1: Sequential LEDs\\r\\n");
     lcd_string(3, 0, "Sequential");

     for (uint8_t i = 0; i < 8; i++)
     {
         shift_register_send(1 << i);
         _delay_ms(300);
     }

     // Pattern 2: Running lights
     puts_USART1("Pattern 2: Running lights\\r\\n");
     lcd_string(3, 0, "Running");

     uint8_t running_pattern = 0x07; // 3 consecutive LEDs
     for (uint8_t i = 0; i < 12; i++)
     {
         shift_register_send(running_pattern);
         running_pattern = (running_pattern << 1) | (running_pattern >> 7);
         _delay_ms(200);
     }

     // Pattern 3: Binary counter
     puts_USART1("Pattern 3: Binary counter\\r\\n");
     lcd_string(3, 0, "Binary count");

     for (uint16_t count = 0; count < 256; count++)
     {
         shift_register_send(count);

         char count_msg[20];
         sprintf(count_msg, "Count: %3d", count);
         lcd_string(4, 0, count_msg);

         _delay_ms(100);
     }

     // Pattern 4: Breathing effect
     puts_USART1("Pattern 4: Breathing effect\\r\\n");
     lcd_string(3, 0, "Breathing");

     for (uint8_t cycle = 0; cycle < 3; cycle++)
     {
         // Expand
         for (uint8_t i = 0; i < 8; i++)
         {
             uint8_t pattern = 0;
             for (uint8_t j = 0; j <= i; j++)
             {
                 pattern |= (1 << j) | (1 << (7 - j));
             }
             shift_register_send(pattern);
             _delay_ms(150);
         }

         // Contract
         for (int8_t i = 7; i >= 0; i--)
         {
             uint8_t pattern = 0;
             for (uint8_t j = 0; j <= i; j++)
             {
                 pattern |= (1 << j) | (1 << (7 - j));
             }
             shift_register_send(pattern);
             _delay_ms(150);
         }
     }

     shift_register_send(0x00); // All off
     spi_transactions += 50;
     lab_score += 150;
 }

 /*
  * =============================================================================
  * LAB EXERCISE 3: SPI TIMING AND MODE CONFIGURATION (20 minutes)
  * =============================================================================
  * OBJECTIVE: Understand SPI timing requirements and mode differences
  * DIFFICULTY: ★★★★☆ (Medium-Hard)
  */

 void lab_ex3_timing_analysis(void)
 {
     /*
      * CHALLENGE: Analyze SPI timing with different configurations
      * TASK: Test timing critical operations and mode compatibility
      * LEARNING: Setup/hold times, clock polarity/phase effects
      */

     puts_USART1("\\r\\n=== Lab 3: SPI Timing Analysis ===\\r\\n");
     puts_USART1("Testing SPI timing with various configurations\\r\\n");

     lcd_clear();
     lcd_string(0, 0, "TIMING ANALYSIS");
     lcd_string(1, 0, "Mode testing");

     // Test each SPI mode with timing-sensitive operations
     for (uint8_t mode = 0; mode < 4; mode++)
     {
         char mode_msg[50];
         sprintf(mode_msg, "Testing SPI Mode %d (CPOL=%d, CPHA=%d)\\r\\n",
                 mode, (mode & 2) ? 1 : 0, (mode & 1) ? 1 : 0);
         puts_USART1(mode_msg);

         spi_master_init(mode, 8); // Use faster clock for timing test

         char lcd_msg[20];
         sprintf(lcd_msg, "Mode %d Test", mode);
         lcd_string(3, 0, lcd_msg);

         // Perform rapid data transfers
         uint32_t start_time = 0; // Simple timing counter

         for (uint16_t i = 0; i < 100; i++)
         {
             shift_register_send(i & 0xFF);
             start_time++;
         }

         char timing_msg[60];
         sprintf(timing_msg, "Mode %d: 100 transfers completed, timing=%ld\\r\\n",
                 mode, start_time);
         puts_USART1(timing_msg);

         // Test with known good pattern
         shift_register_send(0xF0); // Test pattern
         _delay_ms(500);
         shift_register_send(0x0F); // Inverted pattern
         _delay_ms(500);

         spi_transactions += 102;
     }

     shift_register_send(0x00);
     puts_USART1("Timing analysis complete!\\r\\n");
     lab_score += 150;
 }

 /*
  * =============================================================================
  * LAB EXERCISE 4: MULTI-DEVICE SPI BUS (15 minutes)
  * =============================================================================
  * OBJECTIVE: Manage multiple SPI devices on the same bus
  * DIFFICULTY: ★★★★★ (Hard)
  */

 void lab_ex4_multi_device_control(void)
 {
     /*
      * CHALLENGE: Control multiple SPI devices simultaneously
      * TASK: Coordinate between shift register and other SPI devices
      * LEARNING: Device selection, bus arbitration, timing coordination
      */

     puts_USART1("\\r\\n=== Lab 4: Multi-Device Control ===\\r\\n");
     puts_USART1("Managing multiple SPI devices on the bus\\r\\n");

     lcd_clear();
     lcd_string(0, 0, "MULTI-DEVICE SPI");
     lcd_string(1, 0, "Bus management");

     spi_master_init(SPI_MODE_0, 16);

     // Simulate multi-device scenario
     puts_USART1("Scenario: Coordinated LED control with data logging\\r\\n");

     for (uint8_t sequence = 0; sequence < 10; sequence++)
     {
         char seq_msg[30];
         sprintf(seq_msg, "Sequence %d:", sequence + 1);
         lcd_string(3, 0, seq_msg);

         // Device 1: Update LED pattern
         uint8_t led_pattern = (sequence % 2) ? 0xAA : 0x55;

         puts_USART1("Updating LEDs... ");
         spi_select_device(SPI_SS_595);
         spi_transfer(led_pattern);
         spi_deselect_device(SPI_SS_595);

         // Latch the data
         PORTB |= (1 << SPI_LATCH_595);
         _delay_us(1);
         PORTB &= ~(1 << SPI_LATCH_595);

         puts_USART1("Done\\r\\n");

         // Small delay between device operations
         _delay_ms(100);

         // Device 2: Simulate EEPROM write (just SPI transfer for demo)
         puts_USART1("Simulating EEPROM operation... ");
         spi_select_device(SPI_SS_EEPROM);
         spi_transfer(0x02);        // Write command
         spi_transfer(0x00);        // Address high
         spi_transfer(sequence);    // Address low
         spi_transfer(led_pattern); // Data
         spi_deselect_device(SPI_SS_EEPROM);

         puts_USART1("Done\\r\\n");

         char details[40];
         sprintf(details, "LED: 0x%02X, ADDR: %02X\\r\\n", led_pattern, sequence);
         puts_USART1(details);

         sprintf(details, "LED:0x%02X A:%02X", led_pattern, sequence);
         lcd_string(4, 0, details);

         spi_transactions += 5;
         _delay_ms(800);
     }

     // Final test: Complex coordination
     puts_USART1("\\r\\nFinal test: Complex device coordination\\r\\n");

     // Create a dancing LED pattern while simulating data operations
     for (uint8_t i = 0; i < 16; i++)
     {
         // Update LEDs with complex pattern
         uint8_t pattern = (i % 8 < 4) ? (1 << (i % 4)) : (0x80 >> (i % 4));

         shift_register_send(pattern);

         // Simulate read operation from EEPROM
         spi_select_device(SPI_SS_EEPROM);
         spi_transfer(0x03);                      // Read command
         spi_transfer(0x00);                      // Address high
         spi_transfer(i);                         // Address low
         uint8_t dummy_data = spi_transfer(0x00); // Read data
         spi_deselect_device(SPI_SS_EEPROM);

         char coord_msg[50];
         sprintf(coord_msg, "Coord %2d: LED=0x%02X, Read=0x%02X\\r\\n",
                 i, pattern, dummy_data);
         puts_USART1(coord_msg);

         _delay_ms(200);
         spi_transactions += 5;
     }

     shift_register_send(0x00); // All LEDs off
     puts_USART1("Multi-device control complete!\\r\\n");
     lab_score += 200;
 }

 /*
  * =============================================================================
  * LAB MAIN PROGRAM - EXERCISE SELECTION
  * =============================================================================
  */

 void show_lab_menu(void)
 {
     puts_USART1("\\r\\n");
     puts_USART1("==============================================\\r\\n");
     puts_USART1("      SPI MASTER BASIC - LAB EXERCISES       \\r\\n");
     puts_USART1("==============================================\\r\\n");
     puts_USART1("1. SPI Initialization & Basic Transfer       \\r\\n");
     puts_USART1("2. 74HC595 Shift Register Control           \\r\\n");
     puts_USART1("3. SPI Timing and Mode Configuration        \\r\\n");
     puts_USART1("4. Multi-Device SPI Bus Management          \\r\\n");
     puts_USART1("                                              \\r\\n");
     puts_USART1("0. Run All Exercises                         \\r\\n");
     puts_USART1("X. Exit Lab                                   \\r\\n");
     puts_USART1("==============================================\\r\\n");
     char score_msg[50];
     sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
     puts_USART1(score_msg);
     char trans_msg[50];
     sprintf(trans_msg, "SPI Transactions: %d\\r\\n", spi_transactions);
     puts_USART1(trans_msg);
     puts_USART1("Select exercise (1-4, 0, X): ");
 }

 int main(void)
 {
     init_devices();

     puts_USART1("\\r\\n*** SPI MASTER BASIC LAB SESSION ***\\r\\n");
     puts_USART1("Welcome to hands-on SPI communication!\\r\\n");
     puts_USART1("Ensure SPI devices are properly connected\\r\\n");
     puts_USART1("Check: MOSI(PB2), MISO(PB3), SCK(PB1), SS pins\\r\\n");

     lcd_clear();
     lcd_string(1, 0, "SPI MASTER LAB");
     lcd_string(2, 0, "Check connections");
     lcd_string(4, 0, "Use Serial Menu");

     while (1)
     {
         show_lab_menu();
         char choice = getch_USART1();
         putch_USART1(choice);
         putch_USART1('\\r');
         putch_USART1('\\n');

         switch (choice)
         {
         case '1':
             lab_ex1_spi_basic_init();
             lab_ex1_clock_speed_test();
             break;

         case '2':
             lab_ex2_led_patterns();
             break;

         case '3':
             lab_ex3_timing_analysis();
             break;

         case '4':
             lab_ex4_multi_device_control();
             break;

         case '0':
             puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
             lab_ex1_spi_basic_init();
             lab_ex1_clock_speed_test();
             lab_ex2_led_patterns();
             lab_ex3_timing_analysis();
             lab_ex4_multi_device_control();

             char final_buffer[80];
             sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
             puts_USART1(final_buffer);
             break;

         case 'X':
         case 'x':
             puts_USART1("\\r\\nExiting lab. Great work on SPI!\\r\\n");
             puts_USART1("Remember: Check SPI mode compatibility!\\r\\n");
             lcd_clear();
             lcd_string(2, 0, "LAB COMPLETE!");
             char exit_score[30];
             sprintf(exit_score, "Score: %d pts", lab_score);
             lcd_string(3, 0, exit_score);
             // Turn off all LEDs
             shift_register_send(0x00);
             while (1)
                 ;

         default:
             puts_USART1("Invalid choice. Please try again.\\r\\n");
         }

         puts_USART1("\\r\\nPress any key to continue...\\r\\n");
         getch_USART1();
     }

     return 0;
 }