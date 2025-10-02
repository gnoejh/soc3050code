/*
 * Memory Test - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand SRAM and EEPROM memory operations
 * - Learn memory testing and validation techniques
 * - Practice data persistence across power cycles
 * - Master memory addressing and data integrity
 *
 * HARDWARE SETUP:
 * - LEDs on PORTB for memory test status indication
 * - UART for detailed memory test reports
 * - EEPROM for non-volatile data storage testing
 */

#include "config.h"

// Test patterns for memory validation
#define PATTERN_0x55 0x55 // Alternating bits: 01010101
#define PATTERN_0xAA 0xAA // Alternating bits: 10101010
#define PATTERN_0xFF 0xFF // All bits high
#define PATTERN_0x00 0x00 // All bits low

// EEPROM test addresses
#define EEPROM_TEST_START 0x00
#define EEPROM_TEST_SIZE 64

// SRAM test configuration
#define SRAM_TEST_SIZE 256
static uint8_t sram_test_buffer[SRAM_TEST_SIZE];

// Function to test SRAM with various patterns
uint8_t test_sram_pattern(uint8_t pattern, const char *pattern_name)
{
    char buffer[60];
    sprintf(buffer, "Testing SRAM with pattern %s (0x%02X)\r\n", pattern_name, pattern);
    puts_USART1(buffer);

    // Write pattern to SRAM
    for (uint16_t i = 0; i < SRAM_TEST_SIZE; i++)
    {
        sram_test_buffer[i] = pattern;
    }

    // Verify pattern in SRAM
    uint16_t errors = 0;
    for (uint16_t i = 0; i < SRAM_TEST_SIZE; i++)
    {
        if (sram_test_buffer[i] != pattern)
        {
            errors++;
        }
    }

    if (errors == 0)
    {
        puts_USART1("SRAM pattern test: PASSED\r\n");
        return 1; // Success
    }
    else
    {
        sprintf(buffer, "SRAM pattern test: FAILED (%u errors)\r\n", errors);
        puts_USART1(buffer);
        return 0; // Failure
    }
}

// Function to test SRAM with walking bits
uint8_t test_sram_walking_bits()
{
    puts_USART1("Testing SRAM with walking bits pattern\r\n");

    uint16_t errors = 0;

    // Test each bit position
    for (uint8_t bit = 0; bit < 8; bit++)
    {
        uint8_t pattern = (1 << bit);

        // Write walking bit pattern
        for (uint16_t i = 0; i < SRAM_TEST_SIZE; i++)
        {
            sram_test_buffer[i] = pattern;
        }

        // Verify walking bit pattern
        for (uint16_t i = 0; i < SRAM_TEST_SIZE; i++)
        {
            if (sram_test_buffer[i] != pattern)
            {
                errors++;
            }
        }

        // Visual feedback
        PORTB = ~pattern; // Show bit pattern on LEDs
        _delay_ms(200);
    }

    PORTB = 0xFF; // Turn off LEDs

    if (errors == 0)
    {
        puts_USART1("SRAM walking bits test: PASSED\r\n");
        return 1;
    }
    else
    {
        char buffer[50];
        sprintf(buffer, "SRAM walking bits test: FAILED (%u errors)\r\n", errors);
        puts_USART1(buffer);
        return 0;
    }
}

// Function to test EEPROM operations
uint8_t test_eeprom_operations()
{
    puts_USART1("Testing EEPROM read/write operations\r\n");

    uint16_t errors = 0;

    // Test data patterns
    uint8_t test_patterns[] = {0x00, 0xFF, 0x55, 0xAA, 0x33, 0xCC};
    uint8_t num_patterns = sizeof(test_patterns);

    for (uint8_t pattern_idx = 0; pattern_idx < num_patterns; pattern_idx++)
    {
        uint8_t pattern = test_patterns[pattern_idx];

        char buffer[50];
        sprintf(buffer, "EEPROM testing pattern 0x%02X\r\n", pattern);
        puts_USART1(buffer);

        // Write pattern to EEPROM
        for (uint16_t addr = EEPROM_TEST_START; addr < EEPROM_TEST_START + EEPROM_TEST_SIZE; addr++)
        {
            EEPROM_write(addr, pattern);
            _delay_ms(5); // EEPROM write delay
        }

        // Verify pattern in EEPROM
        for (uint16_t addr = EEPROM_TEST_START; addr < EEPROM_TEST_START + EEPROM_TEST_SIZE; addr++)
        {
            uint8_t read_value = EEPROM_read(addr);
            if (read_value != pattern)
            {
                errors++;
                sprintf(buffer, "EEPROM error at 0x%04X: wrote 0x%02X, read 0x%02X\r\n",
                        addr, pattern, read_value);
                puts_USART1(buffer);
            }
        }

        // Visual progress indication
        PORTB = ~(1 << pattern_idx);
        _delay_ms(300);
    }

    PORTB = 0xFF; // Turn off LEDs

    if (errors == 0)
    {
        puts_USART1("EEPROM operations test: PASSED\r\n");
        return 1;
    }
    else
    {
        char buffer[50];
        sprintf(buffer, "EEPROM operations test: FAILED (%u errors)\r\n", errors);
        puts_USART1(buffer);
        return 0;
    }
}

// Function to demonstrate data persistence
void demonstrate_data_persistence()
{
    puts_USART1("Demonstrating EEPROM data persistence\r\n");

    // Check if we have previous boot data
    uint8_t boot_counter = EEPROM_read(0x3F0); // Use address near end of EEPROM
    uint8_t magic_byte = EEPROM_read(0x3F1);

    if (magic_byte == 0xAB) // Magic byte indicates valid data
    {
        boot_counter++;
        char buffer[50];
        sprintf(buffer, "Previous boot detected. Boot count: %u\r\n", boot_counter);
        puts_USART1(buffer);
    }
    else
    {
        puts_USART1("First boot detected. Initializing EEPROM\r\n");
        boot_counter = 1;
        EEPROM_write(0x3F1, 0xAB); // Write magic byte
        _delay_ms(5);
    }

    // Update boot counter
    EEPROM_write(0x3F0, boot_counter);
    _delay_ms(5);

    // Store timestamp pattern (simulated)
    uint8_t timestamp_pattern = boot_counter ^ 0x55;
    EEPROM_write(0x3F2, timestamp_pattern);
    _delay_ms(5);

    char buffer[60];
    sprintf(buffer, "Boot #%u data stored in EEPROM\r\n", boot_counter);
    puts_USART1(buffer);
}

// Function to run comprehensive memory test suite
void run_memory_test_suite()
{
    puts_USART1("\r\n=== COMPREHENSIVE MEMORY TEST SUITE ===\r\n");

    uint8_t tests_passed = 0;
    uint8_t total_tests = 7;

    // SRAM pattern tests
    if (test_sram_pattern(PATTERN_0x55, "0x55"))
        tests_passed++;
    _delay_ms(500);

    if (test_sram_pattern(PATTERN_0xAA, "0xAA"))
        tests_passed++;
    _delay_ms(500);

    if (test_sram_pattern(PATTERN_0xFF, "0xFF"))
        tests_passed++;
    _delay_ms(500);

    if (test_sram_pattern(PATTERN_0x00, "0x00"))
        tests_passed++;
    _delay_ms(500);

    // SRAM walking bits test
    if (test_sram_walking_bits())
        tests_passed++;
    _delay_ms(500);

    // EEPROM tests
    if (test_eeprom_operations())
        tests_passed++;
    _delay_ms(500);

    // Data persistence demonstration
    demonstrate_data_persistence();
    tests_passed++; // Always passes (demonstration)

    // Test results summary
    char buffer[60];
    sprintf(buffer, "\r\n=== TEST RESULTS ===\r\n");
    puts_USART1(buffer);
    sprintf(buffer, "Tests Passed: %u/%u\r\n", tests_passed, total_tests);
    puts_USART1(buffer);

    if (tests_passed == total_tests)
    {
        puts_USART1("ALL MEMORY TESTS PASSED!\r\n");
        // Success indication
        for (uint8_t i = 0; i < 5; i++)
        {
            PORTB = 0x00; // All LEDs on
            _delay_ms(200);
            PORTB = 0xFF; // All LEDs off
            _delay_ms(200);
        }
    }
    else
    {
        puts_USART1("SOME MEMORY TESTS FAILED!\r\n");
        // Failure indication
        for (uint8_t i = 0; i < 3; i++)
        {
            PORTB = 0x55; // Alternating pattern
            _delay_ms(300);
            PORTB = 0xAA;
            _delay_ms(300);
        }
        PORTB = 0xFF;
    }
}

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize EEPROM system
    EEPROM_init(); // Configure EEPROM interface

    // Initialize UART for memory test reports
    Uart1_init();
    puts_USART1("Memory Test Demo Started\r\n");
    puts_USART1("Testing SRAM and EEPROM memory systems\r\n");
    puts_USART1("Educational Memory Validation Laboratory\r\n");

    // Configure LED output for test status indication
    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    uint8_t test_cycle = 0;

    while (1)
    {
        test_cycle++;

        char buffer[50];
        sprintf(buffer, "\r\n>>> Memory Test Cycle #%u <<<\r\n", test_cycle);
        puts_USART1(buffer);

        // Run comprehensive memory test suite
        run_memory_test_suite();

        // Wait before next test cycle
        puts_USART1("\r\nWaiting 10 seconds before next test cycle...\r\n");

        // Countdown with LED indication
        for (uint8_t countdown = 10; countdown > 0; countdown--)
        {
            PORTB = ~(1 << (countdown % 8)); // Rotating LED indicator

            char countdown_buffer[30];
            sprintf(countdown_buffer, "Next test in %u seconds\r\n", countdown);
            puts_USART1(countdown_buffer);

            _delay_ms(1000);
        }

        PORTB = 0xFF; // Turn off LEDs
    }

    return 0;
}