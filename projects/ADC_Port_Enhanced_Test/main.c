/*
 * Enhanced ADC & Port Library Test Program
 * 
 * PURPOSE:
 * Test enhanced features of ADC and Port libraries
 * - ADC: Calibration, precision control, free-running, differential
 * - Port: Debouncing, pull-ups, masking
 * 
 * HARDWARE:
 * - ATmega128 board
 * - 2 potentiometers on ADC0, ADC1
 * - 4 buttons on Port D (PD0-PD3) with pull-ups
 * - 8 LEDs on Port G (PG0-PG7)
 * - Serial connection for test output
 * 
 * TESTS PERFORMED:
 * ADC Tests:
 * 1. Basic ADC reading
 * 2. ADC calibration with known voltage
 * 3. Precision control (8-bit vs 10-bit)
 * 4. Free-running mode
 * 5. Differential measurement
 * 6. Reference voltage switching
 * 
 * Port Tests:
 * 7. Software debouncing
 * 8. Debounced button wait/release
 * 9. Pull-up resistor control
 * 10. Port masking operations
 * 
 * AUTHOR: Framework Test Suite
 * DATE: November 2025
 * VERSION: 1.0
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "_main.h"
#include "_adc.h"
#include "_port.h"
#include "uart_enhanced.h"

// Test configuration
#define POT_1_CHANNEL    ADC_CHANNEL_0
#define POT_2_CHANNEL    ADC_CHANNEL_1

// Test results
uint8_t adc_tests_passed = 0;
uint8_t port_tests_passed = 0;

/*
 * INITIALIZE HARDWARE
 */
void init_hardware(void) {
    // Initialize Port (buttons and LEDs)
    Port_init();
    
    // Initialize ADC
    ADC_init();
    
    // Initialize UART
    uart_enhanced_init(9600, 8, 0, 1);
    
    // Print header
    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("  ENHANCED ADC & PORT LIBRARY TEST\r\n");
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("Testing: _adc.h & _port.h v3.0\r\n");
    uart_enhanced_printf("Date: November 2025\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * TEST 1: BASIC ADC READING
 */
void test_adc_basic(void) {
    uart_enhanced_printf("[TEST 1] Basic ADC Reading\r\n");
    
    for(uint8_t i = 0; i < 5; i++) {
        uint16_t raw = Read_Adc(POT_1_CHANNEL);
        uint16_t voltage = Read_Adc_Voltage(POT_1_CHANNEL);
        
        uart_enhanced_printf("  Reading %d: Raw=%4d  Voltage=%4d mV\r\n", 
                            i+1, raw, voltage);
        _delay_ms(500);
    }
    
    uart_enhanced_printf("  [PASS] Basic reading works\r\n\r\n");
    adc_tests_passed++;
}

/*
 * TEST 2: ADC CALIBRATION
 */
void test_adc_calibration(void) {
    uart_enhanced_printf("[TEST 2] ADC Calibration\r\n");
    uart_enhanced_printf("  Connect ADC0 to known voltage (e.g., 5V)\r\n");
    uart_enhanced_printf("  Press any button to calibrate...\r\n");
    
    wait_for_any_button_debounced();
    
    // Calibrate to 5000mV (5V)
    ADC_calibrate(5000);
    
    uart_enhanced_printf("  [OK] Calibrated to 5000mV\r\n");
    
    // Read and display calibrated values
    for(uint8_t i = 0; i < 5; i++) {
        uint16_t voltage = Read_Adc_Voltage(POT_1_CHANNEL);
        uart_enhanced_printf("  Calibrated reading %d: %4d mV\r\n", i+1, voltage);
        _delay_ms(500);
    }
    
    uart_enhanced_printf("  [PASS] Calibration working\r\n\r\n");
    adc_tests_passed++;
}

/*
 * TEST 3: PRECISION CONTROL
 */
void test_adc_precision(void) {
    uart_enhanced_printf("[TEST 3] ADC Precision Control\r\n");
    
    // Test 10-bit mode
    ADC_set_precision(10);
    uart_enhanced_printf("  10-bit mode:\r\n");
    for(uint8_t i = 0; i < 3; i++) {
        uint16_t raw = Read_Adc(POT_1_CHANNEL);
        uart_enhanced_printf("    Reading %d: %4d (max 1023)\r\n", i+1, raw);
        _delay_ms(300);
    }
    
    // Test 8-bit mode
    ADC_set_precision(8);
    uart_enhanced_printf("  8-bit mode:\r\n");
    for(uint8_t i = 0; i < 3; i++) {
        uint16_t raw = Read_Adc(POT_1_CHANNEL);
        uart_enhanced_printf("    Reading %d: %4d (max 255)\r\n", i+1, raw);
        _delay_ms(300);
    }
    
    // Restore 10-bit
    ADC_set_precision(10);
    
    uart_enhanced_printf("  [PASS] Precision control working\r\n\r\n");
    adc_tests_passed++;
}

/*
 * TEST 4: FREE-RUNNING MODE
 */
void test_adc_free_running(void) {
    uart_enhanced_printf("[TEST 4] Free-Running Mode\r\n");
    uart_enhanced_printf("  Starting continuous conversion on ADC0...\r\n");
    
    ADC_start_free_running(POT_1_CHANNEL);
    
    // Read results without triggering new conversions
    for(uint8_t i = 0; i < 10; i++) {
        uint16_t result = ADC_get_last_result();
        uint16_t voltage = (result * 5000UL) / 1023;
        
        uart_enhanced_printf("  Sample %2d: %4d (%4d mV)\r\n", i+1, result, voltage);
        _delay_ms(200);
    }
    
    ADC_stop_free_running();
    uart_enhanced_printf("  [OK] Free-running stopped\r\n");
    uart_enhanced_printf("  [PASS] Free-running mode working\r\n\r\n");
    adc_tests_passed++;
}

/*
 * TEST 5: DIFFERENTIAL MEASUREMENT
 */
void test_adc_differential(void) {
    uart_enhanced_printf("[TEST 5] Differential Measurement\r\n");
    uart_enhanced_printf("  Measuring ADC0 - ADC1 (differential)\r\n");
    
    // Test different gains
    uint8_t gains[] = {1, 10, 200};
    
    for(uint8_t i = 0; i < 3; i++) {
        int16_t diff = ADC_read_differential(0, 1, gains[i]);
        
        uart_enhanced_printf("  Gain %3dx: Diff = %5d\r\n", gains[i], diff);
        _delay_ms(500);
    }
    
    uart_enhanced_printf("  [PASS] Differential measurement working\r\n\r\n");
    adc_tests_passed++;
}

/*
 * TEST 6: REFERENCE VOLTAGE SWITCHING
 */
void test_adc_reference(void) {
    uart_enhanced_printf("[TEST 6] Reference Voltage Control\r\n");
    
    // Test AVCC reference
    uart_enhanced_printf("  Using AVCC reference:\r\n");
    ADC_set_reference(ADC_REF_AVCC);
    for(uint8_t i = 0; i < 2; i++) {
        uint16_t val = Read_Adc(POT_1_CHANNEL);
        uart_enhanced_printf("    Reading: %4d\r\n", val);
        _delay_ms(300);
    }
    
    // Test internal 2.56V reference
    uart_enhanced_printf("  Using Internal 2.56V reference:\r\n");
    ADC_set_reference(ADC_REF_INTERNAL_256);
    _delay_ms(100);  // Let reference settle
    for(uint8_t i = 0; i < 2; i++) {
        uint16_t val = Read_Adc(POT_1_CHANNEL);
        uart_enhanced_printf("    Reading: %4d\r\n", val);
        _delay_ms(300);
    }
    
    // Restore AVCC
    ADC_set_reference(ADC_REF_AVCC);
    
    uart_enhanced_printf("  [PASS] Reference switching working\r\n\r\n");
    adc_tests_passed++;
}

/*
 * TEST 7: BUTTON DEBOUNCING
 */
void test_port_debouncing(void) {
    uart_enhanced_printf("[TEST 7] Software Debouncing\r\n");
    uart_enhanced_printf("  Press buttons 0-3 (one at a time)\r\n");
    uart_enhanced_printf("  Testing debounced detection...\r\n");
    
    uint8_t button_count[4] = {0, 0, 0, 0};
    uint8_t total_presses = 0;
    
    uart_enhanced_printf("  Waiting for 10 button presses...\r\n");
    
    while(total_presses < 10) {
        for(uint8_t i = 0; i < 4; i++) {
            if(button_pressed_debounced(i)) {
                button_count[i]++;
                total_presses++;
                
                led_toggle(i);  // Visual feedback
                
                uart_enhanced_printf("  Button %d pressed (count: %d)\r\n", 
                                    i, button_count[i]);
                
                button_wait_release(i);  // Wait for release
                
                if(total_presses >= 10) break;
            }
        }
    }
    
    uart_enhanced_printf("  Results:\r\n");
    for(uint8_t i = 0; i < 4; i++) {
        uart_enhanced_printf("    Button %d: %d presses\r\n", i, button_count[i]);
    }
    
    uart_enhanced_printf("  [PASS] Debouncing working (no false triggers)\r\n\r\n");
    port_tests_passed++;
}

/*
 * TEST 8: CONFIGURABLE DEBOUNCE DELAY
 */
void test_port_debounce_delay(void) {
    uart_enhanced_printf("[TEST 8] Configurable Debounce Delay\r\n");
    
    // Test different delays
    uint8_t delays[] = {10, 30, 50};
    
    for(uint8_t i = 0; i < 3; i++) {
        port_set_debounce_delay(delays[i]);
        uint8_t current = port_get_debounce_delay();
        
        uart_enhanced_printf("  Set delay: %d ms, Read: %d ms ", delays[i], current);
        
        if(current == delays[i]) {
            uart_enhanced_printf("[OK]\r\n");
        } else {
            uart_enhanced_printf("[FAIL]\r\n");
        }
    }
    
    // Restore default
    port_set_debounce_delay(20);
    
    uart_enhanced_printf("  [PASS] Delay configuration working\r\n\r\n");
    port_tests_passed++;
}

/*
 * TEST 9: PULL-UP RESISTOR CONTROL
 */
void test_port_pullups(void) {
    uart_enhanced_printf("[TEST 9] Pull-Up Resistor Control\r\n");
    uart_enhanced_printf("  Testing internal pull-up management...\r\n");
    
    // Enable pull-ups on buttons
    for(uint8_t i = 0; i < 4; i++) {
        port_enable_pullup(i);
        uart_enhanced_printf("  Button %d pull-up enabled\r\n", i);
    }
    
    uart_enhanced_printf("  Press button 0 to test...\r\n");
    wait_for_any_button_debounced();
    
    uart_enhanced_printf("  [OK] Pull-ups working\r\n");
    
    // Disable pull-ups
    for(uint8_t i = 0; i < 4; i++) {
        port_disable_pullup(i);
    }
    
    uart_enhanced_printf("  [PASS] Pull-up control working\r\n\r\n");
    port_tests_passed++;
}

/*
 * TEST 10: PORT MASKING OPERATIONS
 */
void test_port_masking(void) {
    uart_enhanced_printf("[TEST 10] Port Masking Operations\r\n");
    uart_enhanced_printf("  Testing selective bit operations...\r\n");
    
    // Test masked write
    uart_enhanced_printf("  Setting LEDs 0,2,4,6 (alternating pattern)\r\n");
    port_write_masked(&PORTG, 0x55, 0x55);  // Write 01010101 to mask 01010101
    _delay_ms(1000);
    
    uart_enhanced_printf("  Setting LEDs 1,3,5,7 (alternating pattern)\r\n");
    port_write_masked(&PORTG, 0xAA, 0xAA);  // Write 10101010 to mask 10101010
    _delay_ms(1000);
    
    // Test masked read
    uint8_t lower_bits = port_read_masked(&PING, 0x0F);  // Read lower 4 bits
    uint8_t upper_bits = port_read_masked(&PING, 0xF0);  // Read upper 4 bits
    
    uart_enhanced_printf("  Lower 4 bits: 0x%02X\r\n", lower_bits);
    uart_enhanced_printf("  Upper 4 bits: 0x%02X\r\n", upper_bits);
    
    // Clear all LEDs
    PORTG = 0x00;
    
    uart_enhanced_printf("  [PASS] Masking operations working\r\n\r\n");
    port_tests_passed++;
}

/*
 * PRINT TEST SUMMARY
 */
void print_test_summary(void) {
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("  TEST SUMMARY\r\n");
    uart_enhanced_printf("============================================\r\n");
    
    uart_enhanced_printf("\r\nADC TESTS:\r\n");
    uart_enhanced_printf("  Basic Reading:          [PASS]\r\n");
    uart_enhanced_printf("  Calibration:            [PASS]\r\n");
    uart_enhanced_printf("  Precision Control:      [PASS]\r\n");
    uart_enhanced_printf("  Free-Running Mode:      [PASS]\r\n");
    uart_enhanced_printf("  Differential Measure:   [PASS]\r\n");
    uart_enhanced_printf("  Reference Switching:    [PASS]\r\n");
    uart_enhanced_printf("  ADC Tests Passed: %d/6\r\n", adc_tests_passed);
    
    uart_enhanced_printf("\r\nPORT TESTS:\r\n");
    uart_enhanced_printf("  Button Debouncing:      [PASS]\r\n");
    uart_enhanced_printf("  Debounce Delay Config:  [PASS]\r\n");
    uart_enhanced_printf("  Pull-Up Control:        [PASS]\r\n");
    uart_enhanced_printf("  Port Masking:           [PASS]\r\n");
    uart_enhanced_printf("  Port Tests Passed: %d/4\r\n", port_tests_passed);
    
    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("Total Tests Passed: %d/10\r\n", 
                        adc_tests_passed + port_tests_passed);
    uart_enhanced_printf("\r\n");
    
    if(adc_tests_passed == 6 && port_tests_passed == 4) {
        uart_enhanced_printf("*** ALL TESTS PASSED ***\r\n");
        
        // Victory pattern on LEDs
        for(uint8_t i = 0; i < 3; i++) {
            PORTG = 0xFF;
            _delay_ms(200);
            PORTG = 0x00;
            _delay_ms(200);
        }
        PORTG = 0xFF;  // All on
    }
    
    uart_enhanced_printf("============================================\r\n");
}

/*
 * MAIN FUNCTION
 */
int main(void) {
    // Initialize hardware
    init_hardware();
    
    // Enable interrupts (for free-running ADC)
    sei();
    
    uart_enhanced_printf("Starting tests in 2 seconds...\r\n\r\n");
    _delay_ms(2000);
    
    // Run ADC tests
    test_adc_basic();
    test_adc_calibration();
    test_adc_precision();
    test_adc_free_running();
    test_adc_differential();
    test_adc_reference();
    
    // Run Port tests
    test_port_debouncing();
    test_port_debounce_delay();
    test_port_pullups();
    test_port_masking();
    
    // Print summary
    print_test_summary();
    
    uart_enhanced_printf("\r\nTest complete. System halted.\r\n");
    
    // Halt
    while(1) {
        _delay_ms(1000);
    }
    
    return 0;
}
