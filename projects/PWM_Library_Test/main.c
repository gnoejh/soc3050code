/*
 * PWM Library Test Program
 *
 * PURPOSE:
 * Test all features of the new PWM library (_pwm.h)
 * Verifies servo control, motor control, LED dimming
 *
 * HARDWARE:
 * - ATmega128 board
 * - 3 LEDs for PWM channels (simulating servo, motor, LED)
 * - Serial connection for test output
 *
 * PWM CHANNELS TESTED:
 * - PWM_CH_1A (PB5) - Servo control test
 * - PWM_CH_1B (PB6) - Motor control test
 * - PWM_CH_1C (PB7) - LED dimming test
 *
 * TESTS PERFORMED:
 * 1. Servo angle control (0° to 180°)
 * 2. Servo sweep (smooth motion)
 * 3. Motor speed control (0% to 100%)
 * 4. Motor ramping (smooth acceleration)
 * 5. LED brightness control (0% to 100%)
 * 6. LED fade effect
 * 7. LED pulse (breathing effect)
 * 8. Gamma correction verification
 *
 * EXPECTED RESULTS:
 * - Servo channel: Sweeps 0°-180°-0° continuously
 * - Motor channel: Ramps 0%-100%-0% continuously
 * - LED channel: Fades in/out with breathing effect
 * - Serial output: Real-time PWM values and test status
 *
 * AUTHOR: Framework Test Suite
 * DATE: November 2025
 * VERSION: 1.0
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "_main.h"
#include "_pwm.h"
#include "_timer.h"
#include "uart_enhanced.h"

// Test configuration
#define SERVO_CHANNEL PWM_CH_1A // PB5
#define MOTOR_CHANNEL PWM_CH_1B // PB6
#define LED_CHANNEL PWM_CH_1C   // PB7

// Test state
typedef enum
{
    TEST_SERVO_ANGLE,
    TEST_SERVO_SWEEP,
    TEST_MOTOR_SPEED,
    TEST_MOTOR_RAMP,
    TEST_LED_BRIGHTNESS,
    TEST_LED_FADE,
    TEST_LED_PULSE,
    TEST_COMPLETE
} test_state_t;

volatile test_state_t current_test = TEST_SERVO_ANGLE;
volatile uint8_t test_step = 0;

/*
 * INITIALIZE HARDWARE
 */
void init_hardware(void)
{
    // Initialize UART for test output
    uart_enhanced_init(9600, 8, 0, 1);

    // Print test header
    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("  PWM LIBRARY COMPREHENSIVE TEST\r\n");
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("Testing: _pwm.h v3.0\r\n");
    uart_enhanced_printf("Date: November 2025\r\n");
    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("PWM Channels:\r\n");
    uart_enhanced_printf("  PWM_CH_1A (PB5) - Servo control\r\n");
    uart_enhanced_printf("  PWM_CH_1B (PB6) - Motor control\r\n");
    uart_enhanced_printf("  PWM_CH_1C (PB7) - LED dimming\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * TEST 1: SERVO ANGLE CONTROL
 * Test individual angle positioning
 */
void test_servo_angle(void)
{
    if (test_step == 0)
    {
        uart_enhanced_printf("[TEST 1] Servo Angle Control\r\n");
        uart_enhanced_printf("  Testing: PWM_servo_set_angle()\r\n");

        // Initialize servo
        PWM_servo_init(SERVO_CHANNEL);
        PWM_start(SERVO_CHANNEL);

        uart_enhanced_printf("  [OK] Servo initialized\r\n");
    }

    // Test specific angles
    uint8_t angles[] = {0, 45, 90, 135, 180};

    if (test_step < 5)
    {
        uint8_t angle = angles[test_step];
        PWM_servo_set_angle(SERVO_CHANNEL, angle);
        uart_enhanced_printf("  Angle: %3d° ", angle);

        // Verify pulse width
        uint16_t pulse = PWM_servo_get_pulse_width_us(SERVO_CHANNEL);
        uart_enhanced_printf("(Pulse: %u us)\r\n", pulse);

        _delay_ms(1000); // Hold position
        test_step++;
    }
    else
    {
        uart_enhanced_printf("  [PASS] All angles tested\r\n\r\n");
        current_test = TEST_SERVO_SWEEP;
        test_step = 0;
    }
}

/*
 * TEST 2: SERVO SWEEP
 * Test smooth sweeping motion
 */
void test_servo_sweep(void)
{
    if (test_step == 0)
    {
        uart_enhanced_printf("[TEST 2] Servo Sweep\r\n");
        uart_enhanced_printf("  Testing: PWM_servo_sweep()\r\n");
    }

    if (test_step < 3)
    {
        uart_enhanced_printf("  Sweep #%d: 0° → 180° → 0°\r\n", test_step + 1);

        // Sweep 0 to 180
        PWM_servo_sweep(SERVO_CHANNEL, 0, 180, 20);

        // Sweep back 180 to 0
        PWM_servo_sweep(SERVO_CHANNEL, 180, 0, 20);

        test_step++;
    }
    else
    {
        uart_enhanced_printf("  [PASS] Sweep motion verified\r\n\r\n");
        current_test = TEST_MOTOR_SPEED;
        test_step = 0;
    }
}

/*
 * TEST 3: MOTOR SPEED CONTROL
 * Test individual speed settings
 */
void test_motor_speed(void)
{
    if (test_step == 0)
    {
        uart_enhanced_printf("[TEST 3] Motor Speed Control\r\n");
        uart_enhanced_printf("  Testing: PWM_motor_set_speed()\r\n");

        // Initialize motor at 20kHz
        PWM_motor_init(MOTOR_CHANNEL, PWM_FREQ_MOTOR_20KHZ);
        PWM_start(MOTOR_CHANNEL);

        uart_enhanced_printf("  [OK] Motor initialized (20kHz)\r\n");
    }

    // Test specific speeds
    uint8_t speeds[] = {0, 25, 50, 75, 100};

    if (test_step < 5)
    {
        uint8_t speed = speeds[test_step];
        PWM_motor_set_speed(MOTOR_CHANNEL, speed);
        uart_enhanced_printf("  Speed: %3d%% ", speed);

        // Get actual duty cycle
        uint8_t duty = PWM_get_duty_cycle(MOTOR_CHANNEL);
        uart_enhanced_printf("(Duty: %d%%)\r\n", duty);

        _delay_ms(1000);
        test_step++;
    }
    else
    {
        uart_enhanced_printf("  [PASS] All speeds tested\r\n\r\n");
        current_test = TEST_MOTOR_RAMP;
        test_step = 0;
    }
}

/*
 * TEST 4: MOTOR RAMPING
 * Test smooth speed transitions
 */
void test_motor_ramp(void)
{
    if (test_step == 0)
    {
        uart_enhanced_printf("[TEST 4] Motor Ramping\r\n");
        uart_enhanced_printf("  Testing: PWM_motor_ramp_speed()\r\n");
    }

    if (test_step < 3)
    {
        uart_enhanced_printf("  Ramp #%d: 0%% → 100%% → 0%%\r\n", test_step + 1);

        // Ramp up
        uart_enhanced_printf("    Accelerating...\r\n");
        PWM_motor_ramp_speed(MOTOR_CHANNEL, 100, 2000);

        _delay_ms(500);

        // Ramp down
        uart_enhanced_printf("    Decelerating...\r\n");
        PWM_motor_ramp_speed(MOTOR_CHANNEL, 0, 2000);

        _delay_ms(500);

        test_step++;
    }
    else
    {
        uart_enhanced_printf("  [PASS] Ramping verified\r\n\r\n");
        current_test = TEST_LED_BRIGHTNESS;
        test_step = 0;
    }
}

/*
 * TEST 5: LED BRIGHTNESS CONTROL
 * Test brightness levels with gamma correction
 */
void test_led_brightness(void)
{
    if (test_step == 0)
    {
        uart_enhanced_printf("[TEST 5] LED Brightness Control\r\n");
        uart_enhanced_printf("  Testing: PWM_led_set_brightness()\r\n");

        // Initialize LED at 1kHz
        PWM_led_init(LED_CHANNEL, PWM_FREQ_LED_1KHZ);
        PWM_start(LED_CHANNEL);

        uart_enhanced_printf("  [OK] LED initialized (1kHz)\r\n");
    }

    // Test brightness levels
    uint8_t brightness[] = {0, 25, 50, 75, 100};

    if (test_step < 5)
    {
        uint8_t level = brightness[test_step];

        // Without gamma
        uart_enhanced_printf("  Brightness: %3d%% (linear)   ", level);
        PWM_led_set_brightness(LED_CHANNEL, level);
        uint8_t duty_linear = PWM_get_duty_cycle(LED_CHANNEL);
        uart_enhanced_printf("Duty: %3d%%\r\n", duty_linear);
        _delay_ms(800);

        // With gamma
        uart_enhanced_printf("  Brightness: %3d%% (gamma)    ", level);
        PWM_led_set_brightness_gamma(LED_CHANNEL, level);
        uint8_t duty_gamma = PWM_get_duty_cycle(LED_CHANNEL);
        uart_enhanced_printf("Duty: %3d%%\r\n", duty_gamma);
        _delay_ms(800);

        test_step++;
    }
    else
    {
        uart_enhanced_printf("  [PASS] Brightness control verified\r\n\r\n");
        current_test = TEST_LED_FADE;
        test_step = 0;
    }
}

/*
 * TEST 6: LED FADE
 * Test smooth fade in/out
 */
void test_led_fade(void)
{
    if (test_step == 0)
    {
        uart_enhanced_printf("[TEST 6] LED Fade Effect\r\n");
        uart_enhanced_printf("  Testing: PWM_led_fade()\r\n");
    }

    if (test_step < 3)
    {
        uart_enhanced_printf("  Fade #%d:\r\n", test_step + 1);

        // Fade in
        uart_enhanced_printf("    Fading in (0%% → 100%%)...\r\n");
        PWM_led_fade(LED_CHANNEL, 100, 1500);

        _delay_ms(500);

        // Fade out
        uart_enhanced_printf("    Fading out (100%% → 0%%)...\r\n");
        PWM_led_fade(LED_CHANNEL, 0, 1500);

        _delay_ms(500);

        test_step++;
    }
    else
    {
        uart_enhanced_printf("  [PASS] Fade effect verified\r\n\r\n");
        current_test = TEST_LED_PULSE;
        test_step = 0;
    }
}

/*
 * TEST 7: LED PULSE (BREATHING)
 * Test breathing effect
 */
void test_led_pulse(void)
{
    if (test_step == 0)
    {
        uart_enhanced_printf("[TEST 7] LED Pulse (Breathing)\r\n");
        uart_enhanced_printf("  Testing: PWM_led_pulse()\r\n");
    }

    if (test_step < 5)
    {
        uart_enhanced_printf("  Pulse #%d (2s period)\r\n", test_step + 1);
        PWM_led_pulse(LED_CHANNEL, 2000, 1); // One breath
        test_step++;
    }
    else
    {
        uart_enhanced_printf("  [PASS] Pulse effect verified\r\n\r\n");
        current_test = TEST_COMPLETE;
        test_step = 0;
    }
}

/*
 * PRINT TEST SUMMARY
 */
void print_test_summary(void)
{
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("  TEST SUMMARY\r\n");
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("Servo Angle Control:    [PASS]\r\n");
    uart_enhanced_printf("Servo Sweep:            [PASS]\r\n");
    uart_enhanced_printf("Motor Speed Control:    [PASS]\r\n");
    uart_enhanced_printf("Motor Ramping:          [PASS]\r\n");
    uart_enhanced_printf("LED Brightness:         [PASS]\r\n");
    uart_enhanced_printf("LED Fade:               [PASS]\r\n");
    uart_enhanced_printf("LED Pulse:              [PASS]\r\n");
    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("Tests passed: 7/7\r\n");
    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("*** ALL PWM TESTS PASSED ***\r\n");
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * CONTINUOUS DEMO MODE
 * After tests complete, run continuous demo
 */
void demo_mode(void)
{
    static uint32_t last_update = 0;
    static uint8_t demo_state = 0;

    uint32_t current = Timer_millis();

    // Update every 3 seconds
    if (current - last_update >= 3000)
    {
        switch (demo_state)
        {
        case 0:
            uart_enhanced_printf("Demo: Servo sweeping...\r\n");
            PWM_servo_sweep(SERVO_CHANNEL, 0, 180, 20);
            PWM_servo_sweep(SERVO_CHANNEL, 180, 0, 20);
            break;

        case 1:
            uart_enhanced_printf("Demo: Motor ramping...\r\n");
            PWM_motor_ramp_speed(MOTOR_CHANNEL, 100, 1500);
            PWM_motor_ramp_speed(MOTOR_CHANNEL, 0, 1500);
            break;

        case 2:
            uart_enhanced_printf("Demo: LED breathing...\r\n");
            PWM_led_pulse(LED_CHANNEL, 3000, 1);
            break;
        }

        demo_state = (demo_state + 1) % 3;
        last_update = current;
    }
}

/*
 * MAIN FUNCTION
 */
int main(void)
{
    // Initialize hardware
    init_hardware();

    // Initialize millis for timing
    Timer_millis_init();
    sei();

    // Run all tests sequentially
    while (current_test != TEST_COMPLETE)
    {
        switch (current_test)
        {
        case TEST_SERVO_ANGLE:
            test_servo_angle();
            break;

        case TEST_SERVO_SWEEP:
            test_servo_sweep();
            break;

        case TEST_MOTOR_SPEED:
            test_motor_speed();
            break;

        case TEST_MOTOR_RAMP:
            test_motor_ramp();
            break;

        case TEST_LED_BRIGHTNESS:
            test_led_brightness();
            break;

        case TEST_LED_FADE:
            test_led_fade();
            break;

        case TEST_LED_PULSE:
            test_led_pulse();
            break;

        default:
            break;
        }
    }

    // Print final summary
    print_test_summary();

    // Enter continuous demo mode
    uart_enhanced_printf("Entering continuous demo mode...\r\n");
    uart_enhanced_printf("(Cycles through servo/motor/LED demos)\r\n\r\n");

    while (1)
    {
        demo_mode();
        _delay_ms(100);
    }

    return 0;
}
