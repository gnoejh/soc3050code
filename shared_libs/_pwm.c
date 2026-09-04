/*
 * =============================================================================
 * EDUCATIONAL ATmega128 PWM LIBRARY - IMPLEMENTATION FILE
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim
 *
 * Professional-quality PWM implementation for servo control, motor control,
 * LED dimming, and general PWM signal generation.
 *
 * =============================================================================
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "_pwm.h"
#include "_timer.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/*
 * =============================================================================
 * INTERNAL STATE TRACKING
 * =============================================================================
 */

/* PWM channel configuration state */
typedef struct
{
    uint8_t initialized;
    uint8_t active;
    uint32_t frequency_hz;
    pwm_mode_t mode;
    timer_id_t timer;
    timer_compare_register_t compare_reg;
    uint16_t top_value; // ICR or OCR for 16-bit timers
    uint8_t current_duty;
} pwm_state_t;

static pwm_state_t pwm_states[8] = {0};

/* Gamma correction table for LED brightness (8-bit to 8-bit) */
static const uint8_t gamma8[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
    2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
    5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
    10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
    17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
    25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
    37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
    51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
    69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
    90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
    115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
    144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
    177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
    215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255};

/*
 * =============================================================================
 * HELPER FUNCTIONS
 * =============================================================================
 */

/* Map PWM channel to timer and compare register */
static void pwm_get_timer_info(pwm_channel_t channel, timer_id_t *timer, timer_compare_register_t *compare_reg)
{
    switch (channel)
    {
    case PWM_CH_0:
        *timer = TIMER_0;
        *compare_reg = TIMER_COMPARE_A;
        break;
    case PWM_CH_1A:
        *timer = TIMER_1;
        *compare_reg = TIMER_COMPARE_A;
        break;
    case PWM_CH_1B:
        *timer = TIMER_1;
        *compare_reg = TIMER_COMPARE_B;
        break;
    case PWM_CH_1C:
        *timer = TIMER_1;
        *compare_reg = TIMER_COMPARE_C;
        break;
    case PWM_CH_2:
        *timer = TIMER_2;
        *compare_reg = TIMER_COMPARE_A;
        break;
    case PWM_CH_3A:
        *timer = TIMER_3;
        *compare_reg = TIMER_COMPARE_A;
        break;
    case PWM_CH_3B:
        *timer = TIMER_3;
        *compare_reg = TIMER_COMPARE_B;
        break;
    case PWM_CH_3C:
        *timer = TIMER_3;
        *compare_reg = TIMER_COMPARE_C;
        break;
    }
}

/* Configure output pin for PWM channel */
static void pwm_configure_pin(pwm_channel_t channel)
{
    switch (channel)
    {
    case PWM_CH_0:
        DDRB |= (1 << PB4); // OC0
        break;
    case PWM_CH_1A:
        DDRB |= (1 << PB5); // OC1A
        break;
    case PWM_CH_1B:
        DDRB |= (1 << PB6); // OC1B
        break;
    case PWM_CH_1C:
        DDRB |= (1 << PB7); // OC1C
        break;
    case PWM_CH_2:
        DDRB |= (1 << PB7); // OC2 (conflicts with OC1C)
        break;
    case PWM_CH_3A:
        DDRE |= (1 << PE3); // OC3A
        break;
    case PWM_CH_3B:
        DDRE |= (1 << PE4); // OC3B
        break;
    case PWM_CH_3C:
        DDRE |= (1 << PE5); // OC3C
        break;
    }
}

/*
 * =============================================================================
 * CORE PWM FUNCTIONS
 * =============================================================================
 */

uint8_t PWM_init(pwm_channel_t channel, uint32_t frequency_hz, pwm_mode_t mode)
{
    if (channel > PWM_CH_3C)
        return 0;

    timer_id_t timer;
    timer_compare_register_t compare_reg;
    pwm_get_timer_info(channel, &timer, &compare_reg);

    // Configure output pin
    pwm_configure_pin(channel);

    // Initialize timer in appropriate PWM mode
    timer_mode_t timer_mode = (mode == PWM_MODE_FAST) ? TIMER_MODE_FAST_PWM : TIMER_MODE_PHASE_CORRECT_PWM;

    // Calculate prescaler (simplified - use 64 for most cases)
    uint16_t prescaler = TIMER_PRESCALER_64;

    // Initialize timer
    Timer_init(timer, timer_mode, prescaler);

    // For 16-bit timers, set TOP value for frequency control
    if (timer == TIMER_1 || timer == TIMER_3)
    {
        uint32_t timer_freq = F_CPU / 64; // With prescaler 64
        uint16_t top = (timer_freq / frequency_hz) - 1;

        if (timer == TIMER_1)
        {
            ICR1 = top;
            pwm_states[channel].top_value = top;
        }
        else if (timer == TIMER_3)
        {
            ICR3 = top;
            pwm_states[channel].top_value = top;
        }
    }

    // Store configuration
    pwm_states[channel].initialized = 1;
    pwm_states[channel].active = 0;
    pwm_states[channel].frequency_hz = frequency_hz;
    pwm_states[channel].mode = mode;
    pwm_states[channel].timer = timer;
    pwm_states[channel].compare_reg = compare_reg;
    pwm_states[channel].current_duty = 0;

    return 1;
}

void PWM_start(pwm_channel_t channel)
{
    if (channel > PWM_CH_3C || !pwm_states[channel].initialized)
        return;

    timer_id_t timer = pwm_states[channel].timer;
    timer_compare_register_t compare_reg = pwm_states[channel].compare_reg;

    // Set compare output mode to non-inverting
    Timer_set_compare_output_mode(timer, compare_reg, TIMER_COM_CLEAR);

    pwm_states[channel].active = 1;
}

void PWM_stop(pwm_channel_t channel)
{
    if (channel > PWM_CH_3C)
        return;

    timer_id_t timer = pwm_states[channel].timer;
    timer_compare_register_t compare_reg = pwm_states[channel].compare_reg;

    // Disconnect OC pin
    Timer_set_compare_output_mode(timer, compare_reg, TIMER_COM_DISCONNECT);

    pwm_states[channel].active = 0;
}

void PWM_set_duty_cycle(pwm_channel_t channel, uint8_t duty_percent)
{
    if (channel > PWM_CH_3C || !pwm_states[channel].initialized)
        return;
    if (duty_percent > 100)
        duty_percent = 100;

    timer_id_t timer = pwm_states[channel].timer;
    timer_compare_register_t compare_reg = pwm_states[channel].compare_reg;
    uint16_t duty_value;

    // Calculate duty value based on timer type
    if (timer == TIMER_0 || timer == TIMER_2)
    {
        // 8-bit timer
        duty_value = (uint16_t)((255UL * duty_percent) / 100);
    }
    else
    {
        // 16-bit timer
        uint16_t top = pwm_states[channel].top_value;
        if (top == 0)
            top = 0xFFFF; // Default if not set
        duty_value = (uint32_t)((top * (uint32_t)duty_percent) / 100);
    }

    Timer_set_compare_value(timer, compare_reg, duty_value);
    pwm_states[channel].current_duty = duty_percent;
}

void PWM_set_duty_cycle_raw(pwm_channel_t channel, uint16_t duty_value)
{
    if (channel > PWM_CH_3C || !pwm_states[channel].initialized)
        return;

    timer_id_t timer = pwm_states[channel].timer;
    timer_compare_register_t compare_reg = pwm_states[channel].compare_reg;

    Timer_set_compare_value(timer, compare_reg, duty_value);
}

uint8_t PWM_get_duty_cycle(pwm_channel_t channel)
{
    if (channel > PWM_CH_3C)
        return 0;
    return pwm_states[channel].current_duty;
}

uint8_t PWM_set_frequency(pwm_channel_t channel, uint32_t frequency_hz)
{
    if (channel > PWM_CH_3C || !pwm_states[channel].initialized)
        return 0;

    // Re-initialize with new frequency
    pwm_mode_t mode = pwm_states[channel].mode;
    return PWM_init(channel, frequency_hz, mode);
}

uint32_t PWM_get_frequency(pwm_channel_t channel)
{
    if (channel > PWM_CH_3C)
        return 0;
    return pwm_states[channel].frequency_hz;
}

/*
 * =============================================================================
 * SERVO MOTOR CONTROL
 * =============================================================================
 */

uint8_t PWM_servo_init(pwm_channel_t channel)
{
    // Servo standard: 50Hz PWM
    return PWM_init(channel, PWM_FREQ_SERVO_50HZ, PWM_MODE_FAST);
}

void PWM_servo_set_angle(pwm_channel_t channel, uint8_t angle_degrees)
{
    if (angle_degrees > SERVO_ANGLE_MAX)
        angle_degrees = SERVO_ANGLE_MAX;

    // Map angle to pulse width: 0° = 544μs, 180° = 2400μs
    uint16_t pulse_us = SERVO_PULSE_MIN_US +
                        ((uint32_t)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * angle_degrees) / SERVO_ANGLE_MAX;

    PWM_servo_set_pulse_us(channel, pulse_us);
}

void PWM_servo_set_pulse_us(pwm_channel_t channel, uint16_t pulse_us)
{
    if (channel > PWM_CH_3C || !pwm_states[channel].initialized)
        return;

    // Calculate duty cycle from pulse width
    // Period = 20000μs (50Hz), pulse_us = 544-2400μs
    // Duty % = (pulse_us / 20000) * 100
    uint8_t duty = (uint32_t)(pulse_us * 100UL) / SERVO_PERIOD_US;

    // For 16-bit timer with better resolution
    timer_id_t timer = pwm_states[channel].timer;
    if (timer == TIMER_1 || timer == TIMER_3)
    {
        uint16_t top = pwm_states[channel].top_value;
        if (top == 0)
            top = (F_CPU / 64 / 50) - 1; // 50Hz with prescaler 64

        // Calculate OCR value for exact pulse width
        // OCR = (pulse_us * F_CPU) / (prescaler * 1000000)
        uint32_t ocr_value = ((uint32_t)pulse_us * (F_CPU / 64)) / 1000000UL;

        PWM_set_duty_cycle_raw(channel, (uint16_t)ocr_value);
    }
    else
    {
        PWM_set_duty_cycle(channel, duty);
    }

    pwm_states[channel].current_duty = duty;
}

void PWM_servo_sweep(pwm_channel_t channel, uint8_t start_angle, uint8_t end_angle, uint16_t step_delay_ms)
{
    if (start_angle <= end_angle)
    {
        // Forward sweep
        for (uint8_t angle = start_angle; angle <= end_angle; angle++)
        {
            PWM_servo_set_angle(channel, angle);
            // Variable delay loop (not compile-time constant safe)
            for (uint16_t i = 0; i < step_delay_ms; i++)
            {
                _delay_ms(1);
            }
        }
    }
    else
    {
        // Reverse sweep
        for (uint8_t angle = start_angle; angle >= end_angle; angle--)
        {
            PWM_servo_set_angle(channel, angle);
            // Variable delay loop
            for (uint16_t i = 0; i < step_delay_ms; i++)
            {
                _delay_ms(1);
            }
            if (angle == 0)
                break; // Prevent underflow
        }
    }
}

void PWM_servo_center(pwm_channel_t channel)
{
    PWM_servo_set_angle(channel, SERVO_ANGLE_CENTER);
}

/*
 * =============================================================================
 * DC MOTOR CONTROL
 * =============================================================================
 */

uint8_t PWM_motor_init(pwm_channel_t channel, uint32_t frequency_hz)
{
    return PWM_init(channel, frequency_hz, PWM_MODE_FAST);
}

void PWM_motor_set_speed(pwm_channel_t channel, uint8_t speed_percent)
{
    PWM_set_duty_cycle(channel, speed_percent);
}

void PWM_motor_ramp_speed(pwm_channel_t channel, uint8_t target_speed, uint16_t ramp_time_ms)
{
    uint8_t current_speed = PWM_get_duty_cycle(channel);
    uint16_t steps = ramp_time_ms / 10; // Update every 10ms
    if (steps == 0)
        steps = 1;

    int16_t speed_diff = target_speed - current_speed;
    int16_t step_size = speed_diff / steps;

    if (step_size == 0)
        step_size = (speed_diff > 0) ? 1 : -1;

    while (current_speed != target_speed)
    {
        current_speed += step_size;
        if ((step_size > 0 && current_speed > target_speed) ||
            (step_size < 0 && current_speed < target_speed))
        {
            current_speed = target_speed;
        }

        PWM_set_duty_cycle(channel, current_speed);
        _delay_ms(10);
    }
}

void PWM_motor_stop(pwm_channel_t channel)
{
    PWM_set_duty_cycle(channel, 0);
}

void PWM_motor_brake(pwm_channel_t channel)
{
    // Set duty to 100% (both terminals HIGH for brake)
    PWM_set_duty_cycle(channel, 100);
}

/*
 * =============================================================================
 * LED DIMMING
 * =============================================================================
 */

uint8_t PWM_led_init(pwm_channel_t channel, uint32_t frequency_hz)
{
    return PWM_init(channel, frequency_hz, PWM_MODE_FAST);
}

void PWM_led_set_brightness(pwm_channel_t channel, uint8_t brightness_percent)
{
    PWM_set_duty_cycle(channel, brightness_percent);
}

void PWM_led_set_brightness_gamma(pwm_channel_t channel, uint8_t brightness_percent)
{
    if (brightness_percent > 100)
        brightness_percent = 100;

    // Map 0-100% to gamma-corrected 0-255
    uint8_t index = (uint32_t)(brightness_percent * 255UL) / 100;
    uint8_t corrected = gamma8[index];

    // Map back to duty cycle (0-100%)
    uint8_t duty = (uint32_t)(corrected * 100UL) / 255;
    PWM_set_duty_cycle(channel, duty);
}

void PWM_led_fade(pwm_channel_t channel, uint8_t target_brightness, uint16_t fade_time_ms)
{
    // Similar to motor ramp
    PWM_motor_ramp_speed(channel, target_brightness, fade_time_ms);
}

void PWM_led_pulse(pwm_channel_t channel, uint16_t period_ms, uint16_t cycles)
{
    uint16_t half_period = period_ms / 2;
    uint16_t count = 0;

    while (cycles == 0 || count < cycles)
    {
        // Fade up
        PWM_led_fade(channel, 100, half_period);
        // Fade down
        PWM_led_fade(channel, 0, half_period);
        count++;
    }
}

/*
 * =============================================================================
 * DIAGNOSTIC FUNCTIONS
 * =============================================================================
 */

uint8_t PWM_get_resolution_bits(pwm_channel_t channel)
{
    if (channel > PWM_CH_3C)
        return 0;

    timer_id_t timer = pwm_states[channel].timer;
    return (timer == TIMER_0 || timer == TIMER_2) ? 8 : 16;
}

timer_id_t PWM_get_timer(pwm_channel_t channel)
{
    if (channel > PWM_CH_3C)
        return TIMER_0;
    return pwm_states[channel].timer;
}

uint8_t PWM_is_active(pwm_channel_t channel)
{
    if (channel > PWM_CH_3C)
        return 0;
    return pwm_states[channel].active;
}

/*
 * =============================================================================
 * ADVANCED FUNCTIONS (Stubs for future implementation)
 * =============================================================================
 */

void PWM_set_phase_shift(pwm_channel_t channel, uint16_t phase_degrees)
{
    // TODO: Implement phase shifting
    // Requires careful timer synchronization
}

uint8_t PWM_synchronize_channels(pwm_channel_t *channels, uint8_t num_channels, uint32_t frequency_hz)
{
    // TODO: Implement multi-channel synchronization
    // Initialize all channels with same frequency
    for (uint8_t i = 0; i < num_channels; i++)
    {
        if (!PWM_init(channels[i], frequency_hz, PWM_MODE_FAST))
        {
            return 0;
        }
    }
    return 1;
}

void PWM_generate_pulse(pwm_channel_t channel, uint16_t pulse_width_us)
{
    // TODO: Implement one-shot pulse generation
}

uint32_t PWM_calculate_max_frequency(pwm_channel_t channel, uint8_t resolution_bits)
{
    // Max frequency = F_CPU / (prescaler * 2^resolution)
    // With prescaler 1: F_CPU / 256 for 8-bit, F_CPU / 65536 for 16-bit
    if (resolution_bits == 8)
    {
        return F_CPU / 256;
    }
    else
    {
        return F_CPU / 65536;
    }
}

void PWM_print_config(pwm_channel_t channel)
{
    // TODO: Implement diagnostic printing
    // Requires UART library
}
