/*
 * Timer PWM Motor Control
 * PWM for DC motor speed control using Timer1
 */

#include "config.h"

// Demo 1: Basic motor speed control
void demo_01_motor_basic_speed(void)
{
    // PB5 (OC1A) for motor PWM
    DDRB |= (1 << 5);

    // Fast PWM, 8-bit
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler 8

    OCR1A = 128; // 50% speed

    while (1)
    {
    }
}

// Demo 2: Variable speed with button
void demo_02_motor_variable_speed(void)
{
    uint8_t speeds[] = {0, 64, 128, 192, 255};
    uint8_t speed_index = 2;
    uint8_t btn_last = 1, btn_curr;

    DDRB |= (1 << 5);
    DDRD &= ~(1 << 7);
    PORTD |= (1 << 7);

    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = speeds[speed_index];

    while (1)
    {
        btn_curr = (PIND & (1 << 7)) ? 1 : 0;
        if (btn_last && !btn_curr)
        {
            speed_index = (speed_index + 1) % 5;
            OCR1A = speeds[speed_index];
        }
        btn_last = btn_curr;
        _delay_ms(50);
    }
}

// Demo 3: Acceleration/deceleration
void demo_03_motor_smooth_accel(void)
{
    uint8_t speed = 0;
    int8_t dir = 1;

    DDRB |= (1 << 5);
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);

    while (1)
    {
        OCR1A = speed;
        speed += dir;
        if (speed == 255)
            dir = -1;
        if (speed == 0)
        {
            dir = 1;
            _delay_ms(500);
        }
        _delay_ms(10);
    }
}

int main(void)
{
    demo_01_motor_basic_speed();
    return 0;
}
