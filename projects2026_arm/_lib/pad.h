/*
 * pad.h - the app board's controls: joystick, three buttons, a knob   LIBS=pad adc
 *
 * The APP BOARD is one circuit shared by lessons 12-19, as the AVR edition
 * shared one SimulIDE board.  Its base diagram is
 * _targets/app-board-diagram.json; each lesson's diagram.json starts from it.
 *
 *   PA0  ADC IN0   joystick HORZ    Wokwi: 0 V = RIGHT, 3.3 V = LEFT (sic)
 *   PA1  ADC IN1   joystick VERT    0 V = DOWN, 3.3 V = UP
 *   PA4  ADC IN4   potentiometer    the "knob": a gain, a speed, a difficulty
 *   PB3  input     joystick SEL     pressed = 0, pull-up      (key: space)
 *   PB4  input     button A         pressed = 0, pull-up      (key: a)
 *   PB5  input     button B         pressed = 0, pull-up      (key: b)
 *   PA6  TIM3_CH1  buzzer           see beep.h
 *   PB8  I2C1 SCL  OLED 0x3C (+ MPU6050 0x68 in 13, 14, 17)   see oled.h
 *   PB9  I2C1 SDA
 *   PA2  USART2    serial monitor TX / RX on PA3
 *   PA5  output    LD4, the on-board LED
 *
 * HORZ runs backwards: Wokwi's joystick documents 0 V at the RIGHT.  pad_read()
 * flips it, so x > 0 always means right and y > 0 always means up.
 */
#ifndef PAD_H
#define PAD_H

#include <stdint.h>

enum { PAD_A = 1u << 0, PAD_B = 1u << 1, PAD_SEL = 1u << 2 };

typedef struct {
    int16_t x, y;        /* joystick, -100..100: right and up positive; dead zone 0 */
    int16_t knob;        /* potentiometer, 0..1000                                  */
    uint8_t down;        /* PAD_* bits held down now (debounced)                    */
    uint8_t pressed;     /* PAD_* bits that went down since the previous pad_read() */
    uint8_t released;    /* PAD_* bits that came up since the previous pad_read()   */
    uint8_t adc_ok;      /* 0 if the ADC failed to start: x, y, knob then stay 0    */
} pad_t;

/* Clocks GPIOA/GPIOB, sets the pins, starts the ADC.  0 = ready; a negative
 * value is adc_init()'s error, and the buttons still work without it. */
int  pad_init(void);

/* Call at a steady rate, 50-200 Hz.  A button must read the same on two calls
 * in a row to count: at 100 Hz that ignores bounce shorter than 10 ms (lesson
 * 05's debouncer, with a two-sample window). */
void pad_read(pad_t *p);

#endif
