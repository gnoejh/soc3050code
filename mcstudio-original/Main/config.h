// https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf

#ifndef CONFIG_H_
#define CONFIG_H_

// System clock frequency
#define F_CPU 16000000UL // 16 MHz

// Baud rate for UART
#define BAUD 9600

// AVR system libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Standard libraries (ensure compatibility with AVR)
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

// Project-specific module headers (add/remove as needed)
#include "_main.h"
#include "_buzzer.h"
#include "_adc.h"
#include "_eeprom.h"
#include "_init.h"
#include "_interrupt.h"
#include "_port.h"
#include "_timer2.h"
#include "_uart.h"
#include "_glcd.h"

// Uncomment the app you want to compile

// UART Examples
//#define SERIAL_POLLING_SINGLE_CHAR
//#define SERIAL_POLLING_STRING
//#define SERIAL_POLLING_ECHO
//#define SERIAL_POLLING_SENTENCE
//#define SERIAL_INTERRUPT_RX
//#define SERIAL_INTERRUPT_TX
//#define SERIAL_INTERRUPT_ECHO
//#define SERIAL_INTERRUPT_SENTENCE
//#define SERIAL_INTERRUPT_CIRCULAR_BUFFER
//#define SERIAL_POLLING_WORD
//#define SERIAL_INTERRUPT_WORD

// ADC Examples
//#define ADC_INTERRUPT
//#define ADC_INTERRUPT_UART_INTERRUPT
//#define ADC_INTERRUPT_UART_POLLING
//#define ADC_POLLING
//#define ADC_POLLING_UART_POLLING
//#define ADC_POLLING_UART_INTERRUPT

// Graphics Examples
#define GRAPHICS_BASICS
//#define GRAPHICS_MOVEMENT
//#define GRAPHICS_RANDOM
//#define GRAPHICS_BOUNCING_BALL
//#define GRAPHICS_MOVING_SQUARE
//#define GRAPHICS_SINE_WAVE

// Stepper Motor Examples
//#define MOTORS_FULLSTEP                  // Full-step control for stepper motors
//#define MOTORS_FULLSTEP_INTERRUPT        // Full-step with interrupt control
//#define MOTORS_HALFSTEP                  // Half-step control for stepper motors
//#define MOTORS_HALFSTEP_INTERRUPT        // Half-step with interrupt control
//#define MOTORS_STEPPER_DEMO              // Demonstration of advanced stepper motor patterns

// Servo Motor Examples
//#define MOTORS_SERVO                     // Servo motor control (standard PWM)
//#define MOTORS_SERVO_ADC                 // Servo motor controlled using ADC
//#define MOTORS_SERVO_UART                // Servo motor controlled via UART

// DC Motor Examples
//#define MOTORS_PWM_FAST                  // DC motor control using fast PWM
//#define MOTORS_PWM_PHASECORRECT          // DC motor control using phase-correct PWM

// Sound Examples
//#define SOUND
//#define SOUND_ATARI
//#define SOUND_TWINKLE

// Game Examples
//#define GAME_HANGMAN
//#define GAME_OBSTACLE
//#define GAME_OBSTACLE_LEVEL
//#define GAME_PUZZLE
//#define GAME_SCRAMBLE
//#define GAME_GUESS_SECRET_WORD
//#define GAME_PONG_UART_CONTROL

// Timer Examples
//#define TIMER_COUNTER
//#define TIMER_CTC
//#define TIMER_FASTPWM
//#define TIMER_NORMAL

// External Interrupt Examples
//#define INTERRUPT_EXTERNAL
//#define INTERRUPT_LAB
//#define INTERRUPT_TIMER
//#define INTERRUPT_TIMER_CTC
//#define INTERRUPT_EXT_TIMER

// Port/Pin Control Examples
//#define BLINK_PORT
//#define BLINK_PIN
//#define BLINK_ASM
//#define BLINK_ASM_MACRO
//#define BLINK_ASM_RANDOM
//#define BLINK_ASM_RANDOM_DELAY
//#define PORT_BLINKING
//#define PORT_ROTATION

// Miscellaneous Examples
//#define CDS
//#define IOT
//#define INLINE
//#define MEMORY_EEPROM
//#define MEMORY_PROGRAM
//#define JOYSTICK

#endif /* CONFIG_H_ */
