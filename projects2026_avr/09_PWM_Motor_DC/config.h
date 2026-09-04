/*
 * config.h - PWM and DC Motor Control
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Focus: fast PWM, duty cycle, and driving a DC motor through an H-bridge
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/* The whole 2026 edition runs at 16 MHz, matching Frequency="16 MHz" on the
 * MCU in the shared SimulIDE board.  build.bat passes -DF_CPU=16000000UL, so
 * this guarded definition only takes effect when a file is compiled on its
 * own - the header and the compiler flag can never disagree. */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 9600
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>


/* --- carried over from the original lesson --- */
#include <stdio.h>
#include <avr/interrupt.h>

// Include shared library headers
#include "_uart.h"
#include "_adc.h"
#include "_init.h"
#include "_pwm.h"

#endif /* CONFIG_H_ */
