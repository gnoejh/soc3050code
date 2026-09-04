/*
 * config.h - Serial Communication
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Focus: UBRR and baud rate, framing, and polled transmit and receive
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
// Target CPU frequency - typical educational ATmega128 setup

// Default baud rate used in examples
// Using 9600 baud with U2X=1 for best accuracy and reliability

// UART configuration macros (U2X=1 mode for better accuracy)
#define UART_BAUD_REGISTER ((F_CPU / (8UL * BAUD)) - 1)
#define UART_8BIT_CHAR ((1 << UCSZ11) | (1 << UCSZ10)) // 8N1: 8 data bits, No parity, 1 stop bit
#define UART_ENABLE_RX_TX ((1 << RXEN1) | (1 << TXEN1))
#define UART_U2X_ENABLE (1 << U2X1)
#define UART_U2X_DISABLE (0 << U2X1)

#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

// Provide optional include of shared port helpers if present
// Path is relative to this project directory: ../../shared_libs
#ifdef __has_include
#if __has_include("../../shared_libs/_port.h")
#include "../../shared_libs/_port.h"
#endif
#endif

#endif /* CONFIG_H_ */
