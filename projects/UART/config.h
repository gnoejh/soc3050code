/*
 * config.h - Project configuration for Serial_Communications
 * Auto-created to satisfy local project includes used by Main.c and serial.c
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Target CPU frequency - typical educational ATmega128 setup
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// Default baud rate used in examples
// Using 9600 baud with U2X=1 for best accuracy and reliability
#ifndef BAUD
#define BAUD 9600
#endif

// UART configuration macros (U2X=1 mode for better accuracy)
#define UART_BAUD_REGISTER ((F_CPU / (8UL * BAUD)) - 1)
#define UART_8BIT_CHAR ((1 << UCSZ11) | (1 << UCSZ10)) // 8N1: 8 data bits, No parity, 1 stop bit
#define UART_ENABLE_RX_TX ((1 << RXEN1) | (1 << TXEN1))
#define UART_U2X_ENABLE (1 << U2X1)
#define UART_U2X_DISABLE (0 << U2X1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
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
