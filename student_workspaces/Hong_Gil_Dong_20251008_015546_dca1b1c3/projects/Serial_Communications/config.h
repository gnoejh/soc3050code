/*
 * Configuration Header - Serial Communications (Polling vs Interrupt)
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Ensure F_CPU is set to 16MHz for proper UART timing
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// Ensure BAUD is set to 9600
#ifndef BAUD
#define BAUD 9600
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

// Include only basic port library (no interrupt manager for educational ISR examples)
#include "../../shared_libs/_port.h"

// Educational UART macros for direct register programming
#define UART_BAUD_REGISTER (F_CPU / 16 / BAUD - 1)

// Educational register setup macros
#define UART_8BIT_CHAR ((1 << UCSZ11) | (1 << UCSZ10))
#define UART_ENABLE_RX_TX ((1 << RXEN1) | (1 << TXEN1))
#define UART_ENABLE_RX_INT (1 << RXCIE1)
#define UART_ENABLE_TX_INT (1 << UDRIE1)

#endif /* CONFIG_H_ */