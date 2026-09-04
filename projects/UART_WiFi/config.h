/*
 * config.h - Project configuration for Bluethooth (ESP-01/Bluetooth)
 * Dual UART configuration: UART0 for ESP-01, UART1 for debug
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Target CPU frequency
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// Baud rates
#ifndef BAUD
#define BAUD 9600 // Debug UART1 default
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

#endif /* CONFIG_H_ */
