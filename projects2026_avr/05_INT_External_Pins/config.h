/*
 * config.h - External Interrupts
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Focus: INT0-INT7, edge sensing, and writing an interrupt service routine
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
#include "../../shared_libs/_uart.h"
#include "../../shared_libs/_init.h"

#endif /* CONFIG_H_ */
