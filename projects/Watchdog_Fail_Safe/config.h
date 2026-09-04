/*
 * Configuration Header for Watchdog Fail-Safe
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H
#define CONFIG_H

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "../../shared_libs/_uart.h"

#endif
