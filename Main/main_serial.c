/*
 * main_serial.c - Serial Communication Examples (Clean Stub)
 * This file serves as a stub to prevent build errors.
 * Actual implementations are in main_blink_asm.c for systematic testing.
 */

#include "config.h"

// Empty implementations to prevent build errors
// Actual working code is in main_blink_asm.c

#ifdef SERIAL_POLLING_STRING
// Stub - real implementation in main_blink_asm.c
void main_serial_polling_string_stub(void)
{
    // This function is not used - real implementation is in main_blink_asm.c
    while (1)
        ;
}
#endif

#ifdef SERIAL_POLLING_ECHO
void main_serial_polling_echo(void)
{
    // Stub for future implementation
    while (1)
        ;
}
#endif

#ifdef SERIAL_INTERRUPT_RX
void main_serial_interrupt_rx(void)
{
    // Stub for future implementation
    while (1)
        ;
}
#endif

#ifdef SERIAL_INTERRUPT_TX
void main_serial_interrupt_tx(void)
{
    // Stub for future implementation
    while (1)
        ;
}
#endif