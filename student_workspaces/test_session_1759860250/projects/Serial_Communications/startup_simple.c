/*
 * Simple startup for ATmega128 - SimulIDE compatibility
 * Avoids ELPM instructions that cause issues in SimulIDE
 */

#include <avr/io.h>

// Minimal startup without problematic startup code
void __attribute__((naked)) __attribute__((section(".init8"))) main_startup(void)
{
    // Initialize stack pointer
    asm volatile("ldi r28, lo8(__stack)");
    asm volatile("ldi r29, hi8(__stack)");
    asm volatile("out __SP_L__, r28");
    asm volatile("out __SP_H__, r29");

    // Clear r1 (assumed to be zero by compiler)
    asm volatile("clr r1");

    // Enable interrupts
    asm volatile("sei");

    // Jump to main
    asm volatile("call main");

    // Loop forever if main returns
    asm volatile("1: rjmp 1b");
}