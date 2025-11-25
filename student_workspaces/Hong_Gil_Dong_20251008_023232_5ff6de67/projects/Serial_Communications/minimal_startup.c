/*
 * Minimal startup for ATmega128 - SimulIDE compatibility
 * Replaces problematic startup code that generates ELPM instructions
 */

#include <avr/io.h>

// Minimal vector table - only reset vector
void __attribute__((naked)) __attribute__((section(".vectors"))) vectors(void)
{
    asm volatile("jmp main"); // Reset vector - jump to main
    // All other vectors will be at default location (0x0000) causing reset
}

// Manual initialization before main
void __attribute__((naked)) __attribute__((section(".init3"))) init_minimal(void)
{
    // Initialize stack pointer to end of RAM
    asm volatile(
        "ldi r28, lo8(%0)\n\t"
        "ldi r29, hi8(%0)\n\t"
        "out %1, r28\n\t"
        "out %2, r29\n\t"
        "clr r1\n\t" // Clear r1 (compiler assumes it's zero)
        :
        : "i"(RAMEND), "i"(_SFR_IO_ADDR(SPL)), "i"(_SFR_IO_ADDR(SPH))
        : "r28", "r29");
}