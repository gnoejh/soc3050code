/*
 * main.c - Phase 0 spike, STM32F103C8 "Blue Pill"
 *
 * This file exists to make one curriculum question concrete: how different is
 * STM32F1 from every other STM32 family?  Compare it against the C031C6 and
 * L031K6 spikes side by side.  Two things differ, and both are in every
 * peripheral lesson, not just this one.
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f103xb.h"

#define TX_PIN   2u     /* PA2 - USART2 TX */
#define RX_PIN   3u     /* PA3 - USART2 RX */
#define LED_PIN  13u    /* PC13 - the Blue Pill LED, active low */
#define BAUD     115200u

static void uart2_putc(char c)
{
    /* DIFFERENCE 1: SR/DR, like the F4.  The C0 and L0 use ISR/TDR. */
    while (!(USART2->SR & USART_SR_TXE)) { }
    USART2->DR = (uint8_t)c;
}

int _write(int fd, const char *buf, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\n') { uart2_putc('\r'); }
        uart2_putc(buf[i]);
    }
    return len;
}

int   _read (int fd, char *b, int l) { (void)fd; (void)b; (void)l; return 0; }
int   _close(int fd)                 { (void)fd; return -1; }
int   _isatty(int fd)                { (void)fd; return 1; }
int   _lseek(int fd, int o, int d)   { (void)fd; (void)o; (void)d; return 0; }
int   _fstat(int fd, void *st)       { (void)fd; (void)st; return 0; }
void  _exit (int c)                  { (void)c; for (;;) { } }
void  _kill (int p, int s)           { (void)p; (void)s; }
int   _getpid(void)                  { return 1; }
void *_sbrk(int incr)
{
    extern char end; static char *brk = 0; char *prev;
    if (brk == 0) { brk = &end; }
    prev = brk; brk += incr; return prev;
}

/*
 * DIFFERENCE 2, and the big one: F1 GPIO configuration.
 *
 * Every other STM32 family gives each pin its own two-bit field in MODER,
 * plus OTYPER, OSPEEDR, PUPDR and a four-bit AFR entry - five registers, one
 * concern each:
 *
 *     GPIOA->MODER |= (2u << (pin * 2));      // alternate function
 *     GPIOA->AFR[0] |= (7u << (pin * 4));     // which alternate function
 *
 * The F1 instead packs MODE (2 bits) and CNF (2 bits) into ONE four-bit field
 * per pin, split across two registers - CRL for pins 0-7, CRH for pins 8-15 -
 * and the meaning of CNF changes depending on whether MODE says input or
 * output.  There is no AFR at all; alternate functions are selected by
 * remapping whole peripherals through AFIO->MAPR.
 *
 * It is not harder.  It is simply unlike the rest of the line, so nothing a
 * student learns here transfers to the F446RE, and nothing they read in a
 * modern STM32 tutorial applies back.
 */
static void gpio_uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2: MODE=11 (output 50 MHz), CNF=10 (alternate push-pull) -> 0xB */
    GPIOA->CRL &= ~(0xFu << (TX_PIN * 4));
    GPIOA->CRL |=  (0xBu << (TX_PIN * 4));

    /* PA3: MODE=00 (input), CNF=01 (floating) -> 0x4 */
    GPIOA->CRL &= ~(0xFu << (RX_PIN * 4));
    GPIOA->CRL |=  (0x4u << (RX_PIN * 4));

    /* PC13 is in CRH, because it is above pin 7.  MODE=10 (2 MHz output),
     * CNF=00 (general purpose push-pull) -> 0x2 */
    GPIOC->CRH &= ~(0xFu << ((LED_PIN - 8u) * 4));
    GPIOC->CRH |=  (0x2u << ((LED_PIN - 8u) * 4));
}

int main(void)
{
    extern uint32_t _sdata, _edata, _sbss, _ebss;

    gpio_uart_init();
    USART2->BRR = (SystemCoreClock + BAUD / 2u) / BAUD;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    printf("\n=== SOC3050 ARM spike: STM32F103C8 ===\n");
    printf("SystemCoreClock : %lu Hz\n", (unsigned long)SystemCoreClock);
    printf(".data in RAM    : %u bytes\n", (unsigned)((uintptr_t)&_edata - (uintptr_t)&_sdata));
    printf(".bss  in RAM    : %u bytes\n", (unsigned)((uintptr_t)&_ebss - (uintptr_t)&_sbss));
    printf("GPIO model      : CRL/CRH, not MODER\n");
    printf("SPIKE OK\n");

    for (;;) {
        GPIOC->ODR ^= (1u << LED_PIN);
        for (volatile uint32_t i = 0; i < 100000u; i++) { }
    }
}
