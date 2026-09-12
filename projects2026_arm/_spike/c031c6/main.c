/*
 * main.c - Phase 0 spike, STM32C031C6
 *
 * Proves assumption A2: a hand-written startup.c + link.ld + 48 MHz clock
 * init produces a program that blinks PA5 and prints over USART2, and that
 * newlib-nano printf works once _write() is retargeted.
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32c031xx.h"

#define LED_PIN   5u    /* PA5 - the Nucleo user LED, hardwired on this board */
#define USART_TX  2u    /* PA2 - USART2 TX, AF1, wired to the ST-LINK VCP     */
#define USART_RX  3u    /* PA3 - USART2 RX, AF1                               */
#define BAUD      115200u

static void uart2_init(uint32_t pclk, uint32_t baud)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR1 |= RCC_APBENR1_USART2EN;

    /* Alternate function 1 on PA2 and PA3.  A wrong AF number here gives a
     * pin that is electrically fine and carries no signal - silent, and the
     * single most common way to lose an afternoon on STM32. */
    GPIOA->AFR[0] &= ~((0xFu << (USART_TX * 4)) | (0xFu << (USART_RX * 4)));
    GPIOA->AFR[0] |=  ((0x1u << (USART_TX * 4)) | (0x1u << (USART_RX * 4)));

    GPIOA->MODER &= ~((3u << (USART_TX * 2)) | (3u << (USART_RX * 2)));
    GPIOA->MODER |=  ((2u << (USART_TX * 2)) | (2u << (USART_RX * 2)));  /* AF */

    USART2->BRR = (pclk + baud / 2u) / baud;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart2_putc(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE_TXFNF)) { }
    USART2->TDR = (uint8_t)c;
}

/* Retarget newlib so printf() reaches USART2.  Everything the C library needs
 * from "the system" funnels through _write; nosys.specs supplies stubs for
 * the rest.  This is what replaces _uart.h's fifteen hand-rolled formatters. */
int _write(int fd, const char *buf, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\n') { uart2_putc('\r'); }
        uart2_putc(buf[i]);
    }
    return len;
}

/* newlib wants these seven.  nosys.specs supplies stubs, but each one it
 * links emits a warning, and this course holds lesson code to zero
 * warnings.  Implementing them is cheaper than explaining the noise. */
int   _read (int fd, char *buf, int len) { (void)fd; (void)buf; (void)len; return 0; }
int   _close(int fd)                     { (void)fd; return -1; }
int   _isatty(int fd)                    { (void)fd; return 1; }
int   _lseek(int fd, int off, int dir)   { (void)fd; (void)off; (void)dir; return 0; }
int   _fstat(int fd, void *st)           { (void)fd; (void)st; return 0; }
void  _exit (int code)                   { (void)code; for (;;) { } }
void  _kill (int pid, int sig)           { (void)pid; (void)sig; }
int   _getpid(void)                      { return 1; }

/* sbrk gives malloc its heap: the gap between the end of .bss and the
 * stack.  printf in newlib-nano allocates its buffer here. */
void *_sbrk(int incr)
{
    extern char end;                 /* set by link.ld, just past .bss */
    static char *brk = 0;
    char *prev;
    if (brk == 0) { brk = &end; }
    prev = brk;
    brk += incr;
    return prev;
}

static void led_init(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << (LED_PIN * 2));
    GPIOA->MODER |=  (1u << (LED_PIN * 2));      /* general purpose output */
}

static void delay_crude(volatile uint32_t n) { while (n--) { __asm__ volatile("nop"); } }

int main(void)
{
    extern uint32_t _sdata, _edata, _sbss, _ebss;

    led_init();
    uart2_init(SystemCoreClock, BAUD);

    /* Measured, not asserted - the same discipline 00_Introduction/Main.c
     * uses on AVR, where the memory figures come from linker symbols. */
    printf("\n=== SOC3050 ARM spike: STM32C031C6 ===\n");
    printf("SystemCoreClock : %lu Hz\n", (unsigned long)SystemCoreClock);
    printf(".data in RAM    : %u bytes\n",
           (unsigned)((uintptr_t)&_edata - (uintptr_t)&_sdata));
    printf(".bss  in RAM    : %u bytes\n",
           (unsigned)((uintptr_t)&_ebss - (uintptr_t)&_sbss));
    printf("SPIKE OK\n");

    for (;;) {
        GPIOA->ODR ^= (1u << LED_PIN);           /* toggle - BSRR comes in lesson 02 */
        delay_crude(400000);
    }
}
