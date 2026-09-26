/*
 * retarget.c - shared library, LIBS=retarget
 *
 * Makes printf() reach the serial monitor: USART2 on PA2/PA3, which the
 * Nucleo routes to its virtual COM port and Wokwi to $serialMonitor.  It is
 * lesson 04's retarget.c, moved here once a second lesson needed it.
 *
 * Everything the C library wants from "the system" funnels through one
 * function, _write().  Implement that and the whole of <stdio.h> works.  The
 * other stubs exist to keep the build at zero warnings: --specs=nosys.specs
 * would supply them, but each one it links emits "is not implemented and will
 * always fail" - seven warnings in every lesson.
 *
 * Transmit is polled: printf() returns only when the last character has been
 * handed to the UART, about 87 us per character at 115200 baud.
 *
 * uart2_init() and _write() are WEAK, like the handlers in startup.c: a
 * lesson that defines its own strong versions replaces them at link time and
 * keeps the rest of this file.  Lesson 08 does exactly that, swapping in an
 * interrupt-driven UART.
 */

#include <stdint.h>
#include "stm32c031xx.h"
#include "retarget.h"

#define USART_TX  2u          /* PA2 - USART2 TX, AF1, wired to the VCP */
#define USART_RX  3u          /* PA3 - USART2 RX, AF1                   */

__attribute__((weak)) void uart2_init(uint32_t pclk, uint32_t baud)
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

__attribute__((weak)) int _write(int fd, const char *buf, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\n') { uart2_putc('\r'); }
        uart2_putc(buf[i]);
    }
    return len;
}

int   _read  (int fd, char *buf, int len) { (void)fd; (void)buf; (void)len; return 0; }
int   _close (int fd)                     { (void)fd; return -1; }
int   _isatty(int fd)                     { (void)fd; return 1; }
int   _lseek (int fd, int off, int dir)   { (void)fd; (void)off; (void)dir; return 0; }
int   _fstat (int fd, void *st)           { (void)fd; (void)st; return 0; }
void  _exit  (int code)                   { (void)code; for (;;) { } }
void  _kill  (int pid, int sig)           { (void)pid; (void)sig; }
int   _getpid(void)                       { return 1; }

/* sbrk hands malloc its heap: the gap between the end of .bss and the stack.
 * newlib-nano's printf allocates its output buffer here, so if this returns
 * nonsense, printf prints nothing and you have a silent board. */
void *_sbrk(int incr)
{
    extern char end;                 /* placed by link.ld, just past .bss */
    static char *brk = 0;
    char *prev;
    if (brk == 0) { brk = &end; }
    prev = brk;
    brk += incr;
    return prev;
}
