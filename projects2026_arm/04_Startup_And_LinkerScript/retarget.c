/*
 * retarget.c - SOC3050 lesson 04
 *
 * Infrastructure, not the subject of this lesson.  It is here so that printf()
 * in Main.c reaches the serial monitor, and so you can see exactly how much
 * code that takes: about seventy lines, most of them stubs.
 *
 * On AVR this course hand-rolled fifteen formatters because avr-libc's printf
 * was too heavy.  Here newlib-nano's printf is affordable, and everything the
 * C library wants from "the system" funnels through one function, _write().
 * Implement that and the whole of <stdio.h> works.
 *
 * The other seven stubs exist to keep the build at zero warnings.  Linking
 * --specs=nosys.specs instead would supply them, but each one it links emits
 * "_close is not implemented and will always fail" - seven warnings in every
 * lesson.  Writing them ourselves is cheaper than teaching people to ignore
 * warnings.
 */

#include <stdint.h>
#include "stm32c031xx.h"

#define USART_TX  2u          /* PA2 - USART2 TX, AF1, wired to the VCP */
#define USART_RX  3u          /* PA3 - USART2 RX, AF1                   */

void uart2_init(uint32_t pclk, uint32_t baud)
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

int _write(int fd, const char *buf, int len)
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
