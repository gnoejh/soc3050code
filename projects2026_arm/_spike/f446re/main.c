/*
 * main.c - Phase 0 spike, STM32F446RE, run under Renode.
 * Proves assumption A3 and, with it, the execution half of A2.
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f446xx.h"

#define LED_PIN  5u     /* PA5 - Nucleo user LED */
#define TX_PIN   2u     /* PA2 - USART2 TX, AF7  */
#define RX_PIN   3u     /* PA3 - USART2 RX, AF7  */
#define BAUD     115200u

static void uart2_putc(char c)
{
    /* SR/DR here.  On the STM32C0 the same code is ISR/TDR - the single
     * clearest example of why two STM32 families is a real cost. */
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

int   _read (int fd, char *b, int l)   { (void)fd; (void)b; (void)l; return 0; }
int   _close(int fd)                   { (void)fd; return -1; }
int   _isatty(int fd)                  { (void)fd; return 1; }
int   _lseek(int fd, int o, int d)     { (void)fd; (void)o; (void)d; return 0; }
int   _fstat(int fd, void *st)         { (void)fd; (void)st; return 0; }
void  _exit (int c)                    { (void)c; for (;;) { } }
void  _kill (int p, int s)             { (void)p; (void)s; }
int   _getpid(void)                    { return 1; }

void *_sbrk(int incr)
{
    extern char end;
    static char *brk = 0;
    char *prev;
    if (brk == 0) { brk = &end; }
    prev = brk; brk += incr;
    return prev;
}

static void uart2_init(uint32_t pclk, uint32_t baud)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->AFR[0] &= ~((0xFu << (TX_PIN * 4)) | (0xFu << (RX_PIN * 4)));
    GPIOA->AFR[0] |=  ((0x7u << (TX_PIN * 4)) | (0x7u << (RX_PIN * 4)));  /* AF7 */
    GPIOA->MODER  &= ~((3u << (TX_PIN * 2)) | (3u << (RX_PIN * 2)));
    GPIOA->MODER  |=  ((2u << (TX_PIN * 2)) | (2u << (RX_PIN * 2)));      /* AF  */

    USART2->BRR = (pclk + baud / 2u) / baud;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << (LED_PIN * 2));
    GPIOA->MODER |=  (1u << (LED_PIN * 2));
}

int main(void)
{
    extern uint32_t _sdata, _edata, _sbss, _ebss;
    volatile float a = 3.5f, b = 2.0f;    /* touches the FPU on purpose */

    led_init();
    uart2_init(SystemCoreClock, BAUD);

    printf("\n=== SOC3050 ARM spike: STM32F446RE ===\n");
    printf("SystemCoreClock : %lu Hz\n", (unsigned long)SystemCoreClock);
    printf(".data in RAM    : %u bytes\n", (unsigned)((uintptr_t)&_edata - (uintptr_t)&_sdata));
    printf(".bss  in RAM    : %u bytes\n", (unsigned)((uintptr_t)&_ebss - (uintptr_t)&_sbss));
    printf("FPU 3.5 * 2.0   : %ld (x1000)\n", (long)(a * b * 1000.0f));
    printf("SPIKE OK\n");

    for (;;) {
        GPIOA->ODR ^= (1u << LED_PIN);
        for (volatile uint32_t i = 0; i < 200000u; i++) { }
    }
}
