/* startup.c - Phase 0 spike, STM32L031K6 (Cortex-M0+)
 * Same shape as the C031C6 startup. L0 uses the same modern peripheral IP as
 * the C0, so this is a near-copy - which is the point of testing it.
 */
#include <stdint.h>
#include "stm32l031xx.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int  main(void);
void Reset_Handler(void);
void Default_Handler(void);
void SystemInit(void);
extern void __libc_init_array(void);

uint32_t SystemCoreClock = 2097000UL;   /* MSI at reset; SystemInit raises it */

#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void)       WEAK_ALIAS;
void HardFault_Handler(void) WEAK_ALIAS;
void SVC_Handler(void)       WEAK_ALIAS;
void PendSV_Handler(void)    WEAK_ALIAS;
void SysTick_Handler(void)   WEAK_ALIAS;
void USART2_IRQHandler(void) WEAK_ALIAS;

__attribute__((section(".isr_vector"), used))
void (* const vectors[])(void) = {
    (void (*)(void))&_estack, Reset_Handler,
    NMI_Handler, HardFault_Handler, 0, 0, 0, 0, 0, 0, 0,
    SVC_Handler, 0, 0, PendSV_Handler, SysTick_Handler,
    [16 + 28] = USART2_IRQHandler,
};

void Default_Handler(void) { for (;;) { } }

void Reset_Handler(void)
{
    uint32_t *src, *dst;
    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; ) { *dst++ = *src++; }
    for (dst = &_sbss;  dst < &_ebss;  ) { *dst++ = 0; }
    SystemInit();
    __libc_init_array();
    (void)main();
    for (;;) { }
}

/* The L0 wakes on MSI at ~2.1 MHz. Switching to the 16 MHz HSI is two steps:
 * turn it on, wait for ready, then point the system clock switch at it and
 * wait for the switch to be acknowledged. Unlike the C0's single HSIDIV write,
 * this is a real clock-source switch - which makes it the better teaching
 * example of the two, if slightly longer. */
void SystemInit(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) { }

    SystemCoreClock = 16000000UL;
}
