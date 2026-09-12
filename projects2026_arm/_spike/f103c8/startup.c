/* startup.c - Phase 0 spike, STM32F103C8 (Cortex-M3) */
#include <stdint.h>
#include "stm32f103xb.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int  main(void);
void Reset_Handler(void);
void Default_Handler(void);
void SystemInit(void);
extern void __libc_init_array(void);

uint32_t SystemCoreClock = 8000000UL;   /* HSI 8 MHz at reset */

#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void)        WEAK_ALIAS;
void HardFault_Handler(void)  WEAK_ALIAS;
void MemManage_Handler(void)  WEAK_ALIAS;
void BusFault_Handler(void)   WEAK_ALIAS;
void UsageFault_Handler(void) WEAK_ALIAS;
void SVC_Handler(void)        WEAK_ALIAS;
void DebugMon_Handler(void)   WEAK_ALIAS;
void PendSV_Handler(void)     WEAK_ALIAS;
void SysTick_Handler(void)    WEAK_ALIAS;
void USART2_IRQHandler(void)  WEAK_ALIAS;

__attribute__((section(".isr_vector"), used))
void (* const vectors[])(void) = {
    (void (*)(void))&_estack, Reset_Handler,
    NMI_Handler, HardFault_Handler, MemManage_Handler, BusFault_Handler,
    UsageFault_Handler, 0, 0, 0, 0,
    SVC_Handler, DebugMon_Handler, 0, PendSV_Handler, SysTick_Handler,
    [16 + 38] = USART2_IRQHandler,
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

/* The F103 wakes on the 8 MHz HSI already selected, so the spike leaves it
 * there. Reaching 72 MHz needs the PLL plus flash latency, which is Phase 4
 * work against something measurable. */
void SystemInit(void) { SystemCoreClock = 8000000UL; }
