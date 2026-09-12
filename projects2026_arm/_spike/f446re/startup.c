/*
 * startup.c - Phase 0 spike, STM32F446RE (Cortex-M4F)
 *
 * Same shape as the C031C6 startup, and deliberately so: one design, two
 * targets.  The differences from the M0+ version are exactly three, and all
 * three are worth a slide:
 *
 *   1. The M4 has more core exceptions (MemManage, BusFault, UsageFault,
 *      DebugMon) where the M0+ has reserved holes.
 *   2. The FPU exists and is DISABLED at reset.  One write to SCB->CPACR
 *      turns it on; forget it and the first float instruction faults.
 *   3. The F4's USART uses SR/DR where the C0 uses ISR/TDR, so peripheral
 *      code does NOT port between the two families unchanged.
 */

#include <stdint.h>
#include "stm32f446xx.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

int  main(void);
void Reset_Handler(void);
void Default_Handler(void);
void SystemInit(void);
extern void __libc_init_array(void);

uint32_t SystemCoreClock = 16000000UL;   /* HSI; see SystemInit() */

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
void TIM2_IRQHandler(void)    WEAK_ALIAS;

/* The spike needs only the core exceptions plus a couple of IRQs; the real
 * _startup/f446re/startup.c carries the full 97-entry table. */
__attribute__((section(".isr_vector"), used))
void (* const vectors[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler, HardFault_Handler, MemManage_Handler, BusFault_Handler,
    UsageFault_Handler, 0, 0, 0, 0,
    SVC_Handler, DebugMon_Handler, 0, PendSV_Handler, SysTick_Handler,
    /* IRQ0.. - padded; entry 28 is TIM2, 38 is USART2 on this part */
    [16 + 28] = TIM2_IRQHandler,
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

void SystemInit(void)
{
    /* Enable the FPU: full access to coprocessors 10 and 11.  This must happen
     * before any floating-point instruction executes, which in practice means
     * before main() - the compiler is free to emit one anywhere. */
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));

    /* The spike runs on the raw 16 MHz HSI.  No PLL, no flash wait states, no
     * voltage scaling - none of which Renode models faithfully anyway.  The
     * 84 MHz PLL configuration belongs in Phase 4, against a target that can
     * actually be measured. */
    SystemCoreClock = 16000000UL;
}
