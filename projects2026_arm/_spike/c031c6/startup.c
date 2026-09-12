/*
 * startup.c - Phase 0 spike, STM32C031C6 (Cortex-M0+)
 *
 * Everything avr-libc used to do for free, written out so a student can read
 * it: the vector table, the reset handler, .data/.bss initialisation, and the
 * clock setup.  On Cortex-M the hardware loads the stack pointer from
 * vectors[0] and jumps to vectors[1], so unlike AVR there is no bootstrap
 * that has to run before C.  That is why this file is C and not assembly.
 */

#include <stdint.h>
#include "stm32c031xx.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

int  main(void);
void Reset_Handler(void);
void Default_Handler(void);
void SystemInit(void);

extern void __libc_init_array(void);

/* ST's header declares this extern; their system_stm32c0xx.c defines it.
 * We do not take that file - the point of lesson 01 is that a student
 * reads every line that runs - so we define it here.  Reset value is
 * HSI48 / 4; SystemInit() raises it to 48 MHz below. */
uint32_t SystemCoreClock = 12000000UL;

/* Every handler below is weak and aliased to Default_Handler.  A lesson that
 * wants one simply defines a C function with the same name: at link time the
 * strong definition wins.  There is no ISR() macro and no vector number.
 *
 * The trap: misspell the name and you have merely defined an unused function.
 * It compiles, it links, it runs, and the interrupt does nothing.  Nothing
 * warns you.  verify-all.ps1 checks for exactly this.
 */
#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))

void NMI_Handler(void)              WEAK_ALIAS;
void HardFault_Handler(void)        WEAK_ALIAS;
void SVC_Handler(void)              WEAK_ALIAS;
void PendSV_Handler(void)           WEAK_ALIAS;
void SysTick_Handler(void)          WEAK_ALIAS;

void WWDG_IRQHandler(void)          WEAK_ALIAS;
void RTC_TAMP_IRQHandler(void)      WEAK_ALIAS;
void FLASH_IRQHandler(void)         WEAK_ALIAS;
void RCC_IRQHandler(void)           WEAK_ALIAS;
void EXTI0_1_IRQHandler(void)       WEAK_ALIAS;
void EXTI2_3_IRQHandler(void)       WEAK_ALIAS;
void EXTI4_15_IRQHandler(void)      WEAK_ALIAS;
void DMA1_Channel1_IRQHandler(void) WEAK_ALIAS;
void DMA1_Channel2_3_IRQHandler(void) WEAK_ALIAS;
void DMA1_Ch4_DMAMUX_IRQHandler(void) WEAK_ALIAS;
void ADC1_IRQHandler(void)          WEAK_ALIAS;
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) WEAK_ALIAS;
void TIM1_CC_IRQHandler(void)       WEAK_ALIAS;
void TIM3_IRQHandler(void)          WEAK_ALIAS;
void TIM14_IRQHandler(void)         WEAK_ALIAS;
void TIM16_IRQHandler(void)         WEAK_ALIAS;
void TIM17_IRQHandler(void)         WEAK_ALIAS;
void I2C1_IRQHandler(void)          WEAK_ALIAS;
void SPI1_IRQHandler(void)          WEAK_ALIAS;
void USART1_IRQHandler(void)        WEAK_ALIAS;
void USART2_IRQHandler(void)        WEAK_ALIAS;

/* vectors[0] is not a function pointer at all - it is the initial stack
 * pointer.  The cast is the price of keeping the whole table in one array. */
__attribute__((section(".isr_vector"), used))
void (* const vectors[])(void) = {
    (void (*)(void))&_estack,       /*  0  initial MSP                      */
    Reset_Handler,                  /*  1  reset                            */
    NMI_Handler,                    /*  2                                   */
    HardFault_Handler,              /*  3                                   */
    0, 0, 0, 0, 0, 0, 0,            /*  4-10 reserved                       */
    SVC_Handler,                    /* 11                                   */
    0, 0,                           /* 12-13 reserved                       */
    PendSV_Handler,                 /* 14                                   */
    SysTick_Handler,                /* 15                                   */
    /* external interrupts, IRQ0 upward */
    WWDG_IRQHandler, 0, RTC_TAMP_IRQHandler, FLASH_IRQHandler,
    RCC_IRQHandler, EXTI0_1_IRQHandler, EXTI2_3_IRQHandler, EXTI4_15_IRQHandler,
    0, DMA1_Channel1_IRQHandler, DMA1_Channel2_3_IRQHandler,
    DMA1_Ch4_DMAMUX_IRQHandler, ADC1_IRQHandler,
    TIM1_BRK_UP_TRG_COM_IRQHandler, TIM1_CC_IRQHandler, 0,
    TIM3_IRQHandler, 0, 0, TIM14_IRQHandler, 0, TIM16_IRQHandler,
    TIM17_IRQHandler, I2C1_IRQHandler, 0, SPI1_IRQHandler, 0,
    USART1_IRQHandler, USART2_IRQHandler,
};

void Default_Handler(void)
{
    /* Park here rather than returning.  An unexpected interrupt that silently
     * returns gives you a program that misbehaves; one that stops gives you a
     * program you can attach a debugger to and read the IPSR of. */
    for (;;) { }
}

void Reset_Handler(void)
{
    uint32_t *src, *dst;

    /* .data holds initialised variables.  They live in FLASH at _sidata and
     * must be copied into RAM before any C code reads them. */
    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; ) { *dst++ = *src++; }

    /* .bss is everything initialised to zero.  The C standard promises it, so
     * somebody has to actually do it. */
    for (dst = &_sbss; dst < &_ebss; ) { *dst++ = 0; }

    SystemInit();
    __libc_init_array();            /* C++ ctors and __attribute__((constructor)) */
    (void)main();

    for (;;) { }                    /* main() must not return, but if it does */
}

/*
 * 48 MHz with no PLL at all.
 *
 * The STM32C0 wakes on HSI48 divided by 4, so 12 MHz.  Clearing HSIDIV gives
 * the full 48 MHz in a single register write - there is no PLL to configure,
 * no lock bit to poll and no source switch to wait for.  That makes this the
 * gentlest clock lesson of any STM32 family.
 *
 * The flash wait state must be raised BEFORE the clock speeds up, never after.
 */
void SystemInit(void)
{
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_0) { }

    RCC->CR &= ~RCC_CR_HSIDIV;      /* HSIDIV = 000 -> divide by 1 -> 48 MHz */
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    SystemCoreClock = 48000000UL;
}
