/*
 * startup.c - shared default for target c031c6 (STM32C031C6, Cortex-M0+)
 *
 * Every lesson that does not carry its own startup.c links this one; the
 * build engine falls back to it automatically.  It is lesson 04's startup.c
 * with the lab exercises taken out - lesson 04 is where you wrote it and
 * broke it, so from lesson 05 on it is infrastructure.  Four jobs, in order:
 *
 *      1.  the vector table         - where the hardware looks on reset
 *      2.  copy .data FLASH -> RAM  - your initialised globals
 *      3.  zero .bss                - the C standard promises this
 *      4.  set the clock            - 12 MHz on reset, 48 MHz after
 *
 * Change it here and every such lesson changes with it.  A lesson that needs
 * something different copies this file into its own folder instead.
 */

#include <stdint.h>
#include "stm32c031xx.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

int  main(void);
void Reset_Handler(void);
void Default_Handler(void);
void SystemInit(void);

extern void __libc_init_array(void);

/* ST's header declares this extern; their system_stm32c0xx.c defines it.  We
 * do not take that file, so it is defined here.  Reset value is HSI48 / 4;
 * SystemInit() raises it to 48 MHz below. */
uint32_t SystemCoreClock = 12000000UL;

/* Every handler below is weak and aliased to Default_Handler.  A lesson that
 * wants one simply defines a C function with the same name: at link time the
 * strong definition wins.  There is no ISR() macro and no vector number.
 *
 * The trap: misspell the name and you have merely defined an unused function.
 * It compiles, it links, it runs, and the interrupt lands in Default_Handler.
 * Nothing warns you.  `arm-none-eabi-nm Main.elf` shows the truth: your
 * handler must be listed with a T (strong), not a W (the weak alias).
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

    src = &_sidata;                                     /* .data: FLASH -> RAM */
    for (dst = &_sdata; dst < &_edata; ) { *dst++ = *src++; }

    for (dst = &_sbss; dst < &_ebss; ) { *dst++ = 0; }  /* .bss: zero          */

    SystemInit();
    __libc_init_array();            /* C++ ctors and __attribute__((constructor)) */
    (void)main();

    for (;;) { }                    /* main() must not return, but if it does */
}

/* 48 MHz with no PLL: the C0 wakes on HSI48 / 4, and clearing HSIDIV gives
 * the full 48 MHz in one write.  The flash wait state must be raised BEFORE
 * the clock speeds up, never after. */
void SystemInit(void)
{
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_0) { }

    RCC->CR &= ~RCC_CR_HSIDIV;      /* HSIDIV = 000 -> divide by 1 -> 48 MHz */
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    SystemCoreClock = 48000000UL;
}
