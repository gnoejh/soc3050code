/*
 * startup.c - SOC3050 lesson 04, STM32C031C6 (Cortex-M0+)
 *
 * THIS FILE IS THE SUBJECT OF THE LESSON.  Read it before you run it.
 *
 * Lesson 02 told you that between power-on and main() somebody has to build
 * the C environment, and that on a bare-metal part there is no operating
 * system to do it.  This file is that somebody.  Four jobs, in order:
 *
 *      1.  the vector table         - where the hardware looks on reset
 *      2.  copy .data FLASH -> RAM  - your initialised globals
 *      3.  zero .bss                - the C standard promises this
 *      4.  set the clock            - 12 MHz on reset, 48 MHz after
 *
 * On Cortex-M the hardware itself loads SP from vectors[0] and jumps to
 * vectors[1], so unlike AVR there is no assembly bootstrap that must run
 * before C can start.  That is why this file is C and not assembly, and it
 * is why you can read every instruction that executes on this chip.
 *
 * Lab.md has you break each of these four jobs in turn and watch what the
 * board does.  That is the point: startup code is invisible when it works.
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
     * must be copied into RAM before any C code reads them.
     *
     * ---- LAB EXERCISE 1 ----  comment the loop out and rebuild.  Main.c
     * prints a global initialised to 0xC0FFEE; watch what it reads instead. */
    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; ) { *dst++ = *src++; }

    /* .bss is everything initialised to zero.  The C standard promises it, so
     * somebody has to actually do it.
     *
     * ---- LAB EXERCISE 2 ----  comment this loop out and rebuild.  The
     * counter in Main.c is a .bss global; see where it starts counting. */
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
