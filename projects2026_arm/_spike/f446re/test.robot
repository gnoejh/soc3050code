*** Settings ***
Suite Setup                   Setup
Suite Teardown                Teardown
Test Teardown                 Test Teardown
Resource                      ${RENODEKEYWORDS}

*** Test Cases ***
Should Print The Spike Banner
    Execute Command           mach create "f446re"
    Execute Command           machine LoadPlatformDescription @${CURDIR}/nucleo_f446re.repl
    Execute Command           sysbus LoadELF @${CURDIR}/Main.elf
    Create Terminal Tester    sysbus.usart2

    Start Emulation

    Wait For Line On Uart     === SOC3050 ARM spike: STM32F446RE ===
    Wait For Line On Uart     SystemCoreClock : 16000000 Hz
    Wait For Line On Uart     FPU 3.5 * 2.0   : 7000 (x1000)
    Wait For Line On Uart     SPIKE OK
