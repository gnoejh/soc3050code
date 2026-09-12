# Phase 0 spike

Throwaway. Proves the plan's riskiest assumptions before any lesson is written.
Delete this directory when Phase 0 closes, or promote the files into
`_startup/` and `_sim/`. See `FINDINGS.md` for results.

## What is here

    c031c6/   Nucleo C031C6 (Cortex-M0+) - builds clean, NOT yet executed
    f446re/   Nucleo F446RE (Cortex-M4F) - builds clean AND runs under Renode

Each target carries the three files the plan says replace avr-libc:
`startup.c` (vector table, reset handler, .data/.bss init, clock),
`link.ld` (memory map and sections), and a `main.c` that blinks PA5 and
prints over USART2.

## Dependencies, none of them vendored yet

Phase 1 moves these into `tools/`. For now they live in a scratch directory
and the paths below must be adjusted:

| What | Version | Where it came from |
|---|---|---|
| arm-none-eabi-gcc | 15.2.1-1.1 | xPack, GitHub Releases, sha256 verified |
| Renode | 1.17.0 portable | builds.renode.io |
| CMSIS Core | CMSIS_6 | `ARM-software/CMSIS_6`, Apache-2.0 |
| STM32C0 headers | main | `STMicroelectronics/cmsis-device-c0`, Apache-2.0 |
| STM32F4 headers | master | `STMicroelectronics/cmsis_device_f4` |

## Rebuild

C031C6:

    arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft \
      -DSTM32C031xx -Os -g3 -std=c11 -Wall -Wextra \
      -ffunction-sections -fdata-sections \
      -I<cmsis-device-c0>/Include -I<CMSIS_6>/CMSIS/Core/Include \
      startup.c main.c --specs=nano.specs --specs=nosys.specs \
      -T link.ld -Wl,--gc-sections -Wl,-Map=Main.map \
      -Wl,--print-memory-usage -o Main.elf

F446RE: same, but

      -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
      -DSTM32F446xx -I<cmsis-device-f4>/Include

## Run the F446RE spike

    renode.exe --console --disable-xwt --plain -e "include @headless.resc"

`headless.resc` uses `emulation RunFor "0.5"` - half a second of **virtual**
time, so the run is deterministic rather than wall-clock dependent - and logs
USART2 to stdout with no GUI and no Robot Framework. Expect the banner and
`SPIKE OK`.

`test.robot` is the richer `renode-test` version. It needs Robot Framework,
which the portable package does not bundle (`pip install robotframework`).

## Running the C031C6 spike

Not possible yet. Renode has no STM32C0 platform description, and writing one
from the SVD would give register maps without register behaviour - which looks
like it works and is worse than no simulation. This target needs Wokwi, which
needs assumption A1 answered first.
