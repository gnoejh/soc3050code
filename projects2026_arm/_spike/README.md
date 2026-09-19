# Phase 0 spike

Throwaway. Proved the plan's riskiest assumptions before any lesson was
written. See `FINDINGS.md` for the results - all six resolved, none fatal.

> **Phase 0 is closed and its outcome has shipped.** The toolchain it validated
> is vendored at `tools/arm-toolchain/` and `tools/cmsis/`; Target A is decided
> (the C031C6); and `04_Startup_And_LinkerScript/` carries the promoted
> `startup.c` and `link.ld` as the first real lesson. This directory is kept
> only as the record of how those decisions were reached - **nothing builds
> from it and nothing depends on it.**
>
> `c031c6/Main.elf` is still worth keeping: the Part 0 decks quote its
> disassembly and section sizes, and `_build/disasm.py` reads it directly.

## What is here

    c031c6/   Nucleo C031C6 (Cortex-M0+) - builds, and RAN in Wokwi (A1b)
    f446re/   Nucleo F446RE (Cortex-M4F) - builds, and runs under Renode (A3)
    l031k6/   Nucleo L031K6 (Cortex-M0+) - ran under Renode; eliminated on RAM
    f103c8/   Blue Pill F103C8 (Cortex-M3) - ran under Renode; not chosen

Each target carries the three files the plan says replace avr-libc:
`startup.c` (vector table, reset handler, .data/.bss init, clock),
`link.ld` (memory map and sections), and a `main.c` that blinks PA5 and
prints over USART2.

## Dependencies

**The two the course needs are now vendored** (2026-09-19), so the commands
below run against the repository rather than a scratch directory:

| What | Version | Vendored at | Provenance |
|---|---|---|---|
| arm-none-eabi-gcc | 15.2.1-1.1 | `tools/arm-toolchain/` | xPack, GitHub Releases, sha256 verified against the published `.sha` |
| CMSIS Core | CMSIS_6 | `tools/cmsis/Core/` | `ARM-software/CMSIS_6`, Apache-2.0 |
| STM32C0 headers | main | `tools/cmsis/Device/ST/STM32C0xx/` | `STMicroelectronics/cmsis-device-c0`, Apache-2.0 |

**Still not vendored**, and not needed until the F4 half of the course:

| What | Version | Where it came from |
|---|---|---|
| Renode | 1.17.0 portable | builds.renode.io |
| STM32F4 headers | master | `STMicroelectronics/cmsis_device_f4` |

So `f446re/`, `l031k6/` and `f103c8/` cannot be rebuilt from a fresh clone
today. Their committed `Main.elf` and `Main.map` are the evidence; the
FINDINGS entries that rest on them were recorded while those tools were
present.

## Rebuild

C031C6:

    ../../../tools/arm-toolchain/bin/arm-none-eabi-gcc.exe \
      -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft \
      -DSTM32C031xx -Os -g3 -std=c11 -Wall -Wextra \
      -ffunction-sections -fdata-sections \
      -I../../../tools/cmsis/Device/ST/STM32C0xx/Include \
      -I../../../tools/cmsis/Core/Include \
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
