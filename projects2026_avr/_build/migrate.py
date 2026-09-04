#!/usr/bin/env python3
"""Import the curated lesson set from projects/ into projects2026_avr/.

Normalises every lesson to the 2026 conventions:
  * one F_CPU (16 MHz), guarded, so the header and the compiler flag agree
  * Main.c / config.h / Slide.md only - no stray duplicate sources
  * a build.bat that just names its libraries and calls the shared engine
  * a simulate.bat that opens the shared master board on this lesson's hex

Re-runnable.  build.bat / simulate.bat / config.h are regenerated every time;
Main.c and Slide.md are copied from projects/ unless the destination copy has
been edited by hand (pass --force to overwrite those too).
"""
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "projects")
DST = os.path.join(ROOT, "projects2026_avr")

# (new folder, source folder, human title, one-line focus)
LESSONS = [
    ("01_Port_Basic", "Port_Basic", "Digital I/O Fundamentals",
     "DDR, PORT and PIN registers; driving LEDs and reading switches"),
    ("02_Port_Button_Debounce", "Port_Button_Debounce_Simple", "Buttons and Debouncing",
     "internal pull-ups, contact bounce, and a simple software debounce"),
    ("03_Port_Keypad_Matrix", "Port_Keypad_Matrix_Basic", "Matrix Keypad Scanning",
     "row and column scanning of a 4x4 keypad without dedicated hardware"),
    ("04_Inline_Assembly", "Inline_Assembly", "Inline AVR Assembly",
     "asm volatile, operand constraints, and the AVR I/O instruction set"),
    ("05_INT_External_Pins", "INT_External_Pins", "External Interrupts",
     "INT0-INT7, edge sensing, and writing an interrupt service routine"),
    ("06_Timer0_Overflow_Blink", "Timer0_Overflow_Blink", "Timer0 Overflow",
     "prescalers, the overflow flag, and interrupt-driven timing"),
    ("07_Timer1_CTC_Precision", "Timer1_CTC_Precision", "Timer1 CTC Mode",
     "Clear-Timer-on-Compare for exact output frequencies"),
    ("08_Timer1_Input_Capture", "Timer1_Input_Capture", "Input Capture",
     "measuring pulse width and frequency with the ICP unit"),
    ("09_PWM_Motor_DC", "Timer_PWM_Motor_DC", "PWM and DC Motor Control",
     "fast PWM, duty cycle, and driving a DC motor through an H-bridge"),
    ("10_PWM_Servo", "Timer_PWM_Motor_Servo", "Servo Control",
     "50 Hz pulse-width positioning of an RC servo"),
    ("11_ADC_Basic", "ADC_Basic", "Analog-to-Digital Conversion",
     "ADMUX, ADCSRA, reference selection, and 10-bit conversion"),
    ("12_ADC_Light_Sensor", "ADC_CDS_Light_Sensor", "Light Sensing with a CDS Cell",
     "voltage dividers and scaling raw counts into engineering units"),
    ("13_UART_Basic", "UART", "Serial Communication",
     "UBRR and baud rate, framing, and polled transmit and receive"),
    ("14_UART_Ring_Buffer", "USART_Ring_Buffer", "Interrupt-Driven UART",
     "RX and TX interrupts feeding a ring buffer"),
    ("15_SPI_Master_Basic", "SPI_Master_Basic", "SPI Master",
     "SPCR, clock modes, and full-duplex shift-register transfer"),
    ("16_I2C_RTC_DS1307", "I2C_RTC_DS1307", "I2C and a Real-Time Clock",
     "TWI start, stop and acknowledge, and talking to a DS1307"),
    ("17_LCD_Character", "LCD_Character_Basic", "Character LCD Output",
     "text output, cursor positioning, and formatting numbers"),
    ("18_GLCD_Graphics", "Port_Graphics_Display", "Graphic LCD",
     "KS0108 pages and columns, pixels, lines and bitmaps"),
    ("19_EEPROM_ReadWrite", "EEPROM_Basic_ReadWrite", "Internal EEPROM",
     "non-volatile storage, write timing, and endurance"),
    ("20_Power_Sleep_Modes", "Power_Sleep_Modes", "Low Power Operation",
     "sleep modes, wake-up sources, and current budgeting"),
    ("21_Watchdog_Reset", "Watchdog_System_Reset", "Watchdog Timer",
     "WDT time-outs, safe reset, and recovering from a hang"),
    ("22_RTOS_Scheduler", "RTOS_UPGRADED", "A Small Cooperative RTOS",
     "task control blocks, a tick-driven scheduler, and context switching"),
]

CONFIG_TEMPLATE = """/*
 * config.h - {title}
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Focus: {focus}
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/* The whole 2026 edition runs at 16 MHz, matching Frequency="16 MHz" on the
 * MCU in the shared SimulIDE board.  build.bat passes -DF_CPU=16000000UL, so
 * this guarded definition only takes effect when a file is compiled on its
 * own - the header and the compiler flag can never disagree. */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 9600
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
{extra}
#endif /* CONFIG_H_ */
"""

BUILD_TEMPLATE = """@echo off
REM {folder} - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\\_build\\resolve-libs.py --all
set LIBS={libs}
call "%~dp0..\\_build\\build-lesson.bat"
"""

SIM_TEMPLATE = """@echo off
REM {folder} - build if needed, then open the shared board in SimulIDE 1.1.0-SR2
call "%~dp0..\\_build\\simulate-lesson.bat"
"""

# Single lines that must not survive into a 2026 config.h: the template already
# supplies these, or the shared engine does.
DROP_LINES = [
    r"^\s*#\s*define\s+(F_CPU|BAUD)\b",
    r"^\s*#\s*include\s*<(avr/io|util/delay|stdint)\.h>",
]

# Whole conditional blocks to remove, matched on their opening line.  The clock
# and baud rate are settled by the template, so the originals go entirely -
# dropping only the #define would leave a dangling #ifndef/#endif pair.
DROP_BLOCKS = [r"^\s*#\s*ifndef\s+(F_CPU|BAUD)\b"]

IF_OPEN = re.compile(r"^\s*#\s*if(n?def)?\b")
IF_CLOSE = re.compile(r"^\s*#\s*endif\b")


def strip_guard(text):
    """Return the lines inside a header's include guard.

    Everything from the line after '#define CONFIG_H...' up to (but not
    including) the file's final '#endif'.
    """
    lines = text.splitlines()
    start = 0
    for i, line in enumerate(lines):
        if re.match(r"^\s*#\s*define\s+CONFIG_H\w*\s*$", line):
            start = i + 1
            break
    end = len(lines)
    for i in range(len(lines) - 1, start - 1, -1):
        if IF_CLOSE.match(lines[i]):
            end = i
            break
    return lines[start:end]


def normalise_config(path, title, focus, owns_lcd=False):
    """Rewrite a lesson config.h: 16 MHz, guarded, lesson extras preserved.

    Conditional blocks in the original are kept intact - only the F_CPU and
    BAUD blocks are removed whole, so constructs like the '#ifdef
    __has_include' probe in the UART lesson survive with their #endif.

    owns_lcd marks a lesson whose Main.c implements lcd_init()/lcd_clear()
    itself.  _glcd.h defines those two as zero-argument macros, which turns the
    lesson's own definitions into a compile error, so for those lessons the
    _glcd.h include is dropped and the plain declarations are kept.
    """
    extra = []
    if os.path.isfile(path):
        text = open(path, encoding="utf-8", errors="replace").read()
        depth = None                      # nesting depth while skipping a block
        for line in strip_guard(text):
            if depth is not None:         # inside a block being dropped
                if IF_OPEN.match(line):
                    depth += 1
                elif IF_CLOSE.match(line):
                    depth -= 1
                    if depth == 0:
                        depth = None
                continue
            if any(re.match(p, line) for p in DROP_BLOCKS):
                depth = 1
                continue
            if any(re.match(p, line) for p in DROP_LINES):
                continue
            if owns_lcd and re.match(r"^\s*#\s*include\s*.*_glcd\.h", line):
                continue
            if not owns_lcd and re.match(r"^\s*(void\s+)?lcd_(init|clear)\s*\(", line):
                continue
            extra.append(line.rstrip())

    block = "\n".join(extra).strip()
    if block:
        note = ""
        if owns_lcd:
            note = ("/* This lesson implements its own character-LCD driver, so it does\n"
                    " * not include _glcd.h - that header defines lcd_init() and\n"
                    " * lcd_clear() as zero-argument macros. */\n")
        block = ("\n\n/* --- carried over from the original lesson --- */\n"
                 + note + block + "\n")
    else:
        block = "\n"
    return CONFIG_TEMPLATE.format(title=title, focus=focus, extra=block)


def pick_slide(srcdir):
    for name in ("Slide.md", "RTOS_LECTURE_SLIDES.md"):
        p = os.path.join(srcdir, name)
        if os.path.isfile(p):
            return p
    return None


def write(path, text, crlf=False):
    nl = "\r\n" if crlf else "\n"
    with open(path, "w", encoding="utf-8", newline=nl) as fh:
        fh.write(text)


def main(argv):
    force = "--force" in argv
    made, notes = [], []
    for folder, srcname, title, focus in LESSONS:
        srcdir = os.path.join(SRC, srcname)
        dstdir = os.path.join(DST, folder)
        if not os.path.isdir(srcdir):
            notes.append("%s: SOURCE MISSING (%s)" % (folder, srcname))
            continue
        os.makedirs(dstdir, exist_ok=True)

        main_src = os.path.join(srcdir, "Main.c")
        if not os.path.isfile(main_src):
            main_src = os.path.join(srcdir, "main.c")
        main_dst = os.path.join(dstdir, "Main.c")
        if force or not os.path.isfile(main_dst):
            shutil.copyfile(main_src, main_dst)

        # Does this lesson ship its own lcd_init()/lcd_clear() implementation?
        lesson_src = open(main_dst, encoding="utf-8", errors="replace").read()
        owns_lcd = bool(re.search(r"^\s*void\s+lcd_(init|clear)\s*\(\s*void\s*\)\s*$",
                                  lesson_src, re.M))

        write(os.path.join(dstdir, "config.h"),
              normalise_config(os.path.join(srcdir, "config.h"), title, focus,
                               owns_lcd=owns_lcd))

        slide = pick_slide(srcdir)
        slide_dst = os.path.join(dstdir, "Slide.md")
        if slide and (force or not os.path.isfile(slide_dst)):
            shutil.copyfile(slide, slide_dst)
        if not slide and not os.path.isfile(slide_dst):
            notes.append("%s: no Slide.md in source - needs authoring" % folder)

        # keep the genuinely useful supporting handouts
        for fn in os.listdir(srcdir):
            if re.search(r"(QUICK_REFERENCE|FLOW_DIAGRAMS|LAB_GUIDE)\.md$", fn):
                shutil.copyfile(os.path.join(srcdir, fn),
                                os.path.join(dstdir, fn))

        write(os.path.join(dstdir, "simulate.bat"),
              SIM_TEMPLATE.format(folder=folder), crlf=True)
        made.append(folder)

    print("imported %d lessons" % len(made))
    for n in notes:
        print("  note:", n)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
