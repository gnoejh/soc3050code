# Timer Library Test

## Purpose
**Library Validation Test Suite** - Verifies that the refined `_timer.h` library works correctly.

## Not a Teaching Project!
This is **NOT** for learning timer fundamentals. For educational timer tutorials, see:
- **`Timer_Programming/`** - Low-level register programming demos
- **Slide.md** - Complete timer theory and concepts

## What This Tests
- ✅ General Timer library (`_timer.h`) supporting all 4 timers
- ✅ Callback-based interrupt system
- ✅ Millisecond timing (Arduino-compatible)
- ✅ Task scheduler functionality
- ✅ All timer modes (Normal, CTC, PWM)

## Usage
This project is for **framework maintainers** to validate library refinements. Students should start with `Timer_Programming/` to learn fundamentals first.

## Relationship
```
Timer_Programming/    →  Learn HOW timers work (registers, modes)
_timer.h library      →  Abstraction layer (our framework)
Timer_Library_Test/   →  Test that abstraction works correctly
```
