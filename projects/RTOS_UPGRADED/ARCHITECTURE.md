# Enhanced RTOS v2.0 Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Enhanced RTOS v2.0                        │
│                    ATmega128 System                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                   Application Layer (8 Tasks)                │
├──────────┬──────────┬──────────┬──────────┬─────────────────┤
│ Task 1   │ Task 2   │ Task 3   │ Task 4   │  Tasks 5-8      │
│   LED    │  UART    │   ADC    │  Button  │  Motor/Watch/   │
│ Sequence │  Status  │ Monitor  │ Monitor  │  Beat/Stats     │
│ (NORMAL) │ (NORMAL) │ (NORMAL) │  (HIGH)  │  (varies)       │
└──────────┴──────────┴──────────┴──────────┴─────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                      RTOS Kernel v2.0                        │
├──────────────────────┬──────────────────────────────────────┤
│   Task Manager       │         Scheduler                    │
│ • Create (8 tasks)   │ • Priority-based                     │
│ • Task States        │ • Round-robin                        │
│ • TCB Management     │ • Context Switch                     │
│ • Statistics         │ • Delay Management                   │
└──────────────────────┴──────────────────────────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                  Hardware Abstraction Layer                  │
├──────────┬──────────┬──────────┬──────────┬────────────────┤
│ Timer1   │  UART1   │   ADC    │  GPIO    │  PWM/Buzzer   │
│ (10ms)   │ (9600)   │ (10-bit) │(PB/PD/PF)│  (Timer1)     │
└──────────┴──────────┴──────────┴──────────┴────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                   ATmega128 Hardware                         │
│  Timers | UART | ADC | GPIO | Interrupts | Memory (8KB RAM) │
└─────────────────────────────────────────────────────────────┘
```

## Task Control Block (TCB) Structure

```
┌─────────────────────────────────────────────────────────────┐
│              Task Control Block (Enhanced v2.0)              │
│                      (~280 bytes per task)                   │
├─────────────────────────────────────────────────────────────┤
│  Offset  │  Field              │  Size    │  Description    │
├──────────┼─────────────────────┼──────────┼─────────────────┤
│  +0      │  task_id            │  1 byte  │  Unique ID      │
│  +1      │  task_name[20]      │ 20 bytes │  Name string    │
│  +21     │  stack_pointer      │  2 bytes │  Current SP     │
│  +23     │  stack[256]         │256 bytes │  Private stack  │
│  +279    │  state              │  1 byte  │  Task state     │
│  +280    │  priority           │  1 byte  │  Priority level │
│  +281    │  delay_ticks        │  4 bytes │  Delay counter  │
│  +285    │  execution_count    │  4 bytes │  Exec counter   │
│  +289    │  task_function      │  2 bytes │  Function ptr   │
│  +291    │  auto_restart       │  1 byte  │  Auto-restart   │
└──────────┴─────────────────────┴──────────┴─────────────────┘

Total: ~292 bytes per task
8 Tasks: ~2336 bytes (29% of 8KB RAM)
```

## Memory Layout

```
ATmega128 RAM: 8192 bytes (0x0000 - 0x1FFF)

Address
0x0000  ┌─────────────────────────────────────────┐
        │         I/O Registers (224 bytes)       │
0x00E0  ├─────────────────────────────────────────┤
        │         Extended I/O (160 bytes)        │
0x0100  ├─────────────────────────────────────────┤
        │                                         │
        │         Internal RAM (7808 bytes)       │
        │                                         │
        │  ┌────────────────────────────────────┐ │
        │  │ Global Variables (~200 bytes)     │ │
        │  ├────────────────────────────────────┤ │
        │  │                                    │ │
        │  │  Task Control Blocks (8 TCBs)     │ │
        │  │  Each: ~292 bytes                 │ │
        │  │  Total: ~2336 bytes               │ │
        │  │                                    │ │
        │  │  ┌──────────────────────────────┐ │ │
        │  │  │ Task 1 Stack (256 bytes)     │ │ │
        │  │  ├──────────────────────────────┤ │ │
        │  │  │ Task 2 Stack (256 bytes)     │ │ │
        │  │  ├──────────────────────────────┤ │ │
        │  │  │      ... (6 more stacks)     │ │ │
        │  │  └──────────────────────────────┘ │ │
        │  │                                    │ │
        │  │  Total RTOS: ~2536 bytes (31%)    │ │
        │  └────────────────────────────────────┘ │
        │                                         │
        │         Free RAM (~5472 bytes)          │
        │                                         │
0x1FFF  └─────────────────────────────────────────┘

Program Flash: 128KB
RTOS Code: ~6-8KB (5-6% of flash)
```

## Task Scheduling Algorithm

```
┌──────────────────────────────────────────────────────────────┐
│          Timer1 Interrupt (Every 10ms)                       │
└────────────────────┬─────────────────────────────────────────┘
                     │
                     ↓
          ┌──────────────────────┐
          │ Increment system_ticks│
          │ system_ticks++       │
          └──────────┬───────────┘
                     │
                     ↓
          ┌──────────────────────┐
          │ Update delay counters│
          │ for BLOCKED tasks    │
          └──────────┬───────────┘
                     │
                     ↓
          ┌──────────────────────┐
          │ Unblock tasks with   │
          │ delay_ticks == 0     │
          └──────────┬───────────┘
                     │
                     ↓
          ┌──────────────────────┐
          │  Find highest        │
          │  priority READY task │
          └──────────┬───────────┘
                     │
          ┌──────────┴──────────┐
          │                     │
          ↓                     ↓
  ┌──────────────┐      ┌──────────────┐
  │ Found task   │      │ No READY     │
  │ with priority│      │ task found   │
  └──────┬───────┘      └──────┬───────┘
         │                     │
         │                     ↓
         │              ┌──────────────┐
         │              │ Round-robin  │
         │              │ next task    │
         │              └──────┬───────┘
         │                     │
         └──────────┬──────────┘
                    │
                    ↓
          ┌──────────────────────┐
          │  Update current_task │
          │  Increment switches  │
          └──────────────────────┘
```

## Task Lifecycle

```
     ┌──────────────┐
     │   Created    │
     │  (Initial)   │
     └──────┬───────┘
            │
            ↓
     ┌──────────────┐
  ┌─→│    READY     │←────┐
  │  │  (Can run)   │     │
  │  └──────┬───────┘     │
  │         │             │
  │         │ Scheduler   │
  │         │ selects     │
  │         ↓             │
  │  ┌──────────────┐     │
  │  │   RUNNING    │     │ Unblocked
  │  │ (Executing)  │     │ delay_ticks=0
  │  └──────┬───────┘     │
  │         │             │
  │         │ task_delay()│
  │         ↓             │
  │  ┌──────────────┐     │
  └──│   BLOCKED    │─────┘
     │  (Waiting)   │
     └──────┬───────┘
            │
            │ Suspend (future feature)
            ↓
     ┌──────────────┐
     │  SUSPENDED   │
     │  (Stopped)   │
     └──────────────┘
            │
            │ No auto-restart
            ↓
     ┌──────────────┐
     │  TERMINATED  │
     │  (Ended)     │
     └──────────────┘
```

## Hardware Integration (Simulator110.simu)

```
┌─────────────────────────────────────────────────────────────┐
│                      ATmega128                               │
│                                                              │
│  PORT B (Outputs - LEDs)                                    │
│  ┌──────┐                                                   │
│  │      │  PB0-PB6 ──→ Sequence LEDs (100Ω → GND)          │
│  │      │  PB7 ──────→ Heartbeat LED (100Ω → GND)          │
│  │      │                                                   │
│  │ CPU  │  PORT D (Inputs - Buttons)                       │
│  │      │  PD0-PD7 ──→ Push Buttons (pull-ups enabled)     │
│  │      │                                                   │
│  │      │  PORT F (Analog Inputs - ADC)                    │
│  │      │  PF0 ──────→ Potentiometer (0-5V)                │
│  │      │  PF1-PF7 ──→ Additional ADC channels             │
│  │      │                                                   │
│  │      │  UART1                                            │
│  │      │  PD2 (RX) ──→ UART RX                            │
│  │      │  PD3 (TX) ──→ UART TX (9600 baud)                │
│  │      │                                                   │
│  │      │  BUZZER                                           │
│  │      │  PG4 ──────→ Buzzer (Active buzzer)              │
│  │      │                                                   │
│  │Timer1│  PWM                                              │
│  │      │  PB5 (OC1A)──→ PWM Output                        │
│  │      │                                                   │
│  │      │  XTAL1/2 ──→ 16MHz Crystal                       │
│  │      │  VCC ───────→ +5V                                │
│  │      │  GND ───────→ Ground                             │
│  └──────┘                                                   │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

## Task Execution Pattern

```
Time (ms) │ Task Execution Timeline
──────────┼─────────────────────────────────────────────────
    0     │ System Start → Initialization
   10     │ Task 1 (LED) │ Task 2 (UART) │ Task 3 (ADC)
   20     │ Task 4 (Button) - HIGH PRIORITY takes precedence
   30     │ Task 5 (Motor) │ Task 6 (Watchdog)
   40     │ Task 7 (Heartbeat) - HIGH PRIORITY
   50     │ Task 8 (Stats)
  500     │ Task 1: LED pattern change
 1000     │ Task 7: Heartbeat toggle
 1500     │ Task 3: ADC reading
 2000     │ Task 2: Status report
 5000     │ Task 6: Watchdog report
10000     │ Task 8: Full statistics report
          │
          ⋮ (Pattern repeats)

Legend:
═══ Task executing
┄┄┄ Task waiting/blocked
⚡  High priority interrupt
```

## Performance Characteristics

```
Metric                    │ Value          │ Notes
──────────────────────────┼────────────────┼─────────────────
CPU Frequency             │ 16 MHz         │ Crystal oscillator
Time Slice                │ 10 ms          │ Configurable
Context Switch Overhead   │ ~10-15 μs      │ <0.15% overhead
Max Tasks                 │ 8              │ Configurable
Stack per Task            │ 256 bytes      │ Configurable
Total RTOS RAM            │ ~2536 bytes    │ 31% of 8KB RAM
Program Size (Flash)      │ 6-8 KB         │ 5-6% of 128KB
ISR Response Time         │ <5 μs          │ Hardware latency
Task Switch Frequency     │ 100 Hz         │ 100 times/sec
Timing Resolution         │ 10 ms          │ Tick resolution
Maximum Task Delay        │ 2^32 ticks     │ ~497 days @ 10ms
Priority Levels           │ 5              │ 0-4
Free RAM (typical)        │ ~5472 bytes    │ 67% available
```

## Improvements Over Basic RTOS

| Feature | Basic RTOS | Enhanced v2.0 |
|---------|------------|---------------|
| Max Tasks | 5 | 8 |
| Stack Size | 128 bytes | 256 bytes |
| Task Stats | No | Yes |
| Execution Count | No | Yes |
| Auto-Restart | No | Yes |
| Hardware Integration | Basic | Full |
| ADC Support | No | Yes |
| Buzzer Support | No | Yes |
| PWM Support | Partial | Full |
| Statistics | None | Comprehensive |
| RAM Usage | ~793 bytes | ~2536 bytes |
| Documentation | Basic | Extensive |

## System Initialization Sequence

```
1. Hardware Initialization
   ├─ UART1 @ 9600 baud
   ├─ ADC with AVCC reference
   ├─ Buzzer on PG4
   ├─ GPIO configuration
   │  ├─ PORTB: 0xFF (all outputs, LEDs)
   │  ├─ PORTD: 0x00 (all inputs, pull-ups)
   │  └─ PORTF: 0x00 (all inputs, ADC)
   └─ PWM Timer1 (10-bit resolution)

2. RTOS Initialization
   ├─ Clear task list
   ├─ Reset statistics
   ├─ Initialize system ticks
   └─ Disable scheduler

3. Task Creation
   ├─ Task 1: LED Sequence (Normal)
   ├─ Task 2: UART Status (Normal)
   ├─ Task 3: ADC Monitor (Normal)
   ├─ Task 4: Button Monitor (High)
   ├─ Task 5: Motor Control (Normal)
   ├─ Task 6: Watchdog (Low)
   ├─ Task 7: Heartbeat (High)
   └─ Task 8: Statistics (Low)

4. RTOS Start
   ├─ Initialize Timer1 (CTC mode)
   ├─ Enable global interrupts (sei)
   ├─ Enable scheduler
   └─ Enter main scheduler loop
```

## Advantages of Enhanced RTOS

### For Education
- More realistic task count (8 vs 5)
- Better hardware integration examples
- Comprehensive statistics tracking
- Clearer demonstration of priority scheduling

### For Applications
- More memory per task (256B stacks)
- Execution tracking for debugging
- Auto-restart feature for continuous tasks
- Better scalability

### For Development
- Self-contained implementation
- Easy to understand and modify
- Well-documented architecture
- Compatible with SimulIDE

## Future Enhancements

Possible additions for v3.0:
- Inter-task communication (queues, semaphores)
- Dynamic task creation/deletion
- Full preemptive context switching with register save/restore
- CPU usage calculation per task
- Task deadlock detection
- Power management integration
- Real-time guarantees
- Multi-core support (hypothetical)

## References

- ATmega128 Datasheet (Timer/Counter, Interrupts)
- FreeRTOS concepts (for comparison)
- SimulIDE documentation
- Shared Library documentation
- Real-Time Systems textbooks

