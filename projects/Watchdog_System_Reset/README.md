# Watchdog Timer - System Reset and Recovery

**Project**: Watchdog_System_Reset  
**Target**: ATmega128 @ 16 MHz  
**Features**: Automatic crash detection, system reset, recovery mechanisms

---

## Overview

This project demonstrates the **Watchdog Timer** on the ATmega128, a critical hardware safety mechanism that provides automatic recovery from software crashes and system hangs. The watchdog timer operates independently of the main CPU clock and will force a system reset if not periodically cleared, making it essential for reliable embedded systems.

### Key Features

- ✅ **Independent hardware timer** - Runs on dedicated ~1 MHz RC oscillator
- ✅ **Automatic crash recovery** - Resets system if software hangs
- ✅ **Configurable timeout** - 8 timeout periods from 16ms to 2.1 seconds
- ✅ **Reset source detection** - Identify cause of reset (watchdog, power-on, etc.)
- ✅ **Recovery mechanisms** - Track and respond to watchdog resets

---

## Hardware Requirements

### System Configuration

- **Microcontroller**: ATmega128
- **System Clock**: 16 MHz crystal oscillator
- **Watchdog Clock**: Independent ~1 MHz RC oscillator
- **UART**: UART1 @ 9600 baud for monitoring
- **LEDs**: PORTC for status indication

### Pin Assignment

- **UART1 TX**: PD3 - Serial output for monitoring
- **UART1 RX**: PD2 - Serial input for control
- **Status LEDs**: PORTC (PC0-PC7) - Visual status indicators

---

## SimulIDE Compatibility

| Version | Status | Notes |
|---------|--------|-------|
| **SimulIDE 1.1.0-SR1** | ⚠️ **Limited** | Watchdog timer not fully functional |
| **SimulIDE 0.4.15** | ⚠️ **Limited** | Watchdog timer not fully functional |
| **Hardware (ATmega128)** | ✅ **Fully Functional** | Complete watchdog support |

**Important**: Watchdog timers are often not accurately simulated in simulators. For reliable testing of watchdog functionality, **use real hardware**.

---

## Theory of Operation

### Watchdog Timer Architecture

The watchdog timer is a separate hardware timer that runs independently of the CPU:

```
Main CPU @ 16 MHz              Watchdog @ ~1 MHz
┌──────────────┐              ┌────────────────┐
│ Application  │              │ WDT Counter    │
│   Code       │─wdt_reset()→│                │
│              │              │ Prescaler      │
└──────────────┘              └────────────────┘
                                      ↓
                              Timeout? → System Reset
```

### How It Works

1. **Enable Watchdog** with chosen timeout period
2. **Execute Normal Code** - do your work
3. **Periodically Call** `wdt_reset()` to prevent timeout
4. **If Code Hangs** - watchdog not reset → timeout occurs
5. **System Resets** automatically
6. **Check MCUCSR** register to detect watchdog reset

### Watchdog Timeout Periods

The watchdog timer can be configured for 8 different timeout periods:

| WDP[2:0] | Timeout | Oscillator Cycles | Use Case |
|----------|---------|-------------------|----------|
| 000 | 16.3 ms | 16K | Very fast loops |
| 001 | 32.5 ms | 32K | Quick tasks |
| 010 | 65 ms | 64K | Normal tasks |
| 011 | 130 ms | 128K | Default |
| 100 | 260 ms | 256K | Moderate tasks |
| 101 | 520 ms | 512K | Slow tasks |
| 110 | 1.0 s | 1M | Long operations |
| 111 | 2.1 s | 2M | Very long tasks |

**Selection Rule**: Choose timeout > 2× your longest task execution time.

---

## Register Configuration

### WDTCR - Watchdog Timer Control Register

```
Bit:    7    6    5    4    3    2    1    0
Name:   -    -    -   WDCE  WDE  WDP2 WDP1 WDP0
```

**Bits:**

- **WDCE** (Watchdog Change Enable): Must be set with WDE to change settings
- **WDE** (Watchdog Enable): 1 = enabled, 0 = disabled
- **WDP[2:0]** (Watchdog Prescaler): Select timeout period (000-111)

### MCUCSR - MCU Control and Status Register

```
Bit:    7    6    5    4    3    2    1    0
Name:   -    -    -    -   WDRF BORF EXTRF PORF
```

**Reset Flags:**

- **WDRF** (Bit 3): Watchdog Reset Flag
- **BORF** (Bit 2): Brown-out Reset Flag
- **EXTRF** (Bit 1): External Reset Flag
- **PORF** (Bit 0): Power-on Reset Flag

**Important**: Flags persist across resets and must be manually cleared!

---

## Enabling the Watchdog

### Critical Timing Sequence

Enabling/disabling the watchdog requires a precise timed sequence:

```c
#include <avr/wdt.h>

void watchdog_enable(uint8_t timeout) {
    cli();  // Disable interrupts - CRITICAL!
    
    wdt_reset();  // Reset watchdog first
    
    // Timed sequence (MUST complete within 4 clock cycles)
    WDTCR = (1 << WDCE) | (1 << WDE);        // Step 1: Enable change mode
    WDTCR = (1 << WDE) | (timeout & 0x07);   // Step 2: Set new timeout
    
    sei();  // Re-enable interrupts
}
```

**⚠️ Warning**: The second write to WDTCR must occur within 4 system clock cycles of the first write, or the change will be ignored!

### Using AVR-LibC Macros

For simpler code, use the AVR-LibC watchdog macros:

```c
#include <avr/wdt.h>

// Enable with 1 second timeout
wdt_enable(WDTO_1S);

// Reset (clear) the watchdog
wdt_reset();

// Disable watchdog
wdt_disable();
```

---

## Disabling the Watchdog

Disabling the watchdog also requires a timed sequence AND clearing the WDRF flag:

```c
void watchdog_disable(void) {
    cli();
    wdt_reset();
    
    // IMPORTANT: Clear WDRF flag first!
    MCUCSR &= ~(1 << WDRF);
    
    // Timed sequence to disable
    WDTCR = (1 << WDCE) | (1 << WDE);  // Enable change mode
    WDTCR = 0x00;                       // Turn off WDE
    
    sei();
}
```

**⚠️ Critical**: If WDRF is not cleared, the watchdog may automatically re-enable after reset!

---

## Demos Included

### Demo 1: Basic Watchdog Reset

**Purpose**: Demonstrate basic watchdog timeout and system reset  
**Method**: Enable watchdog, wait for timeout without clearing  
**Learning**: See watchdog reset in action

```c
// Enable watchdog with selected timeout
watchdog_enable(WDT_1S);

// Wait... system will reset when watchdog times out!
while (1) {
    // NOT calling wdt_reset() - system will reset!
}
```

**Expected Result**: System resets after timeout period.

### Demo 2: Periodic Watchdog Reset

**Purpose**: Show proper watchdog usage in normal operation  
**Method**: Clear watchdog periodically during normal execution  
**Learning**: How to maintain watchdog protection

```c
watchdog_enable(WDT_1S);

while (1) {
    // Do work (must complete < 1 second)
    do_task();
    
    // Clear watchdog (CRITICAL!)
    wdt_reset();
    
    // Continue normal operation
}
```

**Key Point**: Call `wdt_reset()` at least every timeout period.

### Demo 3: Simulated System Hang

**Purpose**: Demonstrate automatic recovery from crash  
**Method**: Normal operation → simulate infinite loop → watchdog resets system  
**Learning**: Watchdog provides automatic crash recovery

```c
watchdog_enable(WDT_2S);

// Normal operation with watchdog clearing
for (int i = 0; i < 50; i++) {
    do_work();
    wdt_reset();  // Clear regularly
}

// Simulate hang - infinite loop WITHOUT wdt_reset()
while (1) {
    // System hangs here...
    // Watchdog NOT cleared → timeout → reset!
}
```

**Expected Result**: System hangs briefly, then watchdog forces reset.

### Demo 4: Reset Recovery System

**Purpose**: Detect and respond to watchdog resets  
**Method**: Check MCUCSR register, track reset count, take recovery action  
**Learning**: Building robust recovery mechanisms

```c
int main(void) {
    // Check if recovering from watchdog reset
    if (MCUCSR & (1 << WDRF)) {
        // Recovered from watchdog reset!
        handle_crash_recovery();
    }
    
    // Clear reset flags
    MCUCSR = 0;
    
    // Normal operation...
}

void handle_crash_recovery(void) {
    // Log error
    // Flash warning LEDs
    // Enter safe mode
    // Notify user
}
```

**Use Case**: Production systems that need automatic recovery.

---

## Best Practices

### ✅ DO

1. **Disable watchdog first in main()**

   ```c
   int main(void) {
       MCUCSR &= ~(1 << WDRF);
       wdt_disable();
       // ... rest of initialization
   }
   ```

2. **Clear watchdog regularly**

   ```c
   while (1) {
       do_task();
       wdt_reset();  // Don't forget!
   }
   ```

3. **Check reset source**

   ```c
   if (MCUCSR & (1 << WDRF)) {
       // Handle watchdog reset
   }
   MCUCSR = 0;  // Clear flags
   ```

4. **Choose appropriate timeout**
   - Timeout should be > 2× longest task time
   - Include safety margin for interrupts

5. **Test on real hardware**
   - Simulators often don't support watchdog
   - Verify actual reset behavior

### ❌ DON'T

1. **Don't forget to clear watchdog**
   - Will cause unintended resets
   - Place `wdt_reset()` strategically

2. **Don't use too short timeout**
   - Must allow time for all tasks
   - Account for interrupt latency

3. **Don't forget to clear WDRF**
   - Watchdog may re-enable after reset
   - Always clear before disabling

4. **Don't ignore reset flags**
   - Check MCUCSR for diagnostics
   - Log resets for debugging

5. **Don't test only in simulator**
   - Watchdog often not simulated
   - Always test on hardware

---

## Common Issues and Solutions

### Issue 1: Watchdog keeps resetting system

**Symptoms**: Continuous reset loop  
**Causes**:

- Timeout too short for task execution
- Forgot to call `wdt_reset()`
- Long interrupt blocking watchdog clear

**Solutions**:

```c
// Increase timeout period
watchdog_enable(WDT_2S);  // Instead of WDT_260MS

// Ensure wdt_reset() called frequently
while (1) {
    task1();
    wdt_reset();  // Add more clears
    task2();
    wdt_reset();
}
```

### Issue 2: Can't disable watchdog after reset

**Symptoms**: Watchdog re-enables automatically  
**Cause**: WDRF flag not cleared

**Solution**:

```c
// ALWAYS clear WDRF before disabling!
MCUCSR &= ~(1 << WDRF);
wdt_disable();
```

### Issue 3: Reset source unknown

**Symptoms**: Can't tell if watchdog caused reset  
**Cause**: Reset flags cleared too early

**Solution**:

```c
// Check MCUCSR BEFORE clearing
uint8_t reset_source = MCUCSR;
if (reset_source & (1 << WDRF)) {
    handle_watchdog_reset();
}
// Now clear flags
MCUCSR = 0;
```

### Issue 4: Doesn't work in simulator

**Symptoms**: Watchdog has no effect in SimulIDE  
**Cause**: Watchdog not fully simulated

**Solution**: Test on real ATmega128 hardware!

---

## Building and Running

### Build for Hardware

```powershell
# Navigate to project
cd w:\soc3050code\projects\Watchdog_System_Reset

# Build
.\build.bat

# Program to ATmega128
avrdude -c arduino -p m128 -P COM3 -U flash:w:Main.hex:i
```

### Testing

1. Connect UART1 to serial terminal (9600 baud)
2. Observe LEDs on PORTC
3. Follow menu prompts to test different demos
4. Monitor watchdog behavior via serial output

---

## Learning Outcomes

After completing this project, you will understand:

✅ **Watchdog Timer Fundamentals**

- Independent hardware timer operation
- Timeout period configuration
- System reset mechanism

✅ **Critical Timing Sequences**

- Timed enable/disable sequences
- 4-cycle deadline importance
- Interrupt management during configuration

✅ **Reset Detection**

- MCUCSR register flags
- Distinguishing reset sources
- Persistent flag behavior

✅ **Recovery Mechanisms**

- Detecting watchdog resets
- Implementing recovery actions
- Building robust systems

✅ **Practical Applications**

- Preventing system hangs
- Automatic crash recovery
- Production reliability features

---

## Applications

### Industrial Control Systems

- Prevent PLC crashes
- Automatic restart on fault
- Maintain process continuity

### Remote/Unattended Systems

- Recover from software bugs
- Reset on communication timeout
- Ensure availability

### Safety-Critical Systems

- Detect and recover from failures
- Meet reliability requirements
- Comply with safety standards

### Consumer Electronics

- Improve reliability
- Reduce support calls
- Better user experience

---

## References

- **ATmega128 Datasheet**: [Section 19 - Watchdog Timer](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf) (Pages 56-58)
- **AVR-LibC**: [Watchdog Timer Documentation](https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html)
- See `Slide.md` for complete theory and diagrams

---

## Related Projects

- **Timer_Normal** - Basic timer operations
- **Timer1_CTC_Precision** - Precision timing
- **INT_External_Pins** - Interrupt handling

---

**Last Updated**: December 1, 2025  
**Author**: SOC 3050 Embedded Systems Course  
**License**: Educational Use
