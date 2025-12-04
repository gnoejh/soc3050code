# Enhanced RTOS v2.0 - Quick Start Guide

## 5-Minute Setup

### Step 1: Build the Project

Open Command Prompt or PowerShell in the project directory:

```batch
cd W:\soc3050code\projects\RTOS_UPGRADED
build.bat
```

Expected output:
```
========================================
Building Enhanced RTOS v2.0
========================================

Compiling Main.c...

[SUCCESS] Compilation complete!

Generating HEX file...
[SUCCESS] HEX file created!

========================================
Build Summary
========================================
   text    data     bss     dec     hex filename
   6842     178    2548    9568    2560 Main.elf

Files Generated:
   - Main.elf (ELF executable with debug info)
   - Main.hex (Intel HEX for programming)
```

### Step 2: Open SimulIDE

1. Navigate to SimulIDE executable:
   ```
   W:\soc3050code\tools\simulide\SimulIDE_1.1.0-SR1_Win64\simulide.exe
   ```

2. Or use the launcher script:
   ```batch
   cd W:\soc3050code\tools\simulide
   SimulIDE_1.1.0-SR1_Win64\simulide.exe
   ```

### Step 3: Load Circuit

1. In SimulIDE: **File → Open**
2. Navigate to: `W:\soc3050code\tools\simulide\Simulator110.simu`
3. Click **Open**

You should see:
- ATmega128 microcontroller
- 8 LEDs on Port B with resistors
- Push buttons on Port D
- Potentiometer on Port F (ADC)
- Serial Port component
- Buzzer
- Motor control components

### Step 4: Load Firmware

1. **Right-click** on ATmega128 (the black chip labeled "atmega128-1")
2. Select **Load Firmware**
3. Navigate to: `W:\soc3050code\projects\RTOS_UPGRADED\Main.hex`
4. Click **Open**

### Step 5: Connect Serial Monitor

1. **Double-click** on the **SerialPort** component (rectangular box with "SerialPort-1706")
2. A Serial Monitor window will open
3. Verify settings:
   - Baud rate: **9600**
   - Data bits: **8**
   - Stop bits: **1**
   - Parity: **None**

### Step 6: Start Simulation

1. Click the **Play** button (▶) or press **F5**
2. Observe the Serial Monitor output
3. Watch the LED patterns on Port B

## Expected Behavior

### Serial Monitor Output

```
========================================
  ATmega128 Enhanced RTOS v2.0
  Hardware-Integrated System
  Simulator110.simu Compatible
========================================

Initializing tasks...
[OK] Task 1: LED Sequence
[OK] Task 2: UART Status
[OK] Task 3: ADC Monitor
[OK] Task 4: Button Handler
[OK] Task 5: PWM Motor
[OK] Task 6: Watchdog
[OK] Task 7: Heartbeat LED
[OK] Task 8: Statistics

All tasks initialized!

========================================
  Enhanced RTOS v2.0
  ATmega128 Real-Time OS
========================================
Tasks: 8

[Status] Report #0 | Ticks: 200 | Switches: 160
[ADC] Ch0: 512 (50%)
[PWM] Duty: 50/255
[Watchdog] System OK | RAM: 5472 bytes

=== RTOS Statistics ===
System Ticks: 1000
Context Switches: 800
Active Tasks: 8

Task Status:
  [0] LED Sequence - READY | Exec: 200
  [1] UART Status - READY | Exec: 100
  [2] ADC Monitor - READY | Exec: 100
  [3] Button Handler - READY | Exec: 1000
  [4] Motor Control - READY | Exec: 200
  [5] Watchdog - READY | Exec: 20
  [6] Heartbeat - READY | Exec: 100
  [7] Statistics - READY | Exec: 10

...
```

### Visual Indicators

1. **LED Sequence (Port B0-B6)**
   - LEDs light up in sequence
   - One LED at a time
   - Pattern repeats every 8 steps

2. **Heartbeat LED (Port B7)**
   - Blinks at 1 Hz (once per second)
   - System alive indicator

### Interactive Features

1. **Button Press**
   - Click any button on Port D
   - Serial Monitor shows: `[Button] PD0 pressed! Count: 1`
   - Buzzer beeps (short tone)
   - Counter increments

2. **ADC Reading**
   - Adjust potentiometer (if available in circuit)
   - ADC readings update every 1.5 seconds
   - Shows raw value (0-1023) and percentage

3. **Status Reports**
   - Every 2 seconds: System status
   - Every 5 seconds: Watchdog health check
   - Every 10 seconds: Full statistics

## Testing Checklist

Use this checklist to verify system operation:

- [ ] Build completes without errors
- [ ] SimulIDE loads Simulator110.simu
- [ ] Firmware loads into ATmega128
- [ ] Serial Monitor opens and shows output
- [ ] Welcome banner displays
- [ ] All 8 tasks initialize successfully
- [ ] LED sequence runs (PB0-PB6)
- [ ] Heartbeat LED blinks (PB7)
- [ ] Status reports appear every 2 seconds
- [ ] ADC readings appear every 1.5 seconds
- [ ] Button presses are detected
- [ ] Buzzer sounds on button press
- [ ] Watchdog reports every 5 seconds
- [ ] Statistics report every 10 seconds

## Common Issues and Solutions

### Issue: Build fails with "avr-gcc not found"

**Solution:**
- Verify AVR toolchain installation
- Check path in `build.bat`:
  ```batch
  set AVR_GCC=w:\soc3050code\tools\avr-toolchain\bin\avr-gcc.exe
  ```
- Adjust path if your installation differs

### Issue: SimulIDE can't find circuit file

**Solution:**
- Verify file exists: `W:\soc3050code\tools\simulide\Simulator110.simu`
- Use absolute path if necessary
- Check file permissions

### Issue: No Serial Monitor output

**Solution:**
1. Verify Serial Monitor is open (double-click SerialPort component)
2. Check baud rate: 9600
3. Restart simulation (Stop, then Play)
4. Verify firmware loaded correctly (ATmega128 should show green dot)

### Issue: LEDs not blinking

**Solution:**
1. Check simulation is running (Play button pressed)
2. Verify Port B connections in circuit
3. Check if LEDs have correct polarity
4. Reload firmware

### Issue: Buttons don't respond

**Solution:**
1. Verify Port D configured as inputs with pull-ups
2. Check button connections in circuit
3. Look for `[Button]` messages in Serial Monitor
4. Try different buttons

### Issue: Compilation warnings

**Solution:**
- Most warnings can be ignored if build succeeds
- Common warning: "unused parameter" - safe to ignore
- Warning about stack size - normal for RTOS

## Quick Reference

### Key Files

- `Main.c` - RTOS implementation and task definitions
- `Main.hex` - Compiled firmware for SimulIDE
- `Main.elf` - Debug symbols and program info
- `build.bat` - Build script
- `README.md` - Complete documentation
- `ARCHITECTURE.md` - System architecture details

### Important Paths

```
W:\soc3050code\
├── projects\RTOS_UPGRADED\      ← Your project files
├── tools\
│   ├── avr-toolchain\bin\       ← Compiler
│   └── simulide\                ← Simulator
│       └── Simulator110.simu    ← Circuit file
└── shared_libs\                 ← Shared libraries (reference)
```

### Keyboard Shortcuts (SimulIDE)

- **F5** - Start simulation
- **Ctrl+S** - Stop simulation
- **Ctrl+R** - Reset simulation
- **Ctrl+O** - Open circuit file
- **Ctrl+Q** - Quit SimulIDE

### RTOS Configuration

Edit in `Main.c`:

```c
#define MAX_TASKS 8           // Maximum concurrent tasks
#define STACK_SIZE 256        // Stack per task (bytes)
#define TIMER_TICK_MS 10      // Scheduler tick period (ms)
```

## Next Steps

Now that you have the RTOS running:

1. **Experiment with tasks**
   - Modify task delays
   - Change LED patterns
   - Adjust priority levels

2. **Add your own task**
   ```c
   void my_custom_task(void) {
       static uint32_t last_time = 0;
       if ((system_ticks - last_time) >= 100) {
           // Your code here
           last_time = system_ticks;
       }
       task_delay(10);
   }
   
   // In main():
   rtos_create_task(my_custom_task, "My Task", PRIORITY_NORMAL, true);
   ```

3. **Study the architecture**
   - Read `ARCHITECTURE.md`
   - Review task scheduling algorithm
   - Understand memory layout

4. **Explore hardware integration**
   - Connect additional sensors
   - Add more LEDs or outputs
   - Interface with other peripherals

5. **Compare with original RTOS**
   - Review `projects/RTOS/` for comparison
   - Note improvements and differences
   - Understand trade-offs

## Getting Help

If you encounter issues:

1. Check this QUICK_START.md
2. Review README.md for detailed information
3. Study ARCHITECTURE.md for system details
4. Compare with working example in `projects/RTOS/`
5. Check SimulIDE documentation
6. Verify all connections in Simulator110.simu

## Success Criteria

You'll know the system is working correctly when:

✅ Build completes successfully  
✅ All 8 tasks initialize  
✅ LEDs sequence properly  
✅ Heartbeat LED blinks at 1 Hz  
✅ Serial output appears regularly  
✅ Buttons trigger responses  
✅ ADC readings display  
✅ System statistics show activity  
✅ No error messages  
✅ RAM usage is reasonable  

**Congratulations!** You now have a working RTOS system.

---

**Time invested:** ~5-10 minutes  
**Result:** Fully functional 8-task RTOS running in simulation  
**Next:** Explore, modify, and learn!

