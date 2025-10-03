# SimulIDE Integration Guide

## 🎯 Overview

This guide explains how to use **SimulIDE 1.1.0 SR1** as a virtual ATmega128 experiment kit. Students without access to physical hardware can develop, test, and debug their projects entirely in simulation.

---

## 📋 Table of Contents

- [Quick Start](#quick-start)
- [Why SimulIDE?](#why-simulide)
- [Setup Instructions](#setup-instructions)
- [Using SimulIDE](#using-simulide)
- [Circuit Overview](#circuit-overview)
- [Workflow Examples](#workflow-examples)
- [Troubleshooting](#troubleshooting)
- [Limitations](#limitations)

---

## 🚀 Quick Start

### Method 1: VS Code Tasks (Recommended)

1. **Open any project** (e.g., `projects/Port_Basic/Main.c`)
2. **Press** `Ctrl+Shift+B` (Build Tasks menu)
3. **Select**: `Build and Simulate Current Project`
4. **SimulIDE launches** automatically with your project loaded!

### Method 2: PowerShell Script

```powershell
# Build and simulate current project
.\cli-simulide.ps1 -ProjectDir "projects\Port_Basic"

# Just simulate (without rebuilding)
.\cli-simulide.ps1 -ProjectDir "projects\Port_Basic" -BuildFirst $false

# Simulate Lab exercises
.\cli-simulide.ps1 -ProjectDir "projects\Port_Basic" -BuildFirst $true
```

### Method 3: Manual Launch

1. Build your project to generate `Main.hex`
2. Open `SimulIDE_1.1.0-SR1_Win64\bin\simulide.exe`
3. Load circuit: `File → Open → Simulator.simu`
4. Right-click MCU → `Load Firmware` → Select your `Main.hex`
5. Click **Play** button (▶) to start simulation

---

## 🎓 Why SimulIDE?

### Benefits for Students

✅ **No Hardware Required**: Work from anywhere with just a laptop  
✅ **Instant Feedback**: See LED/LCD/motor behavior immediately  
✅ **Safe Experimentation**: Can't damage virtual components  
✅ **Built-in Debugging**: Oscilloscope, logic analyzer, serial monitor  
✅ **Cost-Effective**: Free alternative to physical kit ($0 vs $100+)  
✅ **Remote Learning**: Perfect for online courses  

### Benefits for Instructors

✅ **Consistent Environment**: All students use identical setup  
✅ **Easy Grading**: Same simulation behavior for everyone  
✅ **Quick Setup**: No hardware distribution/collection  
✅ **Advanced Tools**: Built-in measurement instruments  
✅ **Version Control**: Circuit files work in Git  

---

## 🔧 Setup Instructions

### Installation (Already Complete!)

SimulIDE is already included in your workspace:
```
soc3050code/
├── SimulIDE_1.1.0-SR1_Win64/     ← SimulIDE installation
│   └── bin/simulide.exe           ← Main executable
├── Simulator.simu                 ← ATmega128 circuit file
├── Simulator.simu                 ← Alternative circuit file
└── cli-simulide.ps1               ← Launch script
```

### Verify Installation

Run this to check:
```powershell
.\cli-simulide.ps1 -ProjectDir "projects\Port_Basic"
```

If SimulIDE doesn't launch, check:
1. `SimulIDE_1.1.0-SR1_Win64\bin\simulide.exe` exists
2. Windows hasn't blocked the executable (Right-click → Properties → Unblock)

---

## 💻 Using SimulIDE

### Basic Workflow

```
1. Write/Edit Code → 2. Build Project → 3. Launch SimulIDE → 4. Test & Debug
         ↑                                                            ↓
         └────────────────── Fix Issues ←──────────────────────────────┘
```

### SimulIDE Interface

```
┌─────────────────────────────────────────────────────────────────┐
│  File  Edit  View  Simulate  Help                         [▶][⏸]│  ← Controls
├─────────────────────────────────────────────────────────────────┤
│  Components                                                      │
│  └─ MCUs, LEDs, Sensors, etc.                                   │
├──────────────────────────┬──────────────────────────────────────┤
│                          │                                      │
│   Circuit Canvas         │   ATmega128 (MCU)                    │
│   (Drag components here) │   ├─ PORTB → LEDs (8x)               │
│                          │   ├─ PORTD → Push Buttons (8x)       │
│   LEDs                   │   ├─ ADC → Potentiometers            │
│   Buttons                │   ├─ TX/RX → Serial Terminal         │
│   Sensors                │   └─ PWM → Motor/Servo               │
│   Motors                 │                                      │
│                          │                                      │
├──────────────────────────┴──────────────────────────────────────┤
│  Serial Monitor / Oscilloscope / Logic Analyzer                 │
└─────────────────────────────────────────────────────────────────┘
```

### Key Controls

| Action | Shortcut | Description |
|--------|----------|-------------|
| **Start Simulation** | Click ▶ or `F5` | Begin running your program |
| **Pause Simulation** | Click ⏸ or `F6` | Pause execution |
| **Reset MCU** | `Ctrl+R` | Restart program from beginning |
| **Load Firmware** | Right-click MCU | Load new HEX file |
| **Zoom In/Out** | Mouse wheel | Adjust circuit view |
| **Pan View** | Middle mouse drag | Move around circuit |

### Serial Terminal

For UART communication (Lab exercises):

1. **Locate** the `SerialTerm-1700` component (bottom of circuit)
2. **Click** to open terminal window
3. **Type** to send data to ATmega128
4. **View** output from `puts_USART1()` calls

Example Lab interaction:
```
*************************************************
*  ATmega128 PORT BASIC - LAB EXERCISES         *
*************************************************

Select exercise (1-11, 0, X): 1    ← You type this
```

---

## 🔌 Circuit Overview

### ATmega128 Pinout in Circuit

```
           ATmega128
    ┌─────────────────────┐
PB0 │ 1  ┤├ LED0     VCC  │ Power
PB1 │ 2  ┤├ LED1     GND  │ Ground
PB2 │ 3  ┤├ LED2     ADC0 │ → Potentiometer (Temp)
PB3 │ 4  ┤├ LED3     ADC1 │ → Potentiometer  
PB4 │ 5  ┤├ LED4     ADC2 │ → CDS Light Sensor
PB5 │ 6  ┤├ LED5     ADC3 │ → Joystick X
PB6 │ 7  ┤├ LED6     ADC4 │ → Joystick Y
PB7 │ 8  ┤├ LED7     ADC5 │ → Accelerometer
    │                     │
PD0 │ ∧  Button0    TX1  │ → Serial Terminal
PD1 │ ∧  Button1    RX1  │ ← Serial Terminal
PD2 │ ∧  Button2    PD4  │ → Stepper Motor
PD3 │ ∧  Button3    PD5  │ → Stepper Motor
PE4 │ ∧  Button4    PD6  │ → Stepper Motor
PE5 │ ∧  Button5    PD7  │ → Stepper Motor
PE6 │ ∧  Button6         │
PE7 │ ∧  Button7    PB4  │ → Buzzer/Speaker
    │                     │
PC0 │ → LCD RS      PB5  │ → Servo Motor
PC1 │ → LCD EN      PB6  │ → DC Motor PWM
PC4-7 → LCD Data    PB7  │ (available)
    └─────────────────────┘
```

### Component Summary

| Component | Quantity | Connection | Usage |
|-----------|----------|------------|-------|
| **LEDs** | 8 | PORTB (PB0-PB7) | Output indicators |
| **Push Buttons** | 8 | PORTD/PORTE | Input switches |
| **Potentiometers** | 3 | ADC0, ADC1, ADC2 | Analog inputs |
| **Joystick (KY023)** | 1 | ADC3, ADC4 | X/Y analog input |
| **LCD Display (Ks0108)** | 1 | PORTC | Graphics display (128x64) |
| **Serial Terminal** | 1 | TX1/RX1 | UART communication |
| **DC Motor** | 1 | PB6 (PWM) | Speed control |
| **Servo Motor** | 1 | PB5 (PWM) | Position control |
| **Stepper Motor** | 1 | PD4-PD7 | Precise movement |
| **Buzzer** | 1 | PB4 | Sound generation |
| **Oscilloscope** | 1 | (probe any pin) | Waveform visualization |
| **Logic Analyzer** | 1 | (probe any pin) | Digital signal capture |

---

## 📖 Workflow Examples

### Example 1: Port_Basic Lab

**Goal**: Test LED blink and button input

```powershell
# 1. Navigate to project
cd projects\Port_Basic

# 2. Build and simulate
..\..\cli-simulide.ps1 -ProjectDir .

# 3. In SimulIDE:
#    - Click Play (▶)
#    - Open Serial Terminal
#    - Type '1' to run Exercise 1
#    - Watch LEDs blink in Knight Rider pattern
#    - Click buttons to test input
```

**Expected Behavior**:
- LEDs animate in patterns
- Serial terminal shows menu
- Buttons trigger events
- Real-time score updates

### Example 2: ADC_Basic Lab

**Goal**: Test analog sensor reading

```powershell
# Build and simulate
.\cli-simulide.ps1 -ProjectDir "projects\ADC_Basic"

# In SimulIDE:
# 1. Adjust potentiometer (click and drag slider)
# 2. Watch ADC value change in serial terminal
# 3. Test calibration exercises
```

**Expected Behavior**:
- ADC values: 0-1023 (10-bit)
- Potentiometer drag changes reading
- Real-time voltage display
- Calibration curves shown

### Example 3: Graphics_Display Lab

**Goal**: Test GLCD graphics

```powershell
# Build and simulate
.\cli-simulide.ps1 -ProjectDir "projects\Graphics_Display"

# In SimulIDE:
# 1. Select exercise from serial terminal
# 2. Use joystick to draw on LCD
# 3. Watch live graph updates
```

**Expected Behavior**:
- LCD shows 128x64 graphics
- Joystick controls cursor
- Real-time drawing
- Games playable

### Example 4: Timer_Basic Lab

**Goal**: Test PWM and timing

```powershell
# Build and simulate
.\cli-simulide.ps1 -ProjectDir "projects\Timer_Basic"

# In SimulIDE:
# 1. Run PWM dimmer exercise
# 2. Attach oscilloscope to PB6
# 3. Watch PWM waveform change
# 4. Test musical note generation
```

**Expected Behavior**:
- LED fades smoothly (PWM)
- Oscilloscope shows duty cycle
- Buzzer plays musical notes
- Accurate timing verified

---

## 🛠️ Troubleshooting

### Problem: SimulIDE won't launch

**Solutions**:
```powershell
# Check if executable exists
Test-Path "SimulIDE_1.1.0-SR1_Win64\bin\simulide.exe"

# Manually launch
Start-Process "SimulIDE_1.1.0-SR1_Win64\bin\simulide.exe"

# Check Windows Defender/Antivirus
# Right-click simulide.exe → Properties → Unblock
```

### Problem: HEX file not loading

**Solutions**:
1. **Rebuild project first**:
   ```powershell
   .\cli-build-project.ps1 -ProjectDir "projects\Port_Basic"
   ```

2. **Check HEX exists**:
   ```powershell
   Test-Path "projects\Port_Basic\Main.hex"
   ```

3. **Manual load**:
   - Right-click ATmega128 in circuit
   - Select "Load Firmware"
   - Navigate to `Main.hex`

### Problem: Serial terminal not showing output

**Solutions**:
1. **Open serial terminal**: Double-click `SerialTerm-1700` component
2. **Check baud rate**: Should be 9600 (matches UART init)
3. **Verify TX/RX connections**: Look for yellow wires to MCU
4. **Reset MCU**: Press `Ctrl+R`

### Problem: LEDs not working

**Solutions**:
1. **Check PORTB connections**: LEDs should be on PB0-PB7
2. **Verify resistors**: Each LED has 330Ω resistor
3. **Active low vs high**: LEDs might be inverted (check `PORTB = 0x00` vs `0xFF`)
4. **Simulation running?**: Click Play (▶)

### Problem: Buttons not responding

**Solutions**:
1. **Check pull-up resistors**: Buttons should have 10kΩ to VCC
2. **Verify port configuration**: DDR should be input (0)
3. **Bounce issues**: Add debounce delay in code
4. **Click correctly**: Single-click button, don't hold

### Problem: Simulation runs too fast/slow

**Solutions**:
1. **Adjust simulation speed**: `Simulate → Simulation Speed`
2. **Change time step**: `Settings → Simulation → Step Size`
3. **Pause to inspect**: Click Pause (⏸) to examine state

### Problem: Circuit file won't open

**Solutions**:
```powershell
# Use absolute path
$circuit = Resolve-Path "Simulator.simu"
Start-Process "SimulIDE_1.1.0-SR1_Win64\bin\simulide.exe" -ArgumentList "`"$circuit`""

# Or copy to SimulIDE folder
Copy-Item "Simulator.simu" "SimulIDE_1.1.0-SR1_Win64\"
```

---

## ⚠️ Limitations

### What Works Perfectly

✅ Digital I/O (GPIO, LEDs, buttons)  
✅ ADC (10-bit analog inputs)  
✅ UART (serial communication)  
✅ Timers (PWM, delays, interrupts)  
✅ External interrupts  
✅ LCD displays  
✅ Motors (DC, servo, stepper)  
✅ Basic sensors (potentiometer, joystick)  

### What Has Limitations

⚠️ **Timing precision**: Not cycle-accurate (close enough for learning)  
⚠️ **I2C/SPI**: Basic support, some devices missing  
⚠️ **EEPROM**: Simulated, but persistence needs manual save  
⚠️ **Watchdog**: Limited implementation  
⚠️ **Power modes**: Sleep modes partially supported  
⚠️ **Real-time**: Can't match exact hardware timing  

### What Doesn't Work

❌ **USB communication**: Not available  
❌ **JTAG debugging**: Use GDB simulator instead  
❌ **Some complex sensors**: Limited library  
❌ **Bluetooth/WiFi**: No wireless simulation  

### Workarounds

**For I2C/SPI**: Use software simulation or logic analyzer  
**For precise timing**: Add virtual oscilloscope probes  
**For missing components**: Create custom components (advanced)  
**For debugging**: Use serial printf debugging instead of JTAG  

---

## 📚 Additional Resources

### SimulIDE Documentation

- **Official Manual**: `SimulIDE_1.1.0-SR1_Win64\data\help\`
- **Component Library**: `SimulIDE_1.1.0-SR1_Win64\data\`
- **Examples**: `SimulIDE_1.1.0-SR1_Win64\examples\`

### ATmega128 Resources

- **Datasheet**: [ATmega128 Datasheet (PDF)](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf)
- **AVR Instruction Set**: [AVR Instruction Set Manual](https://ww1.microchip.com/downloads/en/DeviceDoc/AVR-InstructionSet-Manual-DS40002198.pdf)

### Lab Exercises

- **Lab Guide**: See `LAB_EXERCISE_GUIDE.md`
- **Quick Start**: See `LAB_QUICK_START.md`
- **Implementation**: See `LAB_IMPLEMENTATION_SUMMARY.md`

---

## 🎯 Best Practices

### For Students

1. **Always build before simulating**: Ensure HEX is up-to-date
2. **Save circuit changes**: `File → Save` if you modify circuit
3. **Use serial terminal**: Better than watching LEDs for debugging
4. **Probe with oscilloscope**: Visualize PWM and timing signals
5. **Test incrementally**: Don't write 100 lines then simulate

### For Instructors

1. **Provide master circuit**: Students should use `Simulator.simu`
2. **Standardize settings**: Same simulation speed for all
3. **Demo first**: Show SimulIDE usage before assigning
4. **Grade both**: Test on SimulIDE AND physical hardware (if available)
5. **Backup regularly**: Have students commit `.simu` files to Git

### Development Tips

```c
// Use serial debugging instead of LEDs
printf("ADC Value: %d\n", adc_value);

// Add timing markers
PORTB ^= (1 << PB7);  // Toggle pin for oscilloscope trigger

// Slow down for observation
_delay_ms(1000);  // Give time to see what's happening

// Use meaningful output
puts("Starting Lab Exercise 3...");
```

---

## 🚀 Advanced Usage

### Creating Custom Components

1. Open SimulIDE
2. `File → Create Subcircuit`
3. Design component
4. Save to `data/` folder

### Scripting Automation

```powershell
# Batch simulate all projects
Get-ChildItem "projects\*" -Directory | ForEach-Object {
    .\cli-simulide.ps1 -ProjectDir $_.FullName -BuildFirst $true
    Start-Sleep 5  # Wait for SimulIDE
    Stop-Process -Name "simulide" -Force
}
```

### Integration with CI/CD

```yaml
# Example GitHub Actions workflow
- name: Build for Simulation
  run: |
    .\cli-build-project.ps1 -ProjectDir "projects\Port_Basic"
    
- name: Verify HEX created
  run: |
    Test-Path "projects\Port_Basic\Main.hex"
```

---

## 📞 Support

### Getting Help

1. **Check this guide** first
2. **Read LAB_QUICK_START.md** for build issues
3. **Consult instructor** for course-specific questions
4. **SimulIDE forum**: [SimulIDE Support](https://simulide.forumotion.com/)

### Reporting Issues

When reporting problems, include:
- **OS Version**: Windows 10/11
- **SimulIDE Version**: 1.1.0-SR1
- **Project Name**: e.g., Port_Basic
- **Error Message**: Copy full text
- **Steps to Reproduce**: What you did before error

### Contributing

Found a bug in the circuit? Improve it!
1. Edit `Simulator.simu` in SimulIDE
2. Save changes
3. Test with multiple projects
4. Submit pull request

---

## ✅ Summary

**SimulIDE enables**:
- 🏠 Remote learning without hardware
- 💰 Zero-cost experimentation
- 🔬 Built-in measurement tools
- ⚡ Instant feedback loop
- 🎓 Safe, accessible embedded education

**Quick Commands**:
```powershell
# Build and simulate (one command!)
.\cli-simulide.ps1 -ProjectDir "projects\Port_Basic"

# Or use VS Code task (Ctrl+Shift+B)
→ Build and Simulate Current Project
```

**Ready to simulate!** 🎉

Open any project in VS Code, press `Ctrl+Shift+B`, select `Build and Simulate Current Project`, and watch your code come to life in SimulIDE!

---

*Last Updated: October 2025*  
*SimulIDE Version: 1.1.0-SR1*  
*ATmega128 Framework v2.0*
