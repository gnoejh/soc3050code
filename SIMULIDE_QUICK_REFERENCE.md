# SimulIDE Quick Reference Card

## 🚀 Start Simulating in 30 Seconds

### Step 1: Open Project
```
Open projects/Port_Basic/Main.c in VS Code
```

### Step 2: Build & Simulate
```
Press: Ctrl+Shift+B
Select: "Build and Simulate Current Project"
```

### Step 3: Run in SimulIDE
```
Click Play button (▶) in SimulIDE
Open Serial Terminal (bottom of circuit)
```

**Done!** Your code is now running in simulation! 🎉

---

## ⌨️ Essential Keyboard Shortcuts

| Action | Shortcut |
|--------|----------|
| **Start simulation** | `F5` or click ▶ |
| **Pause simulation** | `F6` or click ⏸ |
| **Reset MCU** | `Ctrl+R` |
| **Stop simulation** | `Ctrl+.` |
| **Zoom in** | `Ctrl++` |
| **Zoom out** | `Ctrl+-` |
| **Fit to view** | `Ctrl+0` |
| **Save circuit** | `Ctrl+S` |

---

## 🔌 Pin Connections Quick Guide

```
PORTB (PB0-PB7) → 8 Red LEDs
  └─ Use for OUTPUT (e.g., LED patterns, status indicators)

PORTD (PD0-PD7) → 8 Push Buttons  
  └─ Use for INPUT (e.g., user controls, switches)

ADC0-ADC5 → Analog Sensors
  ├─ ADC0: Potentiometer (Temperature)
  ├─ ADC2: CDS Light Sensor
  ├─ ADC3: Joystick X-axis
  └─ ADC4: Joystick Y-axis

TX1/RX1 → Serial Terminal
  └─ Use for: printf, menu systems, debugging

PB4 → Buzzer/Speaker
  └─ Use for: Tones, music, alerts

PB5 → Servo Motor (PWM)
  └─ Use for: Position control (0-180°)

PB6 → DC Motor (PWM)
  └─ Use for: Speed control

PORTC → LCD Display (Ks0108)
  └─ 128x64 Graphics LCD
```

---

## 🖱️ Common Actions in SimulIDE

### Load Your Program
```
1. Right-click ATmega128 chip
2. Click "Load Firmware"
3. Select your Main.hex file
4. Click "Open"
```

### View Serial Output
```
1. Double-click "SerialTerm-1700" (bottom of screen)
2. Type to send input to MCU
3. View program output here
```

### Measure Signals
```
Oscilloscope:
1. Drag probe from oscilloscope to any wire
2. Click Play to see waveform

Logic Analyzer:
1. Drag probe from analyzer to digital signals
2. Great for timing analysis
```

### Adjust Sensors
```
Potentiometers:
  Click and drag slider up/down

Joystick (KY023):
  Click and drag joystick in any direction

Buttons:
  Single-click to press (auto-release)
```

---

## 📊 Component Values

| Component | Value | Notes |
|-----------|-------|-------|
| **CPU Clock** | 16 MHz | Matches ATmega128 setting |
| **LED Resistor** | 330Ω | Current limiting |
| **Button Pullup** | 10kΩ | Prevents floating inputs |
| **Potentiometer** | 1kΩ | Analog input range: 0-5V |
| **Serial Baud** | 9600 | Must match UART init |

---

## 💡 Troubleshooting 101

### LEDs not working?
- ✅ Simulation running? (Click ▶)
- ✅ PORTB configured as output? (`DDRB = 0xFF;`)
- ✅ LED active low? (Try `PORTB = 0x00` to turn ON)

### Serial not showing?
- ✅ Terminal window open? (Double-click SerialTerm)
- ✅ Baud rate = 9600?
- ✅ UART initialized? (`Uart1_init();`)

### Buttons not responding?
- ✅ PORTD configured as input? (`DDRD = 0x00;`)
- ✅ Pull-ups enabled? (`PORTD = 0xFF;`)
- ✅ Reading correct port? (`PIND` not `PORTD`)

### Program not loading?
- ✅ HEX file exists? (Check `Main.hex` in project folder)
- ✅ Built recently? (Rebuild: `Ctrl+Shift+B`)
- ✅ Correct path? (Use "Load Firmware" menu)

---

## 📝 Code Snippets for Simulation

### Test LED Output
```c
#include "config.h"

int main(void) {
    DDRB = 0xFF;  // All outputs
    
    while(1) {
        PORTB = 0x00;  // All LEDs ON
        _delay_ms(500);
        PORTB = 0xFF;  // All LEDs OFF
        _delay_ms(500);
    }
}
```

### Test Button Input
```c
#include "config.h"

int main(void) {
    DDRB = 0xFF;   // LEDs output
    DDRD = 0x00;   // Buttons input
    PORTD = 0xFF;  // Enable pullups
    
    while(1) {
        PORTB = PIND;  // Copy buttons to LEDs
    }
}
```

### Test Serial Output
```c
#include "config.h"

int main(void) {
    Uart1_init();
    
    puts_USART1("Hello from SimulIDE!\r\n");
    
    int count = 0;
    while(1) {
        char buffer[50];
        sprintf(buffer, "Count: %d\r\n", count++);
        puts_USART1(buffer);
        _delay_ms(1000);
    }
}
```

### Test ADC Input
```c
#include "config.h"

int main(void) {
    Uart1_init();
    Adc_init();
    
    while(1) {
        uint16_t adc = Read_Adc_Data(0);  // Read ADC0
        
        char buffer[50];
        sprintf(buffer, "ADC: %u (%.2fV)\r\n", 
                adc, adc * 5.0 / 1023);
        puts_USART1(buffer);
        
        _delay_ms(500);
    }
}
```

---

## 🎯 Lab Exercise Workflow

### Before Starting Lab
```powershell
# 1. Navigate to lab project
cd projects/Port_Basic

# 2. Switch to Lab.c (or keep Main.c for demos)
# Edit: #define USE_LAB 1  (in config if available)

# 3. Build and simulate
../../cli-simulide.ps1 -ProjectDir .
```

### During Lab
```
1. Click Play (▶) in SimulIDE
2. Open Serial Terminal
3. Read menu and instructions
4. Type exercise number (e.g., '1')
5. Follow prompts
6. Test with buttons/sensors/LEDs
7. Record your score
8. Move to next exercise
```

### After Completing Lab
```
1. Take screenshot of final score
2. Save any circuit modifications
3. Commit code: git add + commit + push
4. Submit lab report with screenshots
```

---

## 🔬 Debugging Tips

### Use Serial Printf
```c
// Better than watching LEDs!
printf("Variable x = %d\n", x);
puts("Reached checkpoint A");
```

### Toggle Pin for Timing
```c
// Measure with oscilloscope
PORTB ^= (1 << PB7);  // Toggle PB7
my_function();
PORTB ^= (1 << PB7);  // Measure time between toggles
```

### Slow Down Execution
```c
// Give yourself time to observe
_delay_ms(2000);  // 2 second pause
```

### Check Pin States
```c
// Print all port states
printf("PORTB: 0x%02X\n", PORTB);
printf("PINB:  0x%02X\n", PINB);
printf("DDRB:  0x%02X\n", DDRB);
```

---

## 📚 Project Examples

### ✅ Works Great in Simulation
- Port_Basic (LED patterns, buttons)
- ADC_Basic (sensors, calibration)
- Serial_Polling_* (UART communication)
- Timer_Basic (PWM, delays)
- Interrupt_Basic (external interrupts)
- Graphics_Display (LCD drawing)
- CDS_Light_Sensor (analog input)
- LCD_Character_Basic (text display)
- Keypad_Matrix_Basic (input scanning)

### ⚠️ Partially Supported
- I2C projects (basic functionality)
- SPI projects (limited devices)
- Power management (sleep modes)

### ❌ Not Recommended for Simulation
- Projects requiring precise timing
- USB communication
- Bluetooth/WiFi modules

---

## 🎓 Student Checklist

Before each lab session:
- [ ] SimulIDE launches successfully
- [ ] Circuit file loads (`Simulator.simu`)
- [ ] Can build project (`Ctrl+Shift+B`)
- [ ] Serial terminal opens
- [ ] Understand pin connections

During lab:
- [ ] Code compiles without errors
- [ ] HEX file loads into MCU
- [ ] Serial menu appears
- [ ] Components respond correctly
- [ ] Score tracking works

After lab:
- [ ] All exercises completed
- [ ] Screenshots captured
- [ ] Code commented and clean
- [ ] Lab report written
- [ ] Files backed up/committed

---

## 🆘 Getting Help

### Self-Help (Try First!)
1. Check this quick reference
2. Read `SIMULIDE_GUIDE.md` (full manual)
3. Review `LAB_QUICK_START.md`
4. Google error messages

### Ask Instructor When:
- Circuit file won't open
- Build errors persist
- Simulation behaves unexpectedly
- Need clarification on assignment

### Include When Asking:
- Project name (e.g., "Port_Basic")
- What you tried already
- Exact error message (screenshot)
- Expected vs actual behavior

---

## 💻 VS Code Integration

### Available Tasks (Ctrl+Shift+B)
- `Build Current Project` - Compile only
- `Build and Generate HEX` - Ready for simulation
- `Simulate in SimulIDE (Current Project)` - Launch SimulIDE
- **`Build and Simulate Current Project`** ⭐ - Do everything!

### Keyboard Shortcuts
```
Ctrl+Shift+B    → Build menu
Ctrl+`          → Open terminal
Ctrl+P          → Quick file open
Ctrl+Shift+F    → Search in files
```

---

## 🌟 Pro Tips

1. **Save often**: Circuit changes can be lost
2. **Use .gitignore**: Don't commit SimulIDE temp files
3. **Name components**: Right-click → Properties → Label
4. **Color-code wires**: Right-click wire → Color
5. **Group related parts**: Select → Right-click → Create Subcircuit
6. **Test incrementally**: Don't write entire lab then test
7. **Read datasheets**: Understand component behavior
8. **Compare hardware**: If available, test both simulation and real

---

## ⚡ One-Line Commands

```powershell
# Build and simulate current project (from project folder)
..\..\cli-simulide.ps1 -ProjectDir .

# Build specific project from workspace root
.\cli-simulide.ps1 -ProjectDir "projects\Port_Basic"

# Simulate without rebuilding (if HEX exists)
.\cli-simulide.ps1 -ProjectDir "projects\ADC_Basic" -BuildFirst $false

# Just open SimulIDE circuit editor
Start-Process "SimulIDE_1.1.0-SR1_Win64\simulide.exe" -ArgumentList "Simulator.simu"
```

---

## 📖 Further Reading

- **Full Guide**: `SIMULIDE_GUIDE.md`
- **Lab Exercises**: `LAB_EXERCISE_GUIDE.md`
- **Build System**: `LAB_QUICK_START.md`
- **Framework**: `FRAMEWORK_GUIDE.md`

---

**Quick Start**: Open project → `Ctrl+Shift+B` → Build and Simulate → Click ▶ → Done! 🚀

*Simulation makes learning easier, faster, and more accessible!*

---

*SimulIDE 1.1.0-SR1 | ATmega128 Framework | October 2025*
