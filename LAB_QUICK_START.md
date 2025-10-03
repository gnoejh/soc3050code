# Lab Exercises - Quick Start Guide

## Overview

Each project now includes a **Lab.c** file containing hands-on exercises for students. This guide shows how to build and use these lab exercises.

---

## Available Labs (Currently Implemented)

✅ **Port_Basic/Lab.c** - 11 exercises on LED patterns, button control, bit manipulation  
✅ **ADC_Basic/Lab.c** - 7 exercises on sensor calibration, thresholds, data logging  
✅ **Graphics_Display/Lab.c** - 7 exercises on interactive drawing, visualization, games  

🚧 **Coming Soon**: Serial, Timer, Interrupt, and Sensor labs

---

## How to Build a Lab

### Method 1: Temporary Rename (Recommended)

```powershell
# Navigate to project directory
cd projects\Port_Basic

# Backup and swap files
mv Main.c Main.c.backup
mv Lab.c Main.c

# Build (Lab.c is now compiled as Main.c)
.\build.bat

# Restore original files
mv Main.c Lab.c
mv Main.c.backup Main.c
```

### Method 2: Modify Build Script

Edit `build.bat` in the project directory to accept a parameter:

```batch
@echo off
if "%1"=="" (
    set SOURCE=Main.c
) else (
    set SOURCE=%1
)

avr-gcc -mmcu=atmega128 -DF_CPU=7372800UL -Os -Wall -I. ^
    -I..\..\shared_libs %SOURCE% ^
    ..\..\shared_libs\_port.c ^
    ..\..\shared_libs\_init.c ^
    ..\..\shared_libs\_uart.c ^
    ..\..\shared_libs\_adc.c ^
    -o Main.elf
```

Then build with:
```powershell
.\build.bat Lab.c
```

### Method 3: Dedicated Lab Build Script

Create `build_lab.bat` in each project:

```batch
@echo off
echo Building Lab exercises...

avr-gcc -mmcu=atmega128 -DF_CPU=7372800UL -Os -Wall -I. ^
    -I..\..\shared_libs Lab.c ^
    ..\..\shared_libs\_port.c ^
    ..\..\shared_libs\_init.c ^
    ..\..\shared_libs\_uart.c ^
    ..\..\shared_libs\_adc.c ^
    ..\..\shared_libs\_glcd.c ^
    -o Lab.elf -Wl,-Map=Lab.map

if %ERRORLEVEL% EQU 0 (
    echo Build successful!
    avr-objcopy -O ihex -R .eeprom Lab.elf Lab.hex
    echo HEX file created: Lab.hex
) else (
    echo Build failed!
)
```

Then run:
```powershell
.\build_lab.bat
```

---

## How to Program the Device

### Using build_lab.bat Output

```powershell
# After building Lab.elf and Lab.hex
avrdude -c arduino -p m128 -P COM3 -b 19200 -U flash:w:Lab.hex:i
```

### Using Program Script

If you have a `program.bat`:

```powershell
# Modify to use Lab.hex instead of Main.hex
.\program.bat Lab.hex
```

---

## How to Run Lab Exercises

### 1. Connect Serial Monitor

**Using PuTTY:**
- Port: COM3 (or your device port)
- Speed: 9600
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

**Using Arduino Serial Monitor:**
- Select correct COM port
- Set baud rate to 9600
- Set line ending to "Carriage return"

**Using PowerShell:**
```powershell
mode COM3:9600,n,8,1
Get-Content COM3
```

### 2. Navigate Menu

After powering on, you'll see:

```
*************************************************
*  ATmega128 PORT PROGRAMMING LAB               *
*  Hands-On Exercises for Students              *
*************************************************

Welcome to the Port Programming Lab!
Complete all exercises to master port I/O.

========================================
  PORT PROGRAMMING - LAB EXERCISES
========================================

EXERCISE 1: LED Pattern Challenges
  1. Knight Rider Scanner
  2. Binary Counter
  3. Random Sparkle

EXERCISE 2: Button-Controlled Interactions
  4. Button-LED Control
  5. Reaction Time Game
  6. Sequence Memory Game

EXERCISE 3: Bit Manipulation
  7. Bit Rotation
  8. Bit Counting
  9. Parity Checker

EXERCISE 4: Team Challenges
  A. Traffic Light Controller
  B. Morse Code Translator

  0. Run All Exercises
  X. Exit Lab

Select exercise (1-9, A, B, 0, X):
```

### 3. Select Exercise

Press the corresponding key (1-9, A, B):

Example: Press `5` for Reaction Time Game

```
=== Lab 2.2: Reaction Time Game ===
Press the button for the lit LED as fast as possible!

Round 1/5: Get ready...
GO! Press the button!
Correct! Reaction time: 234 ms

Round 2/5: Get ready...
...

Final Score: 3840 points!
```

---

## Lab Exercise Details

### Port_Basic Lab

**Duration:** 60-90 minutes  
**Hardware Required:**
- 8 LEDs on PORTB (active low)
- 4 buttons on PORTD (PD4-7 with pull-ups)

**Key Exercises:**
1. **Knight Rider** - Scanning LED effect
2. **Reaction Game** - Test reflexes, measure timing
3. **Memory Game** - Simon-says style sequence
4. **Traffic Light** - State machine implementation
5. **Morse Code** - Timing and encoding

**Learning Outcomes:**
- Port I/O mastery
- Bit manipulation
- State machines
- Timing control

---

### ADC_Basic Lab

**Duration:** 90-120 minutes  
**Hardware Required:**
- Potentiometer on ADC0
- Temperature sensor (LM35) on ADC1
- Light sensor (CDS) on ADC2

**Key Exercises:**
1. **Temperature Calibration** - Multi-point calibration with ice water, room temp, body heat
2. **Threshold Alarm** - Monitor temperature with hysteresis
3. **Data Logger** - 64-sample circular buffer with statistics
4. **Noise Filtering** - Compare median vs moving average
5. **Multi-Sensor Dashboard** - Real-time 3-channel display

**Learning Outcomes:**
- Sensor calibration techniques
- Noise filtering methods
- Statistical analysis
- Data logging
- Threshold detection

**Special Notes:**
- Prepare ice water for calibration exercise
- Allow 2-5 seconds for sensor stabilization
- Uses enhanced ADC library functions

---

### Graphics_Display Lab

**Duration:** 90-120 minutes  
**Hardware Required:**
- 128x64 GLCD display
- Joystick on ADC0 (X) and ADC1 (Y)
- 4 buttons for controls

**Key Exercises:**
1. **Etch-A-Sketch** - Joystick drawing app
2. **Pattern Generator** - Mathematical art (spirals, Lissajous curves)
3. **Live Graph** - Real-time scrolling sensor plot
4. **Menu System** - Navigable UI with joystick
5. **Dashboard** - Multi-widget sensor display
6. **Pong Game** - Classic game with physics

**Learning Outcomes:**
- Interactive graphics programming
- Data visualization techniques
- UI/UX design principles
- Game development basics
- Animation and physics

**Special Notes:**
- Requires GLCD initialization
- Uses enhanced GLCD library
- Joystick center position: ~512 ADC
- Button logic: active low with pull-ups

---

## Troubleshooting

### Build Errors

**Error:** `undefined reference to 'Read_Adc_Median'`
- **Cause:** Enhanced ADC library not compiled
- **Solution:** Include `_adc.c` in build command
```bash
avr-gcc ... Lab.c ..\..\shared_libs\_adc.c ...
```

**Error:** `undefined reference to 'GLCD_Rectangle_Fill'`
- **Cause:** Enhanced GLCD library not compiled
- **Solution:** Include `_glcd.c` in build command
```bash
avr-gcc ... Lab.c ..\..\shared_libs\_glcd.c ...
```

**Error:** `Lab.c:45: warning: implicit declaration of function 'sqrt'`
- **Cause:** Missing math library
- **Solution:** Add `-lm` to linker flags
```bash
avr-gcc ... -lm
```

### Runtime Issues

**Issue:** Menu not displaying
- Check UART connection (TX/RX pins)
- Verify baud rate (9600)
- Ensure `Uart1_init()` called in Lab.c main()

**Issue:** Buttons not responding
- Verify pull-up resistors enabled: `PORTD |= 0xF0`
- Check button connections (PD4-7)
- Add debouncing delays (10-50ms)

**Issue:** LEDs not lighting
- Check if LEDs are active low (most boards): `LED_PORT = ~value`
- Verify DDRB set as output: `DDRB = 0xFF`
- Test with simple pattern: `PORTB = 0x00` (all on)

**Issue:** ADC readings unstable
- Use filtering: `Read_Adc_Median()` instead of `Read_Adc_Data()`
- Check ADC reference voltage
- Allow ADC to stabilize after `Adc_init()`

**Issue:** GLCD not displaying
- Verify GLCD connections (check pinout)
- Ensure `GLCD_Init()` called before drawing
- Check contrast potentiometer
- Test with simple: `GLCD_Clear(); GLCD_String(0,0,"TEST");`

**Issue:** Scoring doesn't accumulate
- Global `score` variable may be reset
- Check if `score = 0` is in loop instead of initialization
- Use `sprintf()` to debug score values via UART

---

## Customization Guide

### Adding Your Own Exercises

```c
void lab_ex_custom_exercise(void)
{
    /*
     * CHALLENGE: Your exercise title
     * TASK: What students need to do
     * LEARNING: What they will learn
     */
    
    puts_USART1("\r\n=== Lab X.X: Custom Exercise ===\r\n");
    puts_USART1("Description of exercise\r\n\r\n");
    
    // Your code here
    
    lab_score += 50;  // Award points
    puts_USART1("Exercise complete!\r\n");
}
```

### Modifying Menu

In `main()` function, add to switch statement:

```c
case 'C':
case 'c':
    lab_ex_custom_exercise();
    break;
```

And update `print_lab_menu()`:

```c
puts_USART1("EXERCISE X: Custom Exercises\r\n");
puts_USART1("  C. My Custom Exercise\r\n");
```

### Adjusting Difficulty

**Make Easier:**
- Provide partial code templates
- Increase timeout values
- Reduce number of required samples
- Add more hints in comments

**Make Harder:**
- Require code optimization
- Add performance requirements (speed, memory)
- Remove helper functions
- Add multi-step challenges

---

## Best Practices for Lab Sessions

### Before Lab
1. ✅ Test all exercises work on hardware
2. ✅ Verify serial monitor setup
3. ✅ Prepare calibration materials (ice water, thermometer)
4. ✅ Print grading rubrics
5. ✅ Have backup hardware ready

### During Lab
1. 👀 Monitor student progress
2. 💡 Provide hints, not solutions
3. 🤝 Encourage peer collaboration
4. 📝 Note common issues for discussion
5. 🎯 Keep students on time budget

### After Lab
1. 📊 Collect scores/reports
2. 💬 Discuss common challenges
3. 🏆 Recognize achievements
4. 📚 Reinforce key concepts
5. 🔮 Preview next lab

---

## Grading Examples

### Port_Basic Lab Grading

Student completes:
- Exercise 1.1 (Knight Rider): ✅ Full credit
- Exercise 2.2 (Reaction Game): ✅ Score 3200/5000
- Exercise 2.3 (Memory Game): ✅ Reached level 3/5
- Exercise 4.1 (Traffic Light): ✅ All states correct

**Automated Score:** 450/500 points  
**Code Quality:** 18/20 (good comments, clean style)  
**Collaboration:** 10/10 (helped peers)  
**Final Grade:** 478/530 = **90% (A-)**

### ADC_Basic Lab Grading

Student completes:
- Exercise 1.1 (Temp Calibration): ✅ Error < 1°C
- Exercise 2.1 (Threshold Alarm): ✅ Hysteresis working
- Exercise 3.1 (Data Logger): ✅ All statistics correct
- Exercise 3.2 (Noise Filtering): ✅ 40% noise reduction

**Automated Score:** 700/800 points  
**Analysis Report:** 22/25 (thorough, good graphs)  
**Final Grade:** 722/825 = **88% (B+)**

---

## FAQ

**Q: Can I use Lab.c with simulators?**  
A: Yes! SimulIDE and Proteus support ATmega128. Configure virtual UART and LED/button components.

**Q: How do I save my high score?**  
A: Implement EEPROM storage using `_eeprom.h` library functions to persist scores across power cycles.

**Q: Can multiple students work on one board?**  
A: Yes, exercises 4.x are designed for team collaboration. Recommend 2-3 students per board.

**Q: What if I finish early?**  
A: Try "Run All Exercises" (option 0), modify exercises for extra features, or help classmates.

**Q: How to debug exercise code?**  
A: Use UART printf to display variables, add LED indicators at checkpoints, or use simulator step-through.

**Q: Can I copy exercises to a new project?**  
A: Yes, exercises are modular. Copy exercise functions and add to menu. Update includes if different libraries needed.

---

## Additional Resources

### Documentation
- `LAB_EXERCISE_GUIDE.md` - Complete lab documentation
- `FRAMEWORK_GUIDE.md` - Overall system architecture
- `LIBRARY_ENHANCEMENTS_SUMMARY.md` - Library feature reference

### Code Structure
```
projects/
├── Port_Basic/
│   ├── Main.c          # Educational demos (theory)
│   ├── Lab.c           # Hands-on exercises (practice)
│   ├── config.h        # Configuration
│   └── build.bat       # Build script
├── ADC_Basic/
│   ├── Main.c
│   ├── Lab.c
│   └── ...
└── Graphics_Display/
    ├── Main.c
    ├── Lab.c
    └── ...
```

### Support
- GitHub Issues: Report bugs or request features
- Discussion Forum: Share custom exercises
- Email: Contact instructor for help

---

## Coming Soon

### Planned Lab Additions

🚧 **Serial Communication Lab**
- Echo server/client
- Command parsing
- Binary protocols
- Multi-device networks

🚧 **Timer Programming Lab**
- Frequency measurement
- PWM generation
- Event scheduling
- Digital stopwatch

🚧 **Interrupt Programming Lab**
- Edge detection
- Debouncing techniques
- Priority handling
- ISR optimization

🚧 **Sensor Integration Lab**
- Multi-sensor fusion
- Environmental monitoring
- Data analysis
- Visualization

---

**Last Updated:** October 3, 2025  
**Version:** 1.0  
**Status:** Port, ADC, and Graphics labs ready ✅
