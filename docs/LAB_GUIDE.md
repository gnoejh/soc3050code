# Lab Exercises Guide

Complete guide for hands-on laboratory exercises across all ATmega128 projects.

---

## 📋 Available Labs

| Project | Exercises | Duration | Points | Topics |
|---------|-----------|----------|--------|--------|
| **Port_Basic** | 11 | 90 min | 500 | LED patterns, buttons, bit manipulation |
| **ADC_Basic** | 7 | 90 min | 800 | Sensors, calibration, data logging |
| **Graphics_Display** | 7 | 90 min | 925 | Drawing, visualization, games |
| **Serial_Polling_Char** | 7 | 75 min | 650 | UART, communication, protocols |
| **Timer_Basic** | 7 | 85 min | 725 | Timers, PWM, delays |
| **Interrupt_Basic** | 7 | 85 min | 775 | ISRs, debouncing, events |
| **CDS_Light_Sensor** | 6 | 70 min | 675 | Light sensing, auto-control |
| **LCD_Character_Basic** | 6 | 70 min | 725 | LCD display, menus |
| **Keypad_Matrix_Basic** | 7 | 90 min | 975 | Matrix scanning, input |
| **Total** | **65** | **12 hrs** | **6,750** | Full curriculum |

---

## 🚀 Quick Start

### Build and Run a Lab

**Method 1: VS Code Task (Easiest)**
1. Open project folder (e.g., `projects/Port_Basic`)
2. Open `Lab.c` file
3. Press `Ctrl+Shift+B` → Select "Build Current Project"
4. Temporarily rename: `Main.c` → `Main_backup.c`, `Lab.c` → `Main.c`
5. Build again
6. Upload to ATmega128 or use SimulIDE

**Method 2: PowerShell Script**
```powershell
# Build specific lab
cd projects\Port_Basic
cp Main.c Main.backup
cp Lab.c Main.c
..\..\cli-build-project.ps1 -ProjectDir .
cp Main.backup Main.c

# Or use SimulIDE
..\..\cli-simulide.ps1 -ProjectDir .
```

**Method 3: Manual Compile**
```bash
cd projects/Port_Basic
avr-gcc -mmcu=atmega128 -DF_CPU=16000000UL -Os -Wall \
    -I. -I../../shared_libs \
    Lab.c ../../shared_libs/*.c \
    -o Lab.elf
avr-objcopy -O ihex Lab.elf Lab.hex
avrdude -p m128 -c arduino -P COM3 -U flash:w:Lab.hex
```

---

## 📚 Lab Details

### 1. Port_Basic Lab (11 Exercises, 500 Points)

**Learning Objectives:**
- Master digital I/O and bit manipulation
- Understand button debouncing
- Create LED patterns and animations
- Build simple games with timing

**Exercises:**

**Ex1: LED Patterns (15min, 100pts)**
- Knight Rider scanner with bit shifting
- Binary counter (0-255) on LEDs
- Random sparkle pattern generator

**Ex2: Button Interactions (20min, 120pts)**
- Button-controlled LED mapping
- Reaction time game with scoring
- Memory sequence game (Simon Says)

**Ex3: Bit Manipulation (15min, 80pts)**
- Rotate bits left/right with carry
- Count set bits (population count)
- Calculate even/odd parity

**Ex4: Team Challenges (20min, 200pts)**
- Traffic light controller (4-way intersection)
- Morse code SOS transmitter

**Serial Interface:** Menu-driven, automatic scoring

---

### 2. ADC_Basic Lab (7 Exercises, 800 Points)

**Learning Objectives:**
- Read analog sensors accurately
- Calibrate for real-world values
- Implement threshold detection
- Log and analyze data

**Exercises:**

**Ex1: Calibration (25min, 200pts)**
- Multi-point temperature calibration
- Potentiometer voltage scaling (0-5V)

**Ex2: Threshold Detection (15min, 150pts)**
- Temperature alarm system
- Light level detector with hysteresis

**Ex3: Data Logging (30min, 250pts)**
- 64-sample data logger with statistics
- Noise filtering comparison (avg, median, moving)

**Ex4: Multi-Sensor Dashboard (20min, 200pts)**
- Real-time display of all sensors
- Min/max tracking
- Alert conditions

**Advanced Features:** Statistical analysis, filtering, graphing

---

### 3. Graphics_Display Lab (7 Exercises, 925 Points)

**Learning Objectives:**
- Control graphical LCD (128x64)
- Create interactive interfaces
- Visualize data in real-time
- Build simple games

**Exercises:**

**Ex1: Interactive Drawing (25min, 250pts)**
- Etch-A-Sketch with joystick
- Pattern generator (grids, spirals)

**Ex2: Data Visualization (25min, 275pts)**
- Live ADC graph plotter
- Bar chart race animation

**Ex3: UI Design (20min, 200pts)**
- Multi-level menu system
- Dashboard with gauges

**Ex4: Mini-Games (20min, 200pts)**
- Pong game with paddle control
- Score tracking on GLCD

**Hardware:** Uses joystick, buttons, ADC, GLCD

---

### 4. Serial_Polling_Char Lab (7 Exercises, 650 Points)

**Ex1:** Echo test with case conversion  
**Ex2:** Command parser with error handling  
**Ex3:** Simple calculator over UART  
**Ex4:** Data transmission protocol  
**Ex5:** Checksum validation  
**Ex6:** Multi-byte number parsing  
**Ex7:** Interactive terminal menu  

---

### 5. Timer_Basic Lab (7 Exercises, 725 Points)

**Ex1:** LED blink with Timer0  
**Ex2:** PWM brightness control  
**Ex3:** Frequency measurement  
**Ex4:** Event scheduler (multiple timers)  
**Ex5:** Stopwatch with lap times  
**Ex6:** Servo motor control  
**Ex7:** Musical tone generator  

---

### 6. Interrupt_Basic Lab (7 Exercises, 775 Points)

**Ex1:** External interrupt basics  
**Ex2:** Button debouncing with ISR  
**Ex3:** Edge detection (rising/falling)  
**Ex4:** Priority testing  
**Ex5:** ISR efficiency measurement  
**Ex6:** Event counter  
**Ex7:** Rotary encoder reading  

---

### 7. CDS_Light_Sensor Lab (6 Exercises, 675 Points)

**Ex1:** Light level reading  
**Ex2:** Auto-lighting control  
**Ex3:** Light meter with display  
**Ex4:** Day/night detection  
**Ex5:** Light intensity logger  
**Ex6:** Ambient light alarm  

---

### 8. LCD_Character_Basic Lab (6 Exercises, 725 Points)

**Ex1:** Text display basics  
**Ex2:** Scrolling messages  
**Ex3:** Custom character creation  
**Ex4:** Menu navigation  
**Ex5:** Real-time sensor display  
**Ex6:** Progress bar animation  

---

### 9. Keypad_Matrix_Basic Lab (7 Exercises, 975 Points)

**Ex1:** Key scanning basics  
**Ex2:** Password entry system  
**Ex3:** Calculator interface  
**Ex4:** Phone dialer  
**Ex5:** Game controller mapping  
**Ex6:** Multi-key detection  
**Ex7:** Lock/unlock mechanism  

---

## 🎯 Grading Rubrics

### Point Distribution

| Category | Weight | Criteria |
|----------|--------|----------|
| **Functionality** | 60% | Code works as specified |
| **Code Quality** | 20% | Clean, readable, documented |
| **Efficiency** | 10% | Optimized algorithms |
| **Creativity** | 10% | Extra features, polish |

### Lab-Specific Scoring

Each lab exercise has:
- **Base points** - Completing basic requirements
- **Bonus points** - Extra features or optimization
- **Partial credit** - Progress toward solution

Example (Port_Basic Ex2.2 - Reaction Game):
- Base (80pts): Game works, measures time
- Bonus (+20pts): Displays fastest time
- Bonus (+20pts): Multiple difficulty levels

---

## 🛠️ Common Issues

### Build Errors

**Problem:** `undefined reference to _uart_init`  
**Solution:** Include shared library files:
```bash
avr-gcc ... Lab.c ../../shared_libs/_uart.c ...
```

**Problem:** `Main.hex not found`  
**Solution:** Make sure you renamed Lab.c to Main.c before building

### Upload Errors

**Problem:** `avrdude: stk500_recv(): programmer not responding`  
**Solution:** 
- Check COM port (Device Manager)
- Reset ATmega128
- Try different baud rate

### Runtime Issues

**Problem:** Serial terminal shows nothing  
**Solution:**
- Verify baud rate: 9600 (both code and terminal)
- Check UART initialization in code
- Reset board after upload

**Problem:** LEDs don't respond  
**Solution:**
- Check DDR register (must set to output)
- Verify pin connections
- Test with simple PORTB = 0xFF

---

## 📊 Assessment Guidelines

### For Instructors

**Demonstration Requirements:**
- Student must demonstrate each exercise working
- Explain code logic and design decisions
- Answer questions about implementation

**Grading Process:**
1. **Pre-lab quiz** (10%) - Concept understanding
2. **Implementation** (60%) - Working solution
3. **Demo** (20%) - Live demonstration
4. **Report** (10%) - Documentation

**Time Limits:**
- Port_Basic: 90 minutes
- ADC/Graphics: 90 minutes  
- Others: 70-90 minutes

Allow extra time for:
- Hardware setup (first lab)
- Upload/debugging issues
- SimulIDE familiarization

### Sample Assessment

**Port_Basic Lab Grading Sheet:**
```
Exercise 1.1 - Knight Rider:    __ / 30 pts
Exercise 1.2 - Binary Counter:  __ / 35 pts
Exercise 1.3 - Random Sparkle:  __ / 35 pts
Exercise 2.1 - Button Control:  __ / 40 pts
Exercise 2.2 - Reaction Game:   __ / 40 pts
Exercise 2.3 - Memory Game:     __ / 40 pts
Exercise 3.1 - Bit Rotation:    __ / 25 pts
Exercise 3.2 - Bit Counting:    __ / 30 pts
Exercise 3.3 - Parity Check:    __ / 25 pts
Exercise 4.1 - Traffic Light:   __ / 100 pts
Exercise 4.2 - Morse Code:      __ / 100 pts
                               ============
Total Score:                    __ / 500 pts

Grade: __ %

Notes:
_________________________________
_________________________________
```

---

## 🎓 For Students

### Preparation Checklist

**Before Lab:**
- [ ] Read relevant chapters in textbook
- [ ] Review `Main.c` demo code for the project
- [ ] Test hardware connections
- [ ] Install SerialPort terminal or use SimulIDE

**During Lab:**
- [ ] Take notes on challenges encountered
- [ ] Comment your code thoroughly
- [ ] Test each exercise before moving on
- [ ] Ask for help early if stuck

**After Lab:**
- [ ] Clean up and document final code
- [ ] Write brief reflection on learning
- [ ] Review instructor's feedback
- [ ] Archive working code for reference

### Tips for Success

1. **Start Simple:** Get basic version working first
2. **Test Incrementally:** Don't write all code then test
3. **Use Serial Debug:** Print values to understand program flow
4. **Read Error Messages:** They often tell you exactly what's wrong
5. **Collaborate:** Discuss approaches but write your own code
6. **Time Management:** Allocate time per exercise, skip and return if stuck

### Sample Lab Workflow

```
1. Read exercise requirements (5 min)
2. Plan approach on paper (5 min)
3. Write code incrementally (20 min)
4. Test and debug (15 min)
5. Add bonus features if time (10 min)
6. Document and demo (5 min)
```

---

## 📞 Support

**Questions?**
- Check `FRAMEWORK_GUIDE.md` for library functions
- See `SIMULIDE_GUIDE.md` for simulation help
- Consult instructor during lab hours

**Found a bug in Lab.c?**
- Submit issue on GitHub
- Or fix it and submit pull request!

---

## ✅ Summary

**Total Lab Content:**
- 65 exercises across 9 projects
- 12 hours of hands-on learning
- 6,750 total points available
- Progressive difficulty from basic to advanced

**Quick Commands:**
```powershell
# Build lab
cp Lab.c Main.c; .\cli-build-project.ps1; cp Main.backup Main.c

# Simulate lab
.\cli-simulide.ps1 -ProjectDir "projects\Port_Basic"

# Check all labs
.\cli-build-all.ps1 -TestAll
```

**Ready to learn embedded systems hands-on!** 🎉

---

*Last Updated: October 2025*  
*9 Labs Implemented | 65 Exercises | 6,750 Points*

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
All rights reserved for educational purposes.  
**Contact:** [linkedin.com/in/gnoejh53](https://linkedin.com/in/gnoejh53)
