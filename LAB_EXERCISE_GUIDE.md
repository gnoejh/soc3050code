# ATmega128 Educational Framework - Laboratory Exercise Guide

## Overview

This document provides comprehensive guidance for the hands-on laboratory exercises included with each project in the ATmega128 Educational Framework. Each project now contains a `Lab.c` file with interactive, collective exercises designed for classroom learning.

---

## Philosophy

### Educational Structure
- **Main.c**: Educational demonstration functions teaching fundamental concepts
- **Lab.c**: Hands-on exercises for practical skill development and group activities

### Lab Design Principles
1. **Interactive**: Students actively participate, not just observe
2. **Collective**: Suitable for group/classroom activities
3. **Measurable**: Clear objectives with scoring/verification
4. **Progressive**: Difficulty increases from basic to advanced
5. **Engaging**: Game-like elements and visual feedback

---

## Available Lab Modules

### 1. Port Programming Lab (`projects/Port_Basic/Lab.c`)

**Duration**: 90 minutes  
**Difficulty**: ★☆☆☆☆ (Beginner)  
**Hardware**: 8 LEDs on PORTB, 4 buttons on PORTD

#### Exercise Overview

| Exercise | Title | Skills Learned | Duration |
|----------|-------|----------------|----------|
| 1.1 | Knight Rider Scanner | Bit shifting, delays | 5 min |
| 1.2 | Binary Counter | Number representation | 5 min |
| 1.3 | Random Sparkle | Pseudo-random generation | 5 min |
| 2.1 | Button-LED Control | Input reading, output control | 5 min |
| 2.2 | Reaction Time Game | Timing, user interaction | 10 min |
| 2.3 | Sequence Memory Game | Arrays, pattern matching | 10 min |
| 3.1 | Bit Rotation | Rotate left/right operations | 5 min |
| 3.2 | Bit Counting | Population count algorithm | 5 min |
| 3.3 | Parity Checker | Parity calculation | 5 min |
| 4.1 | Traffic Light Controller | State machines | 10 min |
| 4.2 | Morse Code Translator | Character encoding, timing | 10 min |

#### Learning Objectives
- Master port I/O operations (read/write)
- Understand bit manipulation techniques
- Implement state machine logic
- Create timing-based applications
- Develop interactive user interfaces

#### Grading Rubric (Total: 100 points)
- Exercise completion: 60 points (5-10 points each)
- Code quality: 20 points
- Creativity in modifications: 10 points
- Team collaboration: 10 points

#### Sample Exercise: Reaction Time Game

```c
void lab_ex2_reaction_game(void)
{
    // Lights random LED, measures button press speed
    // Calculates score based on reaction time
    // 5 rounds with cumulative scoring
    
    // Learning outcomes:
    // - Timing measurement
    // - Random number generation
    // - User input handling
    // - Score calculation
}
```

**Expected Results**:
- Average reaction time: 200-500ms
- Students learn timing precision
- Competitive element engages students

---

### 2. ADC Programming Lab (`projects/ADC_Basic/Lab.c`)

**Duration**: 90 minutes  
**Difficulty**: ★★★☆☆ (Intermediate)  
**Hardware**: Potentiometer (ADC0), Temperature sensor (ADC1), Light sensor (ADC2)

#### Exercise Overview

| Exercise | Title | Skills Learned | Duration |
|----------|-------|----------------|----------|
| 1.1 | Temperature Calibration | Multi-point calibration | 20 min |
| 1.2 | Potentiometer Calibration | Linear scaling | 10 min |
| 2.1 | Temperature Alarm System | Threshold detection, hysteresis | 10 min |
| 2.2 | Light Level Detector | Multi-level classification | 10 min |
| 3.1 | Sensor Data Logger | Circular buffers, statistics | 15 min |
| 3.2 | Noise Filtering Comparison | Median vs moving average | 15 min |
| 4.1 | Multi-Sensor Dashboard | Real-time data display | 10 min |

#### Learning Objectives
- Perform sensor calibration with reference points
- Implement threshold-based monitoring
- Apply noise filtering techniques
- Calculate statistical measures (min, max, avg, std dev)
- Create data logging systems
- Build real-time dashboards

#### Key Features
- **Calibration Tables**: 10-point lookup with interpolation
- **Statistics**: Min, max, average, standard deviation, histograms
- **Filtering**: Median filter, exponential moving average
- **Logging**: 64-sample circular buffer
- **Visualization**: UART-based bar graphs and dashboards

#### Grading Rubric (Total: 100 points)
- Calibration accuracy: 25 points
- Threshold implementation: 20 points
- Data analysis quality: 25 points
- Dashboard presentation: 20 points
- Documentation: 10 points

#### Sample Exercise: Temperature Calibration

```c
void lab_ex1_temperature_calibration(void)
{
    // 1. Measure ADC at 0°C (ice water)
    // 2. Measure ADC at room temperature
    // 3. Measure ADC at body temperature
    // 4. Create calibration lookup table
    // 5. Test with 10 readings
    
    // Learning outcomes:
    // - Calibration methodology
    // - Linear interpolation
    // - Error analysis
    // - Practical sensor handling
}
```

**Expected Results**:
- Calibration error < 2°C
- Students understand non-linearity
- Hands-on sensor experience

---

### 3. Graphics Programming Lab (`projects/Graphics_Display/Lab.c`)

**Duration**: 90 minutes  
**Difficulty**: ★★★★☆ (Intermediate-Advanced)  
**Hardware**: 128x64 GLCD, Joystick (ADC0-1), 4 buttons

#### Exercise Overview

| Exercise | Title | Skills Learned | Duration |
|----------|-------|----------------|----------|
| 1.1 | Etch-A-Sketch | Interactive drawing, input handling | 15 min |
| 1.2 | Pattern Generator | Parametric equations, math art | 10 min |
| 2.1 | Live Data Graph | Real-time plotting, scrolling | 15 min |
| 2.2 | Bar Chart Race | Multi-data visualization | 10 min |
| 3.1 | Menu System | UI navigation, selection | 10 min |
| 3.2 | Dashboard Design | Layout, information hierarchy | 10 min |
| 4.1 | Pong Game | Game logic, physics, collision | 20 min |

#### Learning Objectives
- Create interactive drawing applications
- Implement data visualization (graphs, charts)
- Design professional user interfaces
- Build menu navigation systems
- Develop game logic and physics
- Understand coordinate systems and transformations

#### Enhanced GLCD Features Used
- `GLCD_Rectangle_Fill()`: Solid shapes for paddles, highlights
- `GLCD_Circle_Fill()`: Ball rendering
- `GLCD_Bar_Horizontal/Vertical()`: Data visualization
- `GLCD_Progress_Bar()`: Status indicators
- `GLCD_String_Large()`: Prominent text display
- `GLCD_Icon_8x8()`: Status icons

#### Grading Rubric (Total: 100 points)
- Interactive drawing: 20 points
- Data visualization: 25 points
- UI design quality: 25 points
- Game functionality: 20 points
- Code organization: 10 points

#### Sample Exercise: Pong Game

```c
void lab_ex4_pong_game(void)
{
    // Single-player pong with joystick paddle control
    // Ball physics: velocity, collision detection
    // Scoring system with lives
    // Progressive difficulty
    
    // Learning outcomes:
    // - Game loop architecture
    // - Collision detection algorithms
    // - Physics simulation (velocity, bounce)
    // - Score tracking and lives system
}
```

**Expected Results**:
- Smooth 20 FPS gameplay
- Accurate collision detection
- Students understand game development basics

---

## Lab Session Workflow

### Pre-Lab (10 minutes)
1. **Setup Check**: Verify hardware connections
2. **Theory Review**: Brief concept recap from Main.c demos
3. **Objective Explanation**: What students will build

### During Lab (60-70 minutes)
1. **Individual Exercises**: Students work through exercises 1-3
2. **Peer Review**: Students compare results
3. **Team Challenges**: Collaborative exercises (Exercise 4)
4. **Debugging Session**: Instructor helps with issues

### Post-Lab (10 minutes)
1. **Results Discussion**: What worked, what didn't
2. **Score Tabulation**: Calculate final scores
3. **Concept Reinforcement**: Key takeaways
4. **Preview Next Lab**: Brief introduction

---

## Building and Running Labs

### Compilation

Each Lab.c is standalone and uses the same build system:

```bash
# Navigate to project directory
cd projects/Port_Basic

# Build Lab.c instead of Main.c
# Temporarily rename files or modify build script

# Option 1: Rename approach
mv Main.c Main.c.demo
mv Lab.c Main.c
.\build.bat
mv Main.c Lab.c
mv Main.c.demo Main.c

# Option 2: Modify build script to accept parameter
# (Requires build script enhancement)
.\build.bat Lab.c
```

### Programming Device

```bash
# After building Lab.elf
.\program.bat
```

### Serial Monitor

All labs provide menu and feedback via UART at 9600 baud:

```bash
# Windows
mode COM3:9600,n,8,1
type COM3

# Or use terminal emulator (PuTTY, TeraTerm)
```

---

## Instructor Guidelines

### Preparation Checklist

- [ ] Verify all hardware connections
- [ ] Test each exercise before class
- [ ] Prepare calibration references (ice water, thermometer)
- [ ] Set up serial monitors for each workstation
- [ ] Print grading rubrics
- [ ] Prepare extension challenges for fast learners

### Common Issues & Solutions

#### Port Programming Lab
- **Issue**: LEDs not lighting
  - **Solution**: Check active-low configuration, verify PORTB connections
  
- **Issue**: Button not responding
  - **Solution**: Verify pull-up resistors, check debouncing delay

#### ADC Programming Lab
- **Issue**: Noisy readings
  - **Solution**: Use median filter, check ADC reference voltage
  
- **Issue**: Calibration inaccurate
  - **Solution**: Allow sensor stabilization time, verify reference temperatures

#### Graphics Programming Lab
- **Issue**: Display not updating
  - **Solution**: Check GLCD initialization, verify SPI connections
  
- **Issue**: Flickering graphics
  - **Solution**: Reduce refresh rate, use double buffering concepts

### Extension Challenges

For advanced students who complete early:

1. **Port Lab**: Add multiple difficulty levels to reaction game
2. **ADC Lab**: Implement Kalman filter for noise reduction
3. **Graphics Lab**: Add two-player mode to Pong game

---

## Assessment Methods

### Formative Assessment (During Lab)
- Observe student progress through exercises
- Check intermediate results
- Evaluate problem-solving approach
- Monitor collaboration quality

### Summative Assessment (End of Lab)
- Final score from exercises
- Code review checklist
- Demonstration to instructor
- Written reflection (optional)

### Sample Assessment Form

```
Student Name: _______________   Date: _______________

Exercise Completion:
[ ] Exercise 1 (Basic): ___/20 points
[ ] Exercise 2 (Intermediate): ___/25 points  
[ ] Exercise 3 (Advanced): ___/25 points
[ ] Exercise 4 (Team Challenge): ___/20 points

Code Quality:
[ ] Proper formatting: ___/5 points
[ ] Comments: ___/5 points

Collaboration:
[ ] Teamwork: ___/5 points

Total: ___/100 points
Grade: _____
```

---

## Lab Variations

### Time-Constrained Version (45 minutes)
- Focus on Exercises 1-2 only
- Provide partial code templates
- Skip calibration exercises

### Advanced Version (120 minutes)
- All exercises required
- Add extension challenges
- Require code optimization
- Include written analysis

### Remote Learning Version
- Use simulator (Proteus, SimulIDE)
- Video demonstration of results
- Breakout rooms for team exercises
- Shared screen debugging

---

## Scoring System

Each lab implements a scoring system to gamify learning:

### Port Programming Lab
- **Max Score**: ~500 points
- Knight Rider: 50 points
- Binary Counter: 50 points
- Random Sparkle: 50 points
- Button Control: 75 points
- Reaction Game: 100 points (time-based)
- Memory Game: 150 points (level-based)
- Traffic Light: 100 points
- Morse Code: 75 points

### ADC Programming Lab
- **Max Score**: ~800 points
- Temperature Calibration: 100 points
- Potentiometer Calibration: 50 points
- Temperature Alarm: 100 points (alarm-based)
- Light Level Detector: 75 points
- Data Logger: 150 points
- Noise Filtering: 125 points
- Multi-Sensor Dashboard: 200 points

### Graphics Programming Lab
- **Max Score**: ~925 points
- Etch-A-Sketch: 100 points
- Pattern Generator: 75 points
- Live Graph: 125 points
- Bar Chart Race: 100 points
- Menu System: 150 points
- Dashboard Design: 175 points
- Pong Game: 200 points

---

## Future Lab Expansions

### Planned Labs

1. **Serial Communication Lab** (`Lab.c` for Serial projects)
   - Exercise 1: Echo server/client
   - Exercise 2: Command parser implementation
   - Exercise 3: Binary protocol design
   - Exercise 4: Multi-device communication

2. **Timer Programming Lab** (`Lab.c` for Timer_Basic)
   - Exercise 1: Frequency counter
   - Exercise 2: PWM signal generator
   - Exercise 3: Event scheduler
   - Exercise 4: Digital stopwatch

3. **Interrupt Programming Lab** (`Lab.c` for Interrupt_Basic)
   - Exercise 1: Edge detection
   - Exercise 2: Button debouncing
   - Exercise 3: Priority handling
   - Exercise 4: ISR efficiency testing

4. **Sensor Integration Lab** (`Lab.c` for sensor projects)
   - Exercise 1: CDS light sensor characterization
   - Exercise 2: Accelerometer tilt detection
   - Exercise 3: Multi-sensor fusion
   - Exercise 4: Environmental monitoring system

---

## Technical Notes

### Memory Considerations

Lab.c files are larger than Main.c demos due to multiple exercises:

- **Port_Basic Lab.c**: ~8KB code
- **ADC_Basic Lab.c**: ~12KB code
- **Graphics_Display Lab.c**: ~15KB code

ATmega128 has 128KB flash, so all labs fit comfortably with libraries.

### UART Buffer Requirements

Labs use extensive UART output:
- Ensure 128-byte RX buffer (default)
- TX buffer can overflow with rapid menu display
- Consider increasing baud rate to 19200 for faster response

### Timing Accuracy

Reaction time and game timing use `_delay_ms()`:
- Accurate to ~±5ms at 7.3728 MHz
- For precise timing, use Timer1 with CTC mode
- Consider Timer0 overflow interrupt for game loop

---

## Resources

### Related Documentation
- `FRAMEWORK_GUIDE.md`: Overall framework architecture
- `LIBRARY_ENHANCEMENTS_SUMMARY.md`: Enhanced library features
- `GLCD_LIBRARY_ENHANCEMENTS.md`: Graphics functions reference

### Code Examples
- `Main.c` in each project: Theoretical demonstrations
- `Lab.c` in each project: Practical exercises
- `shared_libs/`: Library implementations

### Troubleshooting
- Check `build.log` for compilation errors
- Use serial monitor for runtime debugging
- Verify hardware with `Main.c` demos first

---

## Appendix A: Quick Reference

### Lab Menu Access
All labs use serial interface:
1. Power on device
2. Open serial monitor (9600 baud, 8N1)
3. View menu
4. Press number key to select exercise
5. Follow on-screen instructions

### Common Serial Commands
- `1-9, A-B`: Select exercise number
- `0`: Run all exercises
- `Q`: Quit/exit current exercise
- `X`: Exit lab completely

### Hardware Connections Summary

| Component | Port/Pin | Project |
|-----------|----------|---------|
| LEDs (8) | PORTB (PB0-7) | Port_Basic |
| Buttons (4) | PORTD (PD4-7) | All labs |
| Potentiometer | ADC0 | ADC_Basic, Graphics |
| Temperature | ADC1 | ADC_Basic |
| Light Sensor | ADC2 | ADC_Basic |
| Joystick X | ADC0 | Graphics |
| Joystick Y | ADC1 | Graphics |
| GLCD Data | PORTA | Graphics |
| GLCD Control | PORTC | Graphics |
| UART TX | PD1 | All labs |
| UART RX | PD0 | All labs |

---

## Appendix B: Sample Lab Report Template

```
ATmega128 Educational Framework - Lab Report

Student: _______________
Date: _______________
Project: _______________

SECTION 1: OBJECTIVES
What were the learning objectives for this lab?
[Answer]

SECTION 2: PROCEDURE
Briefly describe the exercises completed.
[Answer]

SECTION 3: RESULTS
Exercise 1: [Score/Observations]
Exercise 2: [Score/Observations]
Exercise 3: [Score/Observations]
Exercise 4: [Score/Observations]

SECTION 4: ANALYSIS
What concepts did you learn?
[Answer]

What challenges did you encounter?
[Answer]

How did you solve them?
[Answer]

SECTION 5: CONCLUSION
Overall learning outcome:
[Answer]

Suggestions for improvement:
[Answer]

Total Score: ___/100
```

---

## Revision History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 2025-10-03 | Initial release: Port, ADC, Graphics labs | Framework Team |
| 1.1 | TBD | Add Serial, Timer, Interrupt labs | Planned |
| 1.2 | TBD | Add sensor integration labs | Planned |

---

**End of Laboratory Exercise Guide**

For questions or suggestions, contact the educational framework maintainer.
