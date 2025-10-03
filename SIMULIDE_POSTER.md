# 🎯 SimulIDE ATmega128 - Visual Quick Start Poster

```
╔══════════════════════════════════════════════════════════════════════════════╗
║                                                                              ║
║               🎓 ATmega128 SimulIDE Quick Start Guide                        ║
║                                                                              ║
║                     Get Running in 30 Seconds! 🚀                            ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝


┌──────────────────────────────────────────────────────────────────────────────┐
│  STEP 1: Open Your Project in VS Code                                       │
└──────────────────────────────────────────────────────────────────────────────┘

    📁 projects/Port_Basic/Main.c    ← Click to open
    
    OR double-click any .c file in projects folder


┌──────────────────────────────────────────────────────────────────────────────┐
│  STEP 2: Build and Simulate (One Button!)                                   │
└──────────────────────────────────────────────────────────────────────────────┘

    Press:  Ctrl + Shift + B
    
    Select: ▶ Build and Simulate Current Project
    
    Wait:   3-5 seconds (compiling + launching SimulIDE)


┌──────────────────────────────────────────────────────────────────────────────┐
│  STEP 3: Run in SimulIDE                                                    │
└──────────────────────────────────────────────────────────────────────────────┘

    Click:  ▶ Play Button (top toolbar)
    
    Open:   Serial Terminal (double-click at bottom)
    
    Watch:  LEDs blink, code runs!  ✨


═══════════════════════════════════════════════════════════════════════════════

                          KEYBOARD SHORTCUTS
                          
┌─────────────────┬──────────────────────────────────────────────────────────┐
│  F5             │  Start simulation                                        │
│  F6             │  Pause simulation                                        │
│  Ctrl + R       │  Reset MCU (restart program)                             │
│  Ctrl + .       │  Stop simulation                                         │
│  Mouse Wheel    │  Zoom in/out                                             │
│  Middle Drag    │  Pan around circuit                                      │
└─────────────────┴──────────────────────────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════════════

                        PIN CONNECTIONS MAP
                        
        ┌─────────────────────────────────┐
        │       ATmega128 Chip            │
        │                                 │
  PB0   ├─  LED 0  (Red)                 │
  PB1   ├─  LED 1  (Red)                 │
  PB2   ├─  LED 2  (Red)                 │
  PB3   ├─  LED 3  (Red)                 │
  PB4   ├─  LED 4  (Red) + Buzzer 🔊     │
  PB5   ├─  LED 5  (Red) + Servo Motor   │
  PB6   ├─  LED 6  (Red) + DC Motor      │
  PB7   ├─  LED 7  (Red)                 │
        │                                 │
  PD0   ├─  ▲ Button 0                   │
  PD1   ├─  ▲ Button 1                   │
  PD2   ├─  ▲ Button 2                   │
  PD3   ├─  ▲ Button 3                   │
  PE4   ├─  ▲ Button 4                   │
  PE5   ├─  ▲ Button 5                   │
  PE6   ├─  ▲ Button 6                   │
  PE7   ├─  ▲ Button 7                   │
        │                                 │
  ADC0  ├─  🎚️ Potentiometer (Temp)      │
  ADC1  ├─  🎚️ Potentiometer             │
  ADC2  ├─  💡 CDS Light Sensor          │
  ADC3  ├─  🕹️ Joystick X                │
  ADC4  ├─  🕹️ Joystick Y                │
        │                                 │
  TX1   ├─  📺 Serial Terminal TX        │
  RX1   ├─  📺 Serial Terminal RX        │
        │                                 │
  PORTC ├─  📟 LCD Display (128x64)      │
        │                                 │
        └─────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════════════

                      COMMON ACTIONS CHEAT SHEET
                      
┌────────────────────────────────────────────────────────────────────────────┐
│  ✏️  Load HEX File                                                         │
│      Right-click ATmega128 → Load Firmware → Select Main.hex              │
├────────────────────────────────────────────────────────────────────────────┤
│  💬  Open Serial Terminal                                                  │
│      Double-click "SerialTerm-1700" component (bottom of circuit)         │
├────────────────────────────────────────────────────────────────────────────┤
│  🎚️  Adjust Potentiometer                                                 │
│      Click and drag slider up/down                                         │
├────────────────────────────────────────────────────────────────────────────┤
│  🕹️  Use Joystick                                                          │
│      Click joystick and drag in any direction                              │
├────────────────────────────────────────────────────────────────────────────┤
│  🖱️  Click Button                                                          │
│      Single-click button (auto-releases)                                   │
├────────────────────────────────────────────────────────────────────────────┤
│  📊  Attach Oscilloscope                                                   │
│      Drag probe from scope to any wire/pin                                 │
├────────────────────────────────────────────────────────────────────────────┤
│  🔍  Zoom to Component                                                     │
│      Double-click component or use mouse wheel                             │
└────────────────────────────────────────────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════════════

                        TROUBLESHOOTING 101
                        
┌─────────────────────────┬──────────────────────────────────────────────────┐
│  Problem                │  Solution                                        │
├─────────────────────────┼──────────────────────────────────────────────────┤
│  LEDs not working?      │  • Click Play (▶)                                │
│                         │  • Check DDRB = 0xFF (output)                    │
│                         │  • Try PORTB = 0x00 (LEDs may be active-low)     │
├─────────────────────────┼──────────────────────────────────────────────────┤
│  Serial not showing?    │  • Double-click SerialTerm window                │
│                         │  • Check baud rate = 9600                        │
│                         │  • Verify Uart1_init() called                    │
├─────────────────────────┼──────────────────────────────────────────────────┤
│  Buttons not working?   │  • Check DDRD = 0x00 (input)                     │
│                         │  • Enable pullups: PORTD = 0xFF                  │
│                         │  • Read PIND (not PORTD)                         │
├─────────────────────────┼──────────────────────────────────────────────────┤
│  HEX file won't load?   │  • Rebuild: Ctrl+Shift+B                         │
│                         │  • Check Main.hex exists in project folder       │
│                         │  • Try manual load (right-click MCU)             │
├─────────────────────────┼──────────────────────────────────────────────────┤
│  Simulation too fast?   │  • Simulate → Simulation Speed                   │
│                         │  • Or add _delay_ms() in code                    │
└─────────────────────────┴──────────────────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════════════

                       QUICK CODE SNIPPETS
                       
╭─────────────────────────────────────────────────────────────────────────────╮
│  Test LED Blink                                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│  #include "config.h"                                                        │
│                                                                             │
│  int main(void) {                                                           │
│      DDRB = 0xFF;              // All LEDs output                           │
│                                                                             │
│      while(1) {                                                             │
│          PORTB = 0x00;         // All ON                                    │
│          _delay_ms(500);                                                    │
│          PORTB = 0xFF;         // All OFF                                   │
│          _delay_ms(500);                                                    │
│      }                                                                      │
│  }                                                                          │
╰─────────────────────────────────────────────────────────────────────────────╯

╭─────────────────────────────────────────────────────────────────────────────╮
│  Test Button Input                                                          │
├─────────────────────────────────────────────────────────────────────────────┤
│  #include "config.h"                                                        │
│                                                                             │
│  int main(void) {                                                           │
│      DDRB = 0xFF;              // LEDs output                               │
│      DDRD = 0x00;              // Buttons input                             │
│      PORTD = 0xFF;             // Enable pullups                            │
│                                                                             │
│      while(1) {                                                             │
│          PORTB = PIND;         // Copy buttons to LEDs                      │
│      }                                                                      │
│  }                                                                          │
╰─────────────────────────────────────────────────────────────────────────────╯

╭─────────────────────────────────────────────────────────────────────────────╮
│  Test Serial Output                                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│  #include "config.h"                                                        │
│                                                                             │
│  int main(void) {                                                           │
│      Uart1_init();                                                          │
│      puts_USART1("Hello from SimulIDE!\r\n");                               │
│                                                                             │
│      int count = 0;                                                         │
│      while(1) {                                                             │
│          char buffer[50];                                                   │
│          sprintf(buffer, "Count: %d\r\n", count++);                         │
│          puts_USART1(buffer);                                               │
│          _delay_ms(1000);                                                   │
│      }                                                                      │
│  }                                                                          │
╰─────────────────────────────────────────────────────────────────────────────╯


═══════════════════════════════════════════════════════════════════════════════

                        DOCUMENTATION FILES
                        
📘  SIMULIDE_QUICK_REFERENCE.md      ← You are here! (Keep at desk)
📗  SIMULIDE_GUIDE.md                ← Full manual (30 pages)
📕  LAB_EXERCISE_GUIDE.md            ← Lab instructions
📙  LAB_QUICK_START.md               ← Build system guide
📔  DOCUMENTATION_INDEX.md           ← Master navigation


═══════════════════════════════════════════════════════════════════════════════

                           PRO TIPS
                           
💡  Use serial printf for debugging instead of watching LEDs
💡  Slow down code with _delay_ms(2000) to observe behavior
💡  Attach oscilloscope to PWM pins to see waveforms
💡  Test in SimulIDE before programming physical hardware
💡  Save circuit if you make modifications (Ctrl+S)
💡  Take screenshots of working labs for documentation
💡  Read lab objectives BEFORE starting each exercise


═══════════════════════════════════════════════════════════════════════════════

                       GETTING HELP
                       
❓  Check this poster first
❓  Read SIMULIDE_GUIDE.md troubleshooting section
❓  Review LAB_QUICK_START.md for build issues
❓  Ask instructor with specific error messages
❓  Include: project name, what you tried, error screenshot


═══════════════════════════════════════════════════════════════════════════════

          🎉 YOU'RE READY TO SIMULATE! 🎉
          
  1. Open project in VS Code
  2. Press Ctrl+Shift+B
  3. Select "Build and Simulate"
  4. Click Play (▶)
  5. Start learning! 🚀


═══════════════════════════════════════════════════════════════════════════════

          SimulIDE 1.1.0-SR1 | ATmega128 Framework | October 2025
          
          💻 No Hardware? No Problem! Learn Embedded Systems Anywhere.

═══════════════════════════════════════════════════════════════════════════════
```

---

## 📝 Printing Instructions

### For Best Results

1. **Print on A3 paper** (larger format)
2. **Use color printer** (components color-coded)
3. **Laminate** (keep at desk, wipes clean)
4. **Or:** Display on second monitor

### Alternative Formats

- **PDF**: Print this markdown as PDF in VS Code
- **Poster**: Use poster printing service
- **Handout**: Print 2-per-page on A4

---

**Keep this poster visible while working with SimulIDE!** 🎓
