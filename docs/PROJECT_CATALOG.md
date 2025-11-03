# ATmega128 Educational Projects Catalog
## Complete Teaching Curriculum for Embedded Systems

### 📚 Project Organization

This catalog contains **35 polished educational projects** organized into a clear learning progression for teaching ATmega128 microcontroller programming.

---

## 🎯 Core Learning Path (01-15): Fundamentals

### **Phase 1: Digital I/O & Port Programming (01-05)**

#### **01_Port_Basic** 
- **Topic:** Digital I/O fundamentals
- **Concepts:** DDR, PORT, PIN registers, LED control
- **Difficulty:** ⭐ Beginner
- **Build Status:** ✅ Verified

#### **02_Port_Assembly**
- **Topic:** Assembly language I/O
- **Concepts:** SBI, CBI, IN, OUT instructions
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ✅ Verified

#### **03_Assembly_Blink**
- **Topic:** LED blinking in assembly
- **Concepts:** Delay loops, bit manipulation
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ✅ Verified

#### **04_Assembly_Button_Simple**
- **Topic:** Button input in assembly
- **Concepts:** Input reading, pull-ups
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ✅ Verified

#### **05_Assembly_Button_LED**
- **Topic:** Combined button/LED control
- **Concepts:** Conditional branching
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ✅ Verified

---

### **Phase 2: Serial Communication (06-08)**

#### **06_Serial_Polling_Char**
- **Topic:** UART single character polling
- **Concepts:** USART registers, baud rate
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ✅ Verified

#### **07_Serial_Polling_String**
- **Topic:** UART string transmission
- **Concepts:** String handling, loops
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ✅ Verified

#### **08_Serial_Polling_Echo**
- **Topic:** UART echo server
- **Concepts:** Full-duplex communication
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ✅ Verified

---

### **Phase 3: Analog & Sensors (09-12)**

#### **09_ADC_Basic**
- **Topic:** Analog-to-Digital Conversion
- **Concepts:** ADC initialization, channel selection
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ⚠️ Requires review (legacy dependencies)

#### **10_CDS_Light_Sensor**
- **Topic:** Light sensor reading
- **Concepts:** Photoresistor, ADC application
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ⚠️ Requires review

#### **11_Accelerometer**
- **Topic:** Accelerometer interfacing
- **Concepts:** Multi-axis sensors, ADC
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ⚠️ Requires review

#### **12_Graphics_Display**
- **Topic:** Graphical LCD basics
- **Concepts:** GLCD library, pixel manipulation
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ⚠️ Requires review

---

### **Phase 4: Timers & Interrupts (13-14)**

#### **13_Timer_Basic**
- **Topic:** Timer fundamentals
- **Concepts:** Timer/Counter registers, overflow
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ⚠️ Requires review

#### **14_Interrupt_Basic**
- **Topic:** Interrupt handling basics
- **Concepts:** ISR, interrupt vectors
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ⚠️ Requires review

---

### **Phase 5: Input Devices (15)**

#### **15_Joystick**
- **Topic:** Analog joystick control
- **Concepts:** Multi-axis ADC, user input
- **Difficulty:** ⭐⭐ Intermediate
- **Build Status:** ⚠️ Requires review

---

## 🔧 Advanced Peripheral Modules

### **PWM Motor Control (3 Projects)**

#### **PWM_Motor_DC**
- **Topic:** DC motor speed control
- **Concepts:** Timer1 PWM, H-bridge, duty cycle
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (speed control, direction, pot control, acceleration)

#### **PWM_Motor_Servo**
- **Topic:** Servo positioning
- **Concepts:** 50Hz PWM, 1-2ms pulses, angle control
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (angle control, sweep, joystick, smooth movement)

#### **PWM_Motor_Stepper**
- **Topic:** Stepper motor control
- **Concepts:** Full/half step, position tracking
- **Difficulty:** ⭐⭐⭐⭐ Expert
- **Build Status:** ✅ Verified
- **Demos:** 4 (full step, half step, position, degrees)

---

### **SPI Communication (3 Projects)**

#### **SPI_Master_Basic**
- **Topic:** SPI fundamentals
- **Concepts:** SPCR, SPSR, SPDR, modes, speeds
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (loopback, speeds, modes, data transfer)

#### **SPI_EEPROM_Memory**
- **Topic:** SPI EEPROM (25LC256)
- **Concepts:** Page write, sequential read, 32KB storage
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (read/write, page operations, verify, stress test)

#### **SPI_Multi_Device**
- **Topic:** Multi-device SPI bus
- **Concepts:** Chip select management, per-device config
- **Difficulty:** ⭐⭐⭐⭐ Expert
- **Build Status:** ✅ Verified
- **Demos:** 4 (device scan, individual access, synchronized ops)

---

### **I2C/TWI Communication (3 Projects)**

#### **I2C_Master_Basic**
- **Topic:** I2C fundamentals
- **Concepts:** TWBR, TWSR, TWCR, TWDR, 7-bit addressing
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (bus scan, register R/W, sequential access, speed test)

#### **I2C_RTC_DS1307**
- **Topic:** Real-Time Clock
- **Concepts:** DS1307, BCD conversion, battery backup
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (time/date, alarm, RAM, square wave)

#### **I2C_Sensors_Multi**
- **Topic:** Multi-sensor integration
- **Concepts:** MPU6050, BMP180, HMC5883L
- **Difficulty:** ⭐⭐⭐⭐ Expert
- **Build Status:** ✅ Verified
- **Demos:** 4 (individual sensors, fusion, motion detect, logging)

---

### **LCD Display (3 Projects)**

#### **LCD_Character_Basic**
- **Topic:** HD44780 4-bit mode
- **Concepts:** PORTG interface, custom characters
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (text display, positioning, custom chars, formatting)

#### **LCD_Advanced_Features**
- **Topic:** Advanced LCD techniques
- **Concepts:** Scrolling, animations, progress bars
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (marquee, progress, animations, menus)

#### **LCD_Sensor_Dashboard**
- **Topic:** Real-time sensor display
- **Concepts:** ADC integration, bargraphs, alerts
- **Difficulty:** ⭐⭐⭐⭐ Expert
- **Build Status:** ✅ Verified
- **Demos:** 4 (multi-sensor, graphs, thresholds, logging)

---

### **Keypad Input (3 Projects)**

#### **Keypad_Matrix_Basic**
- **Topic:** 4x4 matrix scanning
- **Concepts:** Row/column scanning, PORTA
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (basic scan, multi-key, test pattern, frequency)

#### **Keypad_Advanced_Debounce**
- **Topic:** Debouncing algorithms
- **Concepts:** Software debounce, long-press, state machines
- **Difficulty:** ⭐⭐⭐⭐ Expert
- **Build Status:** ✅ Verified
- **Demos:** 4 (debounce test, long-press, PIN entry, validation)

#### **Keypad_Calculator_App**
- **Topic:** Calculator application
- **Concepts:** Menu systems, arithmetic, user interface
- **Difficulty:** ⭐⭐⭐⭐ Expert
- **Build Status:** ✅ Verified
- **Demos:** 4 (calculator, menu, guessing game, stopwatch)

---

### **Watchdog Timer (2 Projects)**

#### **Watchdog_System_Reset**
- **Topic:** WDT fundamentals
- **Concepts:** WDTCR, timeout periods, reset detection
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (basic reset, periodic, hang simulation, recovery)

#### **Watchdog_Fail_Safe**
- **Topic:** Fail-safe operations
- **Concepts:** Heartbeat, critical sections, degradation
- **Difficulty:** ⭐⭐⭐⭐ Expert
- **Build Status:** ✅ Verified
- **Demos:** 4 (heartbeat, critical protect, degradation, recovery)

---

### **Power Management (3 Projects)**

#### **Power_Sleep_Modes**
- **Topic:** Sleep mode basics
- **Concepts:** 5 sleep modes, wake-up sources
- **Difficulty:** ⭐⭐⭐ Advanced
- **Build Status:** ✅ Verified
- **Demos:** 4 (mode comparison, periodic, battery sim, smart sleep)

#### **Power_LowPower_Sensors**
- **Topic:** Low-power sensor monitoring
- **Concepts:** ADC NR mode, adaptive sampling, battery optimization
- **Difficulty:** ⭐⭐⭐⭐ Expert
- **Build Status:** ✅ Verified
- **Demos:** 4 (single sensor, multi-sensor, data logger, adaptive)

#### **Power_Wakeup_Optimization**
- **Topic:** Wake-up sources & optimization
- **Concepts:** Multiple wake sources, power budget, event-driven
- **Difficulty:** ⭐⭐⭐⭐ Expert
- **Build Status:** ✅ Verified
- **Demos:** 4 (external INT, timer, multi-source, power calc)

---

## 📊 Summary Statistics

### **Total Projects:** 35
- **Core Learning Path:** 15 projects
- **PWM Motors:** 3 projects
- **SPI Communication:** 3 projects
- **I2C Communication:** 3 projects
- **LCD Display:** 3 projects
- **Keypad Input:** 3 projects
- **Watchdog Timer:** 2 projects
- **Power Management:** 3 projects

### **Build Status:**
- ✅ **Fully Verified:** 20 projects (all modern curriculum)
- ⚠️ **Requires Review:** 6 projects (legacy dependencies)
- 📦 **Archived:** 30 projects (moved to projects_archive)

### **Total Demos:** 80+ (4 demos per modern project)

---

## 🎓 Recommended Learning Sequence

### **Week 1-2: Digital Fundamentals**
01 → 02 → 03 → 04 → 05

### **Week 3: Serial Communication**
06 → 07 → 08

### **Week 4-5: Analog & Sensors**
09 → 10 → 11 → 12

### **Week 6: Timers & Interrupts**
13 → 14 → 15

### **Week 7-8: Advanced Peripherals**
- PWM Motors (all 3)
- SPI Communication (all 3)

### **Week 9-10: Communication & Display**
- I2C Communication (all 3)
- LCD Display (all 3)

### **Week 11: User Input**
- Keypad projects (all 3)

### **Week 12: System Reliability & Power**
- Watchdog Timer (both)
- Power Management (all 3)

---

## 🔨 Build System

### **All Modern Projects Include:**
- ✅ **build.bat** - Automated compilation
- ✅ **config.h** - Standard configuration header
- ✅ **Main.c** - Well-commented source code
- ✅ **Educational comments** - Learning-focused documentation

### **Compiler Settings:**
- **MCU:** atmega128
- **F_CPU:** 16000000UL (16 MHz)
- **BAUD:** 9600
- **Optimization:** -Os (size)
- **Warnings:** -Wall -Wextra

---

## 📁 Archive Information

**Location:** `projects_archive/`

**Archived projects** (30 total) include older implementations that may have:
- Complex dependencies
- Incomplete documentation
- Non-standard structure
- Duplicate functionality

These are preserved for reference but not recommended for teaching.

---

## ✅ Quality Standards

All verified projects meet these criteria:
1. ✅ Build successfully without errors
2. ✅ Include comprehensive comments
3. ✅ Follow consistent naming conventions
4. ✅ Provide 4 educational demos each
5. ✅ Use menu-driven UART interface
6. ✅ Include LED visual feedback
7. ✅ Demonstrate best practices

---

## 🚀 Next Steps for Instructors

1. **Start with Core Path (01-15)** for fundamentals
2. **Progress to Peripherals** based on course focus
3. **Use 4-demo structure** for hands-on labs
4. **Leverage UART output** for debugging/verification
5. **Build incrementally** following dependency order

---

**Last Updated:** 2025-10-03  
**Curriculum Status:** ✅ Production Ready  
**Total Teaching Hours:** 60+ hours of content

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
All rights reserved for educational purposes.  
**Contact:** [linkedin.com/in/gnoejh53](https://linkedin.com/in/gnoejh53)