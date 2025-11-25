# Interrupt Module Documentation Summary

**COURSE:** SOC 3050 - Embedded Systems and Applications  
**MODULE:** Polling vs Interrupt Programming  
**TARGET:** Instructors and Teaching Assistants

---

## OVERVIEW

This module teaches the fundamental difference between polling and interrupt-driven programming using ATmega128 external interrupts. Students learn when to use each approach and how to implement interrupt service routines (ISRs) properly.

**Prerequisites:**
- Port_Basic (GPIO fundamentals)
- Binary number systems
- Basic C programming (functions, loops, variables)

**Hardware Required:**
- ATmega128 development board
- Buttons on PD0, PD1 (active-low with pull-ups)
- LEDs on PORTB (active-low)
- GLCD for status display

**Time Required:** 3 weeks (2 hours/week lab + homework)

---

## LEARNING OBJECTIVES

By completing this module, students will:

1. **Understand Polling:**
   - Continuous checking in while(1) loops
   - Relationship between delay and response time
   - Limitations: missed events, wasted CPU cycles

2. **Understand Interrupts:**
   - Hardware-driven event response
   - ISR (Interrupt Service Routine) execution
   - Vector table and priorities
   - Edge detection modes

3. **ATmega128 Interrupt System:**
   - Configure EICRA/EICRB registers
   - Enable interrupts in EIMSK
   - Use sei()/cli() for global control
   - Read/clear interrupt flags

4. **ISR Programming:**
   - Keep ISRs short and fast
   - Use volatile for shared variables
   - Flag-based ISR-to-main communication
   - Avoid blocking operations in ISRs

5. **Practical Applications:**
   - When to use polling vs interrupts
   - Implement button press detection
   - Count events accurately
   - Handle multiple interrupt sources

---

## THREE-WEEK TEACHING PLAN

### Week 1: Polling Concepts

**Day 1: Polling Basics (Demo 1)**
- Lecture (30 min):
  * What is polling?
  * while(1) loop structure
  * CPU continuously checks input
  * Response time = loop execution time
  
- Lab Activity (60 min):
  * Run demo_01_polling_basics()
  * Observe LED3 following button PD1
  * Watch PORTB/PIND values on GLCD
  * Experiment with different _delay_ms() values
  
- Assignment:
  * Modify delay to 10ms, 100ms, 500ms
  * Document response time for each
  * Calculate theoretical vs observed response

**Day 2: Polling Limitations (Demo 2)**
- Lecture (30 min):
  * Why polling has limits
  * Missed events with slow polling
  * Trade-off: responsiveness vs CPU usage
  * Motivate need for interrupts
  
- Lab Activity (60 min):
  * Run demo_02_polling_limitations()
  * Try pressing button during "slow" phase
  * Count how many presses are missed
  * Compare fast vs slow phases
  
- Assignment:
  * Document which button presses are detected
  * Calculate: With 300ms delay, what minimum pulse width?
  * Essay: When is polling good enough?

**Assessment:**
- Quiz: Polling concepts (5 questions, 10 points)
- Lab submission: Delay experiment results

---

### Week 2: Interrupt Introduction

**Day 1: Interrupt Basics (Demo 3)**
- Lecture (30 min):
  * What are interrupts?
  * Hardware detects events automatically
  * ISR (Interrupt Service Routine) concept
  * ATmega128 external interrupts (INT0-INT7)
  * Vector table and priorities
  
- Lab Activity (60 min):
  * Run demo_03_interrupt_basics()
  * Compare response with polling demo
  * Press button rapidly - all detected!
  * Examine INT0 configuration code
  
- Assignment:
  * Configure INT1 for falling edge
  * Create ISR that toggles LED1
  * Document configuration steps

**Day 2: ISR Communication (Demo 4)**
- Lecture (30 min):
  * ISR programming rules
  * volatile keyword importance
  * Flag-based communication pattern
  * Keeping ISRs short and fast
  
- Lab Activity (60 min):
  * Run demo_04_isr_communication()
  * Watch event counter increment
  * Examine flag checking in main loop
  * Experiment: Remove volatile, observe problems
  
- Assignment:
  * Implement counter that tracks INT0 events
  * Display count on GLCD
  * Add "reset counter" button on INT1

**Assessment:**
- Lab practical: Configure INT2, implement ISR (15 min, 20 points)
- Code review: Check for volatile usage

---

### Week 3: Advanced Techniques

**Day 1-2: Edge Detection Modes (Demo 5)**
- Lecture (30 min):
  * ISC bits control edge/level detection
  * Falling edge: button press (HIGH→LOW)
  * Rising edge: button release (LOW→HIGH)
  * Any change: both edges
  * Low level: continuous (rarely used)
  
- Lab Activity (90 min across 2 days):
  * Run demo_05 with mode 0 (falling)
  * Run demo_05 with mode 1 (rising)
  * Run demo_05 with mode 2 (any change)
  * Run demo_05 with mode 3 (low level) - observe flood!
  * Compare event counts for each mode
  
- Assignment:
  * Create table showing when ISR triggers for each mode
  * Implement "rotary encoder" using any-change mode
  * Document EICRA register values for each configuration

**Assessment:**
- Final project options (choose 1, 30 points total)

---

## FINAL PROJECT OPTIONS

### Option A: Reaction Time Game (25 points + 5 bonus)
- Random LED lights up after delay
- User presses button
- Display reaction time in milliseconds
- Bonus: Track best/worst times

**Grading:**
- Functionality (10pt): Random timing, accurate measurement
- Code quality (8pt): ISR design, volatile usage
- Display (5pt): Clear feedback to user
- Documentation (2pt): Comments, README

### Option B: Multi-Button Menu System (25 points + 5 bonus)
- 3 buttons: UP, DOWN, SELECT (INT0, INT1, INT2)
- Navigate through 5 menu options
- Display current selection on GLCD
- Bonus: Implement sub-menus

**Grading:**
- Navigation (10pt): All buttons work correctly
- Code (8pt): Multiple ISRs, proper priorities
- Display (5pt): Clear menu interface
- Documentation (2pt): User manual

### Option C: Event Logger (25 points + 5 bonus)
- Log timestamps of button presses
- Display last 8 events with time
- Calculate time between events
- Bonus: Export log via UART

**Grading:**
- Logging (10pt): Accurate timestamps
- Code (8pt): ISR + timer integration
- Display (5pt): Formatted log output
- Documentation (2pt): How to read log

### Option D: Custom Proposal (25 points + 5 bonus)
- Student proposes interrupt-based project
- Must demonstrate: ISR, flags, edge detection
- Requires instructor approval

**Grading:**
- Creativity (10pt): Original idea
- Technical (10pt): Correct interrupt usage
- Presentation (3pt): Demo to class
- Documentation (2pt): Design document

---

## GRADING RUBRIC (Total: 100 points)

| Component                  | Points | Criteria                                    |
|----------------------------|--------|---------------------------------------------|
| Week 1 Quiz                | 10     | Polling concepts understanding              |
| Week 1 Lab                 | 10     | Delay experiments completed                 |
| Week 2 Practical           | 20     | Configure interrupt, implement ISR          |
| Week 2 Lab                 | 10     | Counter with reset button                   |
| Week 3 Lab                 | 10     | Edge mode experiments table                 |
| Final Project              | 30     | Functionality, code, display, docs          |
| Participation              | 5      | Lab attendance, questions                   |
| Lab Notebook               | 5      | Weekly notes, diagrams                      |

**Grade Scale:** A (90-100), B (80-89), C (70-79), D (60-69), F (<60)

---

## COMMON STUDENT MISTAKES

### 1. Missing volatile Keyword
**Symptom:** Flag never detected in main loop  
**Cause:** Compiler optimizes away flag check  
**Fix:** Use `volatile uint8_t flag;`  
**Prevention:** Emphasize in lecture, check in code review

### 2. Delays in ISR
**Symptom:** System becomes unresponsive  
**Cause:** `_delay_ms()` called in ISR blocks other interrupts  
**Fix:** Move delays to main loop  
**Prevention:** Show bad example, explain why it's wrong

### 3. Wrong Edge Configuration
**Symptom:** ISR triggers on wrong event  
**Cause:** ISC bits set incorrectly  
**Fix:** Review truth table: ISC01=1, ISC00=0 for falling  
**Prevention:** Provide configuration template

### 4. Forgot sei()
**Symptom:** ISR never runs  
**Cause:** Global interrupts not enabled  
**Fix:** Add `sei();` after interrupt configuration  
**Prevention:** Include in all example code

### 5. Multi-byte Variable Race Condition
**Symptom:** Counter shows garbage values occasionally  
**Cause:** Main reads 16-bit counter while ISR writes it  
**Fix:** Use cli()/sei() around 16-bit reads in main  
**Prevention:** Teach atomic operations in lecture

### 6. Interrupt Flood with Low Level Mode
**Symptom:** System freezes, counter increments rapidly  
**Cause:** Low level mode triggers continuously while button held  
**Fix:** Use falling edge mode instead  
**Prevention:** Demo 5 shows this problem deliberately

---

## ASSESSMENT IDEAS

### Quick Quizzes (5-10 minutes each)

**Week 1 Quiz:**
1. What is the main disadvantage of polling? (Wastes CPU, may miss events)
2. Calculate: With 100ms delay, what's minimum detectable pulse? (100ms)
3. True/False: Polling is always inferior to interrupts. (False - depends on application)

**Week 2 Quiz:**
1. What does ISR stand for? (Interrupt Service Routine)
2. Which register enables INT0? (EIMSK)
3. What must shared variables be declared as? (volatile)
4. What is INT0's vector address? (0x0004)

**Week 3 Quiz:**
1. ISC01=1, ISC00=0 configures which mode? (Falling edge)
2. Which edge detects button press? (Falling, if active-low)
3. Why avoid low level mode? (Causes interrupt flood)

### Practical Exams

**Midterm Practical (15 minutes):**
- Task: Configure INT2 for rising edge, implement ISR that increments counter
- Display counter on GLCD
- Grading: Configuration (10pt), ISR (5pt), Display (5pt)

**Final Practical (30 minutes):**
- Task: Two buttons (INT0, INT1), each toggles different LED
- Display which button was pressed last
- Count total presses
- Grading: Multiple ISRs (10pt), Display (8pt), Counting (7pt)

---

## TROUBLESHOOTING GUIDE (For TAs)

### "My ISR never runs"
1. Check: `sei();` called?
2. Check: `EIMSK |= (1 << INT0);` present?
3. Check: Pin configured as input? `DDRD &= ~(1 << PD0);`
4. Check: Pull-up enabled? `PORTD |= (1 << PD0);`
5. Check: ISR function defined? `ISR(INT0_vect) { ... }`
6. Test: Write 1 to EIFR manually - does ISR run?

### "ISR runs but flag not detected"
1. Check: `volatile` keyword used?
2. Check: Compiler optimization level (-Os or -O2 may optimize away)
3. Test: Add LED toggle in ISR - does it work?
4. Check: Flag cleared in main after checking?

### "Counter shows wrong values"
1. Check: 16-bit counter needs cli()/sei() protection when reading
2. Check: Overflow? Use uint32_t for long-running counters
3. Test: Print counter in ISR and main separately

### "System freezes"
1. Check: Delay in ISR? Remove it!
2. Check: Low level mode? Change to edge mode
3. Check: Infinite loop in ISR? Add timeout
4. Test: Disable interrupt, see if main runs

---

## COURSE INTEGRATION

**Before this module:**
- Port_Basic (GPIO, LED control, button reading)
- Serial_Communications (optional, for enhanced projects)

**After this module:**
- Timer/Counter interrupts (internal events)
- ADC with interrupts (sensor reading)
- UART with interrupts (communication)
- Final project combining all modules

**Suggested Overall Schedule:**
- Weeks 1-2: Port_Basic
- Weeks 3-5: Serial_Communications
- **Weeks 6-8: Interrupt (this module)**
- Weeks 9-11: Timers and ADC
- Weeks 12-15: Final project

---

## RESOURCES FOR INSTRUCTORS

### Documentation Files
- **INTERRUPT_QUICK_REFERENCE.md:** Student printable reference
- **INTERRUPT_FLOW_DIAGRAMS.md:** Visual aids for lectures (10 diagrams)
- **Main.c:** 5 working demos with extensive comments

### External Resources
- ATmega128 Datasheet: Chapter on External Interrupts
- AVR Libc Manual: <avr/interrupt.h> documentation
- Interrupt latency calculator: [online tool]

### Recommended Reading
- "Embedded C Programming" - Mark Siegesmund
- "Making Embedded Systems" - Elecia White (Chapter on interrupts)

---

## INSTRUCTOR SETUP CHECKLIST

### Before Semester
- [ ] Test all 5 demos on hardware/simulator
- [ ] Print INTERRUPT_QUICK_REFERENCE.md (class copies)
- [ ] Prepare lecture slides with flow diagrams
- [ ] Set up grading rubrics in LMS
- [ ] Create quiz question bank

### Week 1 Preparation
- [ ] Load Demo 1 on instructor station
- [ ] Prepare polling delay experiment handout
- [ ] Print Diagram 1 (Polling Flow) as poster

### Week 2 Preparation
- [ ] Set up practical exam station
- [ ] Prepare ISR configuration template
- [ ] Print Diagram 3 (ISR Execution Timeline)

### Week 3 Preparation
- [ ] Finalize project options
- [ ] Create edge mode comparison table handout
- [ ] Schedule project help desk hours

---

## CHANGE LOG

**Version 1.0 (2025):**
- Initial documentation for Interrupt module
- 5 progressive demos with PROGMEM optimization
- 3-week teaching plan with assessments
- Comprehensive troubleshooting guide

**Future Improvements:**
- Add oscilloscope lab (measure ISR timing)
- Video tutorials for edge detection concepts
- Auto-grader for practical exams
- Advanced: Interrupt nesting demonstration

---

## CONTACT

**Course Coordinator:** [Instructor Name]  
**Teaching Assistants:** [TA Names]  
**Office Hours:** [Schedule]  
**Email:** [Contact Email]

**For technical issues:**
- Check INTERRUPT_QUICK_REFERENCE.md first
- Review INTERRUPT_FLOW_DIAGRAMS.md
- Post on course forum
- Attend TA office hours

---

**Last Updated:** 2025  
**Course:** SOC 3050 - Embedded Systems and Applications  
**Module:** Polling vs Interrupt Programming
