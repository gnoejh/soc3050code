# Graphics_Display Documentation Summary

**COURSE:** SOC 3050 - Embedded Systems and Applications  
**MODULE:** Graphic LCD Programming (KS0108)  
**TARGET:** Instructors and Teaching Assistants

---

## OVERVIEW

This module teaches graphic LCD interfacing using the KS0108 controller (128×64 monochrome display). Students learn coordinate systems, dual-controller architecture, page-based memory, drawing algorithms, and text rendering.

**Prerequisites:**
- Port I/O (Port_Basic module)
- Basic C programming
- Binary number representation
- Understanding of RAM vs Flash memory (PROGMEM)

**Hardware Required:**
- ATmega128 development board
- KS0108 GLCD (128×64)
- SimulIDE 0.4.15 (for simulation)
- OR actual hardware with connections

**Time Required:** 4 weeks (2 hours/week lab + homework)

---

## LEARNING OBJECTIVES

By completing this module, students will:

1. **Hardware Understanding:**
   - Explain dual-controller architecture (CS1/CS2)
   - Understand page-based memory organization
   - Configure parallel 8-bit interface

2. **Programming Skills:**
   - Use GLCD text and graphics functions
   - Implement Bresenham's line algorithm concepts
   - Handle coordinate transformations
   - Apply PROGMEM for string storage

3. **Debugging Skills:**
   - Diagnose controller split issues
   - Fix coordinate system errors
   - Identify page addressing problems
   - Use oscilloscope/logic analyzer (hardware)

4. **Design Skills:**
   - Create sensor dashboards
   - Design menu systems
   - Implement progress indicators
   - Develop simple animations

---

## FOUR-WEEK TEACHING PLAN

### Week 1: Text and Memory Basics (Demos 1-3)

**Lecture Topics (30 min):**
- KS0108 architecture overview
- Coordinate system (X=rows, Y=columns)
- Page-based memory layout
- Dual-controller split at y=64

**Lab Activities (90 min):**
1. **Demo 1: Text Header**
   - Run pre-written code
   - Modify header text
   - Experiment with text positions
   - **Assignment:** Display student name and ID

2. **Demo 2: Pixel Drawing**
   - Understand GLCD_Dot(x, y)
   - Observe checker pattern rendering
   - Time the 8192-pixel loop
   - **Assignment:** Create custom pixel pattern (initials)

3. **Demo 3: Page Addressing**
   - Visualize page boundaries
   - Observe CS1/CS2 split
   - Manually calculate page numbers
   - **Assignment:** Draw pattern spanning page boundary

**Common Student Issues:**
- **X/Y confusion:** Students use math convention (x=horizontal)
  - *Solution:* Always say "row" and "column" in class
- **Controller split:** Drawing at y=64 shows on left controller
  - *Solution:* Show GLCD_FLOW_DIAGRAMS.md Diagram 6

**Assessment:**
- Quiz: Coordinate system (5 questions)
- Lab submission: Custom pixel pattern code
- Points: 10% of module grade

---

### Week 2: Drawing Primitives (Demos 4-6)

**Lecture Topics (30 min):**
- Bresenham's line algorithm (intuition, not proof)
- Rectangle drawing (outline vs filled)
- Midpoint circle algorithm
- Integer-only arithmetic advantages

**Lab Activities (90 min):**
1. **Demo 4: Lines**
   - Run line examples
   - Draw lines at different angles
   - Observe diagonal line smoothness
   - **Assignment:** Draw house outline (5 lines)

2. **Demo 5: Rectangles**
   - Nested rectangle patterns
   - Understand inset calculations
   - Experiment with sizes
   - **Assignment:** Draw concentric rectangles (5 layers)

3. **Demo 6: Circles**
   - Concentric circles demonstration
   - Circle radius limitations (screen size)
   - Combined shapes (circles + rectangles)
   - **Assignment:** Draw bullseye target (3 circles)

**Common Student Issues:**
- **Lines off-screen:** Coordinates exceed 63 or 127
  - *Solution:* Teach boundary checking before drawing
- **Circle clipping:** Radius + center goes off-screen
  - *Solution:* Formula: cx ± radius must stay in bounds

**Assessment:**
- Practical exam: Draw specified shape in 15 minutes
- Lab submission: House and target programs
- Points: 20% of module grade

---

### Week 3: Text Layout and Multi-Controller (Demo 7)

**Lecture Topics (30 min):**
- 5×7 font rendering details
- Character spacing (6 pixels)
- 20-column × 8-row text grid
- Left vs right controller text handling
- lcd_string_P() for PROGMEM strings

**Lab Activities (90 min):**
1. **Demo 7: Text Pages**
   - Display text on both controllers
   - Observe automatic column wrapping
   - Calculate maximum text length
   - **Assignment:** Create sensor label layout

2. **Text + Graphics Combination:**
   - Add borders around text
   - Create titled sections
   - Draw progress bars with labels
   - **Assignment:** Design temperature display (thermometer graphic + text)

**Common Student Issues:**
- **Text overflow:** String exceeds 20 characters
  - *Solution:* Use lcd_xy() to start new row
- **PROGMEM errors:** Forgot _P suffix or strncpy_P()
  - *Solution:* Review PROGMEM pattern in Main.c

**Assessment:**
- Lab submission: Sensor dashboard design
- Peer review: Evaluate 2 classmates' layouts
- Points: 15% of module grade

---

### Week 4: Advanced Patterns and Final Project (Demos 8-10)

**Lecture Topics (30 min):**
- Loop-based pattern generation
- Mathematical functions in graphics (sin, cos for future)
- Animation concepts (clear + redraw)
- Real-world applications (sensor dashboards, menu systems)

**Lab Activities (60 min):**
1. **Demos 8-10: Patterns**
   - Radiating lines
   - Nested shapes
   - Grid alignment
   - **Assignment:** Create original pattern

2. **Final Project Worktime (30 min):**
   - Students implement their chosen project
   - Instructor circulates for help

**Final Project Options (Choose 1):**

**Option A: Sensor Dashboard**
- Display 3 sensor readings (ADC values)
- Include labels, units, and borders
- Update readings in loop (simulated)
- **Rubric:** Layout (10pt), Code quality (10pt), Documentation (5pt)

**Option B: Menu System**
- 3-level menu (e.g., Settings → Display → Brightness)
- Navigate with simulated buttons (comments)
- Highlight selected item
- **Rubric:** Navigation logic (10pt), Display clarity (10pt), Code (5pt)

**Option C: Simple Game**
- Paddle + ball (Pong-like)
- Score display
- Boundary collision detection
- **Rubric:** Game mechanics (10pt), Graphics (10pt), Playability (5pt)

**Option D: Custom Design**
- Student proposes idea (requires approval)
- Must demonstrate all learned concepts
- **Rubric:** Creativity (10pt), Technical skill (10pt), Presentation (5pt)

**Assessment:**
- Final project: 40% of module grade
- Code documentation: 10% of module grade
- Lab notebook: 5% of module grade

---

## GRADING RUBRIC (Total: 100 points)

| Component                     | Points | Criteria                                      |
|-------------------------------|--------|-----------------------------------------------|
| Week 1 Quiz                   | 10     | Coordinate system understanding               |
| Week 1 Lab                    | 5      | Custom pixel pattern creativity and correctness|
| Week 2 Practical Exam         | 20     | Draw shape accurately in time limit           |
| Week 2 Lab                    | 10     | House and target submissions                  |
| Week 3 Dashboard Lab          | 15     | Sensor layout design and peer review          |
| Final Project                 | 25     | Functionality, code quality, design           |
| Code Documentation            | 10     | Comments, PROGMEM usage, clarity              |
| Lab Notebook                  | 5      | Weekly notes, diagrams, reflections           |

**Grade Scale:**
- A: 90-100
- B: 80-89
- C: 70-79
- D: 60-69
- F: <60

---

## COMMON STUDENT MISTAKES

### 1. Coordinate System Confusion
**Symptom:** Shapes appear rotated or in wrong locations  
**Cause:** Using x=horizontal, y=vertical (math convention)  
**Fix:** Drill "x=ROWS (vertical), y=COLUMNS (horizontal)"  
**Prevention:** Use row/column terminology exclusively in Week 1

### 2. Controller Split Issues
**Symptom:** Text cut off at y=64, shapes disappear  
**Cause:** Not understanding CS1 (0-63) vs CS2 (64-127) split  
**Fix:** Show Diagram 1 and 6 in GLCD_FLOW_DIAGRAMS.md  
**Prevention:** Assign exercise drawing across y=64 boundary

### 3. Page Boundary Errors
**Symptom:** Horizontal gaps in vertical lines  
**Cause:** Not accounting for page-aligned memory  
**Fix:** Explain page calculation: page = row / 8  
**Prevention:** Demo 3 visualizes page boundaries

### 4. PROGMEM String Errors
**Symptom:** Compilation errors or garbage text  
**Cause:** Using lcd_string() instead of lcd_string_P()  
**Fix:** Show PROGMEM pattern in Main.c lines 85-110  
**Prevention:** Provide template code with PROGMEM strings

### 5. Off-Screen Drawing
**Symptom:** Nothing appears or simulator crashes  
**Cause:** Coordinates outside 0-63 (x) or 0-127 (y)  
**Fix:** Add boundary checks before GLCD_Dot() calls  
**Prevention:** Teach defensive programming in Week 2

### 6. Slow Refresh Issues
**Symptom:** Flickering or slow animation  
**Cause:** Calling lcd_clear() inside tight loops  
**Fix:** Clear once, then redraw only changed regions  
**Prevention:** Discuss performance in Week 4 (Diagram 10)

---

## ASSESSMENT IDEAS

### Quizzes (5-10 minutes each)

**Week 1 Quiz:**
1. What are the valid x-coordinate ranges? (0-63)
2. Which controller handles y=100? (CS2)
3. How many pages are in the display? (8)
4. What is the page number for row 30? (3, since 30/8=3)
5. Why use PROGMEM for strings? (Save SRAM)

**Week 2 Quiz:**
1. What algorithm does GLCD_Line() use? (Bresenham's)
2. Why is integer arithmetic important? (Speed, no floating-point)
3. Draw a circle: center (32,64), radius 20. What controllers are used? (Both, spans y=44..84)

**Week 3 Quiz:**
1. How many characters fit on one row? (20)
2. What is the function for PROGMEM strings? (lcd_string_P)
3. Calculate pixel position for text row 4, column 10. (x=4, y=60)

### Practical Exams (15-30 minutes)

**Midterm Practical:**
- Given: Empty Main.c template
- Task: Draw a house (rectangle + triangle roof + door + 2 windows)
- Time: 15 minutes
- Grading: 5pt per element (total 25pt, scaled to 20pt)

**Final Practical:**
- Given: Sensor reading simulation (random ADC values)
- Task: Display 3 labeled sensors with bar graphs
- Time: 30 minutes
- Grading: Labels (5pt), Bars (10pt), Updates (5pt), Code quality (5pt)

### Final Project Rubric (25 points)

| Criteria                | Excellent (5pt) | Good (4pt) | Satisfactory (3pt) | Poor (<3pt) |
|-------------------------|-----------------|------------|-------------------|-------------|
| **Functionality**       | All features work perfectly | Minor bugs | Missing 1 feature | Doesn't run |
| **Code Quality**        | Well-organized, PROGMEM, comments | Some issues | Hard to read | Messy |
| **Design/Creativity**   | Original, polished | Standard implementation | Basic | Minimal effort |
| **Documentation**       | README + inline comments | Comments only | Minimal docs | None |
| **Presentation**        | Clear demo, answers questions | Adequate demo | Struggles to explain | No demo |

**Bonus Points (up to 5):**
- Animation (2pt)
- User input simulation (2pt)
- Advanced algorithm (3pt)
- Exceeds requirements (5pt)

---

## TROUBLESHOOTING GUIDE (For TAs)

### "Nothing appears on screen"
1. Check power connections (hardware)
2. Verify init_devices() called before lcd_init()
3. Confirm lcd_clear() called after initialization
4. Check GPIO configuration in _glcd.c
5. Test with Demo 1 (known working code)

### "Text is garbled"
1. Verify font array not corrupted (_glcd.c)
2. Check lcd_string_P() used for PROGMEM strings
3. Confirm character is printable ASCII (32-126)
4. Test with single character: lcd_char('A')

### "Lines appear in wrong places"
1. Confirm x/y parameters not swapped
2. Check coordinate ranges (x: 0-63, y: 0-127)
3. Verify controller split logic (y<64 vs y>=64)
4. Test with GLCD_Dot() at known coordinates

### "Shapes disappear partially"
1. Check if coordinates go off-screen
2. Look for y=64 boundary issues
3. Verify both controllers initialized
4. Test with Demo 3 (page addressing)

### "Simulator crashes"
1. Ensure SimulIDE 0.4.15 (older versions unstable)
2. Check for infinite loops without delays
3. Verify .elf file matches current code (rebuild)
4. Restart simulator and reload circuit

---

## RESOURCES FOR INSTRUCTORS

### Documentation Files (Print for students)
- **GLCD_QUICK_REFERENCE.md:** API and hardware reference
- **GLCD_FLOW_DIAGRAMS.md:** Visual learning aids (10 diagrams)
- **Main.c:** Well-commented demo code (600 lines)

### External Resources
- **KS0108 Datasheet:** [Link to manufacturer PDF]
- **Bresenham's Algorithm:** [Wikipedia article]
- **SimulIDE Forum:** [https://simulide.forumotion.com/]
- **AVR-GCC Tutorial:** [GNU AVR documentation]

### Recommended Reading
- "The Art of Electronics" - Horowitz & Hill (Chapter on displays)
- "Making Embedded Systems" - Elecia White (Graphics section)
- ATmega128 Datasheet - Atmel (Parallel port configuration)

### Video Resources (if available)
- KS0108 wiring tutorial (YouTube)
- Bresenham's algorithm visualization
- SimulIDE walkthrough for GLCD projects

---

## EXTENSION ACTIVITIES (For advanced students)

### Challenge 1: Bitmap Images
- Store 128×64 bitmap in PROGMEM
- Write function to display full-screen image
- Create 2-3 frame animation

### Challenge 2: Scrolling Text
- Implement horizontal text scrolling
- Add marquee effect for long messages
- Control speed with delays

### Challenge 3: Custom Fonts
- Design 3×5 small font for more text
- Implement font switching function
- Create digit-only 7-segment style font

### Challenge 4: Touch Sensor Integration
- Simulate button inputs (left/right/select)
- Create interactive menu navigation
- Highlight selected menu item

### Challenge 5: Real Sensor Dashboard
- Connect ADC sensors (temperature, light, etc.)
- Update display with live readings
- Add min/max tracking and display

---

## COURSE INTEGRATION

This Graphics_Display module fits into SOC 3050 curriculum:

**Before this module:**
- Port_Basic (GPIO fundamentals)
- Serial_Communications (UART, interrupts)

**After this module:**
- ADC sensors (readings displayed on GLCD)
- I2C/SPI (external memory, advanced sensors)
- Final project (combines all modules)

**Suggested Schedule:**
- Weeks 1-2: Port_Basic
- Weeks 3-5: Serial_Communications
- Weeks 6-9: Graphics_Display (this module)
- Weeks 10-12: Sensor integration
- Weeks 13-15: Final projects

---

## INSTRUCTOR NOTES

### Setup Before Semester
1. Test all 10 demos on SimulIDE 0.4.15
2. Verify hardware builds (if using real boards)
3. Print 3 copies of GLCD_QUICK_REFERENCE.md (for lab)
4. Prepare demo videos (optional, for online students)

### Week 1 Preparation
- Load Demo 1 on instructor station
- Prepare quiz on coordinate system
- Print Diagram 1 from GLCD_FLOW_DIAGRAMS.md (large poster)

### Week 2 Preparation
- Set up practical exam station
- Prepare shape drawing handout
- Print Diagram 5 (Bresenham's algorithm)

### Week 3 Preparation
- Create sample sensor dashboard (reference implementation)
- Prepare peer review rubric

### Week 4 Preparation
- Finalize project options list
- Set up project help desk hours
- Prepare final exam questions

### Grading Tips
- Use automated build checker (CLI tools available)
- Grade code quality with rubric (consistency)
- Allow resubmissions for Week 1-2 labs (learning curve)
- Provide example solutions AFTER deadlines

---

## CHANGE LOG

**Version 1.0 (2025):**
- Initial documentation for Graphics_Display module
- 10 progressive demos with PROGMEM optimization
- 4-week teaching plan with assessments
- Comprehensive troubleshooting guide

**Future Improvements:**
- Add animated GIF demos (SimulIDE recordings)
- Create auto-grader script for common tasks
- Expand challenge activities
- Add hardware troubleshooting (oscilloscope guide)

---

## CONTACT

**Course Coordinator:** [Instructor Name]  
**Teaching Assistants:** [TA Names]  
**Office Hours:** [Schedule]  
**Email:** [Contact Email]  

**For technical issues with code:**  
- Check GLCD_QUICK_REFERENCE.md first
- Review GLCD_FLOW_DIAGRAMS.md diagrams
- Post on course forum (response within 24 hours)
- Attend TA office hours

---

**Last Updated:** 2025  
**Course:** SOC 3050 - Embedded Systems and Applications  
**Module:** Graphics LCD Programming (KS0108)
