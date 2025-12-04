# GLCD (KS0108) Flow Diagrams - ATmega128

**COURSE:** SOC 3050 - Embedded Systems and Applications  
**TOPIC:** Graphic LCD Visual Learning Aid  
**PURPOSE:** Flowcharts and diagrams for GLCD programming

---

## DIAGRAM 1: Screen Coordinate System

```
KS0108 GLCD 128×64 COORDINATE SYSTEM
=====================================

        Y-AXIS (Columns) →
     0    10   20   30   40   50   60 64  70   80   90  100  110  120 127
  0  +----+----+----+----+----+----+----+----+----+----+----+----+----+
     |              CS1 (Left Controller)   | CS2 (Right Controller)   |
 10  |                                      |                          |
     |    (x=10, y=20)                     |        (x=10, y=90)      |
 20  |         ●                            |             ●            |
X    |                                      |                          |
|  30|                                      |                          |
A    |                                      |                          |
X  40|                                      |                          |
I    |                                      |                          |
S  50|                                      |                          |
     |                                      |                          |
↓  60|                                      |                          |
     |                                      |                          |
  63 +----+----+----+----+----+----+----+----+----+----+----+----+----+
                                           ^
                                      Split at y=64

KEY CONCEPTS:
- Origin (0,0) = Top-left corner
- X-axis: ROWS (vertical, 0..63)  ← UNUSUAL!
- Y-axis: COLUMNS (horizontal, 0..127)
- Split: CS1 controls y=0..63, CS2 controls y=64..127
```

---

## DIAGRAM 2: Page-Based Memory Layout

```
PAGE-BASED MEMORY ORGANIZATION
===============================

Screen: 128 columns × 64 rows
        Divided into 8 PAGES (each page = 8 rows)

+---------------------------------------------------------------+
| Page 0: Rows 0-7    | 128 bytes (one byte per column)       |
+---------------------------------------------------------------+
| Page 1: Rows 8-15   | 128 bytes                              |
+---------------------------------------------------------------+
| Page 2: Rows 16-23  | 128 bytes                              |
+---------------------------------------------------------------+
| Page 3: Rows 24-31  | 128 bytes                              |
+---------------------------------------------------------------+
| Page 4: Rows 32-39  | 128 bytes                              |
+---------------------------------------------------------------+
| Page 5: Rows 40-47  | 128 bytes                              |
+---------------------------------------------------------------+
| Page 6: Rows 48-55  | 128 bytes                              |
+---------------------------------------------------------------+
| Page 7: Rows 56-63  | 128 bytes                              |
+---------------------------------------------------------------+

BYTE-TO-PIXEL MAPPING (within one byte at page P, column C):

Byte value: 0b10110101

Bit 0 (LSB) → Row = P×8 + 0
Bit 1       → Row = P×8 + 1
Bit 2       → Row = P×8 + 2
Bit 3       → Row = P×8 + 3
Bit 4       → Row = P×8 + 4
Bit 5       → Row = P×8 + 5
Bit 6       → Row = P×8 + 6
Bit 7 (MSB) → Row = P×8 + 7

Example: Writing 0xFF to Page 2, Column 50
  → Lights up rows 16-23, column 50 (8 vertical pixels)
```

---

## DIAGRAM 3: Initialization Flow

```
GLCD INITIALIZATION SEQUENCE
=============================

Start Program
     |
     v
+-------------------+
| init_devices()    |  <- Initialize ATmega128 ports
+-------------------+
     |
     v
+-------------------+
| Configure GPIO    |
| - Data port (8-bit)|
| - Control lines   |
|   RS, RW, E, CS1, CS2|
+-------------------+
     |
     v
+-------------------+
| Reset both        |
| controllers       |
| (CS1 and CS2)     |
+-------------------+
     |
     v
+-------------------+
| Send init commands|
| - Display ON      |
| - Start line = 0  |
| - Page = 0        |
| - Column = 0      |
+-------------------+
     |
     v
+-------------------+
| lcd_clear()       |
| - Clear all pages |
| - Reset cursors   |
+-------------------+
     |
     v
Ready for Drawing
```

---

## DIAGRAM 4: Text Rendering Flow

```
TEXT STRING RENDERING
=====================

lcd_string(row, col, "HELLO")
         |
         v
+------------------------+
| Calculate start address|
| Page = row             |
| Column = col × 6       |
+------------------------+
         |
         v
+------------------------+
| For each character:    |
+------------------------+
         |
   +-----+-----+
   |           |
   v           v
[CS1?]     [CS2?]
   |           |
   v           v
Column    Column
0..63     64..127
   |           |
   +-----+-----+
         |
         v
+------------------------+
| Look up 5x7 font data  |
| from font array        |
| (5 bytes per char)     |
+------------------------+
         |
         v
+------------------------+
| Write 5 bytes          |
| + 1 blank (spacing)    |
| Total: 6 bytes/char    |
+------------------------+
         |
         v
+------------------------+
| Increment column       |
| Pointer by 6           |
+------------------------+
         |
         v
    [More chars?]
         |
    Yes  |  No
    +----+----+
    |         |
    v         v
  Loop    Return
```

---

## DIAGRAM 5: Line Drawing Algorithm (Bresenham's)

```
GLCD_Line(x1, y1, x2, y2)
==========================

Start
  |
  v
+-------------------------+
| Calculate:              |
| dx = |x2 - x1|          |
| dy = |y2 - y1|          |
| sx = (x1<x2) ? 1 : -1   |
| sy = (y1<y2) ? 1 : -1   |
+-------------------------+
  |
  v
+-------------------------+
| err = dx - dy           |
+-------------------------+
  |
  v
+-------------------------+
| x = x1, y = y1          |
+-------------------------+
  |
  v
+-------------------------+
| while (true):           |
|   GLCD_Dot(x, y)        |  <- Draw pixel
|                         |
|   if (x==x2 && y==y2)   |
|     break               |
|                         |
|   e2 = 2 * err          |
|                         |
|   if (e2 > -dy):        |
|     err -= dy           |
|     x += sx             |
|                         |
|   if (e2 < dx):         |
|     err += dx           |
|     y += sy             |
+-------------------------+
  |
  v
End

KEY: Bresenham uses only integer arithmetic (fast!)
```

---

## DIAGRAM 6: Dual-Controller Addressing

```
DUAL-CONTROLLER WRITE OPERATION
================================

Write to coordinate (x=30, y=90)
         |
         v
+------------------------+
| Determine controller:  |
| if (y < 64):           |
|   Use CS1 (Left)       |
|   local_y = y          |
| else:                  |
|   Use CS2 (Right)      |
|   local_y = y - 64     |
+------------------------+
         |
         v
+------------------------+
| Calculate page:        |
| page = x / 8           |
| (Example: 30/8 = 3)    |
+------------------------+
         |
         v
+------------------------+
| Calculate column:      |
| col = local_y          |
| (Example: 90-64 = 26)  |
+------------------------+
         |
         v
+------------------------+
| Send commands:         |
| - Select controller    |
|   (CS1=LOW or CS2=LOW) |
| - Set page (0xB8+page) |
| - Set column (0x40+col)|
+------------------------+
         |
         v
+------------------------+
| Write data byte        |
| - RS=HIGH (data mode)  |
| - Send 8-bit value     |
| - Pulse E (enable)     |
+------------------------+
         |
         v
Done

CRITICAL: Must track which controller is active!
```

---

## DIAGRAM 7: Circle Drawing (Midpoint Algorithm)

```
GLCD_Circle(cx, cy, radius)
===========================

Start
  |
  v
+---------------------------+
| x = 0                     |
| y = radius                |
| d = 1 - radius            |
+---------------------------+
  |
  v
+---------------------------+
| while (x <= y):           |
|                           |
|   Draw 8 symmetric points:|
|   (cx+x, cy+y)            |
|   (cx-x, cy+y)            |
|   (cx+x, cy-y)            |
|   (cx-x, cy-y)            |
|   (cx+y, cy+x)            |
|   (cx-y, cy+x)            |
|   (cx+y, cy-x)            |
|   (cx-y, cy-x)            |
|                           |
|   if (d < 0):             |
|     d += 2*x + 3          |
|   else:                   |
|     d += 2*(x-y) + 5      |
|     y--                   |
|   x++                     |
+---------------------------+
  |
  v
End

ADVANTAGE: Only computes 1/8 of circle, uses symmetry for rest
```

---

## DIAGRAM 8: Ten Demo Progression

```
GRAPHICS_DISPLAY DEMO SEQUENCE
===============================

Week 1: Text and Memory Basics
    |
    +---> Demo 1: Text Header
    |       - lcd_string() usage
    |       - Text positioning
    |
    +---> Demo 2: Pixel Drawing
    |       - GLCD_Dot() function
    |       - Checker pattern
    |
    +---> Demo 3: Page Addressing
            - Memory organization
            - Dual controllers

Week 2: Drawing Primitives
    |
    +---> Demo 4: Lines
    |       - GLCD_Line() function
    |       - Bresenham's algorithm
    |
    +---> Demo 5: Rectangles
    |       - GLCD_Rectangle()
    |       - Nested shapes
    |
    +---> Demo 6: Circles
            - GLCD_Circle()
            - Concentric patterns

Week 3: Text Layout
    |
    +---> Demo 7: Text Pages
            - Left/right controller text
            - Character grid

Week 4: Advanced Patterns
    |
    +---> Demo 8: Radiating Lines
    |       - Pattern generation
    |       - Loop-based drawing
    |
    +---> Demo 9: Nested Shapes
    |       - Complex compositions
    |       - Inset calculations
    |
    +---> Demo 10: Grid Pattern
            - Alignment tools
            - Regular spacing
```

---

## DIAGRAM 9: Common Mistake - Coordinate Confusion

```
COMMON ERROR: X/Y CONFUSION
===========================

Student thinks (WRONG):
    x = horizontal, y = vertical (like math class)

Actual KS0108 convention (CORRECT):
    x = ROWS (vertical), y = COLUMNS (horizontal)

Visual comparison:

WRONG ASSUMPTION:          ACTUAL GLCD:
+------ y →               +------ y (cols) →
|                         |
x                         x (rows)
↓                         ↓

To draw at "row 20, column 50":

WRONG CODE:
  GLCD_Dot(50, 20);  // Parameters reversed!

CORRECT CODE:
  GLCD_Dot(20, 50);  // x=row=20, y=col=50

TEACHING TIP: Always say "row" and "column" instead of "x" and "y"
              until students internalize the convention.
```

---

## DIAGRAM 10: Performance Comparison

```
DRAWING PERFORMANCE (Approximate timing on ATmega128 @ 16MHz)
==============================================================

Operation            | Time    | Notes
---------------------|---------|--------------------------------
lcd_clear()          | ~200ms  | Clears all 8192 pixels
GLCD_Dot(x,y)        | ~0.5ms  | Single pixel
GLCD_Line(short)     | ~5ms    | 10-pixel line
GLCD_Line(long)      | ~50ms   | 100-pixel line
GLCD_Circle(r=10)    | ~20ms   | Small circle
GLCD_Circle(r=30)    | ~60ms   | Large circle
GLCD_Rectangle()     | ~15ms   | Outline only
lcd_string(20 chars) | ~30ms   | Full row of text

OPTIMIZATION STRATEGIES:
1. Minimize lcd_clear() calls
2. Use page writes for vertical bars (8 pixels at once)
3. Batch multiple draw operations before delay
4. Pre-calculate coordinates in loops
5. Avoid redrawing unchanged regions

SIMULATOR vs HARDWARE:
- SimulIDE: Instant updates (no real timing)
- Hardware: Delays visible, especially for lcd_clear()
- Add longer _delay_ms() for hardware demonstrations
```

---

## RESOURCES

- **KS0108 Datasheet:** Controller command reference
- **ATmega128 Datasheet:** Parallel port configuration
- **Lab Manual:** Graphics_Display exercises
- **Quick Reference:** GLCD_QUICK_REFERENCE.md
- **Code:** projects/Graphics_Display/Main.c

---

**Last Updated:** 2025  
**For:** SOC 3050 Students  
**Print for lab visual reference!**
