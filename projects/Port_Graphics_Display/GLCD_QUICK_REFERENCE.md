# GLCD (KS0108) Quick Reference - ATmega128

**COURSE:** SOC 3050 - Embedded Systems and Applications  
**TOPIC:** Graphic LCD Programming (128×64 pixels)  
**PURPOSE:** Student quick reference for KS0108 GLCD API

---

## HARDWARE SPECIFICATIONS

### Display Characteristics
- **Resolution:** 128 columns × 64 rows (8,192 pixels)
- **Controller:** KS0108 (dual-chip architecture)
- **Color:** Monochrome (1-bit: ON/OFF)
- **Backlight:** LED (optional, controllable)
- **Contrast:** Adjustable via potentiometer (0-5V)
- **Power:** 5V, ~100mA (with backlight)

### Dual-Controller Architecture
```
+---------------------------+---------------------------+
|    CS1 (Left Half)        |    CS2 (Right Half)       |
|    Columns 0-63           |    Columns 64-127         |
|    Y = 0..63              |    Y = 64..127            |
+---------------------------+---------------------------+
         ATmega128 Interface (8-bit parallel data + control lines)
```

**Key Concept:** Screen is split at y=64 midpoint. Each controller manages 64×64 pixels independently.

---

## COORDINATE SYSTEM

### Screen Orientation
```
        Y-axis (Columns) →
     0         64         127
  0  +----------+----------+
     |  CS1     |  CS2     |
X    |  Left    |  Right   |
|    |  Half    |  Half    |
↓    |          |          |
  63 +----------+----------+

Origin (0,0) = Top-left corner
X-axis: Rows (0..63, vertical)
Y-axis: Columns (0..127, horizontal)
```

### Critical Notes
- **X = row** (vertical position, 0 at top, 63 at bottom)
- **Y = column** (horizontal position, 0 at left, 127 at right)
- **This is OPPOSITE of typical (x,y) math conventions!**
- Many students initially confuse X/Y—always verify visually

---

## MEMORY ORGANIZATION

### Page-Based Layout
```
Screen divided into 8 pages (each page = 8 rows tall)

Page 0:  Rows 0-7    (0x00)
Page 1:  Rows 8-15   (0x01)
Page 2:  Rows 16-23  (0x02)
Page 3:  Rows 24-31  (0x03)
Page 4:  Rows 32-39  (0x04)
Page 5:  Rows 40-47  (0x05)
Page 6:  Rows 48-55  (0x06)
Page 7:  Rows 56-63  (0x07)

Each page has 128 columns (bytes)
Writing 0xFF to (page 2, column 10) → lights 8 vertical pixels at rows 16-23
```

### Why Pages?
- **Optimization:** Writing 8 pixels at once (vertical byte)
- **Speed:** Faster than pixel-by-pixel updates
- **Hardware limitation:** KS0108 controller architecture

### Bit-to-Pixel Mapping (within a byte)
```
Byte value: 0b10110101 written to (page, column)

Bit 0 → Row = (page × 8) + 0  (top pixel)
Bit 1 → Row = (page × 8) + 1
Bit 2 → Row = (page × 8) + 2
...
Bit 7 → Row = (page × 8) + 7  (bottom pixel)

Example: Writing 0xFF to page 3, column 50
  → Lights rows 24-31, column 50 (all 8 pixels ON)
```

---

## TEXT RENDERING

### Text Grid
```
20 characters wide × 8 rows tall

Character positions (column units):
0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  | Row 0
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  | Row 1
...
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  | Row 7
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+

Each character slot: 6 pixels wide (5×7 font + 1 spacing)
Each row: 8 pixels tall
```

### Font Details
- **Font type:** 5×7 dot matrix (ASCII printable characters)
- **Character slot:** 6 pixels wide (includes 1-pixel spacing)
- **Row height:** 8 pixels (font height + 1-pixel spacing)
- **Character set:** 0x20-0x7F (space through tilde)
- **Custom characters:** Possible by modifying font array

### Text vs Pixel Coordinates
| Text Function | Text Coords | Pixel Coords |
|---------------|-------------|--------------|
| `lcd_string(0, 0, "A")` | Row 0, Col 0 | x=0-7, y=0-5 |
| `lcd_string(1, 5, "B")` | Row 1, Col 5 | x=8-15, y=30-35 |
| `lcd_string(7, 19, "Z")` | Row 7, Col 19 | x=56-63, y=114-119 |

---

## API FUNCTION REFERENCE

### Initialization Functions

#### `void lcd_init(void)`
**Purpose:** Initialize GLCD hardware and controllers  
**Usage:** Call once at startup, before any drawing  
**Example:**
```c
lcd_init();
```

#### `void lcd_clear(void)`
**Purpose:** Clear entire screen (all pixels OFF)  
**Usage:** Call before drawing new content  
**Example:**
```c
lcd_clear();
```

---

### Text Functions

#### `void lcd_xy(byte row, byte col)`
**Purpose:** Position cursor for next text output  
**Parameters:**
- `row`: Text row (0..7)
- `col`: Character column (0..19)

**Example:**
```c
lcd_xy(3, 10);         // Position at row 3, column 10
GLCD_4DigitDecimal(1234);  // Print number at cursor position
```

#### `void lcd_char(byte character)`
**Purpose:** Print single ASCII character at current cursor  
**Parameters:**
- `character`: ASCII code (0x20-0x7F)

**Example:**
```c
lcd_xy(2, 5);
lcd_char('A');         // Print 'A' at row 2, column 5
```

#### `void lcd_string(byte row, byte col, char *string)`
**Purpose:** Print null-terminated string  
**Parameters:**
- `row`: Text row (0..7)
- `col`: Starting character column (0..19)
- `string`: Pointer to null-terminated string (in RAM)

**Example:**
```c
lcd_string(0, 0, "Hello World");
lcd_string(1, 5, "SOC3050");
```

**Note:** For PROGMEM strings, use helper function:
```c
// String stored in flash
const char STR_TITLE[] PROGMEM = "ATmega128";

// Helper to print PROGMEM string
void lcd_string_P(byte row, byte col, const char *progmem_str) {
    char buffer[21];
    strncpy_P(buffer, progmem_str, 20);
    buffer[20] = '\0';
    lcd_string(row, col, buffer);
}

// Usage
lcd_string_P(0, 0, STR_TITLE);
```

---

### Graphics Primitives

#### `void GLCD_Dot(unsigned char x, unsigned char y)`
**Purpose:** Draw single pixel  
**Parameters:**
- `x`: Row (0..63)
- `y`: Column (0..127)

**Example:**
```c
GLCD_Dot(32, 64);      // Draw pixel at center of screen
```

**Notes:**
- Turns pixel ON (no erase function in basic API)
- To erase, redraw entire region or use `lcd_clear()`

#### `void GLCD_Line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2)`
**Purpose:** Draw line between two points  
**Parameters:**
- `(x1, y1)`: Starting point (row, column)
- `(x2, y2)`: Ending point (row, column)

**Algorithm:** Bresenham's line algorithm (fast, integer-only)

**Example:**
```c
GLCD_Line(10, 20, 50, 100);  // Diagonal line
GLCD_Line(20, 30, 20, 80);   // Horizontal line
GLCD_Line(10, 50, 40, 50);   // Vertical line
```

#### `void GLCD_Rectangle(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2)`
**Purpose:** Draw rectangle outline  
**Parameters:**
- `(x1, y1)`: Top-left corner
- `(x2, y2)`: Bottom-right corner

**Example:**
```c
GLCD_Rectangle(10, 20, 40, 80);  // Rectangle outline
```

**Notes:**
- Draws outline only (not filled)
- Ensure x1 < x2 and y1 < y2

#### `void GLCD_Circle(unsigned char x, unsigned char y, unsigned char r)`
**Purpose:** Draw circle outline  
**Parameters:**
- `(x, y)`: Center point
- `r`: Radius (pixels)

**Algorithm:** Midpoint circle algorithm (8-way symmetry)

**Example:**
```c
GLCD_Circle(32, 64, 20);  // Circle at screen center, radius 20
```

**Boundary check:**
```c
// Ensure circle fits on screen
if (x - r >= 0 && x + r < GLCD_ROWS &&
    y - r >= 0 && y + r < GLCD_COLS) {
    GLCD_Circle(x, y, r);
}
```

---

### Number Display Functions

#### `void GLCD_2DigitDecimal(unsigned char number)`
**Purpose:** Display 2-digit decimal number (00-99)  
**Parameters:**
- `number`: Value 0-99

**Example:**
```c
lcd_xy(3, 10);
GLCD_2DigitDecimal(42);    // Displays "42"
```

#### `void GLCD_3DigitDecimal(unsigned int number)`
**Purpose:** Display 3-digit decimal number (000-999)  
**Parameters:**
- `number`: Value 0-999

**Example:**
```c
lcd_xy(4, 5);
GLCD_3DigitDecimal(567);   // Displays "567"
```

#### `void GLCD_4DigitDecimal(unsigned int number)`
**Purpose:** Display 4-digit decimal number (0000-9999)  
**Parameters:**
- `number`: Value 0-9999

**Example:**
```c
lcd_xy(5, 8);
GLCD_4DigitDecimal(1234);  // Displays "1234"
```

**Note:** Leading zeros displayed (e.g., 42 → "0042")

---

### Advanced/Low-Level Functions

#### `void GLCD_Axis_xy(unsigned char page, unsigned char column)`
**Purpose:** Set page/column address for direct byte writes  
**Parameters:**
- `page`: Page number (0..7)
- `column`: Column number (0..127)

**Example:**
```c
GLCD_Axis_xy(3, 50);       // Position at page 3, column 50
datal(0xFF);               // Write byte to left controller
```

#### `void datal(byte data)` / `void datar(byte data)`
**Purpose:** Write byte directly to GLCD (left/right controller)  
**Parameters:**
- `data`: 8-bit value (each bit = 1 pixel)

**Example:**
```c
// Draw vertical bar at column 10, pages 2-5
for (byte page = 2; page <= 5; page++) {
    GLCD_Axis_xy(page, 10);
    datal(0xFF);           // Write to left controller
}
```

#### `void ScreenBuffer_clear(void)`
**Purpose:** Clear software framebuffer (if used)  
**Note:** Some implementations maintain shadow buffer for read-modify-write

**Example:**
```c
ScreenBuffer_clear();
// Now safe to use GLCD_Dot() without artifacts
```

---

## COMMON PATTERNS

### Pattern 1: Full Screen Border
```c
// Top and bottom borders
for (byte y = 0; y < GLCD_COLS; y++) {
    GLCD_Dot(0, y);                // Top edge
    GLCD_Dot(GLCD_ROWS - 1, y);    // Bottom edge
}

// Left and right borders
for (byte x = 0; x < GLCD_ROWS; x++) {
    GLCD_Dot(x, 0);                // Left edge
    GLCD_Dot(x, GLCD_COLS - 1);    // Right edge
}
```

### Pattern 2: Centered Text
```c
const char *text = "Hello World";
byte text_len = strlen(text);
byte start_col = (TEXT_COLS - text_len) / 2;
lcd_string(3, start_col, text);
```

### Pattern 3: Progress Bar
```c
void draw_progress_bar(byte row, byte percent) {
    byte bar_width = (TEXT_COLS * percent) / 100;
    lcd_xy(row, 0);
    for (byte i = 0; i < bar_width; i++) {
        lcd_char(0xFF);  // Solid block character
    }
}

// Usage
draw_progress_bar(5, 75);  // 75% progress at row 5
```

### Pattern 4: Checkerboard
```c
for (byte x = 0; x < GLCD_ROWS; x++) {
    for (byte y = 0; y < GLCD_COLS; y++) {
        if (((x >> 3) + (y >> 3)) & 1) {
            GLCD_Dot(x, y);
        }
    }
}
```

### Pattern 5: Crosshair (Center Marker)
```c
byte cx = GLCD_ROWS / 2;
byte cy = GLCD_COLS / 2;

// Horizontal line
GLCD_Line(cx, cy - 10, cx, cy + 10);
// Vertical line
GLCD_Line(cx - 10, cy, cx + 10, cy);
```

### Pattern 6: Simple Animation (Bouncing Dot)
```c
byte x = 0, y = 0;
signed char dx = 1, dy = 1;

while (1) {
    lcd_clear();
    GLCD_Dot(x, y);
    _delay_ms(50);
    
    x += dx;
    y += dy;
    
    if (x == 0 || x == GLCD_ROWS - 1) dx = -dx;
    if (y == 0 || y == GLCD_COLS - 1) dy = -dy;
}
```

---

## DEBUGGING CHECKLIST

### Display Shows Nothing
- [ ] Is GLCD properly powered? (Check 5V and GND)
- [ ] Is contrast adjusted? (Turn potentiometer)
- [ ] Is `lcd_init()` called before drawing?
- [ ] Are data/control lines connected correctly?
- [ ] Check for loose connections/wiring

### Garbled or Partial Display
- [ ] Is clock speed correct? (16MHz crystal)
- [ ] Are timing delays appropriate? (`_delay_ms()`)
- [ ] Is left/right controller split correct? (y=64)
- [ ] Check for buffer overruns (coordinates out of bounds)

### Text Not Displaying
- [ ] Is `lcd_string()` called (not just `lcd_xy()`)?
- [ ] Is string null-terminated?
- [ ] Are row/col within bounds? (row 0..7, col 0..19)
- [ ] Is font library linked correctly?

### Graphics Primitives Not Working
- [ ] Are coordinates within bounds? (x 0..63, y 0..127)
- [ ] Is background cleared first? (`lcd_clear()`)
- [ ] Check for overlapping draws (later draws overwrite)
- [ ] Verify contrast setting (may be too faint)

### Simulator vs Hardware Differences
- [ ] SimulIDE 0.4.15 required (newer versions have bugs)
- [ ] Timing differs: add longer delays for hardware
- [ ] Hardware may need contrast adjustment
- [ ] Check physical connections on real hardware

---

## PERFORMANCE TIPS

### Optimization Strategies

1. **Minimize `lcd_clear()` calls**
   - Clearing entire screen is slow (~200ms)
   - Redraw only changed regions when possible

2. **Use page writes for vertical lines**
   ```c
   // Fast vertical line
   GLCD_Axis_xy(page, y);
   datal(0xFF);  // 8 pixels at once
   ```

3. **Batch operations**
   ```c
   // Draw multiple primitives before delay
   GLCD_Line(...);
   GLCD_Circle(...);
   GLCD_Rectangle(...);
   _delay_ms(100);  // One delay at end
   ```

4. **Use lookup tables for patterns**
   ```c
   const byte patterns[] PROGMEM = {0xAA, 0x55, 0xFF, 0x00};
   ```

5. **Avoid floating-point math**
   - Use integer arithmetic only
   - Pre-calculate values where possible

---

## COORDINATE CONVERSION

### Text to Pixel
```c
byte text_row_to_pixel_x(byte row) {
    return row * 8;  // Each text row = 8 pixels
}

byte text_col_to_pixel_y(byte col) {
    return col * 6;  // Each char = 6 pixels wide
}
```

### Pixel to Page
```c
byte pixel_x_to_page(byte x) {
    return x / 8;    // 8 rows per page
}

byte pixel_x_to_bit(byte x) {
    return x % 8;    // Bit within page byte
}
```

---

## RESOURCES

- **KS0108 Datasheet:** Complete controller specification
- **ATmega128 Datasheet:** Port configuration for parallel interface
- **Lab Manual:** Graphics_Display project exercises
- **Code:** projects/Graphics_Display/Main.c (10 demos)
- **Simulator:** SimulIDE 0.4.15 (tools/simulide/)

---

**Last Updated:** 2025  
**For:** SOC 3050 Students  
**Keep this reference handy during graphics labs!**
