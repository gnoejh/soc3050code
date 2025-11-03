# GLCD Library Test

## Purpose
**Library Validation Test Suite** - Verifies that the standardized `_glcd.h` library works correctly.

## Not a Teaching Project!
This is **NOT** for learning graphics LCD programming. For educational LCD tutorials, see:
- **`Graphics_Display/`** - Educational graphics LCD project
- **`LCD_Character_Basic/`** - Character LCD fundamentals

## What This Tests
- ✅ Standardized GLCD library (`_glcd.h`) - upgraded from ks0108_complete
- ✅ All graphics primitives (pixels, lines, shapes)
- ✅ Text rendering and printf support
- ✅ Screen operations (clear, invert)
- ✅ Performance validation

## Tests Performed
1. Display initialization
2. Clear screen
3. Pixel operations (set, clear, XOR)
4. Line drawing (horizontal, vertical, diagonal)
5. Rectangle drawing (outline and filled)
6. Circle drawing (outline and filled)
7. Text rendering
8. Printf functionality
9. Screen inversion
10. Performance test

## Library Upgrade
```
Old: ks0108_complete.h (Korean comments, minimal docs)
       ↓ UPGRADED
New: _glcd.h (Professional standard, full documentation)
```

## Usage
This project is for **framework maintainers** to validate the GLCD library standardization. Students should use existing Graphics_Display projects for learning.
