# 🎯 Student Quick Reference Card

## 🚀 Build & Program Your ATmega128

### ⚡ **RELIABLE Method (Always Works)**
1. **Edit** `Main/config.h` - Choose your example
2. **Press** `Ctrl+Shift+B` - Build the project  
3. **Press** `Ctrl+Shift+P` → "Tasks: Run Task" → "Program ATmega128"
4. **Test** - Open serial monitor

### 🛠️ **Command Palette Method (100% Reliable)**
1. **Edit** `Main/config.h` - Choose your example
2. **Press** `Ctrl+Shift+P`
3. **Type** "Tasks: Run Task"
4. **Select** "Build ATmega128 Project"
5. **Repeat** for "Program ATmega128 (AVRDUDE)"

### 💻 **Terminal Method (Never Fails)**
```powershell
# Navigate to Main folder
cd Main

# Build 
.\build.ps1

# Program
.\program.ps1
```

## ⚠️ **About Shortcuts**

**We use standard VS Code shortcuts that always work:**
- `Ctrl+Shift+B` = Build (VS Code standard)
- `Ctrl+Shift+P` = Command Palette (access all tasks)
- `Ctrl+Shift+M` = Serial Monitor


## 🐍 **Python Examples**

### Before running Python:
```powershell
# Activate environment
.\activate.ps1

# Run examples
python python\examples\quick_start.py
python python\examples\example_01_basic.py
```

## 🎓 **Common Examples in config.h**

```c
// Choose ONE by uncommenting:
#define SERIAL_POLLING_ECHO          // Echo what you type
#define BLINK_PORT                   // Blink LED
#define ADC_POLLING                  // Read sensors
#define GAME_PONG_UART_CONTROL       // Play Pong game
#define GRAPHICS_BOUNCING_BALL       // Graphics demo
```

## 📱 **Serial Communication**

### Test your program:
1. **Build & Program** your chosen example
2. **Open Serial Monitor**: `Ctrl+Shift+M`
3. **Connect** to COM port (usually COM3)
4. **Set baudrate** to 9600
5. **Send messages** and see responses!

## 🆘 **Need Help?**

1. **Documentation**: Check README.md
2. **Troubleshooting**: See VS_Code_Shortcuts_Fix.md  
3. **Ask instructor**: Include error messages
4. **Check examples**: python\examples\README.md

---

**💡 Remember**: There are multiple ways to do everything - if one doesn't work, try another! 🎯