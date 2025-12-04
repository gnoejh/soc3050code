# 🎓 Project Launcher Dashboard - Quick Reference

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)

## 🚀 Quick Start (3 Steps)

1. **Start Dashboard**: Double-click `start_project_launcher.bat`
2. **Open Browser**: Go to http://localhost:5001
3. **Select Project**: Click any project → Click "Build & Run"

## 🎯 Main Features

| Feature | Description | Location |
|---------|-------------|----------|
| **Project Browser** | Browse 35+ educational projects | Left sidebar |
| **Device Selector** | Choose SimulIDE or Physical Kit | Top main panel |
| **Build & Run** | Compile and deploy projects | Middle main panel |
| **LED Monitor** | See PORTB LED states (bits 7-0) | Right panel top |
| **ADC Monitor** | View analog sensor readings | Right panel middle |
| **Serial Console** | Send/receive serial data | Right panel bottom |

## 📋 Common Tasks

### Run a Project in SimulIDE
```
1. Click "🖥️ SimulIDE" button
2. Select project from left sidebar
3. Click "▶️ Build & Run"
4. SimulIDE opens automatically
5. Watch LEDs and serial output on right
```

### Run on Physical Hardware
```
1. Connect ATmega128 board via USB
2. Click "🔌 Physical Kit" button
3. Click "🔍 Scan Devices"
4. Select project from left sidebar
5. Click "▶️ Build & Run"
6. Monitor live output on right
```

### Send Serial Commands
```
1. Type command in "Serial Monitor" input box
2. Press Enter or click "Send"
3. Response appears in console above
```

### Build Only (No Run)
```
1. Select project
2. Click "🔨 Build"
3. Check build output for errors
4. HEX file created in project folder
```

## 🎨 Interface Guide

### Left Sidebar
- **Search Box**: Filter projects by name
- **Project List**: Click to select
  - Blue = Active project
  - Gray = Not selected

### Main Panel
- **Device Buttons**: 
  - 🖥️ SimulIDE = Blue when active
  - 🔌 Physical Kit = Blue when active
- **Project Details**: Shows description, objectives, demos
- **Control Buttons**:
  - 🔨 Build = Compile only
  - ▶️ Build & Run = Compile + Deploy
  - ⏹️ Stop = Stop running (if supported)
- **Build Output**: Shows compiler messages

### Right Panel
- **LED Display**: 8 LEDs (PB7-PB0)
  - Red glow = ON
  - Gray = OFF
- **ADC Display**: 2 channels (CH0, CH1)
  - Blue progress bar
  - Numeric value (0-1023)
- **Serial Console**:
  - Timestamped messages
  - Auto-scroll
  - Send commands

## ⚡ Keyboard Shortcuts

| Key | Action |
|-----|--------|
| **Ctrl+F** | Focus search box |
| **Enter** (in serial) | Send command |
| **Ctrl+L** | Clear build output |
| **F5** | Reload projects |

## 🔍 Status Indicators

### Header Status (Top Right)
- 🟢 **WebSocket Connected** = Dashboard active
- 🔴 **WebSocket Disconnected** = Refresh page
- 🟢 **Device Connected** = ATmega128 ready
- 🔴 **No Device** = Click "Scan Devices"

### Project Status
- **Blue highlight** = Selected
- **Demos listed** = Multiple functions available
- **"✅ Build successful!"** = Ready to run
- **"❌ Build failed!"** = Check errors

## 🛠️ Troubleshooting Quick Fixes

| Problem | Quick Fix |
|---------|-----------|
| Dashboard won't start | Run: `pip install flask flask-socketio pyserial` |
| Build fails | Check AVR-GCC installed in PATH |
| SimulIDE won't launch | Verify path: `simulators/SimulIDE_1.1.0-SR1_Win64/` |
| No device detected | For simulator: launch SimulIDE first<br>For hardware: check USB cable |
| Serial console empty | 1. Verify device connected<br>2. Check program uses UART/serial |
| Port 5001 in use | Close other apps or change port in config |

## 📊 Data Format Examples

### For Best Auto-Detection in Monitor Panel

**LED States (in your C code)**:
```c
printf("PORTB: 0x%02X\n", PORTB);
```

**ADC Values**:
```c
printf("ADC0: %d\n", adc_value);
printf("ADC1: %d\n", adc_value);
```

**Sensor Data**:
```c
printf("Temperature: %d C\n", temp);
printf("Distance: %d cm\n", distance);
```

## 🎓 Educational Tips

### For Instructors
1. ✅ Use SimulIDE for demonstrations
2. ✅ Show build output to explain errors
3. ✅ Use LED monitor for visual feedback
4. ✅ Demonstrate serial debugging
5. ✅ Let students explore demos independently

### For Students
1. ✅ Start with simple projects (01_Port, 03_Serial)
2. ✅ Read learning objectives before running
3. ✅ Watch build output for compiler learning
4. ✅ Experiment with serial commands
5. ✅ Compare SimulIDE vs hardware behavior

## 📞 Support

**Contact**: Prof. Hong Jeaong  
**LinkedIn**: linkedin.com/in/gnoejh53  
**Institution**: Inha University in Tashkent

## 🎯 Feature Checklist

- [x] Browse 35+ projects
- [x] SimulIDE integration
- [x] Physical hardware support
- [x] One-click build & run
- [x] Real-time LED monitoring
- [x] ADC value visualization
- [x] Serial terminal
- [x] Project search
- [x] Demo selection
- [x] Auto-device detection
- [x] Build output display
- [x] Network access support

---

**Last Updated**: 2025  
**Version**: 1.0.0  
**Dashboard Port**: 5001
