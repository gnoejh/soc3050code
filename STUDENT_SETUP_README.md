# 🎓 ATmega128 Educational Framework - Student Setup Guide

**Complete Learning Progression: Assembly → C → Python → IoT**

## 🚀 Quick Start (After Cloning from GitHub)

### **1. Clone the Repository**
```bash
git clone https://github.com/[your-username]/soc3050code.git
cd soc3050code
```

### **2. Run Student Setup (One Command!)**
```powershell
.\setup.ps1
```

**That's it!** This single command will:
- ✅ Check Python installation
- ✅ Install all required Python packages
- ✅ Test the complete framework
- ✅ Create student tools and shortcuts
- ✅ Validate everything works

### **3. Start Learning!**
```powershell
# Connect your ATmega128 and run:
.\student.ps1 demo
```

---

## 📋 What Students Get After Setup

### **🎮 Ready-to-Use Commands:**
```powershell
.\student.ps1 demo                    # Complete educational progression
.\student.ps1 monitor -Duration 120   # Real-time sensor monitoring
.\student.ps1 visualize data.csv      # Scientific data analysis
.\student.ps1 build                   # Build C code
.\student.ps1 program                 # Program ATmega128
.\student.ps1 check                   # System diagnostics
```

### **🎯 Complete Learning Path:**
1. **Assembly Concepts** - Direct register manipulation
2. **C Programming** - Function abstraction and modularity
3. **Serial Communication** - UART protocols and data exchange
4. **Python Integration** - Real-time hardware communication
5. **Data Science** - Scientific analysis and visualization

### **📁 Key Directories:**
```
soc3050code/
├── Main/                    # C code and build scripts
├── python_env/              # Python integration framework
├── student.ps1              # Main student interface
├── setup.ps1                        # One-time setup script
└── README.md               # This guide
```

---

## 🔧 System Requirements

- **Windows 10/11**
- **Python 3.8+** ([Download here](https://python.org) or Microsoft Store)
- **Internet connection** (for package installation)
- **ATmega128 board** connected via USB
- **Microchip Studio 7.0** (for C compilation)

---

## 🎓 Educational Framework Features

### **Assembly → C Progression:**
- Direct register manipulation (DDRB, PORTB)
- Function-based abstraction
- Interrupt handling and timers
- Serial communication protocols

### **Python Integration:**
- Real-time hardware communication
- Sensor data collection and logging
- Scientific visualization with matplotlib/seaborn
- Statistical analysis with pandas/numpy

### **Data Science Applications:**
- Real-time plotting and dashboards
- CSV data analysis and correlation
- Professional scientific visualizations
- Foundation for IoT and web applications

---

## 🚨 Troubleshooting

### **Setup Script Fails:**
```powershell
# Check Python installation
python --version

# Manual package installation
cd python_env
python setup.py
```

### **ATmega128 Not Found:**
1. Check Device Manager for COM port
2. Update port in commands: `.\student.ps1 demo -Port COM5`
3. Ensure ATmega128 drivers are installed

### **Build Errors:**
1. Install Microchip Studio 7.0
2. Check that AVR-GCC is in PATH
3. Run: `.\student.ps1 build` from root directory

### **Python Import Errors:**
```powershell
# Reinstall packages
cd python_env
python -m pip install --force-reinstall -r requirements.txt
```

---

## 📖 Documentation

- **[Complete Framework Guide](README.md)** - Full documentation
- **[Python Integration Guide](python_env/README.md)** - Python-specific details
- **[C Programming Examples](Main/)** - Hardware programming reference

---

## 🎯 Learning Outcomes

After completing this framework, students will understand:

- ✅ **Low-level Programming** - Direct hardware control
- ✅ **Communication Protocols** - Serial/UART data exchange  
- ✅ **System Integration** - Hardware ↔ Software interfaces
- ✅ **Data Science** - Real sensor data → analysis → visualization
- ✅ **Modern Workflow** - Complete embedded → cloud development pipeline

---

## 🎉 Ready for Modern Embedded Development!

This framework bridges traditional embedded programming with modern data science, giving students the complete skillset needed for IoT and connected device development.

**From bare metal register manipulation to professional data visualization - all in one educational progression!** 🚀