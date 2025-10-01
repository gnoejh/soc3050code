# 🎓 ATmega128 Educational Framework - GitHub Repository

**Complete Learning Progression: Assembly → C → Python → IoT**

## 🚀 For Students: Quick Start After Cloning

### **1. Clone This Repository**
```bash
git clone https://github.com/[your-username]/soc3050code.git
cd soc3050code
```

### **2. One-Command Setup**
```powershell
.\setup.ps1
```

### **3. Start Learning**
```powershell
.\student.ps1 demo
```

**That's it!** The setup script automatically:
- ✅ Checks Python installation
- ✅ Installs all required packages  
- ✅ Tests the complete framework
- ✅ Creates student tools and shortcuts
- ✅ Validates everything works

---

## 🎯 What Students Get

### **Ready-to-Use Commands:**
```powershell
.\student.ps1 demo                    # Complete educational progression
.\student.ps1 monitor -Duration 120   # Real-time sensor monitoring
.\student.ps1 visualize data.csv      # Data analysis & visualization
.\student.ps1 build                   # Build C code
.\student.ps1 program                 # Program ATmega128
.\student.ps1 check                   # System diagnostics
```

### **Complete Learning Path:**
1. **Assembly Concepts** → Direct register manipulation
2. **C Programming** → Function abstraction and modularity
3. **Serial Communication** → UART protocols and data exchange
4. **Python Integration** → Real-time hardware communication
5. **Data Science** → Analysis, visualization, IoT foundation

---

## 📁 Repository Structure

```
soc3050code/
├── setup.ps1                      # One-command setup
├── student.ps1                    # Main student interface
├── README.md                      # Complete documentation
├── GITHUB_SETUP.md               # This quick guide
│
├── Main/                          # C Programming Environment
│   ├── build.ps1, program.ps1    # Build and programming tools
│   ├── educational_demo.c         # Main educational C program
│   └── [C modules]                # Hardware abstraction layers
│
└── python_env/                    # Python Integration Framework
    ├── main.py                    # Main Python application
    ├── setup.py                   # Python environment setup
    └── src/atmega128_educational/ # Python package
```

---

## 🔧 Requirements

- **Windows 10/11**
- **Python 3.8+** ([Download](https://python.org))
- **ATmega128 board** with USB connection
- **Internet connection** (for setup)
- **Microchip Studio 7.0** (for C compilation)

---

## 🎓 Educational Benefits

### **For Students:**
- **Immediate Results** - See C code working through Python interface
- **Real Hardware** - Actual ATmega128 programming, not simulation
- **Modern Workflow** - Complete embedded → data science pipeline
- **Industry Skills** - Professional Python data science tools

### **For Instructors:**
- **Easy Setup** - Students ready to learn in minutes
- **Complete Curriculum** - Assembly → C → Python → IoT progression
- **Real Projects** - Actual sensor data for assessment
- **Future-focused** - Skills for IoT and connected device careers

---

## 🚨 Troubleshooting

### **Setup Issues:**
```powershell
# Check Python
python --version

# Manual setup
cd python_env
python setup.py
```

### **ATmega128 Not Found:**
1. Check Device Manager for COM port
2. Use: `.\student.ps1 demo -Port COM5`
3. Install ATmega128 drivers

---

## 📖 Documentation

- **[Complete Guide](README.md)** - Full framework documentation
- **[Python Integration](python_env/README.md)** - Python-specific details
- **[Quick Setup](STUDENT_SETUP_README.md)** - Student setup guide

---

## 🎉 Ready for Modern Embedded Education!

This framework bridges traditional embedded programming with modern data science, providing students the complete skillset for today's IoT industry.

**From bare metal registers to professional data visualization - all in one repository!** 🚀