# Tools Directory

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
All rights reserved for educational purposes.  
Contact: linkedin.com/in/gnoejh53

## 📁 Directory Structure

```
tools/
├── cli/                    # Command-line interface tools
│   ├── cli-build-project.ps1
│   ├── cli-build-all.ps1
│   ├── cli-program-project.ps1
│   ├── cli-analyze-code.ps1
│   ├── cli-test-system.ps1
│   └── cli-new-project.ps1
│
├── interfaces/             # Python interfaces
│   └── python_interface.py
│
├── setup/                  # Setup and installation scripts
│   ├── setup.bat
│   ├── quick_check.bat
│   ├── simulate-template.bat
│   ├── deploy-simulators.ps1
│   ├── rename-projects.ps1
│   └── check-simulide-environment.ps1
│
└── simulide/              # SimulIDE specific tools
    ├── cli-simulide.ps1
    └── simulide.config
```

---

## 🛠️ CLI Tools (`cli/`)

### **Build Tools**
- **`cli-build-project.ps1`** - Build a single project
- **`cli-build-all.ps1`** - Build all projects for testing

### **Programming Tools**
- **`cli-program-project.ps1`** - Upload HEX to ATmega128 via programmer

### **Development Tools**
- **`cli-analyze-code.ps1`** - Static code analysis
- **`cli-test-system.ps1`** - System diagnostics
- **`cli-new-project.ps1`** - Create new project from template

---

## 🐍 Python Interfaces (`interfaces/`)

### **`python_interface.py`**
High-level Python interface for ATmega128 communication:
- Serial data logging
- Command interface
- Data visualization
- GUI control panel

**Usage:**
```powershell
python tools/interfaces/python_interface.py
```

---

## ⚙️ Setup Scripts (`setup/`)

### **Initial Setup**
- **`setup.bat`** - Initial environment setup
- **`quick_check.bat`** - Quick system verification

### **SimulIDE Management**
- **`simulate-template.bat`** - Create simulation templates
- **`deploy-simulators.ps1`** - Deploy SimulIDE configurations
- **`check-simulide-environment.ps1`** - Verify SimulIDE installation

### **Project Management**
- **`rename-projects.ps1`** - Batch rename projects

---

## 🖥️ SimulIDE Tools (`simulide/`)

### **`cli-simulide.ps1`**
Command-line SimulIDE launcher and manager

### **`simulide.config`**
SimulIDE configuration file

---

## 📝 Usage Examples

### Build a Project
```powershell
.\tools\cli\cli-build-project.ps1 -ProjectDir "projects\01_Port"
```

### Build All Projects
```powershell
.\tools\cli\cli-build-all.ps1
```

### Program ATmega128
```powershell
.\tools\cli\cli-program-project.ps1 -ProjectDir "projects\01_Port" -Programmer arduino -Port COM3
```

### Launch SimulIDE
```powershell
.\tools\simulide\cli-simulide.ps1
```

### Python Interface
```powershell
python .\tools\interfaces\python_interface.py
```

---

## 🔧 Integration with VS Code

All tools are integrated with VS Code tasks. Use:
- **Ctrl+Shift+B** - Build current project
- **Task: Program Current Project** - Upload to board
- **Task: Test System** - Run diagnostics

---

## 📞 Support

For educational use and questions:
- **Author:** Prof. Hong Jeaong
- **Institution:** IUT (Inha University in Tashkent)
- **Contact:** linkedin.com/in/gnoejh53
