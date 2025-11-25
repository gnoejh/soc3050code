# Batch Files Organization

All batch files have been moved to `tools/batch/` for better organization.

## 🚀 Quick Access (Root Wrappers)

For convenience, use these wrappers in the root directory:

- **`build.bat`** - Quick build wrapper → `tools/batch/build-project.bat`
- **`setup.bat`** - Quick setup check → `tools/batch/setup-check.bat`

## 📁 All Batch Files Location

**Path:** `tools/batch/`

### Build & Compilation
- **`build-project.bat`** - Build a specific project
- **`build-all.bat`** - Build all projects in workspace

### Programming & Testing
- **`program-project.bat`** - Upload firmware to hardware
- **`simulate-project.bat`** - Launch SimulIDE simulation

### Setup & Verification
- **`setup-check.bat`** - Comprehensive environment verification
- **`simple-check.bat`** - Quick verification check

### Project Management
- **`new-project.bat`** - Create new project from template

### Dashboard
- **`start-classroom-dashboard.bat`** - Launch multi-user classroom dashboard

---

## 💡 Usage Examples

### From Root Directory (Using Wrappers)
```bash
# Quick build
build.bat

# Quick setup check
setup.bat
```

### From Root Directory (Direct)
```bash
# Build specific project
tools\batch\build-project.bat projects\Serial_Communications

# Upload to hardware
tools\batch\program-project.bat projects\Serial_Communications

# Launch simulator
tools\batch\simulate-project.bat projects\Port_Basic
```

### From Any Directory
```bash
# Use absolute path
W:\soc3050code\tools\batch\build-project.bat

# Or navigate first
cd W:\soc3050code\tools\batch
build-project.bat ..\..\projects\Serial_Communications
```

---

## 🔧 Recommended Workflow

**Most users should use VS Code tasks instead of batch files directly:**

1. Open project in VS Code
2. Press `Ctrl+Shift+P`
3. Type "Tasks: Run Task"
4. Select task (Build, Program, Simulate, etc.)

The batch files are primarily for:
- CI/CD automation
- Command-line enthusiasts
- Quick testing outside VS Code

---

**See also:** `docs/CLI_GUIDE.md` for complete command-line reference
