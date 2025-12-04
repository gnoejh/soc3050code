# PowerShell Scripts Reference

Quick reference for the 6 essential development scripts:

## Build Scripts
- **`cli-build-project.ps1`** - Build current project and program to ATmega128
- **`cli-program-project.ps1`** - Program HEX files with automatic COM port detection
- **`cli-build-all.ps1`** - Build all 35 projects in sequence

## Development Tools  
- **`cli-test-system.ps1`** - Comprehensive testing and debugging
- **`cli-analyze-code.ps1`** - Code quality analysis and optimization
- **`cli-new-project.ps1`** - Create new project with template

## Usage Examples

```powershell
# Build and program current project
.\cli-build-project.ps1

# Program with automatic COM port detection
.\cli-program-project.ps1 -ProjectDir "projects\Graphics_Display" -Programmer arduino

# Build all projects
.\cli-build-all.ps1

# Interactive testing
.\cli-test-system.ps1 -Interactive

# Analyze code quality
.\cli-analyze-code.ps1 -Fix

# Create new UART project
.\cli-new-project.ps1 -ProjectName "MyUART" -ProjectType "uart"
```
---

© 2025 Prof. Hong Jeaong, IUT (Incheon National University)
All rights reserved for educational purposes.

Contact: https://linkedin.com/in/gnoejh53
