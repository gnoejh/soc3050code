# PowerShell Scripts Reference

Quick reference for the 5 essential development scripts:

## Build Scripts
- **`cli-build-project.ps1`** - Build current project and program to ATmega128
- **`cli-build-all.ps1`** - Build all 35 projects in sequence

## Development Tools  
- **`cli-test-system.ps1`** - Comprehensive testing and debugging
- **`cli-analyze-code.ps1`** - Code quality analysis and optimization
- **`cli-new-project.ps1`** - Create new project with template

## Usage Examples

```powershell
# Build and program current project
.\cli-build-project.ps1

# Build all projects
.\cli-build-all.ps1

# Interactive testing
.\cli-test-system.ps1 -Interactive

# Analyze code quality
.\cli-analyze-code.ps1 -Fix

# Create new UART project
.\cli-new-project.ps1 -ProjectName "MyUART" -ProjectType "uart"
```