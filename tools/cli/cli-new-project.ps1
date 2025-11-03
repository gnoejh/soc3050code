# ATmega128 Educational Framework - Project Template Generator
# Creates standardized project templates for consistent educational experience

param(
    [Parameter(Mandatory=$true)]
    [string]$ProjectName,
    
    [Parameter(Mandatory=$false)]
    [string]$ProjectType = "basic",
    
    [Parameter(Mandatory=$false)]
    [string]$Description = "Educational demonstration project",
    
    [Parameter(Mandatory=$false)]
    [switch]$Advanced = $false
)

function Write-Log {
    param([string]$Message, [string]$Level = "INFO")
    
    $color = switch ($Level) {
        "ERROR" { "Red" }
        "WARN"  { "Yellow" }
        "SUCCESS" { "Green" }
        "INFO"  { "Cyan" }
        default { "White" }
    }
    
    Write-Host "[$Level] $Message" -ForegroundColor $color
}

# Project template configurations
$projectTemplates = @{
    "basic" = @{
        Description = "Basic I/O and GPIO control"
        Includes = @("_port.h", "_init.h")
        Hardware = "LEDs on PORTB, switches on PORTC"
        Functions = @("led_control", "button_reading", "basic_patterns")
    }
    
    "uart" = @{
        Description = "Serial communication and UART interface"
        Includes = @("_uart.h", "_init.h")
        Hardware = "UART1 pins, terminal connection"
        Functions = @("uart_communication", "string_processing", "command_interface")
    }
    
    "adc" = @{
        Description = "Analog-to-Digital conversion and sensor reading"
        Includes = @("_adc.h", "_init.h", "_uart.h")
        Hardware = "Analog sensors on ADC channels"
        Functions = @("sensor_reading", "voltage_conversion", "data_logging")
    }
    
    "timer" = @{
        Description = "Timer operations and timing control"
        Includes = @("_timer2.h", "_interrupt.h", "_init.h")
        Hardware = "Timer-controlled LEDs or buzzers"
        Functions = @("timer_setup", "interrupt_handling", "timing_control")
    }
    
    "game" = @{
        Description = "Interactive game implementation"
        Includes = @("_uart.h", "_port.h", "_buzzer.h", "_init.h")
        Hardware = "LEDs, buzzer, buttons, UART interface"
        Functions = @("game_logic", "user_interface", "scoring_system")
    }
    
    "iot" = @{
        Description = "IoT device with data communication"
        Includes = @("_uart.h", "_adc.h", "_eeprom.h", "_init.h")
        Hardware = "Sensors, communication interface, data storage"
        Functions = @("data_collection", "communication_protocol", "data_persistence")
    }
}

function New-ProjectDirectory {
    param([string]$ProjectPath)
    
    if (Test-Path $ProjectPath) {
        Write-Log "Project directory already exists: $ProjectPath" "WARN"
        $response = Read-Host "Overwrite existing project? (y/N)"
        if ($response -ne "y" -and $response -ne "Y") {
            Write-Log "Project creation cancelled" "INFO"
            return $false
        }
        Remove-Item -Path $ProjectPath -Recurse -Force
    }
    
    New-Item -ItemType Directory -Path $ProjectPath -Force | Out-Null
    Write-Log "Created project directory: $ProjectPath" "SUCCESS"
    return $true
}

function New-MainFile {
    param([string]$ProjectPath, [hashtable]$Template)
    
    $mainContent = @"
/*
 * $ProjectName - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - $($Template.Description)
 * - Practice hardware interface programming
 * - Understand embedded systems design patterns
 * - Apply C programming concepts to microcontroller development
 *
 * HARDWARE SETUP:
 * $($Template.Hardware)
 * - UART for debugging and communication
 * - Power supply: 5V DC
 * - Crystal oscillator: 7.3728 MHz
 */

#include "config.h"

// Project-specific constants
#define PROJECT_VERSION_MAJOR 1
#define PROJECT_VERSION_MINOR 0

// Global variables for project state
typedef struct {
    uint8_t initialized;
    uint8_t current_state;
    uint16_t operation_count;
    uint8_t error_flags;
} project_state_t;

static project_state_t project_state = {0};

// Function prototypes
void init_project_system(void);
void run_project_main_loop(void);
void handle_project_commands(void);
void display_project_status(void);
void demonstrate_project_features(void);

/*
 * Initialize project-specific systems
 */
void init_project_system(void)
{
    puts_USART1("Initializing $ProjectName System...\r\n");
    
    // Initialize hardware components
    init_devices();
    Uart1_init();
"@

    # Add template-specific initializations
    foreach ($func in $Template.Functions) {
        $mainContent += "`r`n    // Initialize $func"
        $mainContent += "`r`n    // TODO: Add $func initialization code"
    }
    
    $mainContent += @"

    
    // Set initial state
    project_state.initialized = 1;
    project_state.current_state = 0;
    project_state.operation_count = 0;
    project_state.error_flags = 0;
    
    puts_USART1("$ProjectName System Ready!\r\n");
    puts_USART1("Educational demonstration: $($Template.Description)\r\n");
    puts_USART1("Commands: 'h'=help, 's'=status, 'd'=demo, 'r'=reset\r\n");
}

/*
 * Display current project status
 */
void display_project_status(void)
{
    char buffer[80];
    
    puts_USART1("\r\n=== $ProjectName STATUS ===\r\n");
    
    sprintf(buffer, "Project: $ProjectName v%u.%u\r\n", 
            PROJECT_VERSION_MAJOR, PROJECT_VERSION_MINOR);
    puts_USART1(buffer);
    
    sprintf(buffer, "Initialized: %s\r\n", 
            project_state.initialized ? "YES" : "NO");
    puts_USART1(buffer);
    
    sprintf(buffer, "Current State: %u\r\n", project_state.current_state);
    puts_USART1(buffer);
    
    sprintf(buffer, "Operations: %u\r\n", project_state.operation_count);
    puts_USART1(buffer);
    
    sprintf(buffer, "Error Flags: 0x%02X\r\n", project_state.error_flags);
    puts_USART1(buffer);
    
    sprintf(buffer, "Free RAM: %u bytes\r\n", get_free_ram());
    puts_USART1(buffer);
}

/*
 * Demonstrate project features
 */
void demonstrate_project_features(void)
{
    puts_USART1("\r\n=== $ProjectName DEMONSTRATION ===\r\n");
    puts_USART1("Educational features: $($Template.Description)\r\n");
    puts_USART1("Press any key to stop demonstration...\r\n");
    
    uint8_t demo_step = 0;
    
    while (!is_USART1_received()) {
        // Cycle through demonstration steps
        switch (demo_step % 4) {
            case 0:
                puts_USART1("Demo step 1: Basic operation\r\n");
                // TODO: Add demo step 1 implementation
                break;
                
            case 1:
                puts_USART1("Demo step 2: Advanced features\r\n");
                // TODO: Add demo step 2 implementation
                break;
                
            case 2:
                puts_USART1("Demo step 3: Error handling\r\n");
                // TODO: Add demo step 3 implementation
                break;
                
            case 3:
                puts_USART1("Demo step 4: Performance test\r\n");
                // TODO: Add demo step 4 implementation
                break;
        }
        
        demo_step++;
        project_state.operation_count++;
        
        _delay_ms(2000);
    }
    
    get_USART1(); // Clear received character
    puts_USART1("Demonstration complete\r\n");
}

/*
 * Handle user commands
 */
void handle_project_commands(void)
{
    if (is_USART1_received()) {
        char command = get_USART1();
        
        switch (command) {
            case 'h':
            case 'H':
            case '?':
                puts_USART1("\r\n=== $ProjectName HELP ===\r\n");
                puts_USART1("Available commands:\r\n");
                puts_USART1("h/? - Show this help\r\n");
                puts_USART1("s/S - Show system status\r\n");
                puts_USART1("d/D - Run demonstration\r\n");
                puts_USART1("r/R - Reset system\r\n");
                puts_USART1("Educational objective: $($Template.Description)\r\n");
                break;
                
            case 's':
            case 'S':
                display_project_status();
                break;
                
            case 'd':
            case 'D':
                demonstrate_project_features();
                break;
                
            case 'r':
            case 'R':
                puts_USART1("Resetting project state...\r\n");
                project_state.current_state = 0;
                project_state.operation_count = 0;
                project_state.error_flags = 0;
                puts_USART1("Reset complete\r\n");
                break;
                
            default:
                puts_USART1("Unknown command. Press 'h' for help.\r\n");
                break;
        }
        
        project_state.operation_count++;
    }
}

/*
 * Main project loop
 */
void run_project_main_loop(void)
{
    while (1) {
        // Handle user commands
        handle_project_commands();
        
        // Project-specific main loop operations
        // TODO: Add main loop implementation based on project type
        
        // Small delay to prevent overwhelming the system
        _delay_ms(50);
    }
}

/*
 * Utility function to estimate free RAM
 */
uint16_t get_free_ram(void)
{
    extern uint16_t __heap_start;
    extern void *__brkval;
    
    uint16_t v;
    return (uint16_t)&v - (__brkval == 0 ? (uint16_t)&__heap_start : (uint16_t)__brkval);
}

/*
 * Main function
 */
int main(void)
{
    // Banner
    puts_USART1("$ProjectName Starting...\r\n");
    puts_USART1("ATmega128 Educational Framework\r\n");
    puts_USART1("Project Type: $ProjectType\r\n");
    puts_USART1("Description: $Description\r\n");
    
    // Initialize project systems
    init_project_system();
    
    // Display initial status
    display_project_status();
    
    puts_USART1("\r\nPress 'h' for help\r\n");
    
    // Enter main loop
    run_project_main_loop();
    
    return 0; // Should never reach here
}
"@

    $mainFilePath = Join-Path $ProjectPath "Main.c"
    Set-Content -Path $mainFilePath -Value $mainContent
    Write-Log "Created Main.c with template structure" "SUCCESS"
}

function New-ConfigFile {
    param([string]$ProjectPath, [hashtable]$Template)
    
    $configContent = @"
/*
 * Configuration Header - $ProjectName
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Include shared library headers
"@

    foreach ($include in $Template.Includes) {
        $configContent += "`r`n#include `"$include`""
    }
    
    $configContent += @"


// Project-specific configurations
#define PROJECT_NAME "$ProjectName"
#define PROJECT_TYPE "$ProjectType"

// Hardware pin definitions
// TODO: Define specific pin assignments for $($Template.Hardware)

// Project-specific constants
// TODO: Add project-specific constants and macros

// Function prototypes
uint16_t get_free_ram(void);

#endif /* CONFIG_H_ */
"@

    $configFilePath = Join-Path $ProjectPath "config.h"
    Set-Content -Path $configFilePath -Value $configContent
    Write-Log "Created config.h with project includes" "SUCCESS"
}

function New-ProjectFile {
    param([string]$ProjectPath)
    
    # Generate unique GUID for the project
    $projectGuid = [System.Guid]::NewGuid().ToString().ToUpper()
    
    $projectContent = @"
<?xml version="1.0" encoding="utf-8"?>
<Project DefaultTargets="Build" xmlns="http://schemas.microsoft.com/developer/msbuild/2003" ToolsVersion="14.0">
  <PropertyGroup>
    <SchemaVersion>2.0</SchemaVersion>
    <ProjectVersion>7.0</ProjectVersion>
    <ToolchainName>com.Atmel.AVRGCC8.C</ToolchainName>
    <ProjectGuid>dce6c7e3-ee26-4d79-826b-08594b9ad897</ProjectGuid>
    <avrdevice>ATmega128</avrdevice>
    <avrdeviceseries>none</avrdeviceseries>
    <OutputType>Executable</OutputType>
    <Language>C</Language>
    <OutputFileName>`$(MSBuildProjectName)</OutputFileName>
    <OutputFileExtension>.elf</OutputFileExtension>
    <OutputDirectory>`$(MSBuildProjectDirectory)\`$(Configuration)</OutputDirectory>
    <AssemblyName>$ProjectName</AssemblyName>
    <Name>$ProjectName</Name>
    <RootNamespace>$ProjectName</RootNamespace>
    <ToolchainFlavour>Native</ToolchainFlavour>
    <KeepTimersRunning>true</KeepTimersRunning>
    <OverrideVtor>false</OverrideVtor>
    <CacheFlash>true</CacheFlash>
    <ProgFlashFromRam>true</ProgFlashFromRam>
    <RamSnippetAddress>0x20000000</RamSnippetAddress>
    <UncachedRange />
    <preserveEEPROM>true</preserveEEPROM>
    <OverrideVtorValue>exception_table</OverrideVtorValue>
    <BootSegment>2</BootSegment>
    <ResetRule>0</ResetRule>
    <eraseonlaunchrule>0</eraseonlaunchrule>
    <EraseKey />
    <AsfFrameworkConfig>
      <framework-data xmlns="">
        <options />
        <configurations />
        <files />
        <documentation help="" />
        <offline-documentation help="" />
        <dependencies>
          <content-extension eid="atmel.asf" uuidref="Atmel.ASF" version="3.52.0" />
        </dependencies>
      </framework-data>
    </AsfFrameworkConfig>
    <avrtool>com.atmel.avrdbg.tool.simulator</avrtool>
    <avrtoolserialnumber />
    <avrdeviceexpectedsignature>0x1E9702</avrdeviceexpectedsignature>
    <com_atmel_avrdbg_tool_simulator>
      <ToolOptions>
        <InterfaceProperties>
        </InterfaceProperties>
        <InterfaceName>
        </InterfaceName>
      </ToolOptions>
      <ToolType>com.atmel.avrdbg.tool.simulator</ToolType>
      <ToolNumber>
      </ToolNumber>
      <ToolName>Simulator</ToolName>
    </com_atmel_avrdbg_tool_simulator>
  </PropertyGroup>
  <PropertyGroup Condition=" '`$(Configuration)' == 'Release' ">
    <ToolchainSettings>
      <AvrGcc>
        <avrgcc.common.Device>-mmcu=atmega128 -B "%24(PackRepoDir)\atmel\ATmega_DFP\1.7.374\gcc\dev\atmega128"</avrgcc.common.Device>
        <avrgcc.common.outputfiles.hex>True</avrgcc.common.outputfiles.hex>
        <avrgcc.common.outputfiles.lss>True</avrgcc.common.outputfiles.lss>
        <avrgcc.common.outputfiles.eep>True</avrgcc.common.outputfiles.eep>
        <avrgcc.common.outputfiles.srec>True</avrgcc.common.outputfiles.srec>
        <avrgcc.common.outputfiles.usersignatures>False</avrgcc.common.outputfiles.usersignatures>
        <avrgcc.compiler.general.ChangeDefaultCharTypeUnsigned>True</avrgcc.compiler.general.ChangeDefaultCharTypeUnsigned>
        <avrgcc.compiler.general.ChangeDefaultBitFieldUnsigned>True</avrgcc.compiler.general.ChangeDefaultBitFieldUnsigned>
        <avrgcc.compiler.symbols.DefSymbols>
          <ListValues>
            <Value>NDEBUG</Value>
          </ListValues>
        </avrgcc.compiler.symbols.DefSymbols>
        <avrgcc.compiler.directories.IncludePaths>
          <ListValues>
            <Value>%24(PackRepoDir)\atmel\ATmega_DFP\1.7.374\include\</Value>
            <Value>..\..\shared_libs</Value>
            <Value>..\..\Main</Value>
          </ListValues>
        </avrgcc.compiler.directories.IncludePaths>
        <avrgcc.compiler.optimization.level>Optimize for size (-Os)</avrgcc.compiler.optimization.level>
        <avrgcc.compiler.optimization.PackStructureMembers>True</avrgcc.compiler.optimization.PackStructureMembers>
        <avrgcc.compiler.optimization.AllocateBytesNeededForEnum>True</avrgcc.compiler.optimization.AllocateBytesNeededForEnum>
        <avrgcc.compiler.warnings.AllWarnings>True</avrgcc.compiler.warnings.AllWarnings>
        <avrgcc.linker.libraries.Libraries>
          <ListValues>
            <Value>libm</Value>
          </ListValues>
        </avrgcc.linker.libraries.Libraries>
        <avrgcc.assembler.general.IncludePaths>
          <ListValues>
            <Value>%24(PackRepoDir)\atmel\ATmega_DFP\1.7.374\include\</Value>
          </ListValues>
        </avrgcc.assembler.general.IncludePaths>
      </AvrGcc>
    </ToolchainSettings>
  </PropertyGroup>
  <PropertyGroup Condition=" '`$(Configuration)' == 'Debug' ">
    <ToolchainSettings>
      <AvrGcc>
        <avrgcc.common.Device>-mmcu=atmega128 -B "%24(PackRepoDir)\atmel\ATmega_DFP\1.7.374\gcc\dev\atmega128"</avrgcc.common.Device>
        <avrgcc.common.outputfiles.hex>True</avrgcc.common.outputfiles.hex>
        <avrgcc.common.outputfiles.lss>True</avrgcc.common.outputfiles.lss>
        <avrgcc.common.outputfiles.eep>True</avrgcc.common.outputfiles.eep>
        <avrgcc.common.outputfiles.srec>True</avrgcc.common.outputfiles.srec>
        <avrgcc.common.outputfiles.usersignatures>False</avrgcc.common.outputfiles.usersignatures>
        <avrgcc.compiler.general.ChangeDefaultCharTypeUnsigned>True</avrgcc.compiler.general.ChangeDefaultCharTypeUnsigned>
        <avrgcc.compiler.general.ChangeDefaultBitFieldUnsigned>True</avrgcc.compiler.general.ChangeDefaultBitFieldUnsigned>
        <avrgcc.compiler.symbols.DefSymbols>
          <ListValues>
            <Value>DEBUG</Value>
          </ListValues>
        </avrgcc.compiler.symbols.DefSymbols>
        <avrgcc.compiler.directories.IncludePaths>
          <ListValues>
            <Value>%24(PackRepoDir)\atmel\ATmega_DFP\1.7.374\include\</Value>
            <Value>..\..\shared_libs</Value>
            <Value>..\..\Main</Value>
          </ListValues>
        </avrgcc.compiler.directories.IncludePaths>
        <avrgcc.compiler.optimization.level>Optimize debugging experience (-Og)</avrgcc.compiler.optimization.level>
        <avrgcc.compiler.optimization.PackStructureMembers>True</avrgcc.compiler.optimization.PackStructureMembers>
        <avrgcc.compiler.optimization.AllocateBytesNeededForEnum>True</avrgcc.compiler.optimization.AllocateBytesNeededForEnum>
        <avrgcc.compiler.optimization.DebugLevel>Default (-g2)</avrgcc.compiler.optimization.DebugLevel>
        <avrgcc.compiler.warnings.AllWarnings>True</avrgcc.compiler.warnings.AllWarnings>
        <avrgcc.linker.libraries.Libraries>
          <ListValues>
            <Value>libm</Value>
          </ListValues>
        </avrgcc.linker.libraries.Libraries>
        <avrgcc.assembler.general.IncludePaths>
          <ListValues>
            <Value>%24(PackRepoDir)\atmel\ATmega_DFP\1.7.374\include\</Value>
          </ListValues>
        </avrgcc.assembler.general.IncludePaths>
        <avrgcc.assembler.debugging.DebugLevel>Default (-Wa,-g)</avrgcc.assembler.debugging.DebugLevel>
      </AvrGcc>
    </ToolchainSettings>
  </PropertyGroup>
  <ItemGroup>
    <Compile Include="config.h">
      <SubType>compile</SubType>
    </Compile>
    <Compile Include="Main.c">
      <SubType>compile</SubType>
    </Compile>
  </ItemGroup>
</Project>
"@

    $projectFilePath = Join-Path $ProjectPath "$ProjectName.cproj"
    Set-Content -Path $projectFilePath -Value $projectContent
    Write-Log "Created $ProjectName.cproj with proper Microchip Studio configuration" "SUCCESS"
}

function New-ReadmeFile {
    param([string]$ProjectPath, [hashtable]$Template)
    
    $readmeContent = @"
# $ProjectName

## Project Overview
$Description

**Project Type:** $ProjectType  
**Educational Focus:** $($Template.Description)

## Learning Objectives
- $($Template.Description)
- Understand embedded C programming concepts
- Practice hardware interface development
- Learn debugging and testing techniques

## Hardware Requirements
$($Template.Hardware)

### Pin Connections
```
// TODO: Add specific pin connection details
// Example:
// LED1 → PORTB.0
// LED2 → PORTB.1
// Button1 → PORTC.0 (with pull-up)
// UART1 TX → Pin 3 (for debugging)
// UART1 RX → Pin 2 (for debugging)
```

## Software Features
"@

    foreach ($func in $Template.Functions) {
        $readmeContent += "`r`n- $func implementation"
    }
    
    $readmeContent += @"


## Build Instructions
1. Open `$ProjectName.cproj` in Microchip Studio
2. Select build configuration (Debug/Release)
3. Build the project (F7)
4. Program to ATmega128 (Ctrl+Alt+F5)

## Usage Instructions
1. Connect hardware as specified above
2. Open terminal at 9600 baud, 8N1
3. Reset the microcontroller
4. Follow on-screen prompts and commands:
   - `h` - Show help and available commands
   - `s` - Show system status
   - `d` - Run demonstration
   - `r` - Reset system state

## Educational Notes
This project demonstrates:
- $($Template.Description)
- Structured C programming for embedded systems
- Hardware abstraction layer usage
- Interactive debugging via UART
- Error handling and system state management

## Troubleshooting
### Common Issues
1. **No UART output:** Check baud rate (9600) and connection
2. **Build errors:** Verify include paths and shared library links
3. **Hardware not responding:** Check power supply and pin connections
4. **Unexpected behavior:** Use UART debug commands to check system state

### Debug Commands
- Use `s` command to check system status
- Monitor operation count and error flags
- Reset system with `r` command if needed

## Further Learning
After mastering this project, consider:
- Modifying timing parameters and observing effects
- Adding additional hardware interfaces
- Implementing advanced features based on project type
- Exploring interrupt-driven vs. polling approaches

## Project Structure
```
$ProjectName/
├── Main.c          # Main implementation
├── config.h        # Configuration and includes
├── $ProjectName.cproj  # Microchip Studio project
└── README.md       # This documentation
```

---
**ATmega128 Educational Framework**  
*Building embedded systems knowledge step by step*
"@

    $readmeFilePath = Join-Path $ProjectPath "README.md"
    Set-Content -Path $readmeFilePath -Value $readmeContent
    Write-Log "Created README.md with comprehensive documentation" "SUCCESS"
}

# Main execution
function Main {
    Write-Log "ATmega128 Educational Framework - Project Template Generator" "INFO"
    Write-Log "Project: $ProjectName, Type: $ProjectType" "INFO"
    
    # Validate project type
    if (-not $projectTemplates.ContainsKey($ProjectType)) {
        Write-Log "Invalid project type: $ProjectType" "ERROR"
        Write-Log "Available types: $($projectTemplates.Keys -join ', ')" "INFO"
        exit 1
    }
    
    $template = $projectTemplates[$ProjectType]
    Write-Log "Using template: $($template.Description)" "INFO"
    
    # Create project directory
    $projectsRoot = Join-Path $PSScriptRoot "projects"
    $projectPath = Join-Path $projectsRoot $ProjectName
    
    if (-not (New-ProjectDirectory $projectPath)) {
        exit 1
    }
    
    # Generate project files
    try {
        New-MainFile $projectPath $template
        New-ConfigFile $projectPath $template
        New-ProjectFile $projectPath
        New-ReadmeFile $projectPath $template
        
        Write-Log "Project template created successfully!" "SUCCESS"
        Write-Log "Location: $projectPath" "INFO"
        Write-Log "Next steps:" "INFO"
        Write-Log "1. Open $ProjectName.cproj in Microchip Studio" "INFO"
        Write-Log "2. Implement TODO sections in Main.c" "INFO"
        Write-Log "3. Configure hardware connections as documented" "INFO"
        Write-Log "4. Build and test the project" "INFO"
        
    } catch {
        Write-Log "Error creating project: $($_.Exception.Message)" "ERROR"
        exit 1
    }
}

# Execute main function
Main