"""
Serial Communications - Laboratory Exercises
Hands-on exercises for students to practice Python-ATmega128 communication

PROJECT: Serial_Communications (Python Lab)
COURSE: SOC 3050 - Embedded Systems and Applications
YEAR: 2025
AUTHOR: Professor Hong Jeong

LAB OBJECTIVES:
1. Practice serial communication protocols
2. Implement command parsing
3. Create interactive applications
4. Debug communication issues

EXERCISES:
- Exercise 1: Echo Server Test
- Exercise 2: LED Pattern Controller
- Exercise 3: Multi-Sensor Dashboard
- Exercise 4: Command-Line Interface
- Exercise 5: Data Analysis Tool

© 2025 Prof. Hong Jeong, IUT
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'python_libs'))

from atmega128 import ATmega128Serial, ATmega128Protocol
from visualization import DataLogger
import time
from config import *


def exercise_1_echo_test():
    """
    Exercise 1: Echo Server Test
    
    TASK:
    Send different messages to ATmega128 and verify echo responses.
    Test with special characters, numbers, and long strings.
    
    LEARNING GOALS:
    - Understand request-response pattern
    - Handle different data types
    - Implement error checking
    """
    print("\n" + "=" * 70)
    print("EXERCISE 1: Echo Server Test")
    print("=" * 70)
    print("Test echo functionality with various inputs")
    print()
    
    # TODO: Student implementation
    # 1. Connect to ATmega128
    # 2. Send test messages
    # 3. Verify responses match input
    # 4. Handle errors gracefully
    
    test_messages = [
        "Hello",
        "123",
        "Special!@#$",
        "Long message with spaces and numbers 12345",
    ]
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        passed = 0
        failed = 0
        
        for msg in test_messages:
            print(f"Testing: '{msg}'")
            
            # Send ECHO command
            mcu.send_command(f'ECHO {msg}')
            response = mcu.wait_for_response(timeout=2.0)
            
            # Verify response
            if response and msg in response:
                print(f"  ✅ PASS: Received '{response}'")
                passed += 1
            else:
                print(f"  ❌ FAIL: Expected '{msg}', got '{response}'")
                failed += 1
        
        print(f"\nResults: {passed} passed, {failed} failed")


def exercise_2_led_pattern_controller():
    """
    Exercise 2: LED Pattern Controller
    
    TASK:
    Create custom LED patterns using Python control.
    Implement at least 3 different patterns.
    
    LEARNING GOALS:
    - Timing control with time.sleep()
    - Sequential command sending
    - Pattern design
    
    PATTERNS TO IMPLEMENT:
    - Binary counter (show numbers 0-255)
    - Alternating LEDs
    - Random blinking
    """
    print("\n" + "=" * 70)
    print("EXERCISE 2: LED Pattern Controller")
    print("=" * 70)
    print("Create custom LED patterns")
    print()
    
    # TODO: Student implementation
    # Implement the patterns described above
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        # Pattern 1: Binary Counter
        print("Pattern 1: Binary Counter (0-15)")
        for num in range(16):
            # Send pattern as binary
            pattern = format(num, '08b')
            print(f"  Count: {num:2d} - Binary: {pattern}")
            
            # Set LEDs according to binary representation
            for i in range(8):
                if pattern[7-i] == '1':
                    mcu.send_command(f'LED_ON {i}')
                else:
                    mcu.send_command(f'LED_OFF {i}')
            
            time.sleep(0.5)
        
        # Turn all LEDs off
        for i in range(8):
            mcu.send_command(f'LED_OFF {i}')
        
        print("\n✅ Pattern complete")


def exercise_3_multi_sensor_dashboard():
    """
    Exercise 3: Multi-Sensor Dashboard
    
    TASK:
    Read multiple sensors and display in formatted dashboard.
    Update display in real-time.
    
    LEARNING GOALS:
    - Multi-channel data acquisition
    - Formatted output
    - Real-time display updates
    """
    print("\n" + "=" * 70)
    print("EXERCISE 3: Multi-Sensor Dashboard")
    print("=" * 70)
    print("Real-time sensor monitoring dashboard")
    print("Press Ctrl+C to stop")
    print()
    
    # TODO: Student implementation
    # Read multiple ADC channels and display in dashboard format
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        
        try:
            sample_count = 0
            while True:
                sample_count += 1
                
                # Read multiple channels (example: channels 0, 1, 2)
                sensors = {}
                for ch in range(3):
                    mcu.send_command(f'READ_ADC {ch}')
                    response = mcu.wait_for_response(timeout=1.0)
                    
                    if response:
                        value = protocol.parse_adc_value(response)
                        sensors[f'CH{ch}'] = value if value is not None else 0
                
                # Display dashboard
                print(f"\r[Sample: {sample_count:4d}] " + 
                      " | ".join([f"{k}:{v:4d}" for k, v in sensors.items()]), 
                      end='', flush=True)
                
                time.sleep(0.2)
        
        except KeyboardInterrupt:
            print("\n\n✅ Dashboard stopped")


def exercise_4_command_interface():
    """
    Exercise 4: Command-Line Interface
    
    TASK:
    Create a simple command-line interface with help system.
    Support multiple commands with parameters.
    
    LEARNING GOALS:
    - Command parsing
    - User input handling
    - Help system design
    """
    print("\n" + "=" * 70)
    print("EXERCISE 4: Command-Line Interface")
    print("=" * 70)
    print()
    
    commands = {
        'help': 'Show this help message',
        'led <n> on': 'Turn LED n on',
        'led <n> off': 'Turn LED n off',
        'adc <ch>': 'Read ADC channel',
        'status': 'Get system status',
        'quit': 'Exit program'
    }
    
    def show_help():
        print("\nAvailable commands:")
        for cmd, desc in commands.items():
            print(f"  {cmd:<15} - {desc}")
    
    show_help()
    print()
    
    # TODO: Student implementation
    # Implement command parser and executor
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        
        while True:
            try:
                user_input = input(">>> ").strip().lower()
                
                if not user_input:
                    continue
                
                parts = user_input.split()
                cmd = parts[0]
                
                if cmd == 'quit':
                    break
                
                elif cmd == 'help':
                    show_help()
                
                elif cmd == 'led' and len(parts) >= 3:
                    led_num = parts[1]
                    state = parts[2]
                    if state == 'on':
                        mcu.send_command(f'LED_ON {led_num}')
                        print(f"✅ LED {led_num} turned ON")
                    elif state == 'off':
                        mcu.send_command(f'LED_OFF {led_num}')
                        print(f"✅ LED {led_num} turned OFF")
                
                elif cmd == 'adc' and len(parts) >= 2:
                    channel = parts[1]
                    mcu.send_command(f'READ_ADC {channel}')
                    response = mcu.wait_for_response(timeout=1.0)
                    if response:
                        value = protocol.parse_adc_value(response)
                        print(f"📊 ADC Channel {channel}: {value}")
                
                elif cmd == 'status':
                    mcu.send_command('STATUS')
                    response = mcu.wait_for_response(timeout=1.0)
                    print(f"📋 Status: {response}")
                
                else:
                    print("❌ Unknown command. Type 'help' for available commands.")
            
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ Error: {e}")
    
    print("\n✅ Interface closed")


def exercise_5_data_analysis():
    """
    Exercise 5: Data Analysis Tool
    
    TASK:
    Collect sensor data and perform statistical analysis.
    Calculate min, max, average, and standard deviation.
    
    LEARNING GOALS:
    - Data collection
    - Statistical analysis
    - Result presentation
    """
    print("\n" + "=" * 70)
    print("EXERCISE 5: Data Analysis Tool")
    print("=" * 70)
    print("Collect and analyze sensor data")
    print()
    
    # TODO: Student implementation
    # Collect data and perform analysis
    
    from analysis import StatisticsAnalyzer
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        
        # Collect data
        num_samples = 50
        data = []
        
        print(f"Collecting {num_samples} samples...")
        for i in range(num_samples):
            mcu.send_command('READ_ADC 0')
            response = mcu.wait_for_response(timeout=1.0)
            
            if response:
                value = protocol.parse_adc_value(response)
                if value is not None:
                    data.append(value)
                    print(f"\rProgress: {len(data)}/{num_samples}", end='', flush=True)
            
            time.sleep(0.1)
        
        print(f"\n\n✅ Collected {len(data)} samples\n")
        
        # Analyze data
        if data:
            analyzer = StatisticsAnalyzer()
            stats = analyzer.calculate_stats(data)
            
            print("Statistical Analysis:")
            print("=" * 40)
            print(f"  Samples:     {stats['count']}")
            print(f"  Mean:        {stats['mean']:.2f}")
            print(f"  Median:      {stats['median']:.2f}")
            print(f"  Std Dev:     {stats['std']:.2f}")
            print(f"  Min:         {stats['min']:.2f}")
            print(f"  Max:         {stats['max']:.2f}")
            print(f"  Range:       {stats['range']:.2f}")
            
            # Detect outliers
            outliers = analyzer.detect_outliers(data)
            if outliers:
                print(f"\n  Outliers:    {len(outliers)} detected")
                print(f"  Indices:     {outliers[:10]}" + ("..." if len(outliers) > 10 else ""))


def lab_menu():
    """Laboratory exercises menu"""
    while True:
        print("\n" + "=" * 70)
        print("SERIAL COMMUNICATIONS - LABORATORY EXERCISES")
        print("=" * 70)
        print("Select exercise:")
        print()
        print("  1. Echo Server Test")
        print("  2. LED Pattern Controller")
        print("  3. Multi-Sensor Dashboard")
        print("  4. Command-Line Interface")
        print("  5. Data Analysis Tool")
        print("  0. Exit")
        print()
        
        choice = input("Enter choice (0-5): ").strip()
        
        if choice == '1':
            exercise_1_echo_test()
        elif choice == '2':
            exercise_2_led_pattern_controller()
        elif choice == '3':
            exercise_3_multi_sensor_dashboard()
        elif choice == '4':
            exercise_4_command_interface()
        elif choice == '5':
            exercise_5_data_analysis()
        elif choice == '0':
            print("\n👋 Lab session ended!")
            break
        else:
            print("❌ Invalid choice")


if __name__ == '__main__':
    print("=" * 70)
    print("SOC 3050 - Laboratory Exercises")
    print("© 2025 Prof. Hong Jeong, IUT")
    print("=" * 70)
    
    try:
        lab_menu()
    except KeyboardInterrupt:
        print("\n\n👋 Lab interrupted")

