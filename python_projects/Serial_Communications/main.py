"""
Serial Communications - Python Host Interface
Communicates with ATmega128 Serial_Communications C project

PROJECT: Serial_Communications (Python)
COURSE: SOC 3050 - Embedded Systems and Applications  
YEAR: 2025
AUTHOR: Professor Hong Jeong

PURPOSE:
Demonstrates Python-ATmega128 serial communication patterns.
Mirrors the C project functionality from the host PC side.

FEATURES:
- Interactive serial terminal
- Command sending and response handling
- LED control from Python
- Sensor data reading
- Data logging

PAIRED C PROJECT:
projects/Serial_Communications/Main.c

© 2025 Prof. Hong Jeong, IUT (Inha University in Tashkent)
"""

import sys
import os

# Add python_libs to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'python_libs'))

from atmega128 import ATmega128Serial, ATmega128Protocol
import time
from config import *


def demo_1_simple_communication():
    """
    Demo 1: Basic Communication
    Send commands and receive responses
    """
    print("\n" + "=" * 70)
    print("DEMO 1: Simple Communication Test")
    print("=" * 70)
    print("Testing basic command-response pattern")
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        # Test PING
        print("📤 Sending: PING")
        mcu.send_command('PING')
        response = mcu.wait_for_response(timeout=2.0)
        
        if response:
            print(f"📥 Received: {response}")
        else:
            print("❌ No response (ATmega128 may not be running serial program)")
        
        # Test ECHO
        print("\n📤 Sending: ECHO Hello from Python!")
        mcu.send_command('ECHO Hello from Python!')
        response = mcu.wait_for_response(timeout=2.0)
        
        if response:
            print(f"📥 Received: {response}")


def demo_2_led_control():
    """
    Demo 2: LED Control from Python
    Control ATmega128 LEDs remotely
    """
    print("\n" + "=" * 70)
    print("DEMO 2: LED Control")
    print("=" * 70)
    print("Controlling ATmega128 LEDs from Python")
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        # Knight Rider pattern
        print("Running Knight Rider pattern...")
        for _ in range(2):  # 2 cycles
            # Forward
            for i in range(8):
                mcu.send_command(f'LED_ON {i}')
                time.sleep(0.1)
                mcu.send_command(f'LED_OFF {i}')
            
            # Backward
            for i in range(6, 0, -1):
                mcu.send_command(f'LED_ON {i}')
                time.sleep(0.1)
                mcu.send_command(f'LED_OFF {i}')
        
        print("✅ Pattern complete")


def demo_3_sensor_reading():
    """
    Demo 3: Sensor Data Reading
    Read sensor values from ATmega128
    """
    print("\n" + "=" * 70)
    print("DEMO 3: Sensor Data Reading")
    print("=" * 70)
    print("Reading sensor data from ATmega128")
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        
        print("Reading 10 samples...")
        print(f"{'Sample':<10} {'ADC Value':<15} {'Voltage (V)':<15}")
        print("-" * 40)
        
        for i in range(10):
            # Read ADC channel 0
            mcu.send_command('READ_ADC 0')
            response = mcu.wait_for_response(timeout=1.0)
            
            if response:
                # Parse ADC value
                adc_value = protocol.parse_adc_value(response)
                if adc_value is not None:
                    # Convert to voltage (assuming 5V reference, 10-bit ADC)
                    voltage = (adc_value / 1023.0) * 5.0
                    print(f"{i+1:<10} {adc_value:<15} {voltage:<15.3f}")
                else:
                    print(f"{i+1:<10} {'Error':<15} {'-':<15}")
            else:
                print(f"{i+1:<10} {'No response':<15} {'-':<15}")
            
            time.sleep(0.5)


def demo_4_continuous_monitoring():
    """
    Demo 4: Continuous Data Monitoring
    Continuously monitor and display sensor data
    """
    print("\n" + "=" * 70)
    print("DEMO 4: Continuous Monitoring")
    print("=" * 70)
    print("Monitoring sensor data in real-time")
    print("Press Ctrl+C to stop")
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        
        try:
            while True:
                # Request sensor data
                mcu.send_command('READ_ADC 0')
                response = mcu.wait_for_response(timeout=1.0)
                
                if response:
                    adc_value = protocol.parse_adc_value(response)
                    if adc_value is not None:
                        # Create visual bar graph
                        bar_length = int(adc_value / 1023 * 50)
                        bar = '█' * bar_length + '░' * (50 - bar_length)
                        print(f"\rADC: {adc_value:4d} [{bar}] {adc_value/1023*100:5.1f}%", end='')
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n\n✅ Monitoring stopped")


def demo_5_data_logging():
    """
    Demo 5: Data Logging
    Log sensor data to file
    """
    print("\n" + "=" * 70)
    print("DEMO 5: Data Logging")
    print("=" * 70)
    print("Logging sensor data to file")
    print()
    
    # Import data logger
    from visualization import DataLogger
    
    # Create log directory
    os.makedirs(LOG_DIRECTORY, exist_ok=True)
    
    # Create timestamp for filename
    from datetime import datetime
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(LOG_DIRECTORY, f"sensor_data_{timestamp}")
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        with DataLogger(log_file, format=LOG_FORMAT, 
                       columns=['sample', 'adc_value', 'voltage']) as logger:
            
            protocol = ATmega128Protocol()
            
            print("Logging 20 samples...")
            for i in range(20):
                mcu.send_command('READ_ADC 0')
                response = mcu.wait_for_response(timeout=1.0)
                
                if response:
                    adc_value = protocol.parse_adc_value(response)
                    if adc_value is not None:
                        voltage = (adc_value / 1023.0) * 5.0
                        
                        # Log data
                        logger.log({
                            'sample': i + 1,
                            'adc_value': adc_value,
                            'voltage': voltage
                        })
                        
                        print(f"Sample {i+1:2d}: ADC={adc_value:4d}, V={voltage:.3f}")
                
                time.sleep(0.2)
            
            print(f"\n✅ Data logged to {logger.filename}")


def interactive_terminal():
    """
    Interactive Serial Terminal
    Send custom commands and see responses
    """
    print("\n" + "=" * 70)
    print("INTERACTIVE SERIAL TERMINAL")
    print("=" * 70)
    print("Type commands to send to ATmega128")
    print("Commands: PING, ECHO <text>, LED_ON <n>, LED_OFF <n>, READ_ADC <ch>")
    print("Type 'quit' or 'exit' to stop")
    print("=" * 70)
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        while True:
            try:
                # Get user input
                command = input(">>> ").strip()
                
                if command.lower() in ['quit', 'exit']:
                    break
                
                if not command:
                    continue
                
                # Send command
                mcu.send_command(command)
                
                # Wait for response
                response = mcu.wait_for_response(timeout=2.0)
                if response:
                    print(f"<<< {response}")
                else:
                    print("<<< (no response)")
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    print("\n✅ Terminal closed")


def main_menu():
    """Main menu for demo selection"""
    while True:
        print("\n" + "=" * 70)
        print("SERIAL COMMUNICATIONS - PYTHON INTERFACE")
        print("=" * 70)
        print("Select demonstration:")
        print()
        print("  1. Simple Communication Test")
        print("  2. LED Control")
        print("  3. Sensor Reading")
        print("  4. Continuous Monitoring")
        print("  5. Data Logging")
        print("  6. Interactive Terminal")
        print("  0. Exit")
        print()
        
        choice = input("Enter choice (0-6): ").strip()
        
        if choice == '1':
            demo_1_simple_communication()
        elif choice == '2':
            demo_2_led_control()
        elif choice == '3':
            demo_3_sensor_reading()
        elif choice == '4':
            demo_4_continuous_monitoring()
        elif choice == '5':
            demo_5_data_logging()
        elif choice == '6':
            interactive_terminal()
        elif choice == '0':
            print("\n👋 Goodbye!")
            break
        else:
            print("❌ Invalid choice")


if __name__ == '__main__':
    print("=" * 70)
    print("SOC 3050 - Serial Communications Python Interface")
    print("© 2025 Prof. Hong Jeong, IUT")
    print("=" * 70)
    
    try:
        main_menu()
    except KeyboardInterrupt:
        print("\n\n👋 Program interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

