#!/usr/bin/env python3
"""
GPIO Parallel Communication Reader for SimulIDE 1.1.0
Workaround for UART TX bug in ATmega128

Reads Logic Analyzer CSV export from SimulIDE and decodes parallel GPIO communication.

Author: Professor Hong Jeong
Course: SOC 3050 - Embedded Systems
Year: 2025

Usage:
    python read-gpio-communication.py [csv_file]
    
    If no file specified, uses default: gpio_data.csv
"""

import csv
import time
import sys
from pathlib import Path
from datetime import datetime
from typing import List, Tuple

class GPIOCommunicationReader:
    """
    Reads and decodes GPIO parallel communication from SimulIDE Logic Analyzer
    
    Protocol:
    - 8-bit parallel data on PORTB (Logic Analyzer pins 0-7)
    - Each byte held for 20ms
    - Inter-byte gap: 20ms (0x00)
    - End of message: 0xFF
    """
    
    def __init__(self, csv_file: str):
        self.csv_file = Path(csv_file)
        self.last_pos = 0
        self.last_byte = 0
        self.current_message = ""
        self.message_count = 0
        
    def read_byte_from_pins(self, pin_voltages: List[float]) -> int:
        """
        Convert 8 pin voltages to byte value
        
        Args:
            pin_voltages: List of 8 voltage values (one per bit)
            
        Returns:
            Byte value (0-255)
        """
        byte_value = 0
        threshold = 2.5  # Logic high/low threshold
        
        for bit in range(8):
            if bit < len(pin_voltages) and pin_voltages[bit] > threshold:
                byte_value |= (1 << bit)
        
        return byte_value
    
    def decode_stream(self) -> List[str]:
        """
        Read and decode new data from CSV file
        
        Returns:
            List of complete messages received
        """
        messages = []
        
        if not self.csv_file.exists():
            return messages
        
        try:
            with open(self.csv_file, 'r', encoding='utf-8') as f:
                # Seek to last read position
                f.seek(self.last_pos)
                
                # Read new lines
                reader = csv.DictReader(f)
                
                for row in reader:
                    try:
                        # Read 8 pins (PORTB bits)
                        pin_voltages = []
                        for i in range(8):
                            voltage_str = row.get(f'Pin{i}', '0')
                            # Handle different CSV formats
                            voltage_str = voltage_str.replace('V', '').strip()
                            voltage = float(voltage_str) if voltage_str else 0.0
                            pin_voltages.append(voltage)
                        
                        byte_value = self.read_byte_from_pins(pin_voltages)
                        
                        # Detect rising edge (new data)
                        if byte_value != 0 and self.last_byte == 0:
                            if byte_value == 0xFF:  # End of message marker
                                if self.current_message:
                                    messages.append(self.current_message)
                                    self.message_count += 1
                                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                    print(f"[{timestamp}] 📨 Message #{self.message_count}: {self.current_message}")
                                    self.current_message = ""
                            else:
                                # Add character to current message
                                if 32 <= byte_value <= 126:  # Printable ASCII
                                    char = chr(byte_value)
                                    self.current_message += char
                                    # Show byte in real-time
                                    print(f"  └─ Byte: 0x{byte_value:02X} ('{char}')")
                                else:
                                    # Non-printable character
                                    print(f"  └─ Byte: 0x{byte_value:02X} (non-printable)")
                        
                        self.last_byte = byte_value
                        
                    except (ValueError, KeyError) as e:
                        # Skip malformed rows
                        continue
                
                # Save position for next read
                self.last_pos = f.tell()
                
        except Exception as e:
            print(f"❌ Error reading CSV: {e}")
        
        return messages
    
    def wait_for_file(self, timeout: int = 30):
        """Wait for CSV file to be created by SimulIDE"""
        print(f"⏳ Waiting for {self.csv_file.name}...")
        start_time = time.time()
        
        while not self.csv_file.exists():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                print(f"❌ Timeout: File not created after {timeout}s")
                print(f"")
                print(f"🔧 Troubleshooting:")
                print(f"   1. Open SimulIDE 1.1.0")
                print(f"   2. Double-click Logic Analyzer component")
                print(f"   3. Enable 'Auto Export' checkbox")
                print(f"   4. Set export file: {self.csv_file.absolute()}")
                print(f"   5. Click OK and run simulation")
                return False
            
            time.sleep(0.5)
            print(".", end="", flush=True)
        
        print(f"\n✅ File found!")
        return True

def print_banner():
    """Print startup banner"""
    print("=" * 70)
    print("  GPIO PARALLEL COMMUNICATION READER")
    print("  SimulIDE 1.1.0 UART TX Workaround")
    print("=" * 70)
    print("")
    print("📡 Protocol: 8-bit parallel via PORTB (Logic Analyzer)")
    print("🎯 Purpose: Bypass broken UART TX in SimulIDE 1.1.0")
    print("🐍 Python-C Integration: Real-time message decoding")
    print("")

def print_statistics(reader: GPIOCommunicationReader):
    """Print session statistics"""
    print("")
    print("=" * 70)
    print("📊 SESSION STATISTICS")
    print("=" * 70)
    print(f"Total messages received: {reader.message_count}")
    print(f"CSV file: {reader.csv_file}")
    print(f"Bytes read: {reader.last_pos}")
    print("")

def main():
    """Main program"""
    print_banner()
    
    # Get CSV file path
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        # Default location (same directory as this script)
        script_dir = Path(__file__).parent
        csv_file = script_dir / 'gpio_data.csv'
        
        # Alternative: Check project directory
        if not csv_file.exists():
            project_dir = Path('W:/soc3050code/projects/Serial_Communications')
            csv_file = project_dir / 'gpio_data.csv'
    
    print(f"📂 CSV File: {csv_file}")
    print("")
    
    # Create reader
    reader = GPIOCommunicationReader(csv_file)
    
    # Wait for file if it doesn't exist yet
    if not reader.csv_file.exists():
        if not reader.wait_for_file():
            return 1
    
    print("🚀 Starting real-time monitoring...")
    print("   (Press Ctrl+C to stop)")
    print("")
    
    # Main monitoring loop
    try:
        while True:
            messages = reader.decode_stream()
            
            # Could process messages here with AI/ML
            # for msg in messages:
            #     ai_result = your_ai_model.predict(msg)
            #     print(f"🤖 AI Analysis: {ai_result}")
            
            time.sleep(0.1)  # 100Hz sampling rate
            
    except KeyboardInterrupt:
        print("\n\n⏹️  Stopped by user")
        print_statistics(reader)
        return 0
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())

"""
=============================================================================
USAGE EXAMPLES:
=============================================================================

1. Basic Usage (default CSV file):
   python read-gpio-communication.py

2. Custom CSV file:
   python read-gpio-communication.py path/to/logic_export.csv

3. With AI Integration:
   ```python
   reader = GPIOCommunicationReader('gpio_data.csv')
   while True:
       messages = reader.decode_stream()
       for msg in messages:
           # Process with AI
           ai_result = your_model.predict(msg)
           print(f"AI: {ai_result}")
   ```

4. Monitor in VS Code:
   - Open integrated terminal
   - Run: python read-gpio-communication.py
   - See messages in real-time!

=============================================================================
SIMULIDE CONFIGURATION:
=============================================================================

Required Steps:
1. Open Simulator110.simu in SimulIDE 1.1.0
2. Logic Analyzer (LAnalizer-891) should be connected to PORTB (already done)
3. Double-click Logic Analyzer component
4. Settings:
   - ✅ Auto Export: ON
   - Export File: W:/soc3050code/projects/Serial_Communications/gpio_data.csv
   - Trigger: On change (or None for continuous)
5. Click OK
6. Run simulation
7. Python script will show messages!

=============================================================================
TROUBLESHOOTING:
=============================================================================

Problem: "Waiting for gpio_data.csv..." never finishes
Solution: Check Logic Analyzer settings (Auto Export enabled)

Problem: "No messages received"
Solution: Check that Main_Parallel.c is loaded in ATmega128

Problem: "Garbled messages"
Solution: Check sampling rate (default 100Hz should work)

Problem: CSV file empty
Solution: Run simulation in SimulIDE (press Play button)

=============================================================================
"""
