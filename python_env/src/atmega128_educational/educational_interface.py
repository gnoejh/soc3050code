#!/usr/bin/env python3
"""
ATmega128 Educational Python Interface
Connects to the running C educational demo via serial communication
"""

import serial
import time
import sys
from datetime import datetime

class ATmega128Educational:
    def __init__(self, port='COM3', baud=9600):
        """Initialize connection to ATmega128 running educational demo"""
        try:
            self.ser = serial.Serial(port, baud, timeout=2)
            time.sleep(2)  # Wait for Arduino reset
            print(f"✅ Connected to ATmega128 on {port}")
        except serial.SerialException as e:
            print(f"❌ Failed to connect to {port}: {e}")
            sys.exit(1)
    
    def send_command(self, command):
        """Send command to ATmega128 and return response"""
        if isinstance(command, str):
            command = command.encode()
        
        self.ser.write(command)
        self.ser.flush()
        
        # Read response (may be multiple lines)
        response = ""
        start_time = time.time()
        
        while time.time() - start_time < 3:  # 3 second timeout
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                response += line
                print(f"📥 {line.strip()}")
                
                # Check if we got a complete response
                if "choice:" in line.lower() or "continue" in line.lower():
                    break
            time.sleep(0.1)
        
        return response
    
    def demonstrate_phase1_registers(self):
        """Phase 1: Assembly/Register Demonstration"""
        print("\n" + "="*50)
        print("🔧 PHASE 1: Assembly/Register Demonstration")
        print("="*50)
        
        print("📤 Sending '1' to trigger register progression demo...")
        response = self.send_command('1')
        
        print("\n📊 Educational Analysis:")
        print("- Students see direct register manipulation (DDRB, PORTB)")
        print("- Understanding hardware control at lowest level")
        print("- Bridge between assembly and C syntax")
        
        return response
    
    def demonstrate_phase2_communication(self):
        """Phase 2: Communication Evolution Demonstration"""
        print("\n" + "="*50)
        print("🔗 PHASE 2: Communication Evolution")
        print("="*50)
        
        print("📤 Sending '2' to trigger communication demo...")
        response = self.send_command('2')
        
        print("📤 Sending test character 'A'...")
        time.sleep(1)
        char_response = self.send_command('A')
        
        print("\n📊 Educational Analysis:")
        print("- Basic character I/O → Structured responses → JSON protocols")
        print("- Preparation for Python communication interfaces")
        print("- Protocol design thinking")
        
        return response + char_response
    
    def demonstrate_phase3_sensors(self):
        """Phase 3: Sensor Integration Demonstration"""
        print("\n" + "="*50)
        print("📈 PHASE 3: Sensor Integration")
        print("="*50)
        
        print("📤 Sending '3' to trigger sensor demo...")
        response = self.send_command('3')
        
        print("\n📊 Educational Analysis:")
        print("- Raw ADC → Voltage → Meaningful data → JSON")
        print("- Data processing pipeline demonstration")
        print("- Real-world sensor interfacing concepts")
        
        return response
    
    def interactive_mode(self):
        """Interactive mode - let user control the demo"""
        print("\n" + "="*50)
        print("🎮 INTERACTIVE MODE")
        print("="*50)
        print("Now you can interact directly with the ATmega128!")
        print("Available commands: 1, 2, 3, R")
        print("Type 'quit' to exit Python interface")
        
        while True:
            try:
                user_input = input("\n💻 Python Command: ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    break
                
                if user_input in ['1', '2', '3', 'R', 'r']:
                    print(f"📤 Sending '{user_input}' to ATmega128...")
                    self.send_command(user_input)
                else:
                    print("❓ Available commands: 1, 2, 3, R, quit")
                    
            except KeyboardInterrupt:
                break
        
        print("\n👋 Exiting Python interface...")
    
    def run_complete_demonstration(self):
        """Run complete educational progression demonstration"""
        print("🎓 ATmega128 Educational Framework - Python Integration")
        print("Assembly → C → Python Learning Progression")
        print(f"⏰ Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        # Wait for initial ATmega128 startup
        print("\n⏳ Waiting for ATmega128 startup...")
        time.sleep(3)
        
        # Read initial startup messages
        if self.ser.in_waiting > 0:
            startup = self.ser.read_all().decode('utf-8', errors='ignore')
            print("📥 ATmega128 Startup:")
            for line in startup.split('\n'):
                if line.strip():
                    print(f"    {line.strip()}")
        
        # Run demonstration phases
        try:
            self.demonstrate_phase1_registers()
            input("\n⏸️  Press Enter to continue to Phase 2...")
            
            self.demonstrate_phase2_communication()
            input("\n⏸️  Press Enter to continue to Phase 3...")
            
            self.demonstrate_phase3_sensors()
            input("\n⏸️  Press Enter for interactive mode...")
            
            self.interactive_mode()
            
        except KeyboardInterrupt:
            print("\n⏹️  Demonstration interrupted by user")
        
        print("\n✅ Educational demonstration complete!")
    
    def close(self):
        """Close serial connection"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            print("🔌 Serial connection closed")

def main():
    """Main function"""
    print("🐍 Python Educational Interface for ATmega128")
    print("=" * 50)
    
    # Check if port is specified
    port = 'COM3'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"🔌 Attempting to connect to {port}...")
    print("⚠️  Make sure ATmega128 is programmed with EDUCATIONAL_DEMO")
    
    try:
        # Create interface
        atmega = ATmega128Educational(port)
        
        # Run complete demonstration
        atmega.run_complete_demonstration()
        
    except Exception as e:
        print(f"❌ Error during demonstration: {e}")
    finally:
        if 'atmega' in locals():
            atmega.close()

if __name__ == "__main__":
    main()