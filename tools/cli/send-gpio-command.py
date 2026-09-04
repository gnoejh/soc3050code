#!/usr/bin/env python3
"""
Bidirectional GPIO Communication - Send commands to ATmega128

This script can send commands to ATmega128 by modifying the circuit file
to simulate button presses on PORTD.

Author: Professor Hong Jeong
Course: SOC 3050
"""

import xml.etree.ElementTree as ET
import time
from pathlib import Path

class SimulIDECommandSender:
    """Send commands to ATmega128 via SimulIDE circuit modification"""
    
    def __init__(self, circuit_file: str):
        self.circuit_file = Path(circuit_file)
        self.tree = None
        self.root = None
        self.load_circuit()
    
    def load_circuit(self):
        """Load SimulIDE circuit file"""
        if not self.circuit_file.exists():
            raise FileNotFoundError(f"Circuit file not found: {self.circuit_file}")
        
        self.tree = ET.parse(self.circuit_file)
        self.root = self.tree.getroot()
    
    def set_button_state(self, button_id: int, pressed: bool):
        """
        Set button state in circuit (simulates button press)
        
        Args:
            button_id: Button number (0-7) corresponding to PORTD bits
            pressed: True = pressed, False = released
        """
        # Find button component for PD{button_id}
        for item in self.root.findall(".//item[@itemtype='Push']"):
            # Check if this button is connected to the right pin
            label = item.get('label', '')
            if f'PD{button_id}' in label or f'Button-{button_id}' in label:
                # SimulIDE Push component: Norm_Close="false" means normally open
                # To press: set to true temporarily
                if pressed:
                    item.set('Norm_Close', 'true')
                else:
                    item.set('Norm_Close', 'false')
                
                print(f"  Button {button_id} → {'PRESSED' if pressed else 'RELEASED'}")
                return True
        
        print(f"  ⚠️  Button {button_id} not found in circuit")
        return False
    
    def send_command(self, command_byte: int):
        """
        Send 8-bit command to ATmega128 via PORTD buttons
        
        Args:
            command_byte: 8-bit value (0-255)
        """
        print(f"\n📤 Sending command: 0x{command_byte:02X} ({command_byte})")
        print(f"   Binary: {command_byte:08b}")
        
        # Set each button according to bit value
        for bit in range(8):
            pressed = bool(command_byte & (1 << bit))
            self.set_button_state(bit, pressed)
        
        # Save modified circuit
        self.save_circuit()
        
        print(f"   ✅ Circuit updated - ATmega128 will read command on next poll")
    
    def save_circuit(self):
        """Save modified circuit file"""
        # Backup original
        backup = self.circuit_file.with_suffix('.simu.backup')
        if not backup.exists():
            import shutil
            shutil.copy2(self.circuit_file, backup)
        
        # Save modified
        self.tree.write(self.circuit_file, encoding='utf-8', xml_declaration=True)
    
    def clear_all_buttons(self):
        """Release all buttons (set PORTD to 0xFF)"""
        print("\n🔄 Clearing all buttons...")
        for bit in range(8):
            self.set_button_state(bit, False)
        self.save_circuit()
        print("   ✅ All buttons released")

# Predefined commands (matching Main_Parallel.c)
COMMANDS = {
    'STATUS': 0x01,
    'RESET': 0x02,
    'VERSION': 0x03,
}

def main():
    """Interactive command sender"""
    circuit_file = 'W:/soc3050code/tools/simulide/Simulator110.simu'
    
    print("=" * 70)
    print("  BIDIRECTIONAL GPIO COMMUNICATION - COMMAND SENDER")
    print("=" * 70)
    print("")
    print("📡 Circuit File:", circuit_file)
    print("")
    
    try:
        sender = SimulIDECommandSender(circuit_file)
    except Exception as e:
        print(f"❌ Error loading circuit: {e}")
        return 1
    
    print("Available commands:")
    print("  1 = STATUS  (0x01) - Request status")
    print("  2 = RESET   (0x02) - Reset counter")
    print("  3 = VERSION (0x03) - Get version")
    print("  0 = CLEAR   (0x00) - Clear all buttons")
    print("  q = Quit")
    print("")
    
    while True:
        try:
            choice = input("Enter command (1-3, 0=clear, q=quit): ").strip().lower()
            
            if choice == 'q':
                print("\n👋 Goodbye!")
                break
            
            elif choice == '0':
                sender.clear_all_buttons()
            
            elif choice == '1':
                sender.send_command(COMMANDS['STATUS'])
                print("   📨 Expected response: 'STATUS:OK'")
                time.sleep(0.5)
                sender.clear_all_buttons()
            
            elif choice == '2':
                sender.send_command(COMMANDS['RESET'])
                print("   📨 Expected response: 'RESET:OK'")
                time.sleep(0.5)
                sender.clear_all_buttons()
            
            elif choice == '3':
                sender.send_command(COMMANDS['VERSION'])
                print("   📨 Expected response: 'VER:1.0'")
                time.sleep(0.5)
                sender.clear_all_buttons()
            
            else:
                # Custom byte value
                try:
                    value = int(choice, 0)  # Supports hex (0x..) and decimal
                    if 0 <= value <= 255:
                        sender.send_command(value)
                        time.sleep(0.5)
                        sender.clear_all_buttons()
                    else:
                        print("❌ Value must be 0-255")
                except ValueError:
                    print("❌ Invalid input")
            
        except KeyboardInterrupt:
            print("\n\n👋 Interrupted by user")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
    
    return 0

if __name__ == "__main__":
    import sys
    sys.exit(main())

"""
=============================================================================
USAGE NOTES:
=============================================================================

1. MANUAL METHOD (Using SimulIDE GUI):
   - Click buttons in SimulIDE to send commands
   - Each button = one bit of PORTD
   - ATmega128 reads PORTD in main loop
   - Responds with ACK message

2. AUTOMATIC METHOD (This Script):
   - Modifies circuit file to simulate button presses
   - Sends 8-bit commands programmatically
   - ATmega128 reads and responds

3. INTEGRATION WITH AI:
   ```python
   # AI decides command based on sensor data
   ai_decision = model.predict(sensor_data)
   
   if ai_decision == "request_status":
       sender.send_command(COMMANDS['STATUS'])
   ```

4. REAL-TIME CONTROL:
   ```python
   # Control loop
   while True:
       # Python AI/logic decides command
       command = compute_command()
       sender.send_command(command)
       
       # Wait for response from ATmega128
       response = read_gpio_response()
       process_response(response)
   ```

=============================================================================
"""
