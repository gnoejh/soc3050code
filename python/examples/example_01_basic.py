# Example 1: Basic Communication
# Learn how to connect and send simple commands

from atmega128 import ATmega128
import time

print("🚀 Example 1: Basic Communication")
print("=" * 40)

# Connect to ATmega128
mcu = ATmega128("COM3")  # Change COM3 to your port

if mcu.connected:
    print("✅ Connected successfully!")
    
    # Send some basic commands
    print("\n📤 Sending commands...")
    mcu.send("Hello ATmega128!")
    time.sleep(0.5)
    
    # Turn LED on and off
    print("💡 Controlling LED...")
    mcu.led_on()
    time.sleep(1)
    mcu.led_off()
    time.sleep(1)
    
    # Send custom command
    response = mcu.send_and_wait("READ_STATUS")
    if response:
        print(f"📥 Microcontroller says: {response}")
    
    # Close connection
    mcu.close()
    print("🔌 Disconnected")
else:
    print("❌ Could not connect. Check your port!")
    print("💡 Available ports:", mcu.get_available_ports())

print("\n🎯 Next: Try example_02_sensors.py")