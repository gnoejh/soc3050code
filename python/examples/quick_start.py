# Quick Start Guide for Students
# Copy this code to get started immediately!

from atmega128 import quick_connect, list_ports
import time

print("🚀 ATmega128 Quick Start")
print("=" * 30)

# Step 1: List available ports
print("📡 Finding your ATmega128...")
list_ports()

# Step 2: Connect automatically
print("\n🔗 Connecting...")
mcu = quick_connect()  # Auto-detects port

if mcu and mcu.connected:
    print("✅ Success! Your ATmega128 is ready!")
    
    # Step 3: Try some basic commands
    print("\n🧪 Testing basic functions...")
    
    # Blink LED
    print("💡 Blinking LED...")
    for i in range(3):
        mcu.led_on()
        time.sleep(0.5)
        mcu.led_off()
        time.sleep(0.5)
    
    # Read a sensor
    print("🌡️ Reading temperature...")
    temp = mcu.read_temperature()
    if temp:
        print(f"   Temperature: {temp:.1f}°C")
    
    # Say hello
    print("👋 Saying hello...")
    response = mcu.send_and_wait("Hello from Python!")
    if response:
        print(f"   ATmega128 replied: {response}")
    
    # Clean up
    mcu.close()
    print("\n🎉 All done! Check out the examples/ folder for more.")
    
else:
    print("❌ Connection failed!")
    print("💡 Make sure:")
    print("   - ATmega128 is connected via USB")
    print("   - Correct drivers are installed")
    print("   - No other programs are using the serial port")

print("\n📚 Next steps:")
print("   - examples/example_01_basic.py - Learn basic communication")
print("   - examples/example_02_sensors.py - Read sensor data")
print("   - examples/example_03_realtime.py - Real-time monitoring")
print("   - examples/example_04_iot.py - Web dashboard")