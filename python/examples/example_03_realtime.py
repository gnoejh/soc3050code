# Example 3: Real-time Monitoring
# Learn how to monitor sensors in real-time with graphs

from atmega128 import ATmega128, SensorMonitor
import time
import threading

print("📈 Example 3: Real-time Monitoring")
print("=" * 40)

# Connect to ATmega128
mcu = ATmega128("COM3")  # Change COM3 to your port

if mcu.connected:
    print("✅ Connected successfully!")
    
    print("\n📊 Starting data logging...")
    mcu.start_data_logging("realtime_data.csv", interval=0.5)
    
    print("📈 Starting real-time monitor...")
    print("   Close the graph window to stop monitoring")
    
    # Create sensor monitor
    monitor = SensorMonitor(mcu)
    
    try:
        # This will show a live graph - close window to stop
        monitor.start_monitoring(interval=0.5)
    except KeyboardInterrupt:
        print("\n⏹️ Stopping...")
    
    # Stop logging and close
    mcu.stop_data_logging()
    mcu.close()
    print("🔌 Disconnected")
    
else:
    print("❌ Could not connect. Check your port!")

print("\n🎯 Next: Try example_04_iot.py")