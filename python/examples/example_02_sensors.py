# Example 2: Reading Sensors
# Learn how to read sensor data from ATmega128

from atmega128 import ATmega128
import time

print("🌡️ Example 2: Reading Sensors")
print("=" * 40)

# Connect using context manager (automatically closes)
with ATmega128("COM3") as mcu:  # Change COM3 to your port
    if mcu.connected:
        print("✅ Connected successfully!")
        
        # Read temperature sensor
        print("\n🌡️ Reading temperature...")
        for i in range(5):
            temp = mcu.read_temperature()
            if temp is not None:
                print(f"  Reading {i+1}: {temp:.1f}°C")
            else:
                print(f"  Reading {i+1}: No data")
            time.sleep(1)
        
        # Read all sensors at once
        print("\n📊 Reading all sensors...")
        for i in range(3):
            sensors = mcu.read_sensors()
            if sensors:
                print(f"  Reading {i+1}:")
                for sensor, value in sensors.items():
                    if isinstance(value, float):
                        print(f"    {sensor}: {value:.2f}")
                    else:
                        print(f"    {sensor}: {value}")
            time.sleep(1)
        
        print("\n💾 Data automatically logged!")
        print(f"   Total readings: {len(mcu.data_log)}")
        
        # Save log file
        mcu.save_log("sensor_readings.json")
        
    else:
        print("❌ Could not connect. Check your port!")

print("\n🎯 Next: Try example_03_realtime.py")