#!/usr/bin/env python3
"""
Real-time Sensor Monitoring for ATmega128
Continuously monitors sensors and logs data for Python analysis
"""

import serial
import time
import json
import csv
import threading
from datetime import datetime
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class SensorMonitor:
    def __init__(self, port='COM3', baud=9600, max_samples=100):
        """Initialize sensor monitoring"""
        self.port = port
        self.baud = baud
        self.max_samples = max_samples
        self.is_monitoring = False
        self.data_buffer = deque(maxlen=max_samples)
        self.ser = None
        
        # Data storage
        self.csv_filename = f"sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
    def connect(self):
        """Connect to ATmega128"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Arduino reset time
            print(f"✅ Connected to ATmega128 on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def parse_sensor_data(self, raw_data):
        """Parse sensor data from ATmega128 response"""
        try:
            # Expected format from C code:
            # "Temp: 25.3°C, Light: 512, Sound: 1023"
            
            data_point = {
                'timestamp': datetime.now(),
                'temperature': None,
                'light': None,
                'sound': None,
                'raw': raw_data.strip()
            }
            
            # Parse temperature
            if "Temp:" in raw_data:
                temp_start = raw_data.find("Temp:") + 5
                temp_end = raw_data.find("°C", temp_start)
                if temp_end > temp_start:
                    try:
                        data_point['temperature'] = float(raw_data[temp_start:temp_end].strip())
                    except ValueError:
                        pass
            
            # Parse light sensor
            if "Light:" in raw_data:
                light_start = raw_data.find("Light:") + 6
                light_end = raw_data.find(",", light_start)
                if light_end == -1:
                    light_end = len(raw_data)
                try:
                    data_point['light'] = int(raw_data[light_start:light_end].strip())
                except ValueError:
                    pass
            
            # Parse sound sensor
            if "Sound:" in raw_data:
                sound_start = raw_data.find("Sound:") + 6
                try:
                    data_point['sound'] = int(raw_data[sound_start:].strip())
                except ValueError:
                    pass
            
            return data_point
            
        except Exception as e:
            print(f"⚠️  Error parsing sensor data: {e}")
            return None
    
    def save_to_csv(self, data_point):
        """Save data point to CSV file"""
        try:
            with open(self.csv_filename, 'a', newline='') as csvfile:
                fieldnames = ['timestamp', 'temperature', 'light', 'sound', 'raw']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                # Write header if file is new
                if csvfile.tell() == 0:
                    writer.writeheader()
                
                writer.writerow(data_point)
        except Exception as e:
            print(f"⚠️  Error saving to CSV: {e}")
    
    def start_monitoring(self, duration_seconds=60):
        """Start sensor monitoring for specified duration"""
        if not self.connect():
            return False
        
        print(f"🔍 Starting sensor monitoring for {duration_seconds} seconds...")
        print(f"💾 Data will be saved to: {self.csv_filename}")
        
        # Send command to start sensor demo
        self.ser.write(b'3')  # Sensor demo command
        time.sleep(1)
        
        self.is_monitoring = True
        start_time = time.time()
        
        try:
            while self.is_monitoring and (time.time() - start_time) < duration_seconds:
                if self.ser.in_waiting > 0:
                    raw_data = self.ser.readline().decode('utf-8', errors='ignore')
                    
                    if raw_data.strip():
                        print(f"📥 {raw_data.strip()}")
                        
                        # Parse and store data
                        data_point = self.parse_sensor_data(raw_data)
                        if data_point and any([data_point['temperature'], 
                                             data_point['light'], 
                                             data_point['sound']]):
                            self.data_buffer.append(data_point)
                            self.save_to_csv(data_point)
                            
                            # Print parsed data
                            print(f"📊 Parsed - Temp: {data_point['temperature']}, "
                                  f"Light: {data_point['light']}, "
                                  f"Sound: {data_point['sound']}")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n⏹️  Monitoring stopped by user")
        
        self.is_monitoring = False
        print(f"✅ Monitoring complete. {len(self.data_buffer)} samples collected.")
        
        if self.ser:
            self.ser.close()
        
        return True
    
    def get_latest_data(self):
        """Get latest sensor readings"""
        if self.data_buffer:
            return self.data_buffer[-1]
        return None
    
    def get_all_data(self):
        """Get all collected sensor data"""
        return list(self.data_buffer)
    
    def print_summary(self):
        """Print summary of collected data"""
        if not self.data_buffer:
            print("📊 No data collected yet")
            return
        
        # Calculate statistics
        temperatures = [d['temperature'] for d in self.data_buffer if d['temperature'] is not None]
        light_values = [d['light'] for d in self.data_buffer if d['light'] is not None]
        sound_values = [d['sound'] for d in self.data_buffer if d['sound'] is not None]
        
        print("\n" + "="*50)
        print("📊 SENSOR DATA SUMMARY")
        print("="*50)
        print(f"🔢 Total samples: {len(self.data_buffer)}")
        
        if temperatures:
            print(f"🌡️  Temperature: {min(temperatures):.1f}°C - {max(temperatures):.1f}°C (avg: {sum(temperatures)/len(temperatures):.1f}°C)")
        
        if light_values:
            print(f"💡 Light: {min(light_values)} - {max(light_values)} (avg: {sum(light_values)//len(light_values)})")
        
        if sound_values:
            print(f"🔊 Sound: {min(sound_values)} - {max(sound_values)} (avg: {sum(sound_values)//len(sound_values)})")
        
        print(f"💾 Data saved to: {self.csv_filename}")

def main():
    """Main function for standalone sensor monitoring"""
    import sys
    
    port = 'COM3'
    duration = 30  # Default 30 seconds
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            duration = int(sys.argv[2])
        except ValueError:
            print("Invalid duration, using 30 seconds")
    
    print("🔍 ATmega128 Sensor Monitor")
    print("=" * 30)
    print(f"📡 Port: {port}")
    print(f"⏱️  Duration: {duration} seconds")
    print("⚠️  Make sure ATmega128 is running EDUCATIONAL_DEMO")
    
    monitor = SensorMonitor(port)
    
    if monitor.start_monitoring(duration):
        monitor.print_summary()
        
        # Ask if user wants to visualize data
        try:
            choice = input("\n📈 Would you like to visualize the data? (y/n): ")
            if choice.lower().startswith('y'):
                from .data_visualization import DataVisualizer
                viz = DataVisualizer(monitor.get_all_data())
                viz.plot_sensor_data()
        except ImportError:
            print("⚠️  Data visualization module not available")
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    main()