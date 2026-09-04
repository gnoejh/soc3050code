# ATmega128 Serial Data Logger and Controller
# Example of high-level programming integration

import serial
import time
import json
import matplotlib.pyplot as plt
from datetime import datetime
import threading
import tkinter as tk
from tkinter import ttk

class ATmega128Interface:
    def __init__(self, port='COM3', baudrate=9600):
        """Initialize serial connection to ATmega128"""
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            print(f"✅ Connected to ATmega128 on {port}")
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.serial = None
        
        self.data_log = []
        self.running = False
    
    def send_command(self, command):
        """Send command to ATmega128"""
        if self.serial:
            self.serial.write(f"{command}\n".encode())
            print(f"📤 Sent: {command}")
    
    def read_data(self):
        """Read data from ATmega128"""
        if self.serial and self.serial.in_waiting:
            try:
                data = self.serial.readline().decode().strip()
                timestamp = datetime.now().isoformat()
                
                # Log data with timestamp
                log_entry = {
                    'timestamp': timestamp,
                    'data': data,
                    'parsed': self.parse_sensor_data(data)
                }
                self.data_log.append(log_entry)
                
                print(f"📥 Received: {data}")
                return log_entry
            except Exception as e:
                print(f"❌ Read error: {e}")
        return None
    
    def parse_sensor_data(self, data):
        """Parse sensor data from ATmega128"""
        try:
            # Example: "TEMP:25.6,HUMID:45.2,LIGHT:234"
            if ':' in data:
                pairs = data.split(',')
                parsed = {}
                for pair in pairs:
                    key, value = pair.split(':')
                    parsed[key] = float(value)
                return parsed
        except:
            pass
        return {'raw': data}
    
    def start_monitoring(self):
        """Start continuous monitoring"""
        self.running = True
        while self.running:
            self.read_data()
            time.sleep(0.1)
    
    def stop_monitoring(self):
        """Stop monitoring"""
        self.running = False
    
    def save_log(self, filename=None):
        """Save data log to file"""
        if not filename:
            filename = f"atmega128_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        with open(filename, 'w') as f:
            json.dump(self.data_log, f, indent=2)
        print(f"💾 Data saved to {filename}")
    
    def plot_data(self, sensor_key='TEMP'):
        """Plot sensor data over time"""
        timestamps = []
        values = []
        
        for entry in self.data_log:
            if 'parsed' in entry and sensor_key in entry['parsed']:
                timestamps.append(datetime.fromisoformat(entry['timestamp']))
                values.append(entry['parsed'][sensor_key])
        
        if values:
            plt.figure(figsize=(12, 6))
            plt.plot(timestamps, values, 'b-', linewidth=2)
            plt.title(f'ATmega128 {sensor_key} Data Over Time')
            plt.xlabel('Time')
            plt.ylabel(sensor_key)
            plt.grid(True, alpha=0.3)
            plt.xticks(rotation=45)
            plt.tight_layout()
            plt.show()
    
    def close(self):
        """Close serial connection"""
        if self.serial:
            self.serial.close()
            print("🔌 Serial connection closed")

class ATmegaGUI:
    def __init__(self):
        """Create GUI for ATmega128 control"""
        self.atmega = ATmega128Interface()
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("ATmega128 Control Center")
        self.root.geometry("600x400")
        
        # Control frame
        control_frame = ttk.Frame(self.root)
        control_frame.pack(pady=10)
        
        # Command buttons
        ttk.Button(control_frame, text="LED ON", 
                  command=lambda: self.atmega.send_command("LED_ON")).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="LED OFF", 
                  command=lambda: self.atmega.send_command("LED_OFF")).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Read Sensors", 
                  command=lambda: self.atmega.send_command("READ_ALL")).pack(side=tk.LEFT, padx=5)
        
        # Data display
        self.data_text = tk.Text(self.root, height=15, width=70)
        self.data_text.pack(pady=10)
        
        # Monitoring controls
        monitor_frame = ttk.Frame(self.root)
        monitor_frame.pack(pady=10)
        
        self.monitor_button = ttk.Button(monitor_frame, text="Start Monitoring", 
                                        command=self.toggle_monitoring)
        self.monitor_button.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(monitor_frame, text="Save Log", 
                  command=self.atmega.save_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(monitor_frame, text="Plot Data", 
                  command=lambda: self.atmega.plot_data('TEMP')).pack(side=tk.LEFT, padx=5)
        
        self.monitoring = False
        self.monitor_thread = None
    
    def toggle_monitoring(self):
        """Toggle monitoring on/off"""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_button.config(text="Stop Monitoring")
            self.monitor_thread = threading.Thread(target=self.monitor_loop)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
        else:
            self.monitoring = False
            self.monitor_button.config(text="Start Monitoring")
            self.atmega.stop_monitoring()
    
    def monitor_loop(self):
        """Monitoring loop"""
        while self.monitoring:
            data = self.atmega.read_data()
            if data:
                # Update GUI with new data
                self.root.after(0, self.update_display, data)
            time.sleep(0.1)
    
    def update_display(self, data):
        """Update data display"""
        self.data_text.insert(tk.END, f"{data['timestamp']}: {data['data']}\n")
        self.data_text.see(tk.END)
    
    def run(self):
        """Run the GUI"""
        try:
            self.root.mainloop()
        finally:
            self.atmega.close()

# Example usage functions
def simple_data_logger():
    """Simple data logging example"""
    atmega = ATmega128Interface()
    
    print("🔄 Starting data logging for 30 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 30:
        atmega.read_data()
        time.sleep(1)
    
    print(f"📊 Collected {len(atmega.data_log)} data points")
    atmega.save_log()
    atmega.plot_data()
    atmega.close()

def interactive_control():
    """Interactive control example"""
    atmega = ATmega128Interface()
    
    print("🎮 Interactive ATmega128 Control")
    print("Commands: LED_ON, LED_OFF, READ_SENSORS, quit")
    
    while True:
        command = input("Enter command: ").strip()
        if command.lower() == 'quit':
            break
        atmega.send_command(command)
        time.sleep(0.1)
        atmega.read_data()
    
    atmega.close()

if __name__ == "__main__":
    # Choose your interface
    print("🚀 ATmega128 Python Interface")
    print("1. GUI Control Center")
    print("2. Simple Data Logger")  
    print("3. Interactive Control")
    
    choice = input("Select option (1-3): ")
    
    if choice == "1":
        app = ATmegaGUI()
        app.run()
    elif choice == "2":
        simple_data_logger()
    elif choice == "3":
        interactive_control()
    else:
        print("Invalid choice")