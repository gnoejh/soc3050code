"""
ATmega128 Student Interface
==========================

Simple Python class for communicating with ATmega128 microcontroller.
Perfect for students learning embedded systems and IoT development.

Example Usage:
    from atmega128 import ATmega128
    
    # Connect to microcontroller
    mcu = ATmega128("COM3")
    
    # Send commands
    mcu.send("LED_ON")
    mcu.send("READ_TEMP")
    
    # Read responses
    data = mcu.read()
    print(f"Temperature: {data}")
    
    # Close connection
    mcu.close()
"""

import serial
import time
import json
import csv
from datetime import datetime
from typing import Optional, Dict, List, Any
import threading


class ATmega128:
    """
    Simple interface for ATmega128 microcontroller communication.
    
    This class makes it easy for students to:
    - Send commands to the microcontroller
    - Read sensor data
    - Log data to files
    - Control hardware (LEDs, motors, etc.)
    """
    
    def __init__(self, port: str = "COM3", baudrate: int = 9600, timeout: float = 1.0):
        """
        Initialize connection to ATmega128.
        
        Args:
            port: Serial port (e.g., "COM3", "/dev/ttyUSB0")
            baudrate: Communication speed (default: 9600)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.data_log = []
        
        # Try to connect
        self.connect()
    
    def connect(self) -> bool:
        """
        Connect to the ATmega128.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.connected = True
            print(f"✅ Connected to ATmega128 on {self.port}")
            time.sleep(2)  # Wait for microcontroller to reset
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.connected = False
            return False
    
    def send(self, command: str) -> bool:
        """
        Send a command to the ATmega128.
        
        Args:
            command: Command string (e.g., "LED_ON", "READ_TEMP")
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.connected or not self.serial:
            print("❌ Not connected to ATmega128")
            return False
        
        try:
            # Add newline if not present
            if not command.endswith('\n'):
                command += '\n'
            
            self.serial.write(command.encode())
            print(f"📤 Sent: {command.strip()}")
            return True
        except Exception as e:
            print(f"❌ Send error: {e}")
            return False
    
    def read(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Read response from ATmega128.
        
        Args:
            timeout: Read timeout (uses default if None)
            
        Returns:
            Response string or None if no data
        """
        if not self.connected or not self.serial:
            return None
        
        try:
            # Set temporary timeout if specified
            old_timeout = self.serial.timeout
            if timeout is not None:
                self.serial.timeout = timeout
            
            if self.serial.in_waiting > 0:
                data = self.serial.readline().decode().strip()
                print(f"📥 Received: {data}")
                
                # Log the data
                self.data_log.append({
                    'timestamp': datetime.now().isoformat(),
                    'data': data
                })
                
                # Restore timeout
                if timeout is not None:
                    self.serial.timeout = old_timeout
                
                return data
            
            # Restore timeout
            if timeout is not None:
                self.serial.timeout = old_timeout
                
            return None
        except Exception as e:
            print(f"❌ Read error: {e}")
            return None
    
    def send_and_wait(self, command: str, wait_time: float = 0.1) -> Optional[str]:
        """
        Send command and wait for response.
        
        Args:
            command: Command to send
            wait_time: Time to wait for response
            
        Returns:
            Response string or None
        """
        if self.send(command):
            time.sleep(wait_time)
            return self.read()
        return None
    
    # Convenience methods for common operations
    def led_on(self) -> bool:
        """Turn on LED."""
        return self.send("LED_ON")
    
    def led_off(self) -> bool:
        """Turn off LED.""" 
        return self.send("LED_OFF")
    
    def read_temperature(self) -> Optional[float]:
        """
        Read temperature sensor.
        
        Returns:
            Temperature in Celsius or None if failed
        """
        response = self.send_and_wait("READ_TEMP")
        if response:
            try:
                # Parse "TEMP:25.6" format
                if "TEMP:" in response:
                    temp_str = response.split("TEMP:")[1].split(",")[0]
                    return float(temp_str)
                else:
                    # Try to parse as direct number
                    return float(response)
            except ValueError:
                print(f"⚠️ Could not parse temperature: {response}")
        return None
    
    def read_sensors(self) -> Dict[str, Any]:
        """
        Read all sensors.
        
        Returns:
            Dictionary with sensor data
        """
        response = self.send_and_wait("READ_ALL")
        data = {}
        
        if response:
            try:
                # Parse "TEMP:25.6,HUMID:45.2,LIGHT:234" format
                pairs = response.split(",")
                for pair in pairs:
                    if ":" in pair:
                        key, value = pair.split(":", 1)
                        try:
                            data[key] = float(value)
                        except ValueError:
                            data[key] = value
            except Exception as e:
                print(f"⚠️ Could not parse sensor data: {e}")
                data['raw'] = response
        
        return data
    
    def start_data_logging(self, filename: str = None, interval: float = 1.0):
        """
        Start automatic data logging in background.
        
        Args:
            filename: Log filename (auto-generated if None)
            interval: Logging interval in seconds
        """
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"atmega128_log_{timestamp}.csv"
        
        self.log_filename = filename
        self.logging = True
        
        def log_loop():
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'data'])
                
                while self.logging:
                    data = self.read()
                    if data:
                        writer.writerow([datetime.now().isoformat(), data])
                        csvfile.flush()
                    time.sleep(interval)
        
        self.log_thread = threading.Thread(target=log_loop)
        self.log_thread.daemon = True
        self.log_thread.start()
        print(f"📊 Started logging to {filename}")
    
    def stop_data_logging(self):
        """Stop automatic data logging."""
        if hasattr(self, 'logging'):
            self.logging = False
            print("📊 Stopped data logging")
    
    def save_log(self, filename: str = None):
        """
        Save data log to JSON file.
        
        Args:
            filename: Output filename (auto-generated if None)
        """
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"atmega128_data_{timestamp}.json"
        
        with open(filename, 'w') as f:
            json.dump(self.data_log, f, indent=2)
        
        print(f"💾 Saved {len(self.data_log)} data points to {filename}")
    
    def get_available_ports(self) -> List[str]:
        """
        Get list of available serial ports.
        
        Returns:
            List of port names
        """
        import serial.tools.list_ports
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports
    
    def close(self):
        """Close the serial connection."""
        if hasattr(self, 'logging'):
            self.stop_data_logging()
        
        if self.serial and self.connected:
            self.serial.close()
            self.connected = False
            print("🔌 Disconnected from ATmega128")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


class SensorMonitor:
    """
    Real-time sensor monitoring with plotting capabilities.
    Perfect for students to visualize data from ATmega128.
    """
    
    def __init__(self, atmega: ATmega128):
        """
        Initialize sensor monitor.
        
        Args:
            atmega: Connected ATmega128 instance
        """
        self.atmega = atmega
        self.data_history = {
            'timestamps': [],
            'temperature': [],
            'humidity': [],
            'light': []
        }
        self.max_points = 100
    
    def start_monitoring(self, interval: float = 1.0):
        """
        Start real-time monitoring with live plot.
        
        Args:
            interval: Update interval in seconds
        """
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
        
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
        fig.suptitle('ATmega128 Real-time Sensor Data')
        
        def update_plot(frame):
            # Read new data
            data = self.atmega.read_sensors()
            
            if data:
                now = datetime.now()
                self.data_history['timestamps'].append(now)
                
                # Update data history
                for sensor in ['temperature', 'humidity', 'light']:
                    if sensor.upper() in data:
                        self.data_history[sensor].append(data[sensor.upper()])
                    else:
                        self.data_history[sensor].append(None)
                
                # Keep only recent data
                for key in self.data_history:
                    if len(self.data_history[key]) > self.max_points:
                        self.data_history[key] = self.data_history[key][-self.max_points:]
                
                # Clear and update plots
                for ax in [ax1, ax2, ax3]:
                    ax.clear()
                
                # Temperature plot
                if any(self.data_history['temperature']):
                    ax1.plot(self.data_history['timestamps'], self.data_history['temperature'], 'r-')
                    ax1.set_ylabel('Temperature (°C)')
                    ax1.grid(True)
                
                # Humidity plot
                if any(self.data_history['humidity']):
                    ax2.plot(self.data_history['timestamps'], self.data_history['humidity'], 'b-')
                    ax2.set_ylabel('Humidity (%)')
                    ax2.grid(True)
                
                # Light plot
                if any(self.data_history['light']):
                    ax3.plot(self.data_history['timestamps'], self.data_history['light'], 'g-')
                    ax3.set_ylabel('Light Level')
                    ax3.set_xlabel('Time')
                    ax3.grid(True)
                
                plt.tight_layout()
        
        ani = animation.FuncAnimation(fig, update_plot, interval=interval*1000)
        plt.show()


# Convenience functions for students
def list_ports():
    """List available serial ports."""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    print("📡 Available Serial Ports:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
    return [port.device for port in ports]


def quick_connect(port: str = None):
    """
    Quick connection helper for students.
    
    Args:
        port: Serial port (auto-detect if None)
        
    Returns:
        Connected ATmega128 instance
    """
    if port is None:
        ports = list_ports()
        if not ports:
            print("❌ No serial ports found!")
            return None
        port = ports[0]  # Use first available port
        print(f"🔗 Auto-connecting to {port}")
    
    return ATmega128(port)


if __name__ == "__main__":
    # Simple test when run directly
    print("🧪 Testing ATmega128 interface...")
    
    # List available ports
    ports = list_ports()
    
    if ports:
        # Try to connect to first port
        with quick_connect() as mcu:
            if mcu and mcu.connected:
                print("✅ Connection test successful!")
                
                # Test basic commands
                mcu.led_on()
                time.sleep(1)
                mcu.led_off()
                
                # Test sensor reading
                temp = mcu.read_temperature()
                if temp:
                    print(f"🌡️ Temperature: {temp}°C")
                
            else:
                print("❌ Connection test failed!")
    else:
        print("❌ No serial ports available for testing!")