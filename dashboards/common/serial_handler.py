"""
Serial Communication Handler
Manages serial port connection and communication with ATmega128

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)
All rights reserved for educational purposes.
Contact: linkedin.com/in/gnoejh53
"""

import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime


class SerialHandler:
    """Handle serial communication with ATmega128"""
    
    def __init__(self, port=None, baudrate=9600, timeout=1):
        """
        Initialize serial handler
        
        Args:
            port: COM port (e.g., 'COM3'). If None, will auto-detect
            baudrate: Baud rate (default 9600)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.monitoring = False
        self.callbacks = []
        
        # Data buffer
        self.receive_buffer = []
        self.max_buffer_size = 1000
        
        # Auto-connect if port specified
        if port:
            self.connect(port, baudrate)
    
    def scan_ports(self):
        """Scan available COM ports"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                'port': port.device,
                'description': port.description,
                'hwid': port.hwid,
                'type': self._detect_port_type(port)
            })
        return ports
    
    def _detect_port_type(self, port):
        """Detect if port is SimulIDE or hardware"""
        if 'SimulIDE' in port.description or 'Virtual' in port.description:
            return 'simulator'
        elif 'USB' in port.description or 'Serial' in port.description:
            return 'hardware'
        else:
            return 'unknown'
    
    def auto_detect_device(self):
        """Auto-detect ATmega128 device (SimulIDE or hardware)"""
        ports = self.scan_ports()
        
        # Prefer SimulIDE for educational use
        for port_info in ports:
            if port_info['type'] == 'simulator':
                return port_info['port'], 'simulator'
        
        # Then check for hardware
        for port_info in ports:
            if port_info['type'] == 'hardware':
                return port_info['port'], 'hardware'
        
        return None, None
    
    def connect(self, port=None, baudrate=None):
        """
        Connect to serial port
        
        Args:
            port: COM port (if None, auto-detect)
            baudrate: Baud rate (if None, use default)
        """
        # Auto-detect if port not specified
        if port is None:
            port, device_type = self.auto_detect_device()
            if port is None:
                print("❌ No ATmega128 device found")
                return False
            print(f"🔍 Auto-detected {device_type} on {port}")
        
        if baudrate:
            self.baudrate = baudrate
        
        try:
            # Close existing connection if any
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            # Open new connection
            self.serial = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            
            self.port = port
            self.connected = True
            
            print(f"✅ Connected to {port} at {self.baudrate} baud")
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.stop_monitoring()
        
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            print(f"🔌 Disconnected from {self.port}")
    
    def send(self, data):
        """
        Send data to ATmega128
        
        Args:
            data: String or bytes to send
        """
        if not self.connected or not self.serial:
            print("❌ Not connected")
            return False
        
        try:
            if isinstance(data, str):
                data = data.encode()
            
            self.serial.write(data)
            print(f"📤 Sent: {data.decode(errors='ignore')}")
            return True
            
        except Exception as e:
            print(f"❌ Send error: {e}")
            return False
    
    def send_command(self, command):
        """
        Send command with newline
        
        Args:
            command: Command string
        """
        if not command.endswith('\n'):
            command += '\n'
        return self.send(command)
    
    def receive(self):
        """Receive data from ATmega128"""
        if not self.connected or not self.serial:
            return None
        
        try:
            if self.serial.in_waiting > 0:
                data = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    entry = {
                        'timestamp': datetime.now().isoformat(),
                        'data': data
                    }
                    
                    # Add to buffer
                    self.receive_buffer.append(entry)
                    if len(self.receive_buffer) > self.max_buffer_size:
                        self.receive_buffer.pop(0)
                    
                    # Call registered callbacks
                    for callback in self.callbacks:
                        try:
                            callback(entry)
                        except Exception as e:
                            print(f"❌ Callback error: {e}")
                    
                    return entry
        except Exception as e:
            print(f"❌ Receive error: {e}")
        
        return None
    
    def register_callback(self, callback):
        """
        Register callback function for received data
        
        Args:
            callback: Function to call with received data
        """
        self.callbacks.append(callback)
    
    def start_monitoring(self, interval=0.05):
        """
        Start continuous monitoring in background thread
        
        Args:
            interval: Polling interval in seconds
        """
        if self.monitoring:
            return
        
        self.monitoring = True
        
        def monitor_loop():
            while self.monitoring:
                self.receive()
                time.sleep(interval)
        
        thread = threading.Thread(target=monitor_loop, daemon=True)
        thread.start()
        print("🔄 Started monitoring")
    
    def stop_monitoring(self):
        """Stop continuous monitoring"""
        self.monitoring = False
        print("⏸️ Stopped monitoring")
    
    def get_buffer(self, last_n=None):
        """
        Get receive buffer
        
        Args:
            last_n: Return only last N entries (None for all)
        """
        if last_n:
            return self.receive_buffer[-last_n:]
        return self.receive_buffer
    
    def clear_buffer(self):
        """Clear receive buffer"""
        self.receive_buffer.clear()
        print("🗑️ Buffer cleared")
