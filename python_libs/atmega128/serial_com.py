"""
ATmega128 Serial Communication Handler
Enhanced version of the dashboard SerialHandler for project use

© 2025 Prof. Hong Jeong, IUT (Inha University in Tashkent)
"""

import serial
import serial.tools.list_ports
import time
from datetime import datetime
from typing import Optional, Callable, List, Dict


class ATmega128Serial:
    """
    Serial communication interface for ATmega128
    
    Features:
    - Auto-detection of ATmega128 devices
    - Context manager support (with statement)
    - Command/response protocol
    - Data buffering and logging
    - Callback support for real-time data
    """
    
    def __init__(self, port: Optional[str] = None, baudrate: int = 9600, 
                 timeout: float = 1.0, auto_connect: bool = True):
        """
        Initialize ATmega128 serial connection
        
        Args:
            port: COM port (e.g., 'COM3'). If None, auto-detect
            baudrate: Baud rate (default 9600)
            timeout: Read timeout in seconds
            auto_connect: Automatically connect on initialization
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        
        # Data management
        self.receive_buffer: List[Dict] = []
        self.max_buffer_size = 1000
        self.callbacks: List[Callable] = []
        
        # Logging
        self.log_enabled = False
        self.log_file = None
        
        # Auto-connect if requested
        if auto_connect:
            self.connect(port, baudrate)
    
    def __enter__(self):
        """Context manager entry"""
        if not self.connected:
            self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()
        return False
    
    def scan_ports(self) -> List[Dict]:
        """
        Scan available COM ports
        
        Returns:
            List of port dictionaries with device information
        """
        ports = []
        for port in serial.tools.list_ports.comports():
            port_type = 'unknown'
            if 'SimulIDE' in port.description or 'Virtual' in port.description:
                port_type = 'simulator'
            elif 'USB' in port.description or 'Serial' in port.description:
                port_type = 'hardware'
            
            ports.append({
                'port': port.device,
                'description': port.description,
                'hwid': port.hwid,
                'type': port_type
            })
        return ports
    
    def auto_detect(self) -> Optional[str]:
        """
        Auto-detect ATmega128 device
        
        Returns:
            Port name if found, None otherwise
        """
        ports = self.scan_ports()
        
        # Prefer simulator for educational use
        for port_info in ports:
            if port_info['type'] == 'simulator':
                print(f"🔍 Detected SimulIDE on {port_info['port']}")
                return port_info['port']
        
        # Then check hardware
        for port_info in ports:
            if port_info['type'] == 'hardware':
                print(f"🔍 Detected hardware on {port_info['port']}")
                return port_info['port']
        
        # Use first available port
        if ports:
            print(f"⚠️  Using first available port: {ports[0]['port']}")
            return ports[0]['port']
        
        return None
    
    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> bool:
        """
        Connect to ATmega128
        
        Args:
            port: COM port (auto-detect if None)
            baudrate: Baud rate (use default if None)
        
        Returns:
            True if connected successfully
        """
        # Auto-detect port if not specified
        if port is None:
            port = self.auto_detect()
            if port is None:
                print("❌ No ATmega128 device found")
                print("💡 Tip: Check connections and SimulIDE")
                return False
        
        if baudrate:
            self.baudrate = baudrate
        
        try:
            # Close existing connection
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            # Open new connection
            self.serial = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            
            self.port = port
            self.connected = True
            
            # Small delay for stability
            time.sleep(0.1)
            
            print(f"✅ Connected to {port} @ {self.baudrate} baud")
            return True
            
        except serial.SerialException as e:
            print(f"❌ Connection failed: {e}")
            print("💡 Tip: Check if port is in use by another application")
            self.connected = False
            return False
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from ATmega128"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            print(f"🔌 Disconnected from {self.port}")
        
        if self.log_file:
            self.log_file.close()
            self.log_file = None
    
    def send(self, data: str) -> bool:
        """
        Send data to ATmega128
        
        Args:
            data: String to send
        
        Returns:
            True if sent successfully
        """
        if not self.connected or not self.serial:
            print("❌ Not connected")
            return False
        
        try:
            if isinstance(data, str):
                data = data.encode('utf-8')
            
            self.serial.write(data)
            return True
            
        except Exception as e:
            print(f"❌ Send error: {e}")
            return False
    
    def send_command(self, command: str) -> bool:
        """
        Send command with newline terminator
        
        Args:
            command: Command string
        
        Returns:
            True if sent successfully
        """
        if not command.endswith('\n'):
            command += '\n'
        return self.send(command)
    
    def readline(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Read one line from ATmega128
        
        Args:
            timeout: Override default timeout
        
        Returns:
            Line as string, or None if timeout/error
        """
        if not self.connected or not self.serial:
            return None
        
        try:
            # Temporarily override timeout if specified
            old_timeout = self.serial.timeout
            if timeout is not None:
                self.serial.timeout = timeout
            
            line = self.serial.readline().decode('utf-8', errors='ignore').strip()
            
            # Restore original timeout
            if timeout is not None:
                self.serial.timeout = old_timeout
            
            if line:
                # Log received data
                entry = {
                    'timestamp': datetime.now().isoformat(),
                    'data': line
                }
                
                # Add to buffer
                self.receive_buffer.append(entry)
                if len(self.receive_buffer) > self.max_buffer_size:
                    self.receive_buffer.pop(0)
                
                # Log to file if enabled
                if self.log_file:
                    self.log_file.write(f"{entry['timestamp']},{line}\n")
                    self.log_file.flush()
                
                # Call callbacks
                for callback in self.callbacks:
                    try:
                        callback(entry)
                    except Exception as e:
                        print(f"⚠️  Callback error: {e}")
                
                return line
            
            return None
            
        except Exception as e:
            print(f"❌ Read error: {e}")
            return None
    
    def wait_for_response(self, timeout: float = 2.0) -> Optional[str]:
        """
        Wait for response from ATmega128
        
        Args:
            timeout: Maximum wait time in seconds
        
        Returns:
            Response string or None if timeout
        """
        return self.readline(timeout=timeout)
    
    def register_callback(self, callback: Callable):
        """
        Register callback for received data
        
        Args:
            callback: Function(data_dict) to call on data reception
        """
        self.callbacks.append(callback)
    
    def enable_logging(self, filename: str):
        """
        Enable data logging to file
        
        Args:
            filename: Log file path
        """
        try:
            self.log_file = open(filename, 'a', encoding='utf-8')
            self.log_enabled = True
            print(f"📝 Logging to {filename}")
        except Exception as e:
            print(f"❌ Failed to enable logging: {e}")
    
    def get_buffer(self, last_n: Optional[int] = None) -> List[Dict]:
        """
        Get receive buffer
        
        Args:
            last_n: Return last N entries (None for all)
        
        Returns:
            List of data entries
        """
        if last_n:
            return self.receive_buffer[-last_n:]
        return self.receive_buffer.copy()
    
    def clear_buffer(self):
        """Clear receive buffer"""
        self.receive_buffer.clear()
        print("🗑️  Buffer cleared")
    
    def flush(self):
        """Flush input/output buffers"""
        if self.serial:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()


# Example usage
if __name__ == '__main__':
    print("=" * 70)
    print("ATmega128 Serial Communication Test")
    print("=" * 70)
    
    # Using context manager
    with ATmega128Serial(baudrate=9600) as mcu:
        # Send test command
        mcu.send_command('TEST')
        
        # Wait for response
        response = mcu.wait_for_response(timeout=2.0)
        if response:
            print(f"Response: {response}")
        else:
            print("No response received")

