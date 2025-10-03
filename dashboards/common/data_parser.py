"""
Data Parser Utilities
Parse and format data from ATmega128 serial communication

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)
All rights reserved for educational purposes.
Contact: linkedin.com/in/gnoejh53
"""

import re
from datetime import datetime


class DataParser:
    """Parse various data formats from ATmega128"""
    
    @staticmethod
    def parse_key_value(data, delimiter=',', separator=':'):
        """
        Parse key-value format: "KEY1:value1,KEY2:value2"
        
        Args:
            data: String data to parse
            delimiter: Separator between pairs (default: ',')
            separator: Separator between key and value (default: ':')
        
        Returns:
            Dictionary of parsed data
        """
        result = {}
        
        try:
            pairs = data.split(delimiter)
            for pair in pairs:
                if separator in pair:
                    key, value = pair.split(separator, 1)
                    key = key.strip()
                    value = value.strip()
                    
                    # Try to convert to number
                    try:
                        if '.' in value:
                            result[key] = float(value)
                        else:
                            result[key] = int(value)
                    except:
                        result[key] = value
        except Exception as e:
            print(f"❌ Parse error: {e}")
        
        return result
    
    @staticmethod
    def parse_led_state(data):
        """
        Parse LED state from PORTB value
        
        Args:
            data: String containing hex value (e.g., "PORTB:0xAA")
        
        Returns:
            List of 8 boolean values representing LED states
        """
        # Find hex value
        match = re.search(r'0x([0-9A-Fa-f]{2})', data)
        if match:
            portb_value = int(match.group(1), 16)
            return [(portb_value >> i) & 1 for i in range(8)]
        return [False] * 8
    
    @staticmethod
    def parse_adc_value(data):
        """
        Parse ADC value
        
        Args:
            data: String containing ADC data (e.g., "ADC:0,VALUE:512")
        
        Returns:
            Dictionary with channel, raw value, and voltage
        """
        result = {}
        
        # Find ADC channel
        match = re.search(r'ADC:(\d+)', data)
        if match:
            result['channel'] = int(match.group(1))
        
        # Find value
        match = re.search(r'VALUE:(\d+)', data)
        if match:
            raw_value = int(match.group(1))
            result['raw'] = raw_value
            result['voltage'] = (raw_value / 1023.0) * 5.0  # Assuming 5V reference
        
        return result
    
    @staticmethod
    def parse_sensor_data(data):
        """
        Parse common sensor data formats
        
        Args:
            data: Sensor data string
        
        Returns:
            Dictionary with parsed sensor values
        """
        result = {}
        
        # Temperature
        match = re.search(r'TEMP:([0-9.]+)', data)
        if match:
            result['temperature'] = float(match.group(1))
        
        # Humidity
        match = re.search(r'HUMID:([0-9.]+)', data)
        if match:
            result['humidity'] = float(match.group(1))
        
        # Light
        match = re.search(r'LIGHT:([0-9.]+)', data)
        if match:
            result['light'] = float(match.group(1))
        
        # Distance (ultrasonic)
        match = re.search(r'DIST:([0-9.]+)', data)
        if match:
            result['distance'] = float(match.group(1))
        
        return result
    
    @staticmethod
    def parse_motor_data(data):
        """
        Parse motor control data
        
        Args:
            data: Motor data string (e.g., "MOTOR:DC,SPEED:75,DIR:FWD")
        
        Returns:
            Dictionary with motor parameters
        """
        result = {}
        
        # Motor type
        match = re.search(r'MOTOR:([A-Z]+)', data)
        if match:
            result['type'] = match.group(1)
        
        # Speed
        match = re.search(r'SPEED:(\d+)', data)
        if match:
            result['speed'] = int(match.group(1))
        
        # Direction
        match = re.search(r'DIR:([A-Z]+)', data)
        if match:
            result['direction'] = match.group(1)
        
        return result
    
    @staticmethod
    def parse_ai_prediction(data):
        """
        Parse AI prediction results
        
        Args:
            data: AI prediction string (e.g., "AI:class=cup,conf=0.92")
        
        Returns:
            Dictionary with prediction results
        """
        result = {}
        
        # Class
        match = re.search(r'class=([^,]+)', data)
        if match:
            result['class'] = match.group(1)
        
        # Confidence
        match = re.search(r'conf=([0-9.]+)', data)
        if match:
            result['confidence'] = float(match.group(1))
        
        # Gesture
        match = re.search(r'GESTURE:([A-Z]+)', data)
        if match:
            result['gesture'] = match.group(1)
        
        return result
    
    @staticmethod
    def format_for_display(data, data_type='general'):
        """
        Format parsed data for display
        
        Args:
            data: Parsed data dictionary
            data_type: Type of data ('sensor', 'motor', 'ai', etc.)
        
        Returns:
            Formatted string for display
        """
        if data_type == 'sensor':
            parts = []
            if 'temperature' in data:
                parts.append(f"🌡️ Temp: {data['temperature']:.1f}°C")
            if 'humidity' in data:
                parts.append(f"💧 Humidity: {data['humidity']:.1f}%")
            if 'light' in data:
                parts.append(f"💡 Light: {data['light']:.0f}")
            if 'distance' in data:
                parts.append(f"📏 Distance: {data['distance']:.1f}cm")
            return " | ".join(parts)
        
        elif data_type == 'motor':
            return f"⚙️ {data.get('type', 'Motor')}: Speed={data.get('speed', 0)}%, Dir={data.get('direction', 'STOP')}"
        
        elif data_type == 'ai':
            return f"🤖 AI: {data.get('class', 'unknown')} ({data.get('confidence', 0):.0%} confidence)"
        
        else:
            # General key-value display
            return ", ".join([f"{k}: {v}" for k, v in data.items()])
    
    @staticmethod
    def validate_data(data, data_type='general'):
        """
        Validate parsed data
        
        Args:
            data: Parsed data dictionary
            data_type: Type of data to validate
        
        Returns:
            True if valid, False otherwise
        """
        if data_type == 'sensor':
            # Check temperature range
            if 'temperature' in data:
                if not (-40 <= data['temperature'] <= 125):
                    return False
            
            # Check humidity range
            if 'humidity' in data:
                if not (0 <= data['humidity'] <= 100):
                    return False
        
        elif data_type == 'motor':
            # Check speed range
            if 'speed' in data:
                if not (0 <= data['speed'] <= 100):
                    return False
        
        return True
