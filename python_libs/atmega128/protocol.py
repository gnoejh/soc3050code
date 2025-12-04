"""
ATmega128 Communication Protocol
Standard command protocol for ATmega128 projects

© 2025 Prof. Hong Jeong, IUT
"""

from typing import Dict, Any, Optional
import re


class ATmega128Protocol:
    """
    Standard communication protocol for ATmega128
    
    Command Format:
        COMMAND [PARAM1] [PARAM2] ... \n
    
    Response Format:
        STATUS:DATA\n
        OK:value
        ERROR:message
    """
    
    # Standard commands
    COMMANDS = {
        # System commands
        'PING': 'Check connection',
        'RESET': 'Reset microcontroller',
        'VERSION': 'Get firmware version',
        'STATUS': 'Get system status',
        
        # GPIO commands
        'LED_ON': 'Turn LED on (param: led_number)',
        'LED_OFF': 'Turn LED off (param: led_number)',
        'LED_TOGGLE': 'Toggle LED (param: led_number)',
        'LED_PATTERN': 'Set LED pattern (param: pattern_code)',
        'READ_BUTTON': 'Read button state',
        
        # ADC commands
        'READ_ADC': 'Read ADC channel (param: channel)',
        'READ_TEMP': 'Read temperature sensor',
        'READ_LIGHT': 'Read light sensor',
        
        # Sensor commands
        'READ_ACCEL': 'Read accelerometer (X,Y,Z)',
        'READ_GYRO': 'Read gyroscope (X,Y,Z)',
        'READ_MPU6050': 'Read all MPU6050 data',
        'READ_ENV': 'Read environmental sensors',
        
        # Motor commands
        'MOTOR_SPEED': 'Set motor speed (param: 0-255)',
        'MOTOR_STOP': 'Stop motor',
        'SERVO_ANGLE': 'Set servo angle (param: 0-180)',
        'STEPPER_STEP': 'Step stepper motor (param: steps)',
        
        # Display commands
        'LCD_PRINT': 'Print to LCD (param: text)',
        'LCD_CLEAR': 'Clear LCD',
        'GLCD_PIXEL': 'Draw pixel (params: x,y,color)',
        
        # Communication commands
        'ECHO': 'Echo back received data',
        'DATA_STREAM_START': 'Start continuous data streaming',
        'DATA_STREAM_STOP': 'Stop data streaming',
    }
    
    @staticmethod
    def build_command(cmd: str, *params) -> str:
        """
        Build command string
        
        Args:
            cmd: Command name
            *params: Command parameters
        
        Returns:
            Formatted command string
        """
        if params:
            return f"{cmd} {' '.join(map(str, params))}\n"
        return f"{cmd}\n"
    
    @staticmethod
    def parse_response(response: str) -> Dict[str, Any]:
        """
        Parse response from ATmega128
        
        Args:
            response: Response string
        
        Returns:
            Dictionary with 'status' and 'data' keys
        """
        response = response.strip()
        
        # Try standard format: STATUS:DATA
        if ':' in response:
            parts = response.split(':', 1)
            return {
                'status': parts[0].upper(),
                'data': parts[1] if len(parts) > 1 else '',
                'raw': response
            }
        
        # Simple response (no status prefix)
        return {
            'status': 'OK',
            'data': response,
            'raw': response
        }
    
    @staticmethod
    def parse_sensor_data(data: str) -> Dict[str, float]:
        """
        Parse sensor data in format: "KEY1:VALUE1,KEY2:VALUE2,..."
        
        Args:
            data: Sensor data string
        
        Returns:
            Dictionary of sensor values
        """
        result = {}
        
        # Split by comma
        parts = data.split(',')
        
        for part in parts:
            part = part.strip()
            if ':' in part:
                key, value = part.split(':', 1)
                try:
                    # Try to convert to number
                    if '.' in value:
                        result[key.strip()] = float(value)
                    else:
                        result[key.strip()] = int(value)
                except ValueError:
                    # Keep as string if conversion fails
                    result[key.strip()] = value.strip()
        
        return result
    
    @staticmethod
    def parse_adc_value(response: str) -> Optional[int]:
        """
        Parse ADC response
        
        Args:
            response: ADC response string (e.g., "ADC:512" or "512")
        
        Returns:
            ADC value as integer, or None if parsing fails
        """
        try:
            # Try direct integer
            if response.isdigit():
                return int(response)
            
            # Try KEY:VALUE format
            if ':' in response:
                return int(response.split(':')[1])
            
            # Try extracting first number
            match = re.search(r'\d+', response)
            if match:
                return int(match.group())
            
        except (ValueError, IndexError):
            pass
        
        return None
    
    @staticmethod
    def is_success(response: Dict[str, Any]) -> bool:
        """
        Check if response indicates success
        
        Args:
            response: Parsed response dictionary
        
        Returns:
            True if successful
        """
        return response['status'] in ['OK', 'SUCCESS', 'READY']
    
    @staticmethod
    def get_error_message(response: Dict[str, Any]) -> Optional[str]:
        """
        Get error message from response
        
        Args:
            response: Parsed response dictionary
        
        Returns:
            Error message if present, None otherwise
        """
        if response['status'] == 'ERROR':
            return response['data']
        return None


# Example usage
if __name__ == '__main__':
    protocol = ATmega128Protocol()
    
    # Build commands
    print("Building commands:")
    print(protocol.build_command('LED_ON', 3))
    print(protocol.build_command('READ_ADC', 0))
    print(protocol.build_command('MOTOR_SPEED', 128))
    
    # Parse responses
    print("\nParsing responses:")
    responses = [
        "OK:Connected",
        "ADC:512",
        "ERROR:Invalid command",
        "X:123,Y:456,Z:789"
    ]
    
    for resp in responses:
        parsed = protocol.parse_response(resp)
        print(f"{resp} -> {parsed}")
    
    # Parse sensor data
    print("\nParsing sensor data:")
    sensor_data = "TEMP:25.5,HUM:60,PRESS:1013"
    parsed = protocol.parse_sensor_data(sensor_data)
    print(f"{sensor_data} -> {parsed}")

