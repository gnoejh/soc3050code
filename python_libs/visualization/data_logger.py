"""
Data Logger for ATmega128 Projects
Logs sensor data to various formats (CSV, JSON, TXT)

© 2025 Prof. Hong Jeong, IUT
"""

import csv
import json
from datetime import datetime
from typing import List, Dict, Any, Optional
import os


class DataLogger:
    """
    Log sensor data to files
    
    Supports:
    - CSV format (for Excel, MATLAB, Python)
    - JSON format (for JavaScript, APIs)
    - TXT format (for simple viewing)
    """
    
    def __init__(self, filename: str, format: str = 'csv', 
                 columns: Optional[List[str]] = None, auto_timestamp: bool = True):
        """
        Initialize data logger
        
        Args:
            filename: Output filename (extension will be added if missing)
            format: File format ('csv', 'json', 'txt')
            columns: Column names for CSV
            auto_timestamp: Automatically add timestamp column
        """
        self.format = format.lower()
        self.auto_timestamp = auto_timestamp
        self.columns = columns or []
        
        # Add timestamp column if requested
        if auto_timestamp and 'timestamp' not in self.columns:
            self.columns = ['timestamp'] + self.columns
        
        # Ensure correct extension
        base_name = os.path.splitext(filename)[0]
        self.filename = f"{base_name}.{self.format}"
        
        # Create directory if needed
        directory = os.path.dirname(self.filename)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
        
        # Initialize file
        self.file = None
        self.csv_writer = None
        self.json_data = []
        
        self._initialize_file()
    
    def _initialize_file(self):
        """Initialize file based on format"""
        if self.format == 'csv':
            self.file = open(self.filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.DictWriter(self.file, fieldnames=self.columns)
            self.csv_writer.writeheader()
            self.file.flush()
        
        elif self.format == 'json':
            self.json_data = []
        
        elif self.format == 'txt':
            self.file = open(self.filename, 'w', encoding='utf-8')
            # Write header
            if self.columns:
                header = '\t'.join(self.columns)
                self.file.write(f"{header}\n")
                self.file.write("=" * 80 + "\n")
                self.file.flush()
        
        else:
            raise ValueError(f"Unsupported format: {self.format}")
        
        print(f"📝 Logging to {self.filename} ({self.format} format)")
    
    def log(self, data: Dict[str, Any]):
        """
        Log data entry
        
        Args:
            data: Dictionary of data to log
        """
        # Add timestamp if requested
        if self.auto_timestamp and 'timestamp' not in data:
            data['timestamp'] = datetime.now().isoformat()
        
        # Write based on format
        if self.format == 'csv':
            self.csv_writer.writerow(data)
            self.file.flush()
        
        elif self.format == 'json':
            self.json_data.append(data)
            # Write entire JSON file (inefficient but simple)
            with open(self.filename, 'w', encoding='utf-8') as f:
                json.dump(self.json_data, f, indent=2)
        
        elif self.format == 'txt':
            # Write values in column order
            values = [str(data.get(col, '')) for col in self.columns]
            line = '\t'.join(values)
            self.file.write(f"{line}\n")
            self.file.flush()
    
    def log_values(self, *values):
        """
        Log values in column order
        
        Args:
            *values: Values corresponding to columns
        """
        if len(values) != len(self.columns) - (1 if self.auto_timestamp else 0):
            raise ValueError(f"Expected {len(self.columns)} values")
        
        # Create data dictionary
        start_idx = 1 if self.auto_timestamp else 0
        data = {self.columns[i + start_idx]: values[i] for i in range(len(values))}
        
        self.log(data)
    
    def close(self):
        """Close log file"""
        if self.file:
            self.file.close()
            print(f"✅ Log saved to {self.filename}")
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()
        return False


# Example usage
if __name__ == '__main__':
    # CSV logging example
    print("Testing CSV logger...")
    with DataLogger('test_data', format='csv', 
                   columns=['sensor1', 'sensor2', 'sensor3']) as logger:
        for i in range(10):
            logger.log({
                'sensor1': i * 10,
                'sensor2': i * 20,
                'sensor3': i * 30
            })
    
    # JSON logging example
    print("\nTesting JSON logger...")
    with DataLogger('test_data', format='json',
                   columns=['temperature', 'humidity']) as logger:
        for i in range(5):
            logger.log({
                'temperature': 20 + i,
                'humidity': 50 + i * 2
            })
    
    print("\n✅ Test complete - check test_data.csv and test_data.json")

