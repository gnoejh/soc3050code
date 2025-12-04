"""
Base Dashboard Class
All dashboards inherit from this class for common functionality

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)
All rights reserved for educational purposes.
Contact: linkedin.com/in/gnoejh53
"""

from flask import Flask
from flask_socketio import SocketIO
import json
from pathlib import Path


class BaseDashboard:
    """Base class for all ATmega128 dashboards"""
    
    def __init__(self, name="Dashboard", port=5000, config_file=None):
        """
        Initialize base dashboard
        
        Args:
            name: Dashboard name
            port: HTTP port number
            config_file: Optional config file path
        """
        self.name = name
        self.port = port
        self.config = {}
        
        # Load configuration if provided
        if config_file:
            self.load_config(config_file)
        
        # Flask app initialization
        # Use the dashboards directory as the root for templates and static files
        import os
        dashboards_dir = Path(__file__).parent.parent  # Go up to dashboards/ directory
        self.app = Flask(
            name,
            template_folder=str(dashboards_dir / 'templates'),
            static_folder=str(dashboards_dir / 'static')
        )
        self.app.config['SECRET_KEY'] = f'atmega128_{name}_dashboard'
        
        # SocketIO for real-time communication
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Common state
        self.connected = False
        self.device_type = None  # 'simulator' or 'hardware'
        self.com_port = None
        
        print(f"[OK] {self.name} initialized on port {self.port}")
    
    def load_config(self, config_file):
        """Load configuration from JSON file"""
        try:
            config_path = Path(config_file)
            if config_path.exists():
                with open(config_path, 'r') as f:
                    self.config = json.load(f)
                print(f"[CONFIG] Loaded config from {config_file}")
            else:
                print(f"[WARNING] Config file not found: {config_file}")
        except Exception as e:
            print(f"[ERROR] Error loading config: {e}")
    
    def save_config(self, config_file):
        """Save configuration to JSON file"""
        try:
            with open(config_file, 'w') as f:
                json.dump(self.config, f, indent=2)
            print(f"[SAVE] Saved config to {config_file}")
        except Exception as e:
            print(f"[ERROR] Error saving config: {e}")
    
    def get_config(self, key, default=None):
        """Get configuration value"""
        return self.config.get(key, default)
    
    def set_config(self, key, value):
        """Set configuration value"""
        self.config[key] = value
    
    def run(self, host='0.0.0.0', debug=True):
        """Run the dashboard server"""
        print("=" * 60)
        print(f"[START] Starting {self.name}")
        print("=" * 60)
        print(f"[URL] URL: http://localhost:{self.port}")
        print(f"[NETWORK] Network: http://<your-ip>:{self.port}")
        print("=" * 60)
        
        self.socketio.run(self.app, debug=debug, host=host, port=self.port)
    
    def emit_event(self, event_name, data):
        """Emit SocketIO event to all connected clients"""
        self.socketio.emit(event_name, data)
    
    def log(self, message, level="INFO"):
        """Log message with timestamp"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")
