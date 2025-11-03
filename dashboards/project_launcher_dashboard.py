"""
Project Launcher Dashboard for ATmega128
Educational project selection, building, and deployment interface

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)
All rights reserved for educational purposes.
Contact: linkedin.com/in/gnoejh53

Features:
- Browse all 35+ educational projects
- Select demo functions from Main.c
- Build and deploy to SimulIDE or physical hardware
- Real-time monitoring of LED states, buttons, serial output
- Visual debugging interface
"""

import sys
import os
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import subprocess
import json
import re
import threading
import time
from datetime import datetime

# Import common modules
from common.base_dashboard import BaseDashboard
from common.serial_handler import SerialHandler
from common.data_parser import DataParser

class ProjectLauncherDashboard(BaseDashboard):
    """Educational Project Launcher Dashboard"""
    
    def __init__(self):
        super().__init__(
            name="Project Launcher Dashboard",
            port=5001,
            config_file="dashboards/config/project_launcher_config.json"
        )
        
        # Workspace paths
        self.workspace_root = Path(__file__).parent.parent
        self.projects_dir = self.workspace_root / "projects"
        self.simulators_dir = self.workspace_root / "tools" / "simulide"
        self.simulide_path = self.simulators_dir / "SimulIDE_1.1.0-SR1_Win64" / "simulide.exe"
        self.circuit_file = self.simulators_dir / "Simulator110.simu"
        self.build_script = self.workspace_root / "tools" / "cli" / "cli-build-project.ps1"
        
        # Serial handler
        self.serial_handler = SerialHandler()
        self.serial_handler.register_callback(self.on_serial_data)
        
        # Data parser
        self.parser = DataParser()
        
        # Current state
        self.current_project = None
        self.current_demo = None
        self.simulide_process = None
        
        # Visual state
        self.led_states = [False] * 8
        self.button_states = [False] * 8
        self.adc_values = {}
        
        # Setup routes
        self.setup_routes()
        
        self.log("Project Launcher Dashboard initialized")
    
    def setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            return render_template('project_launcher.html')
        
        @self.app.route('/api/projects')
        def get_projects():
            """Get list of all projects"""
            projects = self.scan_projects()
            return jsonify(projects)
        
        @self.app.route('/api/device/scan')
        def scan_devices():
            """Scan for available devices"""
            ports = self.serial_handler.scan_ports()
            detected_port, detected_type = self.serial_handler.auto_detect_device()
            
            return jsonify({
                'ports': ports,
                'detected_port': detected_port,
                'detected_type': detected_type
            })
        
        @self.app.route('/api/device/connect', methods=['POST'])
        def connect_device():
            """Connect to a device"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            port = data.get('port')
            baudrate = data.get('baudrate', 9600)
            
            success = self.serial_handler.connect(port, baudrate)
            
            if success:
                self.serial_handler.start_monitoring()
                return jsonify({'success': True, 'port': port})
            else:
                return jsonify({'success': False, 'error': 'Connection failed'})
        
        @self.app.route('/api/build', methods=['POST'])
        def build_project():
            """Build a project"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            project_name = data.get('project')
            
            success, output = self.build_project(project_name)
            
            return jsonify({
                'success': success,
                'output': output,
                'project': project_name
            })
        
        @self.app.route('/api/run', methods=['POST'])
        def run_project():
            """Build and run a project"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            project_name = data.get('project')
            device_type = data.get('device', 'simulator')  # 'simulator' or 'hardware'
            
            # Build first
            success, build_output = self.build_project(project_name)
            if not success:
                return jsonify({
                    'success': False,
                    'message': 'Build failed',
                    'output': build_output
                })
            
            # Launch on device
            if device_type == 'simulator':
                success, message = self.launch_simulide(project_name)
            else:
                # For hardware: program the chip first
                port = data.get('port', 'COM3')
                programmer = data.get('programmer', 'arduino')
                
                success, prog_output = self.program_hardware(project_name, port, programmer)
                
                if success:
                    # Connect to serial after programming
                    time.sleep(2)  # Give hardware time to reset after programming
                    
                    # Ensure monitoring is started
                    if not self.serial_handler.connected:
                        detected_port, dev_type = self.serial_handler.auto_detect_device()
                        if detected_port:
                            self.serial_handler.connect(detected_port)
                    
                    # Always start monitoring (restart if needed)
                    if self.serial_handler.connected:
                        self.serial_handler.start_monitoring()
                        self.log(f"Serial monitoring started on {self.serial_handler.port}")
                    
                    message = f"[OK] Programmed and connected on {port}\n{prog_output}"
                else:
                    message = prog_output
            
            return jsonify({
                'success': success,
                'message': message,
                'build_output': build_output
            })
        
        @self.app.route('/api/command', methods=['POST'])
        def send_command():
            """Send command to ATmega128"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            command = data.get('command', '')
            
            success = self.serial_handler.send_command(command)
            
            return jsonify({'success': success, 'command': command})
        
        @self.app.route('/api/file/read', methods=['POST'])
        def read_file():
            """Read a source file from project"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            project_name = data.get('project')
            filename = data.get('filename', 'Main.c')
            
            project_path = self.projects_dir / project_name
            file_path = project_path / filename
            
            if not file_path.exists():
                return jsonify({'success': False, 'error': f'File not found: {filename}'})
            
            try:
                content = file_path.read_text(encoding='utf-8', errors='ignore')
                return jsonify({
                    'success': True,
                    'content': content,
                    'filename': filename,
                    'path': str(file_path)
                })
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/file/save', methods=['POST'])
        def save_file():
            """Save edited source file"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            project_name = data.get('project')
            filename = data.get('filename', 'Main.c')
            content = data.get('content', '')
            
            project_path = self.projects_dir / project_name
            file_path = project_path / filename
            
            if not project_path.exists():
                return jsonify({'success': False, 'error': 'Project not found'})
            
            try:
                # Create backup
                backup_path = file_path.with_suffix(file_path.suffix + '.backup')
                if file_path.exists():
                    import shutil
                    shutil.copy2(file_path, backup_path)
                
                # Save new content
                file_path.write_text(content, encoding='utf-8')
                
                return jsonify({
                    'success': True,
                    'message': 'File saved successfully',
                    'backup': str(backup_path)
                })
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/file/list', methods=['POST'])
        def list_files():
            """List editable files in a project"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            project_name = data.get('project')
            
            project_path = self.projects_dir / project_name
            
            if not project_path.exists():
                return jsonify({'success': False, 'error': 'Project not found'})
            
            files = []
            for ext in ['.c', '.h', '.s']:
                for file_path in project_path.glob(f'*{ext}'):
                    files.append({
                        'name': file_path.name,
                        'size': file_path.stat().st_size,
                        'ext': ext
                    })
            
            return jsonify({
                'success': True,
                'files': sorted(files, key=lambda x: (x['ext'], x['name']))
            })
        
        @self.socketio.on('connect')
        def handle_connect():
            emit('status', {'connected': True, 'message': 'Connected to Project Launcher'})
            self.log('Client connected')
            
            # Send test message to verify WebSocket works
            self.socketio.emit('serial_data', {
                'timestamp': datetime.now().isoformat(),
                'data': '🔌 WebSocket connected - Serial monitor ready'
            })
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.log('Client disconnected')
    
    def scan_projects(self):
        """Scan all projects and extract information"""
        projects = []
        
        if not self.projects_dir.exists():
            self.log("Projects directory not found", "WARNING")
            return projects
        
        for project_dir in sorted(self.projects_dir.iterdir()):
            if not project_dir.is_dir():
                continue
            
            main_c = project_dir / "Main.c"
            if not main_c.exists():
                continue
            
            # Parse project information
            demos = self.extract_demos(main_c)
            description = self.extract_description(main_c)
            objectives = self.extract_objectives(main_c)
            
            # Check for Lab.c and extract lab exercises
            lab_c = project_dir / "Lab.c"
            lab_exercises = []
            if lab_c.exists():
                lab_exercises = self.extract_demos(lab_c)
            
            project_info = {
                'name': project_dir.name,
                'path': str(project_dir),
                'demos': demos,
                'lab_exercises': lab_exercises,
                'description': description,
                'objectives': objectives,
                'has_lab': lab_c.exists(),
                'has_hex': (project_dir / "Main.hex").exists()
            }
            
            projects.append(project_info)
        
        self.log(f"Found {len(projects)} projects")
        return projects
    
    def extract_demos(self, main_c_path):
        """Extract demo function names from Main.c and check which is active"""
        demos = []
        
        try:
            content = main_c_path.read_text(encoding='utf-8', errors='ignore')
            
            # Find all function definitions: void functionName(void)
            # Look for demo_XX_ pattern or any void function
            pattern = r'void\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*\(\s*(?:void)?\s*\)'
            matches = re.findall(pattern, content)
            
            # Find which function is called in main() (uncommented)
            main_pattern = r'int\s+main\s*\(.*?\)\s*\{(.*?)\n\s*while'
            main_match = re.search(main_pattern, content, re.DOTALL)
            active_function = None
            
            if main_match:
                main_body = main_match.group(1)
                # Look for uncommented function call
                call_pattern = r'^\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*\(\s*\)\s*;'
                for line in main_body.split('\n'):
                    if not line.strip().startswith('//'):
                        call_match = re.search(call_pattern, line)
                        if call_match:
                            active_function = call_match.group(1)
                            break
            
            # Filter and format demo functions
            for func_name in matches:
                # Skip main, _init, _delay, etc.
                if func_name in ['main', '_delay_ms', '_delay_us', 'init', 'setup']:
                    continue
                
                # Create readable name
                readable_name = func_name.replace('demo_', 'Demo ').replace('_', ' ').title()
                
                demos.append({
                    'function': func_name,
                    'name': readable_name,
                    'active': (func_name == active_function)
                })
            
        except Exception as e:
            self.log(f"Error extracting demos: {e}", "ERROR")
        
        return demos
    
    def extract_description(self, main_c_path):
        """Extract project description from comments"""
        try:
            content = main_c_path.read_text(encoding='utf-8', errors='ignore')
            
            # Look for title line with === markers
            title_pattern = r'/\*[*\s]*\n\s*\*\s*={3,}\s*\n\s*\*\s*([^\n]+)\n'
            title_match = re.search(title_pattern, content)
            
            if title_match:
                title = title_match.group(1).strip()
                
                # Look for FOCUS line
                focus_pattern = r'\*\s*FOCUS:\s*([^\n]+)'
                focus_match = re.search(focus_pattern, content)
                
                if focus_match:
                    focus = focus_match.group(1).strip()
                    return f"{title} - {focus}"
                else:
                    return title
            
            # Fallback: Look for any comment block at the top
            comment_pattern = r'/\*\*?\s*(.*?)\*/'
            match = re.search(comment_pattern, content, re.DOTALL)
            
            if match:
                desc = match.group(1).strip()
                # Extract first meaningful line
                lines = [line.strip().lstrip('*').strip() for line in desc.split('\n') if line.strip()]
                for line in lines:
                    if len(line) > 10 and not line.startswith('='):
                        return line[:150]
                        
        except Exception as e:
            self.log(f"Error extracting description: {e}", "ERROR")
        
        return "ATmega128 educational project"
    
    def extract_objectives(self, main_c_path):
        """Extract learning objectives"""
        objectives = []
        
        try:
            content = main_c_path.read_text(encoding='utf-8', errors='ignore')
            
            # Look for learning objectives
            obj_pattern = r'LEARNING OBJECTIVES?:?\s*(.*?)(?:\*\/|\n\n)'
            match = re.search(obj_pattern, content, re.DOTALL | re.IGNORECASE)
            
            if match:
                obj_text = match.group(1)
                # Extract numbered points
                points = re.findall(r'(?:\d+\.|\*)\s*([^\n]+)', obj_text)
                objectives = [p.strip() for p in points if p.strip()][:5]  # Max 5
        except Exception as e:
            self.log(f"Error extracting objectives: {e}", "ERROR")
        
        return objectives
    
    def build_project(self, project_name):
        """Build a project using PowerShell script"""
        project_path = self.projects_dir / project_name
        
        if not project_path.exists():
            return False, f"Project not found: {project_name}"
        
        if not self.build_script.exists():
            return False, f"Build script not found: {self.build_script}"
        
        try:
            self.log(f"Building project: {project_name}")
            
            # Run PowerShell build script
            result = subprocess.run(
                [
                    'powershell.exe',
                    '-ExecutionPolicy', 'Bypass',
                    '-File', str(self.build_script),
                    '-ProjectDir', str(project_path)
                ],
                capture_output=True,
                text=True,
                timeout=60
            )
            
            output = result.stdout + result.stderr
            
            # Check for success
            success = result.returncode == 0 or 'Main.hex' in output or (project_path / "Main.hex").exists()
            
            if success:
                self.log(f"Build successful: {project_name}")
            else:
                self.log(f"Build failed: {project_name}", "ERROR")
            
            return success, output
            
        except subprocess.TimeoutExpired:
            return False, "Build timeout (60s exceeded)"
        except Exception as e:
            return False, f"Build error: {str(e)}"
    
    def program_hardware(self, project_name, port='COM3', programmer='arduino'):
        """Program the ATmega128 hardware using avrdude"""
        project_path = self.projects_dir / project_name
        hex_file = project_path / "Main.hex"
        
        if not hex_file.exists():
            return False, "HEX file not found. Build first!"
        
        try:
            self.log(f"Programming {project_name} to hardware on {port}")
            
            # avrdude path (prioritize local tools, then commonly installed locations)
            avrdude_paths = [
                str(self.workspace_root / "tools" / "avrdudess" / "avrdude.exe"),
                r"C:\Program Files (x86)\AVRDUDESS\avrdude.exe",
                r"C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude.exe",
                r"C:\avrdude\avrdude.exe"
            ]
            
            avrdude_exe = None
            for path in avrdude_paths:
                if Path(path).exists():
                    avrdude_exe = path
                    break
            
            if not avrdude_exe:
                return False, "avrdude not found. Please install Arduino IDE or AVRDUDESS."
            
            # Build avrdude command
            # For ATmega128: -p m128, -b 115200 for arduino bootloader
            # Don't quote the path - subprocess handles it correctly
            hex_path = str(hex_file.absolute())
            flash_param = f'flash:w:{hex_path}:a'
            
            cmd = [
                avrdude_exe,
                '-c', programmer,  # arduino, usbasp, etc.
                '-p', 'm128',      # ATmega128
                '-P', port,        # COM port
                '-b', '115200',    # Baud rate for arduino bootloader
                '-U', flash_param  # Flash operation
            ]
            
            self.log(f"Running: {' '.join(cmd)}")
            
            # Run avrdude
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30
            )
            
            output = result.stdout + result.stderr
            
            # Check for success (avrdude returns 0 on success)
            success = result.returncode == 0 or 'bytes of flash verified' in output.lower()
            
            if success:
                self.log(f"Programming successful: {project_name}")
                return True, f"[OK] Programmed to hardware on {port}\n{output}"
            else:
                self.log(f"Programming failed: {output}", "ERROR")
                return False, f"[ERROR] Programming failed:\n{output}"
            
        except subprocess.TimeoutExpired:
            return False, "Programming timeout (30s exceeded)"
        except Exception as e:
            self.log(f"Programming error: {e}", "ERROR")
            return False, f"Programming error: {str(e)}"
    
    def launch_simulide(self, project_name):
        """Launch SimulIDE with the project"""
        project_path = self.projects_dir / project_name
        hex_file = project_path / "Main.hex"
        
        if not hex_file.exists():
            return False, "HEX file not found. Build first!"
        
        if not self.simulide_path.exists():
            return False, f"SimulIDE not found at {self.simulide_path}"
        
        try:
            # Update circuit file with HEX path
            self.update_circuit_hex(hex_file)
            
            # Close existing SimulIDE if running
            if self.simulide_process and self.simulide_process.poll() is None:
                self.simulide_process.terminate()
                time.sleep(1)
            
            # Launch SimulIDE
            self.log(f"Launching SimulIDE with {project_name}")
            self.simulide_process = subprocess.Popen(
                [str(self.simulide_path), str(self.circuit_file)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            # Wait for SimulIDE to start and create virtual port
            time.sleep(3)
            
            # Try to connect to virtual port
            port, dev_type = self.serial_handler.auto_detect_device()
            if port and dev_type == 'simulator':
                self.serial_handler.connect(port)
                self.serial_handler.start_monitoring()
                return True, f"SimulIDE launched and connected on {port}"
            else:
                return True, "SimulIDE launched (waiting for virtual COM port...)"
            
        except Exception as e:
            self.log(f"SimulIDE launch error: {e}", "ERROR")
            return False, f"Error: {str(e)}"
    
    def update_circuit_hex(self, hex_file_path):
        """Update Simulator110.simu with new HEX file path"""
        try:
            content = self.circuit_file.read_text(encoding='utf-8')
            
            # Replace Program path (use forward slashes for SimulIDE)
            hex_path_abs = str(hex_file_path.absolute()).replace('\\', '/')
            content = re.sub(
                r'Program="[^"]*"',
                f'Program="{hex_path_abs}"',
                content
            )
            
            # Enable Auto_Load
            content = re.sub(
                r'Auto_Load="false"',
                'Auto_Load="true"',
                content
            )
            
            self.circuit_file.write_text(content, encoding='utf-8')
            self.log(f"Updated circuit file with: {hex_file_path.name}")
            return True
            
        except Exception as e:
            self.log(f"Circuit update error: {e}", "ERROR")
            return False
    
    def on_serial_data(self, data_entry):
        """Callback for received serial data"""
        data = data_entry['data']
        timestamp = data_entry['timestamp']
        
        # Debug logging
        self.log(f"Serial RX: {data}")
        
        # Parse LED states
        if 'PORTB' in data or 'LED' in data:
            led_states = self.parser.parse_led_state(data)
            if any(led_states):  # If any LED changed
                self.led_states = led_states
                self.emit_event('led_update', {'states': self.led_states})
        
        # Parse ADC values
        if 'ADC' in data:
            adc_info = self.parser.parse_adc_value(data)
            if adc_info:
                self.adc_values[adc_info.get('channel', 0)] = adc_info
                self.emit_event('adc_update', adc_info)
        
        # Parse sensor data
        sensor_data = self.parser.parse_sensor_data(data)
        if sensor_data:
            self.emit_event('sensor_update', sensor_data)
        
        # Send all data to frontend
        self.emit_event('serial_data', {
            'timestamp': timestamp,
            'data': data
        })
        self.log(f"Emitted serial_data event to frontend")

# Create dashboard instance
dashboard = ProjectLauncherDashboard()

if __name__ == '__main__':
    print("=" * 70)
    print("[DASHBOARD] ATmega128 Project Launcher Dashboard")
    print("=" * 70)
    print(f"[WORKSPACE] Workspace: {dashboard.workspace_root}")
    print(f"[CONFIG] Projects: {len(list(dashboard.projects_dir.iterdir()))} found")
    print(f"[SIMULIDE] SimulIDE: {dashboard.simulide_path}")
    print("=" * 70)
    print("[URL] Dashboard URL: http://localhost:5001")
    print("[NETWORK] Network access: http://<your-ip>:5001")
    print("=" * 70)
    print("Press Ctrl+C to stop")
    print("=" * 70)
    
    # Run dashboard
    dashboard.run(host='0.0.0.0', debug=True)
