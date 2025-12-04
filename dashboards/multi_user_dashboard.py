"""
Multi-User Project Launcher Dashboard for ATmega128
Educational project selection, building, and deployment interface for classroom use

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)
All rights reserved for educational purposes.
Contact: linkedin.com/in/gnoejh53

Features:
- Multi-student concurrent access
- Isolated student workspaces
- Session management and user tracking
- Teacher monitoring and admin panel
- Network-accessible web interface
- Real-time collaboration monitoring
"""

import sys
import os
import uuid
import shutil
from pathlib import Path
from datetime import datetime, timedelta
import threading
import time
import json

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from flask import Flask, render_template, jsonify, request, session, redirect, url_for
from flask_socketio import SocketIO, emit, join_room, leave_room
import subprocess

# Import common modules
from common.base_dashboard import BaseDashboard
from common.serial_handler import SerialHandler
from common.data_parser import DataParser

class MultiUserDashboard(BaseDashboard):
    """Multi-User Educational Project Launcher Dashboard"""
    
    def __init__(self):
        super().__init__(
            name="Multi-User Project Dashboard",
            port=5001,
            config_file="dashboards/config/multi_user_config.json"
        )
        
        # Workspace paths
        self.workspace_root = Path(__file__).parent.parent
        self.projects_dir = self.workspace_root / "projects"
        self.student_workspaces_dir = self.workspace_root / "student_workspaces"
        self.simulators_dir = self.workspace_root / "tools" / "simulide"
        self.simulide_path = self.simulators_dir / "SimulIDE_1.1.0-SR1_Win64" / "simulide.exe"
        self.circuit_file = self.simulators_dir / "Simulator110.simu"
        self.build_script = self.workspace_root / "tools" / "cli" / "cli-build-project.ps1"
        
        # Multi-user management
        self.active_sessions = {}  # session_id -> student info
        self.student_workspaces = {}  # student_id -> workspace_path
        self.session_timeout = timedelta(hours=8)  # 8-hour session timeout
        
        # Create student workspaces directory
        self.student_workspaces_dir.mkdir(exist_ok=True)
        
        # Teacher monitoring
        self.teacher_sessions = set()
        self.activity_log = []
        
        # Setup routes
        self.setup_routes()
        self.setup_socketio_events()
        
        # Start cleanup thread
        self.cleanup_thread = threading.Thread(target=self.cleanup_inactive_sessions, daemon=True)
        self.cleanup_thread.start()
        
        self.log("Multi-User Dashboard initialized")
    
    def generate_student_id(self, student_name="Anonymous"):
        """Generate unique student ID"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        unique_id = str(uuid.uuid4())[:8]
        return f"{student_name}_{timestamp}_{unique_id}".replace(" ", "_")
    
    def create_student_workspace(self, student_id):
        """Create isolated workspace for student"""
        workspace_path = self.student_workspaces_dir / student_id
        workspace_path.mkdir(exist_ok=True)
        
        # Create student's project copies
        student_projects_dir = workspace_path / "projects"
        if not student_projects_dir.exists():
            shutil.copytree(self.projects_dir, student_projects_dir)
        
        # Create student's output directory
        (workspace_path / "outputs").mkdir(exist_ok=True)
        
        # Create student's logs directory
        (workspace_path / "logs").mkdir(exist_ok=True)
        
        return workspace_path
    
    def get_student_workspace(self, session_id):
        """Get student workspace path from session"""
        if session_id in self.active_sessions:
            student_id = self.active_sessions[session_id]['student_id']
            return self.student_workspaces.get(student_id)
        return None
    
    def log_activity(self, student_id, action, details=""):
        """Log student activity for monitoring"""
        activity = {
            'timestamp': datetime.now().isoformat(),
            'student_id': student_id,
            'action': action,
            'details': details
        }
        self.activity_log.append(activity)
        
        # Keep only last 1000 activities
        if len(self.activity_log) > 1000:
            self.activity_log = self.activity_log[-1000:]
        
        # Emit to teacher sessions
        for teacher_session in self.teacher_sessions:
            self.socketio.emit('activity_update', activity, room=teacher_session)
    
    def cleanup_inactive_sessions(self):
        """Clean up inactive student sessions"""
        while True:
            try:
                current_time = datetime.now()
                expired_sessions = []
                
                for session_id, session_info in self.active_sessions.items():
                    if current_time - session_info['last_activity'] > self.session_timeout:
                        expired_sessions.append(session_id)
                
                for session_id in expired_sessions:
                    self.cleanup_session(session_id)
                
                time.sleep(300)  # Check every 5 minutes
            except Exception as e:
                self.log(f"Cleanup error: {e}")
                time.sleep(60)
    
    def cleanup_session(self, session_id):
        """Clean up a specific session"""
        if session_id in self.active_sessions:
            student_info = self.active_sessions[session_id]
            student_id = student_info['student_id']
            
            self.log(f"Cleaning up session for {student_id}")
            self.log_activity(student_id, "SESSION_EXPIRED", "Session timed out")
            
            # Remove from active sessions
            del self.active_sessions[session_id]
            
            # Keep workspace but mark as inactive
            # (Don't delete - students might want to resume work)
    
    def get_project_description(self, project_dir):
        """Extract project description from README or Main.c comments"""
        # Try README first
        readme_path = project_dir / "README.md"
        if readme_path.exists():
            try:
                content = readme_path.read_text(encoding='utf-8', errors='ignore')
                lines = content.strip().split('\n')
                if lines:
                    return lines[0][:100]  # First line, max 100 chars
            except:
                pass
        
        # Try extracting from Main.c comments
        main_c_path = project_dir / "Main.c"
        if main_c_path.exists():
            try:
                content = main_c_path.read_text(encoding='utf-8', errors='ignore')
                lines = content.split('\n')
                for line in lines[:20]:  # Check first 20 lines
                    line = line.strip()
                    if line.startswith('//') or line.startswith('/*'):
                        desc = line.replace('//', '').replace('/*', '').replace('*/', '').strip()
                        if len(desc) > 10:  # Meaningful description
                            return desc[:100]
            except:
                pass
        
        return f"ATmega128 project - {project_dir.name}"
    
    def get_directory_size(self, directory):
        """Get total size of directory in bytes"""
        try:
            total_size = 0
            for file_path in directory.rglob('*'):
                if file_path.is_file():
                    total_size += file_path.stat().st_size
            return total_size
        except:
            return 0
    
    def save_student_state(self, student_id):
        """Save current student state for persistence"""
        if student_id not in self.student_workspaces:
            return
        
        workspace_path = self.student_workspaces[student_id]
        state_file = workspace_path / "student_state.json"
        
        try:
            state = {
                'student_id': student_id,
                'last_access': datetime.now().isoformat(),
                'current_project': getattr(self, 'current_project', None),
                'open_files': [],
                'session_data': {}
            }
            
            # Save state
            with open(state_file, 'w', encoding='utf-8') as f:
                json.dump(state, f, indent=2)
                
        except Exception as e:
            self.log(f"Error saving student state: {e}")
    
    def load_student_state(self, student_id):
        """Load saved student state for session resumption"""
        if student_id not in self.student_workspaces:
            return None
        
        workspace_path = self.student_workspaces[student_id]
        state_file = workspace_path / "student_state.json"
        
        try:
            if state_file.exists():
                with open(state_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
        except Exception as e:
            self.log(f"Error loading student state: {e}")
        
        return None
    
    def setup_routes(self):
        """Setup Flask routes for multi-user access"""
        
        @self.app.route('/')
        def index():
            """Main entry point - student login"""
            return render_template('multi_user_login.html')
        
        @self.app.route('/teacher')
        def teacher_dashboard():
            """Teacher monitoring dashboard"""
            return render_template('teacher_dashboard.html')
        
        @self.app.route('/student')
        def student_dashboard():
            """Student project launcher"""
            if 'student_id' not in session:
                return redirect(url_for('index'))
            return render_template('student_dashboard.html')
        
        @self.app.route('/api/login', methods=['POST'])
        def login():
            """Student login"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            student_name = data.get('name', 'Anonymous').strip()
            if not student_name:
                student_name = 'Anonymous'
            
            # Generate student session
            student_id = self.generate_student_id(student_name)
            session_id = str(uuid.uuid4())
            
            # Create workspace
            workspace_path = self.create_student_workspace(student_id)
            
            # Register session
            self.active_sessions[session_id] = {
                'student_id': student_id,
                'student_name': student_name,
                'login_time': datetime.now(),
                'last_activity': datetime.now(),
                'workspace_path': workspace_path
            }
            
            self.student_workspaces[student_id] = workspace_path
            
            # Set Flask session
            session['student_id'] = student_id
            session['session_id'] = session_id
            session.permanent = True
            
            self.log_activity(student_id, "LOGIN", f"Student '{student_name}' logged in")
            
            return jsonify({
                'success': True,
                'student_id': student_id,
                'student_name': student_name,
                'workspace_path': str(workspace_path)
            })
        
        @self.app.route('/api/logout', methods=['POST'])
        def logout():
            """Student logout"""
            session_id = session.get('session_id')
            student_id = session.get('student_id')
            
            if session_id and session_id in self.active_sessions:
                self.log_activity(student_id, "LOGOUT", "Student logged out")
                del self.active_sessions[session_id]
            
            session.clear()
            return jsonify({'success': True})
        
        @self.app.route('/api/projects')
        def get_projects():
            """Get list of projects for current student with metadata"""
            session_id = session.get('session_id')
            workspace_path = self.get_student_workspace(session_id)
            
            if not workspace_path:
                return jsonify({'error': 'No active session'})
            
            student_projects_dir = workspace_path / "projects"
            projects = []
            
            if student_projects_dir.exists():
                for project_dir in student_projects_dir.iterdir():
                    if project_dir.is_dir() and (project_dir / "Main.c").exists():
                        # Get project metadata
                        main_c_path = project_dir / "Main.c"
                        last_modified = main_c_path.stat().st_mtime if main_c_path.exists() else 0
                        
                        # Count files
                        file_count = len([f for f in project_dir.iterdir() if f.is_file()])
                        
                        # Try to get description from README or comments
                        description = self.get_project_description(project_dir)
                        
                        projects.append({
                            'name': project_dir.name,
                            'path': str(project_dir),
                            'has_main': True,
                            'last_modified': last_modified,
                            'file_count': file_count,
                            'description': description,
                            'size': self.get_directory_size(project_dir)
                        })
            
            # Sort by last modified
            projects.sort(key=lambda x: x['last_modified'], reverse=True)
            
            # Update last activity
            if session_id in self.active_sessions:
                self.active_sessions[session_id]['last_activity'] = datetime.now()
            
            return jsonify(projects)
        
        @self.app.route('/api/build', methods=['POST'])
        def build_project():
            """Build a project in student's workspace"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            session_id = session.get('session_id')
            workspace_path = self.get_student_workspace(session_id)
            student_id = session.get('student_id')
            
            if not workspace_path:
                return jsonify({'success': False, 'error': 'No active session'})
            
            project_name = data.get('project')
            student_project_path = workspace_path / "projects" / project_name
            
            if not student_project_path.exists():
                return jsonify({'success': False, 'error': 'Project not found'})
            
            # Log activity
            self.log_activity(student_id, "BUILD_START", f"Building project: {project_name}")
            
            # Build project
            success, output = self.build_student_project(student_project_path)
            
            # Log result
            result_msg = "BUILD_SUCCESS" if success else "BUILD_FAILED"
            self.log_activity(student_id, result_msg, f"Project: {project_name}")
            
            # Update last activity
            if session_id in self.active_sessions:
                self.active_sessions[session_id]['last_activity'] = datetime.now()
            
            return jsonify({
                'success': success,
                'output': output,
                'project': project_name
            })

        @self.app.route('/api/simulate', methods=['POST'])
        def simulate_project():
            """Start simulation for a project"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            session_id = session.get('session_id')
            workspace_path = self.get_student_workspace(session_id)
            student_id = session.get('student_id')
            
            if not workspace_path:
                return jsonify({'success': False, 'error': 'No active session'})
            
            project_name = data.get('project')
            student_project_path = workspace_path / "projects" / project_name
            
            if not student_project_path.exists():
                return jsonify({'success': False, 'error': 'Project not found'})
            
            # Check if project is built (has .hex file)
            hex_file = student_project_path / f"{project_name}.hex"
            if not hex_file.exists():
                return jsonify({
                    'success': False, 
                    'error': 'Project not built. Please build the project first.',
                    'needs_build': True
                })
            
            # Log activity
            self.log_activity(student_id, "SIMULATE_START", f"Starting simulation: {project_name}")
            
            # Start simulation
            success, message = self.start_simulation(student_project_path, project_name)
            
            # Log result
            result_msg = "SIMULATE_SUCCESS" if success else "SIMULATE_FAILED"
            self.log_activity(student_id, result_msg, f"Project: {project_name}")
            
            # Update last activity
            if session_id in self.active_sessions:
                self.active_sessions[session_id]['last_activity'] = datetime.now()
            
            return jsonify({
                'success': success,
                'message': message,
                'project': project_name
            })
        
        @self.app.route('/api/file/read', methods=['POST'])
        def read_file():
            """Read a source file from student's project"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            session_id = session.get('session_id')
            workspace_path = self.get_student_workspace(session_id)
            student_id = session.get('student_id')
            
            if not workspace_path:
                return jsonify({'success': False, 'error': 'No active session'})
            
            project_name = data.get('project')
            filename = data.get('filename', 'Main.c')
            
            project_path = workspace_path / "projects" / project_name
            file_path = project_path / filename
            
            if not file_path.exists():
                return jsonify({'success': False, 'error': f'File not found: {filename}'})
            
            try:
                content = file_path.read_text(encoding='utf-8', errors='ignore')
                
                # Log activity
                self.log_activity(student_id, "FILE_READ", f"Opened {filename} in {project_name}")
                
                # Update last activity
                if session_id in self.active_sessions:
                    self.active_sessions[session_id]['last_activity'] = datetime.now()
                
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
            """Save edited source file to student's workspace"""
            data = request.json
            if not data:
                return jsonify({'success': False, 'error': 'Invalid request data'})
            
            session_id = session.get('session_id')
            workspace_path = self.get_student_workspace(session_id)
            student_id = session.get('student_id')
            
            if not workspace_path:
                return jsonify({'success': False, 'error': 'No active session'})
            
            project_name = data.get('project')
            filename = data.get('filename', 'Main.c')
            content = data.get('content', '')
            
            project_path = workspace_path / "projects" / project_name
            file_path = project_path / filename
            
            if not project_path.exists():
                return jsonify({'success': False, 'error': 'Project not found'})
            
            try:
                # Create backup
                backup_dir = workspace_path / "backups" / project_name
                backup_dir.mkdir(parents=True, exist_ok=True)
                backup_file = backup_dir / f"{filename}.{int(time.time())}.bak"
                
                if file_path.exists():
                    shutil.copy2(file_path, backup_file)
                
                # Save new content
                file_path.write_text(content, encoding='utf-8')
                
                # Log activity
                self.log_activity(student_id, "FILE_SAVE", f"Saved {filename} in {project_name}")
                
                # Update last activity and save state
                if session_id in self.active_sessions:
                    self.active_sessions[session_id]['last_activity'] = datetime.now()
                
                self.save_student_state(student_id)
                
                return jsonify({
                    'success': True,
                    'filename': filename,
                    'saved_at': datetime.now().isoformat()
                })
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/teacher/sessions')
        def get_active_sessions():
            """Get active student sessions (teacher only)"""
            sessions = []
            for session_id, session_info in self.active_sessions.items():
                sessions.append({
                    'session_id': session_id,
                    'student_id': session_info['student_id'],
                    'student_name': session_info['student_name'],
                    'login_time': session_info['login_time'].isoformat(),
                    'last_activity': session_info['last_activity'].isoformat()
                })
            return jsonify(sessions)
        
        @self.app.route('/api/teacher/activity')
        def get_activity_log():
            """Get recent activity log (teacher only)"""
            # Return last 100 activities
            recent_activities = self.activity_log[-100:]
            return jsonify(recent_activities)
    
    def build_student_project(self, project_path):
        """Build project in student's isolated workspace"""
        try:
            self.log(f"Building student project: {project_path}")
            
            if not self.build_script.exists():
                return False, f"Build script not found: {self.build_script}"
            
            # Run build in student's project directory
            result = subprocess.run(
                [
                    'powershell.exe',
                    '-ExecutionPolicy', 'Bypass',
                    '-File', str(self.build_script),
                    '-ProjectDir', str(project_path)
                ],
                cwd=str(project_path),
                capture_output=True,
                text=True,
                timeout=60,
                encoding='utf-8',
                errors='replace'
            )
            
            success = result.returncode == 0
            output = f"STDOUT:\n{result.stdout}\n\nSTDERR:\n{result.stderr}"
            
            return success, output
            
        except subprocess.TimeoutExpired:
            return False, "Build timeout (60s exceeded)"
        except Exception as e:
            return False, f"Build error: {str(e)}"
    
    def start_simulation(self, project_path, project_name):
        """Start SimulIDE simulation for a project using existing tools"""
        try:
            self.log(f"Starting simulation for project: {project_path}")
            
            # Check if SimulIDE executable exists
            if not self.simulide_path.exists():
                return False, f"SimulIDE not found: {self.simulide_path}"
            
            # Look for circuit file in project directory
            circuit_files = list(project_path.glob("*.simu"))
            if not circuit_files:
                # Try common circuit file names
                possible_circuits = [
                    project_path / f"{project_name}.simu",
                    project_path / "circuit.simu",
                    project_path / "simulation.simu"
                ]
                circuit_files = [f for f in possible_circuits if f.exists()]
            
            # If no project-specific circuit, use default from tools/simulide
            if not circuit_files:
                default_circuit = self.simulators_dir / "Simulator110.simu"
                if default_circuit.exists():
                    circuit_file = default_circuit
                else:
                    return False, f"No circuit file found. Please add a .simu file to your project or check default circuit at {default_circuit}"
            else:
                circuit_file = circuit_files[0]
            
            # Check if hex file exists (project should be built first)
            hex_file = project_path / f"{project_name}.hex"
            if not hex_file.exists():
                return False, "Project must be built first. .hex file not found."
            
            # Start SimulIDE with the circuit file
            # Run in background so it doesn't block the web interface
            subprocess.Popen(
                [str(self.simulide_path), str(circuit_file)],
                cwd=str(project_path),
                creationflags=subprocess.CREATE_NEW_CONSOLE if hasattr(subprocess, 'CREATE_NEW_CONSOLE') else 0
            )
            
            return True, f"SimulIDE started successfully with circuit: {circuit_file.name}"
            
        except Exception as e:
            return False, f"Simulation error: {str(e)}"
    
    def setup_socketio_events(self):
        """Setup SocketIO events for real-time communication"""
        
        @self.socketio.on('connect')
        def handle_connect():
            student_id = session.get('student_id')
            if student_id:
                join_room(student_id)
                emit('status', {'message': f'Connected as {student_id}'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            student_id = session.get('student_id')
            if student_id:
                leave_room(student_id)
        
        @self.socketio.on('join_teacher')
        def handle_join_teacher():
            # Simple teacher authentication (in production, use proper auth)
            teacher_session = session.get('session_id')  # Use Flask session instead of request.sid
            self.teacher_sessions.add(teacher_session)
            join_room('teachers')
            emit('teacher_joined', {'message': 'Teacher dashboard connected'})
        
        @self.socketio.on('student_message')
        def handle_student_message(data):
            student_id = session.get('student_id')
            if student_id:
                self.log_activity(student_id, "MESSAGE", data.get('message', ''))
                emit('message_received', {'status': 'ok'})


def main():
    """Run the multi-user dashboard"""
    dashboard = MultiUserDashboard()
    
    print("=" * 70)
    print("[DASHBOARD] ATmega128 Multi-User Project Dashboard")
    print("=" * 70)
    print(f"[WORKSPACE] Workspace: {dashboard.workspace_root}")
    print(f"[CONFIG] Projects: {len(list(dashboard.projects_dir.iterdir()))} found")
    print(f"[STUDENTS] Student Workspaces: {dashboard.student_workspaces_dir}")
    print(f"[SIMULIDE] SimulIDE: {dashboard.simulide_path}")
    print("=" * 70)
    print("🌐 Student Access: http://<server-ip>:5001")
    print("🏫 Teacher Dashboard: http://<server-ip>:5001/teacher")
    print("=" * 70)
    print("📊 Features:")
    print("  • Multi-student concurrent access")
    print("  • Isolated student workspaces")
    print("  • Real-time activity monitoring")
    print("  • Session management")
    print("  • Teacher oversight panel")
    print("=" * 70)
    print("Press Ctrl+C to stop")
    print("=" * 70)
    
    # Run dashboard with production settings
    dashboard.socketio.run(
        dashboard.app,
        host='0.0.0.0',  # Accept connections from any IP
        port=5001,
        debug=False,     # Disable debug in multi-user environment
        use_reloader=False
    )


if __name__ == '__main__':
    main()