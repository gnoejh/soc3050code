# Web Dashboard for ATmega128 using Flask
# Real-time web interface for microcontroller data

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import serial
import json
import threading
import time
from datetime import datetime

app = Flask(__name__)
app.config['SECRET_KEY'] = 'atmega128_dashboard'
socketio = SocketIO(app, cors_allowed_origins="*")

class ATmegaWebInterface:
    def __init__(self, port='COM3', baudrate=9600):
        self.serial_port = port
        self.baudrate = baudrate
        self.serial = None
        self.data_buffer = []
        self.max_buffer_size = 100
        self.running = False
        
        self.connect()
    
    def connect(self):
        """Connect to ATmega128"""
        try:
            self.serial = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            print(f"✅ Connected to ATmega128 on {self.serial_port}")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def send_command(self, command):
        """Send command to ATmega128"""
        if self.serial:
            try:
                self.serial.write(f"{command}\n".encode())
                return True
            except Exception as e:
                print(f"❌ Send error: {e}")
        return False
    
    def read_data(self):
        """Read data from ATmega128"""
        if self.serial and self.serial.in_waiting:
            try:
                data = self.serial.readline().decode().strip()
                if data:
                    entry = {
                        'timestamp': datetime.now().isoformat(),
                        'data': data,
                        'parsed': self.parse_data(data)
                    }
                    
                    # Add to buffer
                    self.data_buffer.append(entry)
                    if len(self.data_buffer) > self.max_buffer_size:
                        self.data_buffer.pop(0)
                    
                    # Emit to web clients
                    socketio.emit('new_data', entry)
                    return entry
            except Exception as e:
                print(f"❌ Read error: {e}")
        return None
    
    def parse_data(self, data):
        """Parse sensor data"""
        try:
            # Example: "TEMP:25.6,HUMID:45.2,LIGHT:234"
            if ':' in data:
                pairs = data.split(',')
                parsed = {}
                for pair in pairs:
                    if ':' in pair:
                        key, value = pair.split(':')
                        try:
                            parsed[key] = float(value)
                        except:
                            parsed[key] = value
                return parsed
        except:
            pass
        return {'message': data}
    
    def start_monitoring(self):
        """Start monitoring in background thread"""
        self.running = True
        def monitor_loop():
            while self.running:
                self.read_data()
                time.sleep(0.1)
        
        thread = threading.Thread(target=monitor_loop)
        thread.daemon = True
        thread.start()

# Global ATmega interface
atmega = ATmegaWebInterface()

@app.route('/')
def dashboard():
    """Main dashboard page"""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>ATmega128 Dashboard</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
        <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
            .container { max-width: 1200px; margin: 0 auto; }
            .header { background: #2c3e50; color: white; padding: 20px; border-radius: 10px; margin-bottom: 20px; }
            .controls { background: white; padding: 20px; border-radius: 10px; margin-bottom: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
            .data-display { background: white; padding: 20px; border-radius: 10px; margin-bottom: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
            .sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-bottom: 20px; }
            .sensor-card { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; border-radius: 10px; text-align: center; }
            .sensor-value { font-size: 2em; font-weight: bold; }
            .sensor-label { font-size: 0.9em; opacity: 0.8; }
            button { background: #3498db; color: white; border: none; padding: 10px 20px; border-radius: 5px; margin: 5px; cursor: pointer; }
            button:hover { background: #2980b9; }
            .log { background: #2c3e50; color: #00ff00; padding: 15px; border-radius: 5px; height: 200px; overflow-y: scroll; font-family: monospace; }
            .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
            .connected { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
            .disconnected { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>🔧 ATmega128 Real-time Dashboard</h1>
                <p>Monitor and control your microcontroller from VS Code</p>
            </div>
            
            <div id="status" class="status disconnected">
                🔴 Disconnected from ATmega128
            </div>
            
            <div class="controls">
                <h3>🎮 Control Panel</h3>
                <button onclick="sendCommand('LED_ON')">💡 LED ON</button>
                <button onclick="sendCommand('LED_OFF')">🌑 LED OFF</button>
                <button onclick="sendCommand('READ_SENSORS')">📊 Read Sensors</button>
                <button onclick="sendCommand('RESET')">🔄 Reset</button>
                <input type="text" id="customCommand" placeholder="Custom command..." style="padding: 10px; margin: 5px;">
                <button onclick="sendCustomCommand()">📤 Send</button>
            </div>
            
            <div class="sensor-grid">
                <div class="sensor-card">
                    <div class="sensor-value" id="temp-value">--</div>
                    <div class="sensor-label">🌡️ Temperature (°C)</div>
                </div>
                <div class="sensor-card">
                    <div class="sensor-value" id="humid-value">--</div>
                    <div class="sensor-label">💧 Humidity (%)</div>
                </div>
                <div class="sensor-card">
                    <div class="sensor-value" id="light-value">--</div>
                    <div class="sensor-label">☀️ Light Level</div>
                </div>
                <div class="sensor-card">
                    <div class="sensor-value" id="voltage-value">--</div>
                    <div class="sensor-label">⚡ Voltage (V)</div>
                </div>
            </div>
            
            <div class="data-display">
                <h3>📈 Real-time Chart</h3>
                <canvas id="sensorChart" width="400" height="100"></canvas>
            </div>
            
            <div class="data-display">
                <h3>📝 Data Log</h3>
                <div id="dataLog" class="log"></div>
            </div>
        </div>

        <script>
            const socket = io();
            let chart;
            let chartData = {
                labels: [],
                datasets: [{
                    label: 'Temperature',
                    data: [],
                    borderColor: 'rgb(255, 99, 132)',
                    tension: 0.1
                }, {
                    label: 'Humidity', 
                    data: [],
                    borderColor: 'rgb(54, 162, 235)',
                    tension: 0.1
                }]
            };

            // Initialize chart
            const ctx = document.getElementById('sensorChart').getContext('2d');
            chart = new Chart(ctx, {
                type: 'line',
                data: chartData,
                options: {
                    responsive: true,
                    animation: false,
                    scales: {
                        x: { display: false }
                    }
                }
            });

            // Socket events
            socket.on('connect', function() {
                document.getElementById('status').innerHTML = '🟢 Connected to Dashboard';
                document.getElementById('status').className = 'status connected';
            });

            socket.on('new_data', function(data) {
                updateDisplay(data);
                logData(data);
                updateChart(data);
            });

            function updateDisplay(data) {
                if (data.parsed) {
                    if (data.parsed.TEMP) {
                        document.getElementById('temp-value').textContent = data.parsed.TEMP.toFixed(1);
                    }
                    if (data.parsed.HUMID) {
                        document.getElementById('humid-value').textContent = data.parsed.HUMID.toFixed(1);
                    }
                    if (data.parsed.LIGHT) {
                        document.getElementById('light-value').textContent = data.parsed.LIGHT;
                    }
                    if (data.parsed.VOLTAGE) {
                        document.getElementById('voltage-value').textContent = data.parsed.VOLTAGE.toFixed(2);
                    }
                }
            }

            function logData(data) {
                const log = document.getElementById('dataLog');
                const timestamp = new Date(data.timestamp).toLocaleTimeString();
                log.innerHTML += `[${timestamp}] ${data.data}\\n`;
                log.scrollTop = log.scrollHeight;
            }

            function updateChart(data) {
                if (data.parsed && (data.parsed.TEMP || data.parsed.HUMID)) {
                    const now = new Date().toLocaleTimeString();
                    
                    chartData.labels.push(now);
                    if (chartData.labels.length > 20) {
                        chartData.labels.shift();
                    }
                    
                    if (data.parsed.TEMP) {
                        chartData.datasets[0].data.push(data.parsed.TEMP);
                        if (chartData.datasets[0].data.length > 20) {
                            chartData.datasets[0].data.shift();
                        }
                    }
                    
                    if (data.parsed.HUMID) {
                        chartData.datasets[1].data.push(data.parsed.HUMID);
                        if (chartData.datasets[1].data.length > 20) {
                            chartData.datasets[1].data.shift();
                        }
                    }
                    
                    chart.update();
                }
            }

            function sendCommand(command) {
                fetch('/send_command', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({command: command})
                });
            }

            function sendCustomCommand() {
                const command = document.getElementById('customCommand').value;
                if (command) {
                    sendCommand(command);
                    document.getElementById('customCommand').value = '';
                }
            }

            // Send command on Enter key
            document.getElementById('customCommand').addEventListener('keypress', function(e) {
                if (e.key === 'Enter') {
                    sendCustomCommand();
                }
            });
        </script>
    </body>
    </html>
    '''

@app.route('/send_command', methods=['POST'])
def send_command():
    """Send command to ATmega128"""
    data = request.json
    command = data.get('command', '')
    
    if atmega.send_command(command):
        return jsonify({'status': 'success', 'command': command})
    else:
        return jsonify({'status': 'error', 'message': 'Failed to send command'})

@app.route('/api/data')
def get_data():
    """Get current data buffer"""
    return jsonify(atmega.data_buffer)

if __name__ == '__main__':
    print("🌐 Starting ATmega128 Web Dashboard...")
    print("📡 Connect to: http://localhost:5000")
    
    # Start monitoring
    atmega.start_monitoring()
    
    # Run Flask app
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)