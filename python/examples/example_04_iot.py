# Example 4: IoT Web Dashboard
# Learn how to create a web interface for your ATmega128

from atmega128 import ATmega128
from flask import Flask, render_template, jsonify, request
import json
import threading
import time
from datetime import datetime

print("🌐 Example 4: IoT Web Dashboard")
print("=" * 40)

# Global variables
mcu = None
app = Flask(__name__)
sensor_data = {
    'temperature': [],
    'humidity': [], 
    'light': [],
    'timestamps': []
}

def background_data_collection():
    """Collect sensor data in background thread."""
    global mcu, sensor_data
    
    while mcu and mcu.connected:
        try:
            data = mcu.read_sensors()
            if data:
                now = datetime.now()
                
                # Keep only last 50 readings
                max_points = 50
                
                sensor_data['timestamps'].append(now.strftime('%H:%M:%S'))
                sensor_data['temperature'].append(data.get('TEMP', 0))
                sensor_data['humidity'].append(data.get('HUMID', 0))
                sensor_data['light'].append(data.get('LIGHT', 0))
                
                # Trim old data
                for key in sensor_data:
                    if len(sensor_data[key]) > max_points:
                        sensor_data[key] = sensor_data[key][-max_points:]
                        
            time.sleep(1)
        except Exception as e:
            print(f"Data collection error: {e}")
            break

@app.route('/')
def dashboard():
    """Main dashboard page."""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>ATmega128 IoT Dashboard</title>
        <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; }
            .container { max-width: 1200px; margin: 0 auto; }
            .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
            .connected { background-color: #d4edda; color: #155724; }
            .disconnected { background-color: #f8d7da; color: #721c24; }
            .controls { margin: 20px 0; }
            .button { padding: 10px 20px; margin: 5px; border: none; border-radius: 5px; cursor: pointer; }
            .button.led-on { background-color: #28a745; color: white; }
            .button.led-off { background-color: #dc3545; color: white; }
            .sensor-values { display: flex; gap: 20px; margin: 20px 0; }
            .sensor-card { flex: 1; padding: 20px; border: 1px solid #ddd; border-radius: 5px; text-align: center; }
            .chart-container { width: 100%; height: 400px; margin: 20px 0; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🌐 ATmega128 IoT Dashboard</h1>
            
            <div id="status" class="status">
                <strong>Status:</strong> <span id="connection-status">Checking...</span>
            </div>
            
            <div class="controls">
                <h3>💡 LED Control</h3>
                <button class="button led-on" onclick="controlLED('on')">Turn LED ON</button>
                <button class="button led-off" onclick="controlLED('off')">Turn LED OFF</button>
            </div>
            
            <div class="sensor-values">
                <div class="sensor-card">
                    <h3>🌡️ Temperature</h3>
                    <div id="temp-value" style="font-size: 2em; font-weight: bold;">--°C</div>
                </div>
                <div class="sensor-card">
                    <h3>💧 Humidity</h3>
                    <div id="humid-value" style="font-size: 2em; font-weight: bold;">--%</div>
                </div>
                <div class="sensor-card">
                    <h3>💡 Light</h3>
                    <div id="light-value" style="font-size: 2em; font-weight: bold;">--</div>
                </div>
            </div>
            
            <div class="chart-container">
                <canvas id="sensorChart"></canvas>
            </div>
        </div>
        
        <script>
            // Chart setup
            const ctx = document.getElementById('sensorChart').getContext('2d');
            const chart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [{
                        label: 'Temperature (°C)',
                        data: [],
                        borderColor: 'rgb(255, 99, 132)',
                        tension: 0.1
                    }, {
                        label: 'Humidity (%)',
                        data: [],
                        borderColor: 'rgb(54, 162, 235)', 
                        tension: 0.1
                    }, {
                        label: 'Light Level',
                        data: [],
                        borderColor: 'rgb(255, 205, 86)',
                        tension: 0.1
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            beginAtZero: true
                        }
                    }
                }
            });
            
            // Update data every second
            function updateData() {
                fetch('/api/data')
                    .then(response => response.json())
                    .then(data => {
                        // Update connection status
                        const statusEl = document.getElementById('connection-status');
                        const statusDiv = document.getElementById('status');
                        if (data.connected) {
                            statusEl.textContent = 'Connected ✅';
                            statusDiv.className = 'status connected';
                        } else {
                            statusEl.textContent = 'Disconnected ❌';
                            statusDiv.className = 'status disconnected';
                        }
                        
                        // Update current values
                        if (data.current) {
                            document.getElementById('temp-value').textContent = 
                                data.current.temperature ? data.current.temperature.toFixed(1) + '°C' : '--°C';
                            document.getElementById('humid-value').textContent = 
                                data.current.humidity ? data.current.humidity.toFixed(1) + '%' : '--%';
                            document.getElementById('light-value').textContent = 
                                data.current.light ? data.current.light.toFixed(0) : '--';
                        }
                        
                        // Update chart
                        chart.data.labels = data.timestamps;
                        chart.data.datasets[0].data = data.temperature;
                        chart.data.datasets[1].data = data.humidity;
                        chart.data.datasets[2].data = data.light;
                        chart.update('none');
                    });
            }
            
            // LED control
            function controlLED(action) {
                fetch('/api/led', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({action: action})
                })
                .then(response => response.json())
                .then(data => {
                    console.log('LED control:', data);
                });
            }
            
            // Start updating
            updateData();
            setInterval(updateData, 1000);
        </script>
    </body>
    </html>
    '''

@app.route('/api/data')
def get_data():
    """API endpoint for sensor data."""
    current_data = {
        'temperature': sensor_data['temperature'][-1] if sensor_data['temperature'] else 0,
        'humidity': sensor_data['humidity'][-1] if sensor_data['humidity'] else 0,
        'light': sensor_data['light'][-1] if sensor_data['light'] else 0
    }
    
    return jsonify({
        'connected': mcu.connected if mcu else False,
        'current': current_data,
        'timestamps': sensor_data['timestamps'],
        'temperature': sensor_data['temperature'],
        'humidity': sensor_data['humidity'],
        'light': sensor_data['light']
    })

@app.route('/api/led', methods=['POST'])
def control_led():
    """API endpoint for LED control."""
    data = request.json
    action = data.get('action')
    
    success = False
    if mcu and mcu.connected:
        if action == 'on':
            success = mcu.led_on()
        elif action == 'off':
            success = mcu.led_off()
    
    return jsonify({'success': success, 'action': action})

if __name__ == '__main__':
    # Connect to ATmega128
    print("🔗 Connecting to ATmega128...")
    mcu = ATmega128("COM3")  # Change COM3 to your port
    
    if mcu.connected:
        print("✅ Connected successfully!")
        
        # Start background data collection
        print("📊 Starting data collection...")
        data_thread = threading.Thread(target=background_data_collection)
        data_thread.daemon = True
        data_thread.start()
        
        # Start web server
        print("🌐 Starting web server...")
        print("📱 Open your browser to: http://localhost:5000")
        print("⏹️  Press Ctrl+C to stop")
        
        try:
            app.run(host='0.0.0.0', port=5000, debug=False)
        except KeyboardInterrupt:
            print("\n⏹️ Stopping server...")
        finally:
            mcu.close()
            print("🔌 Disconnected")
    else:
        print("❌ Could not connect. Check your port!")

print("\n🎉 IoT dashboard complete!")