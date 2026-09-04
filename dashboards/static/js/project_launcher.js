/**
 * Project Launcher Dashboard JavaScript
 * © 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)
 */

// Global state
const state = {
    projects: [],
    currentProject: null,
    currentDemo: null,
    selectedDevice: 'simulator',
    connected: false,
    building: false,
    editor: null,
    currentFile: null,
    originalContent: '',
    editorModified: false,
    availableFiles: []
};

// Socket.IO connection
const socket = io();

// DOM Elements
const elements = {
    projectsList: document.getElementById('projectsList'),
    projectDetails: document.getElementById('projectDetails'),
    projectSearch: document.getElementById('projectSearch'),
    buildBtn: document.getElementById('buildBtn'),
    runBtn: document.getElementById('runBtn'),
    stopBtn: document.getElementById('stopBtn'),
    buildOutput: document.getElementById('buildOutput'),
    clearOutput: document.getElementById('clearOutput'),
    scanDevices: document.getElementById('scanDevices'),
    deviceInfo: document.getElementById('deviceInfo'),
    serialConsole: document.getElementById('serialConsole'),
    serialInput: document.getElementById('serialInput'),
    sendBtn: document.getElementById('sendBtn'),
    wsStatus: document.getElementById('wsStatus'),
    deviceStatus: document.getElementById('deviceStatus'),
    codeEditorSection: document.getElementById('codeEditorSection'),
    editorTabs: document.getElementById('editorTabs'),
    monacoEditor: document.getElementById('monacoEditor'),
    saveFileBtn: document.getElementById('saveFileBtn'),
    revertBtn: document.getElementById('revertBtn'),
    closeEditorBtn: document.getElementById('closeEditorBtn'),
    editorFilePath: document.getElementById('editorFilePath'),
    editorModified: document.getElementById('editorModified')
};

// ============ Initialization ============
document.addEventListener('DOMContentLoaded', () => {
    console.log('🚀 Project Launcher Dashboard initializing...');

    // Load projects
    loadProjects();

    // Setup event listeners
    setupEventListeners();

    // Setup Socket.IO listeners
    setupSocketListeners();
});

// ============ Project Loading ============
async function loadProjects() {
    try {
        const response = await fetch('/api/projects');
        const projects = await response.json();

        state.projects = projects;
        renderProjectsList(projects);

        console.log(`✅ Loaded ${projects.length} projects`);
    } catch (error) {
        console.error('Error loading projects:', error);
        elements.projectsList.innerHTML = '<div class="loading">Error loading projects</div>';
    }
}

function renderProjectsList(projects) {
    if (projects.length === 0) {
        elements.projectsList.innerHTML = '<div class="loading">No projects found</div>';
        return;
    }

    elements.projectsList.innerHTML = projects.map(project => `
        <div class="project-item" data-project="${project.name}">
            <h4>${project.name}</h4>
            <div class="demo-count">${project.demos.length} demos</div>
        </div>
    `).join('');

    // Add click handlers
    document.querySelectorAll('.project-item').forEach(item => {
        item.addEventListener('click', () => selectProject(item.dataset.project));
    });
}

function selectProject(projectName) {
    const project = state.projects.find(p => p.name === projectName);
    if (!project) return;

    state.currentProject = project;
    state.currentDemo = null;

    // Update UI
    document.querySelectorAll('.project-item').forEach(item => {
        item.classList.toggle('active', item.dataset.project === projectName);
    });

    // Directly open code editor instead of showing project details
    openCodeEditor(projectName);

    // Enable build buttons
    elements.buildBtn.disabled = false;
    elements.runBtn.disabled = false;

    console.log(`📂 Selected project: ${projectName}`);
}

function renderProjectDetails(project) {
    const hasObjectives = project.objectives && project.objectives.length > 0;

    // Find active demo for display only
    const activeDemo = project.demos ? project.demos.find(d => d.active) : null;

    elements.projectDetails.innerHTML = `
        <div class="project-info">
            <h2>${project.name}</h2>
            <p class="project-description">${project.description}</p>
            
            ${activeDemo ? `
                <div class="active-function-badge">
                    <span class="badge-icon">▶</span>
                    <span class="badge-text">Active Function: <strong>${activeDemo.name}</strong></span>
                </div>
            ` : ''}
            
            <div class="project-actions" style="margin: 1.5rem 0;">
                <button class="control-btn primary" id="editCodeBtn" style="font-size: 1rem; padding: 0.75rem 1.5rem;">
                    <span class="icon">📝</span> Edit Source Code
                </button>
            </div>
            
            ${hasObjectives ? `
                <div class="objectives">
                    <h4>🎯 Learning Objectives:</h4>
                    <ul>
                        ${project.objectives.map(obj => `<li>${obj}</li>`).join('')}
                    </ul>
                </div>
            ` : ''}
        </div>
    `;

    // Add edit code button handler
    const editCodeBtn = document.getElementById('editCodeBtn');
    if (editCodeBtn) {
        editCodeBtn.addEventListener('click', () => openCodeEditor(project.name));
    }
}

function selectDemo(demoFunction, sourceFile = 'Main.c') {
    state.currentDemo = { function: demoFunction, file: sourceFile };

    document.querySelectorAll('.demo-item').forEach(item => {
        item.classList.toggle('selected',
            item.dataset.demo === demoFunction && item.dataset.file === sourceFile
        );
    });

    console.log(`🎮 Selected demo: ${demoFunction} from ${sourceFile}`);
}

// ============ Device Management ============
function setupEventListeners() {
    // Device selection buttons
    document.querySelectorAll('.device-btn').forEach(btn => {
        btn.addEventListener('click', () => selectDevice(btn.dataset.device));
    });

    // Scan devices
    elements.scanDevices.addEventListener('click', scanDevices);

    // Build & Run buttons
    elements.buildBtn.addEventListener('click', buildProject);
    elements.runBtn.addEventListener('click', runProject);
    elements.stopBtn.addEventListener('click', stopProject);

    // Output controls
    elements.clearOutput.addEventListener('click', () => {
        elements.buildOutput.textContent = '';
    });

    // Serial controls
    elements.sendBtn.addEventListener('click', sendSerialCommand);
    elements.serialInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') sendSerialCommand();
    });

    // Project search
    elements.projectSearch.addEventListener('input', (e) => {
        const query = e.target.value.toLowerCase();
        const filtered = state.projects.filter(p =>
            p.name.toLowerCase().includes(query)
        );
        renderProjectsList(filtered);
    });

    // Code editor controls
    elements.saveFileBtn.addEventListener('click', saveCurrentFile);
    elements.revertBtn.addEventListener('click', revertCurrentFile);
    elements.closeEditorBtn.addEventListener('click', closeCodeEditor);
}

function selectDevice(deviceType) {
    state.selectedDevice = deviceType;

    document.querySelectorAll('.device-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.device === deviceType);
    });

    if (deviceType === 'simulator') {
        elements.deviceInfo.textContent = 'SimulIDE virtual device (auto-detected)';
    } else {
        elements.deviceInfo.textContent = 'Physical ATmega128 kit via USB';
    }

    console.log(`🎯 Selected device: ${deviceType}`);
}

async function scanDevices() {
    try {
        elements.deviceInfo.textContent = 'Scanning for devices...';

        const response = await fetch('/api/device/scan');
        const data = await response.json();

        if (data.detected_port) {
            elements.deviceInfo.textContent = `Found: ${data.detected_type} on ${data.detected_port}`;
            updateDeviceStatus(true, data.detected_type);
        } else {
            elements.deviceInfo.textContent = `Available ports: ${data.ports.join(', ') || 'None'}`;
            updateDeviceStatus(false);
        }

        console.log('🔍 Device scan:', data);
    } catch (error) {
        console.error('Scan error:', error);
        elements.deviceInfo.textContent = 'Scan failed';
    }
}

function updateDeviceStatus(connected, type = '') {
    const dot = elements.deviceStatus.querySelector('.dot');
    const text = elements.deviceStatus.querySelector('.text');

    if (connected) {
        dot.classList.remove('offline');
        dot.classList.add('online');
        text.textContent = type === 'simulator' ? 'SimulIDE' : 'Hardware';
    } else {
        dot.classList.remove('online');
        dot.classList.add('offline');
        text.textContent = 'No Device';
    }
}

// ============ Build & Run ============
async function buildProject() {
    if (!state.currentProject || state.building) return;

    state.building = true;
    elements.buildBtn.disabled = true;
    elements.runBtn.disabled = true;

    appendOutput(`\n🔨 Building ${state.currentProject.name}...\n`);

    try {
        const response = await fetch('/api/build', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ project: state.currentProject.name })
        });

        const result = await response.json();

        appendOutput(result.output);

        if (result.success) {
            appendOutput('\n✅ Build successful!\n');
        } else {
            appendOutput('\n❌ Build failed!\n');
        }

    } catch (error) {
        appendOutput(`\n❌ Error: ${error.message}\n`);
    } finally {
        state.building = false;
        elements.buildBtn.disabled = false;
        elements.runBtn.disabled = false;
    }
}

async function runProject() {
    if (!state.currentProject || state.building) return;

    state.building = true;
    elements.buildBtn.disabled = true;
    elements.runBtn.disabled = true;
    elements.stopBtn.disabled = false;

    appendOutput(`\n▶️ Building and running ${state.currentProject.name}...\n`);

    try {
        const response = await fetch('/api/run', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                project: state.currentProject.name,
                device: state.selectedDevice
            })
        });

        const result = await response.json();

        if (result.build_output) {
            appendOutput(result.build_output);
        }

        if (result.success) {
            appendOutput(`\n✅ ${result.message}\n`);
            updateDeviceStatus(true, state.selectedDevice);
        } else {
            appendOutput(`\n❌ ${result.message}\n`);
        }

    } catch (error) {
        appendOutput(`\n❌ Error: ${error.message}\n`);
    } finally {
        state.building = false;
        elements.buildBtn.disabled = false;
        elements.runBtn.disabled = false;
    }
}

function stopProject() {
    appendOutput('\n⏹️ Stopping project...\n');
    elements.stopBtn.disabled = true;
    // TODO: Implement stop functionality
}

function appendOutput(text) {
    elements.buildOutput.textContent += text;
    elements.buildOutput.scrollTop = elements.buildOutput.scrollHeight;
}

// ============ Serial Communication ============
async function sendSerialCommand() {
    const command = elements.serialInput.value.trim();
    if (!command) return;

    try {
        const response = await fetch('/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command })
        });

        const result = await response.json();

        if (result.success) {
            appendSerialOutput(`> ${command}\n`, 'sent');
            elements.serialInput.value = '';
        }
    } catch (error) {
        console.error('Send error:', error);
    }
}

function appendSerialOutput(text, type = 'received') {
    const timestamp = new Date().toLocaleTimeString();
    const line = document.createElement('div');
    line.className = `serial-line ${type}`;
    line.textContent = `[${timestamp}] ${text}`;

    elements.serialConsole.appendChild(line);
    elements.serialConsole.scrollTop = elements.serialConsole.scrollHeight;
}

// ============ Socket.IO Events ============
function setupSocketListeners() {
    socket.on('connect', () => {
        console.log('✅ WebSocket connected');
        updateWebSocketStatus(true);
    });

    socket.on('disconnect', () => {
        console.log('❌ WebSocket disconnected');
        updateWebSocketStatus(false);
    });

    socket.on('status', (data) => {
        console.log('Status:', data);
    });

    socket.on('serial_data', (data) => {
        appendSerialOutput(data.data, 'received');
    });

    socket.on('led_update', (data) => {
        updateLEDs(data.states);
    });

    socket.on('adc_update', (data) => {
        updateADC(data);
    });

    socket.on('sensor_update', (data) => {
        console.log('Sensor update:', data);
    });
}

function updateWebSocketStatus(connected) {
    const dot = elements.wsStatus.querySelector('.dot');
    const text = elements.wsStatus.querySelector('.text');

    if (connected) {
        dot.classList.remove('offline');
        dot.classList.add('online');
        text.textContent = 'Connected';
    } else {
        dot.classList.remove('online');
        dot.classList.add('offline');
        text.textContent = 'Disconnected';
    }
}

function updateLEDs(states) {
    states.forEach((state, index) => {
        const led = document.querySelector(`.led[data-bit="${index}"]`);
        if (led) {
            led.classList.toggle('on', state);
        }
    });
}

function updateADC(data) {
    const channel = data.channel || 0;
    const value = data.value || 0;
    const percentage = (value / 1023) * 100;

    const channels = elements.adcDisplay.querySelectorAll('.adc-channel');
    if (channels[channel]) {
        const fill = channels[channel].querySelector('.progress-fill');
        const valueSpan = channels[channel].querySelector('.value');

        fill.style.width = `${percentage}%`;
        valueSpan.textContent = value;
    }
}

// ============ Code Editor Functions ============
function initializeMonacoEditor() {
    require.config({ paths: { vs: 'https://cdnjs.cloudflare.com/ajax/libs/monaco-editor/0.44.0/min/vs' } });
    require(['vs/editor/editor.main'], function () {
        state.editor = monaco.editor.create(elements.monacoEditor, {
            value: '// Select a file to edit',
            language: 'c',
            theme: 'vs-dark',
            automaticLayout: true,
            minimap: { enabled: true },
            fontSize: 14,
            lineNumbers: 'on',
            scrollBeyondLastLine: false,
            readOnly: false
        });

        // Track changes
        state.editor.onDidChangeModelContent(() => {
            if (state.currentFile && state.editor.getValue() !== state.originalContent) {
                state.editorModified = true;
                if (elements.editorModified) elements.editorModified.style.display = 'inline';
                if (elements.saveFileBtn) elements.saveFileBtn.disabled = false;
                if (elements.revertBtn) elements.revertBtn.disabled = false;
            } else {
                state.editorModified = false;
                if (elements.editorModified) elements.editorModified.style.display = 'none';
                if (elements.saveFileBtn) elements.saveFileBtn.disabled = true;
                if (elements.revertBtn) elements.revertBtn.disabled = true;
            }
        });

        // Add Ctrl+S keyboard shortcut
        state.editor.addCommand(monaco.KeyMod.CtrlCmd | monaco.KeyCode.KeyS, function () {
            console.log('⌨️ Ctrl+S pressed in editor');
            saveCurrentFile();
        });

        console.log('✅ Monaco Editor initialized');
        console.log('💡 Tip: Use Ctrl+S to save changes');
    });
}

function openCodeEditor(projectName) {
    if (!state.editor) {
        initializeMonacoEditor();
    }

    // Don't overwrite state.currentProject - it's already set by selectProject()
    // state.currentProject should be the full project object, not just the name

    // Get project info to display in editor header
    const project = state.projects.find(p => p.name === projectName);
    if (project) {
        document.getElementById('editorProjectName').textContent = `📝 ${project.name}`;
        document.getElementById('editorProjectDesc').textContent = project.description || '';
    }

    // List available files
    fetch('/api/file/list', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ project: projectName })
    })
        .then(res => res.json())
        .then(data => {
            if (data.success) {
                state.availableFiles = data.files;
                renderEditorTabs(data.files);

                // Auto-open Main.c if it exists
                const mainFile = data.files.find(f => f.name === 'Main.c');
                if (mainFile) {
                    loadFileIntoEditor(projectName, 'Main.c');
                }

                elements.codeEditorSection.style.display = 'block';

                // Show helpful hint about saving
                appendOutput('💡 Tip: Press Ctrl+S or click the Save button to save changes', 'info');
            }
        })
        .catch(err => {
            console.error('Failed to list files:', err);
            appendOutput(`❌ Error listing files: ${err.message}`, 'error');
        });
}

function renderEditorTabs(files) {
    elements.editorTabs.innerHTML = '';

    // Prioritize Main.c and Lab.c
    const priority = ['Main.c', 'Lab.c'];
    const sortedFiles = files.sort((a, b) => {
        const aIndex = priority.indexOf(a.name);
        const bIndex = priority.indexOf(b.name);
        if (aIndex !== -1 && bIndex !== -1) return aIndex - bIndex;
        if (aIndex !== -1) return -1;
        if (bIndex !== -1) return 1;
        return a.name.localeCompare(b.name);
    });

    sortedFiles.forEach(file => {
        if (file.ext === '.c' || file.ext === '.h') {
            const tab = document.createElement('button');
            tab.className = 'editor-tab';
            tab.textContent = file.name;
            tab.dataset.filename = file.name;
            tab.onclick = () => loadFileIntoEditor(state.currentProject, file.name);
            elements.editorTabs.appendChild(tab);
        }
    });
}

function loadFileIntoEditor(projectName, filename) {
    fetch('/api/file/read', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ project: projectName, filename: filename })
    })
        .then(res => res.json())
        .then(data => {
            if (data.success) {
                state.currentFile = filename;
                state.originalContent = data.content;
                state.editorModified = false;

                if (state.editor) {
                    state.editor.setValue(data.content);
                }

                // Update UI
                if (elements.editorFilePath) elements.editorFilePath.textContent = data.path;
                if (elements.editorModified) elements.editorModified.style.display = 'none';
                if (elements.saveFileBtn) elements.saveFileBtn.disabled = true;
                if (elements.revertBtn) elements.revertBtn.disabled = true;

                // Update active tab
                document.querySelectorAll('.editor-tab').forEach(tab => {
                    tab.classList.toggle('active', tab.dataset.filename === filename);
                });

                appendOutput(`📂 Opened: ${filename}`, 'info');
            } else {
                appendOutput(`❌ Error loading file: ${data.error}`, 'error');
            }
        })
        .catch(err => {
            console.error('Failed to load file:', err);
            appendOutput(`❌ Error loading file: ${err.message}`, 'error');
        });
}

function saveCurrentFile() {
    console.log('💾 Save button clicked');
    console.log('Current file:', state.currentFile);
    console.log('Editor modified:', state.editorModified);
    console.log('Current project:', state.currentProject);

    if (!state.currentFile || !state.editorModified) {
        console.warn('⚠️ Save aborted: no file or no modifications');
        return;
    }

    if (!state.currentProject || !state.currentProject.name) {
        console.error('❌ No project selected');
        appendOutput('❌ Error: No project selected', 'error');
        return;
    }

    const content = state.editor.getValue();
    console.log('Content length:', content.length);
    console.log('Saving to project:', state.currentProject.name);

    elements.saveFileBtn.disabled = true;
    elements.saveFileBtn.innerHTML = '<span class="icon">⏳</span> Saving...';

    fetch('/api/file/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            project: state.currentProject.name,
            filename: state.currentFile,
            content: content
        })
    })
        .then(res => res.json())
        .then(data => {
            console.log('💾 Save response:', data);
            if (data.success) {
                state.originalContent = content;
                state.editorModified = false;
                elements.editorModified.style.display = 'none';
                elements.saveFileBtn.innerHTML = '<span class="icon">💾</span> Save';
                elements.revertBtn.disabled = true;

                appendOutput(`✅ ${data.message}`, 'success');
                appendOutput(`💾 Backup created: ${data.backup}`, 'info');
                console.log('✅ File saved successfully');

                // Reload project info to update active function
                loadProjects();
            } else {
                console.error('❌ Save failed:', data.error);
                appendOutput(`❌ Save failed: ${data.error}`, 'error');
                elements.saveFileBtn.disabled = false;
                elements.saveFileBtn.innerHTML = '<span class="icon">💾</span> Save';
            }
        })
        .catch(err => {
            console.error('❌ Failed to save file:', err);
            appendOutput(`❌ Error saving file: ${err.message}`, 'error');
            elements.saveFileBtn.disabled = false;
            elements.saveFileBtn.innerHTML = '<span class="icon">💾</span> Save';
        });
}

function revertCurrentFile() {
    if (!state.currentFile || !state.editorModified) return;

    if (confirm('Revert all changes? This cannot be undone.')) {
        state.editor.setValue(state.originalContent);
        state.editorModified = false;
        elements.editorModified.style.display = 'none';
        elements.saveFileBtn.disabled = true;
        elements.revertBtn.disabled = true;
        appendOutput('↶ Changes reverted', 'info');
    }
}

function closeCodeEditor() {
    if (state.editorModified) {
        if (!confirm('You have unsaved changes. Close anyway?')) {
            return;
        }
    }

    elements.codeEditorSection.style.display = 'none';
    state.currentFile = null;
    state.originalContent = '';
    state.editorModified = false;
}

// ============ Utility Functions ============
function showNotification(message, type = 'info') {
    console.log(`[${type.toUpperCase()}] ${message}`);
    // TODO: Implement toast notifications
}

console.log('✅ Project Launcher Dashboard loaded');
