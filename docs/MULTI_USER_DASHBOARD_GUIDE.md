# ATmega128 Multi-User Classroom Dashboard

## 🎓 Overview

The ATmega128 Multi-User Classroom Dashboard provides a web-based interface that allows multiple students to simultaneously access, edit, build, and simulate ATmega128 projects through their web browsers. No software installation is required on student devices.

## 🌐 Multi-User Web Access Solution

### **For Students:**
- **Web-based access**: Students connect via any modern web browser
- **No installation required**: Just need the classroom URL
- **Isolated workspaces**: Each student gets their own project copies
- **Real-time building**: Compile and build projects online
- **Session management**: Automatic login and workspace assignment

### **For Teachers:**
- **Real-time monitoring**: See all student activity live
- **Session management**: Track who's logged in and working
- **Activity logging**: Monitor builds, errors, and progress
- **Admin dashboard**: Complete oversight of classroom activity

## 🚀 Quick Deployment

### **1. One-Time Setup (Teacher/Administrator)**

```powershell
# Navigate to setup directory
cd w:\soc3050code\tools\setup

# Run deployment script
.\deploy-classroom-dashboard.ps1
```

### **2. Start the Dashboard**

```batch
# Double-click the generated startup script
start-classroom-dashboard.bat

# Or run manually
python dashboards\multi_user_dashboard.py
```

### **3. Student Access**

Students simply open their web browser and go to:
```
http://[server-ip]:5001
```

Example: `http://192.168.0.202:5001`

## 📱 Student Workflow

1. **Login**: Enter name in web browser
2. **Select Project**: Choose from 31+ sample projects  
3. **Edit Code**: Modify Main.c directly in browser
4. **Build**: Click "Build Project" to compile
5. **Debug**: View build output and fix errors
6. **Simulate**: Run in SimulIDE (coming soon)

## 🏫 Teacher Workflow

1. **Access Admin Panel**: Go to `http://[server-ip]:5001/teacher`
2. **Monitor Sessions**: See all active students
3. **Track Activity**: View real-time build attempts and results
4. **Manage Class**: Monitor progress and help students
5. **Export Data**: Download session and activity logs

## 🔧 Technical Architecture

### **Multi-User Features:**
- **Session Isolation**: Each student gets unique workspace
- **Concurrent Access**: Up to 50 simultaneous users
- **Real-time Updates**: WebSocket communication
- **Automatic Cleanup**: Sessions expire after 8 hours
- **Resource Management**: Build queuing and rate limiting

### **Network Configuration:**
- **Server IP**: Auto-detected local network IP
- **Port**: 5001 (configurable)
- **Firewall**: Automatically configured
- **Protocol**: HTTP/WebSocket (HTTPS ready)

### **Student Workspace Isolation:**
```
student_workspaces/
├── Student_20241008_143022_ab12cd34/
│   ├── projects/          # Personal copy of all projects
│   ├── outputs/           # Build results and logs
│   └── logs/              # Session activity
├── Student_20241008_143156_ef56gh78/
│   ├── projects/
│   ├── outputs/
│   └── logs/
```

## 📊 Monitoring & Management

### **Real-time Dashboards:**
- **Student count**: Active concurrent users
- **Build statistics**: Success/failure rates  
- **Session duration**: Average time spent
- **Project popularity**: Most used examples
- **Error tracking**: Common compilation issues

### **Activity Logging:**
- User login/logout events
- Project selection and switching
- Build attempts and results
- Error messages and debugging
- Session timeouts and cleanup

## 🛠 Classroom Setup Scenarios

### **Scenario 1: Computer Lab**
- Install on lab server
- Students access via lab computers
- Teacher monitors from instructor station
- Shared network, isolated workspaces

### **Scenario 2: BYOD (Bring Your Own Device)**
- Students use laptops/tablets
- Connect to classroom WiFi
- Access via any web browser
- Cross-platform compatibility

### **Scenario 3: Remote Learning**
- Deploy on cloud server
- Students access from home
- Teacher monitors remotely
- VPN or public internet access

### **Scenario 4: Hybrid Classroom**
- Some students in-person, some remote
- Single server supports both
- Unified monitoring interface
- Consistent experience everywhere

## 🔐 Security & Management

### **Access Control:**
- Simple name-based student login
- Teacher dashboard with admin access
- Session timeout management
- Workspace isolation and cleanup

### **Resource Protection:**
- Build rate limiting (5 builds/minute per student)
- File size restrictions (10MB max)
- Allowed file types (.c, .h, .txt, .md)
- Automatic garbage collection

### **Data Privacy:**
- Student workspaces are isolated
- No persistent user data required
- Session-based identification
- Automatic cleanup after timeout

## 📁 File Structure

```
dashboards/
├── multi_user_dashboard.py          # Main multi-user server
├── config/
│   └── multi_user_config.json       # Configuration file
├── templates/
│   ├── multi_user_login.html        # Student login page
│   ├── student_dashboard.html       # Student interface
│   └── teacher_dashboard.html       # Teacher monitoring
├── static/
│   └── js/
│       └── socket.io.min.js         # Real-time communication
└── student_workspaces/              # Isolated student directories
```

## 🚀 Production Deployment

### **Windows Service Installation:**
```powershell
# Run as Administrator
.\install-dashboard-service.ps1
```

### **Load Balancing (Multiple Servers):**
- Deploy on multiple machines
- Use nginx reverse proxy
- Round-robin student assignment
- Shared storage for workspaces

### **Cloud Deployment:**
- AWS EC2, Azure VM, or Google Cloud
- Docker containerization ready
- Horizontal scaling support
- Database backend for session storage

## 📞 Support & Troubleshooting

### **Common Issues:**
1. **Port already in use**: Change port in config file
2. **Firewall blocking**: Run deployment script as admin
3. **Python packages missing**: Script auto-installs requirements
4. **Network access denied**: Check WiFi and IP settings

### **Performance Optimization:**
- Increase session timeout for longer labs
- Adjust max students for server capacity  
- Enable build caching for faster compilation
- Use SSD storage for student workspaces

### **Monitoring Tools:**
- Teacher dashboard for real-time status
- Log files in student workspace directories
- System resource monitoring (CPU, RAM, disk)
- Network traffic analysis for optimization

## 🎯 Benefits Summary

✅ **Zero Installation**: Students need only a web browser  
✅ **Instant Access**: Click URL and start coding immediately  
✅ **Isolated Workspaces**: Each student works independently  
✅ **Real-time Monitoring**: Teachers see everything live  
✅ **Scalable**: Supports 50+ concurrent students  
✅ **Cross-Platform**: Works on Windows, Mac, Linux, tablets  
✅ **Professional Workflow**: Real build tools and simulators  
✅ **Educational Focus**: 31+ ready-to-use ATmega128 projects  

The system transforms the traditional software installation classroom into a modern, web-based educational environment where students can focus on learning ATmega128 programming without technical setup barriers.