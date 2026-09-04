# ATmega128 Educational Platform - Development TODO List

**Project**: Multi-User Web-Based IDE for ATmega128 + AI Integration  
**Developer**: Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
**Server**: Korea (Intel i9, 128GB RAM, 10TB HDD, 450Mbps)  
**Users**: 200+ students in Tashkent (30Mbps internet, ~50 concurrent)  
**Last Updated**: October 4, 2025

---

## 🎯 **Project Overview**

### **Goal**
Build a cloud-based educational platform with two separate dashboards:
1. **Project Launcher** (Individual Learning) - Port 5001
2. **Group Project Hub** (Team AI Projects) - Port 5002

### **Key Constraints**
- ✅ Server in Korea, students in Tashkent (200-300ms latency)
- ✅ Low bandwidth: 30Mbps / 50 students = 0.6Mbps per student
- ✅ 200+ students total, ~50 concurrent users
- ✅ Templates change frequently (bug fixes, new features)
- ✅ No physical hardware access (students are remote)
- ✅ Projects include AI modules (TensorFlow Lite, Edge Impulse)

---

## 📋 **Phase 1: Core Infrastructure Setup**

### **1.1 Server Environment Setup (Korea)**

- [ ] **Install Base Software**
  ```bash
  sudo apt update
  sudo apt install -y python3.11 python3-pip python3-venv
  sudo apt install -y nginx redis-server
  sudo apt install -y build-essential git
  ```

- [ ] **Install AVR Toolchain**
  ```bash
  sudo apt install -y gcc-avr avr-libc avrdude
  # Verify: avr-gcc --version
  ```

- [ ] **Install Python Dependencies**
  ```bash
  pip3 install flask flask-socketio flask-session flask-compress
  pip3 install flask-limiter redis rq gunicorn gevent-websocket
  pip3 install python-dotenv
  ```

- [ ] **Install SimulIDE** (headless for server)
  ```bash
  # Download SimulIDE 1.1.0
  # Configure for headless operation
  # Setup VNC/noVNC for remote viewing
  ```

- [ ] **Setup SSL/HTTPS**
  ```bash
  sudo apt install -y certbot python3-certbot-nginx
  sudo certbot --nginx -d your-domain.com
  ```

### **1.2 User Authentication System**

- [ ] **Create `auth_manager.py`**
  - Student ID login (no password, 8-digit ID)
  - Admin login (username + password)
  - Session management with Redis
  - 8-hour session timeout
  - Role-based permissions (admin vs student)

- [ ] **Create `users.json` Database**
  ```json
  {
    "admin": {
      "password_hash": "sha256_hash_here",
      "role": "admin",
      "name": "Prof. Hong Jeaong"
    },
    "students": {
      "12345678": {
        "name": "Student Name",
        "registered_at": "2025-10-04T10:00:00"
      }
    }
  }
  ```

- [ ] **Student Registration API**
  - Auto-register on first login
  - Bulk import from CSV (for class roster)

### **1.3 Workspace Management**

- [ ] **Create `workspace_manager.py`**
  - Copy-on-demand strategy (saves 70-90% storage)
  - Student workspace: `students/student_12345/active_projects/`
  - Only copy project when student first opens it
  - Track which projects student is using

- [ ] **Implement Copy-on-Demand Logic**
  ```python
  def get_student_project_path(student_id, project_name, create_if_missing=True):
      # Check if student has copy
      # If not and create_if_missing, copy from template
      # Return student's project path
  ```

### **1.4 Template Versioning System**

- [ ] **Create `template_manager.py`**
  - Track template versions (Git-like)
  - Calculate file hashes (MD5) to detect changes
  - Store version history in JSON
  - Notify students when template updates available

- [ ] **Version Control Features**
  - `register_template_version()` - When admin updates template
  - `check_template_updates()` - Check if student has old version
  - `get_file_diff()` - Show what changed
  - `update_student_project()` - Merge template updates
  - Backup student work before update

- [ ] **Admin Workflow**
  ```python
  # After editing projects/Port_Basic/Main.c
  POST /api/admin/template/version
  {
    "template": "Port_Basic",
    "description": "Fixed LED toggle bug in demo_03"
  }
  # → Notifies all 45 students using this project
  ```

---

## 📋 **Phase 2: Dashboard 1 - Project Launcher (Individual)**

### **2.1 Backend Development**

- [ ] **Modify `project_launcher_dashboard.py`**
  - Integrate `auth_manager`
  - Integrate `workspace_manager`
  - Integrate `template_manager`
  - Add login/logout routes
  - Protect all routes with `@login_required` decorator

- [ ] **Core API Endpoints**
  - `POST /api/auth/login` - Student ID login
  - `POST /api/auth/logout` - Logout
  - `GET /api/whoami` - Check current user
  - `GET /api/projects` - List projects (from student workspace)
  - `POST /api/file/read` - Read from student's copy
  - `POST /api/file/save` - Save to student's copy (with backup)
  - `POST /api/build` - Build in student workspace
  - `POST /api/run` - Launch SimulIDE with student's hex

- [ ] **Template Update Endpoints**
  - `POST /api/template/check_updates` - Check for updates
  - `POST /api/template/diff` - View changes
  - `POST /api/template/update` - Apply update
  - `POST /api/admin/template/version` - Register new version (admin only)

### **2.2 Frontend Development**

- [ ] **Create `templates/login.html`**
  - Simple student ID input field
  - Auto-uppercase, 8-digit validation
  - "Enter Lab" button
  - Link to admin login (separate form)

- [ ] **Update `templates/project_launcher.html`**
  - Add student info header (name, ID, logout button)
  - Add "Template Update Available" banner (when applicable)
  - Show "View Diff" and "Update Now" buttons
  - Display which version student is using

- [ ] **Update `static/js/project_launcher.js`**
  - Check session on page load
  - Auto-redirect to login if session expired
  - Check for template updates when opening project
  - Handle update workflow (view diff → confirm → apply)

### **2.3 Network Optimization (Critical for 30Mbps)**

- [ ] **Enable GZIP Compression**
  ```python
  from flask_compress import Compress
  Compress(app)
  # Reduces traffic by 70-90%
  ```

- [ ] **Replace Monaco Editor with CodeMirror 6**
  - Monaco: 2MB (too large for 0.6Mbps/student)
  - CodeMirror 6: 200KB (10x smaller!)
  ```html
  <script src="https://cdn.jsdelivr.net/npm/codemirror@6"></script>
  ```

- [ ] **Aggressive Caching**
  ```python
  app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 31536000  # 1 year
  ```

- [ ] **Minify JavaScript/CSS**
  - Remove comments, whitespace
  - Use production builds

### **2.4 Build Queue System**

- [ ] **Setup Redis Queue (RQ)**
  ```bash
  pip3 install rq
  redis-server --daemonize yes
  ```

- [ ] **Create Build Worker**
  ```python
  # worker.py
  from rq import Queue
  from redis import Redis
  
  def compile_project(student_id, project_name):
      # Run AVR-GCC compilation
      # Return build result
  
  # Start 8 workers:
  # rq worker builds &
  ```

- [ ] **Build Queue API**
  - `POST /api/build/queue` - Submit build job
  - `GET /api/build/status/<job_id>` - Check status
  - Rate limit: 10 builds per minute per student

---

## 📋 **Phase 3: Dashboard 2 - Group Project Hub (Teams + AI)**

### **3.1 Backend Development**

- [ ] **Create `group_project_hub_dashboard.py`**
  - Port 5002 (different from Project Launcher)
  - Team management system
  - Real-time collaboration (SocketIO)
  - AI model deployment
  - Version control (Git-like history)

- [ ] **Group Project API Endpoints**
  - `GET /api/teams/list` - List all teams
  - `POST /api/teams/create` - Create new team
  - `POST /api/teams/<id>/join` - Join team
  - `GET /api/teams/<id>/members` - List team members
  - `GET /api/teams/<id>/files` - List team files
  - `POST /api/teams/<id>/file/read` - Read team file
  - `POST /api/teams/<id>/file/save` - Save with version history
  - `GET /api/teams/<id>/history` - View file history

- [ ] **AI Model Management**
  - `GET /api/ai/models` - List available AI models
  - `POST /api/ai/upload` - Upload .tflite model
  - `POST /api/ai/deploy` - Deploy model to team project
  - `GET /api/ai/inference` - Get inference results

- [ ] **Real-Time Collaboration (WebSocket)**
  - `join_team` - Student joins team room
  - `leave_team` - Student leaves
  - `file_updated` - Notify team when file saved
  - `cursor_update` - Show teammate cursor positions
  - `member_online/offline` - Track who's online

### **3.2 Group Project Structure**

- [ ] **Create Templates**
  ```
  group_projects/templates/
  ├── AI_Vision_Robot/
  │   ├── src/
  │   │   ├── Main.c
  │   │   ├── ai_inference.c
  │   │   └── camera_driver.c
  │   ├── models/
  │   │   └── image_classifier.tflite
  │   ├── README.md
  │   └── team_config.json
  ├── Smart_Home_IoT/
  ├── Voice_Recognition/
  └── Gesture_Control/
  ```

- [ ] **Team Configuration Format**
  ```json
  {
    "team_id": "team_01",
    "project_name": "AI_Vision_Robot",
    "members": [
      {
        "student_id": "12345678",
        "name": "Student Name",
        "role": "leader",
        "joined_at": "2025-10-04T10:00:00"
      }
    ],
    "has_ai_model": true,
    "model_type": "tflite",
    "created_at": "2025-10-04T09:00:00"
  }
  ```

### **3.3 Frontend Development**

- [ ] **Create `templates/group_project_hub.html`**
  - Team dashboard (list of teams)
  - Team creation form
  - Project file browser
  - Collaborative code editor
  - AI model uploader
  - Live inference monitor
  - Team chat/activity feed

- [ ] **Advanced UI Components**
  - Real-time cursor tracking (show teammate cursors)
  - Live member presence (who's online)
  - File diff viewer (version history)
  - AI inference visualizations (graphs, metrics)
  - Camera feed viewer (for AI vision projects)
  - Model performance metrics

- [ ] **Create `static/js/group_hub.js`**
  - WebSocket connection management
  - Real-time file sync
  - Collaborative editing logic
  - AI model deployment UI
  - Live inference display

### **3.4 AI Resources Library**

- [ ] **Setup AI Resources Directory**
  ```
  ai_resources/
  ├── models/           # Pre-trained models
  │   ├── mnist_classifier.tflite
  │   ├── gesture_recognition.tflite
  │   └── voice_commands.tflite
  ├── datasets/         # Training datasets
  │   ├── gestures/
  │   └── voice_samples/
  └── libraries/        # SDK/Libraries
      ├── TensorFlowLite_ATmega/
      └── EdgeImpulse_SDK/
  ```

- [ ] **AI Model Templates**
  - Create 3-5 sample AI projects
  - Document model architectures
  - Provide training notebooks (Colab)
  - Include deployment instructions

---

## 📋 **Phase 4: Production Deployment**

### **4.1 NGINX Configuration**

- [ ] **Create `/etc/nginx/sites-available/atmega-lab`**
  ```nginx
  # Upstream servers (load balancing)
  upstream project_launcher {
      server 127.0.0.1:5001;
      server 127.0.0.1:5002;
      server 127.0.0.1:5003;
      server 127.0.0.1:5004;
  }
  
  upstream group_hub {
      server 127.0.0.1:5005;
      server 127.0.0.1:5006;
  }
  
  # HTTP → HTTPS redirect
  server {
      listen 80;
      return 301 https://$host$request_uri;
  }
  
  # HTTPS Server
  server {
      listen 443 ssl http2;
      server_name your-domain.com;
      
      ssl_certificate /etc/letsencrypt/live/your-domain/fullchain.pem;
      ssl_certificate_key /etc/letsencrypt/live/your-domain/privkey.pem;
      
      # GZIP compression
      gzip on;
      gzip_types text/css application/javascript application/json;
      gzip_min_length 1000;
      
      # Static files (1-year cache)
      location /static/ {
          alias /path/to/soc3050code/dashboards/static/;
          expires 1y;
          add_header Cache-Control "public, immutable";
      }
      
      # Project Launcher (5001-5004)
      location /launcher/ {
          proxy_pass http://project_launcher/;
          proxy_set_header Host $host;
          proxy_set_header X-Real-IP $remote_addr;
      }
      
      # Group Hub (5005-5006)
      location /groups/ {
          proxy_pass http://group_hub/;
          proxy_set_header Host $host;
          proxy_set_header X-Real-IP $remote_addr;
      }
      
      # WebSocket support
      location /socket.io/ {
          proxy_pass http://project_launcher;
          proxy_http_version 1.1;
          proxy_set_header Upgrade $http_upgrade;
          proxy_set_header Connection "upgrade";
          proxy_read_timeout 86400;
      }
      
      # Rate limiting
      limit_req_zone $binary_remote_addr zone=api:10m rate=10r/s;
      limit_req zone=api burst=20 nodelay;
  }
  ```

- [ ] **Enable Site**
  ```bash
  sudo ln -s /etc/nginx/sites-available/atmega-lab /etc/nginx/sites-enabled/
  sudo nginx -t
  sudo systemctl restart nginx
  ```

### **4.2 Gunicorn Setup**

- [ ] **Create `gunicorn_config.py`**
  ```python
  bind = "127.0.0.1:5001"
  workers = 4  # 4 workers for 50 concurrent users
  worker_class = "geventwebsocket.gunicorn.workers.GeventWebSocketWorker"
  timeout = 120
  keepalive = 5
  accesslog = "logs/access.log"
  errorlog = "logs/error.log"
  loglevel = "info"
  ```

- [ ] **Create Startup Script `start_dashboards.sh`**
  ```bash
  #!/bin/bash
  
  # Start Redis
  redis-server --daemonize yes
  
  # Start RQ workers (8 workers for compilation)
  for i in {1..8}; do
      rq worker builds --with-scheduler &
  done
  
  # Start Project Launcher (4 instances)
  for port in 5001 5002 5003 5004; do
      gunicorn -c gunicorn_config.py \
          --bind 127.0.0.1:$port \
          dashboards.project_launcher_dashboard:app &
  done
  
  # Start Group Hub (2 instances)
  for port in 5005 5006; do
      gunicorn -c gunicorn_config.py \
          --bind 127.0.0.1:$port \
          dashboards.group_project_hub_dashboard:app &
  done
  
  echo "All dashboards started"
  ```

### **4.3 Systemd Service**

- [ ] **Create `/etc/systemd/system/atmega-lab.service`**
  ```ini
  [Unit]
  Description=ATmega128 Educational Lab Platform
  After=network.target redis.service
  
  [Service]
  Type=forking
  User=www-data
  Group=www-data
  WorkingDirectory=/path/to/soc3050code
  ExecStart=/path/to/start_dashboards.sh
  ExecStop=/usr/bin/killall gunicorn
  Restart=on-failure
  
  [Install]
  WantedBy=multi-user.target
  ```

- [ ] **Enable Service**
  ```bash
  sudo systemctl daemon-reload
  sudo systemctl enable atmega-lab
  sudo systemctl start atmega-lab
  sudo systemctl status atmega-lab
  ```

### **4.4 Monitoring & Logging**

- [ ] **Setup Log Rotation**
  ```bash
  # /etc/logrotate.d/atmega-lab
  /path/to/logs/*.log {
      daily
      rotate 7
      compress
      missingok
      notifempty
  }
  ```

- [ ] **Create Admin Monitoring Dashboard**
  - Active sessions count
  - Active students list
  - Build queue status
  - System resource usage (CPU, RAM, disk)
  - Error log viewer

### **4.5 Backup Strategy**

- [ ] **Daily Student Workspace Backup**
  ```bash
  # Cron job: 0 2 * * * (2 AM daily)
  tar -czf /backups/students_$(date +%Y%m%d).tar.gz students/
  # Keep last 30 days
  find /backups -name "students_*.tar.gz" -mtime +30 -delete
  ```

- [ ] **Template Version Backup**
  ```bash
  # Before updating template, create backup
  cp -r projects/Port_Basic projects/Port_Basic_v4_backup
  ```

---

## 📋 **Phase 5: Testing & Quality Assurance**

### **5.1 Performance Testing**

- [ ] **Load Testing**
  - Test with 50 concurrent users (Apache JMeter)
  - Measure response times
  - Check server resource usage
  - Identify bottlenecks

- [ ] **Bandwidth Testing**
  - Simulate 30Mbps connection
  - Test page load times
  - Test file save/build times
  - Verify GZIP compression working

### **5.2 User Acceptance Testing**

- [ ] **Test with 10 Students (Pilot)**
  - Individual project workflow
  - Group project collaboration
  - Template updates
  - Build and SimulIDE
  - Collect feedback

### **5.3 Security Audit**

- [ ] **Check Security**
  - SQL injection prevention (N/A - using JSON files)
  - XSS protection (Flask auto-escapes)
  - CSRF protection (Flask-WTF)
  - Rate limiting (Flask-Limiter)
  - Session security (HTTPOnly, Secure cookies)
  - File upload validation

---

## 📋 **Phase 6: Documentation & Training**

### **6.1 User Documentation**

- [ ] **Student Guide**
  - How to login
  - How to select and edit projects
  - How to build and test
  - How to update templates
  - FAQ

- [ ] **Team Collaboration Guide**
  - Creating a team
  - Inviting members
  - Working on shared files
  - Deploying AI models
  - Best practices

### **6.2 Admin Documentation**

- [ ] **Admin Guide**
  - How to update templates
  - How to manage students
  - How to monitor system
  - How to create new projects
  - Troubleshooting

### **6.3 API Documentation**

- [ ] **Create OpenAPI/Swagger Docs**
  - Document all API endpoints
  - Include request/response examples
  - Add authentication requirements

---

## 📋 **Phase 7: Future Enhancements**

### **Low Priority / Future Features**

- [ ] **Mobile App** (React Native)
  - View projects on phone
  - Monitor build status
  - Team chat

- [ ] **GitHub Integration**
  - Export student projects to GitHub
  - Clone from GitHub
  - Version control with Git

- [ ] **Video Tutorials**
  - Embedded video lessons
  - Step-by-step guides
  - Screen recording of solutions

- [ ] **AI Model Training**
  - In-browser model training (TensorFlow.js)
  - Upload custom datasets
  - Train and deploy pipeline

- [ ] **Code Quality Analysis**
  - Static analysis (cppcheck)
  - Style checking
  - Complexity metrics

- [ ] **Leaderboard/Gamification**
  - Student rankings
  - Badges/achievements
  - Project completion tracking

---

## 📊 **Resource Estimates**

### **Storage Requirements**
- **Templates**: ~500MB (35 individual + 10 group projects)
- **Per Student**: ~50-100MB (3-5 active projects)
- **200 Students**: ~10-20GB total
- **Backups**: ~30GB (30 days × 1GB/day)
- **AI Resources**: ~2GB (models, datasets, libraries)
- **Total**: ~50GB (well within 10TB capacity)

### **Bandwidth Usage**
- **Per Student Session**: 
  - Initial load: 500KB (with GZIP)
  - Per file edit: 50KB
  - Per build: 200KB
  - Total: ~1MB per hour
- **50 Concurrent Students**: ~50MB/hour = ~0.11Mbps
- **Well within 30Mbps limit**

### **Server Resources**
- **CPU**: 10-30% (i9 handles easily)
- **RAM**: 4-8GB (128GB is overkill, excellent!)
- **Disk I/O**: Low (mostly small file operations)

---

## 🚀 **Deployment Timeline**

### **Week 1: Infrastructure**
- Server setup
- Authentication system
- Workspace management

### **Week 2: Dashboard 1**
- Project Launcher backend
- Frontend updates
- Network optimization

### **Week 3: Dashboard 2**
- Group Project Hub backend
- Real-time collaboration
- AI integration

### **Week 4: Production & Testing**
- NGINX/Gunicorn setup
- Load testing
- Pilot with students

### **Week 5: Launch**
- Full deployment
- Monitor and fix issues
- Collect feedback

---

## 📞 **Support & Maintenance**

### **Ongoing Tasks**
- [ ] Monitor server logs daily
- [ ] Update templates weekly
- [ ] Backup student data daily
- [ ] Review student feedback
- [ ] Add new AI project templates
- [ ] Update documentation

### **Emergency Contacts**
- **Server Issues**: [Your IT support]
- **Network Issues**: [Network admin]
- **Student Support**: [TA or assistant]

---

## ✅ **Success Criteria**

The system is successful when:
1. ✅ 50 students can work simultaneously without lag
2. ✅ Page loads in < 5 seconds on 30Mbps connection
3. ✅ Students can edit, save, build, and run projects smoothly
4. ✅ Template updates propagate without breaking student work
5. ✅ Teams can collaborate in real-time
6. ✅ AI models deploy and run on ATmega128
7. ✅ System uptime > 99%
8. ✅ Student satisfaction > 80%

---

## 📝 **Notes & Decisions**

### **Architectural Decisions**
1. **Two separate dashboards** - Cleaner separation of concerns
2. **Copy-on-demand** - Saves 70-90% storage
3. **Template versioning** - Handles frequent updates gracefully
4. **Session-based auth** - Simple, no passwords for students
5. **Redis sessions** - Scalable, persistent
6. **Build queue** - Prevents CPU overload
7. **NGINX load balancing** - Distributes 50 concurrent users
8. **CodeMirror over Monaco** - 10x smaller for low bandwidth

### **Rejected Alternatives**
- ❌ Multi-process per student (too resource heavy)
- ❌ Full Git integration (too complex for students)
- ❌ Password-based auth (adds friction)
- ❌ Physical hardware programming (students remote)
- ❌ Full project copy per student (wastes storage)

---

**End of TODO List**

**Next Steps**:
1. Transfer this repository to Korea server
2. Start Phase 1: Infrastructure Setup
3. Create `auth_manager.py`, `workspace_manager.py`, `template_manager.py`
4. Test locally before deploying

**Contact**: Prof. Hong Jeaong | linkedin.com/in/gnoejh53
GLCD: https://github.com/efthymios-ks/AVR-KS0108