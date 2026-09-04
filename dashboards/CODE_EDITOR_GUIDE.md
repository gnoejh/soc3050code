# Code Editor Feature Guide
**ATmega128 Project Launcher Dashboard**  
© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
Contact: [linkedin.com/in/gnoejh53](https://linkedin.com/in/gnoejh53)

---

## Overview

The **Code Editor** feature enables students to edit C source files directly in the browser without switching to external editors. This streamlines the learning workflow by integrating code editing, building, and running all in one place.

---

## Features

### ✨ In-Browser Editing
- **Monaco Editor**: Professional code editor (same engine as VS Code)
- **C Syntax Highlighting**: Full syntax highlighting for C and header files
- **Dark Theme**: VS Code dark theme for comfortable reading
- **Line Numbers**: Easy navigation and debugging
- **Auto-completion**: Smart code suggestions
- **Minimap**: Overview of entire file

### 📁 File Management
- **Multiple File Support**: Edit Main.c, Lab.c, and header files
- **Tab System**: Switch between files easily
- **Auto-prioritization**: Main.c and Lab.c appear first
- **File Listing**: Shows all .c and .h files in project

### 💾 Save & Revert
- **Real-time Tracking**: Modified indicator shows unsaved changes
- **Auto-backup**: Creates `.backup` copy before saving
- **Revert Changes**: Undo all edits back to original
- **Save Confirmation**: Visual feedback on successful save

### 🔄 Live Integration
- **Active Function Detection**: Automatically detects which demo is uncommented
- **Project Refresh**: Updates UI after saving to show new active function
- **Build & Run**: Immediately build and test your changes

---

## How to Use

### Step 1: Select a Project
1. Click on any project from the left sidebar
2. Project details will appear in the main panel

### Step 2: Open Code Editor
1. Click the **"📝 Edit Source Code"** button
2. Code editor section appears below project details
3. File tabs show available files (Main.c, Lab.c, etc.)
4. Main.c opens automatically by default

### Step 3: Edit Code
1. **Select File**: Click tab to switch between Main.c and Lab.c
2. **Edit**: Type directly in the editor
3. **Uncomment Function**: Find the function you want to run
   ```c
   int main(void) {
       _init_();
       
       // demo_01_write_port();  // Comment out current function
       demo_02_read_port();      // Uncomment desired function
       
       while(1);
   }
   ```

### Step 4: Save Changes
1. Notice the **"● Modified"** indicator appears
2. Click **"💾 Save"** button
3. Backup created automatically (filename.c.backup)
4. Success message confirms save
5. Project info refreshes to show new active function

### Step 5: Build & Run
1. Click **"▶️ Build & Run"** button
2. Code compiles with your changes
3. Choose SimulIDE or Physical Kit
4. Test your demo function immediately

---

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+S` | Save file (browser default) |
| `Ctrl+F` | Find in code |
| `Ctrl+H` | Find and replace |
| `Ctrl+Z` | Undo |
| `Ctrl+Y` | Redo |
| `Ctrl+/` | Toggle comment |
| `Alt+Up/Down` | Move line up/down |

---

## API Endpoints

### POST /api/file/read
Read file content from project.

**Request:**
```json
{
  "project": "Port_Basic",
  "filename": "Main.c"
}
```

**Response:**
```json
{
  "success": true,
  "content": "// File content here...",
  "filename": "Main.c",
  "path": "W:\\soc3050code\\projects\\Port_Basic\\Main.c"
}
```

### POST /api/file/save
Save edited file content.

**Request:**
```json
{
  "project": "Port_Basic",
  "filename": "Main.c",
  "content": "// Updated code..."
}
```

**Response:**
```json
{
  "success": true,
  "message": "File saved successfully",
  "backup": "W:\\soc3050code\\projects\\Port_Basic\\Main.c.backup"
}
```

### POST /api/file/list
List all editable files in project.

**Request:**
```json
{
  "project": "Port_Basic"
}
```

**Response:**
```json
{
  "success": true,
  "files": [
    {"name": "Main.c", "size": 12543, "ext": ".c"},
    {"name": "Lab.c", "size": 8234, "ext": ".c"},
    {"name": "config.h", "size": 1024, "ext": ".h"}
  ]
}
```

---

## Workflow Example

### Scenario: Student wants to test ADC reading demo

**Traditional Workflow (External Editor):**
1. Open VS Code
2. Navigate to projects/ADC_Basic/Main.c
3. Find main() function
4. Comment out demo_01_basic_read()
5. Uncomment demo_02_multiple_channels()
6. Save file (Ctrl+S)
7. Switch to browser
8. Click Build & Run
9. Test demo

**New Workflow (Integrated Editor):**
1. Click "Edit Source Code" button
2. Scroll to main() in editor
3. Comment out demo_01_basic_read()
4. Uncomment demo_02_multiple_channels()
5. Click Save button
6. Click Build & Run
7. Test demo

**Time Saved**: ~40% fewer steps, no context switching!

---

## Technical Details

### Monaco Editor Configuration
```javascript
monaco.editor.create(element, {
    value: '// Code content',
    language: 'c',
    theme: 'vs-dark',
    automaticLayout: true,
    minimap: { enabled: true },
    fontSize: 14,
    lineNumbers: 'on',
    scrollBeyondLastLine: false,
    readOnly: false
});
```

### File Security
- **Path Validation**: Only allows editing files within projects/ directory
- **Extension Check**: Only .c and .h files shown in tabs
- **Backup System**: Automatic .backup creation before every save
- **No System Files**: Cannot edit files outside workspace

### State Management
```javascript
state = {
    editor: monaco.editor,      // Editor instance
    currentFile: 'Main.c',      // Currently open file
    originalContent: '',        // Content when file opened
    editorModified: false,      // Tracks unsaved changes
    availableFiles: []          // List of editable files
}
```

---

## UI Components

### Editor Section Layout
```
┌─────────────────────────────────────────────────────────┐
│ 📝 Code Editor   [Main.c] [Lab.c]   [↶ Revert] [💾 Save] [✖ Close] │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  1  int main(void) {                                   │
│  2      _init_();                                      │
│  3                                                     │
│  4      // demo_01_write_port();                      │
│  5      demo_02_read_port();  ← Currently active      │
│  6                                                     │
│  7      while(1);                                      │
│  8  }                                                  │
│                                                         │
├─────────────────────────────────────────────────────────┤
│ W:\soc3050code\projects\Port_Basic\Main.c    ● Modified │
└─────────────────────────────────────────────────────────┘
```

### File Tabs
- **Active Tab**: Blue background with white text
- **Inactive Tabs**: Gray background, hover effect
- **Priority Order**: Main.c → Lab.c → Others (alphabetical)

### Status Indicators
- **● Modified**: Orange dot shows unsaved changes
- **File Path**: Shows full path to current file
- **Save Button**: Disabled when no changes, enabled when modified

---

## Troubleshooting

### Problem: Editor doesn't appear
**Solution**: Check browser console (F12) for JavaScript errors. Monaco Editor requires modern browsers (Chrome 90+, Firefox 88+, Edge 90+).

### Problem: File won't save
**Causes**:
- File is read-only (check file permissions)
- Disk full or write-protected
- Project path invalid

**Solution**: Check build output console for error message.

### Problem: Changes don't reflect after save
**Solution**: Click project name in sidebar to reload project info. Active function detection re-parses main().

### Problem: Can't find my file
**Solution**: Only .c and .h files appear in tabs. Assembly (.s) files not editable in browser.

---

## Best Practices

### ✅ Do:
- Save frequently (backup system protects you)
- Use meaningful comments in code
- Uncomment ONE function at a time in main()
- Test each demo before moving to next
- Use Revert if you make mistakes

### ❌ Don't:
- Edit multiple functions in main() at once
- Delete backup files (.backup)
- Modify system files or libraries
- Leave unsaved changes when closing editor
- Edit files outside projects/ directory

---

## Educational Benefits

### For Students:
- **Lower Barrier**: No need to learn VS Code first
- **Faster Iteration**: Edit → Save → Build → Test cycle
- **Visual Feedback**: See which demo is active immediately
- **Safe Experimentation**: Backup system allows risk-free changes
- **Focus on Learning**: Less time on tools, more on concepts

### For Instructors:
- **Guided Workflow**: Students follow standardized process
- **Easy Monitoring**: See exactly which demo students are testing
- **Consistent Environment**: Everyone uses same editor
- **Reduced Support**: No VS Code configuration issues
- **Lab Efficiency**: Students complete exercises faster

---

## Future Enhancements

### Planned Features:
- [ ] **Diff View**: Compare current code with original
- [ ] **Syntax Error Detection**: Real-time error highlighting
- [ ] **Code Formatting**: Auto-format code (Ctrl+Shift+F)
- [ ] **Search Across Files**: Find function definitions project-wide
- [ ] **Undo History**: View and restore previous versions
- [ ] **Collaborative Editing**: Multiple students edit same project (WebRTC)
- [ ] **Code Templates**: Insert common patterns with snippets
- [ ] **Git Integration**: Commit changes from dashboard

---

## Related Documentation

- [PROJECT_LAUNCHER_GUIDE.md](PROJECT_LAUNCHER_GUIDE.md) - Complete dashboard guide
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - API and usage reference
- [FRAMEWORK_GUIDE.md](../FRAMEWORK_GUIDE.md) - ATmega128 development guide
- [PROJECT_CATALOG.md](../PROJECT_CATALOG.md) - All available projects

---

## Support

**Professor**: Hong Jeaong  
**Institution**: Inha University in Tashkent  
**LinkedIn**: [linkedin.com/in/gnoejh53](https://linkedin.com/in/gnoejh53)  
**GitHub**: Check repository for latest updates

---

*Last Updated: January 2025*  
*Version: 1.0.0*
