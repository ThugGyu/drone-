🚁 DRONE Keyboard Control System Usage Guide
==========================================

📋 File Configuration:
=======================
1_start_system.bat       - System startup (Gazebo + ROS2)
2_takeoff.bat            - Takeoff and control activation
3_keyboard_control.bat   - Menu-based keyboard control
4_landing.bat            - Safe landing and control deactivation
5_advanced_teleop.bat    - Advanced real-time teleoperation
6_gui_teleop.bat         - GUI-based graphic control (Korean) 🆕
6_gui_teleop_en.bat      - GUI-based graphic control (English) 🆕
setup_path_fix.bat       - Korean path/UTF-8 problem diagnostic tool 🆕
start_gui_system_en.bat  - Comprehensive system startup (English) 🆕

🚀 Usage Order:
===============
1️⃣ Run "1_start_system.bat"
   ▶ Gazebo simulation and ROS2 bridge will start
   ▶ Please wait about 10 seconds

2️⃣ Run "2_takeoff.bat"  
   ▶ PPO control is activated and drone takes off to 3m altitude

3️⃣ Select control method:
   🎮 Menu-based: Run "3_keyboard_control.bat"
   ⌨️ Real-time: Run "5_advanced_teleop.bat" 
   🖥️ GUI Korean: Run "6_gui_teleop.bat" 🆕
   🖥️ GUI English: Run "6_gui_teleop_en.bat" (Recommended) 🆕
   
   📦 Comprehensive start: "start_gui_system_en.bat" (Full auto execution) 🆕

4️⃣ Run "4_landing.bat" (after flight completion)
   ▶ Drone lands safely and control is deactivated

🎮 Control Method Features:
============================

📱 Menu-based Control (3):
[1] ⬆️  Forward           [2] ⬇️  Backward  
[3] ⬅️  Left              [4] ➡️  Right
[5] 🔺 Up                 [6] 🔻 Down
[7] 🛫 Takeoff (3m)       [8] 🛬 Landing (0.5m)
[9] 🔴 Disable Control    [0] 🟢 Enable Control
[S] 📊 Status Check       [Q] ❌ Exit

⌨️ Real-time Control (5):
W/S: Forward/Backward     R/F: Up/Down
A/D: Left/Right Move      Q/E: Left/Right Rotation
Space: Takeoff/Landing    ESC: Exit

🖥️ GUI Control (6): 🆕
Korean version: 6_gui_teleop.bat
English version: 6_gui_teleop_en.bat (Recommended to prevent character corruption)
• Intuitive graphical interface
• Real-time map and status monitoring  
• Mouse click target setting
• Auto mission function (square/circular flight)
• Keyboard + mouse + button control
• Enhanced safety features
📖 Manual: GUI_Manual_KR.txt (Korean) / GUI_Manual_EN.txt (English)

🗺️ Map Information:
====================
- 50m x 50m flight space
- Surrounded by boundary walls  
- Large building obstacle in center
- 4 pillars at corners
- Additional medium-sized obstacles

⚠️ Precautions:
================
- Start system first, then takeoff
- Wait 2-3 seconds between each command
- Move to safe altitude before landing
- Execute landing command before terminating simulation

🔧 Troubleshooting:
===================
- WSL must be installed
- ROS2 Humble must be installed in WSL
- Gazebo Sim must be installed
- If commands don't work, restart the system

🛠️ Korean Path/UTF-8 Issues:
=============================
❌ Korean path problem: Run setup_path_fix.bat for diagnosis
❌ Character corruption: Use English version (*_en.bat) files recommended
❌ GUI error: Check X11 display setup (setup_x11.bat)

📞 Help:
========
Double-click each .bat file to run.
When menu appears, enter the corresponding number or character and press Enter.

Enjoy your drone flight! 🚁✨ 