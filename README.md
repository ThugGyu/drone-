# 🚁 DRONE Control System

PPO Reinforcement Learning + ROS2 + Gazebo Drone Control System

## ✨ Core Features
- **GUI-based teleoperation** with English interface
- **Real-time 2D map visualization** and mission planning
- **Multiple control methods**: Mouse, keyboard, and button controls
- **Safety features**: Boundary protection and emergency landing
- **PPO reinforcement learning** integration for autonomous flight

## 🎮 System Control Files

### Essential Control Sequence:
1. **`1_start_system.bat`** - System startup (Gazebo + ROS2)
2. **`2_takeoff.bat`** - Drone takeoff to safe altitude
3. **`6_gui_teleop_en.bat`** - GUI control interface (English, recommended)
4. **`4_landing.bat`** - Safe landing and system shutdown

### Additional Controls:
- `0_check_gui_requirements.bat` - GUI requirements checker
- `3_keyboard_control.bat` - Menu-based keyboard control

## 🐍 Core Scripts

| File | Size | Description |
|------|------|-------------|
| `gui_teleop_control_en.py` | 36KB | GUI teleoperation control with map visualization |
| `advanced_teleop_control.py` | 21KB | Advanced real-time teleoperation |
| `check_gui_requirements.py` | - | GUI requirements validation |

## 📋 Project Configuration

- **`package.xml`** - ROS2 package configuration
- **`CMakeLists.txt`** - Build system configuration

## 📖 Documentation

- **`README_Usage_Guide.txt`** - Complete usage guide (Korean)
- **`GUI_Manual_EN.txt`** - GUI user manual (English)
- **`CORE_FILES_SUMMARY.txt`** - Project structure summary

## 📁 Configuration Folders

| Folder | Description |
|--------|-------------|
| `config/` | Configuration files (RVIZ visualization) |
| `launch/` | ROS2 launch files for system startup |
| `maps/` | Simulation environment maps |
| `models/` | Trained PPO models and 3D meshes |
| `install/` | ROS2 installation and build files |

## 🕹️ Control Methods

### 🖥️ GUI Control (Recommended)
- **Mouse**: Click on map to set target position
- **Keyboard**: WASD movement, QE rotation, RF altitude
- **Buttons**: Takeoff, Landing, Emergency controls
- **Features**: Real-time map, mission planning, safety monitoring

### ⌨️ Keyboard Control
- **Menu-based**: Number selection for movements
- **Real-time**: Direct keyboard input for immediate control

### 🎯 Mission Modes
- **Square Mission**: Automated square pattern flight
- **Circle Mission**: Automated circular pattern flight
- **Return Home**: Automatic return to starting position

## ⚙️ System Requirements

- **OS**: Windows 10/11
- **Software**: WSL2, ROS2 Humble, Gazebo Garden
- **Hardware**: NVIDIA GPU (recommended)
- **Python**: 3.8+ with tkinter, numpy, matplotlib

## 🚀 Quick Start

```bash
# 1. Start the system
1_start_system.bat

# 2. Wait 30-60 seconds for initialization

# 3. Takeoff the drone
2_takeoff.bat

# 4. Launch GUI control
6_gui_teleop_en.bat

# 5. Fly and enjoy! 🎮

# 6. Land safely when done
4_landing.bat
```

## 🛡️ Safety Features

- **Boundary Protection**: 20m distance limit from origin
- **Altitude Limits**: 0.5m minimum, 8m maximum
- **Emergency Landing**: Immediate safety landing
- **Real-time Monitoring**: Position, altitude, and status tracking

## 🔧 Troubleshooting

| Issue | Solution |
|-------|----------|
| System won't start | Run `0_check_gui_requirements.bat` |
| GUI doesn't respond | Check X11 display and ROS2 connection |
| Control commands ignored | Verify "Enable Control" is active |
| Mission won't start | Check safety limits and starting position |

## 📞 Support

For technical issues:
1. Check log files in system terminals
2. Review error messages
3. Consult `README_Usage_Guide.txt`
4. Verify system requirements

## ⚠️ Important Notes

- **Always test in simulation before real flight**
- **Follow local aviation regulations**
- **Maintain visual line of sight**
- **Keep emergency procedures ready**

---

**Version**: 1.0 | **Language**: English | **Updated**: 2024

🎯 **Project Status**: Core files optimized and ready for deployment 