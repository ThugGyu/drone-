@echo off
title Drone Keyboard Control
color 0A

:menu
cls
echo.
echo ╔═══════════════════════════════════════════════════════════════╗
echo ║                    🚁 Drone Keyboard Control                  ║
echo ╠═══════════════════════════════════════════════════════════════╣
echo ║                                                               ║
echo ║  [1] ⬆️  Forward         [2] ⬇️  Backward                      ║
echo ║  [3] ⬅️  Left            [4] ➡️  Right                         ║
echo ║  [5] 🔺 Up               [6] 🔻 Down                           ║
echo ║                                                               ║
echo ║  [7] 🛫 Takeoff (3m)     [8] 🛬 Landing (0.5m)                ║
echo ║  [9] 🔴 Disable Control  [0] 🟢 Enable Control                ║
echo ║                                                               ║
echo ║  [S] 📊 Status Check     [Q] ❌ Exit                          ║
echo ║                                                               ║
echo ╚═══════════════════════════════════════════════════════════════╝
echo.
set /p choice="Select command: "

if "%choice%"=="1" (
    echo 🚁 Moving forward...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_target geometry_msgs/msg/Point '{x: 0.0, y: 3.0, z: 3.0}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if "%choice%"=="2" (
    echo 🚁 Moving backward...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_target geometry_msgs/msg/Point '{x: 0.0, y: -3.0, z: 3.0}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if "%choice%"=="3" (
    echo 🚁 Moving left...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_target geometry_msgs/msg/Point '{x: -3.0, y: 0.0, z: 3.0}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if "%choice%"=="4" (
    echo 🚁 Moving right...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_target geometry_msgs/msg/Point '{x: 3.0, y: 0.0, z: 3.0}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if "%choice%"=="5" (
    echo 🚁 Moving up...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_target geometry_msgs/msg/Point '{x: 0.0, y: 0.0, z: 5.0}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if "%choice%"=="6" (
    echo 🚁 Moving down...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_target geometry_msgs/msg/Point '{x: 0.0, y: 0.0, z: 1.5}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if "%choice%"=="7" (
    echo 🛫 Taking off...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_target geometry_msgs/msg/Point '{x: 0.0, y: 0.0, z: 3.0}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if "%choice%"=="8" (
    echo 🛬 Landing...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_target geometry_msgs/msg/Point '{x: 0.0, y: 0.0, z: 0.5}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if "%choice%"=="9" (
    echo 🔴 Disabling control...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_enable std_msgs/msg/Bool '{data: false}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if "%choice%"=="0" (
    echo 🟢 Enabling control...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic pub /ppo_enable std_msgs/msg/Bool '{data: true}' --once"
    timeout /t 2 /nobreak >nul
    goto menu
)

if /i "%choice%"=="s" (
    echo 📊 Checking drone status...
    wsl bash -c "cd /mnt/c/Users/kym70/OneDrive/바탕\ 화면/coding/DRONE && source /opt/ros/humble/setup.bash && ros2 topic echo /drone_status --once"
    pause
    goto menu
)

if /i "%choice%"=="q" (
    echo 👋 Exiting program.
    exit
)

echo ❌ Invalid selection.
timeout /t 2 /nobreak >nul
goto menu 