#!/usr/bin/env python3
"""
🚁 전체 시스템 런치 파일
MAVROS + 기본 제어 + 자율 비행 시스템 통합 실행

사용법:
ros2 launch real_drone_control full_system.launch.py

옵션:
- fcu_url: Flight Controller 연결 URL
- gcs_url: Ground Control Station 연결 URL  
- simulation: 시뮬레이션 모드 (기본값: false)
- auto_start: 자동 미션 시작 (기본값: false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # 런치 인자들
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',  # SITL 기본값
        description='FCU connection URL (실물 드론: /dev/ttyUSB0:57600 or udp://192.168.1.100:14550)'
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url', 
        default_value='udp://:14550@127.0.0.1:14551',
        description='GCS connection URL'
    )
    
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='시뮬레이션 모드 활성화'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false', 
        description='자동으로 미션 시작'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='디버그 모드 활성화'
    )
    
    # MAVROS 노드
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
            'respawn_mavros': True,
        }],
        condition=UnlessCondition(LaunchConfiguration('simulation'))
    )
    
    # 시뮬레이션용 MAVROS (PX4 SITL)
    mavros_sitl_node = Node(
        package='mavros',
        executable='mavros_node', 
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': 'udp://:14540@127.0.0.1:14557',
            'gcs_url': 'udp://:14550@127.0.0.1:14551',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
        }],
        condition=IfCondition(LaunchConfiguration('simulation'))
    )
    
    # 기본 드론 컨트롤러
    basic_controller_node = Node(
        package='real_drone_control',
        executable='basic_drone_controller.py',
        name='basic_drone_controller',
        output='screen',
        parameters=[{
            'debug_mode': LaunchConfiguration('debug'),
        }]
    )
    
    # 자율 비행 노드 (5초 후 시작)
    autonomous_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='real_drone_control',
                executable='autonomous_flight_node.py',
                name='autonomous_flight_node',
                output='screen',
                parameters=[{
                    'auto_start_mission': LaunchConfiguration('auto_start'),
                    'debug_mode': LaunchConfiguration('debug'),
                }]
            )
        ]
    )
    
    # RViz2 시각화 (디버그 모드에서만)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'src/real_drone_control/config/drone_visualization.rviz'],
        condition=IfCondition(LaunchConfiguration('debug'))
    )
    
    # 모니터링 대시보드 (옵션)
    dashboard_node = Node(
        package='real_drone_control',
        executable='monitoring_dashboard.py',
        name='monitoring_dashboard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug'))
    )
    
    # QGroundControl 자동 실행 (시뮬레이션 모드에서만)
    qgc_process = ExecuteProcess(
        cmd=['QGroundControl'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulation'))
    )
    
    return LaunchDescription([
        # 런치 인자들
        fcu_url_arg,
        gcs_url_arg,
        simulation_arg,
        auto_start_arg,
        debug_arg,
        
        # 노드들
        mavros_node,
        mavros_sitl_node,
        basic_controller_node,
        autonomous_node,
        rviz_node,
        dashboard_node,
        
        # 외부 프로세스
        qgc_process,
    ]) 