#!/usr/bin/env python3
"""
🚁 PPO MAVROS 런치 파일
훈련된 PPO 모델과 MAVROS를 함께 실행
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 런치 인수 정의
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',
        description='FCU 연결 URL (시뮬레이션용 기본값)'
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@127.0.0.1:14550',
        description='GCS 연결 URL'
    )
    
    target_system_arg = DeclareLaunchArgument(
        'target_system',
        default_value='1',
        description='MAVLink 시스템 ID'
    )
    
    target_component_arg = DeclareLaunchArgument(
        'target_component',
        default_value='1',
        description='MAVLink 컴포넌트 ID'
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
            'target_system_id': LaunchConfiguration('target_system'),
            'target_component_id': LaunchConfiguration('target_component'),
            'fcu_protocol': 'v2.0',
            'system_id': 255,
            'component_id': 240,
            'startup_px4_usb_quirk': True,
        }],
        respawn=True,
        respawn_delay=2
    )
    
    # PPO 제어기 노드 (MAVROS 시작 후 5초 대기)
    ppo_controller_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ppo_drone_ros2',
                executable='mavros_ppo_controller.py',
                name='ppo_controller',
                output='screen',
                parameters=[{
                    'control_rate': 20,
                    'action_scale': 2.0,
                    'target_x': 0.0,
                    'target_y': 0.0,
                    'target_z': 5.0,
                }],
                respawn=True,
                respawn_delay=2
            )
        ]
    )
    
    # RViz2 (선택적)
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ppo_drone_ros2'),
        'config',
        'drone_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition='false'  # 기본적으로 비활성화
    )
    
    return LaunchDescription([
        # 런치 인수들
        fcu_url_arg,
        gcs_url_arg,
        target_system_arg,
        target_component_arg,
        
        # 노드들
        mavros_node,
        ppo_controller_node,
        # rviz_node,  # 필요시 주석 해제
    ]) 