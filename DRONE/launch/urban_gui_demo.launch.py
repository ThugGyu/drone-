#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('ppo_drone_ros2').find('ppo_drone_ros2')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ppo_drone_ros2'),
            'maps',
            'urban_city.yaml'
        ]),
        description='Path to the urban map file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('ppo_drone_ros2'),
            'config',
            'city_demo.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    # Set environment variables
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([
            FindPackageShare('ppo_drone_ros2'),
            'models'
        ])
    )
    
    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{
            'node_names': ['map_server'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True
        }]
    )
    
    # Static transform publisher (base_link to map)
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    # Static transform publisher (base_link to laser)
    laser_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # GUI Teleop Control Node
    gui_teleop_node = Node(
        package='ppo_drone_ros2',
        executable='gui_teleop_control_en.py',
        name='gui_teleop_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Urban environment simulator (mock drone physics)
    urban_simulator_node = Node(
        package='ppo_drone_ros2',
        executable='urban_drone_simulator.py',
        name='urban_drone_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_file': LaunchConfiguration('map_file')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        rviz_config_arg,
        set_gazebo_model_path,
        map_server_node,
        lifecycle_manager_node,
        static_transform_node,
        laser_transform_node,
        rviz_node,
        gui_teleop_node,
        urban_simulator_node
    ]) 