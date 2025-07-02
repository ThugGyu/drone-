#!/usr/bin/env python3
"""
🏙️ 도심 환경 가상 드론 시연 런치 파일
ROS2 + RViz + PPO 알고리즘 통합 데모
"""

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
            'urban_city_improved.pgm'
        ]),
        description='Path to the urban city map file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('ppo_drone_ros2'),
            'config',
            'city_demo.rviz'
        ]),
        description='Path to RViz configuration file'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='urban_drone',
        description='Name of the drone in urban environment'
    )
    
    # Set environment variables
    set_env_vars = SetEnvironmentVariable(
        'RCUTILS_LOGGING_SEVERITY', 'INFO'
    )
    
    # Map Server Node - serves the urban city map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': PathJoinSubstitution([
                FindPackageShare('ppo_drone_ros2'),
                'maps',
                'urban_city.yaml'
            ])
        }],
        remappings=[
            ('/map', '/map')
        ]
    )
    
    # Lifecycle Manager for Map Server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # Urban Drone Controller Node
    drone_controller_node = Node(
        package='ppo_drone_ros2',
        executable='urban_drone_controller',
        name='urban_drone_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_name': LaunchConfiguration('robot_name'),
            'urban_environment': True,
            'max_altitude': 50.0,  # Higher altitude for urban environment
            'obstacle_avoidance': True,
            'wind_simulation': True,
            'building_detection': True,
            'emergency_landing_zones': [
                [10.0, 50.0],   # Emergency zone 1
                [90.0, 50.0],   # Emergency zone 2  
                [50.0, 10.0],   # Main landing zone
                [50.0, 90.0]    # Secondary landing zone
            ]
        }],
        remappings=[
            ('/cmd_vel', '/drone/cmd_vel'),
            ('/odom', '/drone/odom'),
            ('/scan', '/drone/scan'),
            ('/image', '/drone/camera/image_raw'),
            ('/pose', '/drone/pose')
        ]
    )
    
    # Urban Environment Simulator
    urban_sim_node = Node(
        package='ppo_drone_ros2',
        executable='urban_environment_sim',
        name='urban_environment_sim',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'building_height_variance': True,
            'wind_effects': True,
            'traffic_simulation': False,  # Can be enabled for advanced simulation
            'weather_effects': False,    # Can be enabled for weather simulation
            'day_night_cycle': False     # Can be enabled for lighting changes
        }]
    )
    
    # Sensor Simulation Nodes
    lidar_sim_node = Node(
        package='ppo_drone_ros2',
        executable='urban_lidar_sim',
        name='urban_lidar_sim',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'range_max': 100.0,         # Extended range for urban environment
            'range_min': 0.1,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.01745,  # 1 degree resolution
            'scan_time': 0.1,
            'building_reflection': True,  # Simulate reflections from buildings
            'multi_echo': True           # Multiple returns from complex surfaces
        }],
        remappings=[
            ('/scan', '/drone/scan')
        ]
    )
    
    camera_sim_node = Node(
        package='ppo_drone_ros2', 
        executable='urban_camera_sim',
        name='urban_camera_sim',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_width': 1280,
            'image_height': 720,
            'fov': 90.0,
            'urban_textures': True,      # Use urban building textures
            'lighting_effects': True,    # Simulate urban lighting
            'shadow_simulation': True    # Building shadows
        }],
        remappings=[
            ('/image', '/drone/camera/image_raw'),
            ('/camera_info', '/drone/camera/camera_info')
        ]
    )
    
    # Navigation Stack for Urban Environment
    urban_planner_node = Node(
        package='ppo_drone_ros2',
        executable='urban_path_planner',
        name='urban_path_planner',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'planning_algorithm': 'rrt_star',  # Good for complex urban environments
            'safety_margin': 2.0,             # Larger margin for urban obstacles
            'altitude_planning': True,         # 3D planning for buildings
            'wind_consideration': True,        # Account for wind corridors
            'building_avoidance': True,        # Specific building avoidance
            'emergency_landing_planning': True # Plan for emergency scenarios
        }],
        remappings=[
            ('/goal_pose', '/drone/goal_pose'),
            ('/plan', '/drone/plan'),
            ('/map', '/map')
        ]
    )
    
    # State Monitor for Urban Operations
    state_monitor_node = Node(
        package='ppo_drone_ros2',
        executable='urban_state_monitor', 
        name='urban_state_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'altitude_limits': [5.0, 50.0],    # Min/max altitude in urban area
            'no_fly_zones': [                  # Define no-fly zones
                {'x': 45.0, 'y': 45.0, 'radius': 5.0, 'reason': 'Hospital'},
                {'x': 80.0, 'y': 20.0, 'radius': 3.0, 'reason': 'School'}
            ],
            'emergency_protocols': True,        # Enable emergency procedures
            'building_proximity_warning': 3.0,  # Warn when within 3m of buildings
            'battery_monitoring': True          # Monitor battery for urban missions
        }]
    )
    
    # RViz for Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Static Transform Publishers for Urban Sensors
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_broadcaster',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    base_to_camera_tf = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='base_to_camera_broadcaster',
        arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        map_file_arg,
        rviz_config_arg,
        robot_name_arg,
        
        # Environment variables
        set_env_vars,
        
        # Core nodes
        map_server_node,
        lifecycle_manager_node,
        
        # Drone and simulation
        drone_controller_node,
        urban_sim_node,
        
        # Sensors
        lidar_sim_node,
        camera_sim_node,
        
        # Navigation and planning
        urban_planner_node,
        state_monitor_node,
        
        # Visualization
        rviz_node,
        
        # Transform publishers
        base_to_lidar_tf,
        base_to_camera_tf,
        map_to_odom_tf
    ])

def get_drone_urdf():
    """드론 URDF 반환"""
    return '''<?xml version="1.0"?>
<robot name="demo_drone">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <link name="camera_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 -0.05" rpy="0 0 0"/>
  </joint>
  
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.06"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>''' 