#!/usr/bin/env python3
"""
launch/sensors.launch.py
═════════════════════════
Starts all sensor processing nodes in a single launch command.

WHAT IT STARTS
──────────────
  1. lidar_processor   — filters /scan → /scan/filtered
  2. imu_processor     — filters /imu  → /imu/filtered
  3. camera_processor  — validates /camera/image_raw → /camera/image_processed

All three nodes load their parameters from config/sensors.yaml.

HOW TO RUN
──────────
  # After building and sourcing:
  ros2 launch autonomous_nav_bot sensors.launch.py

  # With Gazebo already running (use simulation clock):
  ros2 launch autonomous_nav_bot sensors.launch.py use_sim_time:=true

  # Verify topics are publishing:
  ros2 topic list | grep -E 'filtered|processed'
  ros2 topic hz /scan/filtered
  ros2 topic hz /imu/filtered

  # Check diagnostics:
  ros2 topic echo /scan/stats
  ros2 topic echo /imu/stats
  ros2 topic echo /camera/stats

TOPIC FLOW AFTER THIS LAUNCH
─────────────────────────────

  Gazebo / Hardware
      │
      ├─ /scan           ──► [lidar_processor]  ──► /scan/filtered
      │
      ├─ /imu            ──► [imu_processor]    ──► /imu/filtered
      │
      └─ /camera/image_raw ► [camera_processor] ──► /camera/image_processed

  Downstream consumers:
      /scan/filtered        → slam_toolbox (Step 3), Nav2 costmaps (Step 5)
      /imu/filtered         → robot_localization EKF (Step 4)
      /camera/image_processed → vision pipeline (future)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir       = get_package_share_directory('autonomous_nav_bot')
    sensors_config = os.path.join(pkg_dir, 'config', 'sensors.yaml')

    # ── Launch arguments ─────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',   # default true because this runs alongside Gazebo
        description='Use Gazebo simulation clock. Set false for real hardware.'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Node 1: LiDAR Processor ──────────────────────────────────────────
    # Reads raw /scan, filters NaN/out-of-range, applies circular
    # moving average, publishes clean scan on /scan/filtered
    lidar_processor_node = Node(
        package='autonomous_nav_bot',
        executable='lidar_processor.py',  # installed to lib/autonomous_nav_bot/
        name='lidar_processor',
        output='screen',
        parameters=[
            sensors_config,                    # load params from YAML
            {'use_sim_time': use_sim_time}     # override sim time setting
        ],
        # Remap the output if a downstream node expects a different name.
        # Leave empty here — SLAM and Nav2 use /scan/filtered directly.
        remappings=[]
    )

    # ── Node 2: IMU Processor ────────────────────────────────────────────
    # Reads raw /imu, rejects spikes, applies moving average per axis,
    # publishes clean readings on /imu/filtered
    imu_processor_node = Node(
        package='autonomous_nav_bot',
        executable='imu_processor.py',
        name='imu_processor',
        output='screen',
        parameters=[
            sensors_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[]
    )

    # ── Node 3: Camera Processor ─────────────────────────────────────────
    # Validates image dimensions and encoding, acts as QoS bridge
    # (BEST_EFFORT from Gazebo → RELIABLE for downstream vision nodes),
    # publishes on /camera/image_processed
    camera_processor_node = Node(
        package='autonomous_nav_bot',
        executable='camera_processor.py',
        name='camera_processor',
        output='screen',
        parameters=[
            sensors_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[]
    )

    # ── Startup message ───────────────────────────────────────────────────
    startup_msg = LogInfo(
        msg='[sensors.launch] Starting sensor processing nodes. '
            'Ensure Gazebo or hardware drivers are running first.'
    )

    return LaunchDescription([
        use_sim_time_arg,
        startup_msg,
        lidar_processor_node,
        imu_processor_node,
        camera_processor_node,
    ])
