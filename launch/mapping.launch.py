#!/usr/bin/env python3
"""
launch/mapping.launch.py
═════════════════════════
Launches a complete SLAM mapping session.

WHAT IT STARTS  (in order of dependency)
──────────────────────────────────────────
  1. Gazebo          — physics simulation with an empty world
  2. robot_state_publisher  — TF tree from URDF
  3. spawn_entity    — places the robot in Gazebo
  4. lidar_processor — filters /scan → /scan/filtered     (Step 2)
  5. imu_processor   — filters /imu  → /imu/filtered      (Step 2)
  6. camera_processor— validates camera stream            (Step 2)
  7. slam_toolbox    — builds the map from /scan/filtered  (Step 3 ← new)
  8. map_saver       — ready to save map on command        (Step 3 ← new)
  9. RViz2           — live map visualisation

HOW TO RUN
──────────
  ros2 launch autonomous_nav_bot mapping.launch.py

  # Use a specific Gazebo world:
  ros2 launch autonomous_nav_bot mapping.launch.py world:=/path/to/world.sdf

DRIVING THE ROBOT TO BUILD THE MAP
────────────────────────────────────
  # In a NEW terminal (after launching):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    --remap cmd_vel:=/cmd_vel

  Arrow keys / WASD to drive. Watch the map grow in RViz2.

SAVING THE MAP WHEN DONE
─────────────────────────
  # Option 1 — topic (specify a name):
  ros2 topic pub /save_map std_msgs/msg/String \
    "{data: 'office_floor_1'}" --once

  # Option 2 — service (auto-timestamped name):
  ros2 service call /save_map_service std_srvs/srv/Trigger

  Map files appear in: <package>/maps/

WHAT TO LOOK FOR IN RVIZ2
──────────────────────────
  • Grey cells  = unexplored (robot hasn't scanned here yet)
  • White cells = free space (confirmed empty by laser)
  • Black cells = obstacles  (walls, furniture, etc.)
  • Orange dots = live LiDAR scan overlay
  • Blue arrow  = robot current pose
  • Green line  = robot trajectory so far

SLAM TOPIC REFERENCE
─────────────────────
  /map             → nav_msgs/OccupancyGrid   — the live occupancy grid
  /scan/filtered   → sensor_msgs/LaserScan    — cleaned LiDAR input to SLAM
  /slam_toolbox/pose_graph → visualisation of the pose graph nodes + edges
  /map_saver/status → std_msgs/String         — save success/failure feedback
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_dir        = get_package_share_directory('autonomous_nav_bot')
    urdf_file      = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    sensors_config = os.path.join(pkg_dir, 'config', 'sensors.yaml')
    slam_config    = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    rviz_config    = os.path.join(pkg_dir, 'rviz',   'mapping.rviz')

    # ── Launch arguments ─────────────────────────────────────────────────

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='',
        description=(
            'Path to a .world or .sdf Gazebo world file. '
            'Leave empty to use the Gazebo default empty world.'
        )
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='All nodes synchronise to Gazebo simulation clock.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world        = LaunchConfiguration('world')

    # ── 1. Gazebo ─────────────────────────────────────────────────────────
    # IncludeLaunchDescription re-uses gazebo_ros's own launch file,
    # so we don't have to manage the Gazebo process directly.
    # 'verbose': true prints Gazebo engine output (useful for debugging).
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world':   world,
            'verbose': 'false',
            'pause':   'false',
        }.items()
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────
    # Processes the XACRO → URDF at launch time and broadcasts TF.
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'robot_description':  robot_description,
        }]
    )

    # ── 3. Spawn Robot in Gazebo ──────────────────────────────────────────
    # spawn_entity.py reads the URDF from the /robot_description topic
    # (which robot_state_publisher publishes) and creates the robot model
    # in the Gazebo physics world at position (0, 0, 0.05).
    # z=0.05 avoids the robot spawning below the floor.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'autonomous_nav_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',     # spawn slightly above floor to avoid physics glitch
            '-Y', '0.0',      # initial yaw (radians)
        ]
    )

    # ── 4–6. Sensor Processing Nodes ─────────────────────────────────────
    # Launched with a small delay so Gazebo is fully up before sensors start.
    lidar_processor = Node(
        package='autonomous_nav_bot',
        executable='lidar_processor.py',
        name='lidar_processor',
        output='screen',
        parameters=[sensors_config, {'use_sim_time': use_sim_time}]
    )

    imu_processor = Node(
        package='autonomous_nav_bot',
        executable='imu_processor.py',
        name='imu_processor',
        output='screen',
        parameters=[sensors_config, {'use_sim_time': use_sim_time}]
    )

    camera_processor = Node(
        package='autonomous_nav_bot',
        executable='camera_processor.py',
        name='camera_processor',
        output='screen',
        parameters=[sensors_config, {'use_sim_time': use_sim_time}]
    )

    # ── 7. SLAM Toolbox ───────────────────────────────────────────────────
    # async_slam_toolbox_node:
    #   "async" = scans are processed as fast as they arrive,
    #   decoupled from the map update rate. This prevents blocking the
    #   ROS 2 callback queue when the solver is working on a loop closure.
    #
    # Reads slam_params.yaml for all configuration (scan topic, resolution,
    # loop closure parameters, etc.).
    #
    # Publishes:
    #   /map               → nav_msgs/OccupancyGrid (the map, 1 Hz)
    #   /map_metadata      → nav_msgs/MapMetaData
    #   /pose              → geometry_msgs/PoseWithCovarianceStamped
    #   /slam_toolbox/...  → diagnostic topics
    # Broadcasts:
    #   TF: map → odom  (the correction transform slam_toolbox computes)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── 8. Map Saver ──────────────────────────────────────────────────────
    # Sits idle until triggered. Once the map looks good, the user
    # can trigger a save via topic or service (see docstring above).
    map_saver = Node(
        package='autonomous_nav_bot',
        executable='map_saver.py',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 9. RViz2 ──────────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── Startup messages ──────────────────────────────────────────────────
    # LogInfo statements appear in the terminal as the launch progresses,
    # making it easy to confirm each stage is reached.
    msg_gazebo   = LogInfo(msg='[mapping] ① Starting Gazebo simulation...')
    msg_rsp      = LogInfo(msg='[mapping] ② Starting robot_state_publisher...')
    msg_spawn    = LogInfo(msg='[mapping] ③ Spawning robot in Gazebo...')
    msg_sensors  = LogInfo(msg='[mapping] ④ Starting sensor processing nodes...')
    msg_slam     = LogInfo(msg='[mapping] ⑤ Starting SLAM Toolbox (async)...')
    msg_saver    = LogInfo(msg='[mapping] ⑥ Map Saver ready.')
    msg_rviz     = LogInfo(msg='[mapping] ⑦ Opening RViz2 for map visualisation...')
    msg_drive    = LogInfo(
        msg=(
            '[mapping] READY TO MAP!\n'
            '  Drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
            '  Save map:   ros2 topic pub /save_map std_msgs/msg/String '
            '"{data: \'my_map\'}" --once'
        )
    )

    # ── Delay sensor / SLAM startup ───────────────────────────────────────
    # TimerAction delays node startup so Gazebo is fully initialised.
    # Without this, sensor nodes try to subscribe to topics that don't
    # exist yet and print confusing "no publishers" warnings.
    delayed_sensors_and_slam = TimerAction(
        period=3.0,    # seconds — Gazebo usually loads in ~2s
        actions=[
            msg_sensors,
            lidar_processor,
            imu_processor,
            camera_processor,
            msg_slam,
            slam_toolbox,
            msg_saver,
            map_saver,
            msg_rviz,
            rviz2,
            msg_drive,
        ]
    )

    return LaunchDescription([
        # Arguments first
        world_arg,
        use_sim_time_arg,

        # Stage 1: Gazebo + robot description (simultaneous)
        msg_gazebo,
        gazebo,
        msg_rsp,
        robot_state_publisher,

        # Stage 2: Spawn robot after RSP is publishing /robot_description
        msg_spawn,
        spawn_robot,

        # Stage 3: Sensors + SLAM after Gazebo stabilises
        delayed_sensors_and_slam,
    ])
