#!/usr/bin/env python3
"""
launch/localization.launch.py
══════════════════════════════
Launches the complete localisation stack for navigation on a pre-built map.

PRE-REQUISITE
─────────────
  You must have a saved map from Step 3:
    maps/my_map.pgm
    maps/my_map.yaml

  Run this launch AFTER Gazebo and sensors are already running, OR
  use bringup.launch.py (Step 7) which combines everything.

WHAT IT STARTS
──────────────
  1. map_server          — loads the .pgm/.yaml map, publishes /map
  2. amcl                — particle filter localiser on that map
  3. lifecycle_manager   — activates map_server + amcl (Nav2 pattern)
  4. ekf_filter_node     — fuses /odom + /imu/filtered → /odometry/filtered
  5. localization_monitor— confidence tracker + operator warnings
  6. rviz2               — map + particle cloud + covariance ellipse view

HOW TO RUN
──────────
  # Replace with your actual map path from Step 3:
  ros2 launch autonomous_nav_bot localization.launch.py \
    map:=$(ros2 pkg prefix autonomous_nav_bot)/share/autonomous_nav_bot/maps/my_map.yaml

  # Shorthand if map is in the package maps/ directory:
  ros2 launch autonomous_nav_bot localization.launch.py map:=office_map.yaml

WHAT HAPPENS ON STARTUP
─────────────────────────
  1. map_server loads the .pgm and publishes it as /map (OccupancyGrid).
  2. AMCL starts with particles spread across the entire map (global localisation).
  3. As LiDAR scans arrive, particles converge to the robot's true position.
  4. This can take 5–30 seconds depending on the map's complexity.
  5. Speed it up: use "2D Pose Estimate" in RViz2 to click the starting pose.

SETTING AN INITIAL POSE FROM THE CLI
──────────────────────────────────────
  ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    '{ header: {frame_id: "map"},
       pose: { pose: {
         position: {x: 1.0, y: 0.5, z: 0.0},
         orientation: {w: 1.0}
       },
       covariance: [0.25, 0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                    0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07] }}' --once

TOPICS TO MONITOR
──────────────────
  /map                        — the loaded occupancy grid
  /particle_cloud             — AMCL hypothesis swarm (visible in RViz2)
  /amcl_pose                  — robot's estimated pose + covariance
  /odometry/filtered          — EKF-fused odometry
  /localization/confidence    — 0.0 (lost) → 1.0 (confident)
  /localization/status        — human-readable diagnostic string
  /tf                         — check map→odom→base_footprint chain

NAV2 LIFECYCLE PATTERN
────────────────────────
  In Nav2, map_server and amcl are "lifecycle nodes" — they go through
  states: Unconfigured → Inactive → Active → Finalized.
  The lifecycle_manager automates this: sets autostart=true so both
  nodes transition to Active automatically when the manager starts.
  Without this, map_server and amcl would start but publish nothing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir        = get_package_share_directory('autonomous_nav_bot')
    ekf_config     = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')
    loc_config     = os.path.join(pkg_dir, 'config', 'localization_params.yaml')
    rviz_config    = os.path.join(pkg_dir, 'rviz',   'localization.rviz')

    # ── Launch arguments ─────────────────────────────────────────────────

    map_arg = DeclareLaunchArgument(
        name='map',
        default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
        description=(
            'Full path to the map YAML file saved during Step 3 mapping. '
            'Example: /ros2_ws/src/autonomous_nav_bot/maps/office_map.yaml'
        )
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Synchronise all nodes to Gazebo simulation clock.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')

    # ── 1. Map Server (lifecycle node) ────────────────────────────────────
    # Loads the .pgm + .yaml map from disk and publishes it on /map.
    # The lifecycle_manager below will call configure() + activate() on it.
    # Without that sequence, map_server loads but publishes nothing.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            loc_config,
            {
                'use_sim_time':   use_sim_time,
                'yaml_filename':  map_file,   # override the empty default
            }
        ]
    )

    # ── 2. AMCL (lifecycle node) ──────────────────────────────────────────
    # Reads /map and /scan/filtered, runs the particle filter,
    # publishes /amcl_pose and /particle_cloud,
    # and broadcasts the map → odom TF transform.
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            loc_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── 3. Nav2 Lifecycle Manager ─────────────────────────────────────────
    # Manages the startup sequence of all Nav2 lifecycle nodes.
    # With autostart=true, it automatically transitions map_server and amcl
    # through: Unconfigured → Inactive → Active.
    # Without this, those nodes are stuck in Unconfigured and do nothing.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    True,
            'node_names':   ['map_server', 'amcl'],
        }]
    )

    # ── 4. EKF Sensor Fusion ──────────────────────────────────────────────
    # Fuses /odom (wheel encoders) with /imu/filtered (gyroscope + accel).
    # Output: /odometry/filtered and the TF odom → base_footprint.
    #
    # This node is NOT a lifecycle node — it starts immediately and runs
    # continuously. Nav2 will use /odometry/filtered as its primary
    # odometry source in the next step.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Nav2 expects odometry on /odometry/filtered by default
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    # ── 5. Localisation Monitor ───────────────────────────────────────────
    # Our custom diagnostic node. Watches particle spread and pose
    # covariance, publishes /localization/confidence, and warns the
    # operator when AMCL is struggling.
    localization_monitor = Node(
        package='autonomous_nav_bot',
        executable='localization_monitor.py',
        name='localization_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 6. RViz2 ──────────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── Startup messages ──────────────────────────────────────────────────
    msg_start = LogInfo(
        msg=(
            '[localization] Starting localisation stack.\n'
            '  Ensure Gazebo and sensor nodes are already running.\n'
            '  Map will load and AMCL particles will spread across it.\n'
            '  Use "2D Pose Estimate" in RViz2 to speed up convergence.'
        )
    )
    msg_ekf   = LogInfo(msg='[localization] EKF filter node started → /odometry/filtered')
    msg_ready = LogInfo(
        msg=(
            '[localization] LOCALISATION STACK READY\n'
            '  Monitor confidence: ros2 topic echo /localization/confidence\n'
            '  Check TF chain:     ros2 run tf2_tools view_frames'
        )
    )

    # Delay monitor + RViz2 until map_server + AMCL are up
    delayed_monitor_and_rviz = TimerAction(
        period=4.0,
        actions=[localization_monitor, rviz2, msg_ready]
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        msg_start,

        # Localisation stack (order matters: map_server before lifecycle_manager)
        map_server,
        amcl,
        lifecycle_manager,

        # EKF runs independently of Nav2 lifecycle
        msg_ekf,
        ekf_node,

        # Monitor + RViz2 after everything is up
        delayed_monitor_and_rviz,
    ])
