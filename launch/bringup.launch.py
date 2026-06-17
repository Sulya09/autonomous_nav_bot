#!/usr/bin/env python3
"""
launch/bringup.launch.py
══════════════════════════
Master launch file — starts the complete autonomous navigation system.

USAGE
─────
  # Full system with a saved map:
  ros2 launch autonomous_nav_bot bringup.launch.py \
    map:=/path/to/your/map.yaml

  # With a specific Gazebo world:
  ros2 launch autonomous_nav_bot bringup.launch.py \
    map:=/path/to/map.yaml \
    world:=/path/to/world.sdf

  # With real-time wall clock (for real hardware, not simulation):
  ros2 launch autonomous_nav_bot bringup.launch.py \
    map:=/path/to/map.yaml \
    use_sim_time:=false

WHAT THIS REPLACES
──────────────────
  Without bringup.launch.py you would need 5 terminals:
    Terminal 1: ros2 launch gazebo_ros gazebo.launch.py
    Terminal 2: ros2 launch autonomous_nav_bot sensors.launch.py
    Terminal 3: ros2 launch autonomous_nav_bot localization.launch.py
    Terminal 4: ros2 launch autonomous_nav_bot navigation.launch.py
    Terminal 5: [motor control nodes manually]

  Now: one command in one terminal.

5-PHASE STARTUP SEQUENCE
──────────────────────────
  The robot stack has strict dependency ordering. Each phase waits for
  the previous one to be ready before starting:

  ┌──────────────────────────────────────────────────────────────────┐
  │ PHASE 1  t=0s   Simulation                                       │
  │          Gazebo physics engine + robot description TF tree        │
  ├──────────────────────────────────────────────────────────────────┤
  │ PHASE 2  t=3s   Robot in world + sensors                         │
  │          spawn_entity + lidar/imu/camera processor nodes          │
  ├──────────────────────────────────────────────────────────────────┤
  │ PHASE 3  t=7s   Localisation                                     │
  │          map_server → AMCL → lifecycle_manager + EKF node         │
  ├──────────────────────────────────────────────────────────────────┤
  │ PHASE 4  t=13s  Navigation                                       │
  │          Full Nav2 stack → lifecycle_manager_navigation           │
  ├──────────────────────────────────────────────────────────────────┤
  │ PHASE 5  t=20s  Control + UI                                     │
  │          emergency_stop → velocity_controller → goal_navigator    │
  │          → waypoint_navigator → RViz2                             │
  └──────────────────────────────────────────────────────────────────┘

SENDING NAVIGATION GOALS
─────────────────────────
  After launch completes (≈ 20 seconds):

  Option A — RViz2:
    1. Use "2D Pose Estimate" to click the robot's starting location
    2. Watch the green particle cloud converge
    3. Use "2D Nav Goal" to click a destination

  Option B — CLI single goal:
    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
      '{ header: {frame_id: "map"},
         pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}' --once

  Option C — Waypoint route:
    ros2 topic pub /waypoints geometry_msgs/msg/PoseArray \
      '{ header: {frame_id: "map"},
         poses: [
           {position: {x:1.0, y:0.5}, orientation:{w:1.0}},
           {position: {x:3.0, y:2.0}, orientation:{w:1.0}},
           {position: {x:0.0, y:0.0}, orientation:{w:1.0}}
         ]}' --once

SAVING A MAP DURING THIS SESSION
──────────────────────────────────
  bringup.launch.py starts in NAVIGATION mode (requires a pre-saved map).
  For mapping, use mapping.launch.py instead:
    ros2 launch autonomous_nav_bot mapping.launch.py

EMERGENCY STOP
───────────────
  The emergency_stop node fires automatically when any obstacle is
  < 0.20m from the robot. To manually trigger:
    ros2 topic pub /emergency_stop std_msgs/msg/Bool '{data: true}' --once

  To clear manual e-stop:
    ros2 topic pub /emergency_stop std_msgs/msg/Bool '{data: false}' --once

KEY MONITORING TOPICS
──────────────────────
  /navigation/status          — goal_navigator status
  /localization/confidence    — AMCL confidence [0.0–1.0]
  /control/zone               — 'CLEAR' / 'CAUTION' / 'DANGER'
  /control/status             — velocity_controller diagnostics
  /scan/stats                 — LiDAR filter stats
  /map_saver/status           — save operation results
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    pkg_dir        = get_package_share_directory('autonomous_nav_bot')

    # ── Config files (single source of truth) ────────────────────────────
    urdf_file   = os.path.join(pkg_dir, 'urdf',   'robot.urdf.xacro')
    sensors_cfg = os.path.join(pkg_dir, 'config', 'sensors.yaml')
    ekf_cfg     = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')
    loc_cfg     = os.path.join(pkg_dir, 'config', 'localization_params.yaml')
    nav2_cfg    = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    ctrl_cfg    = os.path.join(pkg_dir, 'config', 'control_params.yaml')
    rviz_cfg    = os.path.join(pkg_dir, 'rviz',   'navigation.rviz')

    # ── Launch arguments ─────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
        description='Full path to the map YAML file saved during SLAM mapping'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to Gazebo world file (empty = default empty world)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo sim clock (true) or wall clock (false)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')
    world_file   = LaunchConfiguration('world')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ══════════════════════════════════════════════════════════════════════
    # PHASE 1 — SIMULATION (t = 0 s)
    # Physics engine + robot description transform broadcaster
    # ══════════════════════════════════════════════════════════════════════
    phase1_msg = LogInfo(msg='[bringup] ▶ PHASE 1 — Starting Gazebo + RSP')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': robot_description,
        }]
    )

    # ══════════════════════════════════════════════════════════════════════
    # PHASE 2 — ROBOT + SENSORS (t = 3 s)
    # Spawn the URDF model and start all sensor processing nodes
    # ══════════════════════════════════════════════════════════════════════
    phase2_msg = LogInfo(msg='[bringup] ▶ PHASE 2 — Spawning robot + starting sensors')

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'autonomous_nav_bot',
            '-x', '0.0', '-y', '0.0', '-z', '0.05', '-Y', '0.0',
        ]
    )

    lidar_processor = Node(
        package='autonomous_nav_bot', executable='lidar_processor.py',
        name='lidar_processor', output='screen',
        parameters=[sensors_cfg, {'use_sim_time': use_sim_time}]
    )
    imu_processor = Node(
        package='autonomous_nav_bot', executable='imu_processor.py',
        name='imu_processor', output='screen',
        parameters=[sensors_cfg, {'use_sim_time': use_sim_time}]
    )
    camera_processor = Node(
        package='autonomous_nav_bot', executable='camera_processor.py',
        name='camera_processor', output='screen',
        parameters=[sensors_cfg, {'use_sim_time': use_sim_time}]
    )

    phase2 = TimerAction(period=3.0, actions=[
        phase2_msg, spawn_robot,
        lidar_processor, imu_processor, camera_processor,
    ])

    # ══════════════════════════════════════════════════════════════════════
    # PHASE 3 — LOCALISATION (t = 7 s)
    # Load the map, run AMCL particle filter, fuse sensors with EKF
    # ══════════════════════════════════════════════════════════════════════
    phase3_msg = LogInfo(
        msg='[bringup] ▶ PHASE 3 — Starting localisation stack\n'
            '           Use "2D Pose Estimate" in RViz2 once map appears.'
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[loc_cfg, {'use_sim_time': use_sim_time,
                               'yaml_filename': map_file}]
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        parameters=[loc_cfg, {'use_sim_time': use_sim_time}]
    )
    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server', 'amcl']}]
    )
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_cfg, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )
    localization_monitor = Node(
        package='autonomous_nav_bot', executable='localization_monitor.py',
        name='localization_monitor', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    phase3 = TimerAction(period=7.0, actions=[
        phase3_msg, map_server, amcl, lifecycle_manager_loc,
        ekf_node, localization_monitor,
    ])

    # ══════════════════════════════════════════════════════════════════════
    # PHASE 4 — NAVIGATION (t = 13 s)
    # Full Nav2 stack: planner, controller, costmaps, behaviors
    # ══════════════════════════════════════════════════════════════════════
    phase4_msg = LogInfo(msg='[bringup] ▶ PHASE 4 — Starting Nav2 navigation stack')

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_cfg, {'use_sim_time': use_sim_time}]
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_cfg, {'use_sim_time': use_sim_time}]
    )
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_cfg, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel_nav')]
    )
    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server',
        name='smoother_server', output='screen',
        parameters=[nav2_cfg, {'use_sim_time': use_sim_time}]
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_cfg, {'use_sim_time': use_sim_time}]
    )
    waypoint_follower_server = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen',
        parameters=[nav2_cfg, {'use_sim_time': use_sim_time}]
    )
    # Velocity smoother: /cmd_vel_nav → (smooth) → /cmd_vel_raw
    # velocity_controller: /cmd_vel_raw → (safe) → /cmd_vel
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[nav2_cfg, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel',          '/cmd_vel_nav'),   # input from Nav2
            ('cmd_vel_smoothed', '/cmd_vel_raw'),   # output to velocity_controller
        ]
    )
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 'autostart': True,
            'bond_timeout': 4.0,
            'node_names': [
                'planner_server', 'controller_server', 'smoother_server',
                'behavior_server', 'waypoint_follower', 'velocity_smoother',
                'bt_navigator',
            ],
        }]
    )

    phase4 = TimerAction(period=13.0, actions=[
        phase4_msg,
        bt_navigator, planner_server, controller_server,
        smoother_server, behavior_server, waypoint_follower_server,
        velocity_smoother, lifecycle_manager_nav,
    ])

    # ══════════════════════════════════════════════════════════════════════
    # PHASE 5 — CONTROL + UI (t = 20 s)
    # Safety layer + operator interface
    # ══════════════════════════════════════════════════════════════════════
    phase5_msg = LogInfo(
        msg=(
            '[bringup] ▶ PHASE 5 — Starting control layer + RViz2\n'
            '\n'
            '  ╔══════════════════════════════════════════════════════╗\n'
            '  ║  SYSTEM READY — autonomous_nav_bot is online        ║\n'
            '  ║                                                      ║\n'
            '  ║  1. Set initial pose (2D Pose Estimate in RViz2)    ║\n'
            '  ║  2. Watch particles converge                         ║\n'
            '  ║  3. Send goal (2D Nav Goal in RViz2)                ║\n'
            '  ║                                                      ║\n'
            '  ║  Monitor: ros2 topic echo /navigation/status        ║\n'
            '  ║  E-stop:  ros2 topic pub /emergency_stop \\          ║\n'
            '  ║             std_msgs/msg/Bool "{data: true}" --once  ║\n'
            '  ╚══════════════════════════════════════════════════════╝'
        )
    )

    emergency_stop = Node(
        package='autonomous_nav_bot', executable='emergency_stop.py',
        name='emergency_stop', output='screen',
        parameters=[ctrl_cfg, {'use_sim_time': use_sim_time}]
    )
    velocity_controller = Node(
        package='autonomous_nav_bot', executable='velocity_controller.py',
        name='velocity_controller', output='screen',
        parameters=[ctrl_cfg, {'use_sim_time': use_sim_time}]
    )
    goal_navigator = Node(
        package='autonomous_nav_bot', executable='goal_navigator.py',
        name='goal_navigator', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    waypoint_navigator = Node(
        package='autonomous_nav_bot', executable='waypoint_follower.py',
        name='waypoint_navigator', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    map_saver = Node(
        package='autonomous_nav_bot', executable='map_saver.py',
        name='map_saver', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    rviz2 = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    phase5 = TimerAction(period=20.0, actions=[
        phase5_msg,
        emergency_stop, velocity_controller,
        goal_navigator, waypoint_navigator,
        map_saver, rviz2,
    ])

    # ── Assemble launch description ───────────────────────────────────────
    return LaunchDescription([
        # Arguments
        map_arg, world_arg, use_sim_time_arg,

        # Phase 1 — immediate
        phase1_msg, gazebo, robot_state_publisher,

        # Phases 2–5 — time-staggered
        phase2, phase3, phase4, phase5,
    ])
