#!/usr/bin/env python3
"""
launch/navigation.launch.py
════════════════════════════
Launches the complete Nav2 autonomous navigation stack.

PRE-REQUISITES (must already be running)
──────────────────────────────────────────
  • Gazebo with the robot spawned
  • robot_state_publisher
  • Sensor processing nodes (lidar_processor, imu_processor, camera_processor)
  • Localisation stack (map_server + AMCL + EKF)
  • The robot must be localised (particle cloud converged) before sending goals

  OR use bringup.launch.py (Step 7) which starts everything together.

WHAT IT STARTS
──────────────
  Nav2 server nodes (all lifecycle-managed):
    bt_navigator      — Behavior Tree orchestrator
    planner_server    — Global path planner (NavFn)
    controller_server — Local trajectory follower (DWB)
    smoother_server   — Path smoothing
    behavior_server   — Recovery behaviours
    waypoint_follower — Multi-waypoint sequences
    velocity_smoother — Smooth cmd_vel output

  Our custom nodes:
    goal_navigator    — Bridges /goal_pose → NavigateToPose action
    waypoint_navigator— Bridges /waypoints → FollowWaypoints action

  lifecycle_manager — activates all Nav2 nodes in correct order

  rviz2 — pre-configured with costmaps, path display, Nav Goal tool

HOW TO RUN
──────────
  ros2 launch autonomous_nav_bot navigation.launch.py

  # With a specific map (if not already loaded by localization.launch.py):
  ros2 launch autonomous_nav_bot navigation.launch.py

HOW TO SEND GOALS
──────────────────
  Option A — RViz2:
    Click "2D Nav Goal" in the toolbar → click on the map.

  Option B — CLI:
    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
      '{ header: {frame_id: "map"},
         pose: {position: {x: 2.0, y: 1.0, z: 0.0},
                orientation: {w: 1.0}}}' --once

  Option C — Waypoints:
    ros2 topic pub /waypoints geometry_msgs/msg/PoseArray \
      '{ header: {frame_id: "map"},
         poses: [
           {position: {x:1.0,y:0.5,z:0.0}, orientation:{w:1.0}},
           {position: {x:3.0,y:2.0,z:0.0}, orientation:{w:1.0}}
         ]}' --once

KEY TOPICS AFTER LAUNCH
────────────────────────
  /plan                         → nav_msgs/Path     — global planned path
  /cmd_vel                      → geometry_msgs/Twist — final robot command
  /global_costmap/costmap       → nav_msgs/OccupancyGrid
  /local_costmap/costmap        → nav_msgs/OccupancyGrid
  /navigate_to_pose/_action/... — NavigateToPose action interface
  /navigation/status            → std_msgs/String   — goal_navigator status
  /waypoint/status              → std_msgs/String   — waypoint_navigator status
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir      = get_package_share_directory('autonomous_nav_bot')
    nav2_config  = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config  = os.path.join(pkg_dir, 'rviz',   'navigation.rviz')

    # ── Launch arguments ─────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Synchronise to Gazebo clock.'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Nav2 server nodes (all lifecycle nodes) ───────────────────────────
    # These nodes go through: Unconfigured → Inactive → Active
    # managed by the lifecycle_manager below.

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
        # Remap: DWB publishes to /cmd_vel, velocity_smoother subscribes to it
        remappings=[('cmd_vel', '/cmd_vel_nav')]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}]
    )

    waypoint_follower_server = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}]
    )

    # ── Velocity Smoother ─────────────────────────────────────────────────
    # Takes /cmd_vel_nav from the controller and outputs smooth /cmd_vel
    # that respects acceleration limits. The robot's diff_drive plugin
    # reads /cmd_vel.
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel',     '/cmd_vel_nav'),   # input from controller
            ('cmd_vel_smoothed', '/cmd_vel'),  # output to robot
        ]
    )

    # ── Lifecycle Manager ─────────────────────────────────────────────────
    # Activates ALL Nav2 nodes in the correct order.
    # node_names order matters — bt_navigator should be last (depends on others).
    lifecycle_manager_nav2 = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    True,
            'node_names': [
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'waypoint_follower',
                'velocity_smoother',
                'bt_navigator',
            ],
            # Bond timeout: how long a managed node can go silent before
            # the lifecycle manager considers it dead and shuts down.
            'bond_timeout': 4.0,
        }]
    )

    # ── Our custom navigation nodes (NOT lifecycle-managed) ───────────────
    # These are regular nodes that start up immediately.

    goal_navigator_node = Node(
        package='autonomous_nav_bot',
        executable='goal_navigator.py',
        name='goal_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    waypoint_navigator_node = Node(
        package='autonomous_nav_bot',
        executable='waypoint_follower.py',
        name='waypoint_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── RViz2 ────────────────────────────────────────────────────────────
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
            '[navigation] Starting Nav2 stack.\n'
            '  Ensure localisation is running and the robot is localised.\n'
            '  Use "2D Nav Goal" in RViz2 once all nodes are active.'
        )
    )
    msg_ready = LogInfo(
        msg=(
            '[navigation] Nav2 READY\n'
            '  Send goal: click "2D Nav Goal" in RViz2\n'
            '  Monitor  : ros2 topic echo /navigation/status\n'
            '  Cancel   : ros2 topic pub /cancel_navigation '
            'std_msgs/msg/Bool "{data: true}" --once'
        )
    )

    # Delay RViz2 and custom nodes until Nav2 is fully activated (~5s)
    delayed_ui = TimerAction(
        period=5.0,
        actions=[goal_navigator_node, waypoint_navigator_node, rviz2, msg_ready]
    )

    return LaunchDescription([
        use_sim_time_arg,
        msg_start,

        # Nav2 server nodes
        bt_navigator,
        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        waypoint_follower_server,
        velocity_smoother,

        # Lifecycle manager activates them all
        lifecycle_manager_nav2,

        # Custom nodes + RViz2 after Nav2 is up
        delayed_ui,
    ])
