#!/usr/bin/env python3
"""
launch/display.launch.py
════════════════════════
PURPOSE
  A development-only launch file for Step 1 verification.
  Starts the minimum set of nodes needed to see the robot in RViz2
  and manually move its joints — without needing Gazebo or sensors.

WHAT IT STARTS
  1. robot_state_publisher  — reads the URDF, broadcasts all TF frames
  2. joint_state_publisher_gui — slider window to spin wheels manually
  3. rviz2                  — 3D visualization window (pre-configured)

HOW TO RUN
  # After building and sourcing the workspace:
  ros2 launch autonomous_nav_bot display.launch.py

  # With simulation time (when Gazebo is also running):
  ros2 launch autonomous_nav_bot display.launch.py use_sim_time:=true

EXPECTED RESULT
  • An RViz2 window opens showing the full 3D robot model
  • A small "Joint State Publisher" slider window appears
  • Moving the sliders visibly rotates the wheels in RViz2
  • The TF display shows all coordinate frames on the robot
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # ── Resolve paths ────────────────────────────────────────────────────
    pkg_dir = get_package_share_directory('autonomous_nav_bot')

    urdf_file   = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'robot_display.rviz')

    # ── Declare launch arguments ─────────────────────────────────────────
    # These can be overridden on the command line: key:=value
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description=(
            'Set to true when Gazebo is running so nodes use the '
            'simulated clock instead of the wall clock.'
        )
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Process XACRO → URDF string ──────────────────────────────────────
    # Command(['xacro ', urdf_file]) runs the xacro tool at launch time and
    # feeds the resulting URDF XML string directly to robot_state_publisher.
    # This avoids needing a separate pre-build step.
    robot_description = Command(['xacro ', urdf_file])

    # ── Node 1: Robot State Publisher ────────────────────────────────────
    # Reads the URDF and, together with incoming joint_states messages,
    # computes and broadcasts TF transforms for every link in the tree.
    # Without this, RViz2 and Nav2 have no idea where each part of the
    # robot is in space.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )

    # ── Node 2: Joint State Publisher GUI ────────────────────────────────
    # Publishes sensor_msgs/JointState messages for all movable joints
    # (the two drive wheels). The GUI version adds a slider window so
    # you can manually rotate the wheels to test the TF tree visually.
    #
    # NOTE: This node is replaced in later steps:
    #   • Gazebo's diff_drive plugin publishes real joint states
    #   • Real hardware publishes from motor encoder feedback
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── Node 3: RViz2 ────────────────────────────────────────────────────
    # The 3D visualisation window. Opens with a pre-configured layout
    # (rviz/robot_display.rviz) that shows the robot model, TF frames,
    # and the laser scan layer (empty until Gazebo is running).
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── Assemble and return ──────────────────────────────────────────────
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
    ])
