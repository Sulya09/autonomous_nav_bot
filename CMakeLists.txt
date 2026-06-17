cmake_minimum_required(VERSION 3.8)
project(autonomous_nav_bot)

# ══════════════════════════════════════════════════════════════════════════
# CMakeLists.txt — Build Configuration
# ══════════════════════════════════════════════════════════════════════════
# This file is read by colcon (the ROS 2 build tool) when you run:
#
#   colcon build --packages-select autonomous_nav_bot
#
# For this package the "build" step is mostly about INSTALLING files
# (URDF, configs, launch files) into the ROS 2 install/ directory so
# that ROS tools can find them via get_package_share_directory().
#
# WHEN YOU ADD C++ CODE: add ament_target_dependencies() + target calls.
# WHEN YOU ADD PYTHON NODES: they are installed via install(PROGRAMS ...).
# ══════════════════════════════════════════════════════════════════════════

# ── Compiler warnings (good practice for C++ code added in later steps) ──
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Required packages ─────────────────────────────────────────────────────
# ament_cmake is the base build system for ROS 2 CMake packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS 2 libraries (needed when C++ nodes are added in Steps 2–6)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Install Python packages ───────────────────────────────────────────────
# Makes Python modules in src/ importable as: from autonomous_nav_bot import ...
# (Uncomment when the first Python module is added)
# ament_python_install_package(${PROJECT_NAME})

# ── Install Python executable scripts ────────────────────────────────────
# Each PROGRAMS entry is installed to lib/<pkg>/ and becomes runnable via:
#   ros2 run autonomous_nav_bot lidar_processor.py
# It is also referenced by name in Node(executable=...) inside launch files.
install(
  PROGRAMS
    # Step 2: Sensor integration nodes
    src/sensor_integration/lidar_processor.py
    src/sensor_integration/imu_processor.py
    src/sensor_integration/camera_processor.py
    # Step 3: Mapping utility node
    src/mapping/map_saver.py
    # Step 4: Localisation diagnostic node
    src/localization/localization_monitor.py
    # Step 5: Navigation action client nodes
    src/navigation/goal_navigator.py
    src/navigation/waypoint_follower.py
    # Step 6: Motor control nodes
    src/control/emergency_stop.py
    src/control/velocity_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install resource directories ──────────────────────────────────────────
# Each install(DIRECTORY ...) copies a folder into the ROS 2 install space.
# After building, get_package_share_directory('autonomous_nav_bot') returns
# the path to share/autonomous_nav_bot/ where all these land.

# URDF / XACRO robot model files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Configuration files (nav2_params, slam_params, robot_description)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Launch files — all .launch.py files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Saved maps (.pgm + .yaml pairs)
install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# RViz2 configuration files
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Python source packages for sensor nodes, navigation behaviours, etc.
install(
  DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src
)

# ── Testing ───────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Runs standard ROS 2 linters: copyright check, flake8, pep257, etc.
  ament_lint_auto_find_test_dependencies()
endif()

# ── Finalise the package ──────────────────────────────────────────────────
# MUST be the last line — registers this package with the ament index
ament_package()
