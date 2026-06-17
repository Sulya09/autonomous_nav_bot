#!/usr/bin/env python3
"""
src/mapping/map_saver.py
════════════════════════
ROS 2 Node: map_saver

WHY THIS NODE EXISTS
────────────────────
When the robot finishes a mapping session, you need to capture the
occupancy grid and save it to two files:

    maps/my_map.pgm   — the greyscale image (black=wall, white=free, grey=unknown)
    maps/my_map.yaml  — metadata (resolution, origin, free/occupied thresholds)

Without saving, the map disappears when you stop slam_toolbox.

This node provides TWO ways to trigger a save:

  1. TOPIC trigger (easy):
       ros2 topic pub /save_map std_msgs/msg/String \
         "{data: 'office_map'}" --once

  2. SERVICE trigger (scriptable):
       ros2 service call /save_map_service std_srvs/srv/Trigger

Both methods call nav2_map_server's map_saver_cli under the hood:
    ros2 run nav2_map_server map_saver_cli -f <output_path>

This produces <output_path>.pgm and <output_path>.yaml.

OUTPUT LOCATION
───────────────
Maps are saved to the package's maps/ directory.
The filename is either:
  • The string you send on /save_map, OR
  • Auto-generated: map_YYYYMMDD_HHMMSS

WHAT THE OUTPUT FILES MEAN
───────────────────────────
  .pgm (Portable GreyMap image)
      • Pixel value 0   (black)  = occupied  (wall / obstacle)
      • Pixel value 205 (grey)   = unknown   (never scanned)
      • Pixel value 254 (white)  = free space

  .yaml (metadata)
      image:      relative path to the .pgm file
      resolution: metres per pixel (e.g. 0.05 = 5 cm/pixel)
      origin:     [x, y, yaw] of the bottom-left corner in world coords
      occupied_thresh:  pixels above this value are "occupied"
      free_thresh:      pixels below this value are "free"

ROS INTERFACES
──────────────
  Subscribes:   /save_map            → std_msgs/String   (filename)
  Service:      /save_map_service    → std_srvs/Trigger  (auto-name)
  Publishes:    /map_saver/status    → std_msgs/String   (result message)
"""

import os
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MapSaver(Node):
    """
    Saves the current SLAM occupancy grid to disk when triggered
    via topic or service call.
    """

    def __init__(self):
        super().__init__('map_saver')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('maps_directory', '')
        self.declare_parameter('default_map_name', '')
        self.declare_parameter('save_timeout_s', 10.0)

        maps_dir_param  = self.get_parameter('maps_directory').value
        self._timeout   = self.get_parameter('save_timeout_s').value

        # Resolve the maps/ directory.
        # Default: <package_share>/../../maps/  (workspace source maps/)
        if maps_dir_param:
            self._maps_dir = maps_dir_param
        else:
            pkg_share = get_package_share_directory('autonomous_nav_bot')
            # get_package_share_directory returns:
            #   install/autonomous_nav_bot/share/autonomous_nav_bot
            # maps/ lives 4 levels up and then into the src package.
            # For simplicity, save relative to the package share's maps dir.
            self._maps_dir = os.path.join(pkg_share, 'maps')

        os.makedirs(self._maps_dir, exist_ok=True)

        # ── Topic subscriber: /save_map ────────────────────────────────────
        # Send any string to trigger a save. The string becomes the filename.
        # Empty string → auto-generate a timestamped name.
        self._save_sub = self.create_subscription(
            String,
            '/save_map',
            self._topic_save_callback,
            10
        )

        # ── Service: /save_map_service ─────────────────────────────────────
        # Call with no arguments; auto-generates a timestamped filename.
        # Useful for scripted pipelines where the caller needs confirmation.
        self._save_srv = self.create_service(
            Trigger,
            '/save_map_service',
            self._service_save_callback
        )

        # ── Status publisher ───────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/map_saver/status', 10)

        self.get_logger().info(
            f'Map Saver ready\n'
            f'  Output directory : {self._maps_dir}\n'
            f'  Topic trigger    : ros2 topic pub /save_map std_msgs/msg/String '
            f'"{{data: \'my_map\'}}" --once\n'
            f'  Service trigger  : ros2 service call /save_map_service std_srvs/srv/Trigger'
        )

    # ──────────────────────────────────────────────────────────────────────
    # CALLBACKS
    # ──────────────────────────────────────────────────────────────────────

    def _topic_save_callback(self, msg: String) -> None:
        """Save the map with an optional caller-specified filename."""
        name = msg.data.strip() if msg.data.strip() else self._auto_name()
        success, message = self._save_map(name)
        self._publish_status(success, message)

    def _service_save_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        """Save the map and return success/failure to the service caller."""
        name = self._auto_name()
        success, message = self._save_map(name)
        response.success = success
        response.message = message
        self._publish_status(success, message)
        return response

    # ──────────────────────────────────────────────────────────────────────
    # CORE: MAP SAVING LOGIC
    # ──────────────────────────────────────────────────────────────────────

    def _save_map(self, filename: str) -> tuple[bool, str]:
        """
        Call nav2_map_server's map_saver_cli to write .pgm + .yaml to disk.

        Returns (success: bool, message: str).

        nav2_map_server's map_saver_cli command:
            ros2 run nav2_map_server map_saver_cli \
                -f <output_path> \
                --ros-args -p save_map_timeout:=5.0

        The -f flag sets the output filename (without extension).
        The tool subscribes to /map for one message, then writes the files.
        """
        # Sanitise filename — strip path separators to prevent directory traversal
        safe_name = os.path.basename(filename.replace('/', '_').replace('\\', '_'))
        output_path = os.path.join(self._maps_dir, safe_name)

        self.get_logger().info(f"Saving map to: {output_path}.pgm / .yaml ...")

        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', output_path,
            '--ros-args',
            '-p', f'save_map_timeout:={self._timeout}',
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self._timeout + 5.0  # extra margin over the tool's own timeout
            )

            if result.returncode == 0:
                pgm_path  = output_path + '.pgm'
                yaml_path = output_path + '.yaml'
                pgm_size  = os.path.getsize(pgm_path) if os.path.exists(pgm_path) else 0
                message   = (
                    f'Map saved successfully: {safe_name}\n'
                    f'  {pgm_path} ({pgm_size // 1024} KB)\n'
                    f'  {yaml_path}'
                )
                self.get_logger().info(message)
                return True, message

            else:
                message = (
                    f'Map save FAILED (exit code {result.returncode})\n'
                    f'  stderr: {result.stderr.strip()}'
                )
                self.get_logger().error(message)
                return False, message

        except subprocess.TimeoutExpired:
            message = (
                f'Map save TIMED OUT after {self._timeout + 5.0}s. '
                f'Is /map being published? Is slam_toolbox running?'
            )
            self.get_logger().error(message)
            return False, message

        except FileNotFoundError:
            message = (
                'map_saver_cli not found. '
                'Is nav2_map_server installed? '
                'Run: sudo apt install ros-humble-nav2-map-server'
            )
            self.get_logger().error(message)
            return False, message

    # ──────────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────────

    def _auto_name(self) -> str:
        """Generate a timestamped filename: map_YYYYMMDD_HHMMSS"""
        return 'map_' + datetime.now().strftime('%Y%m%d_%H%M%S')

    def _publish_status(self, success: bool, message: str) -> None:
        """Publish save result to /map_saver/status for monitoring tools."""
        status_msg      = String()
        prefix          = '✅ SUCCESS' if success else '❌ FAILED'
        status_msg.data = f'[map_saver] {prefix}: {message}'
        self._status_pub.publish(status_msg)


# ──────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
