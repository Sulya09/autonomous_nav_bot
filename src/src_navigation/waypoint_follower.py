#!/usr/bin/env python3
"""
src/navigation/waypoint_follower.py
════════════════════════════════════
ROS 2 Node: waypoint_navigator

WHY THIS NODE EXISTS
────────────────────
For tasks like patrolling a building, inspecting rooms in order, or
running a delivery route, you need the robot to visit multiple locations
in sequence. Nav2's FollowWaypoints action handles this natively —
this node provides a clean interface to drive it.

TWO WAYS TO SEND WAYPOINTS
────────────────────────────
  1. TOPIC — send a geometry_msgs/PoseArray to /waypoints:

       ros2 topic pub /waypoints geometry_msgs/msg/PoseArray \
         '{ header: {frame_id: "map"},
            poses: [
              {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}},
              {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}},
              {position: {x: 0.5, y: 3.5, z: 0.0}, orientation: {w: 0.707, z: 0.707}}
            ]}' --once

  2. FILE — pass a YAML file path as a parameter:

       ros2 run autonomous_nav_bot waypoint_follower.py \
         --ros-args -p waypoints_file:=/path/to/waypoints.yaml

     YAML format:
       frame_id: map
       waypoints:
         - {x: 1.0, y: 0.5, yaw: 0.0}
         - {x: 3.0, y: 2.0, yaw: 1.57}
         - {x: 0.5, y: 3.5, yaw: 3.14}

WHAT HAPPENS AT EACH WAYPOINT
───────────────────────────────
  Nav2's WaypointFollower server navigates to each point in order.
  At each waypoint it pauses for waypoint_pause_duration ms
  (configured in nav2_params.yaml → waypoint_follower section).

CANCELLATION
─────────────
    ros2 topic pub /cancel_waypoints std_msgs/msg/Bool '{data: true}' --once

ROS INTERFACES
──────────────
  Subscribes:  /waypoints          → geometry_msgs/PoseArray
               /cancel_waypoints   → std_msgs/Bool
  Publishes:   /waypoint/status    → std_msgs/String
               /waypoint/progress  → std_msgs/String  (current / total)
  Action:      /follow_waypoints   → nav2_msgs/FollowWaypoints
  Parameter:   waypoints_file (str) — YAML file to load on startup
"""

import os
from typing import Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import Bool, String

import math


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Convert a yaw angle (radians) to a geometry_msgs/Quaternion.
    For a 2D ground robot, only the z and w components are non-zero.
    """
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WaypointNavigator(Node):
    """
    Loads waypoints from a topic or YAML file and executes them
    sequentially via Nav2's FollowWaypoints action server.
    """

    def __init__(self):
        super().__init__('waypoint_navigator')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')

        waypoints_file = self.get_parameter('waypoints_file').value
        self._frame_id = self.get_parameter('frame_id').value

        # ── Nav2 action client ─────────────────────────────────────────────
        self._action_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )

        # ── State ─────────────────────────────────────────────────────────
        self._goal_handle   = None
        self._is_running    = False
        self._total_wps     = 0
        self._current_wp    = 0

        # ── Subscriptions ─────────────────────────────────────────────────
        self._waypoints_sub = self.create_subscription(
            PoseArray,
            '/waypoints',
            self._waypoints_callback,
            10
        )
        self._cancel_sub = self.create_subscription(
            Bool,
            '/cancel_waypoints',
            self._cancel_callback,
            10
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._status_pub   = self.create_publisher(String, '/waypoint/status',   10)
        self._progress_pub = self.create_publisher(String, '/waypoint/progress', 10)

        # ── Auto-load waypoints from file if parameter is set ──────────────
        if waypoints_file and os.path.isfile(waypoints_file):
            self.get_logger().info(f'Loading waypoints from: {waypoints_file}')
            self.create_timer(
                3.0,    # delay to let Nav2 server start first
                lambda: self._load_and_send_file(waypoints_file)
            )
        elif waypoints_file:
            self.get_logger().warn(
                f'waypoints_file parameter set but file not found: {waypoints_file}'
            )

        self.get_logger().info(
            f'Waypoint Navigator ready\n'
            f'  Topic    : /waypoints  (geometry_msgs/PoseArray)\n'
            f'  File     : {waypoints_file or "(not set)"}\n'
            f'  Cancel   : /cancel_waypoints\n'
            f'  Status   : /waypoint/status'
        )

        self._wait_timer = self.create_timer(2.0, self._check_server)

    # ──────────────────────────────────────────────────────────────────────
    # SERVER READINESS
    # ──────────────────────────────────────────────────────────────────────

    def _check_server(self) -> None:
        if self._action_client.server_is_ready():
            self.get_logger().info('✅ Nav2 FollowWaypoints server ready')
            self._wait_timer.cancel()

    # ──────────────────────────────────────────────────────────────────────
    # WAYPOINTS FROM TOPIC
    # ──────────────────────────────────────────────────────────────────────

    def _waypoints_callback(self, msg: PoseArray) -> None:
        """Receive waypoints as a PoseArray and start navigation."""
        if self._is_running:
            self.get_logger().warn(
                'Waypoint navigation already in progress — '
                'cancel first with: '
                'ros2 topic pub /cancel_waypoints std_msgs/msg/Bool '
                '"{data: true}" --once'
            )
            return

        if not msg.poses:
            self.get_logger().warn('Received empty PoseArray — nothing to do')
            return

        # Convert PoseArray → list of PoseStamped
        waypoints = []
        for pose in msg.poses:
            ps             = PoseStamped()
            ps.header.frame_id = msg.header.frame_id or self._frame_id
            ps.header.stamp    = self.get_clock().now().to_msg()
            ps.pose            = pose
            waypoints.append(ps)

        self.get_logger().info(
            f'Received {len(waypoints)} waypoints via /waypoints topic'
        )
        self._execute(waypoints)

    # ──────────────────────────────────────────────────────────────────────
    # WAYPOINTS FROM FILE
    # ──────────────────────────────────────────────────────────────────────

    def _load_and_send_file(self, path: str) -> None:
        """
        Load waypoints from a YAML file and start navigation.

        Expected YAML format:
            frame_id: map
            waypoints:
              - {x: 1.0, y: 0.5, yaw: 0.0}
              - {x: 3.0, y: 2.0, yaw: 1.57}
        """
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints file: {e}')
            return

        frame = data.get('frame_id', self._frame_id)
        raw   = data.get('waypoints', [])

        if not raw:
            self.get_logger().warn(f'No waypoints found in {path}')
            return

        waypoints = []
        for wp in raw:
            ps             = PoseStamped()
            ps.header.frame_id = frame
            ps.header.stamp    = self.get_clock().now().to_msg()
            ps.pose.position.x = float(wp.get('x', 0.0))
            ps.pose.position.y = float(wp.get('y', 0.0))
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quaternion(float(wp.get('yaw', 0.0)))
            waypoints.append(ps)

        self.get_logger().info(
            f'Loaded {len(waypoints)} waypoints from {path}'
        )
        self._execute(waypoints)

    # ──────────────────────────────────────────────────────────────────────
    # EXECUTE WAYPOINT SEQUENCE
    # ──────────────────────────────────────────────────────────────────────

    def _execute(self, waypoints: list[PoseStamped]) -> None:
        """Send the waypoint list to Nav2's FollowWaypoints action."""
        if not self._action_client.server_is_ready():
            self.get_logger().error('Nav2 FollowWaypoints server not ready')
            return

        self._total_wps  = len(waypoints)
        self._current_wp = 0
        self._is_running = True

        goal             = FollowWaypoints.Goal()
        goal.poses       = waypoints

        self.get_logger().info(
            f'Starting waypoint route: {self._total_wps} stops'
        )
        self._publish_status(f'STARTED: {self._total_wps} waypoints queued')

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    # ──────────────────────────────────────────────────────────────────────
    # ACTION CALLBACKS
    # ──────────────────────────────────────────────────────────────────────

    def _goal_response_callback(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._is_running = False
            self._publish_status('REJECTED by Nav2')
            self.get_logger().error('Waypoint goal rejected')
            return

        self._goal_handle = handle
        self.get_logger().info('Waypoint route accepted — starting execution')
        handle.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        """
        Called after each waypoint is reached.
        feedback.feedback.current_waypoint = index of the completed waypoint.
        """
        completed = feedback_msg.feedback.current_waypoint
        remaining = self._total_wps - completed

        progress_text = f'{completed}/{self._total_wps} waypoints completed'
        self.get_logger().info(f'Waypoint {completed} reached — {remaining} remaining')
        self._publish_status(f'IN PROGRESS: {progress_text}')
        self._publish_progress(progress_text)

    def _result_callback(self, future) -> None:
        self._is_running  = False
        self._goal_handle = None
        result = future.result()
        status = result.status
        missed = list(result.result.missed_waypoints)

        if status == GoalStatus.STATUS_SUCCEEDED:
            if missed:
                self.get_logger().warn(
                    f'Route complete with {len(missed)} missed waypoints: {missed}'
                )
                self._publish_status(
                    f'COMPLETED WITH MISSES: {len(missed)} waypoints skipped'
                )
            else:
                self.get_logger().info(
                    f'✅ All {self._total_wps} waypoints reached successfully'
                )
                self._publish_status(f'ALL DONE ✅: {self._total_wps} waypoints completed')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Waypoint route cancelled')
            self._publish_status('CANCELLED')

        else:
            self.get_logger().error(
                f'Waypoint route failed (status={status}, missed={missed})'
            )
            self._publish_status(f'FAILED ❌: status={status}')

    # ──────────────────────────────────────────────────────────────────────
    # CANCEL
    # ──────────────────────────────────────────────────────────────────────

    def _cancel_callback(self, msg: Bool) -> None:
        if not msg.data or not self._is_running or self._goal_handle is None:
            return
        self.get_logger().info('Cancelling waypoint route...')
        self._goal_handle.cancel_goal_async()

    # ──────────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────────

    def _publish_status(self, text: str) -> None:
        msg      = String()
        msg.data = f'[waypoint_navigator] {text}'
        self._status_pub.publish(msg)

    def _publish_progress(self, text: str) -> None:
        msg      = String()
        msg.data = text
        self._progress_pub.publish(msg)


# ──────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
