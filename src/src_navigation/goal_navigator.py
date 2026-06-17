#!/usr/bin/env python3
"""
src/navigation/goal_navigator.py
═════════════════════════════════
ROS 2 Node: goal_navigator

WHY THIS NODE EXISTS
────────────────────
Nav2's action interface (NavigateToPose) is powerful but verbose to use
directly. This node wraps it with a simpler interface:

  1. RVIZ2 "2D Nav Goal" button → publishes PoseStamped on /goal_pose
  2. This node receives that PoseStamped
  3. Sends a NavigateToPose action to bt_navigator
  4. Streams feedback (distance remaining, recovery count)
  5. Reports success or failure with clear log messages

It also exposes a cancel interface so any topic can abort navigation.

HOW TO SEND A GOAL
──────────────────
  Option A — RViz2 (easiest):
    Click "2D Nav Goal" in the toolbar, then click + drag on the map.
    The arrow sets position + heading.

  Option B — Command line:
    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
      '{ header: {frame_id: "map"},
         pose: {
           position: {x: 2.0, y: 1.5, z: 0.0},
           orientation: {w: 1.0}
         }}' --once

  Option C — Python:
    from geometry_msgs.msg import PoseStamped
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = 2.0
    msg.pose.position.y = 1.5
    msg.pose.orientation.w = 1.0
    publisher.publish(msg)

HOW TO CANCEL
─────────────
    ros2 topic pub /cancel_navigation std_msgs/msg/Bool \
      '{data: true}' --once

NAVIGATION STATES (logged as the robot moves)
──────────────────────────────────────────────
  GOAL RECEIVED      → goal_pose arrived, sending to Nav2
  GOAL ACCEPTED      → bt_navigator accepted the action
  IN PROGRESS        → feedback: distance_remaining + recovery_count
  GOAL REACHED ✅    → robot arrived within xy_goal_tolerance
  GOAL FAILED ❌     → Nav2 could not complete navigation
  GOAL CANCELLED     → cancellation confirmed

ROS INTERFACES
──────────────
  Subscribes:  /goal_pose          → geometry_msgs/PoseStamped
               /cancel_navigation  → std_msgs/Bool
  Publishes:   /navigation/status  → std_msgs/String
               /navigation/active  → std_msgs/Bool (true if navigating)
  Action client: /navigate_to_pose → nav2_msgs/NavigateToPose
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String


class GoalNavigator(Node):
    """
    Bridges the simple /goal_pose topic to Nav2's NavigateToPose action,
    with feedback monitoring and cancellation support.
    """

    def __init__(self):
        super().__init__('goal_navigator')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        action_name = self.get_parameter('navigate_to_pose_action').value

        # ── Nav2 action client ─────────────────────────────────────────────
        # ActionClient wraps the full ROS 2 action protocol:
        #   send_goal → goal response → feedback stream → result
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            action_name
        )

        # ── State tracking ────────────────────────────────────────────────
        self._current_goal_handle: ClientGoalHandle | None = None
        self._is_navigating = False

        # ── Subscriptions ─────────────────────────────────────────────────
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_callback,
            10
        )
        self._cancel_sub = self.create_subscription(
            Bool,
            '/cancel_navigation',
            self._cancel_callback,
            10
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/navigation/status', 10)
        self._active_pub = self.create_publisher(Bool,   '/navigation/active', 10)

        self.get_logger().info(
            f'Goal Navigator ready\n'
            f'  Action server : /{action_name}\n'
            f'  Goal topic    : /goal_pose\n'
            f'  Cancel topic  : /cancel_navigation\n'
            f'  Status topic  : /navigation/status\n'
            f'\n'
            f'  Waiting for Nav2 action server...'
        )

        # Wait for Nav2 action server in a non-blocking way
        self._wait_timer = self.create_timer(2.0, self._check_server)

    # ──────────────────────────────────────────────────────────────────────
    # SERVER READINESS CHECK
    # ──────────────────────────────────────────────────────────────────────

    def _check_server(self) -> None:
        """Poll until the Nav2 action server is available."""
        if self._action_client.server_is_ready():
            self.get_logger().info('✅ Nav2 action server ready — goals can now be sent')
            self._wait_timer.cancel()
        else:
            self.get_logger().info('⏳ Waiting for Nav2 navigate_to_pose server...')

    # ──────────────────────────────────────────────────────────────────────
    # GOAL CALLBACK
    # ──────────────────────────────────────────────────────────────────────

    def _goal_callback(self, pose: PoseStamped) -> None:
        """
        Called when a new goal arrives (from RViz2 "2D Nav Goal" or topic).
        Sends a NavigateToPose action to bt_navigator.
        """
        if not self._action_client.server_is_ready():
            self._publish_status('GOAL REJECTED: Nav2 server not ready yet')
            self.get_logger().warn('Nav2 not ready — goal ignored')
            return

        # Cancel any ongoing navigation before accepting a new goal
        if self._is_navigating and self._current_goal_handle is not None:
            self.get_logger().info('New goal received — cancelling previous navigation')
            self._current_goal_handle.cancel_goal_async()

        # Build the action goal
        goal_msg            = NavigateToPose.Goal()
        goal_msg.pose       = pose
        goal_msg.behavior_tree = ''   # empty = use Nav2's default BT

        x = pose.pose.position.x
        y = pose.pose.position.y
        self.get_logger().info(
            f'GOAL RECEIVED → ({x:.2f}, {y:.2f}) in frame "{pose.header.frame_id}"'
        )
        self._publish_status(f'GOAL RECEIVED: navigating to ({x:.2f}, {y:.2f})')

        # Send the action asynchronously
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    # ──────────────────────────────────────────────────────────────────────
    # ACTION CALLBACKS
    # ──────────────────────────────────────────────────────────────────────

    def _goal_response_callback(self, future) -> None:
        """Called when bt_navigator accepts or rejects the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._is_navigating = False
            self._publish_active(False)
            self._publish_status('GOAL REJECTED by Nav2')
            self.get_logger().error('❌ Goal rejected by bt_navigator')
            return

        self._current_goal_handle = goal_handle
        self._is_navigating = True
        self._publish_active(True)
        self._publish_status('GOAL ACCEPTED — navigation in progress')
        self.get_logger().info('✅ Goal accepted — navigating...')

        # Register callback for when navigation finishes
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        """
        Called repeatedly while the robot is navigating.
        feedback.feedback contains:
          distance_remaining (float)  — metres left to goal
          number_of_recoveries (int)  — how many times recovery was triggered
          navigation_time (Duration)  — elapsed navigation time
          estimated_time_remaining (Duration)
        """
        fb = feedback_msg.feedback
        dist       = fb.distance_remaining
        recoveries = fb.number_of_recoveries

        status = f'IN PROGRESS: {dist:.2f}m remaining'
        if recoveries > 0:
            status += f' (recoveries: {recoveries})'

        self._publish_status(status)

        # Only log at meaningful distance thresholds to avoid spam
        if dist < 0.5:
            self.get_logger().info(f'Approaching goal: {dist:.2f}m remaining')

    def _result_callback(self, future) -> None:
        """Called when navigation is complete (success, failure, or cancel)."""
        result     = future.result()
        status     = result.status

        self._is_navigating = False
        self._current_goal_handle = None
        self._publish_active(False)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ GOAL REACHED — navigation successful')
            self._publish_status('GOAL REACHED ✅')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation CANCELLED by request')
            self._publish_status('GOAL CANCELLED')

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                '❌ GOAL FAILED — Nav2 could not reach the goal.\n'
                '  Possible causes:\n'
                '    • Goal is inside an obstacle\n'
                '    • Path is blocked and all recoveries failed\n'
                '    • Localisation is poor (try "2D Pose Estimate" in RViz2)\n'
                '    • Costmap inflation blocked all paths'
            )
            self._publish_status('GOAL FAILED ❌ — see terminal for details')

        else:
            self.get_logger().warn(f'Navigation ended with unknown status: {status}')
            self._publish_status(f'GOAL ENDED: status={status}')

    # ──────────────────────────────────────────────────────────────────────
    # CANCEL CALLBACK
    # ──────────────────────────────────────────────────────────────────────

    def _cancel_callback(self, msg: Bool) -> None:
        """Cancel ongoing navigation if msg.data is True."""
        if not msg.data:
            return
        if self._current_goal_handle is None or not self._is_navigating:
            self.get_logger().info('Cancel requested but no active navigation')
            return

        self.get_logger().info('Cancelling navigation by request...')
        cancel_future = self._current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda f: self.get_logger().info('Cancel request sent to Nav2')
        )

    # ──────────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────────

    def _publish_status(self, text: str) -> None:
        msg      = String()
        msg.data = f'[goal_navigator] {text}'
        self._status_pub.publish(msg)

    def _publish_active(self, active: bool) -> None:
        msg      = Bool()
        msg.data = active
        self._active_pub.publish(msg)


# ──────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
