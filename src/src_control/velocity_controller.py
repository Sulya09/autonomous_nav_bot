#!/usr/bin/env python3
"""
src/control/velocity_controller.py
════════════════════════════════════
ROS 2 Node: velocity_controller

WHY THIS NODE EXISTS
────────────────────
This is the LAST software node before velocity commands reach the robot's
wheels. It acts as a safety gate with three independent mechanisms:

  1. HARD SPEED CLAMPING
     Guarantees the robot can never exceed its physical velocity limits,
     regardless of what Nav2, teleop, or any other node commands.
     Values are read from control_params.yaml, not from Nav2's config.

  2. EMERGENCY STOP INTEGRATION
     Reads the /emergency_stop Bool from the EmergencyStop node.
     When True, overrides ALL velocity commands with zero (full stop).
     When False, resumes normal pass-through.
     The robot cannot move while the e-stop is active.

  3. DEAD-MAN SWITCH
     If no /cmd_vel_raw message arrives within cmd_timeout_s seconds,
     automatically publishes zero velocity. This stops the robot if:
       • The Nav2 process crashes
       • The network connection drops
       • The planner hangs
     Normal operation resumes as soon as commands start arriving again.

WHERE THIS FITS IN THE PIPELINE
────────────────────────────────
  teleop keyboard
        │
        ├──► /cmd_vel_raw ──┐
        │                   │
  Nav2 velocity smoother    │
        │                   │
        └──► /cmd_vel_raw ──┤   ← both teleop and Nav2 share this topic
                             │      via remapping in their launch configurations
                             ▼
                   [velocity_controller]
                     ├─ dead-man switch check
                     ├─ emergency stop check
                     └─ hard speed clamp
                             │
                             ▼
                         /cmd_vel ──► Gazebo diff_drive / real motors

CAUTION ZONE SPEED REDUCTION
─────────────────────────────
When the EmergencyStop node reports ZONE_CAUTION (via /control/zone),
this controller applies slow_speed_factor to both linear and angular
velocity. This creates a graduated response:

  CLEAR   → 100% commanded speed
  CAUTION → 40% commanded speed (slowing near obstacles)
  DANGER  → 0%  (full stop via e-stop)

ROS INTERFACES
──────────────
  Subscribes:
    /cmd_vel_raw      → geometry_msgs/Twist (from Nav2 or teleop)
    /emergency_stop   → std_msgs/Bool      (from EmergencyStop node)
    /control/zone     → std_msgs/String    ('CLEAR'/'CAUTION'/'DANGER')
  Publishes:
    /cmd_vel          → geometry_msgs/Twist (to robot)
    /control/status   → std_msgs/String    (diagnostic summary)
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

# Zone constants matching emergency_stop.py
ZONE_CLEAR   = 'CLEAR'
ZONE_CAUTION = 'CAUTION'
ZONE_DANGER  = 'DANGER'


class VelocityController(Node):
    """
    Final safety gate applying speed limits, e-stop, dead-man switch,
    and zone-based speed reduction before publishing /cmd_vel.
    """

    def __init__(self):
        super().__init__('velocity_controller')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('input_topic',     '/cmd_vel_raw')
        self.declare_parameter('output_topic',    '/cmd_vel')
        self.declare_parameter('e_stop_topic',    '/emergency_stop')
        self.declare_parameter('cmd_timeout_s',    0.5)
        self.declare_parameter('max_linear_vel',   0.26)
        self.declare_parameter('max_angular_vel',  1.82)
        self.declare_parameter('slow_speed_factor', 0.4)
        self.declare_parameter('stats_interval_s', 5.0)

        input_topic        = self.get_parameter('input_topic').value
        output_topic       = self.get_parameter('output_topic').value
        e_stop_topic       = self.get_parameter('e_stop_topic').value
        self._timeout      = self.get_parameter('cmd_timeout_s').value
        self._max_linear   = self.get_parameter('max_linear_vel').value
        self._max_angular  = self.get_parameter('max_angular_vel').value
        self._slow_factor  = self.get_parameter('slow_speed_factor').value
        stats_interval     = self.get_parameter('stats_interval_s').value

        # ── State ─────────────────────────────────────────────────────────
        self._e_stop_active  = False
        self._current_zone   = ZONE_CLEAR
        self._last_cmd_time  = None     # wall-clock time of last /cmd_vel_raw
        self._dead_man_fired = False    # True when timeout has triggered

        # Diagnostics counters
        self._cmds_received    = 0
        self._cmds_published   = 0
        self._e_stop_blocks    = 0
        self._dead_man_blocks  = 0
        self._clamp_events     = 0

        # ── Subscriptions ─────────────────────────────────────────────────
        self._cmd_sub = self.create_subscription(
            Twist, input_topic, self._cmd_callback, 10
        )
        self._estop_sub = self.create_subscription(
            Bool, e_stop_topic, self._e_stop_callback, 10
        )
        self._zone_sub = self.create_subscription(
            String, '/control/zone', self._zone_callback, 10
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist,  output_topic,     10)
        self._status_pub = self.create_publisher(String, '/control/status', 10)

        # ── Dead-man switch timer ──────────────────────────────────────────
        # Runs at 2× the timeout rate so it reacts within ½ a timeout period.
        self._watchdog = self.create_timer(
            self._timeout / 2.0, self._watchdog_callback
        )

        # ── Periodic diagnostics ───────────────────────────────────────────
        self.create_timer(stats_interval, self._publish_stats)

        # Send an immediate zero velocity so the robot is stationary at boot.
        self._publish_zero()

        self.get_logger().info(
            f'Velocity Controller ready\n'
            f'  Input  : {input_topic}\n'
            f'  Output : {output_topic}\n'
            f'  Limits : linear ≤ ±{self._max_linear}m/s, '
            f'angular ≤ ±{self._max_angular}rad/s\n'
            f'  Timeout: {self._timeout}s dead-man switch\n'
            f'  CAUTION speed factor: {self._slow_factor}'
        )

    # ──────────────────────────────────────────────────────────────────────
    # SUBSCRIPTIONS
    # ──────────────────────────────────────────────────────────────────────

    def _e_stop_callback(self, msg: Bool) -> None:
        """Track emergency stop state. Log on change."""
        was_active = self._e_stop_active
        self._e_stop_active = msg.data
        if self._e_stop_active and not was_active:
            self.get_logger().error('🛑 E-STOP ACTIVATED — zero velocity commanded')
            self._publish_zero()
        elif not self._e_stop_active and was_active:
            self.get_logger().info('✅ E-stop cleared — resuming normal operation')

    def _zone_callback(self, msg: String) -> None:
        """Track the current safety zone from EmergencyStop node."""
        self._current_zone = msg.data

    def _cmd_callback(self, msg: Twist) -> None:
        """
        Process an incoming velocity command through all safety filters.

        Steps:
          1. Record arrival time (resets dead-man timer)
          2. If e-stop active → publish zero, count block, return
          3. Apply zone-based speed reduction (CAUTION zone)
          4. Clamp to hard velocity limits
          5. Publish the safe command
        """
        self._cmds_received += 1
        self._last_cmd_time = time.monotonic()
        self._dead_man_fired = False

        # ── Gate 1: Emergency stop ─────────────────────────────────────────
        if self._e_stop_active:
            self._e_stop_blocks += 1
            self._publish_zero()
            return

        # ── Gate 2: CAUTION zone speed reduction ───────────────────────────
        # Don't clamp to zero (that's DANGER's job), just slow down.
        factor = self._slow_factor if self._current_zone == ZONE_CAUTION else 1.0
        linear  = msg.linear.x  * factor
        angular = msg.angular.z * factor

        # ── Gate 3: Hard velocity clamping ─────────────────────────────────
        clamped_linear  = max(-self._max_linear,  min(self._max_linear,  linear))
        clamped_angular = max(-self._max_angular, min(self._max_angular, angular))

        if clamped_linear != linear or clamped_angular != angular:
            self._clamp_events += 1

        # ── Publish the safe command ───────────────────────────────────────
        safe_cmd             = Twist()
        safe_cmd.linear.x    = clamped_linear
        safe_cmd.angular.z   = clamped_angular
        self._cmd_pub.publish(safe_cmd)
        self._cmds_published += 1

    # ──────────────────────────────────────────────────────────────────────
    # DEAD-MAN SWITCH
    # ──────────────────────────────────────────────────────────────────────

    def _watchdog_callback(self) -> None:
        """
        Called at 2× the timeout rate. If no command has arrived within
        the timeout window, publish zero velocity and log a warning.

        The dead-man switch fires once (not repeatedly) to avoid log spam.
        Normal operation resumes automatically when commands restart.
        """
        if self._last_cmd_time is None:
            return   # never received a command yet — don't trigger

        elapsed = time.monotonic() - self._last_cmd_time
        if elapsed > self._timeout and not self._dead_man_fired:
            self._dead_man_fired = True
            self._dead_man_blocks += 1
            self._publish_zero()
            self.get_logger().warn(
                f'⏱  Dead-man switch fired — no cmd_vel for {elapsed:.2f}s '
                f'(timeout={self._timeout}s). Publishing zero velocity.'
            )

    # ──────────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────────

    def _publish_zero(self) -> None:
        """Publish a full-stop Twist message."""
        self._cmd_pub.publish(Twist())   # all fields default to 0.0

    def _publish_stats(self) -> None:
        state = 'E-STOP' if self._e_stop_active \
            else ('DEAD-MAN' if self._dead_man_fired else self._current_zone)

        text = (
            f'[velocity_controller] '
            f'state={state} | '
            f'rx={self._cmds_received} tx={self._cmds_published} | '
            f'e_stop_blocks={self._e_stop_blocks} | '
            f'dead_man_blocks={self._dead_man_blocks} | '
            f'clamp_events={self._clamp_events}'
        )
        msg      = String()
        msg.data = text
        self._status_pub.publish(msg)

        if self._e_stop_active or self._dead_man_fired:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)


# ──────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
