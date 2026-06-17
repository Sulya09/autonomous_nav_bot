#!/usr/bin/env python3
"""
src/control/emergency_stop.py
══════════════════════════════
ROS 2 Node: emergency_stop

WHY THIS NODE EXISTS
────────────────────
Nav2's costmaps prevent the robot from *planning* a path through obstacles.
But costmaps have latency — they update at 5–10 Hz. A person walking at
1 m/s crosses our robot's stopping distance in 200 ms, faster than the
local costmap can react.

This node monitors the LiDAR scan at 10 Hz — the same rate as the sensor
itself — and provides a hard real-time safety floor below Nav2.

THREE-ZONE SAFETY MODEL
────────────────────────

  ┌─────────────────────────────────────┐
  │  Robot                              │
  │           DANGER  < 0.20m  ←→ STOP │ full e-stop, zero velocity
  │          CAUTION  < 0.40m  ←→ SLOW │ reduce speed to 40%
  │           CLEAR   > 0.40m  ←→  OK  │ normal operation
  └─────────────────────────────────────┘

Only LiDAR rays in the forward hemisphere (±90° of heading) trigger
DANGER/CAUTION. Rays behind the robot are ignored when moving forward,
preventing unnecessary braking near walls the robot just passed.

OUTPUTS
────────
  /emergency_stop     → std_msgs/Bool   — True=STOP, False=OK
  /control/zone       → std_msgs/String — 'CLEAR'/'CAUTION'/'DANGER'
  /control/e_stop_status → std_msgs/String — full diagnostic text

ROS INTERFACES
──────────────
  Subscribes:  /scan/filtered     → sensor_msgs/LaserScan
  Publishes:   /emergency_stop    → std_msgs/Bool
               /control/zone      → std_msgs/String
               /control/e_stop_status → std_msgs/String
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


# Zone identifiers
ZONE_CLEAR   = 'CLEAR'
ZONE_CAUTION = 'CAUTION'
ZONE_DANGER  = 'DANGER'


class EmergencyStop(Node):
    """
    Real-time LiDAR proximity monitor. Publishes e-stop signals
    when the robot approaches obstacles faster than Nav2 can react.
    """

    def __init__(self):
        super().__init__('emergency_stop')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('scan_topic',       '/scan/filtered')
        self.declare_parameter('e_stop_topic',     '/emergency_stop')
        self.declare_parameter('stop_distance',     0.20)
        self.declare_parameter('slow_distance',     0.40)
        self.declare_parameter('slow_speed_factor', 0.4)
        self.declare_parameter('min_danger_rays',   3)
        self.declare_parameter('front_angle_deg',   180.0)
        self.declare_parameter('stats_interval_s',  2.0)

        scan_topic       = self.get_parameter('scan_topic').value
        e_stop_topic     = self.get_parameter('e_stop_topic').value
        self._stop_dist  = self.get_parameter('stop_distance').value
        self._slow_dist  = self.get_parameter('slow_distance').value
        self._slow_factor = self.get_parameter('slow_speed_factor').value
        self._min_danger  = self.get_parameter('min_danger_rays').value
        front_deg         = self.get_parameter('front_angle_deg').value
        stats_interval    = self.get_parameter('stats_interval_s').value

        # Convert front angle to radians for scan masking
        self._front_rad = math.radians(front_deg / 2.0)

        # ── State ─────────────────────────────────────────────────────────
        self._current_zone   = ZONE_CLEAR
        self._e_stop_active  = False
        self._min_distance   = float('inf')
        self._danger_count   = 0   # rays currently in DANGER zone
        self._caution_count  = 0   # rays currently in CAUTION zone

        # Counters for diagnostics
        self._scan_count          = 0
        self._e_stop_activations  = 0

        # ── QoS ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # ── Subscriptions ─────────────────────────────────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, sensor_qos
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._e_stop_pub  = self.create_publisher(Bool,   e_stop_topic,           10)
        self._zone_pub    = self.create_publisher(String, '/control/zone',         10)
        self._status_pub  = self.create_publisher(String, '/control/e_stop_status', 10)

        # Publish e-stop = False immediately at startup so the controller
        # doesn't wait for the first scan before allowing motion.
        self._publish_e_stop(False)

        self.create_timer(stats_interval, self._publish_stats)

        self.get_logger().info(
            f'Emergency Stop ready\n'
            f'  Scan topic     : {scan_topic}\n'
            f'  DANGER  zone   : < {self._stop_dist}m  → FULL STOP\n'
            f'  CAUTION zone   : < {self._slow_dist}m  → {int(self._slow_factor*100)}% speed\n'
            f'  Min danger rays: {self._min_danger}\n'
            f'  Front angle    : ±{front_deg/2:.0f}°'
        )

    # ──────────────────────────────────────────────────────────────────────
    # MAIN CALLBACK
    # ──────────────────────────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan) -> None:
        """
        Evaluate every incoming scan for proximity violations.
        Called at the LiDAR's own rate (10 Hz in our config).
        """
        self._scan_count += 1
        ranges = np.array(msg.ranges, dtype=np.float32)

        # ── Build angular mask for the forward hemisphere ──────────────────
        # Only count rays within ±front_rad of the robot's forward direction
        # (angle = 0 in the laser frame). This prevents braking when a wall
        # appears behind the robot as it drives away from it.
        n    = len(ranges)
        angles = np.linspace(
            msg.angle_min, msg.angle_max,
            n, endpoint=False, dtype=np.float64
        )
        front_mask = np.abs(angles) <= self._front_rad

        # Apply mask, then filter out NaN / Inf (already cleaned by
        # lidar_processor, but defend against direct topic use)
        front_ranges = np.where(
            front_mask & np.isfinite(ranges),
            ranges,
            np.inf   # treat ignored/invalid rays as "no obstacle"
        )

        # ── Zone classification ────────────────────────────────────────────
        # Count how many valid front rays fall inside each zone
        danger_mask  = front_ranges < self._stop_dist
        caution_mask = (front_ranges >= self._stop_dist) & \
                       (front_ranges < self._slow_dist)

        self._danger_count  = int(np.sum(danger_mask))
        self._caution_count = int(np.sum(caution_mask))

        # Use np.nanmin so we don't crash on empty array
        finite = front_ranges[np.isfinite(front_ranges)]
        self._min_distance = float(np.min(finite)) if finite.size > 0 else float('inf')

        # ── Determine zone ─────────────────────────────────────────────────
        new_zone = self._classify_zone()

        # ── Publish on zone change (and periodically in stats) ─────────────
        if new_zone != self._current_zone:
            self._on_zone_change(self._current_zone, new_zone)
            self._current_zone = new_zone

        e_stop = (new_zone == ZONE_DANGER)
        if e_stop != self._e_stop_active:
            if e_stop:
                self._e_stop_activations += 1
            self._e_stop_active = e_stop

        # Always publish e-stop and zone at scan rate so velocity_controller
        # has a fresh signal every cycle (not just on changes).
        self._publish_e_stop(e_stop)
        self._publish_zone(new_zone)

    # ──────────────────────────────────────────────────────────────────────
    # ZONE CLASSIFICATION
    # ──────────────────────────────────────────────────────────────────────

    def _classify_zone(self) -> str:
        """
        Map danger/caution ray counts to a safety zone string.

        DANGER  : At least min_danger_rays below stop_distance
        CAUTION : Not in DANGER, but some rays below slow_distance
        CLEAR   : No rays in either threshold
        """
        if self._danger_count >= self._min_danger:
            return ZONE_DANGER
        if self._caution_count > 0:
            return ZONE_CAUTION
        return ZONE_CLEAR

    def _on_zone_change(self, old: str, new: str) -> None:
        """Log transitions between safety zones."""
        icons = {ZONE_CLEAR: '✅', ZONE_CAUTION: '⚠️ ', ZONE_DANGER: '🛑'}
        msg = (
            f'Zone: {old} → {new} {icons.get(new, "")} | '
            f'min_dist={self._min_distance:.3f}m | '
            f'danger_rays={self._danger_count} caution_rays={self._caution_count}'
        )
        if new == ZONE_DANGER:
            self.get_logger().error(f'E-STOP ACTIVE — {msg}')
        elif new == ZONE_CAUTION:
            self.get_logger().warn(f'CAUTION — {msg}')
        else:
            self.get_logger().info(f'All clear — {msg}')

    # ──────────────────────────────────────────────────────────────────────
    # PUBLISHERS
    # ──────────────────────────────────────────────────────────────────────

    def _publish_e_stop(self, active: bool) -> None:
        msg      = Bool()
        msg.data = active
        self._e_stop_pub.publish(msg)

    def _publish_zone(self, zone: str) -> None:
        msg      = String()
        msg.data = zone
        self._zone_pub.publish(msg)

    def _publish_stats(self) -> None:
        status = (
            f'[emergency_stop] '
            f'zone={self._current_zone} | '
            f'min_dist={self._min_distance:.3f}m | '
            f'danger_rays={self._danger_count} | '
            f'caution_rays={self._caution_count} | '
            f'total_scans={self._scan_count} | '
            f'e_stop_activations={self._e_stop_activations}'
        )
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)
        if self._current_zone == ZONE_CLEAR:
            self.get_logger().info(status)
        else:
            self.get_logger().warn(status)


# ──────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
