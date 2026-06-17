#!/usr/bin/env python3
"""
src/localization/localization_monitor.py
════════════════════════════════════════
ROS 2 Node: localization_monitor

WHY THIS NODE EXISTS
────────────────────
AMCL is probabilistic — it can lose track of the robot's position,
especially after:
  • A long period of low sensor input (featureless corridor)
  • Carrying/pushing the robot without it moving (kidnapped robot problem)
  • The map no longer matching the environment (moved furniture)

When Nav2 tries to plan a path using a wrong position estimate, the robot
drives into walls. This monitor detects poor localisation EARLY so the
operator can intervene before that happens.

HOW CONFIDENCE IS COMPUTED
───────────────────────────
METRIC 1 — PARTICLE SPREAD
  The AMCL particle cloud is a set of hypotheses about where the robot
  could be. When localisation is GOOD, all particles cluster tightly.
  When localisation is POOR, particles are spread across the map.

  spread = mean Euclidean distance of all particles from their centroid
  confidence_spread = max(0, 1 − spread / max_spread)

METRIC 2 — POSE COVARIANCE
  AMCL also reports its own uncertainty as a 6×6 covariance matrix.
  We read the diagonal entries for x, y, and yaw.

  High covariance → AMCL is uncertain → low confidence

COMBINED SCORE
  confidence = 0.6 × confidence_spread + 0.4 × confidence_covariance
  Range: 0.0 (completely lost) → 1.0 (very confident)

WHAT TO DO WHEN CONFIDENCE IS LOW
───────────────────────────────────
  In RViz2: click "2D Pose Estimate" (top toolbar), then click on
  the map where the robot actually is. This reinitialises the particle
  filter around that location and convergence happens in seconds.

  Or via CLI:
    ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
      "{pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}}" --once

ROS INTERFACES
──────────────
  Subscribes:
    /particle_cloud    → geometry_msgs/PoseArray        (AMCL particles)
    /amcl_pose         → geometry_msgs/PoseWithCovarianceStamped (AMCL pose + covariance)

  Publishes:
    /localization/confidence  → std_msgs/Float32  (0.0 = lost, 1.0 = certain)
    /localization/status      → std_msgs/String   (human-readable diagnostics)

  Parameters (config via sensors.yaml or command line):
    confidence_threshold  (float, default 0.5) — warn below this
    max_particle_spread   (float, default 2.0) — spread (m) = 0 confidence
    max_pose_covariance   (float, default 0.5) — covariance = 0 confidence
    stats_interval_s      (float, default 5.0) — how often to log
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import Float32, String


class LocalizationMonitor(Node):
    """
    Monitors AMCL localisation quality and publishes a confidence score.
    Warns loudly when the robot is likely lost on the map.
    """

    def __init__(self):
        super().__init__('localization_monitor')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_particle_spread',  2.0)   # metres
        self.declare_parameter('max_pose_covariance',  0.5)   # m² or rad²
        self.declare_parameter('stats_interval_s',     5.0)

        self._threshold   = self.get_parameter('confidence_threshold').value
        self._max_spread  = self.get_parameter('max_particle_spread').value
        self._max_cov     = self.get_parameter('max_pose_covariance').value
        stats_interval    = self.get_parameter('stats_interval_s').value

        # ── QoS profiles ──────────────────────────────────────────────────
        # Nav2 AMCL publishes with reliable QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ── Subscriptions ─────────────────────────────────────────────────
        self._particle_sub = self.create_subscription(
            PoseArray,
            '/particle_cloud',
            self._particle_callback,
            reliable_qos
        )

        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_callback,
            reliable_qos
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._conf_pub   = self.create_publisher(Float32, '/localization/confidence', 10)
        self._status_pub = self.create_publisher(String,  '/localization/status',     10)

        # ── Internal state ────────────────────────────────────────────────
        self._particle_spread:   float | None = None
        self._pose_covariance_x: float | None = None
        self._pose_covariance_y: float | None = None
        self._pose_covariance_yaw: float | None = None
        self._combined_confidence: float = 0.0

        self._particle_callbacks = 0
        self._pose_callbacks     = 0
        self._low_conf_count     = 0   # consecutive low-confidence readings

        # Periodic diagnostics timer
        self.create_timer(stats_interval, self._publish_stats)

        self.get_logger().info(
            f'Localisation Monitor ready\n'
            f'  Confidence threshold : {self._threshold}\n'
            f'  Max particle spread  : {self._max_spread} m\n'
            f'  Max pose covariance  : {self._max_cov}\n'
            f'  Stats interval       : {stats_interval} s\n'
            f'\n'
            f'  If confidence drops below {self._threshold}:\n'
            f'  → Use "2D Pose Estimate" in RViz2 to reinitialise AMCL'
        )

    # ──────────────────────────────────────────────────────────────────────
    # CALLBACK 1: PARTICLE CLOUD
    # ──────────────────────────────────────────────────────────────────────

    def _particle_callback(self, msg: PoseArray) -> None:
        """
        Compute the spatial spread of the AMCL particle cloud.

        Spread = mean distance of each particle from the swarm centroid.
        A tight swarm means AMCL is confident about the robot's position.
        A spread-out swarm means AMCL has multiple competing hypotheses.
        """
        self._particle_callbacks += 1

        n = len(msg.poses)
        if n < 2:
            return   # not enough particles to compute meaningful spread

        # Extract 2D (x, y) positions — ignore z for a ground robot
        positions = np.array(
            [[p.position.x, p.position.y] for p in msg.poses],
            dtype=np.float64
        )

        centroid = np.mean(positions, axis=0)
        distances = np.linalg.norm(positions - centroid, axis=1)
        self._particle_spread = float(np.mean(distances))

        # Map spread → confidence: 0 spread = 1.0, max_spread = 0.0
        spread_conf = max(0.0, 1.0 - (self._particle_spread / self._max_spread))

        # Update combined confidence (weighted average with covariance score)
        cov_conf = self._covariance_confidence()
        self._combined_confidence = 0.6 * spread_conf + 0.4 * cov_conf

        # Publish confidence immediately on every particle update
        self._publish_confidence()

    # ──────────────────────────────────────────────────────────────────────
    # CALLBACK 2: AMCL POSE
    # ──────────────────────────────────────────────────────────────────────

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Extract AMCL's self-reported position uncertainty (covariance).

        The covariance matrix is 6×6 (x, y, z, roll, pitch, yaw),
        stored as a flat 36-element array in row-major order.
        We extract the diagonal variances for x (index 0), y (index 7),
        and yaw (index 35).

        High variance = AMCL is uncertain = low confidence.
        """
        self._pose_callbacks += 1

        cov = msg.pose.covariance    # flat 36-element array
        self._pose_covariance_x   = cov[0]    # variance of x  position (m²)
        self._pose_covariance_y   = cov[7]    # variance of y  position (m²)
        self._pose_covariance_yaw = cov[35]   # variance of yaw angle  (rad²)

    # ──────────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────────

    def _covariance_confidence(self) -> float:
        """
        Convert pose covariance into a 0–1 confidence score.
        Uses the maximum of x, y, yaw variance (worst-case axis).
        Returns 0.5 (neutral) if covariance data hasn't arrived yet.
        """
        if None in (self._pose_covariance_x,
                    self._pose_covariance_y,
                    self._pose_covariance_yaw):
            return 0.5   # no data yet — return neutral

        worst_cov = max(
            abs(self._pose_covariance_x),
            abs(self._pose_covariance_y),
            abs(self._pose_covariance_yaw)
        )
        return max(0.0, 1.0 - (worst_cov / self._max_cov))

    def _publish_confidence(self) -> None:
        """Publish the latest confidence score."""
        msg      = Float32()
        msg.data = float(self._combined_confidence)
        self._conf_pub.publish(msg)

        # Track consecutive low-confidence readings
        if self._combined_confidence < self._threshold:
            self._low_conf_count += 1
        else:
            self._low_conf_count = 0

        # Escalating warnings for persistent low confidence
        if self._low_conf_count == 5:
            self.get_logger().warn(
                f'⚠️  LOW LOCALISATION CONFIDENCE: {self._combined_confidence:.2f}\n'
                f'   Particle spread: {self._particle_spread:.3f} m\n'
                f'   → Use "2D Pose Estimate" in RViz2 to reinitialise AMCL'
            )
        elif self._low_conf_count == 20:
            self.get_logger().error(
                f'🚨 ROBOT MAY BE LOST (confidence: {self._combined_confidence:.2f})\n'
                f'   Navigation will be unreliable until pose is corrected.\n'
                f'   → Click "2D Pose Estimate" in RViz2 and click the robot\'s true location'
            )

    def _publish_stats(self) -> None:
        """Publish a human-readable status summary on the timer."""
        spread_str = f'{self._particle_spread:.3f}m' \
            if self._particle_spread is not None else 'waiting...'
        cov_x_str  = f'{self._pose_covariance_x:.4f}' \
            if self._pose_covariance_x is not None else 'waiting...'
        cov_y_str  = f'{self._pose_covariance_y:.4f}' \
            if self._pose_covariance_y is not None else 'waiting...'
        cov_yaw_str = f'{self._pose_covariance_yaw:.4f}' \
            if self._pose_covariance_yaw is not None else 'waiting...'

        level = '✅ GOOD' if self._combined_confidence >= self._threshold else '⚠️  LOW'

        status_text = (
            f'[localisation_monitor] {level} | '
            f'confidence={self._combined_confidence:.2f} | '
            f'spread={spread_str} | '
            f'cov_x={cov_x_str} cov_y={cov_y_str} cov_yaw={cov_yaw_str} | '
            f'particles_cb={self._particle_callbacks} '
            f'pose_cb={self._pose_callbacks}'
        )

        msg      = String()
        msg.data = status_text
        self._status_pub.publish(msg)

        if self._combined_confidence >= self._threshold:
            self.get_logger().info(status_text)
        else:
            self.get_logger().warn(status_text)


# ──────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
