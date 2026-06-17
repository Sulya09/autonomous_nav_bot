#!/usr/bin/env python3
"""
src/sensor_integration/lidar_processor.py
══════════════════════════════════════════
ROS 2 Node: lidar_processor

WHY THIS NODE EXISTS
────────────────────
The raw /scan topic from the Gazebo LiDAR plugin (or a real RPLIDAR driver)
contains imperfections:
  • NaN and Inf values — when the laser beam hits nothing in range
  • Readings slightly outside the sensor's valid range
  • Small jitter/noise on every measurement (~1 cm Gaussian noise)

Downstream nodes (SLAM Toolbox, Nav2 costmaps) work much better with
clean data. This node sits between the raw sensor and the rest of the stack:

  Gazebo / Hardware
       │
       ▼
  /scan  (raw, noisy)
       │
       ▼
  [lidar_processor]
       │
       ▼
  /scan/filtered  (clean, clamped, smoothed)
       │
       ▼
  SLAM, Nav2, obstacle avoidance

PROCESSING PIPELINE
───────────────────
  1. NaN / Inf → replace with range_max ("no obstacle here")
  2. Range clamping → clip values to [range_min, range_max]
  3. Circular moving average → smooth noise across adjacent rays
     (circular = handles the 0°/360° wrap-around correctly)
  4. Republish clean scan on /scan/filtered

PARAMETERS  (set in config/sensors.yaml)
──────────────────────────────────────────
  input_topic      (str)  : /scan
  output_topic     (str)  : /scan/filtered
  range_min        (float): 0.12 m  — minimum valid distance
  range_max        (float): 3.5  m  — maximum valid distance
  enable_smoothing (bool) : true    — apply moving average filter
  smoothing_window (int)  : 3       — number of rays to average

ROS INTERFACES
──────────────
  Subscribes:  /scan               → sensor_msgs/LaserScan
  Publishes:   /scan/filtered      → sensor_msgs/LaserScan
               /scan/stats         → std_msgs/String  (every 5 s)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np


class LidarProcessor(Node):
    """
    Subscribes to raw LaserScan, applies validation + smoothing,
    and publishes a cleaned scan ready for SLAM and navigation.
    """

    def __init__(self):
        super().__init__('lidar_processor')

        # ── Declare ROS 2 parameters ───────────────────────────────────────
        # Parameters can be overridden at launch time via a YAML file or
        # the command line: --ros-args -p smoothing_window:=5
        self.declare_parameter('input_topic',      '/scan')
        self.declare_parameter('output_topic',     '/scan/filtered')
        self.declare_parameter('range_min',         0.12)
        self.declare_parameter('range_max',         3.50)
        self.declare_parameter('enable_smoothing',  True)
        self.declare_parameter('smoothing_window',  3)
        self.declare_parameter('stats_interval_s',  5.0)

        # ── Read parameter values ──────────────────────────────────────────
        input_topic         = self.get_parameter('input_topic').value
        output_topic        = self.get_parameter('output_topic').value
        self.range_min      = self.get_parameter('range_min').value
        self.range_max      = self.get_parameter('range_max').value
        self.enable_smoothing = self.get_parameter('enable_smoothing').value
        self.smoothing_window = self.get_parameter('smoothing_window').value
        stats_interval      = self.get_parameter('stats_interval_s').value

        # ── QoS profile for sensor data ────────────────────────────────────
        # SENSOR_DATA profile: Best-Effort reliability is appropriate for
        # streaming sensor data. We don't need every packet delivered
        # reliably — we just need the most recent reading.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ── Subscriber: raw scan ───────────────────────────────────────────
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            sensor_qos
        )

        # ── Publishers ─────────────────────────────────────────────────────
        self.filtered_pub = self.create_publisher(LaserScan, output_topic, sensor_qos)
        self.stats_pub    = self.create_publisher(String, '/scan/stats', 10)

        # ── Diagnostics counters ───────────────────────────────────────────
        self._scan_count   = 0   # total scans received
        self._nan_count    = 0   # total NaN readings fixed across all scans
        self._clamp_count  = 0   # total readings clamped

        # ── Periodic statistics log ────────────────────────────────────────
        self.create_timer(stats_interval, self._publish_stats)

        self.get_logger().info(
            f'LiDAR Processor ready\n'
            f'  Input : {input_topic}\n'
            f'  Output: {output_topic}\n'
            f'  Range : [{self.range_min}, {self.range_max}] m\n'
            f'  Smoothing: {"ON (window=" + str(self.smoothing_window) + ")" if self.enable_smoothing else "OFF"}'
        )

    # ──────────────────────────────────────────────────────────────────────
    # MAIN CALLBACK
    # ──────────────────────────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan) -> None:
        """
        Called every time a new LaserScan arrives.
        Filters the range data and republishes a clean message.
        """
        self._scan_count += 1

        # Convert the list of ranges to a numpy array for fast vectorised ops.
        # float32 matches the LaserScan field type exactly.
        ranges = np.array(msg.ranges, dtype=np.float32)

        # ── Step 1: Replace NaN / Inf with range_max ──────────────────────
        # A NaN in a laser scan means "no return" — the beam went past the
        # sensor's max range without hitting anything. We treat this as
        # "free space up to range_max" so costmaps don't mark it as unknown.
        invalid_mask = ~np.isfinite(ranges)
        nan_in_scan  = int(np.sum(invalid_mask))
        self._nan_count += nan_in_scan
        ranges[invalid_mask] = self.range_max

        # ── Step 2: Clamp to valid range ──────────────────────────────────
        # Readings below range_min are inside the sensor's blind spot and
        # unreliable. Readings above range_max are physically impossible.
        before_clamp   = ranges.copy()
        ranges         = np.clip(ranges, self.range_min, self.range_max)
        self._clamp_count += int(np.sum(ranges != before_clamp))

        # ── Step 3: Circular moving average (optional noise smoothing) ─────
        # "Circular" is key: rays 0° and 359° are physically adjacent, so
        # the smoothing window must wrap around the array ends.
        if self.enable_smoothing and self.smoothing_window > 1:
            ranges = self._circular_moving_average(ranges, self.smoothing_window)

        # ── Step 4: Build and publish the filtered LaserScan message ───────
        filtered_msg                 = LaserScan()
        filtered_msg.header          = msg.header     # keep original timestamp + frame_id
        filtered_msg.angle_min       = msg.angle_min
        filtered_msg.angle_max       = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment  = msg.time_increment
        filtered_msg.scan_time       = msg.scan_time
        filtered_msg.range_min       = self.range_min  # updated range limits
        filtered_msg.range_max       = self.range_max
        filtered_msg.ranges          = ranges.tolist()
        filtered_msg.intensities     = msg.intensities  # pass through unchanged

        self.filtered_pub.publish(filtered_msg)

    # ──────────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────────

    def _circular_moving_average(
        self, data: np.ndarray, window: int
    ) -> np.ndarray:
        """
        Apply a moving average that wraps correctly around a 360° scan.

        Normal convolution treats array edges as boundaries, so rays near
        0° would only be averaged with rays on one side. We fix this by
        tiling the ends of the array before convolving:

            [..., 358°, 359°] + [0°, 1°, ..., 359°] + [0°, 1°, ...]
                 tail padding       original data        head padding

        After convolution with mode='valid', we recover exactly N elements
        — each one averaged with its true circular neighbours.
        """
        half  = window // 2
        # Circular extension: append tail to front, head to back
        extended = np.concatenate([data[-half:], data, data[:half]])
        kernel   = np.ones(window, dtype=np.float32) / window
        smoothed = np.convolve(extended, kernel, mode='valid')
        return smoothed[: len(data)].astype(np.float32)

    def _publish_stats(self) -> None:
        """Log and publish a simple diagnostics summary every N seconds."""
        if self._scan_count == 0:
            return

        msg      = String()
        msg.data = (
            f'[lidar_processor] '
            f'scans={self._scan_count} | '
            f'nan_replacements={self._nan_count} | '
            f'range_clamps={self._clamp_count}'
        )
        self.stats_pub.publish(msg)
        self.get_logger().info(msg.data)


# ──────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    try:
        rclpy.spin(node)          # block here, processing callbacks
    except KeyboardInterrupt:
        pass                       # Ctrl+C is a clean exit
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
