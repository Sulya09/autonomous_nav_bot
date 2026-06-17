#!/usr/bin/env python3
"""
src/sensor_integration/imu_processor.py
════════════════════════════════════════
ROS 2 Node: imu_processor

WHY THIS NODE EXISTS
────────────────────
The raw /imu topic from the Gazebo IMU plugin contains Gaussian noise on
both the accelerometer and gyroscope channels. The robot_localization EKF
node (Step 4) does do its own statistical filtering, BUT it is sensitive
to "spike" readings — momentary, wildly incorrect values that can throw
off the Kalman filter's covariance estimates.

This node acts as a "front door" validator:

  Gazebo / Hardware IMU
         │
         ▼
    /imu   (raw, noisy, occasional spikes)
         │
         ▼
  [imu_processor]
    ├─ reject spikes (readings beyond physical limits)
    ├─ moving average filter on accel + gyro
    └─ repack message with corrected values
         │
         ▼
  /imu/filtered  (clean, ready for robot_localization EKF)

IMU MESSAGE FIELDS (sensor_msgs/Imu)
────────────────────────────────────
  orientation            — quaternion (x, y, z, w) — Gazebo computes this
  angular_velocity       — gyroscope reading   (rad/s)
  linear_acceleration    — accelerometer reading (m/s²)

We filter angular_velocity and linear_acceleration.
We pass orientation through unchanged — it's already integrated and smooth.

SPIKE DETECTION
───────────────
A real MPU-6050 IMU can read at most ±250 °/s gyro (≈ 4.4 rad/s) and
±2g accel (≈ 19.6 m/s²). We use generous thresholds to reject clear errors
without filtering out legitimate aggressive motion.

PARAMETERS  (set in config/sensors.yaml)
──────────────────────────────────────────
  input_topic      (str)  : /imu
  output_topic     (str)  : /imu/filtered
  filter_window    (int)  : 5       — moving average buffer size per axis
  max_accel_ms2    (float): 50.0    — spike threshold (m/s²), ~5g
  max_gyro_rads    (float): 10.0    — spike threshold (rad/s), ~573 °/s

ROS INTERFACES
──────────────
  Subscribes:  /imu               → sensor_msgs/Imu
  Publishes:   /imu/filtered      → sensor_msgs/Imu
               /imu/stats         → std_msgs/String  (every 5 s)
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class ImuProcessor(Node):
    """
    Validates raw IMU data, rejects spikes, applies a per-axis
    moving average filter, and republishes clean IMU messages.
    """

    def __init__(self):
        super().__init__('imu_processor')

        # ── Declare parameters ─────────────────────────────────────────────
        self.declare_parameter('input_topic',    '/imu')
        self.declare_parameter('output_topic',   '/imu/filtered')
        self.declare_parameter('filter_window',   5)
        self.declare_parameter('max_accel_ms2',  50.0)
        self.declare_parameter('max_gyro_rads',  10.0)
        self.declare_parameter('stats_interval_s', 5.0)

        # ── Read parameters ────────────────────────────────────────────────
        input_topic        = self.get_parameter('input_topic').value
        output_topic       = self.get_parameter('output_topic').value
        self.filter_window = self.get_parameter('filter_window').value
        self.max_accel     = self.get_parameter('max_accel_ms2').value
        self.max_gyro      = self.get_parameter('max_gyro_rads').value
        stats_interval     = self.get_parameter('stats_interval_s').value

        # ── Per-axis ring buffers for moving average ───────────────────────
        # deque(maxlen=N) automatically drops the oldest element when full,
        # giving us a sliding window of the last N valid readings per axis.
        w = self.filter_window
        self._ax_buf = deque(maxlen=w)  # accel x
        self._ay_buf = deque(maxlen=w)  # accel y
        self._az_buf = deque(maxlen=w)  # accel z
        self._gx_buf = deque(maxlen=w)  # gyro x
        self._gy_buf = deque(maxlen=w)  # gyro y
        self._gz_buf = deque(maxlen=w)  # gyro z

        # ── QoS: sensor data profile ───────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ── Subscriber & publishers ────────────────────────────────────────
        self.subscription  = self.create_subscription(
            Imu, input_topic, self.imu_callback, sensor_qos
        )
        self.filtered_pub  = self.create_publisher(Imu, output_topic, sensor_qos)
        self.stats_pub     = self.create_publisher(String, '/imu/stats', 10)

        # ── Diagnostics counters ───────────────────────────────────────────
        self._msg_received = 0
        self._spikes_rejected = 0

        # Periodic stats report
        self.create_timer(stats_interval, self._publish_stats)

        self.get_logger().info(
            f'IMU Processor ready\n'
            f'  Input : {input_topic}\n'
            f'  Output: {output_topic}\n'
            f'  Filter window : {self.filter_window} samples\n'
            f'  Spike limits  : accel ±{self.max_accel} m/s², gyro ±{self.max_gyro} rad/s'
        )

    # ──────────────────────────────────────────────────────────────────────
    # MAIN CALLBACK
    # ──────────────────────────────────────────────────────────────────────

    def imu_callback(self, msg: Imu) -> None:
        """
        Validate, filter, and republish a single IMU reading.
        """
        self._msg_received += 1

        # ── Extract raw values ─────────────────────────────────────────────
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # ── Spike detection ────────────────────────────────────────────────
        # If any axis is beyond the physical plausibility threshold, discard
        # the entire reading. A single bad reading is less harmful than
        # letting one corrupted value propagate into the EKF state.
        accel_mag = (ax**2 + ay**2 + az**2) ** 0.5
        if (
            abs(ax) > self.max_accel or
            abs(ay) > self.max_accel or
            abs(az) > self.max_accel or
            abs(gx) > self.max_gyro  or
            abs(gy) > self.max_gyro  or
            abs(gz) > self.max_gyro
        ):
            self._spikes_rejected += 1
            self.get_logger().warn(
                f'IMU spike rejected — '
                f'accel_mag={accel_mag:.1f} m/s²  '
                f'gyro=({gx:.2f},{gy:.2f},{gz:.2f}) rad/s'
            )
            return   # do not publish this reading

        # ── Update sliding window buffers ──────────────────────────────────
        self._ax_buf.append(ax);  self._ay_buf.append(ay);  self._az_buf.append(az)
        self._gx_buf.append(gx);  self._gy_buf.append(gy);  self._gz_buf.append(gz)

        # ── Compute moving average for each axis ───────────────────────────
        # np.mean on a deque iterates over only the filled entries, so
        # the first few readings use a smaller window automatically.
        filtered_ax = float(np.mean(self._ax_buf))
        filtered_ay = float(np.mean(self._ay_buf))
        filtered_az = float(np.mean(self._az_buf))
        filtered_gx = float(np.mean(self._gx_buf))
        filtered_gy = float(np.mean(self._gy_buf))
        filtered_gz = float(np.mean(self._gz_buf))

        # ── Build filtered message ─────────────────────────────────────────
        filtered = Imu()
        filtered.header = msg.header  # preserve original timestamp + frame_id

        # Orientation: already integrated in firmware/Gazebo, pass through.
        # robot_localization will use this directly for yaw estimation.
        filtered.orientation            = msg.orientation
        filtered.orientation_covariance = msg.orientation_covariance

        # Filtered linear acceleration
        filtered.linear_acceleration.x = filtered_ax
        filtered.linear_acceleration.y = filtered_ay
        filtered.linear_acceleration.z = filtered_az
        filtered.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Filtered angular velocity
        filtered.angular_velocity.x = filtered_gx
        filtered.angular_velocity.y = filtered_gy
        filtered.angular_velocity.z = filtered_gz
        filtered.angular_velocity_covariance = msg.angular_velocity_covariance

        self.filtered_pub.publish(filtered)

    # ──────────────────────────────────────────────────────────────────────
    # DIAGNOSTICS
    # ──────────────────────────────────────────────────────────────────────

    def _publish_stats(self) -> None:
        if self._msg_received == 0:
            return
        spike_pct = (self._spikes_rejected / self._msg_received) * 100
        msg       = String()
        msg.data  = (
            f'[imu_processor] '
            f'received={self._msg_received} | '
            f'spikes_rejected={self._spikes_rejected} ({spike_pct:.1f}%) | '
            f'filter_window={self.filter_window}'
        )
        self.stats_pub.publish(msg)
        self.get_logger().info(msg.data)


# ──────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
