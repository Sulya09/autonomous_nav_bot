#!/usr/bin/env python3
"""
src/sensor_integration/camera_processor.py
═══════════════════════════════════════════
ROS 2 Node: camera_processor

WHY THIS NODE EXISTS
────────────────────
The raw camera stream from Gazebo (or a USB camera driver) can have:
  • Frames with mismatched dimensions if the camera reinitialises
  • Incomplete data buffers if the USB bus drops bytes
  • No guaranteed encoding — might be RGB8 or BGR8 depending on driver

This node validates every frame before it reaches object detection or
visual odometry, and republishes on a clean topic with consistent QoS:

  Gazebo / Camera Driver
         │
         ▼
  /camera/image_raw   (unvalidated, mixed QoS)
         │
         ▼
  [camera_processor]
    ├─ validate dimensions, encoding, data length
    ├─ log frame rate and drop rate
    └─ republish validated frames
         │
         ▼
  /camera/image_processed  (validated, ready for vision pipeline)

IMPORTANT: QoS BRIDGING
────────────────────────
Gazebo publishes camera images with BEST_EFFORT reliability. Many vision
nodes subscribe with RELIABLE QoS by default. This mismatch silently
prevents any messages from being received. This node:
  • Subscribes with BEST_EFFORT (matches Gazebo)
  • Publishes with RELIABLE (works with downstream vision nodes)
…acting as a QoS bridge between the two worlds.

EXTENSIBILITY — FUTURE STEPS
─────────────────────────────
The process_image() method is deliberately stubbed. It can be extended to:
  • Convert BGR8 → RGB8  (if driver uses OpenCV convention)
  • Apply undistortion   (using camera_info calibration data)
  • Resize for neural networks that expect 224×224 or 416×416
  • Add overlay text / timestamp burn-in for debugging

PARAMETERS  (set in config/sensors.yaml)
──────────────────────────────────────────
  input_topic      (str)  : /camera/image_raw
  output_topic     (str)  : /camera/image_processed
  expected_width   (int)  : 640
  expected_height  (int)  : 480
  expected_encoding(str)  : rgb8
  stats_interval_s (float): 10.0

ROS INTERFACES
──────────────
  Subscribes:  /camera/image_raw         → sensor_msgs/Image  (BEST_EFFORT)
               /camera/camera_info       → sensor_msgs/CameraInfo
  Publishes:   /camera/image_processed   → sensor_msgs/Image  (RELIABLE)
               /camera/stats             → std_msgs/String
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


# Bytes per pixel for each supported encoding
BYTES_PER_PIXEL = {
    'rgb8':   3,
    'bgr8':   3,
    'rgba8':  4,
    'bgra8':  4,
    'mono8':  1,
    'mono16': 2,
}


class CameraProcessor(Node):
    """
    Validates and republishes camera frames, bridging QoS between
    Gazebo (BEST_EFFORT) and downstream vision nodes (RELIABLE).
    """

    def __init__(self):
        super().__init__('camera_processor')

        # ── Declare parameters ─────────────────────────────────────────────
        self.declare_parameter('input_topic',       '/camera/image_raw')
        self.declare_parameter('output_topic',      '/camera/image_processed')
        self.declare_parameter('expected_width',     640)
        self.declare_parameter('expected_height',    480)
        self.declare_parameter('expected_encoding',  'rgb8')
        self.declare_parameter('stats_interval_s',   10.0)

        # ── Read parameters ────────────────────────────────────────────────
        input_topic             = self.get_parameter('input_topic').value
        output_topic            = self.get_parameter('output_topic').value
        self.expected_width     = self.get_parameter('expected_width').value
        self.expected_height    = self.get_parameter('expected_height').value
        self.expected_encoding  = self.get_parameter('expected_encoding').value
        stats_interval          = self.get_parameter('stats_interval_s').value

        self._bytes_per_px = BYTES_PER_PIXEL.get(self.expected_encoding, 3)

        # ── QoS profiles ───────────────────────────────────────────────────
        # Subscribe with BEST_EFFORT to match Gazebo's publisher QoS.
        # If these don't match, ROS 2 silently delivers zero messages.
        incoming_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        # Publish with RELIABLE so downstream nodes with default QoS work.
        outgoing_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ── Subscriber & publishers ────────────────────────────────────────
        self.img_sub   = self.create_subscription(
            Image, input_topic, self.image_callback, incoming_qos
        )
        self.info_sub  = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, incoming_qos
        )
        self.img_pub   = self.create_publisher(Image, output_topic, outgoing_qos)
        self.stats_pub = self.create_publisher(String, '/camera/stats', 10)

        # ── State ──────────────────────────────────────────────────────────
        self.camera_info     = None          # stored when first received
        self._frames_rx      = 0             # total frames received
        self._frames_dropped = 0             # failed validation
        self._frames_ok      = 0             # published successfully
        self._last_stamp     = None          # for frame rate computation
        self._fps_ema        = 0.0           # exponential moving average FPS

        # Periodic stats report
        self.create_timer(stats_interval, self._publish_stats)

        self.get_logger().info(
            f'Camera Processor ready\n'
            f'  Input : {input_topic}\n'
            f'  Output: {output_topic}\n'
            f'  Expected: {self.expected_width}×{self.expected_height} {self.expected_encoding}\n'
            f'  QoS bridge: BEST_EFFORT → RELIABLE'
        )

    # ──────────────────────────────────────────────────────────────────────
    # CALLBACKS
    # ──────────────────────────────────────────────────────────────────────

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Cache the camera calibration data for use by vision algorithms."""
        if self.camera_info is None:
            self.get_logger().info(
                f'Camera info received — '
                f'{msg.width}×{msg.height}, '
                f'distortion model: {msg.distortion_model}'
            )
        self.camera_info = msg

    def image_callback(self, msg: Image) -> None:
        """
        Validate each frame and republish if it passes all checks.
        """
        self._frames_rx += 1
        self._update_fps(msg)

        # ── Validation: dimensions ─────────────────────────────────────────
        if msg.width != self.expected_width or msg.height != self.expected_height:
            self._frames_dropped += 1
            self.get_logger().warn(
                f'Frame size mismatch: got {msg.width}×{msg.height}, '
                f'expected {self.expected_width}×{self.expected_height} — dropped'
            )
            return

        # ── Validation: encoding ───────────────────────────────────────────
        if msg.encoding != self.expected_encoding:
            self._frames_dropped += 1
            self.get_logger().warn(
                f'Encoding mismatch: got "{msg.encoding}", '
                f'expected "{self.expected_encoding}" — dropped'
            )
            return

        # ── Validation: data integrity ─────────────────────────────────────
        # step = bytes per row (may include padding). Use it to compute
        # the expected total size rather than width * bpp * height.
        expected_bytes = msg.step * msg.height
        if len(msg.data) != expected_bytes:
            self._frames_dropped += 1
            self.get_logger().warn(
                f'Data length mismatch: got {len(msg.data)} bytes, '
                f'expected {expected_bytes} (step={msg.step}×h={msg.height}) — dropped'
            )
            return

        # ── Process and republish ──────────────────────────────────────────
        processed_msg = self._process_image(msg)
        self.img_pub.publish(processed_msg)
        self._frames_ok += 1

    # ──────────────────────────────────────────────────────────────────────
    # PROCESSING (stub — extend this in future steps)
    # ──────────────────────────────────────────────────────────────────────

    def _process_image(self, msg: Image) -> Image:
        """
        Apply any image transformations needed.

        Currently: pass through unchanged.
        Future extensions:
          • Colour space conversion (bgr8 → rgb8)
          • Lens undistortion using self.camera_info
          • Resize for neural network inference
          • Brightness/contrast normalisation
        """
        # Return the message as-is; header is preserved so timestamps are correct.
        return msg

    # ──────────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────────

    def _update_fps(self, msg: Image) -> None:
        """Track frame rate using an exponential moving average."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_stamp is not None and stamp > self._last_stamp:
            instant_fps = 1.0 / (stamp - self._last_stamp)
            alpha = 0.1   # smoothing factor (lower = smoother)
            self._fps_ema = alpha * instant_fps + (1.0 - alpha) * self._fps_ema
        self._last_stamp = stamp

    def _publish_stats(self) -> None:
        if self._frames_rx == 0:
            return
        drop_pct = (self._frames_dropped / self._frames_rx) * 100
        msg       = String()
        msg.data  = (
            f'[camera_processor] '
            f'received={self._frames_rx} | '
            f'published={self._frames_ok} | '
            f'dropped={self._frames_dropped} ({drop_pct:.1f}%) | '
            f'fps≈{self._fps_ema:.1f}'
        )
        self.stats_pub.publish(msg)
        self.get_logger().info(msg.data)


# ──────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
