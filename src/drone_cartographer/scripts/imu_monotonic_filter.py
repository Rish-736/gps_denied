#!/usr/bin/env python3
"""
IMU timestamp monotoniser  --  makes the Gazebo IMU safe for Cartographer.

Cartographer's ImuTracker asserts that every IMU sample is strictly newer than
the last (imu_tracker.cc:40  CHECK time_ <= time). The ros_gz_bridge, however,
occasionally delivers IMU messages out of order or with a duplicate/earlier
stamp (seen: time going backwards ~40 us). One such sample makes Cartographer
abort the whole node -- which took down SLAM mid-flight.

This node sits between the bridge and Cartographer: it republishes /imu only
when the stamp is STRICTLY greater than the last one forwarded, dropping the
rare out-of-order sample. At 250 Hz, dropping a handful per second is invisible
to the scan matcher but keeps the timestamp stream monotonic, so Cartographer
never aborts.

    bridge -> /imu_raw  ->  [this node]  ->  /imu  -> cartographer
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class ImuMonotonicFilter(Node):
    def __init__(self):
        super().__init__('imu_monotonic_filter')
        self.declare_parameter('input_topic', '/imu_raw')
        self.declare_parameter('output_topic', '/imu')
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # Sensor data: best-effort, keep-last. Matches the bridge's stream.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        self.pub = self.create_publisher(Imu, out_topic, qos)
        self.create_subscription(Imu, in_topic, self._on_imu, qos)

        self._last_ns = None
        self._dropped = 0
        self._passed = 0
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f'imu_monotonic_filter: {in_topic} -> {out_topic} '
            '(dropping non-increasing stamps for Cartographer)')

    def _on_imu(self, msg):
        ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self._last_ns is not None and ns <= self._last_ns:
            self._dropped += 1
            return                          # out of order / duplicate -> drop
        self._last_ns = ns
        self._passed += 1
        self.pub.publish(msg)

    def _report(self):
        if self._dropped:
            self.get_logger().info(
                f'IMU stamps: {self._passed} forwarded, {self._dropped} dropped '
                '(out-of-order) so far')


def main():
    rclpy.init()
    node = ImuMonotonicFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
