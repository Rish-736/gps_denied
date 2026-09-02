#!/usr/bin/env python3
"""
Fixes risk #1 from mission_rules_crucial_risks: PX4 has no idea where the
drone is. Cartographer knows (map -> base_link via TF), but nothing was
feeding that into PX4's own EKF2. Without a position estimate, PX4-side
failsafes that need position (geofence, RTL/emergency-recall) cannot work.

This node looks up the map->base_link TF that Cartographer publishes and
republishes it as a PoseStamped on /mavros/vision_pose/pose, which MAVROS
forwards to PX4 as an external vision position estimate (MAVLink
VISION_POSITION_ESTIMATE). PX4 then fuses it into EKF2 -- but only once the
matching EKF2_EV_CTRL/EKF2_GPS_CTRL/EKF2_HGT_REF params are set (see
docs/px4_ekf2_vision_fusion_setup.md for the exact values and why).
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
# TransformException is the base class of LookupException, ConnectivityException
# and ExtrapolationException -- catch it so a transient disconnected TF tree at
# startup (map<->base_link not linked yet) doesn't crash the node.
from tf2_ros import Buffer, TransformListener, TransformException


class VisionPoseBridge(Node):
    def __init__(self):
        super().__init__('vision_pose_bridge')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('rate_hz', 30.0)
        # Safeguard: reject a pose that jumps more than this in one cycle. At
        # 30Hz this is ~15 m/s -- far above our slow drone -- so it only trips on
        # a Cartographer glitch/relocalisation. Without it, one bad SLAM frame
        # would teleport PX4's fused estimate and diverge the EKF (what we saw
        # when the IMU was first tried). Feeding PX4 a smooth estimate is safer
        # than feeding it every raw jump.
        self.declare_parameter('max_jump_m', 0.5)

        self.map_frame = self.get_parameter('map_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        rate_hz = self.get_parameter('rate_hz').value
        self.max_jump = self.get_parameter('max_jump_m').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)

        self._warned_once = False
        self._last_xyz = None       # last accepted position, for jump rejection
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'vision_pose_bridge: {self.map_frame} -> {self.body_frame} '
            f'=> /mavros/vision_pose/pose @ {rate_hz} Hz')

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, Time())
        except TransformException as exc:
            if not self._warned_once:
                self.get_logger().warn(f'TF not ready yet (will keep retrying): {exc}')
                self._warned_once = True
            return

        msg = PoseStamped()
        # Stamp with wall-clock 'now', NOT the TF's stamp. In SITL, Cartographer/
        # TF run on sim time (small numbers) while MAVROS/PX4 run on wall-clock
        # time; passing the sim-time stamp makes PX4 see the measurement as
        # ~1.7e9 seconds old and reject every horizontal update (estimator shows
        # const_pos_mode / pos_horiz_abs=false -> takeoff refused). This node
        # runs on the system clock (no use_sim_time), so now() is wall-clock and
        # aligns with what MAVROS/PX4 expect. EKF2_EV_DELAY absorbs the small
        # latency. On real hardware there is no sim/wall split, so this is also
        # correct there.
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z

        # Jump guard: skip a frame that teleports (SLAM glitch). The first frame
        # is always accepted (no previous to compare).
        if self._last_xyz is not None:
            import math
            jump = math.dist((x, y, z), self._last_xyz)
            if jump > self.max_jump:
                self.get_logger().warn(
                    f'Skipping {jump:.1f}m pose jump (SLAM glitch) to protect EKF')
                return
        self._last_xyz = (x, y, z)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = tf.transform.rotation
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = VisionPoseBridge()
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
