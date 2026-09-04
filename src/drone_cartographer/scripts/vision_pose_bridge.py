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
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
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
        # SLAM-loss detection. In the first full flight Cartographer lost
        # tracking mid-run: the map->base_link TF FROZE (same value forever).
        # The old code kept forwarding that frozen pose to PX4, which then fought
        # the IMU as the drone physically drifted -> the EKF diverged (offset ran
        # to 15m) -> the drone shot up and crashed. The fix: notice the TF stamp
        # has stopped advancing and STOP publishing, so PX4 coasts on its own IMU
        # instead of being fed a lie. We also broadcast /slam_ok so the follower
        # can hover instead of flying blind. `Time()` lookups always "succeed"
        # even when frozen (the last transform stays available), so freshness has
        # to be judged from the transform's own timestamp, not from the lookup.
        self.declare_parameter('stale_timeout_sec', 0.5)
        # Divergence: a SLAM pose sliding faster than the drone can fly means
        # Cartographer has lost tracking. Our hard ceiling is ~1.5 m/s, so 2.5
        # leaves headroom for honest fast frames while still catching a runaway.
        self.declare_parameter('max_slam_speed', 2.5)
        self.declare_parameter('diverge_frames', 5)

        self.map_frame = self.get_parameter('map_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        rate_hz = self.get_parameter('rate_hz').value
        self.max_jump = self.get_parameter('max_jump_m').value
        self.stale_timeout = self.get_parameter('stale_timeout_sec').value
        self.max_slam_speed = self.get_parameter('max_slam_speed').value
        self.diverge_frames = self.get_parameter('diverge_frames').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        # Latched so a subscriber that starts late still learns the current
        # SLAM health immediately.
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
        latched = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.slam_ok_pub = self.create_publisher(Bool, '/slam_ok', latched)

        self._warned_once = False
        self._last_xyz = None       # last accepted position, for jump rejection
        self._last_xyz_wall = self.get_clock().now()
        self._diverge_count = 0     # consecutive implausible-speed frames
        self._last_stamp_ns = None  # last TF stamp, to detect a frozen SLAM pose
        self._stamp_changed_wall = self.get_clock().now()
        self._slam_ok = None        # tri-state so the first result always logs
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'vision_pose_bridge: {self.map_frame} -> {self.body_frame} '
            f'=> /mavros/vision_pose/pose @ {rate_hz} Hz')

    def _set_slam_ok(self, ok):
        if ok != self._slam_ok:
            self._slam_ok = ok
            self.slam_ok_pub.publish(Bool(data=ok))
            if ok:
                self.get_logger().info('SLAM tracking OK -> resuming vision pose')
            else:
                self.get_logger().error(
                    'SLAM lost (pose frozen or diverging) -> pausing vision feed '
                    'so PX4 coasts on IMU; follower will hover/land')

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, Time())
        except TransformException as exc:
            if not self._warned_once:
                self.get_logger().warn(f'TF not ready yet (will keep retrying): {exc}')
                self._warned_once = True
            return

        # Freshness: has the transform's OWN timestamp advanced since last tick?
        # A frozen Cartographer keeps the same stamp; a healthy one advances it.
        now = self.get_clock().now()
        stamp_ns = Time.from_msg(tf.header.stamp).nanoseconds
        if stamp_ns != self._last_stamp_ns:
            self._last_stamp_ns = stamp_ns
            self._stamp_changed_wall = now
        stale_for = (now - self._stamp_changed_wall).nanoseconds / 1e9
        if self._last_stamp_ns is not None and stale_for > self.stale_timeout:
            self._set_slam_ok(False)
            return                  # do NOT feed PX4 a frozen pose
        self._set_slam_ok(True)

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

        # DIVERGENCE guard. The freeze check above catches a stuck pose; this
        # catches the OTHER failure we saw -- the SLAM pose sliding away at an
        # impossible speed (Cartographer mis-matching in the maze's look-alike
        # corridors). A single teleport trips max_jump; a sustained implausible
        # velocity (faster than the drone can physically fly) means SLAM has lost
        # the plot, so we stop feeding PX4 and hover. Counted over consecutive
        # frames so one noisy sample doesn't false-trip.
        now_wall = self.get_clock().now()
        if self._last_xyz is not None:
            jump = math.dist((x, y, z), self._last_xyz)
            dt = (now_wall - self._last_xyz_wall).nanoseconds / 1e9
            speed = jump / dt if dt > 1e-3 else 0.0
            if jump > self.max_jump:
                self.get_logger().warn(
                    f'Skipping {jump:.2f}m pose jump (SLAM glitch) to protect EKF')
                return
            if speed > self.max_slam_speed:
                self._diverge_count += 1
                if self._diverge_count >= self.diverge_frames:
                    self._set_slam_ok(False)
                    return          # SLAM diverging -> do not feed PX4
            else:
                self._diverge_count = 0
        self._last_xyz = (x, y, z)
        self._last_xyz_wall = now_wall

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
