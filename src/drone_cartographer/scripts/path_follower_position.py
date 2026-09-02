#!/usr/bin/env python3
"""
Position-control path follower -- flies a map-frame path with PX4 POSITION
setpoints.

WHY THIS IS WRITTEN THE WAY IT IS
=================================
Nav2's MPPI velocity controller stalled the drone (commanded ~0 m/s with a
valid 128-pose path), so control is done here with position setpoints, which
PX4's position controller tracks precisely -- the standard way indoor drones
fly.

THE FRAME BUG THIS FIXES (root cause of "the drone flies out of the maze")
-------------------------------------------------------------------------
There are TWO different position estimates in this system and the previous
version mixed them:

  * 'map'  -- Cartographer's SLAM frame. /planned_path, /map and the Nav2
              planner all live here. This is the frame the maze exists in.
  * PX4's local frame -- what /mavros/local_position/pose reports and what
              /mavros/setpoint_position/local is interpreted in. PX4's EKF
              builds it from IMU + the vision pose we feed it, so it is only
              *approximately* map, and it can be offset AND rotated (EKF yaw
              takes time to align, and the vision fusion has lag).

The old code read its position from PX4's local frame, compared it against
map-frame path points, and published map-frame numbers straight into the local
setpoint topic. Any rotation between the two frames turns "go 3m up the
corridor" into "go 3m in some other direction" -- which sends the drone out
through the entry gap no matter which frontier the explorer picked.

The fix, done properly:
  1. Measure the drone's position in the SAME frame as the path -- straight
     from the map->base_link TF that Cartographer publishes.
  2. Continuously estimate the rigid 2D transform between map and PX4's local
     frame by comparing the two live pose sources.
  3. Do all path reasoning in map, then transform the final setpoint into
     PX4's local frame before publishing.
That is correct whether the frames happen to agree or not, and the estimated
offset is logged so the disagreement is visible instead of silent.

MONOTONIC PROGRESS
------------------
Path progress only ever moves FORWARD along the path. The old version re-picked
the nearest path point every tick, so after any overshoot it would latch onto a
point behind the drone and command it backwards -- the "moves a second, comes
back to the same spot" oscillation.

SPEED
-----
Speed is not commanded here. The carrot is held a fixed distance ahead, and
PX4's own MPC_XY_CRUISE / MPC_ACC_HOR limits (set in the airframe file to
indoor values) turn that steady position error into steady, gentle motion.
One place owns the flight envelope: the autopilot.

Yaw faces the direction of travel -- the behaviour that flew the maze cleanly.
Rate limiting is left to PX4's MPC_YAWRAUTO_MAX rather than fought here.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PathFollowerPosition(Node):
    def __init__(self):
        super().__init__('path_follower_position')
        self.declare_parameter('target_altitude', 1.2)
        # Distance the carrot is held ahead of the drone along the path. This
        # IS the position error PX4 sees, so it sets the cruise speed together
        # with MPC_XY_P: 0.6m * 0.95 ~= 0.57 m/s, under MPC_XY_CRUISE (0.8).
        # Bigger = faster but cuts corners harder in tight maze turns.
        self.declare_parameter('lookahead', 0.6)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'base_link')
        # Only rotate to face travel when the carrot is meaningfully far away;
        # closer than this the bearing is noise and yaw would hunt.
        self.declare_parameter('yaw_min_distance', 0.25)
        # Hold position if Cartographer's pose goes stale -- never keep flying
        # on a frozen estimate.
        self.declare_parameter('pose_timeout_sec', 1.0)

        self.target_alt = self.get_parameter('target_altitude').value
        self.lookahead = self.get_parameter('lookahead').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.map_frame = self.get_parameter('map_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        self.yaw_min_dist = self.get_parameter('yaw_min_distance').value
        self.pose_timeout = self.get_parameter('pose_timeout_sec').value
        rate_hz = self.get_parameter('rate_hz').value

        # --- pose in the MAP frame (Cartographer, same frame as the path) ---
        self.mx = self.my = self.myaw = 0.0
        self.map_ok = False
        self.last_map_time = None

        # --- pose in PX4's LOCAL frame (what setpoints are interpreted in) ---
        self.lx = self.ly = self.lz = self.lyaw = 0.0
        self.local_ok = False

        # --- estimated map -> local rigid transform (yaw, tx, ty) ---
        self.t_yaw = self.t_x = self.t_y = 0.0
        self.t_init = False

        # --- path state ---
        self.path = []          # [(x, y), ...] in the map frame
        self.prog = 0           # monotonic index: progress only moves forward
        self.reached_sent = False

        # Don't chase a path until we've actually climbed. The explorer
        # publishes a path within ~2s of startup; without this gate the drone
        # lurches sideways toward a waypoint while still on the ground.
        self.airborne = False
        self.hold_xy = None     # latched hold point (NOT the live pose)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Path, '/planned_path', self._on_path, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._on_local, sensor_qos)
        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.reached_pub = self.create_publisher(Bool, '/path_follower/reached', 10)
        # Published purely for RViz: shows exactly what the drone is chasing.
        self.carrot_pub = self.create_publisher(PoseStamped, '/follower/carrot', 5)

        self._align_log = 0
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'path_follower_position: map-frame control, alt={self.target_alt}m, '
            f'lookahead={self.lookahead}m @ {rate_hz} Hz')

    # ---------------------------------------------------------------- inputs
    def _on_path(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            return
        # A replan usually re-issues most of the same route. Resume at the point
        # on the NEW path nearest to where we already were, so progress isn't
        # thrown away and the drone doesn't get sent back to the path's start.
        if self.map_ok:
            self.prog = min(range(len(pts)),
                            key=lambda i: (pts[i][0] - self.mx) ** 2
                            + (pts[i][1] - self.my) ** 2)
        else:
            self.prog = 0
        self.path = pts
        self.reached_sent = False

    def _on_local(self, msg: PoseStamped):
        self.lx = msg.pose.position.x
        self.ly = msg.pose.position.y
        self.lz = msg.pose.position.z
        self.lyaw = yaw_of(msg.pose.orientation)
        self.local_ok = True
        if not self.airborne and self.lz >= 0.9 * self.target_alt:
            self.airborne = True
            self.get_logger().info(
                f'Reached {self.lz:.2f}m -> path following enabled')

    def _read_map_pose(self):
        """Drone pose in the MAP frame, straight from Cartographer's TF.

        Time() means 'latest available', which sidesteps the sim-time (SLAM)
        vs wall-clock (MAVROS) split this node sits across.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, Time())
        except TransformException:
            return
        self.mx = tf.transform.translation.x
        self.my = tf.transform.translation.y
        self.myaw = yaw_of(tf.transform.rotation)
        self.map_ok = True
        self.last_map_time = self.get_clock().now()

    # ------------------------------------------------------------ transform
    def _update_transform(self):
        """Estimate the rigid 2D transform taking map coords -> PX4 local coords.

        Both frames track the same physical drone, so comparing the two live
        poses gives the offset directly. Smoothed hard because the true offset
        is near-constant while both inputs are noisy; a jittery transform would
        shake the setpoint.
        """
        dyaw = wrap(self.lyaw - self.myaw)
        c, s = math.cos(dyaw), math.sin(dyaw)
        tx = self.lx - (c * self.mx - s * self.my)
        ty = self.ly - (s * self.mx + c * self.my)

        if not self.t_init:
            self.t_yaw, self.t_x, self.t_y = dyaw, tx, ty
            self.t_init = True
            return
        a = 0.05
        self.t_yaw = wrap(self.t_yaw + a * wrap(dyaw - self.t_yaw))
        self.t_x += a * (tx - self.t_x)
        self.t_y += a * (ty - self.t_y)

    def _map_to_local(self, x, y, yaw):
        c, s = math.cos(self.t_yaw), math.sin(self.t_yaw)
        return (c * x - s * y + self.t_x,
                s * x + c * y + self.t_y,
                wrap(yaw + self.t_yaw))

    # ----------------------------------------------------------------- logic
    def _hold(self):
        """Latch a fixed hold point instead of re-commanding the live pose.

        Commanding wherever the drone currently is makes the setpoint follow
        the drone's own drift, so it never actually holds station.
        """
        if self.hold_xy is None:
            self.hold_xy = (self.mx, self.my)
        return self.hold_xy[0], self.hold_xy[1], self.myaw

    def _carrot(self):
        """Return (x, y, yaw) in the MAP frame: a point ~lookahead ahead."""
        if not self.airborne or not self.path:
            return self._hold()

        # Arrived at the end of the path?
        gx, gy = self.path[-1]
        if math.hypot(gx - self.mx, gy - self.my) < self.goal_tol:
            if not self.reached_sent:
                self.reached_pub.publish(Bool(data=True))
                self.reached_sent = True
                self.get_logger().info(
                    f'Goal reached at ({self.mx:.2f}, {self.my:.2f})')
            self.hold_xy = (gx, gy)
            return gx, gy, self.myaw
        self.hold_xy = None     # actively flying; drop any stale hold point

        # Advance progress FORWARD ONLY. Scanning from the current index means
        # an overshoot can never latch onto a point behind the drone and
        # command a reversal.
        best_i, best_d = self.prog, float('inf')
        for i in range(self.prog, len(self.path)):
            d = math.hypot(self.path[i][0] - self.mx, self.path[i][1] - self.my)
            if d < best_d:
                best_d, best_i = d, i
            elif d > best_d + 1.5:
                break           # clearly moving away down the path; stop looking
        self.prog = best_i

        # Walk forward along the path accumulating ARC LENGTH (not straight-line
        # distance from the drone) so a path that doubles back around a corner
        # still yields a carrot that is genuinely further along the route.
        tx, ty = self.path[-1]
        acc = 0.0
        for i in range(self.prog, len(self.path) - 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            acc += math.hypot(bx - ax, by - ay)
            if acc >= self.lookahead:
                tx, ty = bx, by
                break

        yaw = self.myaw
        d = math.hypot(tx - self.mx, ty - self.my)
        if d > self.yaw_min_dist:
            yaw = math.atan2(ty - self.my, tx - self.mx)
        return tx, ty, yaw

    def _tick(self):
        self._read_map_pose()
        if not (self.map_ok and self.local_ok):
            return
        self._update_transform()

        # Stale SLAM pose -> stop moving rather than fly blind on a frozen estimate.
        stale = (self.get_clock().now() - self.last_map_time).nanoseconds / 1e9
        if stale > self.pose_timeout:
            tx, ty, tyaw = self._hold()
        else:
            tx, ty, tyaw = self._carrot()

        # Everything above was reasoned in MAP. Convert to PX4's local frame,
        # which is what the setpoint topic is actually interpreted in.
        sx, sy, syaw = self._map_to_local(tx, ty, tyaw)

        qx, qy, qz, qw = yaw_to_quat(syaw)
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = sx
        sp.pose.position.y = sy
        sp.pose.position.z = self.target_alt
        sp.pose.orientation.x = qx
        sp.pose.orientation.y = qy
        sp.pose.orientation.z = qz
        sp.pose.orientation.w = qw
        self.sp_pub.publish(sp)

        carrot = PoseStamped()
        carrot.header.stamp = sp.header.stamp
        carrot.header.frame_id = self.map_frame
        carrot.pose.position.x = tx
        carrot.pose.position.y = ty
        carrot.pose.position.z = self.target_alt
        carrot.pose.orientation.w = 1.0
        self.carrot_pub.publish(carrot)

        # Surface the map<->local disagreement instead of letting it stay
        # silent -- a large offset here is the frame bug reappearing.
        self._align_log += 1
        if self._align_log % 100 == 0:
            off = math.hypot(self.t_x, self.t_y)
            self.get_logger().info(
                f'map=({self.mx:.2f},{self.my:.2f}) '
                f'px4_local=({self.lx:.2f},{self.ly:.2f}) '
                f'offset={off:.2f}m yaw={math.degrees(self.t_yaw):+.1f}deg '
                f'| prog {self.prog}/{len(self.path)}')


def main():
    rclpy.init()
    node = PathFollowerPosition()
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
