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
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from mavros_msgs.srv import SetMode
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
        self.declare_parameter('lookahead', 0.4)
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
        # Reactive LiDAR brake -- the wall safety net. It reads raw /scan in the
        # body frame, so it works even when SLAM/localisation is wrong (exactly
        # the case that steers a "valid" planned path into a wall). If the
        # closest return inside a cone around the travel direction is nearer than
        # brake_distance, the drone holds instead of pushing forward. A separate,
        # larger release distance gives hysteresis: without it the brake chattered
        # on/off every ~50 ms near a wall, jerking the setpoint and destabilising
        # the aircraft (this made a real flight diverge).
        self.declare_parameter('brake_distance', 0.45)
        self.declare_parameter('brake_release_distance', 0.85)
        self.declare_parameter('brake_cone_deg', 50.0)
        # WALL REPULSION -- stops the drone's ARMS clipping a wall. The x500 is
        # ~0.6m across (arms/props reach ~0.3m from centre), but the brake above
        # treats the drone as a point and only looks forward, so a wall beside
        # the aircraft -- during a turn or a cut corner -- was never seen and an
        # arm hit it. This adds a 360-degree potential field: every /scan return
        # closer than influence_radius pushes the setpoint away from it. In a
        # corridor the two walls cancel, so the drone self-centres; at a corner
        # the inside wall pushes it wide so the arm clears. robot_radius is the
        # prop-tip radius the field must protect.
        self.declare_parameter('robot_radius', 0.30)
        self.declare_parameter('repulsion_influence', 0.7)
        self.declare_parameter('repulsion_gain', 0.5)
        self.declare_parameter('repulsion_max', 0.25)
        # Localisation-divergence failsafe: if the estimated map<->PX4-local
        # offset exceeds this (after it has had time to settle), SLAM/EKF have
        # diverged -- LAND. Catches the slow-drift runaway the speed detector in
        # vision_pose_bridge can miss.
        self.declare_parameter('max_offset_m', 3.0)
        # COVERAGE-AWARE YAW: point the camera at the nearest UNSEARCHED cell
        # (from /coverage_grid) instead of always facing travel direction. An
        # omni multirotor can strafe along the path while the camera looks
        # sideways, so this searches the near-wall / side cells that flying
        # head-down a corridor would skip -- the reason coverage stalled at 79%.
        self.declare_parameter('coverage_yaw', True)
        self.declare_parameter('coverage_look_radius', 3.5)
        # SMOOTHING: low-pass the published setpoint and yaw so a jumping carrot
        # (2 s replans, per-tick repulsion, yaw target hops) becomes smooth
        # motion instead of the twitchy flight seen in the sim. Lower alpha =
        # smoother but laggier. yaw_lpf is separate so heading can ease over
        # without the position lagging.
        self.declare_parameter('setpoint_lpf', 0.35)
        self.declare_parameter('yaw_lpf', 0.20)
        # LOCAL-MINIMUM ESCAPE. Reactive repulsion can trap the drone at a spot
        # where wall pushes cancel the goal pull (it "gets stuck at one point").
        # Keeping repulsion_max < lookahead already stops most of this, but as a
        # safety net: if it makes no real progress for escape_stuck_sec while a
        # path exists and nothing is dead ahead, suppress repulsion briefly so
        # the goal pull frees it.
        self.declare_parameter('escape_stuck_sec', 5.0)
        self.declare_parameter('escape_duration_sec', 3.0)
        self.declare_parameter('escape_min_move', 0.25)
        # Safe recovery: if SLAM stays lost this long, stop hovering blind and
        # LAND (PX4 AUTO.LAND descends on baro, needs no position estimate). This
        # is what makes the drone "come down safely" instead of flying off when
        # localisation is gone.
        self.declare_parameter('land_after_slam_lost_sec', 6.0)

        self.target_alt = self.get_parameter('target_altitude').value
        self.lookahead = self.get_parameter('lookahead').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.map_frame = self.get_parameter('map_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        self.yaw_min_dist = self.get_parameter('yaw_min_distance').value
        self.pose_timeout = self.get_parameter('pose_timeout_sec').value
        self.brake_dist = self.get_parameter('brake_distance').value
        self.brake_release = self.get_parameter('brake_release_distance').value
        self.brake_cone = math.radians(self.get_parameter('brake_cone_deg').value)
        self.robot_radius = self.get_parameter('robot_radius').value
        self.repulsion_influence = self.get_parameter('repulsion_influence').value
        self.repulsion_gain = self.get_parameter('repulsion_gain').value
        self.repulsion_max = self.get_parameter('repulsion_max').value
        self.max_offset = self.get_parameter('max_offset_m').value
        self.coverage_yaw = self.get_parameter('coverage_yaw').value
        self.look_radius = self.get_parameter('coverage_look_radius').value
        self.sp_lpf = self.get_parameter('setpoint_lpf').value
        self.yaw_lpf = self.get_parameter('yaw_lpf').value
        self.escape_stuck_sec = self.get_parameter('escape_stuck_sec').value
        self.escape_duration = self.get_parameter('escape_duration_sec').value
        self.escape_min_move = self.get_parameter('escape_min_move').value
        self.land_after = self.get_parameter('land_after_slam_lost_sec').value
        rate_hz = self.get_parameter('rate_hz').value
        # low-pass state for the repulsion vector (avoid setpoint jitter)
        self.rep_x = self.rep_y = 0.0
        # smoothed command state (map frame); None until first active tick so we
        # can snap-initialise it and avoid a startup lurch
        self.cmd_x = self.cmd_y = self.cmd_yaw = None
        self.coverage_grid = None
        # local-minimum escape state
        self._progress_pos = None
        self._progress_time = None
        self._escaping_until = None

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

        # SLAM health (from vision_pose_bridge). Assume OK until told otherwise
        # so a late-joining latched publisher doesn't force a spurious hover.
        self.slam_ok = True
        self.slam_lost_since = None     # wall-clock when SLAM first went bad
        self.landing = False            # latched once we commit to LAND
        # Reactive brake state: nearest obstacle in a body-frame cone, and the
        # cone the /scan spans, filled from the first scan.
        self.scan = None
        self.brake_active = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Path, '/planned_path', self._on_path, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._on_local, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)
        self.create_subscription(Bool, '/slam_ok', self._on_slam_ok, latched)
        self.create_subscription(
            OccupancyGrid, '/coverage_grid', self._on_coverage, latched)
        # Mission complete (explorer has returned to the entry): land & disarm at
        # the entry point instead of hovering there forever. Latched so we catch
        # it even if it fires just before we subscribe.
        self.create_subscription(
            Bool, '/mission/complete', self._on_mission_complete, latched)
        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.reached_pub = self.create_publisher(Bool, '/path_follower/reached', 10)
        # Published purely for RViz: shows exactly what the drone is chasing.
        self.carrot_pub = self.create_publisher(PoseStamped, '/follower/carrot', 5)
        # For the safe-recovery LAND when SLAM is lost for good.
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

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

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _on_coverage(self, msg: OccupancyGrid):
        self.coverage_grid = msg

    def _look_yaw(self, travel_yaw):
        """Coverage-aware heading: if an UNSEARCHED cell is within look_radius,
        face it so the camera searches it; otherwise face travel direction.
        Decoupling camera aim from motion is fine here -- the brake checks the
        travel cone (not the yaw) and repulsion is 360-degree, so aiming the
        camera sideways doesn't reduce obstacle protection."""
        if not self.coverage_yaw or self.coverage_grid is None:
            return travel_yaw
        cov = self.coverage_grid
        info = cov.info
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
        best = None
        best_d = self.look_radius
        for i, v in enumerate(cov.data):
            if v != 0:                  # 0 == free & UNSEARCHED
                continue
            col, row = i % info.width, i // info.width
            wx = ox + (col + 0.5) * res
            wy = oy + (row + 0.5) * res
            d = math.hypot(wx - self.mx, wy - self.my)
            if d < best_d:
                best_d, best = d, (wx, wy)
        if best is None:
            return travel_yaw
        return math.atan2(best[1] - self.my, best[0] - self.mx)

    def _on_slam_ok(self, msg: Bool):
        if msg.data != self.slam_ok:
            self.slam_ok = msg.data
            if not msg.data:
                self.slam_lost_since = self.get_clock().now()
                self.get_logger().error(
                    f'SLAM lost -> HOVERING; will LAND if not recovered in '
                    f'{self.land_after:.0f}s')
            else:
                self.slam_lost_since = None
                self.get_logger().info('SLAM recovered -> resuming path following')

    def _nearest_ahead(self, body_bearing):
        """Closest /scan return within a cone around body_bearing (the travel
        direction in the BODY frame, 0 = straight ahead). Works on raw ranges,
        so it is immune to any map/localisation error -- the point of the brake.
        Returns +inf if nothing is in range."""
        scan = self.scan
        if scan is None:
            return float('inf')
        n = len(scan.ranges)
        if n == 0:
            return float('inf')
        half = self.brake_cone / 2.0
        amin, ainc = scan.angle_min, scan.angle_increment
        nearest = float('inf')
        for i in range(n):
            r = scan.ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
            if not (scan.range_min < r < scan.range_max):
                continue
            ang = amin + i * ainc
            if abs(wrap(ang - body_bearing)) <= half and r < nearest:
                nearest = r
        return nearest

    def _brake_check(self, body_bearing):
        """Hysteretic brake: engage below brake_distance, release only once past
        brake_release_distance. The gap stops the on/off chatter that jerked the
        setpoint and destabilised the aircraft."""
        nearest = self._nearest_ahead(body_bearing)
        if self.brake_active:
            if nearest > self.brake_release:
                self.brake_active = False
                self.get_logger().info(
                    f'Path ahead clear ({nearest:.2f}m) -> resuming')
        else:
            if nearest < self.brake_dist:
                self.brake_active = True
                self.get_logger().warn(
                    f'Obstacle {nearest:.2f}m ahead -> braking (hold)')
        return self.brake_active

    def _repulsion_map(self):
        """360-degree wall-repulsion nudge, returned in the MAP frame.

        Every /scan return closer than influence pushes the setpoint directly
        away from it, weighted by how close it is (measured from the prop tips,
        i.e. r - robot_radius). Summed over all returns:
          * in a straight corridor the two side walls cancel -> the drone
            self-centres, keeping the arms away from both walls;
          * at a corner or when off-centre, the near wall wins -> the setpoint
            is pushed wide so an arm can't clip it while turning.
        Low-pass filtered so the setpoint doesn't jitter. Immune to
        localisation error because it works on raw body-frame ranges."""
        scan = self.scan
        rx = ry = 0.0
        if scan is not None and len(scan.ranges):
            amin, ainc = scan.angle_min, scan.angle_increment
            infl = self.repulsion_influence
            for i, r in enumerate(scan.ranges):
                if math.isinf(r) or math.isnan(r):
                    continue
                if not (scan.range_min < r < scan.range_max):
                    continue
                clearance = r - self.robot_radius      # distance from prop tips
                if clearance >= infl:
                    continue
                ang = amin + i * ainc
                w = (infl - clearance) / infl          # 0..1, grows as it nears
                w = max(0.0, w) ** 2                    # emphasise very close walls
                rx -= math.cos(ang) * w                 # push AWAY from the return
                ry -= math.sin(ang) * w
        # scale + cap the raw (body-frame) push
        mag = math.hypot(rx, ry)
        if mag > 1e-3:
            capped = min(mag * self.repulsion_gain, self.repulsion_max)
            rx, ry = rx / mag * capped, ry / mag * capped
        else:
            rx = ry = 0.0
        # low-pass to avoid jitter
        a = 0.4
        self.rep_x += a * (rx - self.rep_x)
        self.rep_y += a * (ry - self.rep_y)
        # body -> map frame (scan angles are body-frame; carrot is map-frame)
        c, s = math.cos(self.myaw), math.sin(self.myaw)
        return (c * self.rep_x - s * self.rep_y,
                s * self.rep_x + c * self.rep_y)

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

    def _on_mission_complete(self, msg):
        """Explorer signalled the mission is done (back at the entry). Land and
        disarm here rather than hovering. Only act once we're actually airborne
        and not already landing."""
        if msg.data and self.airborne and not self.landing:
            self._commit_land(reason='Mission complete (returned to entry)')

    def _commit_land(self, reason=None):
        """Ask PX4 to land where it is. AUTO.LAND descends on baro and needs no
        position estimate, so it works even with a diverged EKF. Latched so we
        only fire once. Used both for the SLAM-loss failsafe and for the normal
        end-of-mission landing at the entry point."""
        self.landing = True
        if reason is None:
            reason = f'SLAM not recovered in {self.land_after:.0f}s'
        self.get_logger().warn(
            f'{reason} -> commanding AUTO.LAND (descend & disarm at current spot)')
        if self.mode_cli.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'AUTO.LAND'
            self.mode_cli.call_async(req)
        else:
            self.get_logger().warn('set_mode service not ready; retrying LAND')
            self.landing = False    # allow another attempt next tick

    def _publish_setpoint(self, x, y, yaw, frame):
        """Publish a local-frame position setpoint. frame is just for the log."""
        qx, qy, qz, qw = yaw_to_quat(yaw)
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = self.target_alt
        sp.pose.orientation.x = qx
        sp.pose.orientation.y = qy
        sp.pose.orientation.z = qz
        sp.pose.orientation.w = qw
        self.sp_pub.publish(sp)

    def _tick(self):
        self._read_map_pose()
        if not self.local_ok:
            return

        # SLAM-LOSS FAILSAFE. When Cartographer loses tracking, both the map
        # pose AND the map->local transform become unreliable (this is what
        # diverged the EKF and crashed the drone). So hover using PX4's OWN local
        # position directly -- no map, no transform. PX4 coasts on its IMU and
        # holds station until SLAM recovers. If it does NOT recover within
        # land_after seconds, stop hovering blind and LAND -- coming down safely
        # beats drifting off lost. /slam_ok comes from vision_pose_bridge, which
        # catches the frozen AND diverging cases the follower cannot.
        if (not self.slam_ok) or (not self.map_ok):
            if self.slam_lost_since is not None:
                lost_for = (self.get_clock().now()
                            - self.slam_lost_since).nanoseconds / 1e9
                if lost_for > self.land_after and not self.landing:
                    self._commit_land()
            if not self.landing:
                self.cmd_x = self.cmd_y = self.cmd_yaw = None   # reset smoother
                self._publish_setpoint(self.lx, self.ly, self.lyaw, 'local-hover')
            return
        if self.landing:
            return                  # already committed to LAND; PX4 owns it now

        self._update_transform()

        # DIVERGENCE FAILSAFE. A large, settled map<->PX4-local offset means SLAM
        # or the EKF has run away (the slow-drift case the speed detector misses).
        # Once airborne, treat that as lost -> LAND. self.t_init guards against
        # tripping before the transform has had a chance to settle.
        if self.airborne and self.t_init and not self.landing:
            if math.hypot(self.t_x, self.t_y) > self.max_offset:
                self.get_logger().error(
                    f'map<->local offset {math.hypot(self.t_x, self.t_y):.1f}m '
                    '-> localisation diverged, LANDING')
                self._commit_land()
                return

        tx, ty, tyaw = self._carrot()

        # REACTIVE LiDAR BRAKE (hysteretic). If flying toward a too-close return,
        # hold instead of pushing into it. Uses raw /scan in the body frame, so
        # it protects even when the map/localisation is wrong.
        moving = self.airborne and self.path and \
            math.hypot(tx - self.mx, ty - self.my) > 0.05
        if moving:
            body_bearing = wrap(math.atan2(ty - self.my, tx - self.mx) - self.myaw)
            if self._brake_check(body_bearing):
                tx, ty, tyaw = self._hold()

        # LOCAL-MINIMUM ESCAPE. Track real progress; if the drone has a path but
        # hasn't moved for escape_stuck_sec and nothing is dead ahead, it's
        # trapped in a repulsion local minimum -> enter a short escape window
        # during which repulsion is suppressed so the goal pull frees it.
        now = self.get_clock().now()
        if self.airborne and self.path and not self.brake_active:
            if self._progress_pos is None:
                self._progress_pos = (self.mx, self.my)
                self._progress_time = now
            elif math.hypot(self.mx - self._progress_pos[0],
                            self.my - self._progress_pos[1]) > self.escape_min_move:
                self._progress_pos = (self.mx, self.my)
                self._progress_time = now
            elif (now - self._progress_time).nanoseconds / 1e9 > self.escape_stuck_sec:
                if self._escaping_until is None:
                    self.get_logger().warn(
                        'Stuck (repulsion local minimum) -> suppressing repulsion to escape')
                self._escaping_until = (now.nanoseconds
                                        + self.escape_duration * 1e9)
                self._progress_pos = (self.mx, self.my)
                self._progress_time = now
        else:
            self._progress_pos = None

        escaping = (self._escaping_until is not None
                    and now.nanoseconds < self._escaping_until)
        if self._escaping_until is not None and now.nanoseconds >= self._escaping_until:
            self._escaping_until = None

        # WALL REPULSION. Always nudge the setpoint away from nearby walls (even
        # while holding/braking, so a hold near a wall still eases off it). This
        # is what keeps the arms from clipping a wall on a turn. Suppressed
        # briefly during a local-minimum escape.
        if self.airborne and not escaping:
            rmx, rmy = self._repulsion_map()
            tx += rmx
            ty += rmy

        # COVERAGE-AWARE YAW: aim the camera at the nearest unsearched cell.
        tyaw = self._look_yaw(tyaw)

        # SMOOTHING: low-pass the command so a jumping carrot / repulsion / yaw
        # target becomes smooth motion. Snap-initialise on the first tick (and
        # after any hover) so there's no lurch.
        if self.cmd_x is None:
            self.cmd_x, self.cmd_y, self.cmd_yaw = tx, ty, tyaw
        else:
            self.cmd_x += self.sp_lpf * (tx - self.cmd_x)
            self.cmd_y += self.sp_lpf * (ty - self.cmd_y)
            self.cmd_yaw = wrap(self.cmd_yaw
                                + self.yaw_lpf * wrap(tyaw - self.cmd_yaw))

        # Reasoned in MAP; convert to PX4's local frame for the setpoint topic.
        sx, sy, syaw = self._map_to_local(self.cmd_x, self.cmd_y, self.cmd_yaw)
        self._publish_setpoint(sx, sy, syaw, 'map->local')

        carrot = PoseStamped()
        carrot.header.stamp = self.get_clock().now().to_msg()
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
                f'| prog {self.prog}/{len(self.path)} brake={self.brake_active}')


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
