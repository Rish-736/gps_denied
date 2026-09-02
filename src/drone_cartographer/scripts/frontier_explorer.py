#!/usr/bin/env python3
"""
Frontier exploration: decide where to fly next, with no operator input.

A "frontier" is a known-free cell touching unknown space -- the edge of what
has been mapped. Flying to one reveals new area. Repeat until none are left and
the maze is fully mapped, then fly back to the entry point (entry == exit per
the mission rules, which also forbid any operator steering).

Flow:  /map -> find frontier cells -> cluster -> score -> pick one
       -> ask Nav2's planner for an obstacle-free path (ComputePathToPose)
       -> publish it on /planned_path for path_follower_position to fly
       -> repeat.

Only Nav2's PLANNER is used, not its controller: MPPI stalled the drone, so
control lives in path_follower_position.

WHAT WAS WRONG BEFORE
=====================
1. FRAME MIXING. This node read the drone's position from
   /mavros/local_position/pose -- PX4's EKF local frame -- and compared it
   against frontier coordinates computed from Cartographer's map-frame grid.
   Those are two different frames that can be offset and rotated relative to
   each other, so every distance, every bounds check and every goal was
   computed against the wrong origin. It now takes its pose from the
   map->base_link TF, i.e. the same frame the grid and the path live in.

2. THRASHING. Goals were picked purely by "nearest". As the drone moves, the
   nearest frontier flips between candidates on opposite sides of it, so the
   drone starts toward one, the map updates, and the next tick sends it back --
   the "moves a second, returns to the same spot" loop. Selection now scores
   distance against information gain and applies hysteresis toward the goal
   already being pursued, so a committed goal is seen through.

3. NO FAILURE MEMORY. A frontier the planner could never route to was re-picked
   forever. Goals that repeatedly fail to plan or are never reached are now
   blacklisted.

4. FROZEN REPLANNING. Once a goal was active the node went silent for up to
   40s. It now refreshes the path to the current goal every cycle, so the
   follower always has a route consistent with the newest map.
"""
import math
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException

UNKNOWN = -1


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('min_frontier_cells', 8)
        self.declare_parameter('min_goal_distance', 0.7)
        # Cells with 0 <= value < free_threshold count as free. Cartographer
        # stores free space as a RANGE of low probabilities (0..49), not exactly
        # 0 -- requiring ==0 found almost no frontiers and made the explorer
        # wrongly declare the map complete (verified live: ==0 gave 0 clusters,
        # <50 gave 9).
        self.declare_parameter('free_threshold', 50)
        self.declare_parameter('replan_period_sec', 2.0)
        # Scoring: cost = distance - info_weight * frontier_size_in_metres.
        # Higher info_weight favours big unexplored openings over close scraps.
        self.declare_parameter('info_weight', 1.5)
        # Bonus subtracted from the cost of the goal already being pursued, so
        # the drone commits instead of flip-flopping between similar options.
        self.declare_parameter('hysteresis_bonus', 2.0)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('blacklist_after_failures', 3)

        # ARENA BOUNDS -- the drone spawns at (0,0) in the entry cell ON the
        # maze's south boundary, so its LiDAR looks straight out through the
        # 2.5m entry gap into the infinite open ground plane. Cartographer maps
        # that outside area as free, and its edge is a perfectly valid frontier.
        # Without bounds the explorer sends the drone OUT of the maze.
        # Measured from the world SDFs (wall centre lines):
        #   nidar_maze      x[-7.50, 6.50] y[-0.50, 13.50]
        #   nidar_maze_wide x[-8.75, 6.25] y[-1.25, 13.75]
        # Defaults are the wide maze, inset slightly so goals stay off the walls.
        self.declare_parameter('arena_min_x', -8.5)
        self.declare_parameter('arena_max_x', 6.0)
        self.declare_parameter('arena_min_y', -1.0)
        self.declare_parameter('arena_max_y', 13.5)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'base_link')

        g = lambda n: self.get_parameter(n).value
        self.min_cells = g('min_frontier_cells')
        self.min_dist = g('min_goal_distance')
        self.free_threshold = g('free_threshold')
        self.info_weight = g('info_weight')
        self.hysteresis = g('hysteresis_bonus')
        self.goal_timeout = g('goal_timeout_sec')
        self.max_failures = g('blacklist_after_failures')
        self.min_x, self.max_x = g('arena_min_x'), g('arena_max_x')
        self.min_y, self.max_y = g('arena_min_y'), g('arena_max_y')
        self.map_frame, self.body_frame = g('map_frame'), g('body_frame')

        self.map_msg = None
        # Pose in the MAP frame -- same frame as the grid and the path.
        self.px = self.py = 0.0
        self.pose_ok = False

        self.goal = None                # (x, y) currently being pursued
        self.goal_start = None
        self.plan_pending = False
        self.blacklist = {}             # rounded (x, y) -> consecutive failures
        self.entry_xy = None
        self.returning = False
        self.mission_done = False
        self._waiting = 0

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(
            Bool, '/path_follower/reached', self._on_reached, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/frontiers', 5)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.create_timer(g('replan_period_sec'), self._tick)
        self.get_logger().info(
            f'frontier_explorer: arena x[{self.min_x}, {self.max_x}] '
            f'y[{self.min_y}, {self.max_y}], waiting for /map and TF...')

    # ---------------------------------------------------------------- inputs
    def _on_map(self, msg):
        self.map_msg = msg

    def _read_pose(self):
        """Drone position in the MAP frame. Time() = latest available, which
        sidesteps the sim-time (SLAM) vs wall-clock (MAVROS) split."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, Time())
        except TransformException:
            return
        self.px = tf.transform.translation.x
        self.py = tf.transform.translation.y
        self.pose_ok = True
        if self.entry_xy is None:
            self.entry_xy = (self.px, self.py)
            self.get_logger().info(
                f'Entry/exit point recorded at ({self.px:.2f}, {self.py:.2f})')

    def _on_reached(self, msg):
        if not (msg.data and self.goal is not None):
            return
        if self.returning:
            self.mission_done = True
            self.get_logger().info(
                'Returned to entry/exit point -- MISSION COMPLETE '
                '(maze explored and exited).')
        else:
            self.get_logger().info('Frontier reached -> selecting next')
            self.blacklist.pop(self._key(self.goal), None)
        self.goal = None

    # ----------------------------------------------------------- grid helpers
    @staticmethod
    def _key(xy):
        return (round(xy[0], 1), round(xy[1], 1))

    def _to_world(self, mx, my):
        info = self.map_msg.info
        return (info.origin.position.x + (mx + 0.5) * info.resolution,
                info.origin.position.y + (my + 0.5) * info.resolution)

    def _find_frontiers(self):
        """Frontier = known-free cell with at least one unknown 4-neighbour."""
        info = self.map_msg.info
        data = self.map_msg.data
        w, h = info.width, info.height
        free_hi = self.free_threshold
        cells = []
        for my in range(1, h - 1):
            row = my * w
            for mx in range(1, w - 1):
                v = data[row + mx]
                if not (0 <= v < free_hi):
                    continue
                if (data[row + mx + 1] == UNKNOWN or data[row + mx - 1] == UNKNOWN
                        or data[row + w + mx] == UNKNOWN
                        or data[row - w + mx] == UNKNOWN):
                    cells.append((mx, my))
        return set(cells)

    def _cluster(self, cells):
        """Group adjacent frontier cells (8-connected BFS)."""
        clusters, unseen = [], set(cells)
        while unseen:
            seed = unseen.pop()
            group, queue = [seed], deque([seed])
            while queue:
                cx, cy = queue.popleft()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (cx + dx, cy + dy)
                        if n in unseen:
                            unseen.discard(n)
                            group.append(n)
                            queue.append(n)
            if len(group) >= self.min_cells:
                clusters.append(group)
        return clusters

    def _candidates(self, clusters):
        """Turn clusters into scored, in-bounds, reachable-looking goals."""
        res = self.map_msg.info.resolution
        out = []
        for group in clusters:
            cx = sum(c[0] for c in group) / len(group)
            cy = sum(c[1] for c in group) / len(group)
            # Snap to a REAL cell of the cluster nearest the centroid. The raw
            # geometric centroid of an L-shaped cluster can land inside a wall,
            # where the planner can never reach it.
            gmx, gmy = min(group, key=lambda c: (c[0] - cx) ** 2 + (c[1] - cy) ** 2)
            wx, wy = self._to_world(gmx, gmy)
            if not (self.min_x <= wx <= self.max_x
                    and self.min_y <= wy <= self.max_y):
                continue        # outside the maze (open field beyond the entry)
            if self.blacklist.get(self._key((wx, wy)), 0) >= self.max_failures:
                continue        # repeatedly unreachable
            dist = math.hypot(wx - self.px, wy - self.py)
            if dist < self.min_dist:
                continue        # already here; nothing new revealed
            # Prefer close AND large. Size is converted to metres so the two
            # terms are comparable regardless of grid resolution.
            cost = dist - self.info_weight * (len(group) * res)
            if self.goal is not None and math.hypot(
                    wx - self.goal[0], wy - self.goal[1]) < 1.0:
                cost -= self.hysteresis     # stick with what we're already chasing
            out.append((cost, wx, wy, len(group), dist))
        out.sort(key=lambda c: c[0])
        return out

    def _publish_markers(self, clusters):
        arr = MarkerArray()
        for i, group in enumerate(clusters):
            cx = sum(c[0] for c in group) / len(group)
            cy = sum(c[1] for c in group) / len(group)
            wx, wy = self._to_world(cx, cy)
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'frontiers', i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = wx, wy, 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.9, 0.8
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    # ------------------------------------------------------------- main loop
    def _tick(self):
        self._read_pose()

        # Fail loudly, not silently: if /map or TF never arrives the explorer
        # would otherwise sit quiet forever and just look broken.
        if self.map_msg is None or not self.pose_ok:
            self._waiting += 1
            if self._waiting % 5 == 0:
                missing = []
                if self.map_msg is None:
                    missing.append('/map')
                if not self.pose_ok:
                    missing.append(f'TF {self.map_frame}->{self.body_frame}')
                self.get_logger().warn(
                    f'Still waiting for: {", ".join(missing)} - not exploring yet')
            return
        if self.mission_done or self.plan_pending:
            return

        # Give up on a goal we've chased too long, and remember the failure.
        if self.goal is not None and self.goal_start is not None:
            elapsed = (self.get_clock().now() - self.goal_start).nanoseconds / 1e9
            if elapsed > self.goal_timeout:
                k = self._key(self.goal)
                self.blacklist[k] = self.blacklist.get(k, 0) + 1
                self.get_logger().warn(
                    f'Goal {self.goal[0]:.2f},{self.goal[1]:.2f} not reached in '
                    f'{self.goal_timeout:.0f}s -> abandoning '
                    f'(failures={self.blacklist[k]})')
                self.goal = None

        clusters = self._cluster(self._find_frontiers())
        self._publish_markers(clusters)
        cands = self._candidates(clusters)

        if not cands:
            # Exploration finished -> fly back to the entry/exit point so the
            # full in-and-out loop closes.
            if self.entry_xy is None:
                return
            if not self.returning:
                self.returning = True
                self.get_logger().info(
                    'No frontiers left -> returning to entry/exit point')
            self.goal = self.entry_xy
            self._request_path(self.entry_xy[0], self.entry_xy[1], 0, 0.0, 0)
            return

        # Drop a goal that the map has since revealed (no frontier near it any
        # more) so we don't fly to an already-explored spot.
        if self.goal is not None and not self.returning:
            if not any(math.hypot(c[1] - self.goal[0], c[2] - self.goal[1]) < 1.0
                       for c in cands):
                self.get_logger().info('Current goal already explored -> reselecting')
                self.goal = None

        best = cands[0]
        _, wx, wy, size, dist = best
        if self.goal is None or math.hypot(wx - self.goal[0], wy - self.goal[1]) > 1.0:
            self.goal = (wx, wy)
            self.goal_start = self.get_clock().now()
        # Refresh the route to the committed goal every cycle so the follower
        # always has a path consistent with the newest map.
        self._request_path(self.goal[0], self.goal[1], size, dist, len(cands))

    def _request_path(self, wx, wy, size, dist, ncand):
        if not self.planner.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('planner compute_path_to_pose not available yet')
            return
        req = ComputePathToPose.Goal()
        req.goal.header.frame_id = self.map_frame
        req.goal.header.stamp = self.get_clock().now().to_msg()
        req.goal.pose.position.x = float(wx)
        req.goal.pose.position.y = float(wy)
        req.goal.pose.orientation.w = 1.0
        req.planner_id = 'GridBased'
        req.use_start = False       # plan from the drone's current pose

        self._last_goal = (wx, wy, size, dist, ncand)
        self.plan_pending = True
        self.planner.send_goal_async(req).add_done_callback(self._on_plan_response)

    def _on_plan_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('planner rejected the request')
            self.plan_pending = False
            return
        handle.get_result_async().add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future):
        self.plan_pending = False
        path = future.result().result.path
        wx, wy, size, dist, ncand = self._last_goal
        if not path.poses:
            k = self._key((wx, wy))
            self.blacklist[k] = self.blacklist.get(k, 0) + 1
            what = 'entry point' if self.returning else f'frontier ({wx:.2f}, {wy:.2f})'
            self.get_logger().warn(
                f'No path to {what} (failures={self.blacklist[k]}) -> reselecting')
            self.goal = None
            return
        if self.returning:
            self.get_logger().info(
                f'-> RETURNING to entry ({wx:.2f}, {wy:.2f}), '
                f'{len(path.poses)}-pose path')
        else:
            self.get_logger().info(
                f'-> frontier ({wx:.2f}, {wy:.2f}), {size} cells, {dist:.1f}m, '
                f'{len(path.poses)}-pose path [{ncand} candidates]')
        self.path_pub.publish(path)     # position follower flies this


def main():
    rclpy.init()
    node = FrontierExplorer()
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
