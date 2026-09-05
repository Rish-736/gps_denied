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
        # LOWERED to 0.4 (was 1.5): with a bigger info weight the explorer chased
        # large frontiers on the far side of the maze, forcing long fast flights
        # that broke SLAM. Now distance dominates, so it sweeps the maze
        # methodically corridor-by-corridor -- short hops keep flight slow and
        # SLAM locked.
        self.declare_parameter('info_weight', 0.4)
        # Local-first sweep: if any acceptable frontier is within this radius,
        # only those are considered -- the drone finishes its neighbourhood
        # before ever committing to a far one. Far frontiers are used only when
        # nothing nearer remains.
        self.declare_parameter('sweep_radius', 5.0)
        # Bonus subtracted from the cost of the goal already being pursued, so
        # the drone commits instead of flip-flopping between similar options.
        self.declare_parameter('hysteresis_bonus', 2.0)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('blacklist_after_failures', 3)
        # VISITED MEMORY -- so it explores the WHOLE maze and doesn't loop in one
        # area. The drone's own trail is recorded on a coarse grid; a frontier
        # near cells it has already spent time in is penalised, pushing it toward
        # genuinely new ground. (Frontier exploration is coverage-complete on its
        # own -- an explored area stops being a frontier -- but this stops the
        # drone oscillating between two nearby frontiers and re-flying the same
        # corridor, which is what we saw.)
        self.declare_parameter('visit_cell_size', 0.75)
        self.declare_parameter('visit_weight', 0.6)
        self.declare_parameter('visit_radius', 1.5)
        # STUCK DETECTION -- if the drone hasn't made real headway toward a goal
        # for this long, abandon it and pick another (covers "planner returns a
        # path but the drone can't actually get there", e.g. a doorway too tight).
        self.declare_parameter('stuck_window_sec', 12.0)
        self.declare_parameter('stuck_min_move_m', 0.4)
        # How many consecutive empty-frontier cycles before declaring the maze
        # done and returning home. A single empty cycle is usually a transient
        # (the map momentarily has no cluster >= min_cells); returning on that
        # sent the drone home far too early last time.
        self.declare_parameter('empty_cycles_to_finish', 4)
        # Search-done criteria (either one, once the map has no frontiers left):
        #  * coverage >= coverage_complete_pct (a clear high-water mark), OR
        #  * coverage has PLATEAUED (not improved by >0.5% in coverage_stall_sec)
        #    -- the robust one, because the achievable ceiling varies run to run
        #    (a few 1 m cells straddle walls and can never be LOS-confirmed, and
        #    no survivor fits there). Without this the drone circles the last
        #    cells forever and never returns -- failing the rulebook's "exit via
        #    the entry" rule and draining the battery. Remaining cells are
        #    reported in the log, not hidden.
        self.declare_parameter('coverage_complete_pct', 97.0)
        self.declare_parameter('coverage_stall_sec', 40.0)

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
        # A frontier whose cell sits within this margin of the arena boundary is
        # a PHANTOM: free cells hugging a boundary wall, touching the walled-off
        # unknown OUTSIDE the maze (most often the open ground seen through the
        # entry gap). It can never be cleared -- the wall blocks the LiDAR -- so
        # if it stays a candidate the endgame flip-flops and the drone oscillates
        # against the wall until it clips it. Excluding it lets "no real frontiers
        # left" actually become true so the mission can finish and return.
        self.declare_parameter('boundary_margin', 0.5)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'base_link')

        g = lambda n: self.get_parameter(n).value
        self.min_cells = g('min_frontier_cells')
        self.min_dist = g('min_goal_distance')
        self.free_threshold = g('free_threshold')
        self.info_weight = g('info_weight')
        self.sweep_radius = g('sweep_radius')
        self.hysteresis = g('hysteresis_bonus')
        self.goal_timeout = g('goal_timeout_sec')
        self.max_failures = g('blacklist_after_failures')
        self.min_x, self.max_x = g('arena_min_x'), g('arena_max_x')
        self.min_y, self.max_y = g('arena_min_y'), g('arena_max_y')
        self.boundary_margin = g('boundary_margin')
        self.map_frame, self.body_frame = g('map_frame'), g('body_frame')
        self.visit_cell = g('visit_cell_size')
        self.visit_weight = g('visit_weight')
        self.visit_radius = g('visit_radius')
        self.stuck_window = g('stuck_window_sec')
        self.stuck_min_move = g('stuck_min_move_m')
        self.empty_cycles_to_finish = g('empty_cycles_to_finish')
        self.coverage_complete_pct = g('coverage_complete_pct')
        self.coverage_stall_sec = g('coverage_stall_sec')
        self._best_frac = -1.0          # highest coverage seen
        self._best_frac_time = None     # when it last improved

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
        self._return_committed = False   # once True, always go home (no flip-flop)
        self.mission_done = False
        self._waiting = 0
        self.visited = {}               # coarse cell -> visit count (the "memory")
        self.empty_count = 0            # consecutive cycles with no frontier
        self.goal_anchor = None         # (x, y) where the current goal was set
        self.goal_anchor_time = None    # when we last made real headway

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        # Coverage grid from coverage_tracker: which arena cells still need the
        # camera pointed at them. These become goals just like frontiers, so the
        # drone keeps flying until every reachable cell is SEARCHED (not merely
        # mapped) -- the guarantee that no survivor's corner is skipped.
        self.coverage_msg = None
        self.create_subscription(
            OccupancyGrid, '/coverage_grid', self._on_coverage, map_qos)
        self.create_subscription(
            Bool, '/path_follower/reached', self._on_reached, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/frontiers', 5)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        # Latched: tells the follower the mission is over (back at entry) so it
        # LANDS instead of hovering. Latched so the follower gets it even if it
        # is briefly not subscribed at the moment we finish.
        complete_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.complete_pub = self.create_publisher(
            Bool, '/mission/complete', complete_qos)
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.create_timer(g('replan_period_sec'), self._tick)
        self.get_logger().info(
            f'frontier_explorer: arena x[{self.min_x}, {self.max_x}] '
            f'y[{self.min_y}, {self.max_y}], waiting for /map and TF...')

    # ---------------------------------------------------------------- inputs
    def _on_map(self, msg):
        self.map_msg = msg

    def _on_coverage(self, msg):
        self.coverage_msg = msg

    def _coverage_frac(self):
        """Coverage as a percentage float, or None if no coverage grid yet."""
        cov = self.coverage_msg
        if cov is None:
            return None
        searched = sum(1 for v in cov.data if v == 100)
        free = sum(1 for v in cov.data if v in (0, 100))
        return (100.0 * searched / free) if free else 0.0

    def _coverage_pct(self):
        """Human-readable coverage figure for the completion log."""
        cov = self.coverage_msg
        if cov is None:
            return 'coverage n/a'
        searched = sum(1 for v in cov.data if v == 100)
        free = sum(1 for v in cov.data if v in (0, 100))
        unsearched = free - searched
        pct = (100 * searched / free) if free else 0
        return (f'{searched}/{free} cells searched = {pct:.0f}% '
                f'({unsearched} cell(s) unreachable/unmarkable)')

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
        # Record the trail: this coarse cell has now been visited. This is the
        # explorer's memory of where it has already been.
        self.visited[self._visit_key(self.px, self.py)] = \
            self.visited.get(self._visit_key(self.px, self.py), 0) + 1

    def _visit_key(self, x, y):
        return (round(x / self.visit_cell), round(y / self.visit_cell))

    def _visit_penalty(self, wx, wy):
        """How much time the drone has already spent near (wx, wy). Frontiers in
        well-trodden areas score worse, so exploration keeps pushing outward."""
        r = int(self.visit_radius / self.visit_cell) + 1
        cx, cy = self._visit_key(wx, wy)
        count = 0
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                count += self.visited.get((cx + dx, cy + dy), 0)
        return self.visit_weight * min(count, 20)   # cap so it never dominates

    def _on_reached(self, msg):
        if not (msg.data and self.goal is not None):
            return
        if self.returning:
            self.mission_done = True
            self.complete_pub.publish(Bool(data=True))   # -> follower lands
            self.get_logger().info(
                'Returned to entry/exit point -- MISSION COMPLETE '
                '(maze explored and exited). Signalling follower to LAND.')
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
            m = self.boundary_margin
            if (wx - self.min_x < m or self.max_x - wx < m
                    or wy - self.min_y < m or self.max_y - wy < m):
                continue        # boundary phantom (touches walled-off outside)
            if self.blacklist.get(self._key((wx, wy)), 0) >= self.max_failures:
                continue        # repeatedly unreachable
            dist = math.hypot(wx - self.px, wy - self.py)
            if dist < self.min_dist:
                continue        # already here; nothing new revealed
            # Prefer close AND large AND unexplored. Size in metres so the terms
            # are comparable regardless of grid resolution; the visit penalty
            # pushes exploration away from areas already covered (the memory).
            cost = (dist
                    - self.info_weight * (len(group) * res)
                    + self._visit_penalty(wx, wy))
            if self.goal is not None and math.hypot(
                    wx - self.goal[0], wy - self.goal[1]) < 1.0:
                cost -= self.hysteresis     # stick with what we're already chasing
            out.append((cost, wx, wy, len(group), dist))
        return out

    def _coverage_candidates(self):
        """Unsearched free cells from the coverage grid, as goals. These make
        the drone fly to any mapped-but-not-yet-seen area (e.g. a room it flew
        past) so the camera actually sweeps it -- the coverage guarantee. Same
        (cost, wx, wy, size, dist) shape as frontier candidates; size is 0 (a
        coverage cell carries no 'frontier size')."""
        cov = self.coverage_msg
        if cov is None:
            return []
        info = cov.info
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
        out = []
        for i, v in enumerate(cov.data):
            if v != 0:                  # 0 == free & UNSEARCHED (the to-do cells)
                continue
            col, row = i % info.width, i // info.width
            wx = ox + (col + 0.5) * res
            wy = oy + (row + 0.5) * res
            if self.blacklist.get(self._key((wx, wy)), 0) >= self.max_failures:
                continue
            dist = math.hypot(wx - self.px, wy - self.py)
            if dist < self.min_dist:
                continue
            cost = dist + self._visit_penalty(wx, wy)
            if self.goal is not None and math.hypot(
                    wx - self.goal[0], wy - self.goal[1]) < 1.0:
                cost -= self.hysteresis
            out.append((cost, wx, wy, 0, dist))
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

        # Give up on a goal we've chased too long OR made no headway toward
        # (stuck), and remember the failure so we don't re-pick it.
        if self.goal is not None and self.goal_start is not None:
            now = self.get_clock().now()
            elapsed = (now - self.goal_start).nanoseconds / 1e9
            # Stuck: has the drone actually moved since we last saw progress?
            if self.goal_anchor is None:
                self.goal_anchor = (self.px, self.py)
                self.goal_anchor_time = now
            moved = math.hypot(self.px - self.goal_anchor[0],
                               self.py - self.goal_anchor[1])
            if moved > self.stuck_min_move:
                self.goal_anchor = (self.px, self.py)
                self.goal_anchor_time = now
            stuck_for = (now - self.goal_anchor_time).nanoseconds / 1e9
            if elapsed > self.goal_timeout or stuck_for > self.stuck_window:
                k = self._key(self.goal)
                self.blacklist[k] = self.blacklist.get(k, 0) + 1
                why = 'timeout' if elapsed > self.goal_timeout else 'stuck (no progress)'
                self.get_logger().warn(
                    f'Goal {self.goal[0]:.2f},{self.goal[1]:.2f} abandoned '
                    f'[{why}] (failures={self.blacklist[k]})')
                self.goal = None
                self.goal_anchor = None

        clusters = self._cluster(self._find_frontiers())
        self._publish_markers(clusters)
        # Goals = frontiers (expand the map into unknown) + unsearched coverage
        # cells (point the camera at mapped-but-unseen areas). The drone only
        # finishes when BOTH are exhausted -> the whole reachable arena is both
        # mapped AND searched.
        frontier_cands = self._candidates(clusters)
        cands = frontier_cands + self._coverage_candidates()
        cands.sort(key=lambda c: c[0])
        # Local-first sweep: if anything is within sweep_radius, drop the far
        # ones so the drone finishes its neighbourhood before a long (SLAM-
        # stressing) traverse. Far goals survive only when nothing near remains.
        near = [c for c in cands if c[4] <= self.sweep_radius]
        if near:
            cands = near

        # Track the coverage high-water mark and when it last improved (for the
        # plateau test below).
        frac = self._coverage_frac()
        now = self.get_clock().now()
        if frac is not None and frac > self._best_frac + 0.5:
            self._best_frac = frac
            self._best_frac_time = now

        # COMMIT-TO-RETURN latch. Once the whole arena is searched, go home and
        # STAY going home. Without this the endgame flip-flops: a phantom frontier
        # hugging a boundary wall (free cells touching the walled-off unknown
        # OUTSIDE the maze -- it can never be cleared) keeps re-appearing as the
        # drone backs away from it, dragging it out of 'returning' and back to
        # chasing it. The drone then oscillates against the boundary wall until it
        # clips it and tumbles (observed: 100% coverage, then a wall-crash with no
        # SLAM loss). At full coverage there is nothing left to search, so any
        # lingering frontier is a phantom -- ignore them all and fly the entry.
        # Guarded by 'not frontier_cands' (map fully expanded -- only phantoms,
        # now excluded, would remain) so an EARLY coverage spike, when only a few
        # cells are mapped and all happen to be searched, can't send the drone
        # home before the maze is actually explored.
        if (not self._return_committed and not frontier_cands
                and self.entry_xy is not None
                and frac is not None and frac >= self.coverage_complete_pct):
            self._return_committed = True
            self.returning = True
            self.get_logger().info(
                f'Coverage complete ({self._coverage_pct()}) -> committing to '
                'RETURN (ignoring any boundary phantom frontiers)')
        if self._return_committed:
            self.goal = self.entry_xy
            self._request_path(self.entry_xy[0], self.entry_xy[1], 0, 0.0, 0)
            return

        # Search-done: arena fully MAPPED (no frontiers) AND either coverage is
        # high or it has plateaued. Then stop chasing the last unmarkable cells
        # and return home.
        if not self.returning and not frontier_cands and frac is not None:
            stalled = (self._best_frac_time is not None and
                       (now - self._best_frac_time).nanoseconds / 1e9
                       > self.coverage_stall_sec)
            if frac >= self.coverage_complete_pct or stalled:
                cands = []          # fall through to the return-to-entry path

        if not cands:
            # Might just be a transient (a single cycle with no cluster big
            # enough). Only declare the mission finished after several empty
            # cycles in a row -- returning on the first one sent the drone home
            # far too early last time.
            self.empty_count += 1
            if self.empty_count < self.empty_cycles_to_finish and not self.returning:
                return
            if self.entry_xy is None:
                return
            if not self.returning:
                self.returning = True
                self._return_committed = True   # latch: no flip-flop back out
                self.get_logger().info(
                    f'No reachable frontier or unsearched cell for '
                    f'{self.empty_count} cycles -> search done ({self._coverage_pct()}), '
                    'returning to entry/exit point')
            self.goal = self.entry_xy
            self._request_path(self.entry_xy[0], self.entry_xy[1], 0, 0.0, 0)
            return

        # Frontiers exist -> we are NOT finished. Reset the empty counter, and if
        # we had prematurely switched to 'returning', cancel it and keep exploring.
        self.empty_count = 0
        if self.returning:
            self.returning = False
            self.goal = None
            self.get_logger().info(
                'New frontiers appeared -> resuming exploration (not returning)')

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
            self.goal_anchor = (self.px, self.py)   # reset stuck detector
            self.goal_anchor_time = self.goal_start
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
            # No path == the goal is UNREACHABLE (commonly a phantom coverage
            # cell: a 1 m grid cell that clips a thin wall gets misclassified as
            # free but can never be flown to). Strike it hard (2 at once) so two
            # no-paths retire it instead of grinding 3x45 s timeouts -- otherwise
            # dozens of phantom cells stall the whole mission below 100%.
            k = self._key((wx, wy))
            self.blacklist[k] = self.blacklist.get(k, 0) + 2
            what = 'entry point' if self.returning else f'goal ({wx:.2f}, {wy:.2f})'
            self.get_logger().warn(
                f'No path to {what} (unreachable, strikes={self.blacklist[k]}) '
                '-> reselecting')
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
