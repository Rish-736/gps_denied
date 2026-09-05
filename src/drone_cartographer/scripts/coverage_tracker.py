#!/usr/bin/env python3
"""
Coverage grid  --  proves the whole arena was actually SEARCHED, not just mapped.

WHY THIS EXISTS
===============
Frontier exploration maps where the *walls* are. But mapping a corridor's walls
is NOT the same as pointing the camera into every place a survivor could be. In
a flight test the drone declared "done" having flown past whole regions its
camera never looked at -- in a real rescue that is a missed survivor and a lost
mission.

This node maintains a second, coarse grid over the arena (the SAME virtual grid
the mission rules want survivor locations reported in -- see
docs/mission_rules_compliance.md sec 6). Every cell starts UNSEARCHED; a cell
becomes SEARCHED only once the drone has actually been in a pose from which its
camera could see that cell:
    * the cell is within camera range of the drone,
    * within the camera's field of view (around the drone's heading), and
    * has clear line of sight in the occupancy map (no wall in between).
So a cell can only be ticked off if the sensor genuinely covered it.

The explorer consumes this grid: it keeps flying to the nearest UNSEARCHED,
reachable, free cell until none remain, then returns to the entry point. That
turns "no more frontiers" (a guess) into "every reachable cell was searched" (a
guarantee).

Grid-cell size is a PARAMETER, not hardcoded: the NIDAR brief does not state a
cell size, so it must be set to whatever the organizers confirm (default 1.0 m).

Published:
  /coverage_grid  (nav_msgs/OccupancyGrid)  -- for RViz / the GCS. Cell values:
        -1 = unknown (not yet mapped as free),
         0 = free but NOT yet searched  (the to-do list),
       100 = searched (done).
  /coverage/percent (std_msgs/Float32)      -- searched / free, 0..100.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener, TransformException

UNKNOWN, UNSEARCHED, SEARCHED = -1, 0, 100


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CoverageTracker(Node):
    def __init__(self):
        super().__init__('coverage_tracker')
        # Arena bounds + cell size define the mission grid. Cell size is a
        # parameter on purpose (the brief doesn't fix it). Defaults cover the
        # wide sim maze; set to the real 15x15 arena + confirmed cell size.
        self.declare_parameter('arena_min_x', -8.5)
        self.declare_parameter('arena_max_x', 6.0)
        self.declare_parameter('arena_min_y', -1.0)
        self.declare_parameter('arena_max_y', 13.5)
        self.declare_parameter('cell_size', 1.0)
        # Camera model used to decide when a cell counts as SEARCHED (i.e. a
        # survivor there would have been seen). These MUST be set to the real
        # camera's usable HUMAN-DETECTION range and FOV once it is chosen -- not
        # the lens's raw spec, but the distance/angle at which the detector
        # reliably recognises a person. Deliberately conservative defaults
        # (3.0 m / 100 deg, down from a generous 4.0/140): a tighter cone forces
        # the drone to actually approach and face each area rather than ticking
        # cells off from far down a corridor -> a genuinely thorough search.
        self.declare_parameter('camera_range', 3.0)
        self.declare_parameter('camera_fov_deg', 100.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('update_rate_hz', 4.0)
        # Occupancy thresholds (Cartographer stores free as 0..49).
        self.declare_parameter('free_below', 50)
        self.declare_parameter('occupied_at', 65)

        g = lambda n: self.get_parameter(n).value
        self.min_x, self.max_x = g('arena_min_x'), g('arena_max_x')
        self.min_y, self.max_y = g('arena_min_y'), g('arena_max_y')
        self.cell = g('cell_size')
        self.cam_range = g('camera_range')
        self.cam_fov = math.radians(g('camera_fov_deg'))
        self.map_frame, self.body_frame = g('map_frame'), g('body_frame')
        self.free_below = g('free_below')
        self.occupied_at = g('occupied_at')

        self.ncols = max(1, int(math.ceil((self.max_x - self.min_x) / self.cell)))
        self.nrows = max(1, int(math.ceil((self.max_y - self.min_y) / self.cell)))
        # state[row][col]
        self.state = [[UNKNOWN] * self.ncols for _ in range(self.nrows)]

        self.map_msg = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/coverage_grid', latched)
        self.pct_pub = self.create_publisher(Float32, '/coverage/percent', latched)

        self._last_log = 0
        self.create_timer(1.0 / g('update_rate_hz'), self._tick)
        self.get_logger().info(
            f'coverage_tracker: {self.ncols}x{self.nrows} cells @ {self.cell}m over '
            f'x[{self.min_x},{self.max_x}] y[{self.min_y},{self.max_y}], '
            f'camera {math.degrees(self.cam_fov):.0f}deg / {self.cam_range}m')

    # ------------------------------------------------------------------ input
    def _on_map(self, msg):
        self.map_msg = msg

    def _cell_center(self, col, row):
        return (self.min_x + (col + 0.5) * self.cell,
                self.min_y + (row + 0.5) * self.cell)

    # ---------------------------------------------------------- map sampling
    def _map_val(self, wx, wy):
        """Occupancy value at world point, or None if outside the map."""
        info = self.map_msg.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if 0 <= mx < info.width and 0 <= my < info.height:
            return self.map_msg.data[my * info.width + mx]
        return None

    def _classify(self, col, row):
        """UNKNOWN / free / occupied for a coverage cell, by sampling the map
        over the cell (centre + quarter points) so a 1 m cell isn't judged by a
        single pixel."""
        cx, cy = self._cell_center(col, row)
        q = self.cell * 0.25
        free = occ = known = 0
        for dx, dy in ((0, 0), (q, q), (-q, q), (q, -q), (-q, -q)):
            v = self._map_val(cx + dx, cy + dy)
            if v is None or v == UNKNOWN:
                continue
            known += 1
            if v >= self.occupied_at:
                occ += 1
            elif 0 <= v < self.free_below:
                free += 1
        if known == 0:
            return 'unknown'
        if occ > 0:
            return 'occupied'
        if free > 0:
            return 'free'
        return 'unknown'

    def _los_clear(self, x0, y0, x1, y1):
        """Line of sight between two world points across the occupancy map:
        step along the ray at map resolution; blocked if any cell is occupied."""
        info = self.map_msg.info
        res = info.resolution
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(dist / res))
        for i in range(1, steps):  # skip endpoints (drone cell, target cell)
            t = i / steps
            v = self._map_val(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            if v is not None and v >= self.occupied_at:
                return False
        return True

    # --------------------------------------------------------------- main loop
    def _tick(self):
        if self.map_msg is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, Time())
        except TransformException:
            return
        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        dyaw = yaw_of(tf.transform.rotation)
        half_fov = self.cam_fov / 2.0

        free_total = searched_total = 0
        for row in range(self.nrows):
            for col in range(self.ncols):
                st = self.state[row][col]
                if st != SEARCHED:
                    kind = self._classify(col, row)
                    if kind == 'free' and st == UNKNOWN:
                        self.state[row][col] = st = UNSEARCHED
                    elif kind != 'free' and st == UNKNOWN:
                        continue        # wall or still-unknown: not a search cell
                if self.state[row][col] == UNKNOWN:
                    continue
                free_total += 1
                if self.state[row][col] == SEARCHED:
                    searched_total += 1
                    continue
                # Unsearched free cell: can the camera see it from here? Test
                # several points across the cell (centre + quarters), not just
                # the centre -- a 1 m cell that straddles a thin wall has its
                # centre blocked from LOS but its free part is perfectly
                # visible, and a survivor there would be seen. Marking on ANY
                # visible sub-point stops those cells stalling coverage forever.
                cx, cy = self._cell_center(col, row)
                q = self.cell * 0.3
                seen = False
                for sx, sy in ((cx, cy), (cx + q, cy + q), (cx - q, cy + q),
                               (cx + q, cy - q), (cx - q, cy - q)):
                    d = math.hypot(sx - dx, sy - dy)
                    if d > self.cam_range:
                        continue
                    if d > 1e-3:
                        bearing = math.atan2(sy - dy, sx - dx)
                        if abs(wrap(bearing - dyaw)) > half_fov:
                            continue    # outside the camera cone
                    if self._los_clear(dx, dy, sx, sy):
                        seen = True
                        break
                if seen:
                    self.state[row][col] = SEARCHED
                    searched_total += 1

        self._publish(free_total, searched_total)

    def _publish(self, free_total, searched_total):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.cell
        grid.info.width = self.ncols
        grid.info.height = self.nrows
        grid.info.origin.position.x = self.min_x
        grid.info.origin.position.y = self.min_y
        grid.info.origin.orientation.w = 1.0
        grid.data = [self.state[r][c]
                     for r in range(self.nrows) for c in range(self.ncols)]
        self.grid_pub.publish(grid)

        pct = (100.0 * searched_total / free_total) if free_total else 0.0
        self.pct_pub.publish(Float32(data=pct))
        self._last_log += 1
        if self._last_log % 8 == 0:      # ~ every 2s at 4Hz
            self.get_logger().info(
                f'coverage: {searched_total}/{free_total} free cells searched '
                f'({pct:.0f}%)')


def main():
    rclpy.init()
    node = CoverageTracker()
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
