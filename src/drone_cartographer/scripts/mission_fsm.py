#!/usr/bin/env python3
"""
Mission safety manager  --  the failsafe brain (mission rules Section 10).

The rulebook REQUIRES failsafes for: low battery, loss of command & control
link, geofence breach, mission abort, and emergency recall. This node owns all
of them and the mission time budget, and it also remembers the entry/exit point.

Every trigger maps to ONE of two actions that the flight code already knows how
to perform, so the safety logic stays in one place and the follower/explorer
just obey a flag:

  RECALL   (return to the entry point, then land) -- the graceful response.
           Triggers: operator recall, geofence breach, C2-link loss, low
           battery (warning level), or the 30-min time budget expiring.
           -> publishes /mission/recall = true. The frontier_explorer drops
              exploration and flies to the entry; on arrival the normal
              /mission/complete handshake lands the drone.

  LAND-NOW (immediate AUTO.LAND where it is) -- the hard response for when
           returning is unsafe or pointless. Triggers: operator emergency stop,
           or CRITICAL battery. -> publishes /mission/land_now = true, which the
           path follower turns into an immediate AUTO.LAND (baro descent, needs
           no position estimate).

Both flags are LATCHED and one-way: once safety fires it never un-fires.

Layering: PX4-native backstops still exist under this node (COM_OBL_ACT lands if
our companion stops sending setpoints; COM_LOW_BAT_ACT lands on critical battery;
QGC/MAVROS can always force-disarm). This node is the autonomy-level failsafe
that reacts gracefully BEFORE those blunt backstops are needed.

States: WAIT_FIRST_POSE -> EXPLORE -> RECALL / LAND_NOW.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String, Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class MissionSafety(Node):
    def __init__(self):
        super().__init__('mission_fsm')
        # 25 min, 5 min margin under the hard 30-min cap so there is always time
        # to fly back and land before the limit.
        self.declare_parameter('mission_time_budget_sec', 1500.0)
        # Battery fractions (0..1). Low -> graceful RECALL; critical -> LAND NOW.
        self.declare_parameter('batt_low_frac', 0.25)
        self.declare_parameter('batt_crit_frac', 0.12)
        # C2 (command & control) link: the GCS publishes /gcs/heartbeat while the
        # operator link is alive. If it was alive and then goes stale for longer
        # than this, we've lost the link -> RECALL. Never trips if a heartbeat was
        # never seen (e.g. a headless sim run with no GCS), so it can't false-fire.
        self.declare_parameter('gcs_timeout_sec', 3.0)

        g = lambda n: self.get_parameter(n).value
        self.budget_sec = g('mission_time_budget_sec')
        self.batt_low = g('batt_low_frac')
        self.batt_crit = g('batt_crit_frac')
        self.gcs_timeout = g('gcs_timeout_sec')

        self.state = 'WAIT_FIRST_POSE'
        self.entry_pose = None
        self.mission_start_time = None

        # trigger inputs
        self.geofence_breached = False
        self.operator_abort = False
        self.operator_recall = False
        self.batt_frac = None            # None until a valid reading arrives
        self._gcs_last = None            # wall time of last GCS heartbeat
        self._gcs_seen = False           # has the link ever been up?

        # latched outputs
        self.recall_fired = False
        self.land_now_fired = False

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.recall_pub = self.create_publisher(Bool, '/mission/recall', latched)
        self.land_pub = self.create_publisher(Bool, '/mission/land_now', latched)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.exit_pose_pub = self.create_publisher(PoseStamped, '/mission/exit_pose', latched)

        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self._on_pose, 10)
        self.create_subscription(Bool, '/mission/geofence_breach', self._on_geofence, 10)
        self.create_subscription(BatteryState, '/mavros/battery', self._on_batt, 10)
        self.create_subscription(Header, '/gcs/heartbeat', self._on_gcs, 10)
        self.create_subscription(Bool, '/mission/operator_abort', self._on_abort, 10)
        self.create_subscription(Bool, '/mission/operator_recall', self._on_op_recall, 10)

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'mission_fsm (safety): budget={self.budget_sec:.0f}s, '
            f'batt low/crit={self.batt_low:.0%}/{self.batt_crit:.0%}, '
            f'C2 timeout={self.gcs_timeout:.0f}s')

    # ------------------------------------------------------------- trigger inputs
    def _on_geofence(self, msg): self.geofence_breached = msg.data
    def _on_abort(self, msg):
        if msg.data:
            self.operator_abort = True
    def _on_op_recall(self, msg):
        if msg.data:
            self.operator_recall = True
    def _on_batt(self, msg):
        # percentage is 0..1, or -1/NaN when unknown (ignore those).
        p = msg.percentage
        if p is not None and 0.0 <= p <= 1.0:
            self.batt_frac = p
    def _on_gcs(self, msg):
        self._gcs_last = self.get_clock().now()
        self._gcs_seen = True

    def _on_pose(self, msg):
        if self.entry_pose is None:
            self.entry_pose = msg
            self.mission_start_time = self.get_clock().now()
            self.state = 'EXPLORE'
            self.exit_pose_pub.publish(msg)
            self.get_logger().info(
                f'Entry/exit point recorded at x={msg.pose.position.x:.2f}, '
                f'y={msg.pose.position.y:.2f} -- state -> EXPLORE')

    # --------------------------------------------------------------------- logic
    def _comm_lost(self):
        if not self._gcs_seen or self._gcs_last is None:
            return False                 # link never came up -> don't false-trip
        stale = (self.get_clock().now() - self._gcs_last).nanoseconds / 1e9
        return stale > self.gcs_timeout

    def _tick(self):
        if self.state == 'WAIT_FIRST_POSE':
            self.state_pub.publish(String(data=self.state))
            return

        # ---- LAND-NOW conditions (hardest): abort or critical battery ----------
        if not self.land_now_fired:
            reason = None
            if self.operator_abort:
                reason = 'OPERATOR EMERGENCY STOP'
            elif self.batt_frac is not None and self.batt_frac <= self.batt_crit:
                reason = f'CRITICAL BATTERY ({self.batt_frac:.0%})'
            if reason:
                self.land_now_fired = True
                self.state = 'LAND_NOW'
                self.land_pub.publish(Bool(data=True))
                self.get_logger().error(f'FAILSAFE [{reason}] -> LAND NOW')

        # ---- RECALL conditions (graceful return + land) ------------------------
        if not self.recall_fired and not self.land_now_fired:
            elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
            reason = None
            if self.operator_recall:
                reason = 'operator recall'
            elif self.geofence_breached:
                reason = 'geofence breach'
            elif self._comm_lost():
                reason = 'C2 link lost'
            elif self.batt_frac is not None and self.batt_frac <= self.batt_low:
                reason = f'low battery ({self.batt_frac:.0%})'
            elif elapsed >= self.budget_sec:
                reason = 'time budget expired'
            if reason:
                self.recall_fired = True
                self.state = 'RECALL'
                self.recall_pub.publish(Bool(data=True))
                self.get_logger().warn(f'FAILSAFE [{reason}] -> RECALL (return to entry & land)')
            elif int(elapsed) % 30 == 0:
                self.get_logger().info(
                    f'EXPLORE: {self.budget_sec - elapsed:.0f}s left in budget')

        self.state_pub.publish(String(data=self.state))


def main():
    rclpy.init()
    node = MissionSafety()
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
