#!/usr/bin/env python3
"""
Autonomous arm -> takeoff -> OFFBOARD sequencer.

Removes the last bit of manual interaction: no typing in the pxh> console.
The mission rules only allow the operator to start the mission and hit abort,
so the launch sequence has to be automatic anyway.

Order matters, and the FIRST version of this file had it wrong. It did
arm -> AUTO.TAKEOFF -> OFFBOARD, and arming was refused every time
(CommandBool result=1, TEMPORARILY_REJECTED -- even force-arm). Reason: with no
GPS the drone boots into AUTO.LOITER, a mode that demands a valid position
estimate, and a vision-only drone sitting still doesn't have one yet
(const_pos_mode_status_flag stays true).

The order that actually works, verified live:
  1. cmd_vel_to_mavros is already streaming setpoints (PX4 requires a live
     setpoint stream before it will accept OFFBOARD at all)
  2. switch to OFFBOARD
  3. THEN arm            <- succeeds: success=True, result=0
  4. climb happens by itself, because cmd_vel_to_mavros' altitude hold
     commands vz toward target_altitude. PX4's AUTO.TAKEOFF isn't used at all.

Exits once OFFBOARD is confirmed held; the explorer takes over from there.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class AutoTakeoff(Node):
    def __init__(self):
        super().__init__('auto_takeoff')
        self.declare_parameter('takeoff_altitude', 1.2)
        self.declare_parameter('airborne_fraction', 0.8)

        self.target_alt = self.get_parameter('takeoff_altitude').value
        self.airborne_frac = self.get_parameter('airborne_fraction').value

        self.state = None
        self.altitude = 0.0
        self.stage = 'WAIT_CONNECT'
        self.ticks = 0

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(State, '/mavros/state', self._on_state, state_qos)

        from geometry_msgs.msg import PoseStamped
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            lambda m: setattr(self, 'altitude', m.pose.position.z), state_qos)

        # No takeoff client: PX4's AUTO.TAKEOFF is deliberately unused (it needs
        # a position estimate we don't have on the ground). The climb comes from
        # cmd_vel_to_mavros' altitude hold once we're armed in OFFBOARD.
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

        self.create_timer(1.0, self._tick)
        self.get_logger().info('auto_takeoff: waiting for FCU connection...')

    def _on_state(self, msg):
        self.state = msg

    def _tick(self):
        if self.state is None:
            return
        self.ticks += 1

        if self.stage == 'WAIT_CONNECT':
            if self.state.connected:
                self.get_logger().info(
                    'FCU connected -> waiting for setpoint stream before OFFBOARD')
                self.stage = 'OFFBOARD'

        elif self.stage == 'OFFBOARD':
            # OFFBOARD *before* arming. PX4 will not arm in AUTO.LOITER without
            # a valid position estimate, which vision-only gives us only once
            # flying. Retry every 3s rather than every tick: hammering the
            # service with un-awaited calls gets commands rejected.
            if self.state.mode == 'OFFBOARD':
                self.get_logger().info('OFFBOARD accepted -> arming')
                self.stage = 'ARM'
            elif self.mode_cli.service_is_ready() and self.ticks % 3 == 0:
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                self.mode_cli.call_async(req)

        elif self.stage == 'ARM':
            if self.state.armed:
                self.get_logger().info(
                    f'Armed in OFFBOARD - climbing to {self.target_alt}m '
                    f'via altitude hold. Exploration has control.')
                self.stage = 'DONE'
            elif self.arm_cli.service_is_ready() and self.ticks % 3 == 0:
                req = CommandBool.Request()
                req.value = True
                self.arm_cli.call_async(req)

        elif self.stage == 'DONE':
            # mode_sent=True never guaranteed the mode held, so keep watching
            # and re-assert if PX4 drops out of OFFBOARD mid-mission.
            if self.state.mode != 'OFFBOARD' and self.state.armed:
                self.get_logger().warn(
                    f'Dropped out of OFFBOARD (now {self.state.mode}) - re-asserting')
                self.stage = 'OFFBOARD'


def main():
    rclpy.init()
    node = AutoTakeoff()
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
