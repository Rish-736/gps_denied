#!/usr/bin/env python3
"""
Nav2 -> PX4 velocity bridge.

Nav2 emits /cmd_vel in the ROBOT BODY frame (x = forward, y = left). MAVROS's
/mavros/setpoint_velocity/cmd_vel_unstamped is interpreted in the LOCAL/WORLD
ENU frame. Publishing Nav2's output straight through therefore sends the drone
along world +X no matter which way its nose points -- which is exactly why
"fly forward" drove it sideways into a corridor wall. This node rotates body ->
world using the drone's current yaw before publishing.

It also owns altitude, which Nav2 knows nothing about (it's a 2D planner): a
simple P controller holds `target_altitude` by adding a vz term.

Finally it streams continuously at `rate_hz` even when Nav2 is silent (sending
zero velocity = hover). PX4 drops out of OFFBOARD if setpoints stop arriving,
so the stream must never stall between Nav2 goals.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped


def yaw_from_quaternion(q):
    """Extract yaw (rotation about Z) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CmdVelToMavros(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_mavros')
        self.declare_parameter('target_altitude', 1.2)
        self.declare_parameter('altitude_p_gain', 0.8)
        self.declare_parameter('max_vz', 0.4)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)

        self.target_alt = self.get_parameter('target_altitude').value
        self.kp_alt = self.get_parameter('altitude_p_gain').value
        self.max_vz = self.get_parameter('max_vz').value
        rate_hz = self.get_parameter('rate_hz').value
        self.cmd_timeout = self.get_parameter('cmd_timeout_sec').value

        self.yaw = 0.0
        self.altitude = 0.0
        self.have_pose = False
        self.last_cmd = Twist()
        self.last_cmd_time = None

        # MAVROS publishes pose with BEST_EFFORT; a RELIABLE subscription would
        # silently never match and we'd fly with yaw stuck at 0.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._on_pose, sensor_qos)
        self.pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'cmd_vel_to_mavros: body->world rotation + altitude hold at '
            f'{self.target_alt}m, streaming {rate_hz} Hz')

    def _on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_pose(self, msg: PoseStamped):
        self.yaw = yaw_from_quaternion(msg.pose.orientation)
        self.altitude = msg.pose.position.z
        self.have_pose = True

    def _tick(self):
        if not self.have_pose:
            return  # no yaw yet - publishing would send the drone the wrong way

        # Drop stale commands: if Nav2 goes quiet (goal reached, planner
        # thinking, node died) we must hover, not coast on the last command.
        cmd = self.last_cmd
        if self.last_cmd_time is None:
            cmd = Twist()
        else:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if age > self.cmd_timeout:
                cmd = Twist()

        # Body -> world (ENU) rotation about yaw.
        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
        vx_world = cmd.linear.x * cos_y - cmd.linear.y * sin_y
        vy_world = cmd.linear.x * sin_y + cmd.linear.y * cos_y

        # Altitude hold (Nav2 is 2D and never commands z).
        vz = self.kp_alt * (self.target_alt - self.altitude)
        vz = max(-self.max_vz, min(self.max_vz, vz))

        out = Twist()
        out.linear.x = vx_world
        out.linear.y = vy_world
        out.linear.z = vz
        out.angular.z = cmd.angular.z   # yaw rate is frame-independent
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelToMavros()
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
