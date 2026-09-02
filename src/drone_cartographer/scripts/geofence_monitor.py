#!/usr/bin/env python3
"""
Fixes risk #3: PX4's built-in geofence is GPS lat/lon based, which is
useless indoors (no GPS at all at the venue). This node is our own
geofence: it watches the drone's map-frame position (fed by
vision_pose_bridge) against a local bounding box sized to the arena, and
publishes a breach flag the mission FSM (mission_fsm.py) reacts to by
forcing an autonomous return -- exactly the "geofence breach" failsafe the
mission rules require, implemented in local coordinates since GPS doesn't
exist here.

Box is centered on wherever the drone armed (assumed = entry point), sized
generously vs. the stated max arena size (15m x 15m) with margin.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')
        self.declare_parameter('half_size_m', 9.0)  # 18m x 18m box: 15m arena + margin
        # Below both the 8ft (2.44m) arena ceiling and our 2.5m sim maze walls,
        # so a climb-out is caught BEFORE the drone can clear the walls and
        # leave the maze (at which point the LiDAR sees nothing and SLAM dies).
        self.declare_parameter('max_height_m', 2.0)
        self.half_size = self.get_parameter('half_size_m').value
        self.max_height = self.get_parameter('max_height_m').value

        self.origin = None  # recorded on first pose received (= entry point)
        self.breached = False

        self.create_subscription(
            PoseStamped, '/mavros/vision_pose/pose', self._on_pose, 10)
        self.pub = self.create_publisher(Bool, '/mission/geofence_breach', 10)

        self.get_logger().info(
            f'geofence_monitor: box ±{self.half_size}m horiz, '
            f'0-{self.max_height}m height, centered on first pose received')

    def _on_pose(self, msg: PoseStamped):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

        if self.origin is None:
            self.origin = (x, y)
            self.get_logger().info(f'Geofence origin (entry point) set at x={x:.2f}, y={y:.2f}')
            return

        dx = x - self.origin[0]
        dy = y - self.origin[1]
        inside = (abs(dx) <= self.half_size and abs(dy) <= self.half_size
                  and 0.0 <= z <= self.max_height)

        if not inside and not self.breached:
            self.breached = True
            self.get_logger().error(
                f'GEOFENCE BREACH: dx={dx:.2f} dy={dy:.2f} z={z:.2f} '
                f'(limit ±{self.half_size}m, 0-{self.max_height}m)')
        elif inside and self.breached:
            self.breached = False
            self.get_logger().warn('Back inside geofence')

        self.pub.publish(Bool(data=self.breached))


def main():
    rclpy.init()
    node = GeofenceMonitor()
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
