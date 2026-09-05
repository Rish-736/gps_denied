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

The fence is a RECTANGLE in the map frame matching the arena bounds (plus a
margin), NOT a symmetric box centred on the entry. The entry sits on the arena
EDGE (the drone spawns in the boundary gap at the origin), so the arena extends
far in +y and only a little in -y; a symmetric ±box centred there would flag the
whole far half of the maze as "outside" -- which is exactly the false-breach bug
that fired RETURN_TO_EXIT mid-search. Arena bounds are the same ones the coverage
tracker and explorer use, passed in as parameters.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')
        # Arena rectangle in the map frame (same convention as coverage_tracker
        # / frontier_explorer). Defaults cover the 2.0m-corridor nidar_sim maze
        # x[-7,7] y[-1,13]; override per world.
        self.declare_parameter('arena_min_x', -7.0)
        self.declare_parameter('arena_max_x', 7.0)
        self.declare_parameter('arena_min_y', -1.0)
        self.declare_parameter('arena_max_y', 13.0)
        # Margin added outside the arena rectangle before a breach is declared,
        # so localisation jitter near a boundary wall doesn't false-trip.
        self.declare_parameter('margin_m', 1.5)
        # Below both the 8ft (2.44m) arena ceiling and our 2.5m sim maze walls,
        # so a climb-out is caught BEFORE the drone can clear the walls and
        # leave the maze (at which point the LiDAR sees nothing and SLAM dies).
        self.declare_parameter('max_height_m', 2.0)
        g = lambda n: self.get_parameter(n).value
        m = g('margin_m')
        self.min_x = g('arena_min_x') - m
        self.max_x = g('arena_max_x') + m
        self.min_y = g('arena_min_y') - m
        self.max_y = g('arena_max_y') + m
        self.max_height = g('max_height_m')

        self.breached = False

        self.create_subscription(
            PoseStamped, '/mavros/vision_pose/pose', self._on_pose, 10)
        self.pub = self.create_publisher(Bool, '/mission/geofence_breach', 10)

        self.get_logger().info(
            f'geofence_monitor: arena rect x[{self.min_x:.1f},{self.max_x:.1f}] '
            f'y[{self.min_y:.1f},{self.max_y:.1f}] (incl. margin), '
            f'0-{self.max_height}m height')

    def _on_pose(self, msg: PoseStamped):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

        inside = (self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y
                  and 0.0 <= z <= self.max_height)

        if not inside and not self.breached:
            self.breached = True
            self.get_logger().error(
                f'GEOFENCE BREACH: x={x:.2f} y={y:.2f} z={z:.2f} '
                f'(arena x[{self.min_x:.1f},{self.max_x:.1f}] '
                f'y[{self.min_y:.1f},{self.max_y:.1f}], 0-{self.max_height}m)')
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
