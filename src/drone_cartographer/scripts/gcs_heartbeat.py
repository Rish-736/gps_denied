#!/usr/bin/env python3
"""
GCS heartbeat  --  proves the command & control link is alive.

Run this ON THE GROUND CONTROL STATION machine for the whole mission:
    python3 gcs_heartbeat.py
It publishes /gcs/heartbeat at 2 Hz over the local link. mission_fsm watches it:
if the heartbeat was alive and then stops for longer than gcs_timeout_sec, the
C2 link is considered lost and the drone RECALLs (returns & lands) autonomously
-- the loss-of-command-and-control-link failsafe (rules Section 10).

If this is never run (e.g. a headless sim), mission_fsm simply never arms the
C2-loss check, so it can't false-trigger.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


def main():
    rclpy.init()
    node = Node('gcs_heartbeat')
    pub = node.create_publisher(Header, '/gcs/heartbeat', 10)
    node.get_logger().info('GCS heartbeat publishing on /gcs/heartbeat @ 2 Hz')

    def beat():
        h = Header()
        h.stamp = node.get_clock().now().to_msg()
        h.frame_id = 'gcs'
        pub.publish(h)

    node.create_timer(0.5, beat)
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
