#!/usr/bin/env python3
"""
Operator EMERGENCY STOP  (mission rules Section 10: mission abort / e-stop).

Run this to abort the mission and bring the drone straight down:
    python3 estop.py
It publishes /mission/operator_abort=true, which mission_fsm turns into an
immediate LAND-NOW (AUTO.LAND) via the path follower. Publishes repeatedly for a
couple of seconds so the message is guaranteed to land even on a lossy link,
then exits.

This is the software abort over the local link. On the real aircraft the operator
ALSO has PX4's native force-disarm / land from the GCS (QGroundControl) as an
independent hard kill -- both satisfy the rulebook's emergency-stop requirement.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = Node('operator_estop')
    pub = node.create_publisher(Bool, '/mission/operator_abort', 10)
    node.get_logger().warn('OPERATOR EMERGENCY STOP -> commanding LAND NOW')
    # Wait for mission_fsm to be discovered as a subscriber BEFORE publishing --
    # otherwise the messages go out before the link is up and are lost.
    for _ in range(50):                          # up to ~5s for discovery
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error('no subscriber on /mission/operator_abort '
                                '(is mission_fsm running?) -- sending anyway')
    for _ in range(10):                          # publish a burst once connected
        pub.publish(Bool(data=True))
        rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().warn('e-stop sent.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
