#!/usr/bin/env python3
"""
Operator EMERGENCY RECALL  (mission rules Section 10: emergency recall).

Run this to command the drone to stop searching and return to the entry/exit
point, then land:
    python3 recall.py
It publishes /mission/operator_recall=true, which mission_fsm turns into a
RECALL: the explorer abandons exploration and flies home, and the normal
return-to-entry handshake lands the drone. Unlike e-stop this is graceful (it
comes home rather than dropping where it is). Publishes repeatedly for a couple
of seconds then exits.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = Node('operator_recall')
    pub = node.create_publisher(Bool, '/mission/operator_recall', 10)
    node.get_logger().warn('OPERATOR RECALL -> returning to entry & landing')
    # Wait for mission_fsm to be discovered as a subscriber BEFORE publishing --
    # otherwise the messages go out before the link is up and are lost.
    for _ in range(50):                          # up to ~5s for discovery
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error('no subscriber on /mission/operator_recall '
                                '(is mission_fsm running?) -- sending anyway')
    for _ in range(10):                          # publish a burst once connected
        pub.publish(Bool(data=True))
        rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().warn('recall sent.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
