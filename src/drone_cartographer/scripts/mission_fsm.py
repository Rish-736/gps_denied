#!/usr/bin/env python3
"""
Fixes risks #5 and #6:

#5 - "return to exit" needs the drone to remember where it started. This
     node records the first pose it sees as the entry/exit point (the rules
     state entry and exit are the same point), in the map frame, so a later
     Nav2 goal-sender can send the drone back there.

#6 - the 30-minute mission cap is a hard rule, and finding one more
     survivor is worth nothing if the drone never exits. This node runs its
     own countdown, independent of how exploration is going, and force-
     transitions EXPLORE -> RETURN_TO_EXIT once the budget expires -- with
     margin under the real 30-minute limit so there's always time left to
     actually fly back.

This is a scaffold: it owns the state and the entry pose, and publishes
both on topics. It does NOT yet drive Nav2 -- the goal-sender that
subscribes to /mission/exit_pose and actually commands a return doesn't
exist yet (comes with the Nav2/frontier-exploration work). Wiring that up
is the next step once Nav2 is confirmed working.

States: WAIT_FIRST_POSE -> EXPLORE -> RETURN_TO_EXIT -> LANDED
Forced transition to RETURN_TO_EXIT on: time budget expired, OR
/mission/geofence_breach = true.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String


class MissionFSM(Node):
    def __init__(self):
        super().__init__('mission_fsm')
        self.declare_parameter('mission_time_budget_sec', 1500.0)  # 25 min, 5 min margin under the 30 min cap

        self.budget_sec = self.get_parameter('mission_time_budget_sec').value
        self.state = 'WAIT_FIRST_POSE'
        self.entry_pose = None
        self.mission_start_time = None
        self.geofence_breached = False

        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self._on_pose, 10)
        self.create_subscription(Bool, '/mission/geofence_breach', self._on_geofence, 10)
        self.exit_pose_pub = self.create_publisher(PoseStamped, '/mission/exit_pose', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'mission_fsm: budget={self.budget_sec:.0f}s '
            f'(cap is 1800s / 30min per mission rules)')

    def _on_geofence(self, msg: Bool):
        self.geofence_breached = msg.data

    def _on_pose(self, msg: PoseStamped):
        if self.entry_pose is None:
            self.entry_pose = msg
            self.mission_start_time = self.get_clock().now()
            self.state = 'EXPLORE'
            self.get_logger().info(
                f'Entry/exit point recorded at '
                f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f} '
                f'-- state -> EXPLORE')

    def _tick(self):
        if self.state == 'EXPLORE':
            elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
            remaining = self.budget_sec - elapsed

            if self.geofence_breached:
                self.get_logger().error('Geofence breached -- forcing RETURN_TO_EXIT')
                self._go_to_return_state()
            elif remaining <= 0:
                self.get_logger().warn('Time budget expired -- forcing RETURN_TO_EXIT')
                self._go_to_return_state()
            elif int(elapsed) % 30 == 0:
                self.get_logger().info(f'EXPLORE: {remaining:.0f}s left in budget')

        self.state_pub.publish(String(data=self.state))

    def _go_to_return_state(self):
        self.state = 'RETURN_TO_EXIT'
        self.exit_pose_pub.publish(self.entry_pose)


def main():
    rclpy.init()
    node = MissionFSM()
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
