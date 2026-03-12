#!/usr/bin/env python3
"""Test the XR bridge node with simulated controller input."""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy


class XRBridgeTest(Node):
    def __init__(self):
        super().__init__('xr_bridge_test')

        # Publishers (simulating teleop-xr output)
        self.pose_pub = self.create_publisher(
            PoseStamped, 'xr/controller_right/pose', 10)
        self.joy_pub = self.create_publisher(
            Joy, 'xr/controller_right/joy', 10)

        # Subscriber (checking bridge output)
        self.received_target = None
        self.create_subscription(
            PoseStamped, '/ik/right_hand_target',
            self._on_target, 10)

        self.step = 0
        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz
        self.tick_count = 0

    def _on_target(self, msg):
        p = msg.pose.position
        self.received_target = (p.x, p.y, p.z)
        self.get_logger().info(
            f'GOT IK TARGET: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}')

    def _pub_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = 'xr_local'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def _pub_joy(self, trigger):
        msg = Joy()
        msg.buttons = [1 if trigger else 0, 0, 0, 0]
        msg.axes = [0.0, 0.0]
        self.joy_pub.publish(msg)

    def _tick(self):
        self.tick_count += 1

        if self.tick_count < 40:
            # Phase 1 (0-2s): Send pose at origin, no trigger
            self._pub_pose(0.0, 0.0, 0.0)
            if self.tick_count == 1:
                self.get_logger().info('Phase 1: Sending origin pose...')

        elif self.tick_count < 80:
            # Phase 2 (2-4s): Press trigger while at origin
            self._pub_pose(0.0, 0.0, 0.0)
            self._pub_joy(True)
            if self.tick_count == 40:
                self.get_logger().info('Phase 2: Trigger pressed, engaging...')

        elif self.tick_count < 120:
            # Phase 3 (4-6s): Move forward in XR (z=-0.1), trigger held
            self._pub_pose(0.0, 0.0, -0.1)
            self._pub_joy(True)
            if self.tick_count == 80:
                self.get_logger().info(
                    'Phase 3: Moving forward (XR z=-0.1), '
                    'expect robot x += 0.1...')

        elif self.tick_count < 160:
            # Phase 4 (6-8s): Move right in XR (x=+0.05), trigger held
            self._pub_pose(0.05, 0.0, -0.1)
            self._pub_joy(True)
            if self.tick_count == 120:
                self.get_logger().info(
                    'Phase 4: Also moving right (XR x=+0.05), '
                    'expect robot y -= 0.05...')

        elif self.tick_count < 180:
            # Phase 5 (8-9s): Release trigger
            self._pub_pose(0.1, 0.0, -0.2)
            self._pub_joy(False)
            if self.tick_count == 160:
                self.get_logger().info(
                    'Phase 5: Trigger released (clutch), '
                    'target should stop updating...')

        else:
            # Done
            if self.received_target:
                self.get_logger().info(
                    f'TEST PASSED - Last target: {self.received_target}')
            else:
                self.get_logger().error(
                    'TEST FAILED - No IK target received!')
            self.timer.cancel()
            raise SystemExit(0)


def main():
    rclpy.init()
    node = XRBridgeTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
