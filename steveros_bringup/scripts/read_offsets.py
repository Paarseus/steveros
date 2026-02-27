#!/usr/bin/env python3
"""
Read current joint positions and calculate zero_offset_deg values.

Usage:
  1. Position the robot in its URDF zero pose (arms straight down, etc.)
  2. Run: python3 read_offsets.py
  3. Copy the printed zero_offset_deg values into steveros_ros2_control.xacro
"""

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class OffsetReader(Node):
    def __init__(self):
        super().__init__('offset_reader')
        self.received = False
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb, 10)
        self.get_logger().info('Waiting for /joint_states ...')

    def cb(self, msg: JointState):
        if self.received:
            return
        self.received = True

        print('\n' + '=' * 70)
        print('  CURRENT JOINT POSITIONS  (all zero_offset_deg = 0)')
        print('  If the robot is in its URDF zero pose right now,')
        print('  these values ARE the offsets you need.')
        print('=' * 70)

        print(f'\n{"Joint":<40} {"rad":>10} {"deg":>10}')
        print('-' * 62)

        for name, pos in zip(msg.name, msg.position):
            deg = pos * 180.0 / math.pi
            print(f'{name:<40} {pos:>+10.4f} {deg:>+10.2f}')

        print('\n' + '=' * 70)
        print('  XACRO SNIPPET — paste into steveros_ros2_control.xacro')
        print('=' * 70 + '\n')

        for name, pos in zip(msg.name, msg.position):
            deg = pos * 180.0 / math.pi
            print(f'    <!-- {name} -->')
            print(f'    <param name="zero_offset_deg">{deg:.2f}</param>')
            print()

        self.get_logger().info('Done. Shutting down.')
        raise SystemExit(0)


def main():
    rclpy.init()
    node = OffsetReader()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
