"""Bridge between teleop-xr and the IK node.

Subscribes to XR controller poses and joy (trigger), computes deltas
relative to the engage point, and publishes IK targets in the robot frame.

The axis mapping is derived dynamically from the head pose on each engage,
so the user's facing direction always maps to robot +X (forward).

While trigger is held the arm tracks in real time. Releasing the trigger
"clutches" — the arm stays put and the user can reposition their hand.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy


# Axis mapping: teleop-xr already outputs in ROS REP-103 convention
# (+X forward, +Y left, +Z up). Confirmed empirically by calibration.
# Identity map is correct; _axis_map_from_head() adapts for user facing.
DEFAULT_AXIS_MAP = [
    [1.0, 0.0, 0.0],   # robot X (forward) = teleop X
    [0.0, 1.0, 0.0],   # robot Y (left)    = teleop Y
    [0.0, 0.0, 1.0],   # robot Z (up)      = teleop Z
]


class HandState:
    """Tracking state for one hand."""

    def __init__(self, initial_ee: list[float]):
        self.engaged = False
        self.xr_reference: np.ndarray | None = None
        self.ee_start = np.array(initial_ee)
        self.current_target = np.array(initial_ee)


class XRBridgeNode(Node):
    def __init__(self):
        super().__init__('xr_bridge')

        # Parameters
        self.declare_parameter('trigger_button', 0)
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('initial_ee_right', [0.15, -0.18, 0.35])
        self.declare_parameter('initial_ee_left', [0.15, 0.18, 0.35])
        self.declare_parameter('axis_map', sum(DEFAULT_AXIS_MAP, []))  # flat 9 floats

        self._trigger_btn = self.get_parameter('trigger_button').value
        self._scale = self.get_parameter('scale').value

        axis_flat = list(self.get_parameter('axis_map').value)
        self._axis_map = np.array(axis_flat).reshape(3, 3)

        initial_r = list(self.get_parameter('initial_ee_right').value)
        initial_l = list(self.get_parameter('initial_ee_left').value)

        self._hands = {
            'right': HandState(initial_r),
            'left': HandState(initial_l),
        }

        # Latest XR pose per hand (stored continuously, used on each frame)
        self._xr_pose: dict[str, np.ndarray | None] = {'right': None, 'left': None}
        # Latest XR orientation per hand (quaternion: x, y, z, w)
        self._xr_orient: dict[str, tuple | None] = {'right': None, 'left': None}

        # Actual EE positions from IK node FK (used on re-engage)
        self._actual_ee: dict[str, np.ndarray | None] = {'right': None, 'left': None}

        # Head orientation for dynamic axis mapping
        self._head_quat: tuple[float, float, float, float] | None = None

        # Subscribers — head pose for axis calibration
        self.create_subscription(
            PoseStamped, '/xr/head/pose',
            self._on_head_pose, 10,
        )

        # Subscribers — actual EE feedback from IK node
        self.create_subscription(
            PoseStamped, '/ik/right_hand_actual',
            lambda msg: self._on_actual_ee('right', msg), 10,
        )
        self.create_subscription(
            PoseStamped, '/ik/left_hand_actual',
            lambda msg: self._on_actual_ee('left', msg), 10,
        )

        # Subscribers — XR controller poses
        self.create_subscription(
            PoseStamped, 'xr/controller_right/pose',
            lambda msg: self._on_xr_pose('right', msg), 10,
        )
        self.create_subscription(
            PoseStamped, 'xr/controller_left/pose',
            lambda msg: self._on_xr_pose('left', msg), 10,
        )

        # Subscribers — XR controller buttons
        self.create_subscription(
            Joy, 'xr/controller_right/joy',
            lambda msg: self._on_joy('right', msg), 10,
        )
        self.create_subscription(
            Joy, 'xr/controller_left/joy',
            lambda msg: self._on_joy('left', msg), 10,
        )

        # Publishers — IK targets
        self._ik_pubs = {
            'right': self.create_publisher(PoseStamped, '/ik/right_hand_target', 10),
            'left': self.create_publisher(PoseStamped, '/ik/left_hand_target', 10),
        }

        self.get_logger().info(
            f'XR bridge ready: trigger=btn[{self._trigger_btn}], '
            f'scale={self._scale}'
        )

    def _on_head_pose(self, msg: PoseStamped):
        """Store latest head orientation for axis mapping."""
        o = msg.pose.orientation
        self._head_quat = (o.x, o.y, o.z, o.w)

    def _axis_map_from_head(self) -> np.ndarray | None:
        """Compute axis map from current head orientation.

        Uses the head's horizontal facing direction to align controller
        deltas with the robot frame. Called on each engage so the mapping
        adapts to the user's current facing direction.

        teleop-xr outputs in ROS REP-103: +X forward, +Y left, +Z up.
        Head forward = R @ [1, 0, 0] (first column of rotation matrix).
        """
        if self._head_quat is None:
            return None

        qx, qy, qz, qw = self._head_quat

        # Head forward = R @ [1, 0, 0] (first column of rotation matrix).
        fwd_x = 1.0 - 2.0 * (qy * qy + qz * qz)
        fwd_y = 2.0 * (qx * qy + qw * qz)
        # fwd_z = 2.0 * (qx * qz - qw * qy)  # vertical component, projected out

        # Project onto horizontal plane (zero out Z, which is up)
        horiz_norm = np.sqrt(fwd_x * fwd_x + fwd_y * fwd_y)
        if horiz_norm < 1e-6:
            return None  # looking straight up/down

        fwd_h = np.array([fwd_x / horiz_norm, fwd_y / horiz_norm, 0.0])
        up = np.array([0.0, 0.0, 1.0])
        left = np.cross(up, fwd_h)  # [-fwd_y, fwd_x, 0]

        # Each row maps teleop-xr delta → one robot axis
        return np.array([
            fwd_h,   # robot X (forward) = component along user's forward
            left,    # robot Y (left)    = component along user's left
            up,      # robot Z (up)      = vertical component
        ])

    def _on_actual_ee(self, side: str, msg: PoseStamped):
        """Store latest actual EE position from IK node FK."""
        pos = msg.pose.position
        self._actual_ee[side] = np.array([pos.x, pos.y, pos.z])

    def _on_xr_pose(self, side: str, msg: PoseStamped):
        """Store latest XR controller position/orientation and compute IK target if engaged."""
        pos = msg.pose.position
        ori = msg.pose.orientation
        self._xr_pose[side] = np.array([pos.x, pos.y, pos.z])
        self._xr_orient[side] = (ori.x, ori.y, ori.z, ori.w)

        hand = self._hands[side]
        if not hand.engaged or hand.xr_reference is None:
            self.get_logger().debug(
                f'{side} pose received but not engaged', throttle_duration_sec=1.0,
            )
            return

        # Delta in XR space
        delta_xr = self._xr_pose[side] - hand.xr_reference

        # Transform to robot frame and scale
        delta_robot = self._axis_map @ delta_xr * self._scale

        # New target = start + delta
        hand.current_target = hand.ee_start + delta_robot

        self.get_logger().info(
            f'{side} target: [{hand.current_target[0]:.3f}, '
            f'{hand.current_target[1]:.3f}, {hand.current_target[2]:.3f}]',
            throttle_duration_sec=0.5,
        )

        # Publish with actual controller orientation
        target_msg = PoseStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'base'
        target_msg.pose.position.x = float(hand.current_target[0])
        target_msg.pose.position.y = float(hand.current_target[1])
        target_msg.pose.position.z = float(hand.current_target[2])
        target_msg.pose.orientation.x = ori.x
        target_msg.pose.orientation.y = ori.y
        target_msg.pose.orientation.z = ori.z
        target_msg.pose.orientation.w = ori.w
        self._ik_pubs[side].publish(target_msg)

    def _on_joy(self, side: str, msg: Joy):
        """Handle trigger press/release for engage/clutch."""
        if self._trigger_btn >= len(msg.buttons):
            return

        pressed = msg.buttons[self._trigger_btn] == 1
        hand = self._hands[side]

        if pressed and not hand.engaged:
            # Engage — capture references
            if self._xr_pose[side] is not None:
                hand.xr_reference = self._xr_pose[side].copy()

                # Use actual EE position from IK FK, not the (possibly
                # unreachable) mathematical target — prevents drift
                if self._actual_ee[side] is not None:
                    hand.ee_start = self._actual_ee[side].copy()
                    hand.current_target = self._actual_ee[side].copy()
                else:
                    hand.ee_start = hand.current_target.copy()

                # Recompute axis map from head pose (adapts to user facing)
                head_map = self._axis_map_from_head()
                if head_map is not None:
                    self._axis_map = head_map

                hand.engaged = True
                self.get_logger().info(
                    f'{side} engaged at ee=[{hand.ee_start[0]:.3f}, '
                    f'{hand.ee_start[1]:.3f}, {hand.ee_start[2]:.3f}]'
                )
        elif not pressed and hand.engaged:
            # Clutch — release
            hand.engaged = False
            self.get_logger().info(f'{side} clutched')


def main(args=None):
    rclpy.init(args=args)
    node = XRBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
