"""ROS 2 IK node: PoseStamped targets in -> position commands out.

Uses Pink QP-based differential IK with a reduced Pinocchio model.
Publishes to ForwardCommandController for jitter-free real-time streaming.
"""

import tempfile

import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from steveros_ik.pink_solver import PinkSolver


def pose_msg_to_se3(pose) -> 'pin.SE3':
    """Convert geometry_msgs/Pose to Pinocchio SE3."""
    quat = pin.Quaternion(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    )
    return pin.SE3(
        quat.toRotationMatrix(),
        np.array([pose.position.x, pose.position.y, pose.position.z]),
    )


class EETaskHandle:
    """Tracks one end-effector: its target, subscriber, and output publisher."""

    def __init__(self, name: str, frame: str, output_joints: list[str],
                 target_topic: str, command_topic: str):
        self.name = name
        self.frame = frame
        self.output_joints = output_joints
        self.target_topic = target_topic
        self.command_topic = command_topic
        self.has_target = False
        self.target_fresh = False


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        # --- Solver parameters ---
        self.declare_parameter('solver_backend', 'daqp')
        self.declare_parameter('solver_damping', 0.01)
        self.declare_parameter('safety_break', False)
        self.declare_parameter('servo_rate', 100.0)
        self.declare_parameter('posture_cost', 1e-2)
        self.declare_parameter('damping_cost', 0.0)
        self.declare_parameter('max_joint_velocity', 2.0)
        self.declare_parameter('state_blend_alpha', 0.2)

        # --- Controlled joints (all joints solved by IK) ---
        self.declare_parameter('controlled_joints', [
            'dof_right_shoulder_pitch_03',
            'dof_right_shoulder_roll_03',
            'dof_right_shoulder_yaw_02',
            'dof_right_elbow_02',
            'dof_right_wrist_00',
            'dof_left_shoulder_pitch_03',
            'dof_left_shoulder_roll_03',
            'dof_left_shoulder_yaw_02',
            'dof_left_elbow_02',
            'dof_left_wrist_00',
        ])

        # --- End-effector task definitions (parallel arrays) ---
        self.declare_parameter('ee_names', ['right_hand', 'left_hand'])
        self.declare_parameter('ee_frames', ['PRT0001', 'PRT0001_2'])
        self.declare_parameter('ee_position_costs', [1.0, 1.0])
        self.declare_parameter('ee_orientation_costs', [0.0, 0.0])
        self.declare_parameter('ee_lm_dampings', [1e-4, 1e-4])
        self.declare_parameter('ee_target_topics', [
            '/ik/right_hand_target', '/ik/left_hand_target',
        ])
        self.declare_parameter('ee_command_topics', [
            '/right_arm_forward_controller/commands',
            '/left_arm_forward_controller/commands',
        ])

        # --- Output joint groups (which joints go to which controller) ---
        self.declare_parameter('right_hand_joints', [
            'dof_right_shoulder_pitch_03',
            'dof_right_shoulder_roll_03',
            'dof_right_shoulder_yaw_02',
            'dof_right_elbow_02',
            'dof_right_wrist_00',
        ])
        self.declare_parameter('left_hand_joints', [
            'dof_left_shoulder_pitch_03',
            'dof_left_shoulder_roll_03',
            'dof_left_shoulder_yaw_02',
            'dof_left_elbow_02',
            'dof_left_wrist_00',
        ])

        # Read parameters
        self._servo_rate = self.get_parameter('servo_rate').value

        # Get URDF from /robot_description
        urdf_path = self._wait_for_urdf()

        # Build EE task configs
        ee_names = list(self.get_parameter('ee_names').value)
        ee_frames = list(self.get_parameter('ee_frames').value)
        ee_pos_costs = list(self.get_parameter('ee_position_costs').value)
        ee_ori_costs = list(self.get_parameter('ee_orientation_costs').value)
        ee_lm = list(self.get_parameter('ee_lm_dampings').value)
        ee_target_topics = list(self.get_parameter('ee_target_topics').value)
        ee_cmd_topics = list(self.get_parameter('ee_command_topics').value)

        ee_configs = []
        for i, name in enumerate(ee_names):
            ee_configs.append({
                'name': name,
                'frame': ee_frames[i],
                'position_cost': ee_pos_costs[i],
                'orientation_cost': ee_ori_costs[i],
                'lm_damping': ee_lm[i],
            })

        # Create solver
        self.solver = PinkSolver(
            urdf_path=urdf_path,
            controlled_joints=list(self.get_parameter('controlled_joints').value),
            ee_configs=ee_configs,
            posture_cost=self.get_parameter('posture_cost').value,
            damping_cost=self.get_parameter('damping_cost').value,
            solver_backend=self.get_parameter('solver_backend').value,
            solver_damping=self.get_parameter('solver_damping').value,
            safety_break=self.get_parameter('safety_break').value,
            max_joint_velocity=self.get_parameter('max_joint_velocity').value,
            state_blend_alpha=self.get_parameter('state_blend_alpha').value,
        )

        # Build EE handles with subscribers and publishers
        self._ee_handles: dict[str, EETaskHandle] = {}
        for i, name in enumerate(ee_names):
            output_joints = list(
                self.get_parameter(f'{name}_joints').value
            )
            handle = EETaskHandle(
                name=name,
                frame=ee_frames[i],
                output_joints=output_joints,
                target_topic=ee_target_topics[i],
                command_topic=ee_cmd_topics[i],
            )
            self._ee_handles[name] = handle

        # Create subscribers for each EE target
        for name, handle in self._ee_handles.items():
            self.create_subscription(
                PoseStamped,
                handle.target_topic,
                lambda msg, n=name: self._on_target_pose(n, msg),
                10,
            )

        # Create Float64MultiArray publishers for forward controllers
        self._cmd_pubs: dict[str, any] = {}
        for name, handle in self._ee_handles.items():
            self._cmd_pubs[name] = self.create_publisher(
                Float64MultiArray, handle.command_topic, 10,
            )

        # Publish actual EE poses (from FK) so the bridge can track reality
        self._ee_actual_pubs: dict[str, any] = {}
        for name, handle in self._ee_handles.items():
            topic = handle.target_topic.replace('_target', '_actual')
            self._ee_actual_pubs[name] = self.create_publisher(
                PoseStamped, topic, 10,
            )

        # Subscribe to joint states
        self._joint_positions: dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

        # Timer-based solve loop
        dt = 1.0 / self._servo_rate
        self._timer = self.create_timer(dt, self._solve_and_publish)

        self.get_logger().info(
            f'IK node ready: {len(ee_names)} EE tasks, '
            f'{len(self.solver.configuration.controlled_joint_names)} controlled joints, '
            f'solver={self.solver._solver}, rate={self._servo_rate}Hz'
        )

    def _wait_for_urdf(self) -> str:
        """Block until /robot_description is available, save to temp file."""
        urdf_text = None

        def cb(msg):
            nonlocal urdf_text
            urdf_text = msg.data

        sub = self.create_subscription(
            String, '/robot_description', cb,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.get_logger().info('Waiting for /robot_description...')
        while urdf_text is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)

        tmp = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
        tmp.write(urdf_text.encode('utf-8'))
        tmp.close()
        self.get_logger().info(f'URDF loaded ({len(urdf_text)} bytes)')
        return tmp.name

    def _on_joint_states(self, msg: JointState):
        """Store latest joint positions from the robot."""
        if not msg.name or not msg.position:
            return
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

    def _on_target_pose(self, ee_name: str, msg: PoseStamped):
        """Update an end-effector target pose."""
        target = pose_msg_to_se3(msg.pose)
        self.solver.set_ee_target_se3(ee_name, target)
        self._ee_handles[ee_name].has_target = True
        self._ee_handles[ee_name].target_fresh = True

    def _solve_and_publish(self):
        """Timer callback: solve IK and publish position commands."""
        if not self._joint_positions:
            return
        if not any(h.has_target for h in self._ee_handles.values()):
            return

        # Sync solver (only non-controlled joints after first call)
        self.solver.update_state(self._joint_positions)

        # Solve IK
        dt = 1.0 / self._servo_rate
        target_positions = self.solver.compute(dt)

        # Publish position commands and actual EE poses
        for name, handle in self._ee_handles.items():
            # Always publish actual EE pose (from FK) for bridge feedback
            ee_se3 = self.solver.get_ee_pose(name)
            actual_msg = PoseStamped()
            actual_msg.header.stamp = self.get_clock().now().to_msg()
            actual_msg.header.frame_id = 'base'
            actual_msg.pose.position.x = float(ee_se3.translation[0])
            actual_msg.pose.position.y = float(ee_se3.translation[1])
            actual_msg.pose.position.z = float(ee_se3.translation[2])
            quat = pin.Quaternion(ee_se3.rotation)
            actual_msg.pose.orientation.x = float(quat.coeffs()[0])
            actual_msg.pose.orientation.y = float(quat.coeffs()[1])
            actual_msg.pose.orientation.z = float(quat.coeffs()[2])
            actual_msg.pose.orientation.w = float(quat.coeffs()[3])
            self._ee_actual_pubs[name].publish(actual_msg)

            if not handle.has_target:
                continue

            msg = Float64MultiArray()
            msg.data = [
                target_positions.get(joint, 0.0)
                for joint in handle.output_joints
            ]
            self._cmd_pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
