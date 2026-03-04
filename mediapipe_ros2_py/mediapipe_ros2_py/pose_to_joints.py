"""Bridge node: MediaPipe pose world landmarks -> arm joint trajectories.

Subscribes to PoseLandmarks (with world_landmarks in meters, hip-centered),
computes 5-DOF joint angles per arm via limb vectors, and publishes
JointTrajectory messages for the left and right arm controllers.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from mediapipe_ros2_interfaces.msg import PoseLandmarks

# ---------------------------------------------------------------------------
# MediaPipe BlazePose landmark indices (33 total)
# ---------------------------------------------------------------------------
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_ELBOW = 13
RIGHT_ELBOW = 14
LEFT_WRIST = 15
RIGHT_WRIST = 16
LEFT_HIP = 23
RIGHT_HIP = 24

# ---------------------------------------------------------------------------
# Robot joint names (5 DOF per arm)
# ---------------------------------------------------------------------------
RIGHT_JOINT_NAMES = [
    'dof_right_shoulder_pitch_03',
    'dof_right_shoulder_roll_03',
    'dof_right_shoulder_yaw_02',
    'dof_right_elbow_02',
    'dof_right_wrist_00',
]
LEFT_JOINT_NAMES = [
    'dof_left_shoulder_pitch_03',
    'dof_left_shoulder_roll_03',
    'dof_left_shoulder_yaw_02',
    'dof_left_elbow_02',
    'dof_left_wrist_00',
]

# Joint limits [min, max] in radians (from URDF)
RIGHT_LIMITS = [
    (-3.490658, 1.047198),   # shoulder pitch
    (-1.658063, 0.436332),   # shoulder roll
    (-1.671886, 1.671886),   # shoulder yaw
    (0.0, 2.478368),         # elbow
    (-1.378810, 1.378810),   # wrist
]
LEFT_LIMITS = [
    (-1.047198, 3.490658),   # shoulder pitch
    (-0.436332, 1.658063),   # shoulder roll
    (-1.671886, 1.671886),   # shoulder yaw
    (-2.478368, 0.0),        # elbow
    (-1.378810, 1.378810),   # wrist
]

def _pt(landmark):
    """Extract xyz from a geometry_msgs/Point as numpy array."""
    return np.array([landmark.x, landmark.y, landmark.z])


def _vec(a, b):
    """Vector from point a to point b as numpy array."""
    return np.array([b.x - a.x, b.y - a.y, b.z - a.z])


def _unit(v):
    """Normalise vector; returns zero vector if length is ~0."""
    n = np.linalg.norm(v)
    return v / n if n > 1e-8 else np.zeros(3)


def _angle_between(u, v):
    """Unsigned angle in radians between two vectors."""
    u_n = _unit(u)
    v_n = _unit(v)
    dot = np.clip(np.dot(u_n, v_n), -1.0, 1.0)
    return math.acos(dot)


def _build_torso_frame(left_shoulder, right_shoulder, left_hip, right_hip):
    """Build an orthonormal torso frame [right, up, forward] from landmarks.

    Returns a 3x3 rotation matrix R whose columns are the torso axes
    expressed in the MediaPipe world frame.  R.T @ v transforms a
    MediaPipe-frame vector into torso-frame components [right, up, fwd].
    """
    ls = _pt(left_shoulder)
    rs = _pt(right_shoulder)
    lh = _pt(left_hip)
    rh = _pt(right_hip)

    # Up: hip midpoint -> shoulder midpoint
    t_up = _unit((ls + rs) / 2.0 - (lh + rh) / 2.0)

    # Right: left_shoulder -> right_shoulder (MediaPipe x = subject's left,
    # so this vector points toward the subject's right = -x direction)
    t_right = rs - ls
    t_right = t_right - np.dot(t_right, t_up) * t_up   # orthogonalise
    t_right = _unit(t_right)

    # Forward: complete right-handed frame (up x right = backward in MP,
    # so we do right cross... no: right x up gives forward when right is
    # toward -x and up is toward -y in MP coords)
    t_fwd = np.cross(t_up, t_right)

    return np.column_stack([t_right, t_up, t_fwd])


def _compute_arm_angles(shoulder, elbow, wrist,
                        left_shoulder, right_shoulder,
                        left_hip, right_hip,
                        is_right: bool):
    """Compute [pitch, roll, yaw, elbow, wrist] from world landmarks.

    Uses a proper torso coordinate frame and intrinsic Euler decomposition
    matching the URDF kinematic chain (pitch about lateral axis first,
    then roll about the forward axis in the pitched frame).

    URDF zero pose = arms hanging straight down.
    Torso frame: index 0 = right, 1 = up, 2 = forward.

    Right arm URDF conventions (from axis tags):
      pitch axis = torso left  (+x_torso_left = -index0) → positive pitch = backward swing
      roll  axis = (0,0,-1) in child frame → maps to forward → negative roll = abduction
    Left arm mirrors pitch sign; roll axis is (0,0,+1) → positive roll = abduction.
    """
    R = _build_torso_frame(left_shoulder, right_shoulder, left_hip, right_hip)

    # Transform arm vectors into torso frame [right, up, fwd]
    upper = _vec(shoulder, elbow)
    fore = _vec(elbow, wrist)
    upper_t = R.T @ upper
    fore_t = R.T @ fore

    d = _unit(upper_t)

    # --- Pitch & roll via intrinsic Euler decomposition ---
    #
    # At zero position the arm direction is d0 = [0, -1, 0] (straight down).
    # After pitch p about the lateral (right) axis then roll r about the
    # forward axis in the pitched frame:
    #   d = R_fwd(-r) @ R_right(p) @ [0, -1, 0]
    #     = [-cos(p)*sin(r),  -cos(p)*cos(r),  -sin(p)]
    #
    # Solving:
    #   pitch = atan2(-d[2], sqrt(d[0]^2 + d[1]^2))
    #   roll  = atan2(-d[0], -d[1])
    #
    # For the RIGHT arm the URDF pitch axis points in the +left direction
    # (= -right in our frame) with joint axis (0,0,1).  Positive URDF pitch
    # therefore swings the arm backward.  The decomposition above already
    # matches this convention.
    #
    # The URDF right roll axis is (0,0,-1) in the child frame which maps to
    # the forward direction.  Positive URDF roll rotates about -forward,
    # so adduction is positive and abduction is negative.  The decomposition
    # uses rotation about +forward, so we negate roll for the right arm.
    #
    # For the LEFT arm the pitch limits are mirrored and the roll axis is
    # (0,0,+1), so pitch is negated and roll keeps its sign.

    pitch = math.atan2(-d[2], math.sqrt(d[0]**2 + d[1]**2))
    roll = math.atan2(-d[0], -d[1])

    if is_right:
        roll = -roll      # right roll axis (0,0,-1) -> negate
    else:
        pitch = -pitch    # left pitch limits mirrored -> negate

    # --- Elbow flexion ---
    elbow_angle = _angle_between(upper_t, fore_t)
    if not is_right:
        elbow_angle = -elbow_angle   # left URDF range is [-2.478, 0]

    # --- Shoulder yaw (internal/external rotation) ---
    upper_t_n = _unit(upper_t)
    fore_t_n = _unit(fore_t)

    # Project forearm onto plane perpendicular to upper arm
    fore_perp = fore_t_n - np.dot(fore_t_n, upper_t_n) * upper_t_n
    fore_perp_n = _unit(fore_perp)

    # Yaw reference: direction the forearm would point at yaw=0 with the
    # current pitch/roll.  At zero yaw the forearm is in the sagittal plane:
    #   R_fwd(-r) @ R_right(p) @ [0, 0, 1]  (forward at zero)
    p_abs = pitch if is_right else -pitch
    r_abs = -roll if is_right else roll
    cp, sp = math.cos(p_abs), math.sin(p_abs)
    cr, sr = math.cos(r_abs), math.sin(r_abs)
    zero_yaw_fore = np.array([-sr * sp, -cr * sp, cp])

    # Project onto upper-arm perpendicular plane
    ref_zero = zero_yaw_fore - np.dot(zero_yaw_fore, upper_t_n) * upper_t_n
    ref_norm = np.linalg.norm(ref_zero)

    if ref_norm < 0.1 or abs(elbow_angle) < 0.15:
        # Degenerate: arm nearly vertical or elbow nearly straight
        yaw = 0.0
    else:
        ref_zero = ref_zero / ref_norm
        ref_ortho = np.cross(upper_t_n, ref_zero)
        yaw = math.atan2(np.dot(fore_perp_n, ref_ortho),
                         np.dot(fore_perp_n, ref_zero))

    # Left yaw URDF axis is (0,0,-1) vs right (0,0,1) → negate for left
    if not is_right:
        yaw = -yaw

    wrist_angle = 0.0

    return [pitch, roll, yaw, elbow_angle, wrist_angle]


class PoseToJoints(Node):
    def __init__(self):
        super().__init__('pose_to_joints')

        # Parameters
        self.declare_parameter('pose_topic', '/mediapipe/pose/landmarks')
        self.declare_parameter('right_arm_topic',
                               '/right_arm_controller/joint_trajectory')
        self.declare_parameter('left_arm_topic',
                               '/left_arm_controller/joint_trajectory')
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('trajectory_duration', 0.1)

        pose_topic = str(self.get_parameter('pose_topic').value)
        right_topic = str(self.get_parameter('right_arm_topic').value)
        left_topic = str(self.get_parameter('left_arm_topic').value)
        self.alpha = float(self.get_parameter('ema_alpha').value)
        self.traj_dur = float(self.get_parameter('trajectory_duration').value)

        # Publishers
        self.pub_right = self.create_publisher(JointTrajectory, right_topic, 10)
        self.pub_left = self.create_publisher(JointTrajectory, left_topic, 10)

        # Subscriber
        self.sub = self.create_subscription(
            PoseLandmarks, pose_topic, self.on_pose, 10)

        # EMA state
        self._right_smooth = None  # list of 5 floats
        self._left_smooth = None

        self.get_logger().info(
            f'Pose-to-joints bridge started (alpha={self.alpha}, '
            f'dur={self.traj_dur}s)')

    def on_pose(self, msg: PoseLandmarks):
        wl = msg.world_landmarks
        if len(wl) < 33:
            return  # need full BlazePose skeleton

        # --- Right arm ---
        right_raw = _compute_arm_angles(
            wl[RIGHT_SHOULDER], wl[RIGHT_ELBOW], wl[RIGHT_WRIST],
            wl[LEFT_SHOULDER], wl[RIGHT_SHOULDER],
            wl[LEFT_HIP], wl[RIGHT_HIP],
            is_right=True)
        right_raw = [
            np.clip(v, lo, hi)
            for v, (lo, hi) in zip(right_raw, RIGHT_LIMITS)]
        right_smooth = self._ema(right_raw, self._right_smooth)
        self._right_smooth = right_smooth
        self._publish(self.pub_right, RIGHT_JOINT_NAMES, right_smooth)

        # --- Left arm ---
        left_raw = _compute_arm_angles(
            wl[LEFT_SHOULDER], wl[LEFT_ELBOW], wl[LEFT_WRIST],
            wl[LEFT_SHOULDER], wl[RIGHT_SHOULDER],
            wl[LEFT_HIP], wl[RIGHT_HIP],
            is_right=False)
        left_raw = [
            np.clip(v, lo, hi)
            for v, (lo, hi) in zip(left_raw, LEFT_LIMITS)]
        left_smooth = self._ema(left_raw, self._left_smooth)
        self._left_smooth = left_smooth
        self._publish(self.pub_left, LEFT_JOINT_NAMES, left_smooth)

    def _ema(self, raw, prev):
        """Exponential moving average filter."""
        if prev is None:
            return list(raw)
        a = self.alpha
        return [a * r + (1.0 - a) * p for r, p in zip(raw, prev)]

    def _publish(self, pub, joint_names, positions):
        """Build and publish a JointTrajectory message."""
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        sec = int(self.traj_dur)
        nanosec = int((self.traj_dur - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points.append(pt)
        pub.publish(traj)


def main():
    rclpy.init()
    rclpy.spin(PoseToJoints())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
