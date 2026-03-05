"""Bridge node: MediaPipe pose world landmarks -> arm joint trajectories.

Subscribes to PoseLandmarks (with world_landmarks in meters, hip-centered),
computes 5-DOF joint angles per arm via limb vectors, and publishes
JointTrajectory messages for the left and right arm controllers.

Filtering pipeline (per arm):
  1. Compute raw joint angles from landmarks
  2. Clip to URDF joint limits
  3. Velocity clamp (reject physically impossible jumps)
  4. One Euro Filter (adaptive smoothing: low jitter at rest, low lag in motion)
  5. Dead zone (suppress micro-oscillations below threshold)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
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

# Maximum joint velocity (rad/s) - protects hardware from tracking jumps
MAX_JOINT_VELOCITY = [
    6.0,   # shoulder pitch
    6.0,   # shoulder roll
    4.0,   # shoulder yaw  (more conservative: depth-derived, noisier)
    6.0,   # elbow
    6.0,   # wrist
]

# Default home position (arms hanging straight down) for tracking-loss decay
RIGHT_HOME = [0.0, 0.0, 0.0, 0.0, 0.0]
LEFT_HOME = [0.0, 0.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# One Euro Filter - adaptive low-pass filter
# ---------------------------------------------------------------------------
class OneEuroFilter:
    """Speed-adaptive low-pass filter for real-time signal smoothing.

    At low speeds: heavy smoothing (controlled by min_cutoff) to eliminate jitter.
    At high speeds: light smoothing (controlled by beta) to reduce lag.
    """

    def __init__(self, freq=30.0, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    def _smoothing_factor(self, cutoff):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        te = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def __call__(self, x, t=None):
        if self.x_prev is None:
            self.x_prev = x
            self.dx_prev = 0.0
            self.t_prev = t
            return x

        if t is not None and self.t_prev is not None:
            dt = t - self.t_prev
            if dt > 0:
                self.freq = 1.0 / dt
            self.t_prev = t

        # Filter the derivative
        dx = (x - self.x_prev) * self.freq
        a_d = self._smoothing_factor(self.d_cutoff)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev

        # Adaptive cutoff based on speed
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._smoothing_factor(cutoff)
        x_hat = a * x + (1.0 - a) * self.x_prev

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat

    def reset(self):
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None


class MultiChannelOneEuroFilter:
    """One Euro Filter applied independently to N channels (e.g. 5 joint angles).

    Supports per-channel min_cutoff for axis-specific filtering strength.
    """

    def __init__(self, n, freq=30.0, min_cutoff=1.0, beta=0.007, d_cutoff=1.0,
                 per_channel_min_cutoff=None):
        if per_channel_min_cutoff is not None:
            self.filters = [
                OneEuroFilter(freq, mc, beta, d_cutoff)
                for mc in per_channel_min_cutoff
            ]
        else:
            self.filters = [
                OneEuroFilter(freq, min_cutoff, beta, d_cutoff)
                for _ in range(n)
            ]

    def __call__(self, values, t=None):
        return [f(v, t) for f, v in zip(self.filters, values)]

    def update_params(self, min_cutoff=None, beta=None):
        for f in self.filters:
            if min_cutoff is not None:
                f.min_cutoff = min_cutoff
            if beta is not None:
                f.beta = beta

    def reset(self):
        for f in self.filters:
            f.reset()


# ---------------------------------------------------------------------------
# Tracking loss handler
# ---------------------------------------------------------------------------
class TrackingLossHandler:
    """Graceful handling of MediaPipe tracking loss.

    When tracking is lost:
      1. Hold last valid position for hold_frames
      2. Gradually blend toward home position over decay_frames
    When tracking resumes, output is immediately valid again.
    """

    def __init__(self, home_position, hold_frames=10, decay_frames=60):
        self.home = list(home_position)
        self.hold_frames = hold_frames
        self.decay_frames = decay_frames
        self.last_valid = None
        self.frames_lost = 0

    def update(self, angles, is_valid):
        if is_valid:
            self.last_valid = list(angles)
            self.frames_lost = 0
            return angles, True

        self.frames_lost += 1
        if self.last_valid is None:
            return self.home, False

        if self.frames_lost <= self.hold_frames:
            return self.last_valid, False

        # Gradually blend toward home
        t = min(1.0, (self.frames_lost - self.hold_frames) / self.decay_frames)
        blended = [h * t + lv * (1.0 - t)
                    for h, lv in zip(self.home, self.last_valid)]
        return blended, False


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
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
        self.declare_parameter('trajectory_duration', 0.1)

        # One Euro Filter parameters (tunable at runtime via ros2 param set)
        self.declare_parameter('filter_min_cutoff', 1.0)
        self.declare_parameter('filter_beta', 0.007)
        self.declare_parameter('filter_d_cutoff', 1.0)
        # Heavier filtering on shoulder yaw (depth-derived, noisier)
        self.declare_parameter('filter_yaw_min_cutoff', 0.5)

        # Dead zone threshold (rad) - suppress micro-oscillations
        self.declare_parameter('dead_zone', 0.015)

        # Tracking loss parameters
        self.declare_parameter('tracking_hold_frames', 10)
        self.declare_parameter('tracking_decay_frames', 60)

        # Keep legacy ema_alpha param declared (unused, for backwards compat)
        self.declare_parameter('ema_alpha', 0.3)

        pose_topic = str(self.get_parameter('pose_topic').value)
        right_topic = str(self.get_parameter('right_arm_topic').value)
        left_topic = str(self.get_parameter('left_arm_topic').value)
        self.traj_dur = float(self.get_parameter('trajectory_duration').value)

        min_cutoff = float(self.get_parameter('filter_min_cutoff').value)
        beta = float(self.get_parameter('filter_beta').value)
        d_cutoff = float(self.get_parameter('filter_d_cutoff').value)
        yaw_min_cutoff = float(self.get_parameter('filter_yaw_min_cutoff').value)

        self.dead_zone = float(self.get_parameter('dead_zone').value)

        hold_frames = int(self.get_parameter('tracking_hold_frames').value)
        decay_frames = int(self.get_parameter('tracking_decay_frames').value)

        # Per-channel min_cutoff: [pitch, roll, yaw, elbow, wrist]
        # Yaw gets heavier filtering since it depends on depth (weakest axis)
        per_channel_mc = [min_cutoff, min_cutoff, yaw_min_cutoff,
                          min_cutoff, min_cutoff]

        # One Euro Filters (replace EMA)
        self._right_filter = MultiChannelOneEuroFilter(
            5, min_cutoff=min_cutoff, beta=beta, d_cutoff=d_cutoff,
            per_channel_min_cutoff=per_channel_mc)
        self._left_filter = MultiChannelOneEuroFilter(
            5, min_cutoff=min_cutoff, beta=beta, d_cutoff=d_cutoff,
            per_channel_min_cutoff=per_channel_mc)

        # Tracking loss handlers
        self._right_tracking = TrackingLossHandler(
            RIGHT_HOME, hold_frames, decay_frames)
        self._left_tracking = TrackingLossHandler(
            LEFT_HOME, hold_frames, decay_frames)

        # State for velocity clamping
        self._right_prev = None
        self._left_prev = None
        self._prev_time = None

        # State for dead zone
        self._right_output = None
        self._left_output = None

        # Publishers
        self.pub_right = self.create_publisher(JointTrajectory, right_topic, 10)
        self.pub_left = self.create_publisher(JointTrajectory, left_topic, 10)

        # Subscriber
        self.sub = self.create_subscription(
            PoseLandmarks, pose_topic, self.on_pose, 10)

        # Dynamic parameter callback for runtime tuning
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'Pose-to-joints bridge started '
            f'(filter: OneEuro min_cutoff={min_cutoff}, beta={beta}, '
            f'yaw_min_cutoff={yaw_min_cutoff}, '
            f'dead_zone={self.dead_zone:.3f} rad, '
            f'dur={self.traj_dur}s)')

    def _on_param_change(self, params):
        """Handle runtime parameter changes (ros2 param set)."""
        for p in params:
            if p.name == 'filter_min_cutoff':
                for f in (self._right_filter, self._left_filter):
                    f.update_params(min_cutoff=p.value)
                self.get_logger().info(f'Updated filter min_cutoff={p.value}')
            elif p.name == 'filter_beta':
                for f in (self._right_filter, self._left_filter):
                    f.update_params(beta=p.value)
                self.get_logger().info(f'Updated filter beta={p.value}')
            elif p.name == 'dead_zone':
                self.dead_zone = float(p.value)
                self.get_logger().info(f'Updated dead_zone={p.value}')
        return SetParametersResult(successful=True)

    def on_pose(self, msg: PoseLandmarks):
        wl = msg.world_landmarks
        has_pose = len(wl) >= 33

        # Extract timestamp for filter dt computation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Compute dt for velocity clamping
        dt = None
        if self._prev_time is not None and t > self._prev_time:
            dt = t - self._prev_time
        self._prev_time = t

        if has_pose:
            # --- Right arm ---
            right_raw = _compute_arm_angles(
                wl[RIGHT_SHOULDER], wl[RIGHT_ELBOW], wl[RIGHT_WRIST],
                wl[LEFT_SHOULDER], wl[RIGHT_SHOULDER],
                wl[LEFT_HIP], wl[RIGHT_HIP],
                is_right=True)
            right_raw = [
                np.clip(v, lo, hi)
                for v, (lo, hi) in zip(right_raw, RIGHT_LIMITS)]
            right_raw = self._velocity_clamp(right_raw, self._right_prev, dt)
            self._right_prev = list(right_raw)
            right_smooth = self._right_filter(right_raw, t)
            right_out = self._apply_dead_zone(right_smooth, self._right_output)
            self._right_output = right_out
            right_final, _ = self._right_tracking.update(right_out, True)
            self._publish(self.pub_right, RIGHT_JOINT_NAMES, right_final)

            # --- Left arm ---
            left_raw = _compute_arm_angles(
                wl[LEFT_SHOULDER], wl[LEFT_ELBOW], wl[LEFT_WRIST],
                wl[LEFT_SHOULDER], wl[RIGHT_SHOULDER],
                wl[LEFT_HIP], wl[RIGHT_HIP],
                is_right=False)
            left_raw = [
                np.clip(v, lo, hi)
                for v, (lo, hi) in zip(left_raw, LEFT_LIMITS)]
            left_raw = self._velocity_clamp(left_raw, self._left_prev, dt)
            self._left_prev = list(left_raw)
            left_smooth = self._left_filter(left_raw, t)
            left_out = self._apply_dead_zone(left_smooth, self._left_output)
            self._left_output = left_out
            left_final, _ = self._left_tracking.update(left_out, True)
            self._publish(self.pub_left, LEFT_JOINT_NAMES, left_final)
        else:
            # Tracking lost - use hold-and-decay
            right_final, _ = self._right_tracking.update(None, False)
            self._publish(self.pub_right, RIGHT_JOINT_NAMES, right_final)
            left_final, _ = self._left_tracking.update(None, False)
            self._publish(self.pub_left, LEFT_JOINT_NAMES, left_final)

    def _velocity_clamp(self, raw, prev, dt):
        """Reject physically impossible joint velocity spikes."""
        if prev is None or dt is None or dt <= 0:
            return raw
        clamped = []
        for r, p, max_vel in zip(raw, prev, MAX_JOINT_VELOCITY):
            max_delta = max_vel * dt
            delta = np.clip(r - p, -max_delta, max_delta)
            clamped.append(p + delta)
        return clamped

    def _apply_dead_zone(self, filtered, prev_output):
        """Suppress changes smaller than dead_zone threshold."""
        if prev_output is None:
            return list(filtered)
        out = []
        for f, p in zip(filtered, prev_output):
            if abs(f - p) < self.dead_zone:
                out.append(p)
            else:
                out.append(f)
        return out

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
