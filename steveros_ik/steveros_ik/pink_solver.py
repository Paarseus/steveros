"""Pink QP-based differential IK solver for steveros.

Wraps Pink's solve_ik with a PinkConfiguration (full/reduced model)
and task management. Follows the NVIDIA Isaac Lab PinkIKController
pattern, adapted for direct use without Isaac Lab dependencies.
"""

from __future__ import annotations

import logging

import numpy as np
import pinocchio as pin
import qpsolvers
from pink import solve_ik
from pink.tasks import DampingTask, FrameTask, PostureTask

from .pink_configuration import PinkConfiguration

logger = logging.getLogger(__name__)


class PinkSolver:
    """Differential IK solver using Pink (QP on Pinocchio).

    Maintains a PinkConfiguration with full/reduced model, manages
    FrameTasks for end-effectors, and provides a single compute() call
    that returns target joint positions.
    """

    def __init__(
        self,
        urdf_path: str,
        controlled_joints: list[str],
        ee_configs: list[dict],
        posture_cost: float = 1e-2,
        damping_cost: float = 0.0,
        solver_backend: str = "daqp",
        solver_damping: float = 0.01,
        safety_break: bool = False,
        max_joint_velocity: float = 2.0,
        state_blend_alpha: float = 0.2,
    ):
        self.configuration = PinkConfiguration(urdf_path, controlled_joints)
        self._max_joint_velocity = max_joint_velocity
        self._state_blend_alpha = state_blend_alpha

        # Select QP solver (prefer requested, fall back to first available)
        if solver_backend in qpsolvers.available_solvers:
            self._solver = solver_backend
        else:
            self._solver = qpsolvers.available_solvers[0]
        self._damping = solver_damping
        self._safety_break = safety_break

        # End-effector frame tasks (targets updated from teleop input)
        self.ee_tasks: dict[str, FrameTask] = {}
        for cfg in ee_configs:
            task = FrameTask(
                cfg["frame"],
                position_cost=cfg.get("position_cost", 1.0),
                orientation_cost=cfg.get("orientation_cost", 0.0),
                lm_damping=cfg.get("lm_damping", 1.0),
            )
            task.set_target_from_configuration(self.configuration)
            self.ee_tasks[cfg["name"]] = task

        # Fixed tasks (posture regularization)
        self._fixed_tasks: list = []

        if posture_cost > 0:
            posture_task = PostureTask(cost=posture_cost)
            posture_task.set_target_from_configuration(self.configuration)
            self._fixed_tasks.append(posture_task)

        if damping_cost > 0:
            self._fixed_tasks.append(DampingTask(cost=damping_cost))

        self._initialized = False

    def update_state(self, joint_positions: dict[str, float]) -> None:
        """Sync solver state from actual robot joint positions.

        On first call, initializes all joints from joint_states.
        On subsequent calls, non-controlled joints (e.g. legs) are set
        directly from joint_states. Controlled joints are alpha-blended
        between the solver's integrated state and actual feedback to
        prevent drift while avoiding feedback oscillation.

        Args:
            joint_positions: All joint positions from /joint_states (name -> value).
        """
        if not self._initialized:
            # First call: initialize everything from actual state
            q = np.array([
                joint_positions.get(name, 0.0)
                for name in self.configuration.all_joint_names
            ])
            self.configuration.update(q)
            self._initialized = True
        else:
            controlled_set = set(self.configuration.controlled_joint_names)
            full_q = self.configuration.full_q.copy()
            alpha = self._state_blend_alpha
            for i, name in enumerate(self.configuration.all_joint_names):
                actual = joint_positions.get(name)
                if actual is None:
                    continue
                if name not in controlled_set:
                    full_q[i] = actual
                else:
                    full_q[i] = (1.0 - alpha) * full_q[i] + alpha * actual
            self.configuration.full_q = full_q
            # Re-run FK on full model with updated non-controlled joints
            pin.forwardKinematics(
                self.configuration.full_model,
                self.configuration.full_data,
                self.configuration.full_q,
            )
            pin.updateFramePlacements(
                self.configuration.full_model,
                self.configuration.full_data,
            )

    def set_ee_target_se3(self, name: str, target: 'pin.SE3') -> None:
        """Set an end-effector target as a Pinocchio SE3 object.

        Args:
            name: EE task name (e.g. "right_hand").
            target: Target pose as pin.SE3.
        """
        if name in self.ee_tasks:
            self.ee_tasks[name].set_target(target)

    def set_ee_target_matrix(self, name: str, T: np.ndarray) -> None:
        """Set an end-effector target as a 4x4 homogeneous matrix.

        Args:
            name: EE task name.
            T: 4x4 transformation matrix.
        """
        se3 = pin.SE3(T[:3, :3], T[:3, 3])
        self.set_ee_target_se3(name, se3)

    def get_ee_pose(self, name: str) -> 'pin.SE3':
        """Get current end-effector pose from FK.

        Args:
            name: EE task name.

        Returns:
            Current SE3 pose of the end-effector frame.
        """
        frame = self.ee_tasks[name].frame
        return self.configuration.get_transform_frame_to_world(frame)

    @property
    def is_initialized(self) -> bool:
        """Whether joint state has been received at least once."""
        return self._initialized

    def compute(self, dt: float) -> dict[str, float]:
        """Run one IK solve step and return target joint positions.

        Call update_state() before this to sync with actual robot state.
        Call set_ee_target_*() to update end-effector goals.

        Args:
            dt: Timestep in seconds.

        Returns:
            Dict of controlled joint name -> target position (rad).
            On solver failure, returns current positions unchanged.
        """
        all_tasks = list(self.ee_tasks.values()) + self._fixed_tasks

        try:
            velocity = solve_ik(
                self.configuration,
                all_tasks,
                dt,
                solver=self._solver,
                damping=self._damping,
                safety_break=self._safety_break,
            )
            # Clamp joint velocities for safe, smooth motion
            velocity = np.clip(
                velocity, -self._max_joint_velocity, self._max_joint_velocity,
            )
            self.configuration.integrate_inplace(velocity, dt)
        except Exception as e:
            logger.warning("IK solve failed: %s", e)

        return {
            name: float(self.configuration.q[i])
            for i, name in enumerate(self.configuration.controlled_joint_names)
        }
