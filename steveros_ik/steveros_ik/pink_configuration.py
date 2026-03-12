"""Extended Pink Configuration with full/reduced model support.

Adapted from NVIDIA Isaac Lab's PinkKinematicsConfiguration.
Maintains both a full robot model and a reduced (controlled-only) model
for selective joint control with accurate full-body FK.
"""

import numpy as np
import pinocchio as pin
from pink.configuration import Configuration
from pink.exceptions import FrameNotFound


class PinkConfiguration(Configuration):
    """Configuration maintaining both a controlled (reduced) and full Pinocchio model.

    The controlled model only contains joints being actively solved by IK.
    The full model contains all joints so that FK and Jacobians account for
    non-controlled joint positions (e.g. legs while solving arm IK).
    """

    def __init__(
        self,
        urdf_path: str,
        controlled_joint_names: list[str],
    ):
        self._controlled_joint_names = list(controlled_joint_names)

        # Build full model from URDF (kinematic model only, no meshes needed)
        self.full_model = pin.buildModelFromUrdf(urdf_path)
        self.full_data = self.full_model.createData()
        self.full_q = pin.neutral(self.full_model)

        # All joint names excluding the universe joint, in Pinocchio tree order
        self._all_joint_names = [
            self.full_model.names[i]
            for i in range(1, self.full_model.njoints)
        ]

        # Indices of controlled joints within the full joint list
        self._controlled_joint_indices = [
            i for i, name in enumerate(self._all_joint_names)
            if name in self._controlled_joint_names
        ]

        if len(self._controlled_joint_indices) != len(self._controlled_joint_names):
            missing = set(self._controlled_joint_names) - set(self._all_joint_names)
            raise ValueError(f"Joints not found in URDF: {missing}")

        # Build reduced model by locking non-controlled joints
        joints_to_lock = [
            self.full_model.getJointId(name)
            for name in self._all_joint_names
            if name not in self._controlled_joint_names
        ]

        if not joints_to_lock:
            controlled_model = self.full_model
            controlled_data = self.full_data
            controlled_q = self.full_q.copy()
        else:
            controlled_model = pin.buildReducedModel(
                self.full_model, joints_to_lock, self.full_q,
            )
            controlled_data = controlled_model.createData()
            controlled_q = np.array([
                self.full_q[self.full_model.joints[
                    self.full_model.getJointId(self._all_joint_names[i])
                ].idx_q]
                for i in self._controlled_joint_indices
            ])

        # Pink Configuration operates on the reduced model
        super().__init__(
            controlled_model, controlled_data, controlled_q,
            copy_data=True, forward_kinematics=True,
        )

    def update(self, q: np.ndarray | None = None) -> None:
        """Update configuration, running FK on both full and reduced models.

        Args:
            q: Joint positions. Either full-size (all joints) or
               controlled-size (controlled joints only).
        """
        if q is not None:
            n_all = len(self._all_joint_names)
            n_ctrl = len(self._controlled_joint_indices)

            if len(q) == n_all:
                # Full q vector: extract controlled subset for reduced model
                controlled_q = np.array([q[i] for i in self._controlled_joint_indices])
                super().update(controlled_q)
                self.full_q = q.copy()
            elif len(q) == n_ctrl:
                # Controlled q vector (from integrate_inplace)
                super().update(q)
                full_q = self.full_q.copy()
                for i, idx in enumerate(self._controlled_joint_indices):
                    full_q[idx] = q[i]
                self.full_q = full_q
            else:
                raise ValueError(
                    f"q must have {n_all} (all) or {n_ctrl} (controlled) elements, "
                    f"got {len(q)}"
                )
        else:
            super().update()

        # Always update full model FK so Jacobians and transforms are correct
        pin.forwardKinematics(self.full_model, self.full_data, self.full_q)
        pin.computeJointJacobians(self.full_model, self.full_data, self.full_q)
        pin.updateFramePlacements(self.full_model, self.full_data)

    def get_frame_jacobian(self, frame: str) -> np.ndarray:
        """Compute frame Jacobian from full model, sliced to controlled columns.

        Args:
            frame: Frame name from the URDF.

        Returns:
            6 x n_controlled Jacobian matrix in LOCAL reference frame.
        """
        if not self.full_model.existFrame(frame):
            raise FrameNotFound(frame, self.full_model.frames)
        frame_id = self.full_model.getFrameId(frame)
        J = pin.getFrameJacobian(
            self.full_model, self.full_data, frame_id, pin.ReferenceFrame.LOCAL,
        )
        return J[:, self._controlled_joint_indices]

    def get_transform_frame_to_world(self, frame: str) -> pin.SE3:
        """Get frame pose from the full model (accounts for locked joint positions).

        Args:
            frame: Frame name from the URDF.

        Returns:
            SE3 transform from frame to world.
        """
        if not self.full_model.existFrame(frame):
            raise FrameNotFound(frame, self.full_model.frames)
        frame_id = self.full_model.getFrameId(frame)
        try:
            return self.full_data.oMf[frame_id].copy()
        except IndexError as e:
            raise FrameNotFound(frame, self.full_model.frames) from e

    def get_transform(self, frame: str, base_frame: str) -> pin.SE3:
        """Get transform of frame relative to base_frame.

        Args:
            frame: Target frame name.
            base_frame: Reference frame name.

        Returns:
            SE3 transform from frame to base_frame.
        """
        T_frame = self.get_transform_frame_to_world(frame)
        T_base = self.get_transform_frame_to_world(base_frame)
        return T_base.actInv(T_frame)

    def check_limits(self, tol: float = 1e-6, safety_break: bool = True) -> None:
        """Only enforce limits when safety_break is enabled."""
        if safety_break:
            super().check_limits(tol, safety_break)

    @property
    def controlled_joint_names(self) -> list[str]:
        """Controlled joint names in Pinocchio model order."""
        return [self._all_joint_names[i] for i in self._controlled_joint_indices]

    @property
    def all_joint_names(self) -> list[str]:
        """All joint names in Pinocchio model order."""
        return list(self._all_joint_names)
