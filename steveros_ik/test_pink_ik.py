"""Tests for steveros_ik Pink IK implementation."""

import numpy as np
import pinocchio as pin
import pytest

from steveros_ik.pink_configuration import PinkConfiguration
from steveros_ik.pink_solver import PinkSolver

URDF = '/home/nemo/Desktop/steve/steveros/steveros_description/urdf/kbot.urdf'

ALL_JOINTS = [
    'dof_left_hip_pitch_04', 'dof_left_hip_roll_03', 'dof_left_hip_yaw_03',
    'dof_left_knee_04', 'dof_left_ankle_02',
    'dof_left_shoulder_pitch_03', 'dof_left_shoulder_roll_03',
    'dof_left_shoulder_yaw_02', 'dof_left_elbow_02', 'dof_left_wrist_00',
    'dof_right_hip_pitch_04', 'dof_right_hip_roll_03', 'dof_right_hip_yaw_03',
    'dof_right_knee_04', 'dof_right_ankle_02',
    'dof_right_shoulder_pitch_03', 'dof_right_shoulder_roll_03',
    'dof_right_shoulder_yaw_02', 'dof_right_elbow_02', 'dof_right_wrist_00',
]

ARM_JOINTS = [
    'dof_right_shoulder_pitch_03', 'dof_right_shoulder_roll_03',
    'dof_right_shoulder_yaw_02', 'dof_right_elbow_02', 'dof_right_wrist_00',
    'dof_left_shoulder_pitch_03', 'dof_left_shoulder_roll_03',
    'dof_left_shoulder_yaw_02', 'dof_left_elbow_02', 'dof_left_wrist_00',
]

RIGHT_ARM = [
    'dof_right_shoulder_pitch_03', 'dof_right_shoulder_roll_03',
    'dof_right_shoulder_yaw_02', 'dof_right_elbow_02', 'dof_right_wrist_00',
]

LEFT_ARM = [
    'dof_left_shoulder_pitch_03', 'dof_left_shoulder_roll_03',
    'dof_left_shoulder_yaw_02', 'dof_left_elbow_02', 'dof_left_wrist_00',
]

EE_CONFIGS = [
    {'name': 'right_hand', 'frame': 'PRT0001', 'position_cost': 1.0,
     'orientation_cost': 0.0, 'lm_damping': 1.0},
    {'name': 'left_hand', 'frame': 'PRT0001_2', 'position_cost': 1.0,
     'orientation_cost': 0.0, 'lm_damping': 1.0},
]


def make_solver(**kwargs):
    defaults = dict(
        urdf_path=URDF, controlled_joints=ARM_JOINTS, ee_configs=EE_CONFIGS,
        posture_cost=1e-3, solver_backend='daqp', solver_damping=0.01,
        safety_break=False,
    )
    defaults.update(kwargs)
    return PinkSolver(**defaults)


def zero_states(solver):
    return {name: 0.0 for name in solver.configuration.all_joint_names}


# === PinkConfiguration ===

class TestPinkConfiguration:
    def test_loads_urdf(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        assert cfg.full_model.njoints == 21
        assert cfg.full_model.nq == 20
        assert len(cfg.all_joint_names) == 20

    def test_all_joint_names_present(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        assert set(cfg.all_joint_names) == set(ALL_JOINTS)

    def test_controlled_joint_count(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        assert len(cfg.controlled_joint_names) == 10
        assert set(cfg.controlled_joint_names) == set(ARM_JOINTS)

    def test_reduced_model_dimensions(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        assert cfg.model.nq == 10
        assert cfg.model.nv == 10

    def test_update_full_q(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        q_full = np.zeros(20)
        q_full[0] = 0.1
        cfg.update(q_full)
        assert cfg.full_q[0] == 0.1
        assert np.allclose(cfg.q, np.zeros(10))

    def test_update_controlled_q(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        q_ctrl = np.ones(10) * 0.05
        cfg.update(q_ctrl)
        assert np.allclose(cfg.q, q_ctrl)
        for i, idx in enumerate(cfg._controlled_joint_indices):
            assert abs(cfg.full_q[idx] - 0.05) < 1e-10

    def test_update_wrong_size_raises(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        with pytest.raises(ValueError):
            cfg.update(np.zeros(7))

    def test_fk_ee_frames_exist(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        for frame in ['PRT0001', 'PRT0001_2']:
            T = cfg.get_transform_frame_to_world(frame)
            assert isinstance(T, pin.SE3)
            assert T.translation.shape == (3,)

    def test_fk_nonexistent_frame_raises(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        with pytest.raises(Exception):
            cfg.get_transform_frame_to_world('nonexistent_frame')

    def test_jacobian_shape(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        J = cfg.get_frame_jacobian('PRT0001')
        assert J.shape == (6, 10)

    def test_jacobian_nonzero(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        J = cfg.get_frame_jacobian('PRT0001')
        assert np.linalg.norm(J) > 0

    def test_get_transform_relative(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        T_rel = cfg.get_transform('PRT0001', 'base')
        T_world = cfg.get_transform_frame_to_world('PRT0001')
        T_base = cfg.get_transform_frame_to_world('base')
        T_expected = T_base.actInv(T_world)
        assert np.allclose(T_rel.homogeneous, T_expected.homogeneous, atol=1e-10)

    def test_locked_joints_affect_fk(self):
        cfg = PinkConfiguration(URDF, ARM_JOINTS)
        T_zero = cfg.get_transform_frame_to_world('RFootBushing_GPF_1517_12').translation.copy()
        q_full = np.zeros(20)
        for i, name in enumerate(cfg.all_joint_names):
            if 'hip' in name or 'knee' in name or 'ankle' in name:
                q_full[i] = 0.3
        cfg.update(q_full)
        T_moved = cfg.get_transform_frame_to_world('RFootBushing_GPF_1517_12').translation.copy()
        assert not np.allclose(T_zero, T_moved, atol=1e-4)

    def test_missing_joint_raises(self):
        with pytest.raises(ValueError, match='not found'):
            PinkConfiguration(URDF, ['nonexistent_joint_xyz'])

    def test_all_joints_controlled(self):
        cfg = PinkConfiguration(URDF, ALL_JOINTS)
        assert cfg.model.nq == 20
        assert len(cfg.controlled_joint_names) == 20

    def test_single_arm(self):
        cfg = PinkConfiguration(URDF, RIGHT_ARM)
        assert cfg.model.nq == 5
        J = cfg.get_frame_jacobian('PRT0001')
        assert J.shape == (6, 5)


# === PinkSolver ===

class TestPinkSolver:
    def test_init(self):
        solver = make_solver()
        assert len(solver.ee_tasks) == 2
        assert 'right_hand' in solver.ee_tasks
        assert 'left_hand' in solver.ee_tasks
        assert not solver.is_initialized

    def test_update_state(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        assert solver.is_initialized

    def test_compute_returns_all_controlled(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('right_hand')
        target.translation[2] += 0.01
        solver.set_ee_target_se3('right_hand', target)
        result = solver.compute(0.02)
        assert len(result) == 10
        assert set(result.keys()) == set(solver.configuration.controlled_joint_names)

    def test_right_target_moves_right_arm(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('right_hand')
        target.translation[2] += 0.05
        solver.set_ee_target_se3('right_hand', target)
        result = solver.compute(0.02)
        assert any(abs(result[j]) > 1e-6 for j in RIGHT_ARM if j in result)
        for j in LEFT_ARM:
            if j in result:
                assert abs(result[j]) < 1e-4

    def test_left_target_moves_left_arm(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('left_hand')
        target.translation[2] += 0.05
        solver.set_ee_target_se3('left_hand', target)
        result = solver.compute(0.02)
        assert any(abs(result[j]) > 1e-6 for j in LEFT_ARM if j in result)
        for j in RIGHT_ARM:
            if j in result:
                assert abs(result[j]) < 1e-4

    def test_bimanual_both_arms_move(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        for name in ['right_hand', 'left_hand']:
            target = solver.get_ee_pose(name)
            target.translation[0] += 0.03
            solver.set_ee_target_se3(name, target)
        result = solver.compute(0.02)
        assert any(abs(result[j]) > 1e-6 for j in RIGHT_ARM if j in result)
        assert any(abs(result[j]) > 1e-6 for j in LEFT_ARM if j in result)

    def test_set_ee_target_matrix(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        T = np.eye(4)
        T[:3, 3] = [0.1, -0.2, 0.0]
        solver.set_ee_target_matrix('right_hand', T)
        result = solver.compute(0.02)
        assert len(result) == 10

    def test_get_ee_pose(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        pose = solver.get_ee_pose('right_hand')
        assert isinstance(pose, pin.SE3)
        assert pose.translation[1] < 0

    def test_symmetry(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        rh = solver.get_ee_pose('right_hand').translation
        lh = solver.get_ee_pose('left_hand').translation
        assert abs(rh[0] - lh[0]) < 0.01
        assert abs(rh[2] - lh[2]) < 0.01
        assert abs(rh[1] + lh[1]) < 0.01

    def test_posture_drift_bounded(self):
        solver = make_solver()
        states = zero_states(solver)
        states['dof_right_shoulder_pitch_03'] = 0.3
        states['dof_right_elbow_02'] = -0.5
        solver.update_state(states)
        target = solver.get_ee_pose('right_hand')
        target_pos = target.translation.copy()
        solver.set_ee_target_se3('right_hand', target)
        solver.compute(0.02)
        ee_after = solver.get_ee_pose('right_hand').translation
        pos_err = np.linalg.norm(target_pos - ee_after)
        assert pos_err < 0.02

    def test_repeated_compute_converges(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('right_hand')
        target.translation[0] += 0.02
        solver.set_ee_target_se3('right_hand', target)
        target_pos = target.translation.copy()
        errors = []
        for _ in range(100):
            solver.compute(0.02)
            current = solver.get_ee_pose('right_hand')
            errors.append(np.linalg.norm(target_pos - current.translation))
        assert errors[-1] < errors[0]
        assert errors[-1] < 0.005

    def test_no_posture_task(self):
        solver = make_solver(posture_cost=0.0)
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('right_hand')
        target.translation[2] += 0.01
        solver.set_ee_target_se3('right_hand', target)
        result = solver.compute(0.02)
        assert len(result) == 10

    def test_with_damping_task(self):
        solver = make_solver(damping_cost=1e-2)
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('right_hand')
        target.translation[2] += 0.01
        solver.set_ee_target_se3('right_hand', target)
        result = solver.compute(0.02)
        assert len(result) == 10

    def test_fallback_solver(self):
        solver = make_solver(solver_backend='nonexistent_solver_xyz')
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('right_hand')
        target.translation[2] += 0.01
        solver.set_ee_target_se3('right_hand', target)
        result = solver.compute(0.02)
        assert len(result) == 10

    def test_joint_limits_respected(self):
        solver = make_solver(solver_damping=1e-4)
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('right_hand')
        target.translation[0] += 0.5
        target.translation[2] += 0.5
        solver.set_ee_target_se3('right_hand', target)
        for _ in range(200):
            result = solver.compute(0.02)
        model = solver.configuration.full_model
        for name, val in result.items():
            jid = model.getJointId(name)
            idx_q = model.joints[jid].idx_q
            lo = model.lowerPositionLimit[idx_q]
            hi = model.upperPositionLimit[idx_q]
            assert lo - 0.01 <= val <= hi + 0.01

    def test_state_reset_between_solves(self):
        solver = make_solver()
        states = zero_states(solver)
        solver.update_state(states)
        target = solver.get_ee_pose('right_hand')
        target.translation[2] += 0.05
        solver.set_ee_target_se3('right_hand', target)
        solver.compute(0.02)
        # update_state() intentionally preserves controlled joints after init
        # (prevents feedback oscillation). Force a full reinit to truly reset.
        solver._initialized = False
        solver.update_state(states)
        pose_after_reset = solver.get_ee_pose('right_hand')
        solver2 = make_solver()
        solver2.update_state(states)
        pose_fresh = solver2.get_ee_pose('right_hand')
        assert np.allclose(pose_after_reset.translation, pose_fresh.translation, atol=1e-6)

    def test_orientation_cost(self):
        ee_configs_ori = [
            {'name': 'right_hand', 'frame': 'PRT0001', 'position_cost': 1.0,
             'orientation_cost': 1.0, 'lm_damping': 1.0},
        ]
        solver = PinkSolver(
            URDF, ARM_JOINTS[:5], ee_configs_ori,
            posture_cost=1e-3, solver_damping=0.01, safety_break=False,
        )
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('right_hand')
        R_delta = pin.utils.rpyToMatrix(0.0, 0.1, 0.0)
        target.rotation = R_delta @ target.rotation
        solver.set_ee_target_se3('right_hand', target)
        result = solver.compute(0.02)
        assert any(abs(v) > 1e-6 for v in result.values())


# === Integration ===

class TestIntegration:
    def test_solve_time(self):
        import time
        solver = make_solver()
        solver.update_state(zero_states(solver))
        target = solver.get_ee_pose('right_hand')
        target.translation[0] += 0.01
        solver.set_ee_target_se3('right_hand', target)
        for _ in range(10):
            solver.compute(0.02)
        t0 = time.perf_counter()
        N = 1000
        for _ in range(N):
            solver.compute(0.02)
        ms_per = (time.perf_counter() - t0) / N * 1000
        assert ms_per < 5.0

    def test_incremental_tracking(self):
        solver = make_solver(solver_damping=0.005)
        solver.update_state(zero_states(solver))
        base_pos = solver.get_ee_pose('right_hand').translation.copy()
        max_err = 0.0
        for step in range(100):
            target = solver.get_ee_pose('right_hand')
            desired = base_pos.copy()
            desired[0] += 0.0005 * step
            target.translation = desired
            solver.set_ee_target_se3('right_hand', target)
            solver.compute(0.02)
            current = solver.get_ee_pose('right_hand').translation
            max_err = max(max_err, np.linalg.norm(desired - current))
        assert max_err < 0.05

    def test_bimanual_independence(self):
        solver = make_solver()
        solver.update_state(zero_states(solver))
        for name in ['right_hand', 'left_hand']:
            target = solver.get_ee_pose(name)
            solver.set_ee_target_se3(name, target)
        solver.compute(0.02)
        lh_before = solver.get_ee_pose('left_hand').translation.copy()
        target = solver.get_ee_pose('right_hand')
        target.translation[0] += 0.03
        solver.set_ee_target_se3('right_hand', target)
        for _ in range(50):
            solver.compute(0.02)
        lh_after = solver.get_ee_pose('left_hand').translation.copy()
        drift = np.linalg.norm(lh_after - lh_before)
        assert drift < 0.005
