#!/usr/bin/env python3
"""Mathematical validation tests for steveros_ik.

Validates:
  1. Reduced model FK matches full model FK (consistency)
  2. Jacobian correctness via numerical differentiation
  3. q-vector mapping between full and controlled representations
  4. IK solution validity (FK roundtrip)
  5. Velocity integration correctness

Usage:
    python3 test_math_validation.py
"""

import os
import sys
import traceback

import numpy as np
import pinocchio as pin

# Add package to path so we can import without colcon
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from steveros_ik.pink_configuration import PinkConfiguration
from steveros_ik.pink_solver import PinkSolver

URDF_PATH = os.path.join(
    SCRIPT_DIR, "..", "steveros_description", "urdf", "kbot.urdf"
)

CONTROLLED_JOINTS = [
    "dof_right_shoulder_pitch_03",
    "dof_right_shoulder_roll_03",
    "dof_right_shoulder_yaw_02",
    "dof_right_elbow_02",
    "dof_right_wrist_00",
    "dof_left_shoulder_pitch_03",
    "dof_left_shoulder_roll_03",
    "dof_left_shoulder_yaw_02",
    "dof_left_elbow_02",
    "dof_left_wrist_00",
]

EE_FRAMES = ["PRT0001", "PRT0001_2"]  # right hand, left hand

# ─────────────────────────── helpers ───────────────────────────

passed = 0
failed = 0


def report(name: str, ok: bool, detail: str = ""):
    global passed, failed
    tag = "PASS" if ok else "FAIL"
    if ok:
        passed += 1
    else:
        failed += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def se3_pos_diff(a: pin.SE3, b: pin.SE3) -> float:
    return float(np.linalg.norm(a.translation - b.translation))


def se3_rot_diff(a: pin.SE3, b: pin.SE3) -> float:
    return float(np.linalg.norm(pin.log3(a.rotation.T @ b.rotation)))


def random_arm_q(rng: np.random.Generator, n: int = 10, scale: float = 0.5):
    """Random controlled joint angles within a moderate range."""
    return rng.uniform(-scale, scale, size=n)


def build_full_q(cfg: PinkConfiguration, ctrl_q: np.ndarray) -> np.ndarray:
    """Build a full q from the current full_q, overwriting controlled entries."""
    full = cfg.full_q.copy()
    for i, idx in enumerate(cfg._controlled_joint_indices):
        full[idx] = ctrl_q[i]
    return full


# ──────────────────── TEST 1: Reduced Model Consistency ────────────────────

def test_reduced_model_consistency():
    print("\n=== TEST 1: Reduced Model FK vs Full Model FK ===")
    rng = np.random.default_rng(42)

    cfg = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)

    # Also build an independent full model for comparison
    full_model = pin.buildModelFromUrdf(URDF_PATH)
    full_data = full_model.createData()

    n_configs = 10
    max_pos_err = 0.0
    max_rot_err = 0.0

    for trial in range(n_configs):
        ctrl_q = random_arm_q(rng)
        full_q = build_full_q(cfg, ctrl_q)

        # Update PinkConfiguration with full q
        cfg.update(full_q)

        # Independent FK on the full model
        pin.forwardKinematics(full_model, full_data, full_q)
        pin.updateFramePlacements(full_model, full_data)

        for frame in EE_FRAMES:
            fid = full_model.getFrameId(frame)
            T_ref = full_data.oMf[fid]
            T_cfg = cfg.get_transform_frame_to_world(frame)

            pos_err = se3_pos_diff(T_ref, T_cfg)
            rot_err = se3_rot_diff(T_ref, T_cfg)
            max_pos_err = max(max_pos_err, pos_err)
            max_rot_err = max(max_rot_err, rot_err)

    tol = 1e-10
    report(
        f"Full model FK matches independent FK (pos)",
        max_pos_err < tol,
        f"max_pos_err={max_pos_err:.2e}, tol={tol:.0e}",
    )
    report(
        f"Full model FK matches independent FK (rot)",
        max_rot_err < tol,
        f"max_rot_err={max_rot_err:.2e}, tol={tol:.0e}",
    )

    # Now check reduced model FK (cfg.data is the reduced model data)
    # The reduced model has locked joints baked in; check that the EE
    # frames that exist in the reduced model give the same FK.
    # Note: reduced model frames may have different IDs. Check by name.
    max_pos_err_red = 0.0
    max_rot_err_red = 0.0

    for trial in range(n_configs):
        ctrl_q = random_arm_q(rng)
        full_q = build_full_q(cfg, ctrl_q)

        cfg.update(full_q)

        for frame in EE_FRAMES:
            # Full model result (from cfg.get_transform_frame_to_world)
            T_full = cfg.get_transform_frame_to_world(frame)

            # Reduced model result (from cfg.data which is the reduced/Pink model)
            if cfg.model.existFrame(frame):
                red_fid = cfg.model.getFrameId(frame)
                T_red = cfg.data.oMf[red_fid]
                pos_err = se3_pos_diff(T_full, T_red)
                rot_err = se3_rot_diff(T_full, T_red)
                max_pos_err_red = max(max_pos_err_red, pos_err)
                max_rot_err_red = max(max_rot_err_red, rot_err)

    tol_reduced = 1e-10
    report(
        f"Reduced model FK matches full model FK (pos)",
        max_pos_err_red < tol_reduced,
        f"max_pos_err={max_pos_err_red:.2e}, tol={tol_reduced:.0e}",
    )
    report(
        f"Reduced model FK matches full model FK (rot)",
        max_rot_err_red < tol_reduced,
        f"max_rot_err={max_rot_err_red:.2e}, tol={tol_reduced:.0e}",
    )


# ──────────────────── TEST 2: Jacobian Correctness ────────────────────

def test_jacobian_correctness():
    print("\n=== TEST 2: Jacobian via Numerical Differentiation ===")
    rng = np.random.default_rng(123)
    cfg = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)

    n_configs = 5
    eps = 1e-7
    max_err = 0.0
    worst_joint = ""
    worst_frame = ""

    for trial in range(n_configs):
        ctrl_q = random_arm_q(rng)
        full_q = build_full_q(cfg, ctrl_q)
        cfg.update(full_q)

        for frame in EE_FRAMES:
            # Analytic Jacobian (LOCAL frame, sliced to controlled joints)
            J_analytic = cfg.get_frame_jacobian(frame)
            assert J_analytic.shape == (6, len(CONTROLLED_JOINTS)), (
                f"Jacobian shape {J_analytic.shape} unexpected"
            )

            # Numerical Jacobian via finite differences
            J_numerical = np.zeros_like(J_analytic)
            T0 = cfg.get_transform_frame_to_world(frame).copy()

            for j_idx in range(len(CONTROLLED_JOINTS)):
                # Perturb the j-th controlled joint
                full_q_plus = full_q.copy()
                ctrl_joint_full_idx = cfg._controlled_joint_indices[j_idx]
                full_q_plus[ctrl_joint_full_idx] += eps

                # Compute FK at perturbed configuration
                pin.forwardKinematics(cfg.full_model, cfg.full_data, full_q_plus)
                pin.updateFramePlacements(cfg.full_model, cfg.full_data)
                fid = cfg.full_model.getFrameId(frame)
                T_plus = cfg.full_data.oMf[fid].copy()

                # Restore original FK
                pin.forwardKinematics(cfg.full_model, cfg.full_data, full_q)
                pin.updateFramePlacements(cfg.full_model, cfg.full_data)

                # Compute twist in LOCAL frame:
                # delta = T0^{-1} * T_plus, then log to get the local twist
                dT = T0.actInv(T_plus)
                twist = pin.log6(dT).vector / eps
                J_numerical[:, j_idx] = twist

            col_err = np.linalg.norm(J_analytic - J_numerical, axis=0)
            err = float(np.max(col_err))
            if err > max_err:
                max_err = err
                worst_joint = CONTROLLED_JOINTS[int(np.argmax(col_err))]
                worst_frame = frame

    # Tolerance: numerical differentiation with eps=1e-7 gives ~O(eps) accuracy
    # for the first-order approximation, so errors ~1e-5 to 1e-6 are expected.
    tol = 1e-4
    report(
        "Analytic Jacobian matches numerical Jacobian",
        max_err < tol,
        f"max_col_err={max_err:.2e}, tol={tol:.0e}, worst_joint={worst_joint}, worst_frame={worst_frame}",
    )

    # Also verify Jacobian shape and non-degeneracy
    cfg.update(build_full_q(cfg, np.zeros(10)))
    for frame in EE_FRAMES:
        J = cfg.get_frame_jacobian(frame)
        rank = np.linalg.matrix_rank(J, tol=1e-6)
        report(
            f"Jacobian rank for {frame} at zero config",
            rank >= 3,
            f"rank={rank} (shape {J.shape}, need >=3 for position control)",
        )


# ──────────────────── TEST 3: q-Vector Mapping ────────────────────

def test_q_vector_mapping():
    print("\n=== TEST 3: q-Vector Mapping (Full vs Controlled) ===")
    rng = np.random.default_rng(999)
    cfg = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)

    n_configs = 10
    max_pos_err = 0.0
    max_rot_err = 0.0

    for trial in range(n_configs):
        ctrl_q = random_arm_q(rng)
        full_q = build_full_q(cfg, ctrl_q)

        # Path A: update with full q
        cfg_a = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)
        cfg_a.update(full_q)

        # Path B: update with controlled q only
        cfg_b = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)
        cfg_b.update(ctrl_q)

        for frame in EE_FRAMES:
            T_a = cfg_a.get_transform_frame_to_world(frame)
            T_b = cfg_b.get_transform_frame_to_world(frame)

            pos_err = se3_pos_diff(T_a, T_b)
            rot_err = se3_rot_diff(T_a, T_b)
            max_pos_err = max(max_pos_err, pos_err)
            max_rot_err = max(max_rot_err, rot_err)

    tol = 1e-10
    report(
        "update(full_q) == update(ctrl_q) FK (pos)",
        max_pos_err < tol,
        f"max_pos_err={max_pos_err:.2e}, tol={tol:.0e}",
    )
    report(
        "update(full_q) == update(ctrl_q) FK (rot)",
        max_rot_err < tol,
        f"max_rot_err={max_rot_err:.2e}, tol={tol:.0e}",
    )

    # Verify that _controlled_joint_indices round-trips correctly
    cfg_c = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)
    ctrl_q = rng.uniform(-0.3, 0.3, size=10)
    full_q = build_full_q(cfg_c, ctrl_q)
    cfg_c.update(full_q)

    # Extract back from full_q
    extracted = np.array([cfg_c.full_q[i] for i in cfg_c._controlled_joint_indices])
    roundtrip_err = float(np.max(np.abs(extracted - ctrl_q)))
    report(
        "Controlled indices roundtrip full_q -> ctrl_q",
        roundtrip_err < 1e-15,
        f"max_err={roundtrip_err:.2e}",
    )

    # Verify reduced model q matches the controlled subset
    reduced_q = np.array(cfg_c.q)
    ctrl_match_err = float(np.max(np.abs(reduced_q - ctrl_q)))
    report(
        "Reduced model q matches controlled joint values",
        ctrl_match_err < 1e-15,
        f"max_err={ctrl_match_err:.2e}",
    )

    # Verify non-controlled joints in full_q remain zero after ctrl update
    cfg_d = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)
    ctrl_q = rng.uniform(-0.5, 0.5, size=10)
    cfg_d.update(ctrl_q)
    all_names = cfg_d.all_joint_names
    ctrl_set = set(CONTROLLED_JOINTS)
    max_locked_err = 0.0
    for i, name in enumerate(all_names):
        if name not in ctrl_set:
            max_locked_err = max(max_locked_err, abs(cfg_d.full_q[i]))
    report(
        "Non-controlled joints remain at initial (zero) after ctrl update",
        max_locked_err < 1e-15,
        f"max_locked_err={max_locked_err:.2e}",
    )


# ──────────────────── TEST 4: IK Solution Validity ────────────────────

def test_ik_roundtrip():
    print("\n=== TEST 4: IK Solution Validity (FK Roundtrip) ===")

    ee_configs = [
        {"name": "right_hand", "frame": "PRT0001", "position_cost": 1.0,
         "orientation_cost": 0.0, "lm_damping": 1.0},
        {"name": "left_hand", "frame": "PRT0001_2", "position_cost": 1.0,
         "orientation_cost": 0.0, "lm_damping": 1.0},
    ]

    solver = PinkSolver(
        urdf_path=URDF_PATH,
        controlled_joints=CONTROLLED_JOINTS,
        ee_configs=ee_configs,
        posture_cost=1e-3,
        damping_cost=0.0,
        solver_backend="daqp",
        solver_damping=0.01,
        safety_break=False,
        max_joint_velocity=2.0,
    )

    # Initialize at zero configuration
    all_names = solver.configuration.all_joint_names
    zero_state = {name: 0.0 for name in all_names}
    solver.update_state(zero_state)

    # Get current EE pose
    T0_right = solver.get_ee_pose("right_hand")
    T0_left = solver.get_ee_pose("left_hand")
    print(f"  Initial right hand pos: {T0_right.translation}")
    print(f"  Initial left  hand pos: {T0_left.translation}")

    # Set a reachable target: move right hand 3cm forward (in world x)
    target_right = T0_right.copy()
    target_right.translation[0] += 0.03  # 3cm in x
    solver.set_ee_target_se3("right_hand", target_right)

    # Keep left hand at current pose
    solver.set_ee_target_se3("left_hand", T0_left)

    # Run IK for 100 iterations (2 seconds at 50 Hz)
    dt = 0.02
    n_iters = 100
    for _ in range(n_iters):
        result = solver.compute(dt)

    # Check how close the right hand got to the target
    T_final_right = solver.get_ee_pose("right_hand")
    T_final_left = solver.get_ee_pose("left_hand")

    pos_err_right = float(np.linalg.norm(
        T_final_right.translation - target_right.translation
    ))
    pos_err_left = float(np.linalg.norm(
        T_final_left.translation - T0_left.translation
    ))

    print(f"  Final right hand pos:   {T_final_right.translation}")
    print(f"  Target right hand pos:  {target_right.translation}")
    print(f"  Right hand pos error:   {pos_err_right:.6f} m")
    print(f"  Left hand drift:        {pos_err_left:.6f} m")

    # FK roundtrip: take the final joint positions and verify FK independently
    final_q = solver.configuration.full_q.copy()
    indep_model = pin.buildModelFromUrdf(URDF_PATH)
    indep_data = indep_model.createData()
    pin.forwardKinematics(indep_model, indep_data, final_q)
    pin.updateFramePlacements(indep_model, indep_data)

    fid_right = indep_model.getFrameId("PRT0001")
    T_indep_right = indep_data.oMf[fid_right]
    fk_consistency_err = float(np.linalg.norm(
        T_indep_right.translation - T_final_right.translation
    ))
    report(
        "FK roundtrip: solver FK == independent FK",
        fk_consistency_err < 1e-10,
        f"err={fk_consistency_err:.2e}",
    )

    # IK convergence: right hand should reach within 5mm of target
    report(
        "IK convergence: right hand within 5mm of target",
        pos_err_right < 0.005,
        f"pos_err={pos_err_right * 1000:.2f} mm",
    )

    # Left hand should not drift more than 5mm
    report(
        "IK stability: left hand drift < 5mm",
        pos_err_left < 0.005,
        f"drift={pos_err_left * 1000:.2f} mm",
    )

    # Test 4b: larger motion — move both hands in well-conditioned direction
    # At zero config the z-direction is near a kinematic singularity (sv~0.007)
    # so we test in x (forward) which is the best-conditioned direction (sv~0.46).
    print("\n  --- 4b: Move both hands simultaneously (forward in x) ---")
    solver2 = PinkSolver(
        urdf_path=URDF_PATH,
        controlled_joints=CONTROLLED_JOINTS,
        ee_configs=ee_configs,
        posture_cost=1e-3,
        solver_backend="daqp",
        solver_damping=0.01,
        safety_break=False,
        max_joint_velocity=2.0,
    )
    solver2.update_state(zero_state)

    T0_r = solver2.get_ee_pose("right_hand")
    T0_l = solver2.get_ee_pose("left_hand")

    target_r = T0_r.copy()
    target_l = T0_l.copy()
    target_r.translation[0] += 0.03  # 3cm forward in x
    target_l.translation[0] += 0.03  # 3cm forward in x
    solver2.set_ee_target_se3("right_hand", target_r)
    solver2.set_ee_target_se3("left_hand", target_l)

    for _ in range(200):
        solver2.compute(dt)

    T_final_r = solver2.get_ee_pose("right_hand")
    T_final_l = solver2.get_ee_pose("left_hand")
    err_r = float(np.linalg.norm(T_final_r.translation - target_r.translation))
    err_l = float(np.linalg.norm(T_final_l.translation - target_l.translation))
    print(f"  Right hand pos error:  {err_r * 1000:.2f} mm")
    print(f"  Left  hand pos error:  {err_l * 1000:.2f} mm")

    report(
        "Both hands reach within 5mm of target (x-direction)",
        err_r < 0.005 and err_l < 0.005,
        f"right={err_r * 1000:.2f}mm, left={err_l * 1000:.2f}mm",
    )

    # Test 4c: verify z-direction is poorly conditioned (workspace characterization)
    print("\n  --- 4c: Workspace characterization (z-direction singularity) ---")
    solver3 = PinkSolver(
        urdf_path=URDF_PATH,
        controlled_joints=CONTROLLED_JOINTS,
        ee_configs=ee_configs,
        posture_cost=1e-3,
        solver_backend="daqp",
        solver_damping=0.01,
        safety_break=False,
        max_joint_velocity=2.0,
    )
    solver3.update_state(zero_state)
    cfg_z = solver3.configuration

    # Compute SVD of position Jacobian for right hand in WORLD frame
    fid_r = cfg_z.full_model.getFrameId("PRT0001")
    J_world = pin.getFrameJacobian(
        cfg_z.full_model, cfg_z.full_data, fid_r, pin.ReferenceFrame.WORLD,
    )
    # Right arm columns (indices 15-19 in full model)
    right_arm_full_idx = [15, 16, 17, 18, 19]
    J_pos_right = J_world[:3, right_arm_full_idx]
    sv = np.linalg.svd(J_pos_right, compute_uv=False)
    print(f"  Right arm position singular values at zero config: {sv}")
    condition_number = sv[0] / sv[-1] if sv[-1] > 1e-10 else float("inf")
    print(f"  Condition number: {condition_number:.1f}")
    report(
        "z-direction singularity documented (condition number > 10)",
        condition_number > 10,
        f"condition_number={condition_number:.1f} (expected: high at zero config)",
    )


# ────────────── TEST 5: Velocity Integration Correctness ──────────────

def test_velocity_integration():
    print("\n=== TEST 5: Velocity Integration Correctness ===")
    rng = np.random.default_rng(777)
    cfg = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)

    # Start from a known configuration
    ctrl_q_init = random_arm_q(rng, scale=0.3)
    full_q_init = build_full_q(cfg, ctrl_q_init)
    cfg.update(full_q_init)

    # Apply a known velocity
    n_ctrl = len(CONTROLLED_JOINTS)
    velocity = rng.uniform(-0.5, 0.5, size=n_ctrl)
    dt = 0.02

    # Record pre-integration state
    q_before = np.array(cfg.q).copy()
    full_q_before = cfg.full_q.copy()

    # Integrate via Pink's integrate_inplace (which calls pin.integrate internally)
    cfg.integrate_inplace(velocity, dt)

    q_after = np.array(cfg.q)
    full_q_after = cfg.full_q.copy()

    # For revolute joints, pin.integrate(q, v*dt) = q + v*dt
    expected_ctrl_q = q_before + velocity * dt
    integration_err = float(np.max(np.abs(q_after - expected_ctrl_q)))
    report(
        "integrate_inplace: q += v*dt for revolute joints",
        integration_err < 1e-14,
        f"max_err={integration_err:.2e}",
    )

    # Verify full model q was updated consistently
    expected_full_q = full_q_before.copy()
    for i, idx in enumerate(cfg._controlled_joint_indices):
        expected_full_q[idx] = expected_ctrl_q[i]

    full_q_err = float(np.max(np.abs(full_q_after - expected_full_q)))
    report(
        "Full q updated consistently after integrate_inplace",
        full_q_err < 1e-14,
        f"max_err={full_q_err:.2e}",
    )

    # Verify non-controlled joints are unchanged
    ctrl_idx_set = set(cfg._controlled_joint_indices)
    max_locked_change = 0.0
    for i in range(len(cfg.all_joint_names)):
        if i not in ctrl_idx_set:
            max_locked_change = max(max_locked_change,
                                     abs(full_q_after[i] - full_q_before[i]))
    report(
        "Non-controlled joints unchanged after integration",
        max_locked_change < 1e-15,
        f"max_change={max_locked_change:.2e}",
    )

    # Verify FK is consistent between full and reduced models after integration
    T_full = {}
    T_red = {}
    for frame in EE_FRAMES:
        T_full[frame] = cfg.get_transform_frame_to_world(frame)
        if cfg.model.existFrame(frame):
            red_fid = cfg.model.getFrameId(frame)
            T_red[frame] = cfg.data.oMf[red_fid]

    max_pos_err = 0.0
    max_rot_err = 0.0
    for frame in EE_FRAMES:
        if frame in T_red:
            pos_err = se3_pos_diff(T_full[frame], T_red[frame])
            rot_err = se3_rot_diff(T_full[frame], T_red[frame])
            max_pos_err = max(max_pos_err, pos_err)
            max_rot_err = max(max_rot_err, rot_err)

    tol = 1e-10
    report(
        "FK consistent (full vs reduced) after integration (pos)",
        max_pos_err < tol,
        f"max_pos_err={max_pos_err:.2e}",
    )
    report(
        "FK consistent (full vs reduced) after integration (rot)",
        max_rot_err < tol,
        f"max_rot_err={max_rot_err:.2e}",
    )

    # Multi-step integration: accumulate 50 steps and verify FK consistency
    print("\n  --- 5b: Multi-step integration (50 steps) ---")
    cfg2 = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)
    ctrl_q_init2 = random_arm_q(rng, scale=0.2)
    cfg2.update(build_full_q(cfg2, ctrl_q_init2))

    for step in range(50):
        v = rng.uniform(-0.3, 0.3, size=n_ctrl)
        cfg2.integrate_inplace(v, dt)

    # After 50 integration steps, verify full/reduced model agreement
    max_pos_err2 = 0.0
    max_rot_err2 = 0.0
    for frame in EE_FRAMES:
        T_f = cfg2.get_transform_frame_to_world(frame)
        if cfg2.model.existFrame(frame):
            red_fid = cfg2.model.getFrameId(frame)
            T_r = cfg2.data.oMf[red_fid]
            max_pos_err2 = max(max_pos_err2, se3_pos_diff(T_f, T_r))
            max_rot_err2 = max(max_rot_err2, se3_rot_diff(T_f, T_r))

    report(
        "FK consistent after 50 integration steps (pos)",
        max_pos_err2 < 1e-10,
        f"max_pos_err={max_pos_err2:.2e}",
    )
    report(
        "FK consistent after 50 integration steps (rot)",
        max_rot_err2 < 1e-10,
        f"max_rot_err={max_rot_err2:.2e}",
    )

    # Verify accumulated q matches manual summation
    cfg3 = PinkConfiguration(URDF_PATH, CONTROLLED_JOINTS)
    q_start = random_arm_q(rng, scale=0.1)
    cfg3.update(build_full_q(cfg3, q_start))

    q_manual = q_start.copy()
    velocities = [rng.uniform(-0.2, 0.2, size=n_ctrl) for _ in range(20)]
    for v in velocities:
        cfg3.integrate_inplace(v, dt)
        q_manual += v * dt

    q_accum_err = float(np.max(np.abs(np.array(cfg3.q) - q_manual)))
    report(
        "Accumulated integration matches manual q += v*dt",
        q_accum_err < 1e-12,
        f"max_err={q_accum_err:.2e}",
    )


# ─────────────────────────── MAIN ───────────────────────────

def main():
    global passed, failed
    print("=" * 70)
    print("  steveros_ik Mathematical Validation Tests")
    print("=" * 70)

    try:
        test_reduced_model_consistency()
    except Exception:
        print(f"  [FAIL] TEST 1 CRASHED:")
        traceback.print_exc()
        failed += 1

    try:
        test_jacobian_correctness()
    except Exception:
        print(f"  [FAIL] TEST 2 CRASHED:")
        traceback.print_exc()
        failed += 1

    try:
        test_q_vector_mapping()
    except Exception:
        print(f"  [FAIL] TEST 3 CRASHED:")
        traceback.print_exc()
        failed += 1

    try:
        test_ik_roundtrip()
    except Exception:
        print(f"  [FAIL] TEST 4 CRASHED:")
        traceback.print_exc()
        failed += 1

    try:
        test_velocity_integration()
    except Exception:
        print(f"  [FAIL] TEST 5 CRASHED:")
        traceback.print_exc()
        failed += 1

    print("\n" + "=" * 70)
    total = passed + failed
    print(f"  RESULTS: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  ALL CHECKS PASSED")
    else:
        print(f"  WARNING: {failed} CHECK(S) FAILED — see details above")
    print("=" * 70)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
