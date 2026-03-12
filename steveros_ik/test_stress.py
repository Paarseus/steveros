#!/usr/bin/env python3
"""Stress tests for steveros_ik PinkSolver: singularities, unreachable targets,
workspace mapping, velocity clamping, convergence analysis, dt sensitivity,
and opposing bimanual targets.

Run standalone:  python3 test_stress.py
"""

from __future__ import annotations

import sys
import time
import numpy as np
import pinocchio as pin

from steveros_ik.pink_solver import PinkSolver

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
URDF = "/home/nemo/Desktop/steve/steveros/steveros_description/urdf/kbot.urdf"

ARM_JOINTS = [
    "dof_right_shoulder_pitch_03", "dof_right_shoulder_roll_03",
    "dof_right_shoulder_yaw_02",   "dof_right_elbow_02",
    "dof_right_wrist_00",
    "dof_left_shoulder_pitch_03",  "dof_left_shoulder_roll_03",
    "dof_left_shoulder_yaw_02",    "dof_left_elbow_02",
    "dof_left_wrist_00",
]

RIGHT_ARM = ARM_JOINTS[:5]
LEFT_ARM  = ARM_JOINTS[5:]

EE_CONFIGS = [
    {"name": "right_hand", "frame": "PRT0001",
     "position_cost": 1.0, "orientation_cost": 0.0, "lm_damping": 1.0},
    {"name": "left_hand",  "frame": "PRT0001_2",
     "position_cost": 1.0, "orientation_cost": 0.0, "lm_damping": 1.0},
]

DT = 0.02
N_ITERS = 200
PASS_TAG = "\033[92mPASS\033[0m"
FAIL_TAG = "\033[91mFAIL\033[0m"

results: list[tuple[str, bool, str]] = []


def record(name: str, passed: bool, detail: str = "") -> None:
    tag = PASS_TAG if passed else FAIL_TAG
    results.append((name, passed, detail))
    print(f"  [{tag}] {name}" + (f"  -- {detail}" if detail else ""))


def make_solver(**kw) -> PinkSolver:
    defaults = dict(
        urdf_path=URDF, controlled_joints=ARM_JOINTS, ee_configs=EE_CONFIGS,
        posture_cost=1e-3, solver_damping=0.01, safety_break=False,
        max_joint_velocity=2.0,
    )
    defaults.update(kw)
    return PinkSolver(**defaults)


def zero_states(solver: PinkSolver) -> dict[str, float]:
    return {n: 0.0 for n in solver.configuration.all_joint_names}


def joint_limits(solver: PinkSolver) -> dict[str, tuple[float, float]]:
    model = solver.configuration.full_model
    limits = {}
    for name in ARM_JOINTS:
        jid = model.getJointId(name)
        idx_q = model.joints[jid].idx_q
        limits[name] = (float(model.lowerPositionLimit[idx_q]),
                        float(model.upperPositionLimit[idx_q]))
    return limits


def check_no_nan(result: dict[str, float]) -> bool:
    return all(np.isfinite(v) for v in result.values())


def check_limits(result: dict[str, float], limits: dict, tol: float = 0.02) -> bool:
    for name, val in result.items():
        if name in limits:
            lo, hi = limits[name]
            if val < lo - tol or val > hi + tol:
                return False
    return True


# ===================================================================
# 1. SINGULARITY BEHAVIOUR
# ===================================================================
def test_singularity() -> None:
    print("\n=== 1. SINGULARITY BEHAVIOUR ===")
    solver = make_solver()
    # Drive the right arm to near full extension (elbow near max)
    states = zero_states(solver)
    states["dof_right_shoulder_pitch_03"] = -1.5   # arm forward
    states["dof_right_elbow_02"]          =  0.01   # nearly straight
    solver.update_state(states)

    # Target slightly further out along the extended direction
    ee_pose = solver.get_ee_pose("right_hand")
    target = ee_pose.copy()
    target.translation[0] += 0.05   # push further in reach direction
    solver.set_ee_target_se3("right_hand", target)

    all_finite = True
    max_delta = 0.0
    limits = joint_limits(solver)
    lim_ok = True
    for _ in range(N_ITERS):
        result = solver.compute(DT)
        if not check_no_nan(result):
            all_finite = False
        if not check_limits(result, limits):
            lim_ok = False

    record("Singularity: no NaN/Inf",       all_finite)
    record("Singularity: joint limits ok",   lim_ok)


# ===================================================================
# 2. UNREACHABLE TARGETS
# ===================================================================
def test_unreachable() -> None:
    print("\n=== 2. UNREACHABLE TARGETS ===")
    solver = make_solver()
    solver.update_state(zero_states(solver))

    far_targets = [
        ("2m forward",  np.array([2.0,  0.0,  0.0])),
        ("2m up",       np.array([0.0,  0.0,  2.0])),
        ("2m right",    np.array([0.0, -2.0,  0.0])),
        ("3m diagonal", np.array([1.5, -1.5,  1.5])),
    ]

    limits = joint_limits(solver)

    for label, pos in far_targets:
        solver2 = make_solver()
        solver2.update_state(zero_states(solver2))
        target = solver2.get_ee_pose("right_hand")
        target.translation = pos
        solver2.set_ee_target_se3("right_hand", target)

        all_finite = True
        lim_ok = True
        for _ in range(N_ITERS):
            result = solver2.compute(DT)
            if not check_no_nan(result):
                all_finite = False
            if not check_limits(result, limits):
                lim_ok = False

        record(f"Unreachable ({label}): no NaN",      all_finite)
        record(f"Unreachable ({label}): limits ok",    lim_ok)


# ===================================================================
# 3. WORKSPACE BOUNDARY MAPPING
# ===================================================================
def test_workspace() -> None:
    print("\n=== 3. WORKSPACE BOUNDARY MAPPING ===")
    # Sweep targets in a grid and let the solver converge; record achieved EE pos.
    radius_range = np.linspace(0.05, 0.8, 8)
    theta_range  = np.linspace(-np.pi, np.pi, 12, endpoint=False)
    phi_range    = np.linspace(-np.pi / 2, np.pi / 2, 7)

    for hand, label in [("right_hand", "Right"), ("left_hand", "Left")]:
        achieved: list[np.ndarray] = []
        for r in radius_range:
            for th in theta_range:
                for ph in phi_range:
                    x = r * np.cos(ph) * np.cos(th)
                    y = r * np.cos(ph) * np.sin(th)
                    z = r * np.sin(ph)
                    solver = make_solver()
                    solver.update_state(zero_states(solver))

                    # offset from shoulder approx location
                    origin = solver.get_ee_pose(hand).translation.copy()

                    target = solver.get_ee_pose(hand)
                    target.translation = origin + np.array([x, y, z])
                    solver.set_ee_target_se3(hand, target)

                    for _ in range(N_ITERS):
                        solver.compute(DT)
                    pos = solver.get_ee_pose(hand).translation.copy()
                    achieved.append(pos)

        pts = np.array(achieved)
        mn = pts.min(axis=0)
        mx = pts.max(axis=0)
        span = mx - mn
        print(f"  {label} hand workspace envelope:")
        print(f"    X: [{mn[0]:.4f}, {mx[0]:.4f}]  span={span[0]:.4f} m")
        print(f"    Y: [{mn[1]:.4f}, {mx[1]:.4f}]  span={span[1]:.4f} m")
        print(f"    Z: [{mn[2]:.4f}, {mx[2]:.4f}]  span={span[2]:.4f} m")

        record(f"Workspace ({label}): has nonzero volume",
               all(s > 0.05 for s in span),
               f"spans: {span[0]:.3f} x {span[1]:.3f} x {span[2]:.3f}")


# ===================================================================
# 4. VELOCITY CLAMPING
# ===================================================================
def test_velocity_clamping() -> None:
    print("\n=== 4. VELOCITY CLAMPING ===")
    max_vel = 2.0
    solver = make_solver(max_joint_velocity=max_vel)
    solver.update_state(zero_states(solver))

    # Far-away target to induce large desired velocities
    target = solver.get_ee_pose("right_hand")
    target.translation = np.array([2.0, -2.0, 2.0])
    solver.set_ee_target_se3("right_hand", target)

    # We need to measure velocity = delta_q / dt for each step
    prev_q = {n: 0.0 for n in ARM_JOINTS}
    max_observed_vel = 0.0
    any_nan = False

    for step in range(N_ITERS):
        result = solver.compute(DT)
        if not check_no_nan(result):
            any_nan = True
            break
        for name in ARM_JOINTS:
            vel = abs(result[name] - prev_q[name]) / DT
            if vel > max_observed_vel:
                max_observed_vel = vel
        prev_q = {n: result[n] for n in ARM_JOINTS}

    clamped = max_observed_vel <= max_vel + 0.01  # small tolerance
    record("Velocity clamping: no NaN",          not any_nan)
    record("Velocity clamping: enforced <= 2.0", clamped,
           f"max observed vel = {max_observed_vel:.4f} rad/s")


# ===================================================================
# 5. CONVERGENCE RATE ANALYSIS
# ===================================================================
def test_convergence_rate() -> None:
    print("\n=== 5. CONVERGENCE RATE ANALYSIS ===")
    distances = [0.01, 0.05, 0.10, 0.20]
    convergence_threshold = 0.001  # 1 mm
    max_steps = 500

    for dist in distances:
        solver = make_solver(solver_damping=0.005)
        solver.update_state(zero_states(solver))

        origin = solver.get_ee_pose("right_hand").translation.copy()
        target = solver.get_ee_pose("right_hand")
        target.translation[0] += dist
        solver.set_ee_target_se3("right_hand", target)
        target_pos = target.translation.copy()

        errors = []
        converged_at = None
        for step in range(max_steps):
            solver.compute(DT)
            err = np.linalg.norm(target_pos - solver.get_ee_pose("right_hand").translation)
            errors.append(err)
            if converged_at is None and err < convergence_threshold:
                converged_at = step + 1

        final_err = errors[-1]
        if converged_at is not None:
            record(f"Convergence ({dist*100:.0f}cm): converged in {converged_at} iters",
                   True, f"final err = {final_err*1000:.3f} mm")
        else:
            record(f"Convergence ({dist*100:.0f}cm): did NOT converge within {max_steps}",
                   False, f"final err = {final_err*1000:.3f} mm")

        # Print first 10 errors as a mini convergence curve
        sample_indices = [0, 1, 2, 5, 10, 20, 50, 100, 200, min(499, max_steps-1)]
        sample_indices = [i for i in sample_indices if i < len(errors)]
        print(f"    Convergence curve ({dist*100:.0f}cm target offset, X-axis):")
        for i in sample_indices:
            bar = "#" * max(1, int(errors[i] * 500))
            print(f"      iter {i:4d}: {errors[i]*1000:8.3f} mm  {bar}")


# ===================================================================
# 6. DT SENSITIVITY
# ===================================================================
def test_dt_sensitivity() -> None:
    print("\n=== 6. DT SENSITIVITY ===")
    dt_values = [0.001, 0.01, 0.02, 0.05, 0.1]
    max_steps = 300

    for dt in dt_values:
        solver = make_solver(solver_damping=0.01)
        solver.update_state(zero_states(solver))

        target = solver.get_ee_pose("right_hand")
        target.translation[0] += 0.05
        solver.set_ee_target_se3("right_hand", target)
        target_pos = target.translation.copy()

        all_finite = True
        limits = joint_limits(solver)
        lim_ok = True
        errors = []

        for _ in range(max_steps):
            result = solver.compute(dt)
            if not check_no_nan(result):
                all_finite = False
                break
            if not check_limits(result, limits):
                lim_ok = False
            err = np.linalg.norm(target_pos - solver.get_ee_pose("right_hand").translation)
            errors.append(err)

        final_err = errors[-1] if errors else float("inf")
        stable = all_finite and lim_ok
        converged = final_err < 0.005

        record(f"dt={dt:.3f}: stable (finite + limits)",  stable,
               f"final err = {final_err*1000:.3f} mm")
        record(f"dt={dt:.3f}: converged < 5mm",           converged,
               f"final err = {final_err*1000:.3f} mm")


# ===================================================================
# 7. OPPOSING BIMANUAL TARGETS
# ===================================================================
def test_opposing_bimanual() -> None:
    print("\n=== 7. OPPOSING BIMANUAL TARGETS ===")
    solver = make_solver()
    solver.update_state(zero_states(solver))

    # Set both hands to converge on the same point in front of the chest
    collision_point = np.array([0.3, 0.0, 0.0])

    for hand in ["right_hand", "left_hand"]:
        target = solver.get_ee_pose(hand)
        target.translation = collision_point.copy()
        solver.set_ee_target_se3(hand, target)

    all_finite = True
    limits = joint_limits(solver)
    lim_ok = True
    errors_r = []
    errors_l = []

    for _ in range(N_ITERS):
        result = solver.compute(DT)
        if not check_no_nan(result):
            all_finite = False
        if not check_limits(result, limits):
            lim_ok = False
        err_r = np.linalg.norm(collision_point - solver.get_ee_pose("right_hand").translation)
        err_l = np.linalg.norm(collision_point - solver.get_ee_pose("left_hand").translation)
        errors_r.append(err_r)
        errors_l.append(err_l)

    # Check for oscillation: error should not grow in later iterations
    # Compare average error of last 20 iters to average of iters 80-100
    late_avg_r = np.mean(errors_r[-20:])
    mid_avg_r  = np.mean(errors_r[80:100]) if len(errors_r) >= 100 else np.mean(errors_r[-40:-20])
    oscillating = late_avg_r > mid_avg_r * 1.5  # growing error suggests oscillation

    record("Opposing bimanual: no NaN/Inf",       all_finite)
    record("Opposing bimanual: joint limits ok",   lim_ok)
    record("Opposing bimanual: no divergence",     not oscillating,
           f"late avg err R={late_avg_r*1000:.2f}mm, mid avg={mid_avg_r*1000:.2f}mm")

    print(f"    Final right hand error: {errors_r[-1]*1000:.2f} mm")
    print(f"    Final left  hand error: {errors_l[-1]*1000:.2f} mm")


# ===================================================================
# MAIN
# ===================================================================
def main() -> None:
    t0 = time.perf_counter()

    print("=" * 68)
    print("  steveros_ik PinkSolver -- STRESS TESTS")
    print("=" * 68)

    test_singularity()
    test_unreachable()
    test_workspace()
    test_velocity_clamping()
    test_convergence_rate()
    test_dt_sensitivity()
    test_opposing_bimanual()

    elapsed = time.perf_counter() - t0

    # Summary
    print("\n" + "=" * 68)
    print("  SUMMARY")
    print("=" * 68)
    total = len(results)
    passed = sum(1 for _, p, _ in results if p)
    failed = total - passed
    for name, p, detail in results:
        tag = PASS_TAG if p else FAIL_TAG
        line = f"  [{tag}] {name}"
        if detail:
            line += f"  -- {detail}"
        print(line)
    print(f"\n  {passed}/{total} passed, {failed} failed   ({elapsed:.1f}s)")
    print("=" * 68)

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
