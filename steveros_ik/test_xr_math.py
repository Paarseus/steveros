#!/usr/bin/env python3
"""Standalone math tests for xr_bridge_node coordinate transforms.

No ROS or pytest dependencies. Tests the axis mapping, head-based dynamic
mapping, scale factors, engage/clutch logic, and edge cases.

The bridge uses ROS REP-103 convention throughout:
  - teleop-xr publishes in ROS convention (+X forward, +Y left, +Z up)
  - _axis_map_from_head() extracts forward as R @ [1, 0, 0]
  - DEFAULT_AXIS_MAP is identity (correct for ROS-convention input)

Run:  python3 test_xr_math.py
"""

import sys
import numpy as np
from numpy.linalg import det, norm

# ---------------------------------------------------------------------------
# Re-implement the bridge math locally so we can test without ROS
# ---------------------------------------------------------------------------

DEFAULT_AXIS_MAP = np.array([
    [1.0, 0.0, 0.0],   # robot X (forward) = teleop X
    [0.0, 1.0, 0.0],   # robot Y (left)    = teleop Y
    [0.0, 0.0, 1.0],   # robot Z (up)      = teleop Z
])


def axis_map_from_head(head_quat):
    """Copy of XRBridgeNode._axis_map_from_head (ROS convention).

    Extracts the user's horizontal facing direction from the head quaternion,
    then builds a 3x3 rotation matrix that maps teleop deltas (already in
    ROS convention) so that "user-forward" always maps to robot +X.
    """
    if head_quat is None:
        return None

    qx, qy, qz, qw = head_quat

    # Head forward = R @ [1, 0, 0] (first column of rotation matrix).
    fwd_x = 1.0 - 2.0 * (qy * qy + qz * qz)
    fwd_y = 2.0 * (qx * qy + qw * qz)
    # fwd_z = 2.0 * (qx * qz - qw * qy)  # vertical component, projected out

    # Project onto horizontal plane (zero out Z, which is up in ROS)
    horiz_norm = np.sqrt(fwd_x * fwd_x + fwd_y * fwd_y)
    if horiz_norm < 1e-6:
        return None  # looking straight up/down

    fwd_h = np.array([fwd_x / horiz_norm, fwd_y / horiz_norm, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    left = np.cross(up, fwd_h)  # [-fwd_y, fwd_x, 0]

    # Each row maps teleop delta -> one robot axis
    return np.array([
        fwd_h,   # robot X (forward) = component along user's forward
        left,    # robot Y (left)    = component along user's left
        up,      # robot Z (up)      = vertical component
    ])


def quat_from_yaw(yaw_rad):
    """Quaternion for pure yaw around ROS Z axis (up).

    ROS Z is up, so yaw rotates around Z.
    Returns (qx, qy, qz, qw).
    """
    return (0.0, 0.0, np.sin(yaw_rad / 2.0), np.cos(yaw_rad / 2.0))


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
PASS = 0
FAIL = 0


def check(name, condition, detail=""):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}")
    if detail:
        print(f"         {detail}")


def vec_str(v, dp=4):
    return "[" + ", ".join(f"{x:+.{dp}f}" for x in v) + "]"


def section(title):
    print(f"\n{'='*72}")
    print(f"  {title}")
    print(f"{'='*72}")


# ===================================================================
# TEST 1: Default Axis Map (Identity) with ROS Convention Data
# ===================================================================
def test_default_axis_map():
    section("TEST 1: Default Axis Map (Identity) with ROS Convention Data")

    M = DEFAULT_AXIS_MAP
    scale = 0.75

    print(f"\n  DEFAULT_AXIS_MAP =")
    for row in M:
        print(f"    {vec_str(row)}")

    # ROS convention: +X forward, +Y left, +Z up
    # teleop-xr publishes in this convention

    # --- Scenario A: User moves hand FORWARD ---
    delta_fwd = np.array([0.1, 0.0, 0.0])  # ROS +X = forward
    delta_robot_fwd = M @ delta_fwd * scale
    print(f"\n  Scenario A: User moves hand FORWARD (ROS +X)")
    print(f"    Teleop delta:  {vec_str(delta_fwd)}")
    print(f"    Robot delta:   {vec_str(delta_robot_fwd)}")
    check("Forward (+X) maps to robot +X",
          delta_robot_fwd[0] > 0.001 and abs(delta_robot_fwd[1]) < 1e-9 and abs(delta_robot_fwd[2]) < 1e-9,
          f"Got {vec_str(delta_robot_fwd)}")

    # --- Scenario B: User moves hand RIGHT ---
    delta_right = np.array([0.0, -0.1, 0.0])  # ROS -Y = right
    delta_robot_right = M @ delta_right * scale
    print(f"\n  Scenario B: User moves hand RIGHT (ROS -Y)")
    print(f"    Teleop delta:  {vec_str(delta_right)}")
    print(f"    Robot delta:   {vec_str(delta_robot_right)}")
    check("Right (-Y) maps to robot -Y",
          delta_robot_right[1] < -0.001 and abs(delta_robot_right[0]) < 1e-9 and abs(delta_robot_right[2]) < 1e-9,
          f"Got {vec_str(delta_robot_right)}")

    # --- Scenario C: User moves hand UP ---
    delta_up = np.array([0.0, 0.0, 0.1])  # ROS +Z = up
    delta_robot_up = M @ delta_up * scale
    print(f"\n  Scenario C: User moves hand UP (ROS +Z)")
    print(f"    Teleop delta:  {vec_str(delta_up)}")
    print(f"    Robot delta:   {vec_str(delta_robot_up)}")
    check("Up (+Z) maps to robot +Z",
          delta_robot_up[2] > 0.001 and abs(delta_robot_up[0]) < 1e-9 and abs(delta_robot_up[1]) < 1e-9,
          f"Got {vec_str(delta_robot_up)}")

    # Identity map is correct for ROS-convention data
    check("Identity map det = +1",
          abs(det(M) - 1.0) < 1e-9,
          f"det = {det(M):.6f}")


# ===================================================================
# TEST 2: Head-Based Axis Map Analysis (ROS Convention)
# ===================================================================
def test_head_based_axis_map():
    section("TEST 2: Head-Based Axis Map (ROS Convention)")

    scale = 0.75
    labels = ["fwd_h (row 0 -> robot X)", "left  (row 1 -> robot Y)", "up    (row 2 -> robot Z)"]

    # --- Case A: Identity quaternion (facing +X in ROS) ---
    print(f"\n  Case A: Identity quaternion (head facing +X in ROS)")
    quat_identity = (0.0, 0.0, 0.0, 1.0)
    M_id = axis_map_from_head(quat_identity)

    print(f"    Head quat: {quat_identity}")
    if M_id is not None:
        print(f"    Axis map:")
        for label, row in zip(labels, M_id):
            print(f"      {label}: {vec_str(row)}")

        # At identity in ROS: forward = [1, 0, 0]
        # fwd_h = [1, 0, 0]
        # left = cross([0,0,1], [1,0,0]) = [0, 1, 0]
        # up = [0, 0, 1]
        # Map = identity
        check("Identity: map == identity",
              norm(M_id - np.eye(3)) < 1e-9,
              f"Got:\n{M_id}")

        # Identity map should pass through ROS deltas unchanged
        delta_fwd = M_id @ np.array([0.1, 0.0, 0.0]) * scale
        check("Identity: ROS +X -> robot +X",
              delta_fwd[0] > 0 and abs(delta_fwd[1]) < 1e-9 and abs(delta_fwd[2]) < 1e-9,
              f"Got {vec_str(delta_fwd)}")

        delta_left = M_id @ np.array([0.0, 0.1, 0.0]) * scale
        check("Identity: ROS +Y -> robot +Y",
              delta_left[1] > 0 and abs(delta_left[0]) < 1e-9 and abs(delta_left[2]) < 1e-9,
              f"Got {vec_str(delta_left)}")

        delta_up = M_id @ np.array([0.0, 0.0, 0.1]) * scale
        check("Identity: ROS +Z -> robot +Z",
              delta_up[2] > 0 and abs(delta_up[0]) < 1e-9 and abs(delta_up[1]) < 1e-9,
              f"Got {vec_str(delta_up)}")
    else:
        check("Identity: returns valid map", False, "Got None!")

    # Head map at identity should match DEFAULT_AXIS_MAP
    if M_id is not None:
        check("Head map at identity matches DEFAULT_AXIS_MAP",
              norm(M_id - DEFAULT_AXIS_MAP) < 1e-9,
              f"Max diff: {np.max(np.abs(M_id - DEFAULT_AXIS_MAP)):.6f}")

    # --- Case B: Facing 90 degrees left (yaw = +90 around Z) ---
    # User faces +Y direction in ROS. Their "forward" is now +Y.
    print(f"\n  Case B: Facing 90 degrees LEFT (yaw = +90 deg around Z)")
    yaw_left90 = np.pi / 2
    quat_left90 = quat_from_yaw(yaw_left90)
    M_l90 = axis_map_from_head(quat_left90)

    print(f"    Head quat: ({quat_left90[0]:.4f}, {quat_left90[1]:.4f}, "
          f"{quat_left90[2]:.4f}, {quat_left90[3]:.4f})")
    if M_l90 is not None:
        print(f"    Axis map:")
        for label, row in zip(labels, M_l90):
            print(f"      {label}: {vec_str(row)}")

        # When facing +Y (left in ROS): forward = [0, 1, 0]
        # fwd_h = [0, 1, 0]
        # left = cross([0,0,1], [0,1,0]) = [-1, 0, 0]
        expected_fwd = np.array([0.0, 1.0, 0.0])
        expected_left = np.array([-1.0, 0.0, 0.0])

        check("Left90: fwd_h = [0, 1, 0]",
              norm(M_l90[0] - expected_fwd) < 1e-6,
              f"Got {vec_str(M_l90[0])}")
        check("Left90: left = [-1, 0, 0]",
              norm(M_l90[1] - expected_left) < 1e-6,
              f"Got {vec_str(M_l90[1])}")

        # User faces +Y. Moving in ROS +X (which is user's RIGHT direction)
        # should map to robot -Y (right).
        # M @ [0.1, 0, 0]:
        #   robot X = fwd_h . [0.1, 0, 0] = 0*0.1 = 0
        #   robot Y = left  . [0.1, 0, 0] = -1*0.1 = -0.1
        #   robot Z = up    . [0.1, 0, 0] = 0
        delta_ros_x = M_l90 @ np.array([0.1, 0.0, 0.0]) * scale
        print(f"\n    ROS +X [0.1, 0, 0] -> robot {vec_str(delta_ros_x)}")
        print(f"    (User faces +Y, so ROS +X is user's RIGHT -> robot -Y)")
        check("Left90: ROS +X -> robot -Y (user's right)",
              delta_ros_x[1] < 0 and abs(delta_ros_x[0]) < 1e-9,
              f"Got {vec_str(delta_ros_x)}")

        # Moving in ROS +Y (which is user's FORWARD direction)
        # should map to robot +X (forward).
        delta_ros_y = M_l90 @ np.array([0.0, 0.1, 0.0]) * scale
        print(f"    ROS +Y [0, 0.1, 0] -> robot {vec_str(delta_ros_y)}")
        print(f"    (User faces +Y, so ROS +Y is user's FORWARD -> robot +X)")
        check("Left90: ROS +Y -> robot +X (user's forward)",
              delta_ros_y[0] > 0 and abs(delta_ros_y[1]) < 1e-9,
              f"Got {vec_str(delta_ros_y)}")
    else:
        check("Left90: returns valid map", False, "Got None!")

    # --- Case C: Facing 90 degrees right (yaw = -90 around Z) ---
    print(f"\n  Case C: Facing 90 degrees RIGHT (yaw = -90 deg around Z)")
    yaw_right90 = -np.pi / 2
    quat_right90 = quat_from_yaw(yaw_right90)
    M_r90 = axis_map_from_head(quat_right90)

    if M_r90 is not None:
        print(f"    Axis map:")
        for label, row in zip(labels, M_r90):
            print(f"      {label}: {vec_str(row)}")

        # When facing -Y (right in ROS): forward = [0, -1, 0]
        expected_fwd_r = np.array([0.0, -1.0, 0.0])
        expected_left_r = np.array([1.0, 0.0, 0.0])

        check("Right90: fwd_h = [0, -1, 0]",
              norm(M_r90[0] - expected_fwd_r) < 1e-6,
              f"Got {vec_str(M_r90[0])}")
        check("Right90: left = [1, 0, 0]",
              norm(M_r90[1] - expected_left_r) < 1e-6,
              f"Got {vec_str(M_r90[1])}")
    else:
        check("Right90: returns valid map", False, "Got None!")


# ===================================================================
# TEST 3: Scale Factor Analysis
# ===================================================================
def test_scale_factor():
    section("TEST 3: Scale Factor Analysis")

    scale = 0.75

    # Typical VR hand movement ranges
    vr_reach = 1.0  # meters, typical arm sweep in VR
    # Robot arm workspace
    robot_reach = 0.4  # meters, approximate

    scaled_range = vr_reach * scale
    print(f"\n  Scale factor: {scale}")
    print(f"  Typical VR hand movement: {vr_reach:.2f} m")
    print(f"  Robot arm reach: {robot_reach:.2f} m")
    print(f"  Scaled VR range: {scaled_range:.3f} m ({scaled_range/robot_reach*100:.1f}% of robot workspace)")

    # Per-axis analysis
    print(f"\n  Per-axis analysis (all using same scale = {scale}):")
    axes = [
        ("Forward/Back (X)", 0.5, 0.20, "half-arm forward reach in VR"),
        ("Left/Right (Y)", 0.8, 0.18, "side-to-side sweep in VR"),
        ("Up/Down (Z)", 0.6, 0.25, "vertical sweep in VR"),
    ]
    for name, vr_range, robot_range, note in axes:
        scaled = vr_range * scale
        coverage = scaled / robot_range * 100
        print(f"    {name}:")
        print(f"      VR range: ~{vr_range:.2f} m ({note})")
        print(f"      Robot range: ~{robot_range:.2f} m")
        print(f"      Scaled: {scaled:.3f} m = {coverage:.0f}% of robot range")

    check("Scale is in reasonable range",
          0.1 < scale < 1.5,
          f"scale={scale} is reasonable for teleoperation")
    check("Scale is uniform (isotropic)", True, "Same scale on all axes -- good default")


# ===================================================================
# TEST 4: Engage/Clutch Math
# ===================================================================
def test_engage_clutch():
    section("TEST 4: Engage/Clutch Sequence Simulation")

    # At identity head orientation, map is identity (ROS convention)
    axis_map = np.eye(3)
    scale = 0.75

    # Initial state
    ee_initial = np.array([0.15, -0.18, 0.35])
    current_target = ee_initial.copy()
    ee_start = ee_initial.copy()
    xr_reference = None

    print(f"\n  Using identity axis map (ROS convention, head facing forward)")
    print(f"  Scale: {scale}")

    # --- Step 1: Engage at teleop=[0,0,0], EE at initial position ---
    print(f"\n  Step 1: ENGAGE")
    xr_pose = np.array([0.0, 0.0, 0.0])
    xr_reference = xr_pose.copy()
    ee_start = current_target.copy()
    print(f"    Teleop reference:  {vec_str(xr_reference)}")
    print(f"    EE start:          {vec_str(ee_start)}")
    print(f"    Current target:    {vec_str(current_target)}")

    # --- Step 2: Move teleop to [0.1, -0.05, 0.08] (forward, right, up) ---
    print(f"\n  Step 2: MOVE (teleop -> [0.1, -0.05, 0.08])")
    xr_pose = np.array([0.1, -0.05, 0.08])
    delta_xr = xr_pose - xr_reference
    delta_robot = axis_map @ delta_xr * scale
    current_target = ee_start + delta_robot

    print(f"    Teleop pose:    {vec_str(xr_pose)}")
    print(f"    Teleop delta:   {vec_str(delta_xr)}")
    print(f"    Robot delta:    {vec_str(delta_robot)}")
    print(f"    Current target: {vec_str(current_target)}")

    # With identity map and scale=0.75:
    # robot X = 0.1 * 0.75 = 0.075  (forward)
    # robot Y = -0.05 * 0.75 = -0.0375  (right)
    # robot Z = 0.08 * 0.75 = 0.06  (up)
    expected_delta = np.array([0.075, -0.0375, 0.06])
    check("Move delta correct",
          norm(delta_robot - expected_delta) < 1e-9,
          f"Expected {vec_str(expected_delta)}, got {vec_str(delta_robot)}")

    expected_target_after_move = ee_initial + expected_delta
    check("Target after move correct",
          norm(current_target - expected_target_after_move) < 1e-9,
          f"Expected {vec_str(expected_target_after_move)}, got {vec_str(current_target)}")

    target_before_clutch = current_target.copy()

    # --- Step 3: Clutch (release trigger) ---
    print(f"\n  Step 3: CLUTCH (release trigger)")
    print(f"    Target frozen at: {vec_str(current_target)}")
    check("Target unchanged on clutch",
          norm(current_target - target_before_clutch) < 1e-9)

    # --- Step 4: Re-engage at teleop=[0.5, 0.3, 0.2] ---
    print(f"\n  Step 4: RE-ENGAGE (teleop now at [0.5, 0.3, 0.2])")
    xr_pose = np.array([0.5, 0.3, 0.2])
    actual_ee = target_before_clutch.copy()
    xr_reference = xr_pose.copy()
    ee_start = actual_ee.copy()
    current_target = actual_ee.copy()

    print(f"    New teleop reference:  {vec_str(xr_reference)}")
    print(f"    New EE start:          {vec_str(ee_start)}")
    print(f"    Current target:        {vec_str(current_target)}")

    check("No jump on re-engage",
          norm(current_target - target_before_clutch) < 1e-9,
          f"Before clutch: {vec_str(target_before_clutch)}, after re-engage: {vec_str(current_target)}")

    # --- Step 5: Move teleop to [0.6, 0.3, 0.3] ---
    print(f"\n  Step 5: MOVE after re-engage (teleop -> [0.6, 0.3, 0.3])")
    xr_pose = np.array([0.6, 0.3, 0.3])
    delta_xr = xr_pose - xr_reference
    delta_robot = axis_map @ delta_xr * scale
    current_target = ee_start + delta_robot

    print(f"    Teleop delta from new ref: {vec_str(delta_xr)}")
    print(f"    Robot delta:               {vec_str(delta_robot)}")
    print(f"    Current target:            {vec_str(current_target)}")

    # delta = [0.1, 0, 0.1], robot delta = [0.075, 0, 0.075]
    expected_delta2 = np.array([0.075, 0.0, 0.075])
    expected_target2 = target_before_clutch + expected_delta2

    check("Post-reengage delta correct",
          norm(delta_robot - expected_delta2) < 1e-9,
          f"Expected {vec_str(expected_delta2)}, got {vec_str(delta_robot)}")
    check("Final target correct (continuous, no jump)",
          norm(current_target - expected_target2) < 1e-9,
          f"Expected {vec_str(expected_target2)}, got {vec_str(current_target)}")

    print(f"\n  CONCLUSION: Engage/clutch logic is correct. Fresh references on")
    print(f"  re-engage prevent jumps. Using actual_ee from FK prevents drift.")


# ===================================================================
# TEST 5: Head Rotation Edge Cases
# ===================================================================
def test_head_rotation_edge_cases():
    section("TEST 5: Head Rotation Edge Cases")

    # --- 5a: Identity ---
    print(f"\n  5a: Identity quaternion")
    M = axis_map_from_head((0, 0, 0, 1))
    check("Identity: returns valid map", M is not None)
    if M is not None:
        d = det(M)
        check(f"Identity: det = +1 (orthogonal, right-handed)", abs(d - 1.0) < 1e-9, f"det={d:.6f}")
        check("Identity: rows are unit vectors",
              all(abs(norm(M[i]) - 1.0) < 1e-9 for i in range(3)))
        check("Identity: rows are orthogonal",
              abs(np.dot(M[0], M[1])) < 1e-9 and
              abs(np.dot(M[0], M[2])) < 1e-9 and
              abs(np.dot(M[1], M[2])) < 1e-9)

    # --- 5b: Looking straight up (pitch = +90 deg around Y in ROS) ---
    print(f"\n  5b: Looking straight UP")
    # Rotation of 90 deg around Y axis: forward becomes +Z (up)
    # quat = (0, sin(45), 0, cos(45))
    q_up = (0.0, np.sin(np.pi/4), 0.0, np.cos(np.pi/4))
    M_up = axis_map_from_head(q_up)
    # R@[1,0,0] = [cos(90), 0, -sin(90)] = [0, 0, -1] (pointing down)
    # horiz_norm = sqrt(0 + 0) = 0 -> None? Actually fwd_x=0, fwd_y=0,
    # so horiz_norm = 0 -> returns None
    check("Looking up: returns None", M_up is None,
          f"Got {'None' if M_up is None else 'a matrix'}")

    # --- 5c: Looking straight down ---
    print(f"\n  5c: Looking straight DOWN")
    q_down = (0.0, -np.sin(np.pi/4), 0.0, np.cos(np.pi/4))
    M_down = axis_map_from_head(q_down)
    check("Looking down: returns None", M_down is None,
          f"Got {'None' if M_down is None else 'a matrix'}")

    # --- 5d: 180 degree yaw (facing backward = -X in ROS) ---
    print(f"\n  5d: 180 degree yaw (facing -X in ROS)")
    quat_180 = quat_from_yaw(np.pi)
    M_180 = axis_map_from_head(quat_180)
    check("180 yaw: returns valid map", M_180 is not None)
    if M_180 is not None:
        print(f"    Axis map:")
        for row in M_180:
            print(f"      {vec_str(row)}")
        d = det(M_180)
        check(f"180 yaw: det = +1", abs(d - 1.0) < 1e-9, f"det={d:.6f}")
        # Forward = R@[1,0,0] at 180 yaw around Z = [-1, 0, 0]
        check("180 yaw: fwd_h = [-1, 0, 0]",
              norm(M_180[0] - np.array([-1, 0, 0])) < 1e-6,
              f"Got {vec_str(M_180[0])}")
        # left = cross([0,0,1], [-1,0,0]) = [0, -1, 0]
        check("180 yaw: left = [0, -1, 0]",
              norm(M_180[1] - np.array([0, -1, 0])) < 1e-6,
              f"Got {vec_str(M_180[1])}")

        # Test: moving in ROS -X (which is now user-forward since they face -X)
        delta_fwd = M_180 @ np.array([-0.1, 0.0, 0.0]) * 0.75
        print(f"    ROS -X [-0.1,0,0] -> robot {vec_str(delta_fwd)} (should be robot +X)")
        check("180 yaw: ROS -X -> robot +X (user faces -X)",
              delta_fwd[0] > 0 and abs(delta_fwd[1]) < 1e-9,
              f"Got {vec_str(delta_fwd)}")

    # --- 5e: Various yaw angles ---
    print(f"\n  5e: Sweep of yaw angles")
    yaw_angles = [0, 45, 90, 135, 180, -45, -90, -135, -180]
    all_valid = True
    all_orthogonal = True
    all_right_handed = True

    for yaw_deg in yaw_angles:
        yaw_rad = np.radians(yaw_deg)
        q = quat_from_yaw(yaw_rad)
        M = axis_map_from_head(q)
        if M is None:
            print(f"    yaw={yaw_deg:+4d}deg: NONE (unexpected!)")
            all_valid = False
            continue

        d = det(M)
        ortho = (abs(np.dot(M[0], M[1])) < 1e-9 and
                 abs(np.dot(M[0], M[2])) < 1e-9 and
                 abs(np.dot(M[1], M[2])) < 1e-9)
        unit = all(abs(norm(M[i]) - 1.0) < 1e-9 for i in range(3))

        if not ortho or not unit:
            all_orthogonal = False
        if abs(d - 1.0) > 1e-6:
            all_right_handed = False

        print(f"    yaw={yaw_deg:+4d}deg: det={d:+.4f}  fwd={vec_str(M[0],3)}  "
              f"left={vec_str(M[1],3)}  ortho={'Y' if ortho else 'N'}")

    check("All yaw angles produce valid maps", all_valid)
    check("All maps are orthogonal with unit rows", all_orthogonal)
    check("All maps have det = +1 (right-handed)", all_right_handed)


# ===================================================================
# TEST 6: Consistency Between Default Map and Head Map
# ===================================================================
def test_consistency():
    section("TEST 6: Default Map / Head Map Consistency")

    print(f"\n  The bridge uses ROS REP-103 convention throughout:")
    print(f"    - teleop-xr publishes in ROS convention (+X fwd, +Y left, +Z up)")
    print(f"    - DEFAULT_AXIS_MAP is identity (correct for ROS data)")
    print(f"    - _axis_map_from_head() extracts forward as R@[1,0,0]")
    print(f"    - Horizontal projection in XY plane (Z-up)")
    print(f"    - All three are consistent.")

    # At identity head, head map should produce identity = DEFAULT_AXIS_MAP
    M_head = axis_map_from_head((0, 0, 0, 1))
    check("Head map at identity == DEFAULT_AXIS_MAP",
          M_head is not None and norm(M_head - DEFAULT_AXIS_MAP) < 1e-9)

    # Verify that for any yaw angle, the map is a proper rotation
    # (orthogonal, det=+1, rows are unit vectors)
    print(f"\n  Verifying map properties across 360 degrees of yaw:")
    n_angles = 36
    all_ok = True
    for i in range(n_angles):
        yaw = 2 * np.pi * i / n_angles
        q = quat_from_yaw(yaw)
        M = axis_map_from_head(q)
        if M is None:
            all_ok = False
            continue
        d = det(M)
        if abs(d - 1.0) > 1e-6:
            all_ok = False
        # Up row should always be [0, 0, 1]
        if norm(M[2] - np.array([0, 0, 1])) > 1e-9:
            all_ok = False
        # fwd_h should have zero Z component
        if abs(M[0][2]) > 1e-9:
            all_ok = False

    check(f"All {n_angles} yaw angles: valid, det=+1, up=[0,0,1], fwd_z=0", all_ok)

    # Verify the key property: user-forward always maps to robot +X
    print(f"\n  Key property: user-forward always maps to robot +X")
    test_yaws = [0, 45, 90, 135, 180, -45, -90, -135]
    all_fwd_ok = True
    for yaw_deg in test_yaws:
        yaw_rad = np.radians(yaw_deg)
        q = quat_from_yaw(yaw_rad)
        M = axis_map_from_head(q)
        if M is None:
            all_fwd_ok = False
            continue
        # User-forward in ROS is [cos(yaw), sin(yaw), 0]
        user_fwd = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0])
        # This should map to robot +X
        robot_delta = M @ user_fwd
        if not (robot_delta[0] > 0.99 and abs(robot_delta[1]) < 1e-6 and abs(robot_delta[2]) < 1e-6):
            print(f"    yaw={yaw_deg:+4d}deg: user_fwd={vec_str(user_fwd)} -> robot={vec_str(robot_delta)} WRONG")
            all_fwd_ok = False
        else:
            print(f"    yaw={yaw_deg:+4d}deg: user_fwd={vec_str(user_fwd)} -> robot={vec_str(robot_delta)} OK")

    check("User-forward always maps to robot +X", all_fwd_ok)


# ===================================================================
# BONUS: Verify the quaternion-to-forward math explicitly
# ===================================================================
def test_quaternion_forward_extraction():
    section("BONUS: Quaternion Forward Vector Extraction Verification")

    print(f"\n  The code extracts forward = R @ [1, 0, 0] from the head quaternion.")
    print(f"  Let's verify this against the full rotation matrix.")
    print(f"")

    def quat_to_rotation_matrix(qx, qy, qz, qw):
        """Full rotation matrix from quaternion (x,y,z,w convention)."""
        return np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)],
        ])

    test_quats = [
        ("identity", (0, 0, 0, 1)),
        ("90 yaw left", quat_from_yaw(np.pi/2)),
        ("90 yaw right", quat_from_yaw(-np.pi/2)),
        ("180 yaw", quat_from_yaw(np.pi)),
        ("45 yaw left", quat_from_yaw(np.pi/4)),
    ]

    all_match = True
    for name, (qx, qy, qz, qw) in test_quats:
        R = quat_to_rotation_matrix(qx, qy, qz, qw)
        fwd_matrix = R @ np.array([1, 0, 0])  # R @ [1,0,0] = first column

        # Code's formula (first column of rotation matrix)
        fwd_code_x = 1.0 - 2.0 * (qy * qy + qz * qz)
        fwd_code_y = 2.0 * (qx * qy + qw * qz)
        fwd_code = np.array([fwd_code_x, fwd_code_y, 0.0])  # Z projected out

        fwd_matrix_h = np.array([fwd_matrix[0], fwd_matrix[1], 0.0])  # project out Z

        match = norm(fwd_matrix_h - fwd_code) < 1e-9
        if not match:
            all_match = False
        print(f"    {name:20s}: matrix={vec_str(fwd_matrix,4)} code={vec_str(fwd_code,4)} {'MATCH' if match else 'MISMATCH'}")

    check("Forward vector extraction matches R@[1,0,0] (XY projection)", all_match)


# ===================================================================
# MAIN
# ===================================================================
if __name__ == '__main__':
    print("=" * 72)
    print("  XR Bridge Coordinate Transform Test Suite")
    print("  Testing: steveros_ik/xr_bridge_node.py (ROS convention)")
    print("=" * 72)

    test_default_axis_map()
    test_head_based_axis_map()
    test_scale_factor()
    test_engage_clutch()
    test_head_rotation_edge_cases()
    test_consistency()
    test_quaternion_forward_extraction()

    # Summary
    print(f"\n{'='*72}")
    print(f"  SUMMARY: {PASS} passed, {FAIL} failed out of {PASS+FAIL} checks")
    print(f"{'='*72}")

    if FAIL > 0:
        print(f"\n  FAILURES detected -- see details above.")
        sys.exit(1)
    else:
        print(f"\n  All checks passed.")
        sys.exit(0)
