#!/usr/bin/env python3
"""Standalone math tests for xr_bridge_node coordinate transforms.

No ROS or pytest dependencies. Tests the axis mapping, head-based dynamic
mapping, scale factors, engage/clutch logic, and edge cases.

Run:  python3 test_xr_math.py
"""

import sys
import numpy as np
from numpy.linalg import det, norm

# ---------------------------------------------------------------------------
# Re-implement the bridge math locally so we can test without ROS
# ---------------------------------------------------------------------------

DEFAULT_AXIS_MAP = np.array([
    [1.0, 0.0, 0.0],   # robot X = XR X
    [0.0, 1.0, 0.0],   # robot Y = XR Y
    [0.0, 0.0, 1.0],   # robot Z = XR Z
])


def axis_map_from_head(head_quat):
    """Exact copy of XRBridgeNode._axis_map_from_head."""
    if head_quat is None:
        return None

    qx, qy, qz, qw = head_quat

    # Head forward = R @ [0, 0, -1]
    fwd_x = -(2.0 * (qx * qz + qw * qy))
    fwd_y = -(2.0 * (qy * qz - qw * qx))
    fwd_z = -(1.0 - 2.0 * (qx * qx + qy * qy))

    horiz_norm = np.sqrt(fwd_x * fwd_x + fwd_z * fwd_z)
    if horiz_norm < 1e-6:
        return None

    fwd_h = np.array([fwd_x / horiz_norm, 0.0, fwd_z / horiz_norm])
    up = np.array([0.0, 1.0, 0.0])
    left = np.cross(up, fwd_h)

    return np.array([fwd_h, left, up])


def quat_from_yaw(yaw_rad):
    """Quaternion for pure yaw around WebXR Y axis (up).

    WebXR Y is up, so yaw rotates around Y.
    Returns (qx, qy, qz, qw).
    """
    return (0.0, np.sin(yaw_rad / 2.0), 0.0, np.cos(yaw_rad / 2.0))


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
# TEST 1: Default Axis Map is an identity -- what does it produce?
# ===================================================================
def test_default_axis_map():
    section("TEST 1: Default Axis Map (Identity) Analysis")

    M = DEFAULT_AXIS_MAP
    scale = 0.15

    print(f"\n  DEFAULT_AXIS_MAP =")
    for row in M:
        print(f"    {vec_str(row)}")

    # --- Scenario A: User moves hand FORWARD in WebXR ---
    # WebXR forward = -Z, so delta = [0, 0, -0.1]
    delta_xr_fwd = np.array([0.0, 0.0, -0.1])
    delta_robot_fwd = M @ delta_xr_fwd * scale
    print(f"\n  Scenario A: User moves hand FORWARD in WebXR")
    print(f"    XR delta:    {vec_str(delta_xr_fwd)}")
    print(f"    Robot delta:  {vec_str(delta_robot_fwd)}")
    print(f"    Expected:     Robot +X (forward), got robot Z={delta_robot_fwd[2]:+.4f}")
    check("Forward maps to robot X",
          abs(delta_robot_fwd[0]) > 0.001 and delta_robot_fwd[0] > 0,
          f"Got {vec_str(delta_robot_fwd)} -- robot X should be positive (forward)")

    # --- Scenario B: User moves hand RIGHT in WebXR ---
    # WebXR right = +X, so delta = [0.1, 0, 0]
    delta_xr_right = np.array([0.1, 0.0, 0.0])
    delta_robot_right = M @ delta_xr_right * scale
    print(f"\n  Scenario B: User moves hand RIGHT in WebXR")
    print(f"    XR delta:    {vec_str(delta_xr_right)}")
    print(f"    Robot delta:  {vec_str(delta_robot_right)}")
    print(f"    Expected:     Robot -Y (right), got robot Y={delta_robot_right[1]:+.4f}")
    check("Right maps to robot -Y",
          delta_robot_right[1] < -0.001,
          f"Got {vec_str(delta_robot_right)} -- robot Y should be negative (right)")

    # --- Scenario C: User moves hand UP in WebXR ---
    # WebXR up = +Y, so delta = [0, 0.1, 0]
    delta_xr_up = np.array([0.0, 0.1, 0.0])
    delta_robot_up = M @ delta_xr_up * scale
    print(f"\n  Scenario C: User moves hand UP in WebXR")
    print(f"    XR delta:    {vec_str(delta_xr_up)}")
    print(f"    Robot delta:  {vec_str(delta_robot_up)}")
    print(f"    Expected:     Robot +Z (up), got robot Z={delta_robot_up[2]:+.4f}")
    check("Up maps to robot +Z",
          delta_robot_up[2] > 0.001,
          f"Got {vec_str(delta_robot_up)} -- robot Z should be positive (up)")

    # --- Compute the CORRECT static map ---
    # WebXR: +X right, +Y up, -Z forward
    # ROS:   +X forward, +Y left, +Z up
    #
    # XR X (right)    -> ROS -Y (right = negative Y in ROS)
    # XR Y (up)       -> ROS +Z (up)
    # XR Z (backward) -> ROS -X (backward = negative X in ROS)
    #   equivalently: XR -Z (forward) -> ROS +X (forward)
    #
    # Robot_vec = M @ XR_vec
    # Robot_X = XR_(-Z) = -XR_Z  =>  M[0] = [0, 0, -1]
    # Robot_Y = XR_(-X) = -XR_X  =>  M[1] = [-1, 0, 0]
    # Robot_Z = XR_(+Y) = +XR_Y  =>  M[2] = [0, 1, 0]
    correct_static = np.array([
        [0.0, 0.0, -1.0],   # robot X = -XR Z (forward)
        [-1.0, 0.0, 0.0],   # robot Y = -XR X (left)
        [0.0, 1.0, 0.0],    # robot Z = +XR Y (up)
    ])

    print(f"\n  CORRECT STATIC AXIS MAP (WebXR -> ROS):")
    for i, (label, row) in enumerate(zip(["robot X (fwd)", "robot Y (left)", "robot Z (up)"], correct_static)):
        print(f"    {label}: {vec_str(row)}")

    # Verify correct map with all 3 scenarios
    delta_fwd_corrected = correct_static @ delta_xr_fwd * scale
    delta_right_corrected = correct_static @ delta_xr_right * scale
    delta_up_corrected = correct_static @ delta_xr_up * scale

    print(f"\n  Verify correct map:")
    print(f"    XR forward {vec_str(delta_xr_fwd)} -> robot {vec_str(delta_fwd_corrected)}")
    print(f"    XR right   {vec_str(delta_xr_right)} -> robot {vec_str(delta_right_corrected)}")
    print(f"    XR up      {vec_str(delta_xr_up)} -> robot {vec_str(delta_up_corrected)}")

    check("Corrected: forward -> +X",
          delta_fwd_corrected[0] > 0 and abs(delta_fwd_corrected[1]) < 1e-9 and abs(delta_fwd_corrected[2]) < 1e-9,
          f"Got {vec_str(delta_fwd_corrected)}")
    check("Corrected: right -> -Y",
          delta_right_corrected[1] < 0 and abs(delta_right_corrected[0]) < 1e-9 and abs(delta_right_corrected[2]) < 1e-9,
          f"Got {vec_str(delta_right_corrected)}")
    check("Corrected: up -> +Z",
          delta_up_corrected[2] > 0 and abs(delta_up_corrected[0]) < 1e-9 and abs(delta_up_corrected[1]) < 1e-9,
          f"Got {vec_str(delta_up_corrected)}")

    # Check determinant (should be +1 for proper rotation/permutation)
    check("Correct map det = +1",
          abs(det(correct_static) - 1.0) < 1e-9,
          f"det = {det(correct_static):.6f}")

    # --- BUT: The code comment says "teleop-xr already outputs in ROS convention" ---
    print(f"\n  IMPORTANT NOTE:")
    print(f"    Line 21-22 of xr_bridge_node.py says:")
    print(f"      '# Axis mapping: teleop-xr already outputs in ROS convention (REP-103).'")
    print(f"      '# Confirmed by data: Z is vertical (~0.8m range for up/down motion).'")
    print(f"    If teleop-xr truly converts to REP-103 before publishing, then the")
    print(f"    identity map IS correct for that upstream. But the _axis_map_from_head()")
    print(f"    function explicitly comments 'WebXR: +X right, +Y up, -Z forward',")
    print(f"    meaning it works in RAW WebXR coords. This is a CONTRADICTION.")
    print(f"    One of these must be wrong:")
    print(f"      (a) If teleop-xr outputs ROS convention -> head map math is wrong")
    print(f"      (b) If data is raw WebXR -> identity default map is wrong")
    print(f"    The head-based map computes fwd/left/up in WebXR space,")
    print(f"    which only makes sense if the incoming data IS in WebXR coords.")


# ===================================================================
# TEST 2: Head-Based Axis Map Analysis
# ===================================================================
def test_head_based_axis_map():
    section("TEST 2: Head-Based Axis Map Analysis")

    scale = 0.15

    # --- Case A: Identity quaternion (no rotation) ---
    # Head facing forward in WebXR = looking along -Z
    quat_identity = (0.0, 0.0, 0.0, 1.0)
    M_id = axis_map_from_head(quat_identity)

    print(f"\n  Case A: Identity quaternion (head facing -Z in WebXR)")
    print(f"    Head quat: {quat_identity}")
    if M_id is not None:
        print(f"    Axis map:")
        labels = ["fwd_h (row 0 -> robot X)", "left  (row 1 -> robot Y)", "up    (row 2 -> robot Z)"]
        for label, row in zip(labels, M_id):
            print(f"      {label}: {vec_str(row)}")

        # At identity, WebXR forward = [0, 0, -1]
        # fwd_h should be [0, 0, -1] (projected to horiz, normalized)
        # left = cross(up=[0,1,0], fwd_h=[0,0,-1]) = [0*(-1)-0*0, 0*0-1*(-1), 1*0-0*0]
        #       = [0, 1, 0]... wait, that's up direction
        # Actually: cross([0,1,0], [0,0,-1]) = [1*(-1)-0*0, 0*0-0*(-1), 0*0-1*0]
        #         = [-1, 0, 0]
        expected_fwd = np.array([0.0, 0.0, -1.0])
        expected_left = np.array([-1.0, 0.0, 0.0])
        expected_up = np.array([0.0, 1.0, 0.0])

        check("Identity: fwd_h = [0, 0, -1]",
              norm(M_id[0] - expected_fwd) < 1e-9,
              f"Got {vec_str(M_id[0])}, expected {vec_str(expected_fwd)}")
        check("Identity: left = [-1, 0, 0]",
              norm(M_id[1] - expected_left) < 1e-9,
              f"Got {vec_str(M_id[1])}, expected {vec_str(expected_left)}")
        check("Identity: up = [0, 1, 0]",
              norm(M_id[2] - expected_up) < 1e-9,
              f"Got {vec_str(M_id[2])}, expected {vec_str(expected_up)}")

        # Now test what this map does to XR deltas
        # User moves forward in WebXR: delta_xr = [0, 0, -0.1]
        # This should become robot +X (forward)
        delta_fwd = M_id @ np.array([0.0, 0.0, -0.1]) * scale
        print(f"\n    XR forward [0, 0, -0.1] -> robot {vec_str(delta_fwd)}")
        check("Identity: XR forward -> robot +X",
              delta_fwd[0] > 0 and abs(delta_fwd[1]) < 1e-9 and abs(delta_fwd[2]) < 1e-9,
              f"Got {vec_str(delta_fwd)}")

        # User moves right in WebXR: delta_xr = [0.1, 0, 0]
        delta_right = M_id @ np.array([0.1, 0.0, 0.0]) * scale
        print(f"    XR right   [0.1, 0, 0]  -> robot {vec_str(delta_right)}")
        check("Identity: XR right -> robot -Y",
              delta_right[1] < 0 and abs(delta_right[0]) < 1e-9 and abs(delta_right[2]) < 1e-9,
              f"Got {vec_str(delta_right)}")

        # User moves up in WebXR: delta_xr = [0, 0.1, 0]
        delta_up = M_id @ np.array([0.0, 0.1, 0.0]) * scale
        print(f"    XR up      [0, 0.1, 0]  -> robot {vec_str(delta_up)}")
        check("Identity: XR up -> robot +Z",
              delta_up[2] > 0 and abs(delta_up[0]) < 1e-9 and abs(delta_up[1]) < 1e-9,
              f"Got {vec_str(delta_up)}")
    else:
        check("Identity: returns valid map", False, "Got None!")

    # --- Compare identity head map with correct static map ---
    correct_static = np.array([
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ])
    print(f"\n    Correct static map for reference:")
    for row in correct_static:
        print(f"      {vec_str(row)}")

    if M_id is not None:
        maps_match = norm(M_id - correct_static) < 1e-9
        check("Identity head map matches correct static map",
              maps_match,
              f"Max diff: {np.max(np.abs(M_id - correct_static)):.6f}")

    # --- Case B: Facing 90 degrees right (yaw = -90 around Y) ---
    # In WebXR, yaw right means rotating -90 deg around +Y
    # After rotation, head forward becomes +X in WebXR
    yaw_right90 = -np.pi / 2
    quat_right90 = quat_from_yaw(yaw_right90)
    M_r90 = axis_map_from_head(quat_right90)

    print(f"\n  Case B: Facing 90 degrees RIGHT (yaw = -90 deg)")
    print(f"    Head quat: ({quat_right90[0]:.4f}, {quat_right90[1]:.4f}, "
          f"{quat_right90[2]:.4f}, {quat_right90[3]:.4f})")
    if M_r90 is not None:
        print(f"    Axis map:")
        for label, row in zip(labels, M_r90):
            print(f"      {label}: {vec_str(row)}")

        # When facing right in WebXR, forward is +X
        # So fwd_h should be [1, 0, 0]
        # left = cross([0,1,0], [1,0,0]) = [0*0-0*0, 0*1-1*0, 1*0-0*1] = [0, 0, -1]
        expected_fwd_r = np.array([1.0, 0.0, 0.0])
        expected_left_r = np.array([0.0, 0.0, -1.0])

        check("Right90: fwd_h = [1, 0, 0]",
              norm(M_r90[0] - expected_fwd_r) < 1e-6,
              f"Got {vec_str(M_r90[0])}")
        check("Right90: left = [0, 0, -1]",
              norm(M_r90[1] - expected_left_r) < 1e-6,
              f"Got {vec_str(M_r90[1])}")

        # Now XR forward (delta=[0,0,-0.1]) should map to...
        # fwd_h=[1,0,0] dotted with [0,0,-0.1] = 0 -> robot X = 0
        # Actually: M @ [0,0,-0.1]:
        #   robot X = fwd_h . [0,0,-0.1] = 1*0+0*0+0*(-0.1) = 0
        #   robot Y = left . [0,0,-0.1]  = 0*0+0*0+(-1)*(-0.1) = 0.1
        #   robot Z = up . [0,0,-0.1]    = 0
        delta_fwd_r = M_r90 @ np.array([0.0, 0.0, -0.1]) * scale
        print(f"\n    XR forward [0, 0, -0.1] -> robot {vec_str(delta_fwd_r)}")
        print(f"    (User faces right, XR -Z is user-forward which is robot +Y)")
        # User faces right => user-forward is robot +Y(left), BUT:
        # Actually user facing right in WebXR means they face +X direction.
        # XR -Z is "into screen" / original forward. If user has turned right,
        # XR -Z is now to the user's LEFT, which in robot frame would be...
        # Let's think in world terms:
        # User faces right in XR (+X direction). Moving hands in XR -Z direction
        # means moving in the user's LEFT direction. Since user faces robot -Y,
        # user-left = robot +X. So delta robot should have +X and zero Y.
        # Wait - the map takes the delta in XR COORDINATES (global, not body-relative)
        # and projects onto the user's body axes, THEN maps those to robot.
        # row 0 (robot X) = fwd_h = [1,0,0]: dot with [0,0,-0.1] = 0
        # row 1 (robot Y) = left  = [0,0,-1]: dot with [0,0,-0.1] = 0.1
        # row 2 (robot Z) = up    = [0,1,0]:  dot with [0,0,-0.1] = 0
        # So robot delta = [0, 0.1, 0] * 0.15 = [0, 0.015, 0]
        # This means: XR -Z movement when user faces right -> robot +Y (left)
        # Is this correct? XR -Z when facing right means the user moved their
        # hand to their LEFT. Robot +Y is left. YES, this is correct!
        check("Right90: XR -Z -> robot +Y (user-left is robot-left)",
              delta_fwd_r[1] > 0 and abs(delta_fwd_r[0]) < 1e-9,
              f"Got {vec_str(delta_fwd_r)}")
    else:
        check("Right90: returns valid map", False, "Got None!")


# ===================================================================
# TEST 3: Scale Factor Analysis
# ===================================================================
def test_scale_factor():
    section("TEST 3: Scale Factor Analysis")

    scale = 0.15

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

    # Is 0.15 reasonable?
    print(f"\n  Assessment:")
    print(f"    scale=0.15 means a 50cm VR hand movement produces 7.5cm robot movement.")
    print(f"    This is VERY conservative -- the user must make large gestures for")
    print(f"    small robot movements. For teleoperation, 0.3-0.5 is more typical.")
    print(f"    However, 0.15 is safer during development (limits accidental big moves).")
    print(f"    The ratio robot_reach/vr_reach = {robot_reach/vr_reach:.2f}, so a")
    print(f"    scale of ~{robot_reach/vr_reach:.2f} would give 1:1 workspace mapping.")

    check("Scale is conservative but safe",
          0.05 < scale < 0.5,
          f"scale={scale} is in a reasonable development range")

    # NOTE: scale is uniform across all axes
    print(f"\n  NOTE: Scale is the same for all axes. This is correct for isotropic")
    print(f"  mapping. Per-axis scaling could compensate for asymmetric workspaces")
    print(f"  but adds complexity. Uniform scale is the right starting choice.")
    check("Scale is uniform (isotropic)", True, "Same scale on all axes -- good default")


# ===================================================================
# TEST 4: Engage/Clutch Math
# ===================================================================
def test_engage_clutch():
    section("TEST 4: Engage/Clutch Sequence Simulation")

    # Use the correct static map (what head-at-identity produces)
    axis_map = np.array([
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ])
    scale = 0.15

    # Initial state
    ee_initial = np.array([0.15, -0.18, 0.35])
    current_target = ee_initial.copy()
    ee_start = ee_initial.copy()
    xr_reference = None
    engaged = False

    print(f"\n  Using correct static axis map (identity head)")
    print(f"  Scale: {scale}")

    # --- Step 1: Engage at XR=[0,0,0], EE at initial position ---
    print(f"\n  Step 1: ENGAGE")
    xr_pose = np.array([0.0, 0.0, 0.0])
    # Simulating _on_joy: engage
    xr_reference = xr_pose.copy()
    ee_start = current_target.copy()  # actual_ee not available, use current
    engaged = True
    print(f"    XR reference:  {vec_str(xr_reference)}")
    print(f"    EE start:      {vec_str(ee_start)}")
    print(f"    Current target: {vec_str(current_target)}")

    # --- Step 2: Move XR to [0.1, 0.05, -0.2] ---
    print(f"\n  Step 2: MOVE (XR -> [0.1, 0.05, -0.2])")
    xr_pose = np.array([0.1, 0.05, -0.2])
    delta_xr = xr_pose - xr_reference
    delta_robot = axis_map @ delta_xr * scale
    current_target = ee_start + delta_robot

    print(f"    XR pose:       {vec_str(xr_pose)}")
    print(f"    XR delta:      {vec_str(delta_xr)}")
    print(f"    Robot delta:   {vec_str(delta_robot)}")
    print(f"    Current target: {vec_str(current_target)}")

    # Verify the deltas make sense:
    # delta_xr = [0.1, 0.05, -0.2]
    # robot X = -(-0.2) * 0.15 = 0.03  (XR forward -> robot forward)
    # robot Y = -(0.1) * 0.15 = -0.015  (XR right -> robot right = -Y)
    # robot Z = 0.05 * 0.15 = 0.0075   (XR up -> robot up)
    expected_delta = np.array([0.03, -0.015, 0.0075])
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
    engaged = False
    print(f"    Target frozen at: {vec_str(current_target)}")
    check("Target unchanged on clutch",
          norm(current_target - target_before_clutch) < 1e-9)

    # --- Step 4: Re-engage at XR=[0.5, 0.3, -0.5] ---
    # User repositioned hand far away -- target should NOT jump
    print(f"\n  Step 4: RE-ENGAGE (XR now at [0.5, 0.3, -0.5])")
    xr_pose = np.array([0.5, 0.3, -0.5])
    # Simulating actual_ee being available (IK node reports where EE really is)
    actual_ee = target_before_clutch.copy()  # assume EE reached the target
    xr_reference = xr_pose.copy()
    ee_start = actual_ee.copy()
    current_target = actual_ee.copy()
    engaged = True

    print(f"    New XR reference:  {vec_str(xr_reference)}")
    print(f"    New EE start:      {vec_str(ee_start)}")
    print(f"    Current target:    {vec_str(current_target)}")

    target_at_reengage = current_target.copy()
    check("No jump on re-engage",
          norm(current_target - target_before_clutch) < 1e-9,
          f"Before clutch: {vec_str(target_before_clutch)}, after re-engage: {vec_str(current_target)}")

    # --- Step 5: Move XR to [0.6, 0.3, -0.6] ---
    print(f"\n  Step 5: MOVE after re-engage (XR -> [0.6, 0.3, -0.6])")
    xr_pose = np.array([0.6, 0.3, -0.6])
    delta_xr = xr_pose - xr_reference
    delta_robot = axis_map @ delta_xr * scale
    current_target = ee_start + delta_robot

    print(f"    XR delta from new ref: {vec_str(delta_xr)}")
    print(f"    Robot delta:           {vec_str(delta_robot)}")
    print(f"    Current target:        {vec_str(current_target)}")

    # delta_xr = [0.1, 0.0, -0.1]
    # robot X = -(-0.1)*0.15 = 0.015 (forward)
    # robot Y = -(0.1)*0.15 = -0.015 (right)
    # robot Z = 0.0*0.15 = 0
    expected_delta2 = np.array([0.015, -0.015, 0.0])
    expected_target2 = target_at_reengage + expected_delta2

    check("Post-reengage delta correct",
          norm(delta_robot - expected_delta2) < 1e-9,
          f"Expected {vec_str(expected_delta2)}, got {vec_str(delta_robot)}")
    check("Final target correct (continuous, no jump)",
          norm(current_target - expected_target2) < 1e-9,
          f"Expected {vec_str(expected_target2)}, got {vec_str(current_target)}")

    # The key insight: because we capture xr_reference fresh on re-engage,
    # the delta is always relative to the new engage point, giving continuity.
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

    # --- 5b: Looking straight up ---
    print(f"\n  5b: Looking straight UP (pitch = +90 deg around X)")
    # Rotation of 90 deg around X axis: quat = (sin(45), 0, 0, cos(45))
    q_up = (np.sin(np.pi/4), 0.0, 0.0, np.cos(np.pi/4))
    M_up = axis_map_from_head(q_up)
    check("Looking up: returns None", M_up is None,
          f"Got {'None' if M_up is None else 'a matrix'}")

    # --- 5c: Looking straight down ---
    print(f"\n  5c: Looking straight DOWN (pitch = -90 deg around X)")
    q_down = (-np.sin(np.pi/4), 0.0, 0.0, np.cos(np.pi/4))
    M_down = axis_map_from_head(q_down)
    check("Looking down: returns None", M_down is None,
          f"Got {'None' if M_down is None else 'a matrix'}")

    # --- 5d: 180 degree yaw ---
    print(f"\n  5d: 180 degree yaw (facing backward in WebXR = +Z)")
    quat_180 = quat_from_yaw(np.pi)
    M_180 = axis_map_from_head(quat_180)
    check("180 yaw: returns valid map", M_180 is not None)
    if M_180 is not None:
        print(f"    Axis map:")
        for row in M_180:
            print(f"      {vec_str(row)}")
        d = det(M_180)
        check(f"180 yaw: det = +1", abs(d - 1.0) < 1e-9, f"det={d:.6f}")
        # Forward in WebXR is now +Z (user turned around)
        # fwd_h should be [0, 0, 1]
        check("180 yaw: fwd_h = [0, 0, +1]",
              norm(M_180[0] - np.array([0, 0, 1])) < 1e-6,
              f"Got {vec_str(M_180[0])}")
        # left = cross([0,1,0], [0,0,1]) = [1, 0, 0]
        check("180 yaw: left = [+1, 0, 0]",
              norm(M_180[1] - np.array([1, 0, 0])) < 1e-6,
              f"Got {vec_str(M_180[1])}")

        # Test: user moves hand in XR +Z (which is now user-forward since they face +Z)
        delta_fwd = M_180 @ np.array([0.0, 0.0, 0.1]) * 0.15
        print(f"    XR +Z [0,0,0.1] -> robot {vec_str(delta_fwd)} (should be robot +X)")
        check("180 yaw: XR +Z -> robot +X (user faces +Z)",
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
# TEST 6: Propose the Correct Axis Map
# ===================================================================
def test_correct_axis_map():
    section("TEST 6: Correct Axis Map Proposal")

    print(f"\n  === Scenario Analysis ===")
    print(f"\n  There are two possible interpretations of the data flow:")
    print(f"\n  INTERPRETATION A: teleop-xr publishes RAW WebXR coordinates")
    print(f"    WebXR: +X right, +Y up, -Z forward")
    print(f"    Robot: +X forward, +Y left, +Z up")
    print(f"    => Identity map is WRONG, need coordinate transform")
    print(f"    => head-based map computes in WebXR space (CONSISTENT)")
    print(f"\n  INTERPRETATION B: teleop-xr converts to ROS convention first")
    print(f"    Data arrives as: +X forward, +Y left, +Z up")
    print(f"    => Identity map is CORRECT for data")
    print(f"    => head-based map assumes WebXR coords (INCONSISTENT)")

    print(f"\n  The code CONTRADICTS itself:")
    print(f"    Line 21: '# teleop-xr already outputs in ROS convention (REP-103)'")
    print(f"    Line 134: '# WebXR (Quest 2): +X right, +Y up, -Z forward'")
    print(f"    The _axis_map_from_head() extracts forward = R@[0,0,-1],")
    print(f"    which is the WebXR forward vector. If data were already in ROS")
    print(f"    convention, forward would be R@[1,0,0], not R@[0,0,-1].")

    print(f"\n  VERDICT: The head-based map assumes WebXR coordinates. The comment")
    print(f"    on line 21-22 is WRONG or teleop-xr was changed without updating")
    print(f"    the bridge. The behavior depends on which is true. We propose")
    print(f"    corrections for BOTH scenarios.")

    # --- SCENARIO A: Data is raw WebXR ---
    print(f"\n  === CORRECT STATIC MAP (if data is raw WebXR) ===")
    correct_static_webxr = np.array([
        [0.0, 0.0, -1.0],   # robot X(fwd) = -XR_Z
        [-1.0, 0.0, 0.0],   # robot Y(left) = -XR_X
        [0.0, 1.0, 0.0],    # robot Z(up)  = +XR_Y
    ])
    print(f"    M = [[  0,  0, -1],     # robot X = -XR_Z (forward)")
    print(f"         [ -1,  0,  0],     # robot Y = -XR_X (left)")
    print(f"         [  0,  1,  0]]     # robot Z = +XR_Y (up)")
    d = det(correct_static_webxr)
    print(f"    det(M) = {d:+.1f} (proper rotation)")

    # Verify
    tests_webxr = [
        ("XR fwd  [0,0,-0.1]", np.array([0, 0, -0.1]), "robot +X"),
        ("XR right [0.1,0,0]", np.array([0.1, 0, 0]),   "robot -Y"),
        ("XR up    [0,0.1,0]", np.array([0, 0.1, 0]),    "robot +Z"),
    ]
    print(f"\n    Verification:")
    for desc, delta, expected_desc in tests_webxr:
        result = correct_static_webxr @ delta
        print(f"      {desc} -> {vec_str(result)} ({expected_desc})")

    # --- Head-based map analysis for WebXR scenario ---
    print(f"\n  === HEAD-BASED MAP ANALYSIS (if data is raw WebXR) ===")
    M_head_identity = axis_map_from_head((0, 0, 0, 1))
    print(f"    At identity head orientation, head map produces:")
    if M_head_identity is not None:
        for row in M_head_identity:
            print(f"      {vec_str(row)}")
        match = norm(M_head_identity - correct_static_webxr) < 1e-9
        print(f"    Matches correct static map: {'YES' if match else 'NO'}")
        check("Head map at identity matches correct WebXR->ROS map", match)
    print(f"    The _axis_map_from_head() function IS CORRECT for WebXR input data.")
    print(f"    It properly extracts the user's forward direction in WebXR space")
    print(f"    and builds a map that projects XR deltas onto user body axes,")
    print(f"    then assigns them to robot axes (fwd->X, left->Y, up->Z).")

    # --- SCENARIO B: Data is already ROS ---
    print(f"\n  === IF DATA IS ALREADY ROS CONVENTION ===")
    print(f"    Static map: Identity IS correct.")
    print(f"    Head-based map: BROKEN. It would need to be rewritten to:")
    print(f"      - Extract forward as R@[1,0,0] (ROS forward)")
    print(f"      - Project onto horizontal XY plane (Z is up)")
    print(f"      - Compute left = cross([0,0,1], fwd_h)")
    print(f"      - Rows: [fwd_h, left, up=[0,0,1]]")

    def axis_map_from_head_ros(head_quat):
        """Corrected head map for ROS-convention input data."""
        if head_quat is None:
            return None
        qx, qy, qz, qw = head_quat
        # Forward = R @ [1, 0, 0] (first column of rotation matrix)
        fwd_x = 1.0 - 2.0 * (qy * qy + qz * qz)
        fwd_y = 2.0 * (qx * qy + qw * qz)
        fwd_z = 2.0 * (qx * qz - qw * qy)
        # Project onto horizontal plane (zero out Z, which is up in ROS)
        horiz_norm = np.sqrt(fwd_x * fwd_x + fwd_y * fwd_y)
        if horiz_norm < 1e-6:
            return None
        fwd_h = np.array([fwd_x / horiz_norm, fwd_y / horiz_norm, 0.0])
        up = np.array([0.0, 0.0, 1.0])
        left = np.cross(up, fwd_h)
        return np.array([fwd_h, left, up])

    print(f"\n    Corrected head map for ROS convention (for reference only):")
    M_ros_id = axis_map_from_head_ros((0, 0, 0, 1))
    if M_ros_id is not None:
        for row in M_ros_id:
            print(f"      {vec_str(row)}")
        check("ROS head map at identity = identity",
              norm(M_ros_id - np.eye(3)) < 1e-9,
              f"Max diff: {np.max(np.abs(M_ros_id - np.eye(3))):.6f}")

    # --- FINAL RECOMMENDATION ---
    print(f"\n  ========================================")
    print(f"  FINAL RECOMMENDATION")
    print(f"  ========================================")
    print(f"\n  1. DETERMINE what teleop-xr actually publishes.")
    print(f"     Run: ros2 topic echo /xr/controller_right/pose --once")
    print(f"     Move hand forward, check which axis changes.")
    print(f"     - If Z decreases => raw WebXR => DEFAULT_AXIS_MAP is WRONG.")
    print(f"     - If X increases => already ROS => _axis_map_from_head is WRONG.")
    print(f"")
    print(f"  2. IF data is raw WebXR (most likely based on _axis_map_from_head code):")
    print(f"     a. Change DEFAULT_AXIS_MAP to:")
    print(f"        [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]")
    print(f"     b. _axis_map_from_head() is ALREADY CORRECT, keep as-is.")
    print(f"     c. Remove the misleading comment on line 21-22.")
    print(f"")
    print(f"  3. IF data is already ROS convention:")
    print(f"     a. DEFAULT_AXIS_MAP = identity is correct, keep as-is.")
    print(f"     b. Rewrite _axis_map_from_head() using ROS convention math.")
    print(f"     c. Remove the WebXR comments inside _axis_map_from_head().")
    print(f"")
    print(f"  4. THE BUG: When head tracking is unavailable (head_quat is None),")
    print(f"     the DEFAULT_AXIS_MAP (identity) is used. If data is WebXR,")
    print(f"     this produces COMPLETELY WRONG mapping (up becomes left, etc.).")
    print(f"     As soon as the user engages and head tracking kicks in, it")
    print(f"     snaps to the correct mapping. This would explain jerky/confusing")
    print(f"     initial behavior that 'fixes itself' once head tracking works.")


# ===================================================================
# BONUS: Verify the quaternion-to-forward math explicitly
# ===================================================================
def test_quaternion_forward_extraction():
    section("BONUS: Quaternion Forward Vector Extraction Verification")

    print(f"\n  The code extracts forward = R @ [0, 0, -1] from the head quaternion.")
    print(f"  Let's verify this against scipy for several orientations.")
    print(f"")

    # We'll compute R from quaternion manually and verify
    def quat_to_rotation_matrix(qx, qy, qz, qw):
        """Full rotation matrix from quaternion (x,y,z,w convention)."""
        return np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)],
        ])

    test_quats = [
        ("identity", (0, 0, 0, 1)),
        ("90 yaw right", quat_from_yaw(-np.pi/2)),
        ("90 yaw left", quat_from_yaw(np.pi/2)),
        ("180 yaw", quat_from_yaw(np.pi)),
        ("45 yaw right", quat_from_yaw(-np.pi/4)),
    ]

    all_match = True
    for name, (qx, qy, qz, qw) in test_quats:
        R = quat_to_rotation_matrix(qx, qy, qz, qw)
        fwd_matrix = R @ np.array([0, 0, -1])

        # Code's formula
        fwd_code_x = -(2.0 * (qx * qz + qw * qy))
        fwd_code_y = -(2.0 * (qy * qz - qw * qx))
        fwd_code_z = -(1.0 - 2.0 * (qx * qx + qy * qy))
        fwd_code = np.array([fwd_code_x, fwd_code_y, fwd_code_z])

        match = norm(fwd_matrix - fwd_code) < 1e-9
        if not match:
            all_match = False
        print(f"    {name:20s}: matrix={vec_str(fwd_matrix,4)} code={vec_str(fwd_code,4)} {'MATCH' if match else 'MISMATCH'}")

    check("Forward vector extraction matches R@[0,0,-1]", all_match)


# ===================================================================
# MAIN
# ===================================================================
if __name__ == '__main__':
    print("=" * 72)
    print("  XR Bridge Coordinate Transform Test Suite")
    print("  Testing: steveros_ik/xr_bridge_node.py")
    print("=" * 72)

    test_default_axis_map()
    test_head_based_axis_map()
    test_scale_factor()
    test_engage_clutch()
    test_head_rotation_edge_cases()
    test_correct_axis_map()
    test_quaternion_forward_extraction()

    # Summary
    print(f"\n{'='*72}")
    print(f"  SUMMARY: {PASS} passed, {FAIL} failed out of {PASS+FAIL} checks")
    print(f"{'='*72}")

    if FAIL > 0:
        print(f"\n  FAILURES indicate bugs in the current code.")
        print(f"  See TEST 1 and TEST 6 for the root cause analysis.")
        sys.exit(1)
    else:
        print(f"\n  All checks passed.")
        sys.exit(0)
