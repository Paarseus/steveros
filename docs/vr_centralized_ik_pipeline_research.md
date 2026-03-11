# VR Teleoperation with Centralized IK Pipeline: Research & Architecture Guide

## Context: SteveROS (K-Scale KBot, 20 DOF Humanoid)

This document covers the standard pipeline architectures used across major humanoid robotics codebases for VR teleoperation with centralized inverse kinematics, with specific recommendations for integrating **teleop-xr** into SteveROS.

---

## 1. The Standard VR Teleoperation Pipeline

Across all major humanoid codebases (Unitree, PAL Robotics, LeRobot, NVIDIA Isaac, PickNik), the pipeline follows the same fundamental stages:

```
 VR Headset (XR Device)
       |
       | 6DoF hand poses + button states (WebXR / OpenXR)
       v
 [Stage 1] XR State Extraction
       |
       | SE3 poses (position + quaternion) per hand
       v
 [Stage 2] Retargeting / Mapping
       |
       | Scaled delta poses or absolute target poses in robot frame
       v
 [Stage 3] CENTRALIZED IK SOLVER          <-- The heart of the pipeline
       |
       | Joint angle commands (q_target)
       v
 [Stage 4] Joint Smoothing / Filtering
       |
       | Filtered joint commands
       v
 [Stage 5] Joint Controller (PD / Trajectory)
       |
       | Torque / position commands
       v
 Robot Hardware (motors via CAN, DDS, etc.)
```

### Why Centralize IK?

- **Single source of truth** for the robot's kinematic model (one URDF, one solver)
- **Whole-body coordination** - arms, torso, and (optionally) legs solved simultaneously
- **Constraint unification** - joint limits, self-collision, workspace bounds all enforced in one place
- **Solver-agnostic** - swap between optimization-based IK, learned policies, or analytical solvers without touching the rest of the pipeline
- **Data collection** - a single IK node can log both task-space targets and joint-space solutions for imitation learning

---

## 2. How Major Codebases Implement This

### 2.1 Unitree `xr_teleoperate` (H1/G1 Humanoids)

**Architecture:** Python-native, no ROS dependency in core loop

| Component | Implementation |
|-----------|---------------|
| XR Input | Vuer (televuer module) over WebSocket |
| IK Solver | **Pinocchio 3.1 + CasADi** (optimization-based) |
| Robot Model | URDF loaded into Pinocchio |
| Smoothing | Weighted moving average filter on joint angles |
| Communication | DDS (unitree_sdk2_python) |
| Sim | Isaac Lab for testing |

**IK Details:**
- `robot_arm_ik.py` constructs a CasADi optimization problem
- Objective: minimize SE3 pose error between desired and current end-effector
- Pinocchio computes FK and Jacobians; CasADi handles the nonlinear optimization (IPOPT solver)
- Left and right arms solved independently but through the **same solver class**
- Joint limits enforced as hard constraints in the optimization

**Key Insight:** Unitree's approach is *centralized per-arm* - the same `robot_arm_ik.py` class handles both arms, with the URDF defining which kinematic chain to solve. Lower body uses a separate locomotion controller.

### 2.2 teleop-xr / qrafty-ai (WebXR + PyRoki)

**Architecture:** WebXR frontend + Python backend with whole-body IK

| Component | Implementation |
|-----------|---------------|
| XR Input | WebXR API (zero-install, browser-based) |
| IK Solver | **PyRoki** (JAX-based, differentiable, Levenberg-Marquardt) |
| Collision | **Ballpark** (sphere decomposition for collision geometry) |
| Robot Model | URDF loaded into PyRoki |
| Interfaces | Generic Python API, ROS2 Interface, dora-rs Interface |

**IK Details:**
- PyRoki formulates IK as a nonlinear least-squares problem
- Cost terms are modular and composable:
  - `PoseCost` - SE3 log-space error (geodesic distance) for end-effector targets
  - `JointLimitCost` - soft penalty near joint limits
  - `JointRegularizationCost` - minimal displacement from current config
  - `SelfCollisionCost` - sphere-based self-collision avoidance
  - `WorldCollisionCost` - obstacle avoidance
  - `ManipulabilityCost` - stay away from singularities
- Levenberg-Marquardt solver with automatic block-sparse Jacobian computation
- Runs on CPU for real-time single-robot control, GPU for batch/offline
- **Whole-body IK** by default - all joints solved simultaneously

**Key Insight:** teleop-xr's approach is *truly centralized whole-body IK*. Unlike Unitree's per-arm approach, PyRoki solves all kinematic chains simultaneously, enabling coordination (e.g., bimanual manipulation, torso lean to extend reach).

### 2.3 PAL Robotics (TIAGo, TALOS)

**Architecture:** ROS2 controller plugin

| Component | Implementation |
|-----------|---------------|
| XR Input | WebXR API via browser |
| IK Solver | **Dedicated VR teleop controller** (ros2_control plugin) |
| Collision | Built-in self-collision avoidance in IK loop |
| Robot Model | URDF via robot_state_publisher |

**Key Insight:** PAL's approach bakes IK into a ros2_control controller. The VR pose targets come in, and the controller internally runs IK + self-collision checks + joint smoothing, outputting joint commands directly. This is the most "centralized" approach - the IK is a controller, not a separate node.

### 2.4 MoveIt Servo (PickNik / ROS2 Standard)

**Architecture:** ROS2 node wrapping MoveIt's IK + collision pipeline

| Component | Implementation |
|-----------|---------------|
| Input | `geometry_msgs/TwistStamped` or `geometry_msgs/PoseStamped` |
| IK Solver | **pick_ik** (or KDL, or any MoveIt IK plugin) |
| Collision | MoveIt planning scene (self + world) |
| Output | `trajectory_msgs/JointTrajectory` via controller action |

**IK Details:**
- MoveIt Servo runs an inner loop at up to 1000+ Hz
- Accepts twist commands (velocity) or pose commands (position)
- Uses the configured MoveIt IK solver (pick_ik in your case) for Cartesian-to-joint conversion
- Enforces joint limits, velocity limits, singularity detection, and collision avoidance
- Outputs to ros2_control joint trajectory controllers

**Key Insight:** This is the "ROS standard" approach. Already partially set up in SteveROS (pick_ik configured, MoveIt installed). However, MoveIt Servo solves **per planning group** (per-arm), not whole-body.

### 2.5 LeRobot / LeVR (HuggingFace)

**Architecture:** Python-native pipeline with modular processors

| Component | Implementation |
|-----------|---------------|
| XR Input | VR headset via LeVR extension |
| IK Solver | Modular kinematics processor (`InverseKinematicsEEToJoints`) |
| Robot Model | URDF-based calibrated kinematics |
| Output | Joint commands via Robot class interface |

**Key Insight:** LeRobot treats IK as a *processor step* in a data pipeline. The same pipeline that does teleoperation also does data collection and policy inference. This is the most "ML-friendly" centralized IK - designed from the ground up for imitation learning.

### 2.6 NVIDIA Isaac Lab / GR00T

**Architecture:** Simulation-first with CloudXR streaming

| Component | Implementation |
|-----------|---------------|
| XR Input | Apple Vision Pro / Meta Quest via CloudXR |
| IK Solver | GPU-accelerated whole-body IK in Isaac Sim |
| Robot Model | USD/URDF in Isaac Sim |
| Output | Joint commands to sim or real robot |

**Key Insight:** NVIDIA's approach is the most compute-heavy but also the most capable - GPU-accelerated whole-body IK with full physics simulation in the loop. Designed for data collection at scale.

---

## 3. Comparison: Which Centralized IK Approach for SteveROS?

| Criteria | MoveIt Servo + pick_ik | teleop-xr + PyRoki | Unitree-style (Pinocchio+CasADi) |
|----------|----------------------|--------------------|---------------------------------|
| **Already in SteveROS** | Partial (pick_ik configured) | No | No |
| **Whole-body IK** | No (per-group) | Yes | No (per-arm) |
| **Real-time capable** | Yes (1000+ Hz) | Yes (CPU real-time) | Yes (~100 Hz) |
| **Self-collision** | Yes (MoveIt scene) | Yes (Ballpark spheres) | Manual |
| **ROS2 integration** | Native | Available (ROS2 interface) | Requires bridge |
| **VR interface** | Needs separate node | Built-in (WebXR) | Needs Vuer |
| **ML/data collection** | Extra work | Moderate | Moderate |
| **Orientation control** | Yes | Yes | Yes |
| **Setup complexity** | Low (already have MoveIt) | Medium (install pyroki, ballpark) | Medium (install pinocchio, casadi) |

---

## 4. Recommended Architecture for SteveROS with teleop-xr

Since you want to use **teleop-xr in teleop mode** with **centralized IK**, here is the recommended architecture:

### Option A: teleop-xr with PyRoki as Centralized IK (Recommended)

This is the cleanest approach - teleop-xr already uses PyRoki for whole-body IK internally.

```
 Meta Quest (VR Headset)
       |
       | WebXR hand poses + buttons (zero-install, browser)
       v
 teleop-xr Server (Python)
       |
       | Raw 6DoF poses per hand
       v
 PyRoki Whole-Body IK Solver  <-- CENTRALIZED IK
       |  - Loads SteveROS URDF (kbot.urdf)
       |  - PoseCost for left_hand + right_hand targets
       |  - JointLimitCost (from URDF limits)
       |  - SelfCollisionCost (Ballpark sphere decomposition)
       |  - JointRegularizationCost (smooth motion)
       |  - ManipulabilityCost (avoid singularities)
       |  - Levenberg-Marquardt solver
       |
       | 10 joint angles (5 per arm)
       v
 ROS2 Bridge Node
       |
       | sensor_msgs/JointState or trajectory_msgs/JointTrajectory
       v
 ros2_control (SteveROSHardware)
       |
       | right_arm_controller + left_arm_controller
       v
 Robstride Motors (CAN bus)
```

**Advantages:**
- Truly centralized: one solver for both arms (and potentially torso/legs later)
- Whole-body self-collision avoidance
- teleop-xr handles the VR interface, video streaming, and visualization
- PyRoki's modular cost terms let you add constraints incrementally
- Same pipeline works for data collection (log task-space + joint-space)

**What you need to build:**
1. Configure PyRoki with your KBot URDF
2. Define end-effector frames (right wrist, left wrist) in the URDF
3. Write a thin ROS2 bridge node that takes PyRoki's joint output and publishes to ros2_control
4. (Optional) Add a clutch mechanism via VR grip buttons

### Option B: teleop-xr (Teleop Mode) + MoveIt Servo as Centralized IK

Use teleop-xr only for VR input, and route poses through MoveIt Servo.

```
 Meta Quest (VR Headset)
       |
       | WebXR hand poses + buttons
       v
 teleop-xr Server (Teleop Mode - raw state only)
       |
       | Raw 6DoF poses per hand
       v
 ROS2 Bridge Node (custom)
       |
       | geometry_msgs/PoseStamped per hand
       v
 MoveIt Servo (per planning group)
       |  - pick_ik solver for right_arm
       |  - pick_ik solver for left_arm
       |  - MoveIt planning scene for collision
       |
       | JointTrajectory commands
       v
 ros2_control (SteveROSHardware)
       |
       v
 Robstride Motors
```

**Advantages:**
- Leverages existing MoveIt + pick_ik setup in SteveROS
- Mature collision checking and singularity handling
- Less new code to write

**Disadvantages:**
- Not truly "centralized" - each arm is solved independently
- No whole-body coordination
- Need to configure MoveIt Servo (not yet done in SteveROS)
- pick_ik's `rotation_scale: 0.0` means no orientation control (needs reconfiguration)

---

## 5. Standard Pipeline Stages in Detail

### Stage 1: XR State Extraction

teleop-xr handles this entirely. It provides:
- Left/right hand 6DoF poses (position + quaternion) in headset frame
- Button states (grip, trigger, thumbstick)
- Head pose (for potential torso mapping)

### Stage 2: Retargeting / Frame Mapping

Transform VR hand poses into robot base frame:

```python
# Pseudocode for retargeting
T_vr_to_robot = calibrated_transform()  # VR space -> robot base frame
T_hand_vr = get_hand_pose_from_teleop_xr()
T_hand_robot = T_vr_to_robot @ T_hand_vr

# Optional: scale workspace
T_hand_robot.translation *= workspace_scale_factor  # e.g., 0.8 for smaller robot

# Optional: clutch (only track when grip held)
if grip_button_pressed:
    ik_target = T_hand_robot
else:
    ik_target = current_ee_pose  # hold position
```

Key decisions:
- **Absolute vs. delta mapping:** Absolute maps VR hand directly to robot EE. Delta maps VR hand *movement* to robot EE movement. Delta is more common because it allows clutching and workspace resizing.
- **Workspace scaling:** Your robot's arms are probably different length than a human's. Scale the VR workspace to match the robot's reachable workspace.
- **Clutch mechanism:** Essential for safety. Grip button = "engage", release = "freeze robot".

### Stage 3: Centralized IK Solver

This is the heart of the pipeline. The solver takes task-space targets and outputs joint-space commands.

**PyRoki formulation for SteveROS:**

```python
import pyroki as pk

# Load robot
robot = pk.Robot.from_urdf("steveros.urdf")

# Define cost terms
costs = [
    pk.costs.PoseCost(
        robot=robot,
        link_name="right_wrist_link",
        target_pose=T_right_hand_robot,
        position_weight=10.0,
        orientation_weight=5.0,
    ),
    pk.costs.PoseCost(
        robot=robot,
        link_name="left_wrist_link",
        target_pose=T_left_hand_robot,
        position_weight=10.0,
        orientation_weight=5.0,
    ),
    pk.costs.JointLimitCost(robot=robot, weight=1.0),
    pk.costs.SelfCollisionCost(robot=robot, weight=2.0),
    pk.costs.JointRegularizationCost(
        robot=robot,
        target_config=current_joint_config,
        weight=0.001,  # small: prefer minimal motion
    ),
]

# Solve
result = pk.solve(robot, costs, q_init=current_joint_config)
q_target = result.q  # joint angles for all 10 arm joints
```

### Stage 4: Joint Smoothing / Filtering

Raw IK solutions can be jerky. Apply filtering before sending to hardware:

```python
# Exponential moving average (simplest)
alpha = 0.3  # 0.0 = no change, 1.0 = instant
q_smoothed = alpha * q_target + (1 - alpha) * q_previous

# Or: weighted moving average (Unitree approach)
# Or: low-pass Butterworth filter
# Or: rate limiter (cap max joint velocity per timestep)
```

### Stage 5: Joint Controller

SteveROS already has this: `right_arm_controller` and `left_arm_controller` (JointTrajectoryController via ros2_control).

The bridge node publishes `trajectory_msgs/JointTrajectory` messages with the smoothed joint targets and a short time_from_start (e.g., 20-50ms for 20-50 Hz teleop).

---

## 6. Frequency / Timing Considerations

| Stage | Typical Rate | SteveROS Target |
|-------|-------------|-----------------|
| VR tracking | 72-120 Hz (headset native) | 72 Hz (Quest) |
| XR state publish | 30-90 Hz (network limited) | 50 Hz |
| IK solve | 100-1000 Hz (solver dependent) | 50-100 Hz |
| Joint command publish | 50-100 Hz | 50 Hz |
| ros2_control loop | 100 Hz (configured) | 100 Hz |
| Motor command (CAN) | 100 Hz | 100 Hz |

The bottleneck is typically the network between VR headset and the IK solver. 50 Hz is sufficient for smooth arm teleoperation.

---

## 7. Configuration Changes Needed in SteveROS

### If Using Option A (teleop-xr + PyRoki):

1. **URDF end-effector frames**: Verify that `kbot.urdf` has named frames at the wrist tips (or add them as fixed-joint links)
2. **pick_ik `rotation_scale`**: Change from `0.0` to `0.5` or `1.0` in `kinematics.yaml` if you also want MoveIt planning with orientation
3. **New package**: `steveros_teleop` - a ROS2 package containing:
   - `teleop_bridge_node.py` - subscribes to teleop-xr output, publishes JointTrajectory
   - `config/teleop_params.yaml` - workspace scaling, clutch config, filter params
4. **Launch file**: `teleop.launch.py` that starts teleop-xr server + bridge node + controllers

### If Using Option B (teleop-xr + MoveIt Servo):

1. **MoveIt Servo config**: Create `servo_params.yaml` with:
   - `robot_link_command_frame: base`
   - `command_in_type: pose_stamped`
   - Publish rate, singularity threshold, collision margins
2. **pick_ik reconfiguration**: Set `rotation_scale: 0.5` for orientation control
3. **Bridge node**: Convert teleop-xr poses to `PoseStamped` for MoveIt Servo input

---

## 8. Emerging Trend: Neural Teleoperation (Future Path)

The cutting edge (2025-2026) is replacing the IK+PD pipeline with **learned policies**:

1. Collect demonstrations using the IK pipeline above
2. Train an RL/imitation learning policy in simulation (Isaac Lab, MuJoCo)
3. The learned policy directly maps VR inputs to joint commands
4. Benefits: handles external forces, adapts to users, produces natural motion
5. SteveROS is well-positioned for this: MuJoCo sim already set up

This is the path taken by NVIDIA GR00T, Boston Dynamics Atlas (2026), and the "Learning Adaptive Neural Teleoperation for Humanoid Robots" paper.

---

## Sources

- [teleop-xr (qrafty-ai)](https://github.com/qrafty-ai/teleop_xr)
- [PyRoki: A Modular Toolkit for Robot Kinematic Optimization](https://github.com/chungmin99/pyroki)
- [PyRoki Paper (IROS 2025)](https://ar5iv.labs.arxiv.org/html/2505.03728)
- [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
- [LeRobot (HuggingFace)](https://github.com/huggingface/lerobot)
- [LeVR: VR Teleoperation for LeRobot](https://arxiv.org/html/2509.14349v1)
- [MoveIt Servo Documentation](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [pick_ik Tutorial](https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html)
- [PAL Robotics VR Teleoperation](https://docs.pal-robotics.com/edge/manipulation/vr-teleoperation)
- [XRoboToolkit Paper](https://arxiv.org/html/2508.00097)
- [NVIDIA Isaac Lab Teleoperation](https://developer.nvidia.com/blog/streamline-robot-learning-with-whole-body-control-and-enhanced-teleoperation-in-nvidia-isaac-lab-2-3/)
- [Learning Adaptive Neural Teleoperation for Humanoid Robots](https://arxiv.org/abs/2511.12390)
- [CHILD: Whole-Body Humanoid Teleoperation System](https://arxiv.org/pdf/2508.00162)
- [SpesRobotics/teleop (original)](https://github.com/SpesRobotics/teleop)
- [leggedrobotics/unity_ros_teleoperation](https://github.com/leggedrobotics/unity_ros_teleoperation)
- [antonioarbues/ik-template (Pinocchio ROS2)](https://github.com/antonioarbues/ik-template)
- [GMR: General Motion Retargeting (ICRA 2026)](https://github.com/YanjieZe/GMR)
