# Cartesian Controller Research: Replacing MoveIt Servo for a 5-DOF Robot Arm

## Problem Statement

MoveIt Servo does not work well with our 5-DOF robot arm (steveros). The default KDL kinematics solver
used by MoveIt struggles with under-actuated arms (< 6 DOF), often failing to find IK solutions
or requiring "approximate solutions" workarounds. We need a lightweight Cartesian controller
that can handle 5-DOF constraints natively.

---

## Candidates Evaluated

### 1. FZI `cartesian_controllers`

- **Repo**: https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers
- **ROS support**: ROS1 (`master` branch) and ROS2 (`ros2` branch)
- **Language**: C++ (ros2_control plugin)
- **IK approach**: Forward Dynamics Compliance Control (FDCC) — uses a simulated "virtual twin"
  of the robot to solve IK via forward dynamics integration. Integrates from joint accelerations
  → velocities → positions, giving delay-free, noise-suppressing behavior.
- **Controllers provided**:
  - `cartesian_motion_controller` — Cartesian pose tracking (visual servoing, teleoperation)
  - `cartesian_force_controller` — Force/torque control with F/T sensor
  - `cartesian_compliance_controller` — Hybrid motion + force (impedance-like)
- **Interface**: Subscribes to `geometry_msgs/PoseStamped` target pose topics
- **Hardware interfaces**: `position` and `velocity` joint command interfaces
- **5-DOF support**: Not explicitly documented. The forward dynamics solver _should_ naturally
  handle under-actuation (it finds a physically plausible configuration even if the full 6D
  target cannot be reached). The pd_gains can potentially be zeroed for the unconstrained
  Cartesian dimension. **Needs testing.**
- **Maturity**: High — presented at ROSCon 2019, used with UR robots, active maintenance
- **Simulation**: Built-in MuJoCo simulation package for testing

### 2. Pinocchio + Pink (Python Differential IK)

- **Repos**:
  - Pinocchio: https://github.com/stack-of-tasks/pinocchio
  - Pink: https://github.com/stephane-caron/pink
- **ROS support**: Pinocchio is available as ROS2 package (`ros-${DISTRO}-pinocchio`).
  Pink is Python-only (`pip install pin-pink`). No native ros2_control integration.
- **Language**: Pinocchio is C++ with Python bindings; Pink is pure Python
- **IK approach**: Task-based differential IK via QP (quadratic programming). Define weighted
  tasks (end-effector pose, joint limits, posture regularization, etc.) and solve a QP at each
  control step. Local/greedy — converges to nearest optimum.
- **5-DOF support**: Excellent. Task-based IK naturally handles under-actuation: you define
  tasks for only the Cartesian dimensions you care about (e.g., position + partial orientation).
  Weighted tasks degrade gracefully when the robot cannot satisfy all constraints.
- **Key features**:
  - `FrameTask` — drive a frame to a target SE3 pose
  - `RelativeFrameTask` — targets relative to another robot frame
  - `PostureTask` — regularize joint configuration
  - `JointCouplingTask`, `LowAccelerationTask`, `DampingTask`
  - Control Barrier Functions for constraint enforcement
- **Maturity**: Well-maintained, good documentation, active community
- **Limitation**: Python-only means it runs outside the real-time ros2_control loop. You'd
  write a ROS2 node that computes IK via Pink and publishes joint commands to a
  `JointTrajectoryController` or `ForwardCommandController`.
- **Reference ROS2 template**: https://github.com/antonioarbues/ik-template (Pinocchio IK
  for leader-follower teleoperation)

### 3. CRISP Controllers (TUM)

- **Repo**: https://github.com/utiasDSL/crisp_controllers
- **ROS support**: ROS2 (ros2_control plugin)
- **Language**: C++ with Python/Gymnasium interfaces
- **IK approach**: Pinocchio-based rigid body dynamics for Cartesian Impedance Control and
  Operational Space Control
- **Controllers provided**:
  - Cartesian Impedance Controller
  - Operational Space Controller
- **Hardware interface**: **Joint torque** interface (effort commands)
- **5-DOF support**: Not explicitly documented. Pinocchio-based, so mathematically should
  handle it.
- **Maturity**: Newer project (2025), validated on Franka FR3 and KUKA iiwa14.
  Designed for learning-based policy deployment and teleoperation.
- **Limitation**: Requires torque-controlled robot. If steveros uses position/velocity
  interfaces (likely with Robstride motors), this is **not directly compatible**.

### 4. ICube `cartesian_controllers_ros2`

- **Repo**: https://github.com/ICube-Robotics/cartesian_controllers_ros2
- **ROS support**: ROS2 (ros2_control plugin)
- **Language**: C++
- **IK approach**: Variable Impedance Control (VIC) framework with pluggable implementations.
  Uses `kinematics_interface` for robot model (can use KDL or Pinocchio backend).
- **Controllers provided**:
  - `cartesian_vic_controller` — main VIC controller
  - `cartesian_vic_teleop_controller` — bilateral teleoperation
  - `cartesian_vic_servo` — admittance control via MoveIt2 Servo backend
  - `cartesian_state_broadcaster` — broadcasts Cartesian state
- **5-DOF support**: Uses `kinematics_interface` which supports Pinocchio backend via
  `kinematics_interface_pinocchio`. Should handle 5 DOF with damped least-squares.
- **Maturity**: Active, from ICube Lab (University of Strasbourg)

### 5. `kinematics_interface_pinocchio` (Drop-in KDL Replacement)

- **Repo**: https://github.com/ros-controls/kinematics_interface (official) and
  https://github.com/justagist/kinematics_interface_pinocchio
- **ROS support**: ROS2 — `apt install ros-${DISTRO}-kinematics-interface-pinocchio`
- **What it does**: Pinocchio-based plugin for the `kinematics_interface` API. Drop-in
  replacement for the KDL backend. Supports damped least-squares Jacobian inversion.
- **Use case**: Can be used with _any_ ros2_control controller that uses `kinematics_interface`,
  including FZI and ICube controllers. Better singularity handling than KDL.

---

## Comparison Matrix

| Criterion                      | FZI cartesian_controllers | Pinocchio + Pink     | CRISP (TUM)          | ICube VIC            |
|-------------------------------|--------------------------|----------------------|----------------------|----------------------|
| **ROS2 native**               | Yes (ros2_control)       | Manual integration   | Yes (ros2_control)   | Yes (ros2_control)   |
| **Language**                  | C++                      | Python               | C++                  | C++                  |
| **Real-time capable**         | Yes                      | No (Python)          | Yes                  | Yes                  |
| **Joint interface**           | Position / Velocity      | Any (external)       | **Torque only**      | Position / Velocity  |
| **5-DOF handling**            | Implicit (physics-based) | Explicit (task-based) | Implicit             | Via kinematics_interface |
| **Requires MoveIt**           | No                       | No                   | No                   | Optional (servo variant) |
| **Force/compliance control**  | Yes                      | No                   | Yes (impedance)      | Yes (VIC)            |
| **Complexity to integrate**   | Low (ros2_control plugin)| Medium (custom node) | Low but needs torque | Medium               |
| **Maturity / community**      | High                     | High                 | Medium               | Medium               |
| **Simulation included**       | Yes (MuJoCo)             | Examples only        | Yes (MuJoCo)         | No                   |
| **Documentation quality**     | Good                     | Good                 | Good (paper)         | Moderate             |

---

## Decision Tree

```
                    ┌─────────────────────────────────┐
                    │ Need Cartesian control for       │
                    │ 5-DOF arm, replacing MoveIt Servo│
                    └───────────────┬─────────────────┘
                                    │
                    ┌───────────────▼─────────────────┐
                    │ Does your robot have a           │
                    │ torque (effort) interface?        │
                    └──┬─────────────────────────┬────┘
                       │ YES                     │ NO
                       ▼                         ▼
              ┌────────────────┐    ┌────────────────────────────┐
              │ CRISP (TUM)    │    │ Do you need real-time      │
              │ Pinocchio-based│    │ C++ performance?           │
              │ impedance ctrl │    └──┬─────────────────────┬───┘
              └────────────────┘       │ YES                 │ NO
                                       ▼                     ▼
                          ┌──────────────────┐   ┌──────────────────────┐
                          │ Do you need       │   │ Pinocchio + Pink     │
                          │ force/compliance  │   │                      │
                          │ control?          │   │ • Best 5-DOF support │
                          └──┬────────────┬──┘   │ • Task-based IK      │
                             │YES         │NO    │ • Python ROS2 node   │
                             ▼            ▼      │ • Publishes joints   │
                  ┌───────────────┐ ┌─────────┐  │   to ros2_control    │
                  │ FZI Cartesian │ │ FZI      │  └──────────────────────┘
                  │ Compliance    │ │ Motion   │
                  │ Controller    │ │Controller│
                  │               │ │          │
                  │ + pinocchio   │ │+ pinocchio│
                  │   kinematics  │ │  kinematics│
                  │   interface   │ │  interface │
                  └───────────────┘ └──────────┘
```

---

## Recommendation for steveros (5-DOF, Position/Velocity Interface)

### Primary recommendation: **FZI `cartesian_controllers`** (motion controller)

**Why:**
1. **Native ros2_control plugin** — integrates directly with our existing ros2_control stack
2. **Position/velocity interface** — matches steveros' Robstride motor interface
3. **No MoveIt dependency** — lightweight, direct Cartesian control via topic subscription
4. **Physics-based solver** — the FDCC approach should gracefully handle under-actuation
   (5 DOF) by finding the best physically plausible configuration
5. **Battle-tested** — used with UR robots, presented at ROSCon, actively maintained
6. **Built-in MuJoCo simulation** — we can test in simulation before hardware

**Integration plan:**
1. Clone `cartesian_controllers` (ros2 branch) into workspace
2. Configure `cartesian_motion_controller` with steveros' 5 joints
3. Swap `kinematics_interface_kdl` for `kinematics_interface_pinocchio` for better
   singularity handling
4. Set pd_gains: zero out the Cartesian dimension we can't control (likely `rot_z` or
   whichever axis maps to the missing 6th DOF)
5. Test in MuJoCo simulation first
6. Publish target poses to the controller's target topic

### Secondary recommendation: **Pinocchio + Pink** (if FZI doesn't handle 5-DOF well)

**Why as fallback:**
1. Task-based IK gives **explicit control** over which Cartesian dimensions to solve for
2. Can define a `FrameTask` for position (3D) + partial orientation (2D) = 5 constraints
   matching our 5 DOF exactly
3. QP solver handles joint limits, velocity limits, and singularities natively
4. Easy to prototype in Python

**Trade-off:** Requires writing a custom ROS2 node (not a ros2_control plugin), and Python
adds latency. For teleoperation or visual servoing at ~30-100 Hz, this is usually acceptable.

---

## Key Reference Repositories

| Repository | Purpose |
|-----------|---------|
| [FZI cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers) | Primary candidate — C++ Cartesian controllers for ros2_control |
| [Pink](https://github.com/stephane-caron/pink) | Fallback — Python task-based IK with Pinocchio |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | Core dynamics library used by Pink and CRISP |
| [kinematics_interface_pinocchio](https://github.com/justagist/kinematics_interface_pinocchio) | Drop-in KDL replacement for ros2_control |
| [ik-template (ROS2 + Pinocchio)](https://github.com/antonioarbues/ik-template) | Reference for Pinocchio IK in ROS2 teleoperation |
| [CRISP controllers](https://github.com/utiasDSL/crisp_controllers) | Torque-based Cartesian impedance (if we switch to torque interface) |
| [ICube cartesian_controllers_ros2](https://github.com/ICube-Robotics/cartesian_controllers_ros2) | VIC framework with pluggable kinematics backend |
| [ROSCon 2019 FZI Talk (PDF)](https://roscon.ros.org/2019/talks/roscon2019_cartesiancontrollers.pdf) | Detailed explanation of FDCC approach |

---

## Next Steps

1. Clone FZI `cartesian_controllers` (ros2 branch) into steveros workspace
2. Configure for steveros' 5 joints and test in MuJoCo simulation
3. If 5-DOF handling is insufficient, prototype with Pink in a Python ROS2 node
4. Evaluate latency and tracking performance for teleoperation use case
