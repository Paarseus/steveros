# Production Humanoid Architecture Research — SteveROS (KBot, 20-DOF, 5-DOF Arms)

> Research compiled 2026-03-09 from MoveIt2 docs, ROS Discourse, production humanoid teardowns, and academic papers.

---

## TL;DR — The Stack You Need

```
┌─────────────────────────────────────────────────────┐
│  LAYER 5: Task Planning                              │
│  BehaviorTree.CPP + MoveIt Task Constructor          │
├─────────────────────────────────────────────────────┤
│  LAYER 4: Motion Planning                            │
│  MoveIt2 (OMPL RRTConnect + STOMP) + MoveIt Servo    │
│  IK: pick_ik (weighted, 5-DOF native)                │
├─────────────────────────────────────────────────────┤
│  LAYER 3: Whole-Body Control (future)                │
│  Pinocchio + TSID (QP-based, priority tasks)         │
├─────────────────────────────────────────────────────┤
│  LAYER 2: ros2_control Controllers                   │
│  joint_trajectory_controller (current)               │
│  + admittance_controller (when F/T sensor added)     │
├─────────────────────────────────────────────────────┤
│  LAYER 1: Hardware Interface                         │
│  steveros_hardware.cpp (CAN, MIT mode)               │
└─────────────────────────────────────────────────────┘
```

**Reference architecture:** PAL Robotics TALOS — the only production humanoid running MoveIt2 + ros2_control end-to-end.

---

## 1. Joint Planning

### Recommended: OMPL via MoveIt2

| Use Case | Planner |
|----------|---------|
| General point-to-point | **RRTConnect** (default, <200ms, 95% success) |
| Smooth trajectories | **RRTConnect → STOMP chain** (MoveIt2 pipeline supports this natively) |
| Deterministic moves | **Pilz PTP** (trapezoidal velocity profile, repeatable) |
| GPU-accelerated (future) | **NVIDIA cuMotion** (80x speedup, Developer Preview) |

**Avoid:** CHOMP (local minima, jerky near obstacles — STOMP is strictly better).

### Parallel Planning
MoveIt2 can run multiple planners simultaneously and return the best result. Consistently outperforms any single planner.

---

## 2. Cartesian Planning & the 5-DOF Problem

### The Core Issue

Your 5-DOF arms (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist) **cannot independently control all 6 Cartesian DOF** (x, y, z, roll, pitch, yaw). One orientation axis is unconstrained — typically roll around the tool approach vector.

**What breaks with 5-DOF:**
- KDL IK solver → fails completely (do NOT use)
- `computeCartesianPath()` → fails at intermediate IK solves
- Pilz LIN/CIRC → sporadic IK failures (GitHub #3289)
- Full 6-DOF pose goals in OMPL → over-constrained, no solution

### IK Solver: pick_ik (RECOMMENDED)

| Solver | Speed | 5-DOF | Cost Functions | Verdict |
|--------|-------|-------|----------------|---------|
| KDL | <1ms | **NO** | No | Avoid |
| TRAC-IK | <1ms | Workarounds | No | OK for 6-DOF |
| **pick_ik** | 10-50ms | **Native** | **Yes** | **Best for 5-DOF** |
| IKFast | <0.01ms | Yes (analytical) | No | Best speed, hard toolchain |
| Pinocchio/Pink | ~us | Excellent | Yes (QP) | Best for whole-body |

**pick_ik configuration for 5-DOF arms:**
```yaml
left_arm:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.05
  position_scale: 1.0
  rotation_scale: 0.5        # Soft orientation constraint
  position_threshold: 0.001   # 1mm accuracy
  orientation_threshold: 0.05  # Relaxed (~3 degrees)
```

### Strategy
1. **Joint-space goals** for point-to-point motions (RRTConnect — works perfectly with any DOF count)
2. **Cartesian goals** via pick_ik with relaxed orientation, then plan in joint space
3. **Teleoperation** via MoveIt Servo (Jacobian pseudoinverse handles 5-DOF naturally at velocity level)
4. **Never** use full 6-DOF Cartesian pose goals or `computeCartesianPath()` with 5-DOF

---

## 3. Real-Time Cartesian Control / Teleoperation

### MoveIt Servo (PRIMARY — use this)

Accepts streaming velocity commands, outputs joint commands at configurable rate (default 100Hz).

**Architecture:**
```
MediaPipe / VR Controller (pose @ 30-90Hz)
    → Retargeting node (compute Twist)
    → MoveIt Servo (Jacobian IK + collision check + singularity handling)
    → joint_trajectory_controller
    → Hardware Interface (CAN → Robstride)
```

**Key config:**
```yaml
publish_period: 0.01              # 100Hz
low_latency_mode: true            # Essential for teleop
check_collisions: true
smoothing_filter_plugin_name: "online_signal_smoothing::RuckigFilterPlugin"
```

**Why Servo works for 5-DOF:** At the velocity level, the Jacobian pseudoinverse finds the minimum-norm solution naturally. You send only the velocity components you care about — no full-pose IK needed.

### Dedicated Cartesian Controllers (for force/compliance tasks later)

| Controller | Package | Use Case | Needs F/T? |
|-----------|---------|----------|------------|
| admittance_controller | ros2_controllers (official) | Compliant motion | Yes |
| cartesian_motion_controller | FZI cartesian_controllers | Smooth Cartesian tracking | No |
| cartesian_compliance_controller | FZI cartesian_controllers | Force + motion hybrid | Yes |
| cartesian_vic_controller | ICube | Variable impedance | Yes |

**For KBot now:** MoveIt Servo is the right choice (no F/T sensor needed). Add FZI or admittance controller when you add force/torque sensors.

---

## 4. Whole-Body Control (When You Need It)

WBC treats the entire robot as one coordinated system — arm motions affect balance, so they must be planned together.

### When You Don't Need It Yet
If KBot is stationary (bolted down or standing with locked legs), MoveIt2 planning for arms alone is fine. WBC becomes critical when:
- The robot is standing and reaching (shifts CoM)
- Walking while manipulating
- Coordinating both arms + torso

### The Standard WBC Stack

```
Pinocchio (dynamics library, ~1μs per evaluation)
    + TSID (Task Space Inverse Dynamics, QP solver)
    = Priority-based whole-body controller at 200Hz-1kHz
```

**How it works:** Formulate a QP with prioritized tasks:
1. Balance (highest priority — keep CoM over support polygon)
2. Hand target (Cartesian pose)
3. Gaze direction
4. Posture (lowest priority — stay near default pose)

The QP solver finds joint torques/accelerations satisfying all constraints simultaneously.

### Open-Source WBC Frameworks

| Framework | Maturity | Used On | ROS2 |
|-----------|----------|---------|------|
| **Pinocchio + TSID** | High | TALOS, HRP-4 | Yes (ROS2 package) |
| **OCS2** (ETH) | High | ANYmal, bipedal research | ros2 branch |
| **Crocoddyl** | High | TALOS (walking + manipulation) | Via Pinocchio |
| **mc_rtc** (CNRS) | Very high | HRP-2/4/5P | ROS wrapper |
| **IHMC** | Extremely high | Atlas (DRC), Valkyrie, Nadia | Java, ROS wrapper |
| **Drake** (MIT/TRI) | Very high | Atlas origin, Punyo | Experimental ros2 |

**Recommendation:** Start with **Pinocchio + TSID** when you need WBC. It's what PAL uses on TALOS, has native ROS2 packages, and is the most approachable for a ros2_control-based system.

---

## 5. The Full Production Stack

### What You Have Now
- [x] ros2_control hardware interface (Robstride CAN)
- [x] joint_trajectory_controller
- [x] URDF with collision meshes
- [x] MuJoCo simulation
- [x] MediaPipe pose teleop (prototype)
- [x] MoveIt2 config (from commit history)

### What Production Humanoids Add

| Layer | What | Tool | Priority |
|-------|------|------|----------|
| **Safety** | Hardware E-stop, CAN watchdog, joint limits | Custom + ros2_control joint_limits | **P0** |
| **State Estimation** | Trunk IMU + EKF | `robot_localization` + BNO085/Orientus IMU | P1 |
| **Diagnostics** | Motor temp, CAN errors, tracking error | `diagnostic_updater` + Foxglove | P1 |
| **Motion Planning** | MoveIt2 with pick_ik for 5-DOF | MoveIt2 + pick_ik | P1 |
| **Teleop** | MoveIt Servo for Cartesian control | MoveIt Servo + RuckigFilter | P1 |
| **Perception** | Depth camera + object detection | RealSense + YOLOv8/AnyGrasp | P2 |
| **Balance** | Standing CoM controller | Pinocchio + simple QP | P2 |
| **Task Planning** | Behavior trees for sequencing | BehaviorTree.CPP | P3 |
| **Locomotion** | Walking pattern generator or RL | ZMP (open-rdc) or K-Scale ksim | P3 |
| **Manipulation** | Grasp planning | MoveIt Task Constructor + GPD/AnyGrasp | P4 |
| **RL Integration** | Sim-to-real policies | MuJoCo train → ONNX → ros2_control controller | P5 |
| **VLA** | End-to-end vision-language-action | GR00T N1 / Helix / custom | P6 |

---

## 6. How Production Humanoids Are Architected

### The Two Camps

**Camp A: Classical (PAL TALOS, HRP, research labs)**
```
MoveIt2 / OMPL → Whole-Body QP (TSID) → ros2_control → Hardware
```
- Deterministic, debuggable, well-understood
- Good for structured tasks
- The PAL TALOS reference architecture

**Camp B: Learned (Figure, Unitree, Tesla, Agility)**
```
VLM task decomposition → RL policy (trained in sim) → PD control → Hardware
```
- End-to-end, generalizes better
- Requires massive sim infrastructure
- Figure's Helix replaced ~110k lines of C++ with neural priors

**Camp C: Hybrid (emerging standard)**
```
VLM for task understanding → MPC/RL for motion → Classical safety layer → Hardware
```
- Best of both worlds
- MoveIt2 as safety fallback + constraint enforcement
- RL handles dynamic motions, classical handles precision tasks

### The Three-Tier Control Pattern (universal)

| Tier | Rate | What | SteveROS Equivalent |
|------|------|------|-------------------|
| High-level | 1-30 Hz | Perception, task planning, VLA | BehaviorTree.CPP + perception |
| Mid-level | 50-500 Hz | MPC/WBC, trajectory optimization | MoveIt2 + joint_trajectory_controller |
| Low-level | 500-2000 Hz | Joint PD/impedance control | steveros_hardware.cpp (MIT mode) |

---

## 7. Key Technology Choices Summary

| Decision | Choice | Why |
|----------|--------|-----|
| IK Solver | **pick_ik** | Only solver with native 5-DOF support + cost functions |
| Joint Planner | **RRTConnect + STOMP** | Fast initial path + smooth optimization |
| Cartesian Control | **MoveIt Servo** | Built-in collision/singularity handling, works at velocity level |
| Teleoperation | **MoveIt Servo** (now), **Pinocchio/Pink** (upgrade) | Servo for quick start, Pink for multi-chain whole-body |
| Whole-Body Control | **Pinocchio + TSID** | PAL TALOS uses it, native ROS2, C++ fast |
| Task Planning | **BehaviorTree.CPP** | Standard in ROS2, Nav2 backbone, Groot2 editor |
| Locomotion (future) | **RL in MuJoCo** via K-Scale ksim | You already have MuJoCo model |
| Simulation | **MuJoCo** (control) + **Gazebo/Isaac** (perception) | MuJoCo best for contact/RL, add others for cameras |
| State Estimation | **robot_localization** EKF | Trunk IMU + joint encoders |
| Diagnostics | **Foxglove** + diagnostic_updater | Industry standard for ROS2 monitoring |
| Safety | Hardware E-stop + CAN watchdog + joint_limits | Non-negotiable for production |

---

## Sources

### MoveIt2
- [MoveIt Planners Overview](https://moveit.ai/documentation/planners/)
- [pick_ik Tutorial](https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html)
- [MoveIt Servo Tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [STOMP Planner](https://moveit.ai/moveit%202/ros/2023/05/19/optimization-based-planning-with-stomp.html)
- [Parallel Planning](https://picknik.ai/moveit%202/parallel%20planning/motion%20planning/2023/02/15/parallel-planning-with-MoveIt-2.html)
- [Guide to Cartesian Planners](https://picknik.ai/cartesian%20planners/moveit/motion%20planning/2021/01/07/guide-to-cartesian-planners-in-moveit.html)

### Whole-Body Control
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [TSID](https://github.com/stack-of-tasks/tsid)
- [OCS2](https://github.com/leggedrobotics/ocs2)
- [Crocoddyl](https://github.com/loco-3d/crocoddyl)
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [IHMC Open Robotics](https://github.com/ihmcrobotics/ihmc-open-robotics-software)
- [Benchmarking WBC on TALOS](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.826491/full)

### IK & Cartesian Control
- [pick_ik GitHub](https://github.com/PickNikRobotics/pick_ik)
- [TRAC-IK ROS2](https://github.com/aprotyas/trac_ik)
- [FZI Cartesian Controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)
- [ICube Cartesian VIC](https://github.com/ICube-Robotics/cartesian_controllers_ros2)
- [Pink (Pinocchio IK)](https://github.com/stephane-caron/pink)

### Production Humanoids
- [PAL TALOS](https://pal-robotics.com/robot/talos/)
- [Figure AI Helix](https://www.humanoidsdaily.com/news/from-pixels-to-torque-figure-unveils-helix-02)
- [Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym)
- [K-Scale Labs](https://github.com/kscalelabs)
- [Humanoid-Gym](https://github.com/roboterax/humanoid-gym)
- [MuJoCo Playground](https://playground.mujoco.org/)

### Ecosystem
- [BehaviorTree.CPP ROS2](https://www.behaviortree.dev/docs/ros2_integration/)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- [Foxglove](https://foxglove.dev/)
- [ROS2 Walking Pattern Generator](https://github.com/open-rdc/ROS2_Walking_Pattern_Generator)
- [RoMoCo Locomotion Toolbox](https://arxiv.org/html/2509.19545v1)
