# MoveIt2 Post-Implementation Analysis for KBot 20-DOF Humanoid

*Generated: March 2026 | Multi-agent research synthesis*

---

## Table of Contents

1. [Your Current State — What You Have](#1-your-current-state)
2. [What MoveIt2 Unlocks — Core Capabilities](#2-what-moveit2-unlocks)
3. [MoveIt Servo — Real-Time Teleoperation](#3-moveit-servo)
4. [MoveIt Task Constructor — Complex Multi-Step Tasks](#4-moveit-task-constructor)
5. [Perception Pipeline — Environment-Aware Planning](#5-perception-pipeline)
6. [Grasping Pipelines — From Geometric to Deep Learning](#6-grasping-pipelines)
7. [RL + MoveIt2 — Hybrid Planning](#7-rl-and-moveit2)
8. [Teleoperation Frameworks — Data Collection](#8-teleoperation-frameworks)
9. [Sim-to-Real Pipelines — Training Infrastructure](#9-sim-to-real-pipelines)
10. [Complementary ROS2 Ecosystem Tools](#10-complementary-ros2-ecosystem)
11. [Open-Source Humanoid Projects — Reference Architectures](#11-open-source-humanoid-projects)
12. [K-Scale Ecosystem Status](#12-k-scale-ecosystem-status)
13. [Recommended Roadmap — Prioritized Next Steps](#13-recommended-roadmap)

---

## 1. Your Current State

Your SteveROS workspace is well-architected with three packages:

| Package | Purpose | Status |
|---------|---------|--------|
| `steveros_hardware` | ros2_control SystemInterface plugin, CAN-based Robstride motor control | Complete |
| `steveros_description` | URDF/XACRO (official KBot 20-DOF), 24 STL meshes, RViz config | Complete |
| `steveros_bringup` | Launch files, 4 per-limb JointTrajectoryControllers @ 100Hz | Complete |

**Joint Configuration (20 DOF):**
- Right Arm (5): shoulder pitch/roll, shoulder yaw, elbow, wrist (Motor IDs 21-25)
- Left Arm (5): shoulder pitch/roll, shoulder yaw, elbow, wrist (Motor IDs 11-15)
- Right Leg (5): hip pitch/roll/yaw, knee, ankle (Motor IDs 41-45)
- Left Leg (5): hip pitch/roll/yaw, knee, ankle (Motor IDs 31-35)

**Motor Types:** RS02 (17-60 Nm), RS03 (60 Nm), RS04 (120 Nm) — all quasi-direct-drive, back-drivable.

**What's Working:**
- Hardware interface with soft-enable Kp ramp for safe activation
- Per-limb JointTrajectoryControllers (position + velocity command)
- Mock hardware mode for testing without physical robot
- Right arm calibrated, left arm partially calibrated, legs not yet calibrated
- MIT mode CAN encoding with torque feedforward slot (ready for gravity compensation)

**What's Missing (and where MoveIt2 fits):**
- No collision-free motion planning
- No inverse kinematics
- No teleoperation
- No perception/obstacle avoidance
- No multi-step task planning
- No simulation (MuJoCo planned)

---

## 2. What MoveIt2 Unlocks

After running the MoveIt2 Setup Assistant and generating your config package, you gain:

### Motion Planning Algorithms

| Planner | Type | Best For |
|---------|------|----------|
| **RRTConnect** (default) | Sampling-based (OMPL) | Fast collision-free paths for any configuration |
| **PRM** | Sampling-based (OMPL) | Repeated queries in same environment |
| **KPIECE** | Sampling-based (OMPL) | High-DOF spaces |
| **CHOMP** | Gradient optimization | Smooth trajectories, pulls paths out of collision |
| **STOMP** | Stochastic optimization | Trajectory refinement, cost minimization |
| **Pilz PTP/LIN/CIRC** | Deterministic industrial | Predictable trapezoidal velocity profiles |

**Pipeline Chaining (new in MoveIt2):** OMPL generates a seed trajectory → STOMP/CHOMP refines it → higher-quality paths than any single planner.

### Inverse Kinematics

- **KDL** (default): Numerical solver, works for any chain
- **LMA**: Better for redundant chains (7+ DOF)
- **Cached IK**: Pre-computed solutions for repetitive tasks
- **Custom analytical solvers**: Can be plugged in for speed

### Planning Groups You Should Define (SRDF)

```
right_arm:    [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist]  (5 DOF)
left_arm:     [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist]  (5 DOF)
dual_arms:    [all 10 arm joints]  (10 DOF, bimanual planning)
right_leg:    [hip_pitch, hip_roll, hip_yaw, knee, ankle]  (5 DOF)
left_leg:     [hip_pitch, hip_roll, hip_yaw, knee, ankle]  (5 DOF)
upper_body:   [all 10 arm joints + torso if applicable]
```

### Key Capabilities After Setup

1. **Collision-free arm motion planning** — plan paths that avoid self-collision between arms, legs, and torso
2. **Dual-arm / bimanual planning** — plan both arms simultaneously through a combined planning group
3. **Constrained planning** — keep end-effector level (carrying a tray), constrain motion to a plane
4. **Interactive RViz planning** — drag end-effectors to goals, visualize and execute plans
5. **Programmatic control** — `MoveGroupInterface` in C++ or Python for autonomous applications
6. **Self-collision matrix** — auto-generated from URDF, optimized for links that can never collide

---

## 3. MoveIt Servo — Real-Time Teleoperation

MoveIt Servo is the real-time servoing component — low-latency Cartesian and joint control via inverse Jacobian.

### What It Does

- Accepts **three input types**: individual joint velocities, end-effector twist (TwistStamped), end-effector pose targets
- Uses SVD pseudo-inverse of the Jacobian — works with **any DOF count** including your 5-DOF arms
- No trajectory planning involved → maximum speed
- Safety: singularity checking, real-time collision checking, joint limit enforcement

### Input Devices

Since input is standard `geometry_msgs/TwistStamped`, virtually anything works:
- **Gamepad** (Xbox, PS5)
- **SpaceNav 6-DOF mouse**
- **VR controllers** (Meta Quest 3 via Unity/OpenXR)
- **Autonomous vision systems** (closed-loop visual servoing)
- Multiple devices can be combined simultaneously

### Signal Smoothing

- **ButterworthFilterPlugin**: Low-pass filter, no overshoot
- **RuckigFilterPlugin**: Enforces acceleration/velocity limits for smoothest output

### Visual Servoing

Pattern for precision tasks:
1. MoveIt2 planning brings robot to rough "approach" configuration
2. Switch to MoveIt Servo for final precise alignment using camera feedback
3. Four PID controllers track target: x/y/z translation + angle-axis rotation

### Data Collection for Imitation Learning

MoveIt Servo is particularly valuable for collecting demonstration data because it generates smooth, stable trajectories. These clean observation-action pairs work well for behavior cloning, diffusion policies, and VLA systems.

### Real-Time Kernel

For best performance, Servo automatically configures SCHED_FIFO with priority 40 on RT kernels. Options: Real-time Ubuntu 22.04 LTS or `linux-image-rt-amd64` on Debian.

---

## 4. MoveIt Task Constructor — Complex Multi-Step Tasks

MTC decomposes complex manipulation into a hierarchy of interdependent planning subtasks.

### Architecture

**Stage Types (by result flow):**
- **Generators**: Compute results independently (e.g., `CurrentState`, `GenerateGraspPose`)
- **Propagators**: Receive from one side, pass to other (e.g., `MoveRelative` for Cartesian approach)
- **Connectors**: Bridge gap between two states (e.g., free-motion planning between configurations)

**Container Types (by hierarchy):**
- **Serial**: Sequence of stages (approach → grasp → lift)
- **Parallel**: Alternative strategies — pick with left OR right hand, fallback planners
- **Wrappers**: Modify/filter results (IK Wrapper generates IK solutions from pose targets)

### Canonical Pick-and-Place Pipeline

```
1. CurrentState (Generator)
2. Open Hand (MoveTo)
3. Move to Pick (Connect — free-motion planning)
4. Approach Object (MoveRelative — Cartesian)
5. Generate Grasp Pose (Generator)
6. Close Hand (MoveTo)
7. Attach Object (ModifyPlanningScene)
8. Lift Object (MoveRelative — Cartesian)
9. Move to Place (Connect — free-motion planning)
10. Lower Object (MoveRelative)
11. Open Hand (MoveTo)
12. Detach Object (ModifyPlanningScene)
13. Retreat (MoveRelative)
```

### KBot-Specific Applications

- **Bimanual handoffs**: Pick with one arm → transfer to other arm (Parallel Container)
- **Sequential tool use**: Grasp tool → use tool → replace tool
- **Assembly tasks**: Multi-stage with precise sequencing
- **Fallback strategies**: Try right arm, if unreachable, try left arm

---

## 5. Perception Pipeline — Environment-Aware Planning

### How It Works

1. Depth cameras publish point clouds (`sensor_msgs/PointCloud2`) or depth images
2. MoveIt listens via TF transforms, converts to world frame
3. Self-filtering removes points corresponding to robot's own body
4. Remaining points → Octomap (3D voxel grid)
5. Octomap becomes collision object → all planners automatically avoid obstacles

### Configuration (`sensors_3d.yaml`)

Key parameters:
- `octomap_frame`: Fixed world frame
- `octomap_resolution`: Meters (detail vs performance tradeoff)
- `max_range`: Maximum sensor range
- `point_subsample`: Downsample factor for performance

### Practical Setup for KBot

Mount a depth camera (Intel RealSense D435i or OAK-D) on head/chest:
1. Publish point clouds to a ROS2 topic
2. Configure `sensors_3d.yaml` with correct topic and TF frame
3. MoveIt2 automatically incorporates environment into all planning queries

### Advanced: Object Pose Estimation Pipeline

```
RGB-D Camera
    ↓
YOLO (2D object detection)
    ↓
FoundationPose or MegaPose (6D pose estimation from depth)
    ↓
PoseStamped → MoveIt2 Planning Scene
    ↓
MTC plans grasp approach → pick → place
```

**FoundationPose** (NVIDIA): Pre-trained for 6D pose estimation on unseen objects without fine-tuning.

---

## 6. Grasping Pipelines

### Geometric (MoveIt Grasps)

- Generates grasp candidates for simple shapes (cuboids, cylinders)
- Filters by IK reachability
- Cartesian planning for approach/lift/retreat
- Supports parallel-jaw and suction grippers
- Does NOT consider friction or dynamics

### Deep Learning — GPD (Grasp Pose Detection)

- Detects 6-DOF grasp poses from 3D point clouds
- Pipeline: sample candidates → CNN classification → cluster viable grasps
- Integrates as MoveIt "GraspPlanning" plugin

### Deep Learning — Dex-Net

- Samples grasps from depth images (not point clouds)
- Higher success rate than GPD
- Trained on millions of images → novel object grasping in clutter

### Integration with MTC

Both GPD and Dex-Net integrate via the `DeepGraspPose` generator stage:
- Action client in MTC connects to grasp detection action server
- Grasp candidates with costs feed directly into pick pipeline

---

## 7. RL and MoveIt2

### The Hybrid Pattern (State of the Art)

The dominant approach for humanoids: **classical control for arms (MoveIt2), RL for legs**.

This is exactly what PAL Robotics TALOS and the G1Pilot project (Unitree G1) do — nobody runs pure MoveIt2 planning on legs.

### How RL Enhances MoveIt2 Manipulation

- DRL-based planners (SAC, PPO) achieve higher success rates and lower planning times than classical planners
- Once trained, produce time-deterministic behavior with implicit collision avoidance
- 2025 benchmark (100K+ queries): DRL > RRTConnect in success rate and planning time

### Production RL Deployment Pattern

```
1. Train in simulation (MuJoCo Playground / Isaac Lab / Humanoid-Gym)
2. Export policy to ONNX format
3. Deploy via ROS2 C++ node with ONNX Runtime
4. Policy reads /joint_states + IMU → publishes to ros2_control
5. Inference: ~50-200Hz (~1ms GPU / ~7.5ms CPU)
```

### Imitation Learning Pipeline for KBot

```
1. Teleoperate via MoveIt Servo → smooth trajectories
2. Record observation-action pairs (joint states, EE poses, camera)
3. Train behavior cloning / diffusion policies / VLA models
4. Deploy via MoveIt Servo for execution + safety checking
```

### NVIDIA cuRobo/cuMotion — GPU-Accelerated Planning

- MoveIt2 plugin via Isaac ROS cuMotion
- ~45ms planning time vs ~1200ms for stock MoveIt2
- 90% success rate in obstacle environments vs MoveIt2's 62%
- Runs on Jetson Orin NX at ~100ms latency
- **DiffusionSeeder**: 12x average speedup for complex problems

---

## 8. Teleoperation Frameworks

### GMR — General Motion Retargeting (ICRA 2026)

- Leading open-source framework, 17+ humanoid robots supported
- Real-time whole-body retargeting from human to humanoid
- Supports NVIDIA XRoboToolkit SDK (TWIST2)
- You can request KBot support by providing URDF + meshes

### VR-Based Teleoperation

- **Unity + ROS2 + OpenXR** (ETH Zurich): Open-source, Meta Quest 3 support, hand tracking
- **High-speed (1kHz)**: 7 IMUs + VR headset + trackers, demonstrated punching at human speed (Nadia humanoid)

### GELLO — Leader Arms for Demonstration

- 3D-printed kinematically-scaled leader arms with Dynamixel servos (~$300/arm)
- Outperforms VR and SpaceMouse in user studies
- Data feeds directly into LeRobot for imitation learning

### LeRobot + lerobot-ros (August 2025)

- HuggingFace's end-to-end learning framework
- **lerobot-ros**: Lightweight ROS2 interface for ros2_control/MoveIt2 → LeRobot
- Collect teleop demonstrations → train Diffusion Policy or ACT → deploy back through ROS2
- Most accessible path to learning-based manipulation

---

## 9. Sim-to-Real Pipelines

### MuJoCo Playground (RSS 2025 Outstanding Demo Paper Award)

- Google DeepMind, `pip install playground`
- Train locomotion policies in minutes on a single GPU
- Verified sim-to-real on Unitree G1 and Booster T1 humanoids
- Deployment: ONNX Runtime, 50Hz, C++ ROS2 node via ros2_control
- Supports both MuJoCo MJX (JAX) and MuJoCo Warp backends

### NVIDIA Isaac Lab (v2.3)

- Built on Isaac Sim, 30+ environments, 16+ robot models (G1, H1, Cassie, Digit)
- Automatic Domain Randomization (ADR) + Population Based Training (PBT)
- Foundation for NVIDIA GR00T humanoid foundation model
- Best for: rough terrain locomotion, perceptive locomotion, whole-body loco-manipulation

### Humanoid-Gym (RobotEra)

- RL framework on NVIDIA Isaac Gym, specifically for humanoid locomotion
- Zero-shot sim-to-real verified on XBot-S (1.2m) and XBot-L (1.65m)
- Sim-to-sim verification: Isaac Gym → MuJoCo → real hardware
- 1,854 GitHub stars, significant community adoption

### Key Sim-to-Real Techniques

- **Domain randomization**: mass, friction, motor dynamics, sensor noise
- **Sim-to-sim verification**: train in one simulator, validate in another before real deployment
- **Servo tuning**: K-Scale's `ktune` methodology for matching simulated vs real motor response

---

## 10. Complementary ROS2 Ecosystem

### BehaviorTree.ROS2

- Standard for task orchestration — composing navigation + manipulation + perception
- Used by Nav2, MoveIt Pro, and most production ROS2 systems
- ROS2 wrappers for BehaviorTree.CPP: action clients, service clients, topic subscribers

### Visualization & Monitoring

| Tool | Use Case |
|------|----------|
| **RViz2** | Local 3D visualization, MoveIt interactive planning |
| **Foxglove Studio** | Remote WebSocket monitoring, no ROS2 install needed |
| **Rerun** | Programmatic data embedding via SDKs |
| **MCAP** | High-performance rosbag format |

### Whole-Body Control — TSID

- [stack-of-tasks/tsid](https://github.com/stack-of-tasks/tsid) with Pinocchio dynamics
- Task hierarchy: self-collision avoidance → CoM stabilizer → foot contacts → arm tasks
- PAL TALOS uses this exact architecture
- Bridge between independent limb control and coordinated humanoid behavior

---

## 11. Open-Source Humanoid Projects

### PAL Robotics TALOS

- 32 DOF, 1.75m, 100kg, Apache 2.0 URDF
- Full ROS-based: TSID whole-body control + gravity compensation + MoveIt manipulation
- GMR motion retargeting support (October 2025)
- Best reference architecture for ROS2 humanoid manipulation

### Unitree G1 / H1

- G1 rated "Best Humanoid SDK" in 2025
- **G1Pilot**: ROS2 package — lower body left to Unitree's controller, upper body via MoveIt2 + teleoperation
- **unitree_il_lerobot**: Official imitation learning on HuggingFace LeRobot
- Supported in MuJoCo Playground, Isaac Lab, and Humanoid-Gym

### Rapid Robotics 3PRO

- Industrial bimanual humanoid powered by MoveIt Pro
- Pick, pack, palletize, and kit — the most prominent real-world MoveIt2 humanoid case study

### bit-bots/humanoid_robots_ros2

- Collection of ROS2 packages for different humanoids with URDF and MoveIt configurations
- Useful for benchmarking algorithms across platforms

---

## 12. K-Scale Ecosystem Status

**Important: K-Scale Labs shut down November 2025.** All IP was open-sourced under MIT/CERN-OHL-S-2.0.

### Valuable Artifacts (still accessible on GitHub)

| Artifact | Value for SteveROS |
|----------|-------------------|
| Validated Robstride motor gains | Already integrated in your XACRO |
| MuJoCo models | Use for simulation setup |
| `urdf2mjcf` converter | URDF → MJCF conversion |
| `ktune` servo tuning | Match sim to real motor dynamics |
| `ksim` RL training library | MuJoCo + JAX, PPO converges in ~30 min on RTX 4090 |
| Zeroing procedures | Already referenced in your calibration docs |

### What K-Scale Did NOT Use

- K-Scale ran Rust-based K-OS with gRPC, NOT ROS2
- Your ROS2 workspace is a community-driven integration, not a K-Scale product
- No upstream maintenance — fork what you need

---

## 13. Recommended Roadmap — Prioritized Next Steps

### Phase A: MoveIt2 Foundation (Immediate)

| Step | Action | Unlocks |
|------|--------|---------|
| A1 | Run MoveIt2 Setup Assistant → generate SRDF + config package | All MoveIt2 capabilities |
| A2 | Define planning groups: `right_arm`, `left_arm`, `dual_arms` | Per-arm and bimanual planning |
| A3 | Test in RViz with interactive motion planning plugin | Visual validation |
| A4 | Write first MoveGroupInterface program (C++ or Python) | Programmatic control |

### Phase B: Real-Time Control + Teleoperation

| Step | Action | Unlocks |
|------|--------|---------|
| B1 | Enable MoveIt Servo for gamepad teleoperation | Real-time arm control |
| B2 | Add SpaceNav 6-DOF mouse support | Precise teleoperation |
| B3 | Collect demonstration data via teleoperation | Foundation for imitation learning |
| B4 | Set up lerobot-ros for standardized data recording | LeRobot ecosystem access |

### Phase C: Perception-Aware Manipulation

| Step | Action | Unlocks |
|------|--------|---------|
| C1 | Mount depth camera (RealSense D435i), publish point clouds | Raw 3D data |
| C2 | Configure MoveIt2 perception pipeline (Octomap) | Obstacle-aware planning |
| C3 | Add YOLO object detection | Object identification |
| C4 | Add FoundationPose for 6D pose estimation | Precise object localization |
| C5 | Integrate MoveIt Task Constructor for pick-and-place | Multi-step manipulation |

### Phase D: Simulation + RL Training

| Step | Action | Unlocks |
|------|--------|---------|
| D1 | Set up MuJoCo via mujoco_ros2_control | Physics simulation |
| D2 | Use K-Scale's urdf2mjcf for model conversion | MuJoCo-compatible model |
| D3 | Tune actuators with ktune to match real motors | Accurate sim dynamics |
| D4 | Train locomotion with MuJoCo Playground or Humanoid-Gym | Walking policies |
| D5 | Deploy ONNX policies via C++ ROS2 node @ 50Hz | Real-time RL execution |

### Phase E: Advanced Integration

| Step | Action | Unlocks |
|------|--------|---------|
| E1 | NVIDIA cuRobo/cuMotion as MoveIt2 plugin | 10-25x faster planning |
| E2 | GMR motion retargeting for whole-body teleoperation | Human-to-robot mapping |
| E3 | GELLO leader arms for manipulation demonstrations | High-quality demo data |
| E4 | BehaviorTree.ROS2 for task orchestration | Composable complex behaviors |
| E5 | TSID whole-body control (stack-of-tasks/tsid) | Coordinated loco-manipulation |

### What NOT to Pursue Yet

- **Nav2 for bipedal navigation** — only relevant once stable locomotion exists
- **MoveIt Pro (commercial)** — open-source MoveIt2 + MTC covers most needs
- **Full Gazebo Harmonic** — MuJoCo is better for humanoid contact physics
- **Pure MoveIt2 for leg planning** — RL is the established approach for bipedal locomotion

---

## Key Sources

- [MoveIt2 Documentation](https://moveit.picknik.ai/main/)
- [MoveIt Servo Tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor)
- [MuJoCo Playground](https://playground.mujoco.org/)
- [Isaac Lab](https://github.com/isaac-sim/IsaacLab)
- [Humanoid-Gym](https://github.com/roboterax/humanoid-gym)
- [GMR Motion Retargeting](https://github.com/YanjieZe/GMR)
- [lerobot-ros](https://discourse.openrobotics.org/t/lerobot-ros/)
- [LeRobot](https://github.com/huggingface/lerobot)
- [GELLO Leader Arms](https://github.com/wuphilipp/gello_software)
- [stack-of-tasks/tsid](https://github.com/stack-of-tasks/tsid)
- [cuRobo](https://github.com/NVlabs/curobo)
- [PAL TALOS](https://github.com/pal-robotics/talos_robot)
- [G1Pilot (Unitree)](https://github.com/hucebot/g1pilot)
- [K-Scale KBot](https://github.com/kscalelabs/kbot)
- [K-Scale ksim](https://github.com/kscalelabs/ksim)
- [Rapid Robotics Case Study](https://picknik.ai/case-study-rapid-robotics/)
