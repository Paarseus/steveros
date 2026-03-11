# VR Teleoperation for Humanoid Robots: Deep Research Report

**Date:** 2026-03-10
**Prepared for:** SteveBots / Steveros Project
**Analyst:** Claude Opus 4.6 (Multi-Agent Research Team)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Complete VR Teleop Pipeline](#2-complete-vr-teleop-pipeline---every-step)
3. [Project Catalog](#3-project-catalog)
4. [Projects from humanoid_projects_report.md](#4-projects-from-the-original-report)
5. [Comparison Tables](#5-comparison-tables)
6. [Recommendations for SteveBots](#6-recommendations-for-stevebots)

---

## 1. Executive Summary

This report catalogs and analyzes **33 open-source VR teleoperation projects** for humanoid and robot arm control. Research covered all projects from the original `humanoid_projects_report.md` plus the broader ecosystem.

### Key Findings

- **Meta Quest 3 is the dominant headset.** Supported by 10+ projects. Quest 2 also widely supported. Apple Vision Pro gaining traction but costs 7x more.
- **WebRTC is the standard for video streaming.** Used by K-Scale, Unitree, Reachy 2, NVIDIA (CloudXR), OpenArm, and PAL Robotics.
- **Pinocchio is the standard IK library.** Used by Unitree, Open-TeleVision, and NVIDIA Isaac Lab for closed-loop inverse kinematics.
- **GMR is the de facto standard retargeting library.** Supports 17 humanoid robots, multiple input formats (SMPLX, BVH, FBX, PICO).
- **iCub3 is the most comprehensive system** — full-body haptics, omnidirectional treadmill, facial expression mirroring, 1 kHz control, <25ms latency.
- **Data collection is a primary motivator.** Most teams use VR teleop to collect demonstration data for imitation learning, not just real-time remote control.
- **Haptic feedback remains rare.** Only OpenArm (bilateral force) and iCub3 (full-body suit) provide meaningful force feedback.

### Tier Distribution

| Tier | Description | Projects |
|------|-------------|----------|
| **Whole-Body VR Teleop** | Full locomotion + manipulation | SONIC/GR00T, TWIST/TWIST2, OmniH2O, CLONE, HumanPlus, GMR |
| **Upper-Body VR Teleop** | Arms + hands, no walking | Unitree XR, Open-TeleVision, FFTAI, K-Scale K-Bot, Reachy 2, OpenArm |
| **Arm/Hand Teleop** | Robot arm focus | Open-Teach, HATO, Holo-Dex, BunnyVisionPro, VisionProTeleop, Spes, TeleopXR |
| **Retargeting Libraries** | Building blocks | GMR, dex-retargeting, AnyTeleop, Pyroki |
| **Physical Teleop** | Leader-follower, no VR | Mobile ALOHA, GELLO, UMI, DexCap, Red Rabbit RX1 |

---

## 2. Complete VR Teleop Pipeline — Every Step

```
┌─────────────────────────────────────────────────────────────┐
│                    OPERATOR SIDE                             │
│                                                             │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │  VR Headset  │───▶│  VR SDK/API  │───▶│  Retargeting  │  │
│  │  (Tracking)  │    │  (OpenXR)    │    │  (IK/Neural)  │  │
│  └─────────────┘    └──────────────┘    └───────┬───────┘  │
│         ▲                                        │          │
│         │ Stereo Video                          │ Joint     │
│         │ (WebRTC)                              │ Commands  │
│         │                                        ▼          │
│  ┌──────┴──────┐                        ┌───────────────┐  │
│  │   Video     │◀───── Network ────────▶│  Transport    │  │
│  │  Decoder    │    (WiFi/Internet)      │  (ZMQ/WS/    │  │
│  └─────────────┘                        │   gRPC/UDP)   │  │
│                                          └───────┬───────┘  │
└──────────────────────────────────────────────────┼──────────┘
                                                    │
┌──────────────────────────────────────────────────┼──────────┐
│                    ROBOT SIDE                     │          │
│                                                   ▼          │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │  Stereo     │    │  Safety &    │◀───│  Joint        │  │
│  │  Cameras    │    │  Collision   │    │  Controller   │  │
│  │  (ZED/OAK)  │    │  Avoidance   │    │  (PD/Torque)  │  │
│  └──────┬──────┘    └──────┬───────┘    └───────────────┘  │
│         │ H.264            │                     ▲          │
│         ▼                  ▼                     │          │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │  GStreamer   │    │  Motor       │───▶│  Actuators    │  │
│  │  Encoder     │    │  Drivers     │    │  (CAN/SDK)    │  │
│  └─────────────┘    └──────────────┘    └───────────────┘  │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### Stage 1: VR Tracking Input

**What happens:** The VR headset tracks the operator's head, hands, and (optionally) body pose at high frequency.

| VR Device | Tracking Method | Hand Tracking | Frequency | Latency | Joints Tracked |
|-----------|----------------|---------------|-----------|---------|----------------|
| Meta Quest 2 | Inside-out (4 cameras) | Controller + hand | 72 Hz | ~20ms | 26 per hand |
| Meta Quest 3 | Inside-out (passthrough) | Controller + hand | 120 Hz | ~12ms | 26 per hand |
| Meta Quest Pro | Inside-out + eye/face | Controller + hand + face | 90 Hz | ~15ms | 26 per hand + face |
| Apple Vision Pro | Inside-out + LiDAR | Hand only (no controllers) | 90 Hz | ~12ms | 26 per hand |
| PICO 4 Ultra | Inside-out | Controller + hand + body | 90 Hz | ~15ms | Full body via PICO SDK |
| HTC VIVE Pro | Lighthouse (external) | SenseGlove/controller | 90 Hz | ~10ms | 6-DOF per tracker |

**SDK/API Options:**
- **OpenXR** (standard): Cross-platform, supported by Quest 3, PICO, VIVE. Provides `XrHandJointLocationEXT` for 26 keypoints per hand.
- **Oculus SDK**: Quest-specific, deeper access to hand/eye/face tracking.
- **WebXR**: Browser-based, no native app needed. Used by K-Scale, Spes Robotics, TeleopXR. Lower barrier to entry.
- **SteamVR/OpenVR**: VIVE ecosystem. Used by iCub3 with lighthouse tracking.
- **visionOS ARKit**: Apple Vision Pro. Used by Open-TeleVision, BunnyVisionPro, CLONE.

**Coordinate Frames:**
- OpenXR: Right-handed, Y-up, meters. Origin at headset initial position.
- Quest local space: Head-relative or stage-relative transforms.
- Conversion to robot frame requires a calibration step (usually a T-pose or button press).

### Stage 2: Motion Retargeting

**What happens:** Human motion data is converted to robot joint commands, accounting for different body proportions and joint configurations.

| Method | Projects Using It | Pros | Cons |
|--------|-------------------|------|------|
| **Direct Mapping** | Unitree XR, FFTAI, Reachy 2, OpenVR | Simplest, lowest latency | No embodiment adaptation |
| **Closed-Form IK (CLIK)** | Unitree (Pinocchio), K-Scale (JAX) | Real-time, well-understood | Singularities, no dynamics |
| **Differentiable IK (Pyroki)** | TeleopXR, OpenArm | Gradient-based, collision-aware | Newer, less battle-tested |
| **Optimization-Based (GMR)** | TWIST, TWIST2, SONIC | Real-time on CPU, 17 robots | No dynamics awareness |
| **Gradient Optimization (SMPL)** | OmniH2O | Leverages human body model | Complex setup |
| **Physics-Based (PHC)** | CLONE | Closed-loop correction | Computationally heavy |
| **RL Motion Tracking** | SONIC, TWIST, HumanPlus | Handles dynamics, contacts | Requires sim training per robot |
| **Neural (Transformer)** | HumanPlus (HST) | End-to-end learned | Needs large training data |
| **DexPilot Optimizer** | BunnyVisionPro, dex-retargeting | Best for dexterous hands | Hand-specific only |
| **Whole-Body QP** | iCub3 | Full kinematic chain + balance | Most complex |

**Dexterous Hand Retargeting (dex-retargeting library):**
1. `PositionOptimizer`: Minimizes fingertip position error between human and robot
2. `DexPilotOptimizer`: Minimizes pairwise finger distance vectors (captures grasp shape)
3. `SeqRetargeting`: Adds temporal smoothness constraints across frames
- Uses Pinocchio for forward kinematics, MediaPipe for hand detection
- Supports any URDF via the `dex-urdf` companion library

**Workspace Mismatch Solutions:**
- **Scaling**: Multiply human positions by robot/human limb length ratio (iCub3, GMR)
- **Relative/Delta control**: Track deltas from a reference pose, not absolute positions (Quest2ROS2, Reachy 2)
- **Workspace mapping**: Define operator workspace region → robot workspace region mapping
- **Nullspace projection**: Unitree uses joint angle offsets near manipulability limits

### Stage 3: Communication & Streaming

**What happens:** Joint commands flow from operator to robot; stereo video flows from robot to operator.

#### Control Command Transport

| Protocol | Latency | Used By | Notes |
|----------|---------|---------|-------|
| **ZeroMQ (ZMQ)** | <1ms local | SONIC/GR00T, HATO | Fastest. No serialization overhead. Pub/sub + req/rep patterns. |
| **WebSocket** | 5-50ms | Unitree XR, FFTAI, K-Scale | Browser-compatible. Works over internet. |
| **gRPC/Protobuf** | 2-10ms | Reachy 2 | Typed, language-agnostic. Good for structured commands. |
| **ROS 2 Topics** | 5-20ms | Spes, TeleopXR, HumanPlus | Native ROS integration. DDS-based. |
| **CycloneDDS** | 2-5ms | Unitree XR (robot control) | Direct DDS. Lower overhead than full ROS 2. |
| **UDP raw** | <1ms | K-Scale (firmware), dex-teleop | Fastest but unreliable. Good for high-frequency control. |
| **Redis** | 1-3ms | TWIST, TWIST2 (IPC) | In-memory. Good for inter-process on same machine. |
| **YARP** | <5ms local | iCub3 | Custom robotics middleware. Supports remote via VPN. |

#### Video Streaming Pipeline

```
Robot Stereo Camera (ZED Mini / OAK-D / RealSense)
    ↓ USB3 / MIPI
Raw stereo frames (1280x720 @ 30fps per eye, ~1.3 Gbps)
    ↓
GStreamer pipeline (or FFmpeg)
    ↓ H.264/H.265 hardware encode (NVIDIA NVENC or CPU x264)
Compressed stream (~10-30 Mbps)
    ↓
WebRTC (SRTP + DTLS)
    ↓ WiFi 6 (802.11ax) or Ethernet
VR headset browser/app
    ↓ Hardware decode
Stereo render in headset display
```

**Measured Glass-to-Glass Latencies:**
| System | Motion Latency | Video Latency | Notes |
|--------|---------------|---------------|-------|
| iCub3 | — | **<25ms** (local fiber) | Gold standard |
| XRoboToolkit | — | **<100ms** | <1% packet loss |
| Reachy 2 | 74ms | **135ms** (Quest 2) | Only project documenting both |
| SONIC/GR00T | — | — | 122ms wrist tracking (total) |
| BEAVR (paper) | — | **<35ms** | Zero-copy architecture |

**Stereo Camera Options:**
| Camera | Resolution | FPS | Interface | Used By |
|--------|-----------|-----|-----------|---------|
| ZED Mini | 2560x720 | 60 | USB 3.0 | Open-TeleVision, OmniH2O |
| ZED 2i | 4416x1242 | 15-100 | USB 3.0 | Various |
| Intel RealSense D435 | 1280x720 | 90 | USB 3.0 | FFTAI, HumanPlus |
| OAK-D | 1280x800 | 60 | USB 3.0 | FFTAI |
| Orbbec | 1280x800 | 30 | USB 3.0 | Red Rabbit RX1 |

### Stage 4: Robot Control

**What happens:** Retargeted joint commands are executed on the robot hardware with safety constraints.

**IK Solvers Comparison:**

| Solver | Language | Speed | Features | Used By |
|--------|----------|-------|----------|---------|
| **Pinocchio CLIK** | C++/Python | <1ms | Analytical Jacobian, derivatives | Unitree, GMR, Spes |
| **KDL** | C++ | ~2ms | ROS default, chain-based | Legacy ROS projects |
| **TRAC-IK** | C++ | ~1ms | KDL + SQP hybrid, better convergence | RX1 (TracIK) |
| **MuJoCo IK** | C/Python | <1ms | Built-in, physics-aware | Simulation projects |
| **JAXopt** | Python/JAX | <1ms (GPU) | Differentiable, GPU-accelerated | K-Scale |
| **Pyroki** | Python | <1ms | Differentiable, collision-aware | TeleopXR, OpenArm |
| **mink** | Python | ~1ms | Pinocchio wrapper, task-based | Research projects |

**Safety Mechanisms During Teleop:**
| Safety Feature | Projects Implementing It |
|---------------|------------------------|
| Dead man's switch (must hold trigger) | PAL TALOS, Reachy 2 (A button) |
| Emergency stop button (VR controller) | PAL TALOS (Meta button), Walk These Ways |
| Joint velocity limiting | Unitree XR, GMR (configurable) |
| Self-collision avoidance | PAL TALOS (repulsor-based), TeleopXR (Ballpark geometry) |
| Workspace bounds | dex-teleop, BunnyVisionPro |
| Singularity detection | BunnyVisionPro, Unitree XR (nullspace projection) |
| Smooth torque ramp-down on disconnect | Reachy 2 (`turn_off_smoothly()`, 3-second ramp) |
| Orientation check (roll/pitch limit) | Walk These Ways (>1.6 rad triggers safe pose) |

**Control Frequency Requirements:**
| Level | Frequency | Use Case |
|-------|-----------|----------|
| High-level planning | 10-30 Hz | VR input sampling, trajectory planning |
| Mid-level control | 50-100 Hz | Joint position commands, IK solving |
| Low-level PD control | 200-1000 Hz | Motor-level torque/position control |

### Stage 5: Feedback to Operator

**What happens:** The operator receives visual, haptic, and audio feedback from the robot.

#### Visual Feedback
- **Stereo video** (most common): ZED camera on robot head → WebRTC → VR headset. Creates depth perception.
- **Mixed reality**: Quest 3 passthrough + robot camera overlay (PAL, Unitree)
- **3D visualization**: URDF rendering in browser alongside camera feed (K-Scale, TeleopXR)
- **NeRF viewer**: Unity ROS Teleop (ETH) supports real-time NeRF rendering

#### Haptic Feedback (Rare)
| System | Hardware | Feedback Type | Intensity |
|--------|----------|--------------|-----------|
| iCub3 | SenseGlove DK1 | Mechanical finger brakes | **Up to 20N per finger** |
| iCub3 | iFeel suit | Vibrotactile body feedback | Contact + weight |
| iCub3 | iFeel shoes | Foot contact vibration | Binary contact |
| OpenArm | Bilateral force | Felt through leader arm | Proportional |
| BunnyVisionPro | FSR → ERM | Fingertip vibration | Proportional |
| Quest2ROS | Controller haptics | Vibration freq + amplitude | Configurable |
| Unity ROS Teleop | Bhaptics gloves | Finger vibration | Configurable |

#### Audio Feedback
- iCub3: Binaural audio relay (robot microphones → operator headphones)
- Most other systems: None

### Stage 6: Data Collection for Imitation Learning

**What happens:** VR teleop demonstrations are recorded and used to train autonomous policies.

```
VR Teleop Session
    ↓ Record at each timestep:
    ├─ Robot joint positions/velocities
    ├─ End-effector poses
    ├─ Camera images (RGB-D)
    ├─ VR controller/hand poses
    ├─ Gripper state
    └─ Timestamps
    ↓
HDF5 / LeRobot v2 Format
    ↓
Training Pipeline
    ├─ ACT (Action Chunking with Transformers) — ALOHA, Open-TeleVision
    ├─ Diffusion Policy — UMI, iDP3
    ├─ VLA (Vision-Language-Action) — FFTAI, Open X-Humanoid
    └─ PPO + Domain Randomization — SONIC (780K synthetic from VR demos)
    ↓
Autonomous Policy Deployment
```

**Notable data collection rates:**
- TWIST2: 100 demos in 15 minutes using PICO VR
- NVIDIA SONIC: 780,000 synthetic trajectories (≈6,500 hours) generated in 11 hours from initial VR demos
- Phospho/LeRobot: Auto-saves in LeRobot v2 format, uploads to HuggingFace Hub

---

## 3. Project Catalog

### 3.1 Tier 1: Whole-Body VR Teleoperation (Locomotion + Manipulation)

#### NVIDIA GR00T Whole-Body Control / SONIC
- **GitHub:** NVlabs/GR00T-WholeBodyControl (~1,100 stars)
- **License:** Apache 2.0 (code), NVIDIA Open Model License (weights)
- **VR Hardware:** PICO VR headset + 2 ankle trackers + controllers
- **Supported Robots:** Unitree G1 (primary); generalizable architecture
- **Pipeline:** PICO VR → GMR retargeting → SONIC foundation model (42M params, trained on 100M+ mocap frames) → ZMQ → C++ inference (Jetson Orin, TensorRT + CUDA Graph)
- **Retargeting:** RL-based motion tracking; distilled from large-scale human motion data
- **Locomotion:** Yes — walking, running, jumping, kneeling, crawling, getting up
- **Hand Tracking:** Yes, bimanual manipulation
- **Control Frequencies:** 100 Hz input capture, 10 Hz kinematic planner (<5ms), 50 Hz policy, 500 Hz PD control
- **Latency:** 122ms mean wrist tracking, 1-2ms policy inference, 12ms motion generation
- **Key Strength:** Most comprehensive open-source whole-body controller; 100% zero-shot success on 50 real-world trajectories

#### TWIST (Teleoperated Whole-Body Imitation System)
- **GitHub:** YanjieZe/TWIST (~707 stars)
- **License:** MIT
- **VR Hardware:** OptiTrack MoCap; PICO VR via GMR retargeter
- **Supported Robots:** Unitree G1
- **Pipeline:** MoCap/VR → GMR retargets to robot joints + root velocities → Teacher RL policy (privileged, IsaacGym) → Student distillation via DAgger → Redis IPC → Ethernet to robot
- **Retargeting:** GMR (optimization-based, real-time on CPU)
- **Locomotion:** Yes — whole-body loco-manipulation
- **Latency:** ~39-50 Hz policy execution on hardware
- **Key Strength:** Unified single neural network for all whole-body skills; fully open-sourced training pipeline

#### TWIST2
- **GitHub:** amazon-far/TWIST2 (~711 stars)
- **License:** MIT (Amazon + Stanford)
- **VR Hardware:** PICO 4U (mocap-free)
- **Supported Robots:** Unitree G1
- **Pipeline:** PICO VR whole-body tracking → Custom 2-DoF robot neck (~$250) for egocentric vision → GMR retargeting → RL policy (IsaacGym) → Redis IPC → Ethernet to robot
- **Locomotion:** Yes, full-body
- **Latency:** ~39 Hz average
- **Key Strength:** Portable, mocap-free; 100 demos in 15 minutes; scalable data collection

#### GMR (General Motion Retargeting)
- **GitHub:** YanjieZe/GMR (~1,800 stars)
- **License:** MIT
- **Supported Robots:** **17 humanoids** — Unitree G1/H1/H1-2, Booster T1/K1, Fourier N1/GR3, ENGINEAI PM01, Stanford ToddlerBot, Galexea R1 Pro, Kuavo, Berkeley Humanoid Lite, PND Adam Lite, Tienkung, PAL Talos
- **Input Formats:** SMPLX (AMASS, OMOMO), BVH (LAFAN1, Nokov, Xsens), FBX (OptiTrack), PICO (XRoboToolkit)
- **Retargeting:** Optimization-based IK, real-time on CPU, velocity limits configurable
- **Key Strength:** Widest robot support of any retargeting library; the de facto standard for humanoid whole-body retargeting

#### OmniH2O / Human2Humanoid
- **GitHub:** LeCAR-Lab/human2humanoid (~964 stars)
- **License:** CC BY-NC 4.0 (non-commercial only)
- **VR Hardware:** Apple Vision Pro; also RGB camera (HybrIK pose estimation)
- **Supported Robots:** Unitree H1 EDU with Inspire Hands
- **Pipeline:** VisionPro or RGB camera → 3-step retargeting (FK, SMPL shape fitting, gradient optimization on AMASS data) → RL teacher policy → DAgger student distillation → Unitree SDK2 + custom C++ (Inspire Hand) → ZED mini for vision; NVIDIA Orin NX onboard
- **Locomotion:** Yes, whole-body with mobility
- **Hand Tracking:** Yes, Inspire Hand dexterous manipulation
- **Key Strength:** First universal dexterous human-to-humanoid system; released OmniH2O-6 dataset

#### CLONE (Closed-Loop Whole-Body Humanoid Teleoperation)
- **GitHub:** humanoid-clone/CLONE (~256 stars)
- **License:** CC BY-NC 4.0 (non-commercial only)
- **VR Hardware:** Apple Vision Pro
- **Pipeline:** Vision Pro → wrist 6D poses + head 3D position → PHC (Physics-based Human Motion Capture) retargeting → MoE-based policy with closed-loop error correction → LiDAR odometry for state estimation
- **Locomotion:** Yes, whole-body for long-horizon tasks
- **Key Strength:** Closed-loop error correction (most other systems are open-loop)

#### HumanPlus
- **GitHub:** MarkFzp/humanplus (~825 stars)
- **License:** Not specified (Stanford)
- **VR Hardware:** None — uses single RGB camera with WHAM (body) + HaMeR (hand) pose estimation
- **Supported Robots:** Unitree H1 (33-DoF)
- **Pipeline:** RGB camera (25 Hz) → WHAM body + HaMeR hand estimation → Humanoid Shadowing Transformer (HST, 50 Hz) → RL-trained in legged_gym → sim-to-real; ROS 2 + Unitree SDK2; Jetson edge
- **Locomotion:** Yes, full-body
- **Hand Tracking:** Yes, via HaMeR
- **Key Strength:** No VR headset needed at all; 60-100% success on diverse tasks (shoe wearing, shelf unloading, folding, typing)

---

### 3.2 Tier 2: Upper-Body / Arm+Hand VR Teleoperation

#### Unitree XR Teleoperate
- **GitHub:** unitreerobotics/xr_teleoperate (~1,300 stars)
- **VR Hardware:** Apple Vision Pro, Meta Quest 3, PICO 4 Ultra Enterprise
- **Supported Robots:** Unitree G1 (23/29 DoF), H1 (4-DoF arm), H1_2 (7-DoF arm)
- **End-Effectors:** Dex1-1 gripper, Dex3-1 hand, Inspire DFX/FTP, BrainCo hand
- **Pipeline:** XR device → TeleVuer module → CycloneDDS to robot → Pinocchio CLIK for IK → SE(3) interpolation → Nullspace projection near singularities → dex-retargeting for hands
- **Locomotion:** Yes, in "motion control mode" (R3 controller for walking)
- **Latency:** 30 FPS default (configurable)
- **Key Strength:** Most mature manufacturer-supported XR teleop; broadest XR device support

#### Open-TeleVision
- **GitHub:** OpenTeleVision/TeleVision (~1,200 stars)
- **VR Hardware:** Apple Vision Pro, Meta Quest 3
- **Pipeline:** Stereoscopic ZED camera → WebSocket (WSS) for VisionPro, HTTPS for local → ngrok tunneling for remote → ACT (Action Chunking with Transformers) for imitation learning; DETR detection
- **Key Strength:** Immersive stereoscopic "active visual feedback"; demonstrated cross-coast internet teleoperation (MIT → UC San Diego)

#### FFTAI Teleoperation (Fourier)
- **GitHub:** FFTAI/teleoperation (~154 stars)
- **VR Hardware:** Apple Vision Pro, Meta Quest 3
- **Supported Robots:** Fourier GR1T1, GR1T2, GR2
- **End-Effectors:** Fourier 6-DoF hand, 12-DoF hand, Inspire hand
- **Pipeline:** WebXR → WebSocket data streaming → Multi-camera support (OAK-D, RealSense, ZED, USB) → Data collection for VLA training
- **Key Strength:** Integrated data collection for VLA training; multi-camera support

#### K-Scale K-Bot VR Teleoperation
- **Docs:** docs.kscale.dev/robots/k-bot/teleop/
- **GitHub:** kscalelabs/kbot_vr_teleop
- **VR Hardware:** Meta Quest Pro, Quest 2, Quest 3 (browser-based — no app install needed)
- **Supported Robots:** K-Bot humanoid (4'7", 77 lbs, $8,999)
- **Pipeline:** Quest browser app (React) → WebSocket to signaling server → UDP to robot firmware → JAX/JAXopt IK on intermediary PC → WebRTC video streaming → GStreamer encoding; Rerun visualization
- **Locomotion:** Yes, joystick-controlled lower body
- **Key Strength:** Fully open-source humanoid + teleop stack; browser-based (no app install)

#### Reachy 2 Teleoperation (Pollen Robotics)
- **GitHub:** pollen-robotics/Reachy2Teleoperation (~16 stars)
- **License:** Apache 2.0
- **VR Hardware:** Meta Quest 2, Quest 3 (recommended), HTC Vive, Valve Index
- **Pipeline:** Unity LTS 2022.3 app → GStreamer WebRTC → gRPC API (`SendFullBodyCartesianCommands`) → Robot onboard IK (`ComputeArmIK`, `ComputeHeadIK`) → Progressive control (connect → configure → A-button for arms)
- **Latency:** **74ms motion, 135ms glass-to-glass video** (only project documenting both)
- **Key Strength:** Commercial-grade humanoid; documented latency numbers; now open-sourced via HuggingFace

#### OpenArm (via teleop_xr)
- **GitHub:** enactic/openarm + qrafty-ai/teleop_xr
- **VR Hardware:** Any WebXR headset (via teleop_xr framework)
- **Pipeline:** WebXR frontend → Pyroki differentiable IK → WebRTC video → Also supports physical bilateral teleoperation with disturbance-observer-based acceleration control for sensorless force estimation
- **Haptic Feedback:** Yes — bilateral force feedback through leader arm (no extra hardware needed)
- **Key Strength:** Only system offering true bilateral force feedback without specialized haptic hardware (besides iCub3)

---

### 3.3 Tier 3: VR-to-Robot Arm Frameworks

#### Open-Teach
- **GitHub:** aadhithya14/Open-Teach (~359 stars), MIT license
- **VR Hardware:** Meta Quest 3
- **Supported Robots:** Franka, xArm, Jaco, Allegro Hand, Hello Stretch
- **Latency:** **90 Hz control loop**
- **Key Strength:** 38 tasks across diverse robot morphologies; most versatile arm teleop

#### HATO
- **GitHub:** ToruOwO/hato (~166 stars)
- **VR Hardware:** Meta Quest 2
- **Supported Robots:** UR5e (single or bimanual), Ability Hand, Robotiq gripper
- **Latency:** 10 Hz
- **Key Strength:** Visuotactile learning; multimodal data (RGB-D, touch, position)

#### Holo-Dex
- **GitHub:** SridharPandian/Holo-Dex (~54 stars)
- **VR Hardware:** Oculus Quest (via APK)
- **Supported Robots:** Allegro Hand + Kinova Arm
- **Latency:** 30 Hz
- **Key Strength:** Mixed reality immersion for dexterous manipulation

#### BunnyVisionPro
- **GitHub:** Dingry/BunnyVisionPro (~345 stars), MIT license
- **VR Hardware:** Apple Vision Pro
- **Supported Robots:** xArm7, Ability Hand
- **Pipeline:** Vision Pro → dex-retargeting (DexPilot optimizer) → Singularity checks + collision avoidance → Docker server; pip client
- **Key Strength:** Safety-first design with haptic feedback; bimanual dexterous focus

#### VisionProTeleop (Improbable-AI)
- **GitHub:** Improbable-AI/VisionProTeleop (~736 stars), MIT license
- **VR Hardware:** Apple Vision Pro
- **Pipeline:** visionOS "Tracking Streamer" app → `avp_stream` Python library (pip) → WebRTC bidirectional → Room codes + TURN relay (no same-network required) → 26-joint hand skeleton data with prediction
- **Key Strength:** Most polished Vision Pro streaming library; building block for BunnyVisionPro, CLONE, and others

#### Spes Robotics Teleop
- **GitHub:** SpesRobotics/teleop (~183 stars), Apache 2.0
- **VR Hardware:** Any WebXR device; smartphones (30 fps), VR controllers (90 fps)
- **Supported Robots:** uFactory Lite 6, UR5e, extensible
- **Pipeline:** WebXR API → Flask Python server → Pinocchio IK → ROS 2 integration (cartesian_controllers, joint trajectory)
- **Key Strength:** Lowest barrier to entry; phone-based teleoperation possible

#### Qrafty TeleopXR
- **GitHub:** qrafty-ai/teleop_xr (~30 stars), Apache 2.0
- **VR Hardware:** Any WebXR headset
- **Supported Robots:** Unitree H1, Franka Panda, SO101, OpenArm, TeaArm
- **Pipeline:** WebXR → Pyroki differentiable IK + collision checking → WebRTC video → Three modes: live teleop, relative delta, absolute target → ROS 2 and dora-rs interfaces
- **Key Strength:** Differentiable IK with collision avoidance; H1 whole-body support

#### Unity ROS Teleoperation (ETH Zurich)
- **GitHub:** leggedrobotics/unity_ros_teleoperation (~138 stars), BSD-3
- **VR Hardware:** Quest 3 (primary), any OpenXR device
- **Supported Robots:** ANYmal, ALMA, Franka Panda, Dynaarm, GR2, B2W, Tytan
- **Key Strength:** Most feature-rich visualization (NeRF, point clouds, audio); Bhaptics glove haptic feedback

#### XRoboToolkit (PICO / XR Robotics)
- **GitHub:** XR-Robotics (~80 stars)
- **VR Hardware:** PICO 4 Ultra, Meta Quest, any OpenXR device
- **Latency:** <100ms, <1% packet loss
- **Key Strength:** Cross-platform SDK; used by TWIST2 for PICO integration

#### Humanoid-Teleoperation (YanjieZe)
- **GitHub:** YanjieZe/Humanoid-Teleoperation (~199 stars), MIT
- **VR Hardware:** Apple Vision Pro (required)
- **Supported Robots:** Fourier GR1 with Inspire Hands
- **Key Strength:** Clean Vision Pro to Diffusion Policy training pipeline

#### LeRobot/Phospho
- **VR Hardware:** Meta Quest Pro, Quest 2, Quest 3, Quest 3s (native Meta Store app)
- **Pipeline:** Phospho app → WiFi → Phosphobot server → Robot. Auto-saves in LeRobot v2 format → HuggingFace Hub upload
- **Key Strength:** Easiest setup for data collection; native Quest app on Meta Store

#### Quest2ROS / Quest2ROS2
- **GitHub:** Quest2ROS/quest2ros (~36 stars)
- **VR Hardware:** Meta Quest 2, Quest 3
- **Pipeline:** Standalone Quest app → ROS TCP Endpoint → PoseStamped, Twist, buttons → Haptic feedback subscriber (vibration frequency + amplitude)
- **Key Strength:** Simple ROS bridge; haptic feedback via controller vibration

#### dex-teleop (GeneralTrajectory)
- **GitHub:** GeneralTrajectory/dex-teleop (~61 stars), MIT
- **VR Hardware:** HTC Vive Tracker 3.0 + Base Stations (arms); Meta Quest 2/3/Pro (hands)
- **Latency:** **100 Hz arm, 60 Hz hands** (highest documented)
- **Key Strength:** Dual-VR-system approach; separate frequencies per subsystem

---

### 3.4 Retargeting Libraries

#### dex-retargeting
- **GitHub:** dexsuite/dex-retargeting (~819 stars), MIT
- **Optimizers:** PositionOptimizer, DexPilotOptimizer, SeqRetargeting
- **Input:** Human hand video (MediaPipe), hand-object datasets
- **Robot Hands:** Any URDF via dex-urdf library
- **Dependencies:** Pinocchio + MediaPipe
- **Key Strength:** Standard library for hand retargeting; used by BunnyVisionPro, AnyTeleop, Unitree XR

#### AnyTeleop
- **Paper:** RSS 2023
- **Components released as:** dex-retargeting + dex-urdf
- **Key Strength:** Unified framework for different arms, hands, and realities

---

### 3.5 Physical Teleoperation (Non-VR, for Reference)

#### Mobile ALOHA
- **GitHub:** MarkFzp/mobile-aloha (~4,400 stars), MIT
- **Teleoperation:** Physical leader-follower (4x Interbotix XS arms + AgileX Tracer)
- **Key Strength:** Most popular teleoperation project; HDF5 + ACT pipeline

#### GELLO
- **GitHub:** wuphilipp/gello_software (~418 stars), MIT
- **Teleoperation:** Physical leader-follower (3D-printed kinematic replica, <$300)
- **Supported Robots:** Franka FR3, UR, xArm, YAM
- **Key Strength:** Intuitive kinematic-matched teleop; extremely low cost

#### UMI (Universal Manipulation Interface)
- **GitHub:** real-stanford/universal_manipulation_interface (~1,300 stars), MIT
- **Teleoperation:** Hand-held gripper + SpaceMouse
- **Key Strength:** Hardware-agnostic policy transfer; in-the-wild data collection

#### DexCap
- **GitHub:** j96w/DexCap (~357 stars), MIT
- **Hardware:** Rokoko mocap gloves, chest cameras, Intel NUC
- **Key Strength:** Portable mocap for dexterous demos; claims 3x faster than VR teleop

---

## 4. Projects from the Original Report

### Projects WITH VR Teleoperation

| Project | VR Hardware | VR Type | Details |
|---------|------------|---------|---------|
| **OpenArm** | Any WebXR (via teleop_xr) | Upper body + bilateral force | Pyroki IK, collision-aware, force feedback through leader arm |
| **Reachy 2** | Quest 2/3, VIVE, Index | Upper body + head | Unity app, gRPC, 74ms motion / 135ms video latency |
| **iCub3** | VIVE Pro Eye + full body suit | **Full-body + haptics** | 20N/finger force, body suit, treadmill, facial expressions, 1 kHz |
| **K-Scale K-Bot** | Quest Pro/2/3 (browser) | Upper body + joystick legs | JAX IK, WebSocket, browser-based (no install) |
| **NVIDIA GR00T/SONIC** | PICO + ankle trackers | **Whole-body** | Foundation model, 42M params, 100% zero-shot success |
| **NVIDIA Isaac Lab** | Vision Pro, Quest 3, PICO | Sim teleop | CloudXR, Se3Rel/Abs + Dexpilot retargeters |
| **LeRobot** | Quest Pro/2/3/3s (Phospho) | Upper body | Native Quest app, LeRobot v2 format, HuggingFace upload |
| **Unitree** | AVP, Quest 3, PICO 4 Ultra | Upper body + locomotion | Pinocchio CLIK, dex-retargeting, CycloneDDS, 30 Hz |
| **PAL TALOS** | Quest 3 | Upper body + WBC | Whole Body Controller with self-collision avoidance (proprietary) |

### Projects WITHOUT VR Teleoperation

| Project | Teleoperation Method | Notes |
|---------|---------------------|-------|
| **Reachy Mini** | No teleop (autonomous behaviors) | Has app ecosystem but no VR |
| **InMoov ROS2** | Community Quest 2 project (archived) | Unity → Arduino serial, very basic |
| **Red Rabbit RX1** | Physical exoskeleton arms | Leader-follower, no VR, ROS1 |
| **Upkie** | No teleop | Balance-focused |
| **Poppy Humanoid** | No VR teleop | Abandoned |
| **Drake** | N/A (framework) | |
| **Pinocchio** | N/A (library) | Used by other teleop projects for IK |
| **mjbots** | N/A (motor controller) | |

---

## 5. Comparison Tables

### 5.1 VR Headset Ecosystem Support

| Headset | Projects Supporting |
|---------|-------------------|
| **Meta Quest 3** | Unitree XR, Open-TeleVision, FFTAI, Open-Teach, K-Bot, TeleopXR, Unity ROS, Quest2ROS, dex-teleop, Reachy 2, PAL |
| **Meta Quest 2** | HATO, Holo-Dex, K-Bot, Quest2ROS, Reachy 2, dex-teleop, OpenVR, Phospho |
| **Meta Quest Pro** | K-Bot, dex-teleop, Phospho |
| **Apple Vision Pro** | Unitree XR, Open-TeleVision, FFTAI, OmniH2O, CLONE, Humanoid-Teleop, BunnyVisionPro, VisionProTeleop, NVIDIA Isaac Lab |
| **PICO 4 Ultra** | Unitree XR, SONIC/TWIST/TWIST2, XRoboToolkit, NVIDIA Isaac Lab |
| **HTC VIVE** | dex-teleop, Reachy 2, iCub3 (VIVE Pro Eye) |
| **Any OpenXR** | TeleopXR, Spes Teleop, Unity ROS Teleop, XRoboToolkit |

### 5.2 Latency Benchmarks

| System | Control Freq | Glass-to-Glass Latency | Notes |
|--------|-------------|----------------------|-------|
| iCub3 | **1 kHz** PD, 500 Hz walk | **<25ms** (local fiber) | Gold standard |
| dex-teleop | **100 Hz** arm, 60 Hz hand | — | Highest documented |
| Open-Teach | **90 Hz** | — | |
| SONIC | 50 Hz policy, 500 Hz PD | 122ms wrist tracking | |
| TWIST/TWIST2 | ~39-50 Hz | — | |
| HumanPlus | 50 Hz output | — | 25 Hz camera input |
| Reachy 2 | — | **135ms** video, 74ms motion | Only project documenting both |
| XRoboToolkit | — | **<100ms** | <1% packet loss |
| Unitree XR | 30 Hz default | — | Configurable |

### 5.3 Communication Protocol Breakdown

| Protocol | Used By |
|----------|---------|
| **WebSocket** | Unitree XR, FFTAI, K-Scale, Open-TeleVision, XRoboToolkit |
| **WebRTC** | VisionProTeleop, K-Bot (video), TeleopXR, Reachy 2, NVIDIA CloudXR |
| **ZMQ** | SONIC/GR00T, HATO |
| **Redis** | TWIST, TWIST2 (IPC) |
| **ROS TCP Endpoint** | Quest2ROS, Unity ROS Teleop |
| **ROS 2 native** | Spes Teleop, TeleopXR, HumanPlus |
| **CycloneDDS** | Unitree XR (robot control) |
| **gRPC** | Reachy 2 |
| **UDP raw** | K-Bot (firmware), dex-teleop |
| **YARP** | iCub3 |

### 5.4 Feature Matrix: Full VR Teleop Systems

| Feature | SONIC | TWIST2 | OmniH2O | Unitree XR | Reachy 2 | K-Bot | iCub3 |
|---------|:-----:|:------:|:-------:|:----------:|:--------:|:-----:|:-----:|
| Walking | Yes | Yes | Yes | Joystick | Mobile base | Joystick | Treadmill |
| Arm control | Yes | Yes | Yes | Yes | Yes | Yes | Yes |
| Hand dexterity | Bimanual | No | Inspire | Multi-hand | Gripper | WIP | SenseGlove |
| Stereo video | No | No | ZED | Multi-camera | WebRTC | WebRTC | YARP |
| Haptic feedback | No | No | No | No | No | No | **20N/finger** |
| Quest support | No | No | No | Yes | Yes | Yes | No |
| Vision Pro | No | No | Yes | Yes | No | No | No |
| PICO support | Yes | Yes | No | Yes | No | No | No |
| Open source | Yes | Yes | NC only | Yes | Yes | Yes | Yes |
| RL training | Yes | Yes | Yes | No | No | No | No |
| Data collection | Yes | Yes | Yes | No | Yes | No | No |

---

## 6. Recommendations for SteveBots

### 6.1 Most Relevant Architecture for SteveBots

Given SteveBots' existing stack (ROS 2 + ros2_control + MuJoCo + MediaPipe), the most practical VR teleop path is:

#### Option A: Meta Quest 3 + WebXR (Recommended — Lowest Cost)

```
Meta Quest 3 ($500)
    ↓ WebXR API (browser, no app install)
WebSocket / WebRTC server on robot PC
    ↓
ROS 2 node: vr_to_joints
    ├─ Receives 6-DOF hand poses from Quest
    ├─ Pinocchio CLIK for arm IK
    ├─ OR: Reuse existing pose_to_joints architecture with VR input
    └─ Publishes /right_arm_controller/joint_trajectory
            /left_arm_controller/joint_trajectory
    ↓
Existing ros2_control hardware interface (MuJoCo or real CAN)
```

**Why this approach:**
- Quest 3 is the most widely supported headset ($500)
- WebXR works in the browser — no Unity/native app needed
- Reuses your existing ros2_control + joint trajectory pipeline
- Spes Robotics (`SpesRobotics/teleop`) provides a working WebXR → Python server in Apache 2.0
- TeleopXR (`qrafty-ai/teleop_xr`) already supports Unitree H1 with Pyroki IK

#### Option B: Build on Unitree XR Teleoperate (More Features)

Fork or adapt `unitreerobotics/xr_teleoperate` which already supports:
- Quest 3, Vision Pro, and PICO
- Pinocchio CLIK for arm IK
- dex-retargeting for hands
- CycloneDDS communication
- Multiple display modes (immersive, ego, pass-through)

The main adaptation: replace Unitree's robot model with K-Bot URDF and adjust IK target frames.

#### Option C: Leverage Existing MediaPipe Pipeline (Unique Approach)

SteveBots already has a working camera → pose → joints pipeline via MediaPipe. This could be extended:
1. Mount a stereo camera on the robot head
2. Stream stereo video to VR headset via WebRTC
3. Keep using MediaPipe pose tracking with the camera watching the operator
4. No new IK code needed — reuse `pose_to_joints`

This is unique in the ecosystem — no other project uses MediaPipe for VR-like teleoperation.

### 6.2 Implementation Roadmap

| Phase | What | Effort | Dependencies |
|-------|------|--------|-------------|
| **Phase 1** | WebXR bridge: Quest 3 hand poses → ROS 2 topic | 2-3 days | Quest 3, WebSocket server |
| **Phase 2** | VR-to-joints node: hand 6-DOF → arm joint commands via IK | 3-5 days | Pinocchio or Pyroki |
| **Phase 3** | Stereo video: camera on robot → WebRTC → Quest 3 | 3-5 days | ZED camera, GStreamer |
| **Phase 4** | MuJoCo sim integration: test VR teleop in simulation first | 1-2 days | Existing MuJoCo setup |
| **Phase 5** | Data collection: record VR demos in LeRobot format | 2-3 days | HDF5 recording node |

### 6.3 Key Libraries to Use

| Library | Purpose | License |
|---------|---------|---------|
| **Pinocchio** (stack-of-tasks/pinocchio) | IK solver for arm control | BSD-2 |
| **dex-retargeting** (dexsuite/dex-retargeting) | Hand pose retargeting (future) | MIT |
| **GMR** (YanjieZe/GMR) | Whole-body retargeting (future locomotion) | MIT |
| **avp_stream** (Improbable-AI/VisionProTeleop) | Vision Pro streaming (if using AVP) | MIT |
| **GStreamer** | Video encoding/streaming | LGPL |
| **aiortc** (Python WebRTC) | WebRTC in Python | BSD |

### 6.4 What Makes SteveBots Unique

SteveBots' MediaPipe-based teleoperation is genuinely novel in this space:
- **No headset required**: Camera-based approach is the cheapest entry point (~$0 additional hardware)
- **Complementary to VR**: MediaPipe teleop for quick demos, VR teleop for precision manipulation
- **Data collection**: Both modalities can feed the same training pipeline
- **Accessible**: Anyone with a webcam can teleoperate; VR is an upgrade path for serious use

---

## Appendix A: Complete Project List (33 Projects)

| # | Project | GitHub Stars | VR Hardware | Type |
|---|---------|:-----------:|-------------|------|
| 1 | NVIDIA SONIC/GR00T WBC | ~1,100 | PICO | Whole-body |
| 2 | GMR | ~1,800 | Multiple | Retargeting library |
| 3 | Unitree XR Teleoperate | ~1,300 | Quest/AVP/PICO | Upper-body |
| 4 | Open-TeleVision | ~1,200 | AVP/Quest 3 | Upper-body |
| 5 | OmniH2O | ~964 | AVP/RGB camera | Whole-body |
| 6 | HumanPlus | ~825 | RGB camera (no VR) | Whole-body |
| 7 | dex-retargeting | ~819 | N/A (library) | Retargeting |
| 8 | VisionProTeleop | ~736 | Vision Pro | Streaming library |
| 9 | TWIST | ~707 | PICO/MoCap | Whole-body |
| 10 | TWIST2 | ~711 | PICO | Whole-body |
| 11 | Open-Teach | ~359 | Quest 3 | Arm teleop |
| 12 | DexCap | ~357 | MoCap gloves | Physical teleop |
| 13 | BunnyVisionPro | ~345 | Vision Pro | Arm+hand |
| 14 | CLONE | ~256 | Vision Pro | Whole-body |
| 15 | Humanoid-Teleop | ~199 | Vision Pro | Arm+hand |
| 16 | Spes Teleop | ~183 | Any WebXR | Arm teleop |
| 17 | HATO | ~166 | Quest 2 | Arm+hand |
| 18 | FFTAI Teleop | ~154 | AVP/Quest 3 | Upper-body |
| 19 | Unity ROS Teleop (ETH) | ~138 | Quest 3/OpenXR | Arm+viz |
| 20 | XRoboToolkit | ~80 | PICO/Quest | SDK |
| 21 | dex-teleop | ~61 | VIVE+Quest | Arm+hand |
| 22 | Holo-Dex | ~54 | Quest | Arm+hand |
| 23 | Quest2ROS | ~36 | Quest 2/3 | ROS bridge |
| 24 | TeleopXR | ~30 | Any WebXR | Arm+IK |
| 25 | Reachy 2 Teleop | ~16 | Quest 2/3/VIVE | Upper-body |
| 26 | iCub3 Avatar | N/A | VIVE Pro + full suit | Full-body+haptics |
| 27 | K-Scale K-Bot | N/A | Quest Pro/2/3 | Upper-body |
| 28 | PAL TALOS VR | N/A | Quest 3 | Upper-body (proprietary) |
| 29 | OpenArm (teleop_xr) | ~1,897 | Any WebXR | Upper-body+force |
| 30 | LeRobot/Phospho | ~8,000 | Quest (native app) | Upper-body |
| 31 | InMoov VR | ~8 (archived) | Quest 2 | Basic (Unity→Arduino) |
| 32 | OpenVR (Franka) | N/A | Quest 2 | Simple arm |
| 33 | LeVR | N/A | Quest (OpenXR) | Framework adapter |

## Appendix B: Key Architectural Insights

1. **WebRTC is the dominant video streaming protocol.** K-Scale, Unitree, Reachy 2, NVIDIA (CloudXR), OpenArm (via teleop_xr), and PAL all use WebRTC. Only iCub3 uses YARP's custom image carriers.

2. **Pinocchio is the standard open-source IK library.** Unitree's `xr_teleoperate`, Open-TeleVision, and NVIDIA Isaac Lab all build on Pinocchio CLIK. K-Scale chose JAX/JAXopt as an alternative for GPU-accelerated IK. Pyroki is a newer differentiable IK option.

3. **iCub3 is uniquely comprehensive.** Only system with full-body haptic feedback (20N/finger force feedback gloves, body suit vibration, haptic shoes), omnidirectional treadmill, facial expression mirroring, and voice relay. 1 kHz control, sub-25ms local latency.

4. **Browser-based vs. native apps.** K-Scale's browser web app (no install) is most frictionless. Reachy 2 and PAL require Unity/WebXR apps. Phospho distributes native Quest app via Meta Store. Unitree uses web-based TeleVuer.

5. **Data collection is a primary motivator.** LeRobot/Phospho, NVIDIA Isaac Lab, Unitree, and Open-TeleVision all emphasize VR as a means to collect demo data for imitation learning. NVIDIA's pipeline generates 780,000 synthetic trajectories (6,500 hours) in 11 hours from VR demos.

6. **Retargeting complexity varies dramatically.** From direct joint mapping (RX1, InMoov) through Cartesian IK (Reachy 2, PAL) to whole-body geometric retargeting with QP (iCub3) and learned universal token spaces (SONIC). Trend is toward learned retargeting that generalizes across embodiments.

7. **Haptic feedback remains rare.** Only OpenArm (bilateral force) and iCub3 (full-body haptic suit) provide meaningful force feedback. This is a significant gap for manipulation tasks.

---

*Report compiled 2026-03-10 for the SteveBots/Steveros project by a multi-agent research team using Claude Opus 4.6.*
