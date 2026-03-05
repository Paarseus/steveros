# Teleoperation & Pose Mimicking: Comprehensive Literature Review

**Date:** March 2026
**Scope:** State-of-the-art methods for robot teleoperation, pose estimation, motion retargeting, and imitation learning (2023-2026)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Pose Estimation Models](#2-pose-estimation-models)
3. [Hand Pose Estimation](#3-hand-pose-estimation)
4. [Full-Body Motion Capture from Video](#4-full-body-motion-capture-from-video)
5. [Vision-Based Teleoperation Systems](#5-vision-based-teleoperation-systems)
6. [Single-Camera (Monocular) Teleoperation](#6-single-camera-monocular-teleoperation)
7. [VR & Haptic Teleoperation](#7-vr--haptic-teleoperation)
8. [Custom Hardware Interfaces](#8-custom-hardware-interfaces)
9. [Imitation Learning from Teleoperation](#9-imitation-learning-from-teleoperation)
10. [Foundation Models (VLAs)](#10-foundation-models-vlas)
11. [Shared Autonomy & Assisted Teleoperation](#11-shared-autonomy--assisted-teleoperation)
12. [Latency Compensation](#12-latency-compensation)
13. [Sim-to-Real Transfer](#13-sim-to-real-transfer)
14. [Key Datasets & Benchmarks](#14-key-datasets--benchmarks)
15. [Motion Retargeting](#15-motion-retargeting)
16. [Comparative Analysis](#16-comparative-analysis)
17. [Recommendations for Our Project](#17-recommendations-for-our-project)
18. [References](#18-references)

---

## 1. Executive Summary

The teleoperation landscape has undergone a rapid transformation from 2023-2026, driven by three converging trends:

1. **Cost democratization**: Hardware costs dropped from $200K+ (commercial systems) to $300 (GELLO) to $60 (U-ARM), with UMI eliminating the robot entirely during data collection.

2. **Vision-Language-Action (VLA) models**: Pi0, OpenVLA, RT-2, and Gemini Robotics have established VLMs as the backbone for robot policies. Open-source availability (Pi0, OpenVLA, SmolVLA) is accelerating adoption.

3. **Diffusion/flow matching dominance**: Diffusion Policy and its variants have largely supplanted simple behavior cloning for policy learning, with flow matching (Pi0) emerging as a faster alternative.

**For our single-camera MediaPipe + ROS2 setup**, the research validates this as a viable low-cost entry point, while identifying clear upgrade paths (One Euro filtering, depth cameras, leader-follower arms) depending on precision requirements.

---

## 2. Pose Estimation Models

### 2.1 Model Comparison Table

| Model | COCO AP | Speed (FPS) | Parameters | Real-Time | Open Source | Fine-Tunable |
|-------|---------|-------------|------------|-----------|-------------|-------------|
| **MediaPipe BlazePose** | ~75* | 30+ (mobile CPU) | ~3-26MB | Yes | Yes | No |
| **RTMPose-m** | 75.8 | 90+ CPU, 430+ GPU | ~13M | Yes | Yes (Apache 2.0) | Yes |
| **RTMO** (CVPR 2024) | 74.8 | 141 (V100) | - | Yes | Yes | Yes |
| **ViTPose-G** | 80.9 | <30 (GPU) | ~1B | No | Yes | Yes |
| **ViTPose-B** | 75.8 | ~30 (GPU) | ~86M | Marginal | Yes | Yes |
| **HRNet-W48** | 74.9 | ~30 (GPU) | ~63M | Marginal | Yes | Yes |
| **OpenPose** | ~61 | <15 (GPU) | - | No | $25K/yr commercial | Limited |
| **YOLO11 Pose** | Competitive | 30+ (T4 GPU) | Varies | Yes | Yes | Yes |

*MediaPipe uses a different evaluation methodology; direct AP comparison is approximate.

### 2.2 MediaPipe BlazePose (Current System)

- Detects 33 3D body landmarks in real-time on mobile/edge devices
- Mean Pearson's correlation of 0.91 (upper limb) vs. Qualisys gold-standard mocap
- Average joint angle error ~10 degrees
- **Limitations**: Cannot be fine-tuned, single-person only, depth ambiguity from monocular estimation, sensitive to motion blur

### 2.3 RTMPose / RTMO (Recommended Upgrade)

RTMPose (OpenMMLab/MMPose) achieves the best speed-accuracy trade-off:
- RTMPose-m: 75.8% AP at 90+ FPS on CPU (i7-11700)
- RTMO (CVPR 2024): One-stage variant, 74.8% AP at 141 FPS on V100
- RTMW (2024): Whole-body extension with 2D and 3D estimation
- **Supports**: ONNX, TensorRT, ncnn, OpenVINO, RKNN across CPU/GPU/Jetson/mobile

### 2.4 ViTPose / ViTPose++ (Maximum Accuracy)

- ViTPose-G: **80.9 AP** (COCO test-dev) — current single-model record
- ViTPose ensemble: **81.1 AP** — #1 on COCO leaderboard
- ViTPose++: Multi-task training (body, whole-body, animal) via task-specific FFN experts
- Extensions: Poseidon (multi-frame, 87.8-88.3 mAP on PoseTrack), EViTPose (30-44% fewer GFLOPs)

### 2.5 3D Pose from Monocular Camera

| Model | Approach | Accuracy | Compute | Year |
|-------|----------|----------|---------|------|
| **PoseMamba** | State-space model | Best (MPJPE) | 16% of MotionBERT | 2024 |
| **MotionAGFormer** | Transformer-GCN hybrid | Competitive | Moderate | WACV 2024 |
| **MotionBERT** | Transformer lifting | Strong baseline | High | 2023 |
| **FinePOSE** | Diffusion-based | Good for ambiguity | Moderate | 2024 |
| **MHFormer** | Multi-hypothesis transformer | Good | Moderate | 2022 |

**Trend**: State-space models (Mamba) and diffusion models are rapidly displacing pure transformers, offering dramatically lower compute with equal or better accuracy.

### 2.6 MMPose Framework

The most comprehensive open-source pose estimation toolbox (Apache 2.0):
- Tasks: 2D body, hand, face, whole-body (133 keypoints), animal pose
- Models: RTMPose, RTMO, RTMW, ViTPose, HRNet, and many more
- Deployment: ONNX, TensorRT, ncnn, OpenVINO
- **Key advantage over MediaPipe**: Fully customizable, fine-tunable on custom datasets
- GitHub: [open-mmlab/mmpose](https://github.com/open-mmlab/mmpose)

---

## 3. Hand Pose Estimation

### 3.1 Comparison

| Method | Type | Accuracy | Real-Time | Notes |
|--------|------|----------|-----------|-------|
| **MediaPipe Hands** | 21 landmarks | RMSE ~0.28 px | Yes (mobile) | Cannot fine-tune; no metacarpal landmarks |
| **WiLoR** (CVPR 2025) | MANO mesh | Best on FreiHAND/HO3D | Yes | End-to-end; pip installable |
| **HaMeR** (CVPR 2024) | MANO mesh | PA-MPJPE 6.0mm (FreiHAND) | Yes (GPU) | Strong in-the-wild performance |
| **FrankMocap** (2021) | Modular body+hand+face | Good | Yes | Inaccurate wrist rotations |
| **Hand4Whole** (2022) | Kinematic chain-aware | Good | Yes | Handles invisible hands |
| **InterWild** (2023) | Two-hand interaction | Good | Yes | Robust to hand-hand interaction |
| **Leap Motion 2** | Dedicated IR hardware | 0.7mm accuracy | Yes | Best for gloved/industrial scenarios |
| **MANUS Data Gloves** | Commercial gloves | Gold standard | Yes | ROS 2 integration; expensive |

### 3.2 Key Finding for Teleoperation

Vision-only hand tracking (MediaPipe, HaMeR) provides convenience but lacks the precision and occlusion robustness needed for fine dexterous manipulation. Exoskeleton/glove-based systems remain superior for high-quality data collection, though the gap is narrowing rapidly with WiLoR and HaMeR.

---

## 4. Full-Body Motion Capture from Video

### 4.1 SMPL/SMPL-X Body Model

SMPL-X (10,475 vertices, 54 joints, ~100 parameters) is increasingly the "universal language" for articulated human movement. Data captured in SMPL format can be retargeted to any robot embodiment.

### 4.2 Key Recovery Frameworks

| Method | Year | Key Innovation | Speed | Global Trajectory |
|--------|------|---------------|-------|-------------------|
| **WHAM** | CVPR 2024 | Fuses 2D lifting + video + SLAM camera velocity | 200 FPS | Yes |
| **GVHMR** | SIGGRAPH Asia 2024 | Gravity-view coordinates; robust to camera errors | Fast | Yes |
| **TRAM** | ECCV 2024 | Robustified DROID-SLAM + metric depth | Moderate | Yes (60% error reduction) |
| **TokenHMR** | CVPR 2024 | VQ-VAE tokenized pose; TALS loss | Fast | Per-frame |
| **HMR2.0** (4D Humans) | ICCV 2023 | Fully transformer-based SMPL regression | Fast | Per-frame |
| **SMPLer-X / SMPLest-X** | 2025 | Foundation models trained on tens of millions of poses | Varies | Varies |

### 4.3 Marker-Based vs. Markerless MoCap

| Aspect | Marker-Based (Vicon, OptiTrack) | Markerless / Video-Based |
|--------|--------------------------------|--------------------------|
| **Accuracy** | Sub-millimeter (63 ± 5 μm) | ~20-30mm under good conditions |
| **Cost** | >$150,000 + expert setup | <$700 (e.g., OpenCap with smartphones) |
| **Setup time** | Hours of calibration | Minutes |
| **Environment** | Lab-constrained | Anywhere |
| **Real-time** | Yes | Yes (with modern models) |

For most robotics teleoperation research, ~20-30mm accuracy of markerless approaches is sufficient for arm-level control but marginal for fine dexterous manipulation.

---

## 5. Vision-Based Teleoperation Systems

### 5.1 Depth Camera Approaches

| Camera | Technology | Depth Error | Status | Price |
|--------|-----------|-------------|--------|-------|
| **Intel RealSense D435i** | Stereo vision | 1-3% | Being discontinued | ~$300 |
| **Azure Kinect DK** | Time-of-Flight | Best-in-class | Discontinued | N/A |
| **Orbbec Femto Bolt/Mega** | Recommended successor | Good | Active | ~$300-400 |
| **Luxonis OAK-D** | Stereo + on-device NN | Good | Active | ~$200 |
| **ZED 2** (Stereolabs) | Stereo vision | Good | Active | ~$450 |

### 5.2 Learning-Based Pose Estimation for Teleop

- **RTMPose**: Strongest candidate for real-time teleoperation pipelines (90+ FPS CPU)
- **YOLO11 Pose**: Emerging production standard combining detection + pose in single pass
- **MoReNet** (ICIRA 2024): Vision-based retargeting enabling RGB-only dexterous hand teleoperation

---

## 6. Single-Camera (Monocular) Teleoperation

### 6.1 The Dominant Pattern

**Webcam → MediaPipe/RTMPose → Joint Angle Computation → ROS2 → MoveIt Servo / Joint Commands**

### 6.2 Key Open-Source Implementations

| Project | Robot | Approach | Framework |
|---------|-------|----------|-----------|
| [FrankaTeleop](https://github.com/gjcliff/FrankaTeleop) | Franka Panda 7-DOF | MediaPipe hand + MoveIt Servo | ROS2 Iron |
| [hand-teleop](https://github.com/Joeclinton1/hand-teleop) | LeRobot-compatible | MediaPipe/WiLoR/AprilTag | Python |
| [teleoperation-robot-arm](https://github.com/mernaahany/teleoperation-robot-arm) | 6-DOF arm | MediaPipe 21 landmarks → ROS | ROS |
| [LeRobot](https://github.com/huggingface/lerobot) | SO100/Koch/many | Leader-follower, phone, gamepad | Python |

### 6.3 Solving the Depth Ambiguity

| Approach | Description | Practicality |
|----------|-------------|-------------|
| **Relative motion mapping** | Map *changes* in 2D pose to *changes* in robot position | **Best for our setup** |
| **Workspace constraints** | Constrain depth using known robot workspace bounds | Good |
| **DepthNet decoupling** | Disentangle absolute depth from camera intrinsics | Complex |
| **Render & Compare** | Generate synthetic views to resolve depth/occlusion | Complex |
| **Humanoid model + optimization** | MediaPipe 2D + CoM loss + joint penalty | Moderate |

**Recommendation**: Relative motion mapping avoids the depth ambiguity problem almost entirely for control purposes.

### 6.4 Two Fundamental Retargeting Approaches

**Joint Space (Direct Angle Mapping)**:
- Pros: Simple, avoids IK singularities
- Cons: Unpredictable end-effector path with different proportions
- Best when: Human and robot have similar kinematics (humanoids)

**Cartesian Space (Task Space Mapping)**:
- Pros: Intuitive for operators; faster learning curve
- Cons: Requires real-time IK; singularity susceptible
- Best when: Human and robot have different proportions (almost always)

ICRA 2021 study found both achieve comparable performance after training, but operators develop proficiency **faster with Cartesian space**.

### 6.5 Jitter Reduction (Critical Challenge)

| Technique | Best For | Complexity |
|-----------|----------|------------|
| **Exponential moving average** | Starting point (alpha=0.3-0.5) | Low |
| **One Euro Filter** | Best default for interactive use | Low |
| **Dead zone / threshold** | Suppressing micro-jitter at rest | Low |
| **Kalman filter** | Predictive smoothing with motion model | Medium |
| **FLK (Filter with Learned Kinematics)** | State-of-the-art; handles dropped frames | High |
| **Velocity clamping** | Rejecting impossible spikes | Low |

### 6.6 When to Upgrade from Single Camera

| Upgrade To | When | Cost |
|------------|------|------|
| Depth camera (RealSense, ZED) | Need accurate depth for grasping | $200-700 |
| Stereo/multi-camera | Need occlusion resolution | $100-400 |
| VR headset (Quest 3) | Need precise 6-DOF hand tracking | $300-500 |
| Leader-follower arms (GELLO) | Need high-fidelity demos for IL | $300 |
| Leader-follower arms (ALOHA) | Need bimanual precision | $20K |
| MoCap suit | Need full-body; highest precision | $1K-50K |

**TeleOpBench finding**: Monocular vision yields the largest completion times and poor success on medium-to-hard tasks vs. MoCap/VR/exoskeletons, but "dramatically reduces deployment cost and complexity."

---

## 7. VR & Haptic Teleoperation

### 7.1 VR Headsets for Teleoperation

| Device | Hand Tracking Latency | Accuracy | Price | Key Projects |
|--------|----------------------|----------|-------|-------------|
| **Apple Vision Pro** | ~128ms | Superior precision & robustness | $3,500 | Bunny-VisionPro, Open-TeleVision |
| **Meta Quest 3** | ~70ms | 0.46mm (controller) | $500 | OPEN TEACH, DROID |
| **Meta Quest 3S** | ~70ms | Similar to Quest 3 | $300 | Budget option |
| **Meta Quest 2** | ~90ms | Good (controller) | $250 used | DROID data collection |

### 7.2 Key VR Teleoperation Systems

**Bunny-VisionPro** (UC San Diego, 2024):
- Apple Vision Pro + XArm7 + Ability Hand
- 11% higher success rate, 45% faster than AnyTeleop+
- Novel low-cost haptic feedback finger cots ($1.20 each)
- [GitHub](https://github.com/Dingry/BunnyVisionPro)

**Open-TeleVision** (CoRL 2024):
- Stereoscopic video streaming from robot to VR headset
- Remote operation demonstrated (MIT → UC San Diego over internet)
- Validated on 4 long-horizon tasks across 2 humanoid robots
- [GitHub](https://github.com/OpenTeleVision/TeleVision)

**OPEN TEACH** (CoRL 2024, NYU):
- Meta Quest 3, calibration-free, up to 90Hz
- 86% average success rate across 10 tasks
- Supports: Franka, xArm, Jaco, Allegro, Hello Stretch
- [GitHub](https://github.com/aadhithya14/Open-Teach)

**TeleMoMa** (UT Austin, 2024):
- Modular multi-modal teleoperation for mobile manipulation
- Supports: RGB-D cameras, VR, keyboard, joystick, phone — any combination
- Finding: VR + Vision combined overcomes individual limitations
- [GitHub](https://github.com/UT-Austin-RobIn/telemoma)

### 7.3 Haptic Feedback

**Key finding**: Haptic feedback significantly reduces maximum contact forces and mental workload, but benefits diminish at latencies >500ms.

| System | Type | Cost | Force Feedback |
|--------|------|------|----------------|
| Geomagic Touch/Phantom | Stylus, 6-DOF input / 3-DOF force | $3K-16K | Yes |
| DOGlove (2025) | Open-source, 21-DOF capture, 5-DOF force | <$600 | Yes |
| SenseGlove Nova 2 | Commercial VR gloves | ~$5K+ | Yes |
| Bunny-VisionPro cots | Low-cost ERM actuators | ~$5 | Basic vibrotactile |

**FACTR** (2025): Low-cost bilateral system with force feedback, gravity compensation. **64.7% improvement in task completion rate** and **83.3% improvement in ease of use**.

---

## 8. Custom Hardware Interfaces

### 8.1 Cost Comparison

| System | Cost | Assembly | Robot Support | Force Feedback | Open-Source |
|--------|------|----------|---------------|----------------|-------------|
| **U-ARM** | ~$60 | Hours | Multiple via LeRobot | No | Yes |
| **GELLO** | ~$300 | 30 min | Franka, UR5, xArm | Yes (2025) | Yes |
| **UMI Gripper** | ~$200 | Hours | Any (zero-shot transfer) | Implicit | Yes |
| **Koch v1.1** | ~$200-300 | Hours | Koch arms | No | Yes |
| **ALOHA 2** | ~$20K | Significant | ViperX/WidowX | Passive gravity comp | Yes |
| **Mobile ALOHA** | ~$32K | Major build | ViperX + mobile base | No | Yes |

### 8.2 GELLO (Berkeley, IROS 2024)

- 3D-printed kinematically-equivalent scaled replica of target robot arm
- Uses Dynamixel XL-330R servos ($35 each) as joint encoders
- User studies: faster completion, higher success vs. VR controllers and spacemice
- 2025: Force feedback extension added
- [GitHub (software)](https://github.com/wuphilipp/gello_software), [GitHub (hardware)](https://github.com/wuphilipp/gello_mechanical)

### 8.3 UMI (Universal Manipulation Interface, RSS 2024)

- Handheld parallel-jaw gripper with GoPro camera — **no robot needed during data collection**
- Faster and more ergonomic than traditional teleoperation
- Zero-shot deployment across UR5, Franka, quadrupeds, aerial manipulators
- Variants: Fast-UMI, DexUMI, UMI-on-Legs, UMI-on-Air
- [GitHub](https://github.com/real-stanford/universal_manipulation_interface)

### 8.4 ALOHA 2 (Google DeepMind/Stanford, 2024)

- 2× ViperX 6-DOF follower + 2× WidowX leader arms
- Gripper force reduced from 14.68N to 0.84N (dramatically reduces fatigue)
- Passive gravity compensation replaces rubber bands
- [GitHub](https://github.com/tonyzhaozh/aloha)

### 8.5 LeRobot (Hugging Face)

Unifying platform for robot learning:
- Standardized dataset format, pretrained policies (ACT, Diffusion Policy, Pi0)
- Hardware: SO-100 arm (~$100), Koch v1.1, LeKiwi mobile base
- 12,000+ GitHub stars; NVIDIA partnership for Isaac integration
- [GitHub](https://github.com/huggingface/lerobot)

---

## 9. Imitation Learning from Teleoperation

### 9.1 Policy Architectures

| Method | Architecture | Demos Needed | Inference Speed | Multimodal Actions | Open Source |
|--------|-------------|-------------|----------------|-------------------|-------------|
| **ACT** | CVAE + Transformer | ~50 per task | 0.01s (single forward pass) | Limited | Yes (LeRobot) |
| **Diffusion Policy** | Conditional DDPM | 100+ | Slower (iterative denoising) | Excellent | Yes |
| **3D Diffusion Policy** | Point cloud + diffusion | 10-40 | Moderate | Excellent | Yes |
| **Pi0** | Flow matching on VLM | Fine-tune: 500-2K | 50 Hz | Good | Yes (Feb 2025) |
| **OpenVLA** | DINOv2 + CLIP + Llama-2 | Fine-tune: 500-2K | Moderate | Good | Yes |

### 9.2 ACT (Action Chunking with Transformers)

- Predicts action *chunks* (sequences) rather than single actions, reducing compounding errors
- ~80M parameters, trains in ~5 hours on RTX 2080 Ti
- 80-96% success on fine bimanual manipulation from just 50 demos
- Extensions: InterACT (bimanual), Bi-ACT (force/torque), Haptic-ACT (80% vs 50% vision-only)
- [Paper](https://arxiv.org/abs/2304.13705)

### 9.3 Diffusion Policy

- Represents policies as conditional denoising diffusion processes
- 46.9% average improvement over prior SOTA across 15 tasks
- Gracefully handles multimodal action distributions
- Extensions: FlowPolicy (single-step inference), 3DP (sparse point clouds, 10-40 demos)
- [Paper](https://dl.acm.org/doi/10.1177/02783649241273668)

### 9.4 Data Collection Best Practices

- Keep demonstrations short and direct
- Move smoothly without pausing
- Haptic feedback during collection improves throughput by ~6%
- Interactive/human-in-the-loop methods (DAgger) significantly improve sample efficiency
- Data quality matters as much as quantity
- 50 demos sufficient for ACT; 500-2K for foundation model fine-tuning

---

## 10. Foundation Models (VLAs)

### 10.1 Evolution

```
BC-Z (2021) → RT-1 (2022) → RT-2 (2023) → Octo/OpenVLA (2024) → Pi0/Pi0.5 (2024-2025) → Gemini Robotics (2025)
```

### 10.2 Key Models

| Model | Org | Size | Data | Open Source | Key Innovation |
|-------|-----|------|------|-------------|---------------|
| **RT-2** | DeepMind | Large VLM | Google fleet | No | VLM fine-tuned for actions |
| **Octo** | UC Berkeley | 27-93M | Open X-Embodiment | Yes | Lightweight, flexible fine-tuning |
| **OpenVLA** | Stanford | 7B | Open X-Embodiment | Yes | DINOv2+CLIP+Llama-2, outperforms RT-2 |
| **Pi0** | Physical Intelligence | ~3B+ | 7 robots, 68 tasks | Yes (Feb 2025) | Flow matching on PaliGemma |
| **Pi0.5** | Physical Intelligence | - | Heterogeneous | Yes | Open-world generalization |
| **Gemini Robotics** | DeepMind | Large | Google fleet | No | Extends Gemini 2.0 to physical manipulation |
| **GR00T N1** | NVIDIA | - | Simulation + real | Partial | Dual-system VLA for humanoids |

### 10.3 Pi0 Deep Dive

- Flow matching (not diffusion) for continuous actions at 50 Hz
- Open-sourced February 2025 (weights + code on LeRobot)
- Pi0-FAST: 5× faster training via DCT-based action tokenization
- Pi0.5: First system to perform full kitchen/bedroom cleaning in unseen homes
- [GitHub](https://github.com/Physical-Intelligence/openpi)

---

## 11. Shared Autonomy & Assisted Teleoperation

### 11.1 Key Approaches

- **Sampling-based grasp/collision prediction** (2025): Neural nets predict costs for sampled configurations in real time
- **Shared autonomy via MPC** (2024): Model Predictive Control for goal-reaching assistance
- **Dexterous arm-hand VLA via shared autonomy** (2025): Human guides arm, autonomous policy handles hand control (~90% success across 50+ objects)

### 11.2 Safety During Teleoperation

| Method | Description | Source |
|--------|-------------|--------|
| Gesture interlocks | Thumbs-up to enable, thumbs-down to e-stop | FrankaTeleop |
| Control Barrier Functions | Guarantee forward invariance of safe states | [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0921889019306426) |
| MoveIt Servo | Built-in collision checking, joint limits, velocity scaling | ROS2 |
| Velocity/acceleration limits | Kalman smoothing with soft inequality constraints | Standard practice |

---

## 12. Latency Compensation

### 12.1 Strategies

| Strategy | Description | Best For |
|----------|-------------|----------|
| **Predictive displays** | Render predicted future states | Visual feedback |
| **Model-mediated teleop** | Local model responds instantly, syncs periodically | High-latency remote |
| **PiLSTM** (2025) | Physics-informed LSTM, 29.7% prediction improvement | Motion prediction |
| **TCN vs LSTM** | TCN better for regular trajectories; LSTM for quasi-periodic | Depends on motion type |
| **Local autonomy** | Increase robot autonomy when delay exceeds thresholds | Very high latency |

### 12.2 Key Finding

Delays as small as 0.2s degrade teleoperator performance; at 0.5s, effective control becomes nearly impossible. Haptic feedback benefits diminish at latencies >500ms.

---

## 13. Sim-to-Real Transfer

### 13.1 Simulation Environments

| Environment | Speed | Key Features | Teleoperation Support |
|------------|-------|-------------|----------------------|
| **Robosuite v1.5** (2024) | Good | Humanoid support, composite controllers | SpaceMouse, DualSense, MuJoCo drag |
| **Isaac Lab** (NVIDIA) | ~9,200 samples/sec (4096 envs) | GPU-accelerated, cuRobo planning | SpaceMouse, XR |
| **SAPIEN / ManiSkill3** (RSS 2025) | 30,000+ FPS | GPU-parallelized rendering | AnyTeleop integration |
| **Robomimic** | N/A (framework) | BC, BC-RNN, Diffusion Policy implementations | From Robosuite demos |

### 13.2 Digital Twins

- **RoboTwin** (CVPR 2025): AI-generated 3D models from 2D images; LLMs decompose tasks into subtasks
- **NVIDIA pipeline**: Teleoperation ground-truth + synthetic variations + domain randomization

---

## 14. Key Datasets & Benchmarks

| Dataset | Scale | Robots | Collection Method | Format | License |
|---------|-------|--------|------------------|--------|---------|
| **DROID** | 76K trajectories, 350 hrs | 18 robots, 13 institutions | Quest 2 + Franka | RLDS | CC-BY 4.0 |
| **Open X-Embodiment** | 1M+ trajectories | 22 embodiments, 34 labs | Various | RLDS | Open |
| **RoboTurk** | 137.5 hrs initial | Various | Phone 6-DOF | Standard | Open |
| **LeRobot Hub** | Growing | Multiple | Various | Parquet + MP4 | Open |
| **TeleOpBench** | Benchmark | Multiple modalities | Vision, VR, MoCap, exo | Standard | Open |

### DROID Impact

Policies co-trained with DROID outperform OXE-trained policies by **22% in-distribution** and **17% out-of-distribution**.

---

## 15. Motion Retargeting

### 15.1 Strategies

| Strategy | Description | Pros | Cons |
|----------|-------------|------|------|
| **Joint-space (geometric)** | Map human joint angles directly | Simple, no IK needed | Poor EE tracking when proportions differ |
| **Cartesian-space (IK-based)** | Track end-effector via IK | Intuitive, precise EE control | Singularity susceptible |
| **Optimization-based hybrid** (2025) | Geometric IK + QP in Cartesian + joint constraints | Best of both worlds | Complex |
| **Deep learning retargeting** (2024) | Learn embodiment mappings from data | Handles large DOF differences | Needs training data |
| **GMR** (ICRA 2026) | SMPL-X to multiple robots, real-time on CPU | Most general | Requires SMPL-X input |

### 15.2 GMR (General Motion Retargeting)

- Real-time retargeting from SMPL-X to multiple humanoid robots
- Supports whole-body teleoperation (TWIST framework)
- [GitHub](https://github.com/YanjieZe/GMR)

### 15.3 Singularity Avoidance

- **Stereographic SEW angle** (2024): Reduces algorithmic singularity from a line to a half-line
- **DNN-based singularity-free IK** (2024): Bayesian-optimized networks trained to produce singularity-free solutions
- **Force feedback**: Communicate singularity proximity back to operator through master device

---

## 16. Comparative Analysis

### 16.1 Teleoperation Modality Comparison

| Modality | Accuracy | Latency | Cost | Setup | Learning Curve | Fatigue | Best For |
|----------|----------|---------|------|-------|---------------|---------|----------|
| **Single webcam** | Low-Medium | Low | $20-100 | Minutes | Low | Low | Prototyping, simple tasks |
| **Depth camera** | Medium | Low | $200-700 | 30 min | Low | Low | 3D tasks, grasping |
| **VR (Quest 3)** | Medium-High | ~70ms | $500 | 1 hour | Medium | Medium | 6-DOF manipulation |
| **VR (Vision Pro)** | High | ~128ms | $3,500 | 1 hour | Medium | Medium | Dexterous manipulation |
| **Leader-follower (GELLO)** | High | <5ms | $300 | 30 min | **Lowest** | Low | Data collection, precision |
| **Leader-follower (ALOHA)** | Highest | <5ms | $20K | Days | Low | Low-Medium | Bimanual manipulation |
| **Haptic device** | High | <10ms | $3K-16K | 1 hour | Medium | Medium | Contact-rich tasks |
| **MoCap** | Highest | <5ms | $1K-150K | Hours | Low | Low | Full-body retargeting |

### 16.2 Policy Architecture Comparison

| Policy | Data Efficiency | Multimodal | Speed | Complexity | Best For |
|--------|----------------|-----------|-------|-----------|----------|
| **ACT** | Best (50 demos) | Limited | Fastest | Low | Single-task, bimanual |
| **Diffusion Policy** | Moderate (100+) | Excellent | Slow | Medium | Complex manipulation |
| **3D Diffusion Policy** | Good (10-40) | Excellent | Moderate | Medium | 3D-aware tasks |
| **Pi0** | Fine-tune: 500-2K | Good | 50 Hz | High | Multi-task, generalist |
| **OpenVLA** | Fine-tune: 500-2K | Good | Moderate | High | Language-conditioned |

---

## 17. Recommendations for Our Project

### 17.1 Current Setup Assessment

Our pipeline: **Webcam → MediaPipe Pose → Joint Angles → ROS2 → Robot**

This is a validated, low-cost approach used by multiple open-source projects (FrankaTeleop, teleoperation-robot-arm). The research confirms it's viable for prototyping and simple tasks.

### 17.2 Immediate Improvements (Tier 1: High Impact, Low Effort)

1. **Replace EMA with One Euro Filter** — Eliminates jitter-vs-lag trade-off (already implemented)
2. **Reduce QoS depth from 5 to 1** — Eliminates stale frame latency (already implemented)
3. **Add velocity clamping** — Prevents dangerous tracking glitch spikes (already implemented)
4. **Add tracking recovery manager** — Graceful degradation on tracking loss (already implemented)
5. **Use relative motion mapping** — Avoid absolute depth estimation; map deltas

### 17.3 Medium-Term Upgrades (Tier 2)

6. **Upgrade to RTMPose** — Better accuracy than MediaPipe, still real-time, fine-tunable via MMPose
7. **Add depth camera** — Intel RealSense D435 or Orbbec Femto for resolving depth ambiguity (~$300)
8. **Use Cartesian space mapping with IK** — More intuitive for operators per ICRA 2021 study
9. **Integrate MoveIt Servo** — Built-in collision checking, joint limits, velocity scaling

### 17.4 Long-Term Path (Tier 3)

10. **Build a GELLO leader arm** — $300, 30 min assembly, dramatically better data quality
11. **Collect demonstrations for ACT** — 50 demos per task, train in 5 hours on single GPU
12. **Integrate with LeRobot** — Standardized format, pretrained policies, growing community
13. **Explore Pi0 fine-tuning** — Open-source foundation model, multi-task generalization

### 17.5 Decision Tree

```
Do you need fine dexterous manipulation?
├── Yes → GELLO/ALOHA hardware + ACT/Diffusion Policy
└── No → Single camera is viable
    │
    ├── Is depth accuracy critical?
    │   ├── Yes → Add depth camera (RealSense/Orbbec)
    │   └── No → Relative motion mapping suffices
    │
    ├── Need to collect training data for IL?
    │   ├── Yes → Upgrade to GELLO ($300) or UMI ($200)
    │   └── No → Continue with webcam teleoperation
    │
    └── Need multi-task generalization?
        ├── Yes → Pi0 or OpenVLA fine-tuning
        └── No → ACT with 50 task-specific demos
```

---

## 18. References

### Pose Estimation
- [ViTPose GitHub](https://github.com/ViTAE-Transformer/ViTPose) — ViTPose/ViTPose++
- [RTMPose Paper](https://arxiv.org/abs/2303.07399) — Real-time multi-person pose
- [RTMO (CVPR 2024)](https://openaccess.thecvf.com/content/CVPR2024/papers/Lu_RTMO_Towards_High-Performance_One-Stage_Real-Time_Multi-Person_Pose_Estimation_CVPR_2024_paper.pdf)
- [MMPose GitHub](https://github.com/open-mmlab/mmpose) — Comprehensive pose toolbox
- [PoseMamba](https://arxiv.org/html/2408.03540v1) — State-space model for 3D pose
- [Papers With Code COCO Leaderboard](https://paperswithcode.com/sota/pose-estimation-on-coco-test-dev)
- [Monocular 3D HPE Survey (2025)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12031093/)

### Hand Pose
- [WiLoR (CVPR 2025)](https://github.com/rolpotamias/WiLoR) — End-to-end hand mesh recovery
- [HaMeR (CVPR 2024)](https://github.com/geopavlakos/hamer) — Hand mesh recovery
- [FrankMocap](https://github.com/facebookresearch/frankmocap) — Modular body+hand+face
- [Hand4Whole](https://github.com/mks0601/Hand4Whole_RELEASE)

### Body Motion Capture
- [WHAM (CVPR 2024)](https://wham.is.tue.mpg.de/) — World-grounded human motion
- [GVHMR (SIGGRAPH Asia 2024)](https://dl.acm.org/doi/10.1145/3680528.3687565)
- [TRAM (ECCV 2024)](https://github.com/yufu-wang/tram) — Global trajectory recovery
- [TokenHMR (CVPR 2024)](https://github.com/saidwivedi/TokenHMR)
- [SMPL-X](https://github.com/vchoutas/smplx) — Body model

### Teleoperation Systems
- [Bunny-VisionPro](https://github.com/Dingry/BunnyVisionPro) — Vision Pro teleoperation
- [Open-TeleVision](https://github.com/OpenTeleVision/TeleVision) — Immersive stereoscopic
- [OPEN TEACH](https://github.com/aadhithya14/Open-Teach) — Quest 3 teleoperation
- [TeleMoMa](https://github.com/UT-Austin-RobIn/telemoma) — Multi-modal mobile manipulation
- [AnyTeleop](https://yzqin.github.io/anyteleop/) — Vision-based unified teleoperation
- [FrankaTeleop](https://github.com/gjcliff/FrankaTeleop) — MediaPipe + ROS2 + MoveIt Servo
- [VisionProTeleop](https://github.com/Improbable-AI/VisionProTeleop) — MIT Vision Pro bridge

### Hardware
- [GELLO Software](https://github.com/wuphilipp/gello_software) / [Hardware](https://github.com/wuphilipp/gello_mechanical) — $300 leader arm
- [UMI](https://github.com/real-stanford/universal_manipulation_interface) — Robot-free data collection
- [ALOHA 2](https://aloha-2.github.io/) / [GitHub](https://github.com/tonyzhaozh/aloha) — Bimanual teleoperation
- [Mobile ALOHA](https://github.com/MarkFzp/mobile-aloha) — Mobile bimanual
- [U-ARM](https://arxiv.org/html/2509.02437) — Ultra low-cost $60 leader arm
- [DOGlove](https://arxiv.org/html/2502.07730v1) — Open-source haptic glove <$600
- [LeRobot](https://github.com/huggingface/lerobot) — Unifying robot learning platform

### Imitation Learning & Foundation Models
- [ACT Paper](https://arxiv.org/abs/2304.13705) — Action Chunking with Transformers
- [Diffusion Policy](https://dl.acm.org/doi/10.1177/02783649241273668) — IJRR 2024
- [Pi0 Paper](https://arxiv.org/abs/2410.24164) / [GitHub](https://github.com/Physical-Intelligence/openpi)
- [OpenVLA](https://arxiv.org/abs/2406.09246) — Open-source 7B VLA
- [Octo](https://octo-models.github.io/) — Lightweight generalist policy
- [RT-2](https://robotics-transformer2.github.io/) — Vision-Language-Action model

### Datasets & Benchmarks
- [DROID](https://droid-dataset.github.io/) — 76K trajectories, 350 hours
- [Open X-Embodiment](https://robotics-transformer-x.github.io/) — 1M+ trajectories, 22 robots
- [RoboTurk](https://roboturk.stanford.edu/) — Crowdsourced teleoperation
- [Robosuite](https://robosuite.ai/) / [Robomimic](https://robomimic.github.io/) — Sim benchmarks
- [TeleOpBench](https://arxiv.org/abs/2505.12748) — Modality comparison benchmark

### Retargeting & IK
- [GMR (ICRA 2026)](https://github.com/YanjieZe/GMR) — General Motion Retargeting
- [Joint vs Task Space Mapping (ICRA 2021)](https://arxiv.org/abs/2011.02508)
- [Stereographic SEW Angle (2024)](https://www.sciencedirect.com/science/article/pii/S0094114X24002519)
- [MoReNet (ICIRA 2024)](https://link.springer.com/chapter/10.1007/978-981-96-0777-8_13)

### Surveys
- [Diffusion Models for Manipulation (Frontiers 2025)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1606247/full)
- [Interactive IL Survey (Frontiers 2025)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1682437/full)
- [Foundation Models for Manipulation (SAGE/IJRR 2025)](https://journals.sagepub.com/doi/abs/10.1177/02783649251390579)
- [VLA Models Review (2025)](https://arxiv.org/html/2510.07077v1)
- [Bilateral Teleoperation Survey (2025)](https://www.sciencedirect.com/org/science/article/pii/S1526149225000049)
- [Network Latency in Teleoperation (Sensors 2024)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11207977/)
