# Research Report: IMU-Based Teleoperation for SteveROS

**Date:** 2026-03-05
**Context:** SteveROS KBot humanoid (20-DOF, 5-DOF per arm, Robstride motors, ROS 2)
**Current System:** MediaPipe BlazePose → Pose-to-Joints IK Bridge → JointTrajectoryController
**Problem:** Noisy, jittery teleoperation with frequent tracking errors

---

## Table of Contents

1. [Current System Analysis](#1-current-system-analysis)
2. [Why MediaPipe Is Noisy](#2-why-mediapipe-is-noisy)
3. [IMU-Based Teleoperation: Literature Review](#3-imu-based-teleoperation-literature-review)
4. [IMU vs MediaPipe: Head-to-Head Comparison](#4-imu-vs-mediapipe-head-to-head-comparison)
5. [Sensor Fusion: IMU + Vision](#5-sensor-fusion-imu--vision)
6. [ML Models for IMU Pose Estimation](#6-ml-models-for-imu-pose-estimation)
7. [Alternative Teleoperation Methods](#7-alternative-teleoperation-methods)
8. [Noise Filtering Techniques](#8-noise-filtering-techniques)
9. [Cost-Benefit Analysis](#9-cost-benefit-analysis)
10. [Recommendations](#10-recommendations)
11. [Cross-Modal Distillation: Train with IMUs, Deploy Vision-Only](#11-cross-modal-distillation-train-with-imus-deploy-vision-only)
12. [References](#12-references)

---

## 1. Current System Analysis

### Architecture

```
USB Camera (640x480, 30fps)
  → v4l2_camera node
  → MediaPipe BlazePose (33 landmarks, metric world coordinates)
  → Pose-to-Joints Bridge (torso frame IK, EMA filter α=0.3)
  → JointTrajectoryController (0.1s duration per command)
  → Robstride CAN-bus motors
```

### What Works Well

- **Metric world coordinates**: BlazePose outputs hip-centered 3D coordinates in meters, eliminating manual depth estimation
- **Torso frame construction**: Orthonormal frame from shoulders/hips decouples arm IK from camera orientation
- **Modular design**: Pose detection, IK, and control are separate ROS 2 nodes — any can be replaced independently
- **Zero-cost input**: Only requires a USB webcam

### Current Weaknesses

| Issue | Impact | Root Cause |
|-------|--------|------------|
| **High jitter** | Robot arms oscillate/shake | MediaPipe per-frame variance, single-sample EMA insufficient |
| **No wrist rotation** | Wrist hardcoded to 0.0 | BlazePose provides position only, not orientation |
| **Yaw singularities** | Yaw jumps to 0 when elbow extends | Degenerate frame when forearm aligns with upper arm |
| **~150-200ms latency** | Noticeable delay in tracking | Dominated by MediaPipe inference (~50-100ms) |
| **No per-landmark confidence** | Bad frames not rejected | Bridge processes all landmarks equally regardless of quality |
| **Hard joint clamping** | Jerky motion at limits | `np.clip()` with no smooth saturation |
| **Camera FOV constraint** | Loses tracking when hands leave frame | Inherent to vision-only systems |

### Latency Budget

| Component | Latency |
|-----------|---------|
| Camera capture | ~33ms |
| MediaPipe inference | 50-100ms |
| ROS 2 messaging | ~1-2ms |
| IK computation | ~1-5ms |
| EMA filter | <1ms |
| Trajectory controller | 1-2ms |
| Motor CAN response | 5-10ms |
| **Total** | **~100-200ms** |

---

## 2. Why MediaPipe Is Noisy

MediaPipe BlazePose processes each frame independently with minimal temporal consistency:

1. **Per-frame re-estimation**: Each frame is treated semi-independently. The tracking confidence threshold helps, but landmark positions can still jump 5-10mm between consecutive frames
2. **Monocular depth ambiguity**: 3D pose from a single RGB camera is ill-posed — small appearance changes cause disproportionate depth estimate shifts
3. **Lighting/background sensitivity**: Performance degrades with poor lighting, busy backgrounds, or when the subject wears baggy clothing
4. **Occlusion**: When one arm crosses in front of the body or leaves the frame, landmarks either disappear or snap to incorrect positions
5. **Fast motion blur**: At 30fps, rapid arm movements cause motion blur that degrades landmark detection
6. **No process model**: Unlike a Kalman filter, MediaPipe has no dynamics model — it cannot predict where a joint should be based on velocity/acceleration

The current EMA filter (α=0.3) provides first-order smoothing but is insufficient:
- It's a single-pole IIR filter with exponential decay response
- No frequency-domain design (cutoff frequency not specified)
- Fixed smoothing regardless of motion speed (smooths equally during fast and slow movements)
- Only 1-frame memory — no multi-sample averaging

---

## 3. IMU-Based Teleoperation: Literature Review

### State of the Art

IMU-based teleoperation has matured significantly, with systems ranging from $50 DIY prototypes to industrial deployments:

**Relative Orientation Vector (ROV) method (Kiarostami et al., 2024)**: Lightweight controller using 3 IMUs + 10 potentiometers, capturing 17 DOF of arm-hand motion. Computes joint angles directly from quaternion data, avoiding Euler angle singularities. Notable for simplicity and expandability.

**Wearable IMU for collaborative robots (Kruger et al., 2021)**: Wireless system using only IMUs (no gloves, markers, or constraining wearables) to determine upper-body orientation for UR-type robot teleoperation. Demonstrated practical untethered operation.

**Open-source 7-DOF arm tracking (2020)**: Two IMUs + one potentiometer per arm, built with off-the-shelf components and 3D printing for UR5 teleoperation. Total cost under $200.

**Vision + IMU multimodal system (Li et al., 2020)**: Combined a vision-based hand pose regression network (TransTeleop) with IMU-based arm tracking, enabling unlimited workspace operation.

**TeLeMan (2022)**: Teleoperation of a legged manipulator combining an IMU motion capture suit with VR headset for explosive ordnance disposal. Demonstrated whole-body loco-manipulation.

### IMU Accuracy Characteristics

| Metric | IMU-Based | MediaPipe (Vision) |
|--------|-----------|-------------------|
| Joint angle RMSE (controlled) | 1-3° | 5-10° |
| Position estimation error | ~35mm (vs. Vicon) | Higher, varies with distance |
| Trunk rotation accuracy | 92.4% | Not specifically reported |
| Best-case angular error | 1.09° (rotation) | 5.72° (knee, post-processed) |
| Update rate | 100-4000 Hz | 6-60 Hz |
| Sensor-to-data latency | 0.25-10ms | 15-164ms |

### IMU Limitations

- **Drift**: Gyroscope integration accumulates error over time. Consumer IMUs require recalibration every 5-10 minutes; high-end systems (Xsens) manage this better with advanced sensor fusion
- **Magnetic interference**: Magnetometer-based heading correction is affected by nearby electronics, motors, and metal structures (very relevant for a robot with CAN-bus motors)
- **Requires wearing sensors**: Must be attached to body segments — less convenient than camera-only
- **No absolute position**: IMUs measure orientation and acceleration, not absolute position. Position requires double integration of acceleration (extremely noisy)
- **Calibration**: Initial alignment to body segment axes required at startup

---

## 4. IMU vs MediaPipe: Head-to-Head Comparison

| Factor | IMU | MediaPipe | Winner |
|--------|-----|-----------|--------|
| **Angular accuracy** | 1-3° RMSE | 5-10° RMSE | IMU |
| **Update rate** | 100-4000 Hz | 6-60 Hz | IMU |
| **Latency** | 0.25-10ms | 15-164ms | IMU |
| **Jitter** | Very low | Moderate-high | IMU |
| **Occlusion handling** | Immune | Fails | IMU |
| **Fast motion** | Excellent | Poor (blur) | IMU |
| **Absolute position** | Poor (drift) | Fair | MediaPipe |
| **Long-term stability** | Drift accumulates | No drift | MediaPipe |
| **Setup complexity** | Attach sensors, calibrate | Point camera | MediaPipe |
| **Cost** | $50-500+ | $0 (webcam) | MediaPipe |
| **Wearability** | Must wear sensors | Nothing worn | MediaPipe |
| **Lighting dependency** | None | Requires good lighting | IMU |
| **Workspace** | Unlimited | Camera FOV only | IMU |
| **Wrist orientation** | Directly measured | Not available | IMU |

**Verdict**: IMUs are superior for teleoperation accuracy, latency, and robustness. MediaPipe wins on convenience and cost. For a serious teleoperation system, IMUs or sensor fusion are strongly preferred.

---

## 5. Sensor Fusion: IMU + Vision

The consensus in recent literature is that **neither IMU nor vision alone is optimal** — fusion yields the best results.

### Classical Approaches

**Extended Kalman Filter (EKF)**: Most established approach. Al-Mutib et al. (2017) demonstrated EKF fusion of 6-DOF IMU with monocular camera, showing the fused system outperforms either sensor alone. Ligorio & Sabatini (2013) achieved 1° orientation RMSE and 3.5mm position RMSE with EKF fusion, vs 1.6° for IMU-only.

Two coupling paradigms:
- **Loosely coupled**: Vision and IMU pipelines run independently, exchanging estimates. Simpler, more modular.
- **Tightly coupled**: Raw measurements jointly processed in a single estimator. More accurate, more complex.

**Complementary filter**: Simpler alternative exploiting IMU gyroscopes (accurate short-term, high frequency) and vision/accelerometers (accurate long-term, low frequency). The Madgwick filter is a popular variant with open-source implementations.

### Deep Learning Fusion

**3D pose from single image + IMU (2023)**: Transformer-based architecture fusing temporal IMU features with image spatial features, achieving 46mm MPJPE on TotalCapture at 74 FPS on iPhone 12.

**Multimodal gesture recognition (2025)**: Late fusion using log-likelihood ratio (LLR) combining IMU data with capacitive sensing and RGB video for teleoperation.

### Practical Fusion Architecture for SteveROS

```
IMU sensors (200+ Hz) → Quaternion orientation per body segment
                                    ↓
                            Complementary Filter / EKF
                                    ↑
MediaPipe BlazePose (30 Hz) → Absolute position reference (drift correction)
                                    ↓
                            Fused 3D Pose (high-rate, drift-free)
                                    ↓
                            Pose-to-Joints IK Bridge
                                    ↓
                            JointTrajectoryController
```

**Key insight**: Use IMUs as the primary high-rate orientation source and MediaPipe as a low-frequency drift-correction reference. This gives IMU temporal resolution with vision's absolute positioning.

---

## 6. ML Models for IMU Pose Estimation

### Evolution of Deep Learning for Sparse IMU Tracking

| Model | Year | Venue | IMUs | Key Innovation | Latency |
|-------|------|-------|------|---------------|---------|
| Deep Inertial Poser (DIP) | 2018 | SIGGRAPH Asia | 6 | First DL sparse IMU pose | — |
| TransPose | 2021 | ACM TOG | 6 | Added global translation | — |
| Physical Inertial Poser (PIP) | 2022 | CVPR | 6 | Physics-aware constraints | — |
| Transformer Inertial Poser | 2022 | SIGGRAPH Asia | 6 | Terrain generation, SBPs | — |
| IMUPoser | 2023 | CHI | 1-3 (consumer) | Variable sparse input | — |
| Fast Inertial Poser | 2024 | Nature Comms | 6 | Embedded deployment | 15ms @ 65 FPS |
| IMUOptimize | 2024 | arXiv | Variable | Optimal placement search | — |
| MobilePoser | 2024 | UIST | 1-3 (consumer) | On-device + physics | Real-time |
| NeurIPS 2024 | 2024 | NeurIPS | 6 | Anti-jitter Transformer | — |

### Key Insights

1. **6 IMUs is the sweet spot**: Placed on wrists, lower legs, head, and pelvis — this provides enough constraints for full-body pose with redundancy
2. **For arm-only tracking, 2-3 IMUs suffice**: Upper arm + forearm (+ optional hand)
3. **Physics-aware models (PIP) prevent impossible poses**: Floor penetration, bone length violation, etc.
4. **Transformer architectures need anti-jitter modifications**: Naive self-attention produces frame-to-frame oscillation
5. **Training on AMASS dataset with SMPL body model** is the standard pipeline
6. **Fine-tuning on task-specific data** significantly improves real-world performance

### Relevance to SteveROS

Training a custom model for KBot arm teleoperation is feasible but potentially unnecessary for the 5-DOF arm case:
- With 2 IMUs per arm (upper arm + forearm), direct quaternion → joint angle computation (ROV method) may be sufficient
- ML models become valuable when: (a) using fewer sensors than DOF, (b) wanting to infer wrist orientation, or (c) fusing IMU + vision
- If you go the ML route, start with DIP/TransPose codebases and fine-tune

---

## 7. Alternative Teleoperation Methods

### GELLO Leader Arm ($300/arm)

Joint-matched kinematic replica of the target arm using Dynamixel servos and 3D-printed parts. **Already in SteveROS roadmap (Step 15).**

- Joint-to-joint mapping — no IK, no estimation, no noise from sensors
- User physically feels singularities and joint limits
- User study results: outperforms VR controllers and SpaceMouse for demo quality
- Requires custom mechanical design to match KBot arm kinematics
- **Verdict**: Best approach for imitation learning data collection

### Meta Quest 3 + OPEN TEACH ($500)

VR controller-based teleoperation at up to 90Hz with millimeter-level accuracy.

- 6-DOF pose at 72-90Hz, sub-degree rotational accuracy
- Immersive mixed reality with robot camera view
- Supports multiple robot platforms (Franka, xArm, Jaco)
- Main noise source: IK retargeting and workspace mismatch
- **Verdict**: Strong for quick prototyping and intuitive control

### Learning from Demonstration (LfD)

These don't replace teleoperation — they **consume** teleoperation data and handle noise at the policy level:

**ACT (Action Chunking with Transformers)**: Predicts action sequences ("chunks"), inherently smoothing noisy demonstrations. Trained as CVAE — latent variable captures and marginalizes demonstration noise. 80-96% success with only 50-100 demos (10-20 min of data).

**Diffusion Policy**: Starts from Gaussian noise and iteratively denoises into action sequences. The denoising process itself is a form of noise rejection. 46.9% average improvement over prior SOTA across 12 benchmarks. Key finding: haptic feedback during data collection was critical (0/10 success without vs 10/10 with).

**Key insight**: Cleaner demonstrations produce better policies with less data. Invest in demo quality first (GELLO), then scale quantity.

### UMI (Universal Manipulation Interface, $100-200)

Handheld gripper with GoPro — no robot needed during data collection. Demonstrations collected "in the wild" and transferred zero-shot to any platform. Interesting for manipulation research but less suited to humanoid arm control.

### Apple Vision Pro ($3,500)

Premium hand tracking via custom silicon. Projects like Bunny-VisionPro achieve bimanual dexterous teleoperation with haptic feedback. Overkill for current KBot capabilities (no dexterous hands yet).

### DOGlove ($600)

21-DOF hand motion capture with 5-DOF force feedback. Best open-source option for dexterous manipulation. Relevant when KBot gains finger dexterity, not now.

---

## 8. Noise Filtering Techniques

### Comparison (ranked by practicality for SteveROS)

**1. One Euro Filter — RECOMMENDED FIRST CHOICE**

Adaptive low-pass filter that provides strong smoothing during slow movement (killing jitter) and minimal smoothing during fast movement (reducing lag). Key insight: at low speeds, humans are sensitive to jitter not lag; at high speeds, sensitive to lag not jitter.

- ~20 lines of code to implement
- Two parameters: `min_cutoff` (smoothness) and `beta` (speed responsiveness)
- No system model needed
- Specifically designed for noisy interactive input
- **Can be added to the existing pipeline in an afternoon**

**2. EMA (current, α=0.3)**

Simple but inferior to One Euro due to fixed smoothing-lag tradeoff. Could serve as fallback.

**3. Kalman Filter**

Optimal for linear systems with Gaussian noise when dynamics are known. Excels at multi-sensor fusion (e.g., IMU + MediaPipe). Extended Kalman Filter handles nonlinear systems. More complex to tune than One Euro.

**4. Kalman Smoothing with Constraints**

Adds soft inequality constraints (acceleration/jerk limits). Prevents jerky motion that could damage hardware. Best for scenarios where motor safety matters.

**5. Moving Average**

Requires window buffer, fixed lag proportional to window size. Inferior to EMA for real-time use.

**6. N-euro Predictor (LSTM-based)**

Learned filter that predicts future values, achieving 36% reduction in prediction error. Requires domain-specific training. Overkill for most teleoperation.

---

## 9. Cost-Benefit Analysis

| Approach | Cost | Accuracy | Latency | Noise | Setup | Best For |
|----------|------|----------|---------|-------|-------|----------|
| **One Euro filter on MediaPipe** | $0 | Fair | 30-100ms | Medium (improved) | Minutes | Immediate improvement |
| **ESP32 + BNO055 IMUs (2-3/arm)** | $50-150 | Good | 2-10ms | Low | Days | DIY IMU teleoperation |
| **IMU + MediaPipe fusion** | $50-150 | Very Good | 10-30ms | Low | 1-2 weeks | Best of both worlds |
| **GELLO leader arm** | ~$300/arm | Excellent | <5ms | Very Low | 1-2 weeks | Imitation learning data |
| **Meta Quest 3 + OPEN TEACH** | ~$500 | Good | 20-50ms | Low-Med | Hours | Quick prototyping |
| **Rokoko suit** | $5K-$13K | Good | 10-20ms | Low-Med | Hours | Full-body teleoperation |
| **Xsens Humanoid** | $15K-$30K+ | Excellent | <10ms | Very Low | Days | Professional humanoid |
| **Apple Vision Pro** | $3,500 | Very Good | 15-30ms | Low | Hours | Premium VR teleoperation |

---

## 10. Recommendations

### The noise problem can be attacked at three levels:

```
Level 1: Signal Filtering    → Cheap, immediate, handles the symptom
Level 2: Better Input Hardware → Eliminates noise at the source
Level 3: Noise-Robust Learning → Absorbs remaining noise during policy training
```

### Phased Implementation Plan

#### Phase 1: Immediate (0 cost, 1 afternoon)

**Add a One Euro filter** to replace the EMA in `pose_to_joints.py`. This is the single highest-impact, lowest-effort change:

```python
class OneEuroFilter:
    def __init__(self, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    def __call__(self, x, t):
        if self.t_prev is None:
            self.x_prev = x
            self.t_prev = t
            return x
        dt = t - self.t_prev
        if dt <= 0:
            return self.x_prev
        # Derivative estimation
        a_d = self._alpha(dt, self.d_cutoff)
        dx = (x - self.x_prev) / dt
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        # Adaptive cutoff
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(dt, cutoff)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat

    def _alpha(self, dt, cutoff):
        tau = 1.0 / (2 * 3.14159 * cutoff)
        return 1.0 / (1.0 + tau / dt)
```

Also add per-landmark confidence checking — skip frames where key landmarks (shoulders, hips, elbows, wrists) fall below a confidence threshold.

#### Phase 2: Near-term (cost: $50-150, timeline: 1-2 weeks)

**Build a DIY IMU system** for arm tracking:
- 2x ESP32 + BNO055 per arm (upper arm + forearm)
- BNO055 provides onboard sensor fusion and quaternion output
- Use ROV method (Kiarostami et al., 2024) for direct quaternion → joint angle computation
- Publish to same JointTrajectory topics (drop-in replacement for pose_to_joints)
- **Bonus**: IMU on hand gives wrist orientation — solving the wrist=0 problem

Optional: Keep MediaPipe running as a low-frequency drift correction reference via complementary filter.

#### Phase 3: Medium-term (cost: ~$300, timeline: 2-4 weeks)

**Build GELLO leader arm** (already in roadmap Step 15):
- Custom kinematic replica matched to KBot arm geometry
- Dynamixel servos + 3D-printed links
- Joint-to-joint mapping eliminates all estimation noise
- Best for collecting imitation learning demonstrations

#### Phase 4: Policy Learning (no additional hardware)

**Use ACT or Diffusion Policy** via the LeRobot framework:
- ACT: Simpler, faster training, good with small datasets (50-100 demos)
- Diffusion Policy: Better for multimodal tasks, more robust to noisy demos
- Both algorithms absorb residual teleoperation noise during training

### What NOT to Do

- **Don't buy Xsens/Rokoko/OptiTrack** — overkill for a single-robot project
- **Don't build custom flex sensor gloves** — KBot has no dexterous hands yet
- **Don't train a full-body ML pose model from scratch** — unnecessary for 5-DOF arm tracking; direct quaternion computation is sufficient
- **Don't use Apple Vision Pro** — premium cost for capabilities Quest 3 provides at 14% of the price

### Should You Use IMUs? Yes, But...

**Yes, IMUs will significantly improve teleoperation quality** over raw MediaPipe:
- 3-10x better angular accuracy (1-3° vs 5-10°)
- 10-100x higher update rate (100-400Hz vs 30Hz)
- 10-100x lower latency (1-10ms vs 50-100ms)
- Immune to occlusion and lighting issues
- Can measure wrist orientation (solving the wrist=0 problem)

**But start with the One Euro filter first** — it's free, takes an afternoon, and may provide enough improvement to defer hardware changes. If it's still too noisy after that, build the IMU system.

**And consider GELLO as the ultimate solution** — if the goal is collecting demonstrations for imitation learning, a joint-matched leader arm eliminates the entire noise problem by design. No filtering, no estimation, no sensors to drift.

---

## 11. Cross-Modal Distillation: Train with IMUs, Deploy Vision-Only

### 11.1 Core Concept

**Can you train a computer vision model using IMU data as ground truth during training, then remove the IMUs at deployment?**

**Yes — this is a well-established paradigm** known by several names:
- **Learning Using Privileged Information (LUPI)** — Vapnik & Vashist (2009)
- **Cross-modal knowledge distillation** — teacher (IMU) → student (vision)
- **Sensor-to-vision transfer** — train with rich sensors, deploy with cameras only

The idea: IMUs provide high-accuracy orientation data (1-3° error) during a training phase. A vision model learns to replicate these measurements from camera images alone. After training, the IMUs are removed entirely.

### 11.2 Evidence This Works: Successful Deployments

This paradigm has been validated across multiple domains, from research to production:

| Application | Teacher (Training) | Student (Deployment) | Maturity |
|-------------|-------------------|---------------------|----------|
| Autonomous driving | LiDAR + Radar | Cameras only | **Production** (Tesla removed radar/ultrasonics in 2022-23) |
| Monocular depth estimation | Depth sensors / LiDAR | Single RGB camera | **Production** (Depth Anything V2, many products) |
| Human pose estimation | MoCap systems | Single RGB camera | **Production** (4DHumans, HMR, SPIN) |
| Robot locomotion | Privileged sim state | Proprioception only | Deployed on real robots (ANYmal, MIT Mini Cheetah) |
| Manipulation | Force/tactile + vision | Vision only | Active research (70-85% of sensor accuracy) |
| **IMU → Vision teleoperation** | **IMU + cameras** | **Cameras only** | **All components exist; integration is engineering** |

### 11.3 How It Works for Human Pose Estimation (Most Relevant)

The MoCap-to-vision pipeline is the dominant paradigm in the field. Key systems:

**HMR (CVPR 2018)** — Kanazawa et al. Trains a CNN to predict SMPL body model parameters from a single RGB image. MoCap data provides ground truth + a discriminator ensures predicted poses are realistic. Vision-only at deployment.

**SPIN (ICCV 2019)** — Kolotouros et al. Combines regression with optimization (SMPLify) in a training loop. MoCap provides ground truth SMPL parameters. At deployment, runs a single forward pass (no optimization needed).

**4DHumans (CVPR 2024)** — Goel et al. State-of-the-art vision-only human pose and shape estimation. Transformer architecture, trained on MoCap ground truth from AMASS dataset. Real-time with single RGB camera.

**MotionBERT (ICCV 2023)** — Zhu et al. Pretrained motion representation model on large-scale MoCap data. Fine-tuned for vision-based pose tasks. Demonstrates that pretraining on body sensor data transfers effectively to vision-only deployment.

**WHAM (CVPR 2024)** — Shin et al. Uses motion features from MoCap datasets to improve world-coordinate human motion estimation from video. Handles global translation/rotation that pure vision struggles with.

**IMUs can replace MoCap as the ground truth source** — they're cheaper, portable, and don't require a lab. The HPS system (Guzov et al., CVPR 2021) explicitly used IMU suits to create training data for vision-only pose estimation.

### 11.4 Cross-Modal Distillation Methods

#### Direct IMU-Vision Distillation Papers

**EgoDistill (NeurIPS 2023)** — Tan, Nagarajan & Grauman (UT Austin). **The closest direct match.** During training, a lightweight model takes sparse video frames + IMU to approximate features from a heavy video model. At inference, the lightweight model replaces the expensive video model, achieving **200x fewer GFLOPs**. Key finding: IMU coupled with a single image provides **better cross-modality distillation** than images alone or images with audio. Uses GoPro IMU (BOSCH BMI260, ~$3).

**COMODO (2025)** — Chen, Wongso et al. The reverse direction: distills from video (teacher) to IMU (student) for activity recognition. Demonstrates the **bidirectionality** of the paradigm — knowledge transfers both ways between IMU and vision modalities.

**IMU2CLIP (Meta/FAIR, 2022)** — Moon et al. Aligns IMU representations with CLIP's video/text embeddings using contrastive learning. After training, IMU embeddings are interchangeable with video embeddings.

#### IMU as Ground Truth for Vision Training

**3DPW Dataset (ECCV 2018)** — von Marcard et al. **Foundational for this exact approach.** Fuses 6 IMUs with a moving camera to produce the first in-the-wild dataset with accurate 3D SMPL pose annotations (26mm average error). This dataset is now a standard benchmark used to train and evaluate monocular vision-based pose estimators — an indirect but large-scale validation of "training with IMU ground truth, deploying with vision only."

**Depth as Privileged Information for 3D HPE (2024)** — Simoni et al. Uses depth exclusively during training to boost RGB-only 3D pose estimation. Shows privileged information **significantly enhances performance even on limited, small datasets**. General across any 3D HPE method.

#### Autonomous Driving Scale Validation

**BEVDistill (ICLR 2023)** — Chen et al. Camera student closes a **15% mAP gap** to within 2-5% of the LiDAR teacher. Framework directly applicable to IMU→vision.

**MonoDistill (ICLR 2022)** — Chong et al. Uses LiDAR point clouds during training to teach monocular 3D estimation. LiDAR is privileged information — removed at inference.

#### The Sim-to-Real Analogy (Same Paradigm)

The most successful robotics examples use the same principle with simulated privileged information:

**Learning Quadrupedal Locomotion (Science Robotics, 2020)** — Lee et al. (ETH Zurich). **Teacher** trained in simulation with privileged terrain info (height map, friction). **Student** trained via behavior cloning using only onboard sensors (joint encoders + IMU). Student achieves near-teacher performance on real-world rough terrain, stairs, and obstacles. Canonical example of privileged learning in robotics.

**Rapid Motor Adaptation / RMA (RSS 2021)** — Kumar et al. (UC Berkeley). Teacher has privileged access to physics parameters (friction, mass). Student learns an adaptation module that infers these from proprioceptive history alone. No privileged info needed at deployment.

**Extreme Parkour (2024)** — Zhang et al. (CMU). Teacher trained with privileged height-map + contact info. Student uses only proprioception + depth camera. Performs extreme parkour without privileged information.

### 11.5 UMI: A Real System Already Doing This for Teleoperation

**Universal Manipulation Interface (UMI, RSS 2024)** — Cheng Chi et al. (Stanford). **The most directly relevant system.** UMI uses GoPro cameras with built-in IMUs during **data collection** to achieve precise 6DoF tracking via inertial-monocular SLAM (based on ORB-SLAM3). The IMU provides metric scale and handles motion blur. At **deployment**, the learned diffusion policy uses only wrist-mounted camera images — **no IMU required**.

This is precisely the "train with IMU, deploy vision-only" paradigm applied to teleoperation, and it works in production-quality research demos.

- Project: https://umi-gripper.github.io/
- Code: https://github.com/real-stanford/universal_manipulation_interface

### 11.6 Implementation Blueprint for Teleoperation

#### Phase 1: Data Collection (IMU + Camera)
- Operator wears 2-4 IMUs (ESP32 + BNO055, ~$100 total) on upper arm, forearm, torso
- Camera captures operator at 30-60 FPS
- IMUs provide ground truth joint angles/orientations at 200+ Hz
- Collect **diverse** movements: reaching, pointing, manipulation gestures
- **Minimum**: 10-20 hours paired data; **ideal**: 50+ hours with varied operators

#### Phase 2: Training
```
L_total = L_joint_MSE + λ₁ * L_bone_length + λ₂ * L_temporal_smoothness + λ₃ * L_adversarial
```

| Loss Component | Purpose |
|---------------|---------|
| `L_joint_MSE` | Direct supervision from IMU joint angles |
| `L_bone_length` | Physics constraint — bones don't change length |
| `L_temporal_smoothness` | Prevents jittery predictions |
| `L_adversarial` | Ensures physically plausible poses (HMR-style discriminator) |

**Recommended architecture**: Fine-tune a pretrained vision backbone (ViTPose, DINOv2, or HRNet) with a regression head. Pretrained models reduce paired data requirements by 5-10x.

#### Phase 3: Deployment
- Remove all IMUs
- Camera-only system estimates operator pose
- Same pose → robot joint mapping as IMU pipeline

#### Phase 4: Periodic Recalibration (Optional)
- Every few weeks, operator wears IMUs for a 10-minute session
- Collect fresh paired data to correct for environmental drift
- Fine-tune vision model on new data
- This corrects for lighting changes, camera drift, appearance changes

### 11.7 Expected Performance

| Metric | IMU-Only | Vision After Distillation | Gap |
|--------|----------|--------------------------|-----|
| Joint angle accuracy | 1-3° RMSE | 5-10° RMSE | ~3-5x |
| Latency | 5-15 ms | 30-50 ms | +20-40 ms |
| Occlusion robustness | Immune | Vulnerable | Significant |
| Drift | Accumulates over time | None | Vision advantage |
| Setup burden | Must wear sensors | Nothing worn | Major advantage |
| Workspace | Unlimited | Camera FOV | Trade-off |

**For gross arm movements in teleoperation (reaching, pointing, grasping postures): the 5-10° accuracy gap is typically acceptable.**

### 11.8 Key Challenges and Mitigations

| Challenge | Mitigation |
|-----------|-----------|
| Self-occlusion (arm behind body) | Multi-camera setup; temporal models that hallucinate through brief occlusions |
| Lighting sensitivity | Data augmentation (random brightness, contrast); foundation model backbones with lighting robustness |
| Fast motion blur | Higher camera FPS (60+); temporal smoothing; hybrid IMU fallback |
| Generalization to new operators | Train with multiple operators; clothing/appearance augmentation; domain adversarial training |
| Accuracy gap vs. IMU teacher | Acceptable for teleoperation; can use hybrid mode (vision + occasional IMU) for critical tasks |

### 11.9 Loss Functions: What Works for Cross-Modal Distillation

Naive loss functions often fail across modalities. Key findings from recent literature:

| Loss | When to Use | Caveat |
|------|-------------|--------|
| **MSE / L2** | Direct regression (joint angles, positions) | Can force student features toward teacher's mean, conflicting with the student modality's optimal distribution |
| **Smooth L1 (Huber)** | Regression with outlier robustness | Less gradient signal for small errors |
| **KL Divergence** | Soft label distillation | Fails in cross-modal settings because logit distributions can't be identical across modalities (C2KD, CVPR 2024) |
| **Contrastive (InfoNCE)** | Representation alignment | Best for learning shared embeddings; requires negative sampling |
| **Feature Disentanglement** | Heterogeneous modalities | Decompose features into low-freq (modality-agnostic, use MSE) and high-freq (modality-specific, use logMSE) |
| **Adversarial** | Ensuring plausible outputs | Used in HMR; training instability risk |

**Best practice (C2KD, CVPR 2024)**: Use **soft constraints** rather than hard alignment. The modality gap means forcing exact feature matching overfits. Feature disentanglement into modality-invariant and modality-specific components outperforms forcing complete alignment.

### 11.10 Additional Key Systems

**Learning by Cheating (CoRL 2020)** — Chen et al. Decomposes autonomous driving into: (1) privileged agent with ground-truth layout learns to act; (2) vision-only agent learns by imitating the privileged agent. First method to achieve 100% success on all CARLA benchmark tasks. Directly demonstrates the train-with-privileged, deploy-without paradigm.

**Asymmetric Actor-Critic (RSS 2018)** — Pinto et al. The foundational paper: critic trains on full simulator state while the actor (policy) receives only RGBD images. Proves asymmetric inputs significantly outperform symmetric inputs. Combined with domain randomization, achieves sim-to-real transfer with zero real-world training data.

**Video Inertial Poser (VIP) / 3DPW Dataset** — Combines IMU sensors with a moving camera for in-the-wild 3D pose. IMU data provides pseudo ground truth for training monocular methods. Produced the 3DPW dataset — first benchmark with accurate 3D poses captured outdoors — now a standard evaluation benchmark.

**Key2Mesh (2024)** — Pre-trains mesh estimation on unpaired MoCap data, then domain-adapts to visual domain using detected 2D keypoints. Outperforms HMR and SPIN in unpaired settings. Shows MoCap data serves as effective privileged pre-training even without paired image-3D data.

**PhysHMR (2025)** — Physics-based framework that distills motion knowledge from a MoCap-trained expert to a vision-conditioned policy in a physics simulator. Produces physically grounded motion reconstructions aligned with input video.

**Forces for Free (Science Robotics, 2024)** — Trains vision-based contact force estimator by observing a compliant robot hand, replacing dedicated force sensors with a camera. Force sensors provide ground-truth labels during training; only a camera is needed at deployment.

### 11.11 Advanced: Hybrid Deployment Strategy

Rather than fully removing IMUs, a practical middle ground:

1. **Train** vision model with IMU ground truth (as above)
2. **Deploy vision-only** as the primary system (no wearables needed)
3. **Optionally wear IMUs** for high-precision tasks — the system fuses both with EKF
4. **Periodic recalibration** sessions with IMUs to maintain vision accuracy

This gives the best of both worlds: zero-burden operation for casual use, full precision when needed.

### 11.12 Key Papers for This Topic

| # | Paper | Year | Venue | Key Contribution |
|---|-------|------|-------|-----------------|
| 1 | Vapnik & Vashist, "Learning Using Privileged Information" | 2009 | *Machine Learning* | Theoretical foundation for LUPI |
| 2 | Kanazawa et al., "HMR" | 2018 | CVPR | MoCap→vision pose estimation with adversarial training |
| 3 | Kolotouros et al., "SPIN" | 2019 | ICCV | Self-improving regression + optimization loop |
| 4 | Lee et al., "Quadrupedal Locomotion" | 2020 | *Science Robotics* | Privileged teacher → sensorimotor student |
| 5 | Kumar et al., "RMA" | 2021 | RSS | Rapid adaptation without privileged info |
| 6 | Guzov et al., "HPS" | 2021 | CVPR | IMU suits as ground truth for vision training |
| 7 | Moon et al., "IMU2CLIP" | 2022 | arXiv | Contrastive IMU-vision alignment |
| 8 | Chen et al., "BEVDistill" | 2023 | CVPR | LiDAR→camera cross-modal distillation |
| 9 | Goel et al., "4DHumans" | 2024 | CVPR | SOTA vision-only pose from MoCap training |
| 10 | Shin et al., "WHAM" | 2024 | CVPR | World-grounded motion from MoCap features |
| 11 | Chen et al., "Learning by Cheating" | 2020 | CoRL | Privileged→vision for autonomous driving |
| 12 | Pinto et al., "Asymmetric Actor-Critic" | 2018 | RSS | Critic sees privileged state, actor sees vision |
| 13 | Huo et al., "C2KD" | 2024 | CVPR | Bridging modality gap with soft constraints |
| 14 | Ponton et al., "Forces for Free" | 2024 | *Science Robotics* | Force sensor→vision distillation |
| 15 | Huang et al., "DIP" + VIP/3DPW | 2018 | SIGGRAPH Asia | IMU pseudo-GT for vision pose benchmarks |

### 11.13 Bottom Line

**This is not speculative — it's the dominant paradigm in human pose estimation.** Every state-of-the-art vision-only pose estimator (HMR, SPIN, CLIFF, 4DHumans, TokenHMR) was trained using body-sensor ground truth (MoCap) and deploys with cameras only. IMUs can serve as a more accessible, affordable alternative to MoCap for generating these training labels.

For SteveROS teleoperation specifically:
- **Short-term**: Use IMUs to collect high-quality paired training data
- **Medium-term**: Train a vision model on this data, deploy camera-only for casual use
- **Long-term**: Fine-tune foundation models (4DHumans, MotionBERT) with your IMU-labeled data for near-MoCap accuracy with zero wearable burden

---

## 12. References

### IMU Teleoperation Systems
1. Kiarostami et al. (2024). "An IMUs and Potentiometer-Based Controller for Robotic Arm-Hand Teleoperation." *Sensors and Actuators A: Physical*.
2. Kruger et al. (2021). "A Wearable IMU System for Flexible Teleoperation of a Collaborative Industrial Robot." *MDPI Sensors*, 21(17), 5871.
3. Li et al. (2020). "A Mobile Robot Hand-Arm Teleoperation System by Vision and IMU." *IEEE IROS 2020*.
4. Darvish et al. (2023). "Teleoperation of Humanoid Robots: A Survey." *IEEE Transactions on Robotics*, 39(3), 1706-1727.

### ML for IMU Pose Estimation
5. Huang et al. (2018). "Deep Inertial Poser." *ACM TOG (SIGGRAPH Asia)*.
6. Yi et al. (2021). "TransPose: Real-time 3D Human Translation and Pose Estimation with Six Inertial Sensors." *ACM TOG*.
7. Yi et al. (2022). "Physical Inertial Poser (PIP)." *CVPR 2022*.
8. Yi et al. (2022). "Transformer Inertial Poser." *SIGGRAPH Asia 2022*.
9. Ponton et al. (2024). "Fast Human Motion Reconstruction from Sparse Inertial Measurement Units." *Nature Communications*.
10. Kang et al. (2024). "IMUOptimize: Optimal IMU Placement with Transformer Architecture." *arXiv:2402.08923*.
11. Mollyn et al. (2023). "IMUPoser: Full-Body Pose Estimation using IMUs in Phones, Watches, and Earbuds." *CHI 2023*.

### Sensor Fusion
12. Al-Mutib et al. (2017). "Pose Estimation Based on Fusion of IMU Data and Vision Data Using EKF." *MDPI Sensors*.
13. Ligorio & Sabatini (2013). "EKF-Based Methods for Pose Estimation Using Visual, Inertial and Magnetic Sensors." *MDPI Sensors*.

### Alternative Teleoperation
14. Wu et al. (2024). "GELLO: A General, Low-Cost, and Intuitive Teleoperation Framework." *IROS 2024*.
15. Chi et al. (2024). "Universal Manipulation Interface (UMI)." *RSS 2024*.
16. Fu et al. (2024). "Mobile ALOHA." *CoRL 2024*.
17. Zhao et al. (2023). "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware (ACT)." *RSS 2023*.
18. Chi et al. (2023). "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion." *RSS 2023*.
19. Qin et al. (2023). "AnyTeleop: A General Vision-Based Dexterous Robot Teleoperation System." *RSS 2023*.

### Filtering
20. Casiez et al. (2012). "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive Systems." *CHI 2012*.

### Open-Source Frameworks
21. OPEN TEACH — https://open-teach.github.io/
22. Bunny-VisionPro — https://arxiv.org/abs/2407.03162
23. VisionProTeleop — https://github.com/Improbable-AI/VisionProTeleop
24. Fourier Teleoperation System — https://github.com/FFTAI/teleoperation
25. MobilePoser — https://github.com/SPICExLAB/MobilePoser

### Cross-Modal Distillation / Privileged Learning
26. Vapnik & Vashist (2009). "A New Learning Paradigm: Learning Using Privileged Information." *Machine Learning*.
27. Kanazawa et al. (2018). "End-to-end Recovery of Human Shape and Pose (HMR)." *CVPR 2018*.
28. Kolotouros et al. (2019). "SPIN: Learning to INfer 3D Human Pose with Self-Improving Network." *ICCV 2019*.
29. Goel et al. (2024). "4DHumans: Reconstructing and Tracking Humans with Transformers." *CVPR 2024*.
30. Zhu et al. (2023). "MotionBERT: A Unified Perspective on Learning Human Motion Representations." *ICCV 2023*.
31. Shin et al. (2024). "WHAM: Reconstructing World-grounded Humans with Accurate 3D Motion." *CVPR 2024*.
32. Dwivedi et al. (2024). "TokenHMR: Advancing Human Mesh Recovery with a Tokenized Pose Representation." *CVPR 2024*.
33. Guzov et al. (2021). "Human POSEitioning System (HPS)." *CVPR 2021*.
34. Moon et al. (2022). "IMU2CLIP: Multimodal Contrastive Learning for IMU Motion Sensors." *arXiv:2210.14395*.
35. Chen et al. (2023). "BEVDistill: Cross-Modal BEV Knowledge Distillation." *CVPR 2023*.
36. Chong et al. (2022). "MonoDistill: Learning Spatial Features for Monocular 3D Object Detection." *ICLR 2022*.
37. Lee et al. (2020). "Learning Quadrupedal Locomotion over Challenging Terrain." *Science Robotics*.
38. Kumar et al. (2021). "RMA: Rapid Motor Adaptation for Legged Robots." *RSS 2021*.
39. Li et al. (2022). "See, Hear, and Feel: Smart Sensory Fusion for Robotic Manipulation." *CoRL 2022*.
40. Yang et al. (2024). "Depth Anything V2." *arXiv:2406.09414*.
41. Iskakov et al. (2019). "Learnable Triangulation of Human Pose." *ICCV 2019*.
42. Chen et al. (2020). "Learning by Cheating." *CoRL 2020*.
43. Pinto et al. (2018). "Asymmetric Actor Critic for Image-Based Robot Learning." *RSS 2018*.
44. Huo et al. (2024). "C2KD: Bridging the Modality Gap for Cross-Modal Knowledge Distillation." *CVPR 2024*.
45. (2024). "Forces for Free: Vision-Based Contact Force Estimation with a Compliant Hand." *Science Robotics*.
46. (2024). "Key2Mesh: MoCap-to-Visual Domain Adaptation for Human Mesh Estimation." *arXiv:2404.07094*.
47. (2025). "PhysHMR: Physics-Based Human Motion Reconstruction from Vision." *arXiv:2510.02566*.
48. Tan et al. (2023). "EgoDistill: Egocentric Head Motion Distillation for Efficient Video Understanding." *NeurIPS 2023*.
49. Chi et al. (2024). "Universal Manipulation Interface (UMI)." *RSS 2024*. https://umi-gripper.github.io/
50. von Marcard et al. (2018). "Recovering Accurate 3D Human Pose in the Wild Using IMUs and a Moving Camera (3DPW)." *ECCV 2018*.
51. Simoni et al. (2024). "Depth-based Privileged Information for Boosting 3D Human Pose Estimation on RGB." *arXiv:2409.11104*.
52. Chen et al. (2025). "COMODO: Cross-Modal Video-to-IMU Distillation." *arXiv:2503.07259*.
53. Gupta et al. (2016). "Cross Modal Distillation for Supervision Transfer." *CVPR 2016*.
54. Hoffman et al. (2016). "Learning with Side Information through Modality Hallucination." *CVPR 2016*.
