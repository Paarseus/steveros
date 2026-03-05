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
11. [References](#11-references)

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

## 11. References

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
