# VR Teleoperation for Robotic Arms: Research & Universal Patterns

## Universal Teleoperation Pipeline

Every VR/XR teleoperation system follows the same 8-step pattern from human input to robot command:

### Step 1: Capture
Read human input from the device at its native rate.

| Device Type | Output | Example Systems |
|-------------|--------|-----------------|
| Backdrivable leader arm | Joint angles (radians) from servo encoders | ALOHA, GELLO |
| VR controller | 6DOF grip SE3 pose (position + quaternion) + buttons | teleop_xr, BEAVR |
| VR hand tracking | 21-27 hand joint positions/transforms per hand | Open-TeleVision, OPEN TEACH, Bunny-VisionPro |
| RGB camera | 21 MediaPipe hand keypoints (3D, wrist-relative) | AnyTeleop |
| Handheld tool + SLAM | 6DOF camera trajectory from visual-inertial SLAM | UMI |
| Mocap gloves | 21 joint positions + quaternions per hand | DexCap |

### Step 2: Snapshot (Calibration)
On activation (trigger press, deadman grip, pause/resume), record the initial state of both the human input device and the robot:

```
human_init = current_human_pose
robot_init = current_robot_pose  (from FK or joint state feedback)
```

This avoids the need for absolute spatial calibration between the VR space and robot workspace. All motion is relative from this moment forward.

Systems that skip this step (ALOHA, GELLO) do so because leader and follower share the same joint representation, so joint angles transfer 1:1 with no offset.

### Step 3: Compute Delta
Compute the relative motion since the snapshot:

```
delta = inv(human_init) @ human_current
```

For homogeneous 4x4 matrices this gives a relative SE3 transform. For joint-space systems (ALOHA, GELLO), this step is trivial -- the current joint angles ARE the command.

### Step 4: Frame Transform
Rotate the delta from the input device's coordinate frame to the robot's coordinate frame using a fixed rotation matrix.

Common frame mismatches:

| Input Frame | Robot Frame | Transform |
|-------------|-------------|-----------|
| WebXR RUB (Right-Up-Back) | ROS FLU (Forward-Left-Up) | `[[0,0,-1],[-1,0,0],[0,1,0]]` |
| WebXR Y-up | Robot Z-up | `[[0,0,-1],[-1,0,0],[0,1,0]]` |
| Unity left-handed Y-up | Robot Z-up right-handed | `[[0,1,0],[-1,0,0],[0,0,1]]` |
| Apple Vision Pro Y-up | Robot Z-up | `[[0,0,-1],[-1,0,0],[0,1,0]]` |

The rotation is applied differently to translation and orientation:

```
translation_robot = R_input2robot @ delta_translation
rotation_robot    = R_input2robot @ delta_rotation @ inv(R_input2robot)  # conjugation
```

The conjugation (similarity transform) for rotations is critical -- it preserves the rotation axis direction while changing the coordinate frame it's expressed in.

### Step 5: Apply to Robot Initial Pose
Add the transformed delta to the robot's initial pose from the snapshot:

```
target_pose = robot_init @ transformed_delta
```

Or equivalently:
```
target_position    = robot_init_position + R_input2robot @ (human_current_pos - human_init_pos)
target_orientation = (R_input2robot @ delta_rotation @ inv(R_input2robot)) @ robot_init_orientation
```

### Step 6: Solve (if needed)
Convert the Cartesian target pose to robot joint angles. The method depends on the system:

| Method | Description | Systems |
|--------|-------------|---------|
| None (direct joint copy) | 1:1 joint angle transfer, no IK needed | ALOHA, GELLO |
| Damped least-squares IK | `J^T(JJ^T + lambda*I)^{-1} * err`, iterative | Bunny-VisionPro (Pinocchio) |
| Optimization-based IK | NLopt SLSQP or JAX least-squares, multi-cost | teleop_xr (PyRoki), AnyTeleop (dex-retargeting) |
| Cartesian velocity command | `velocity = (target - current) * gain`, no IK | OPEN TEACH (Kinova) |
| PyBullet IK | `calculateInverseKinematics` with null-space | DexCap, BEAVR (hand only) |
| Robot-side Cartesian control | Send Cartesian pose, robot firmware does IK | UMI (UR5 servoL), BEAVR (xArm) |

### Step 7: Filter
Smooth the output to remove jitter and ensure safe motion:

| Filter | Formula | Typical Alpha |
|--------|---------|---------------|
| Low-pass (exponential) | `y = y_prev + alpha * (x - y_prev)` | 0.1 - 0.5 |
| Moving average | `y = mean(buffer[-N:])` | N = 3-10 |
| Complementary | `y = alpha * sensor_A + (1-alpha) * sensor_B` | 0.8 |
| Velocity dead-zone | zero out velocities below threshold | 0.001 - 0.01 |

### Step 8: Send
Publish the command to the robot controller:

| Protocol | Rate | Systems |
|----------|------|---------|
| Dynamixel USB serial (1Mbaud) | 50-1000 Hz | ALOHA, GELLO |
| ROS1 topics (JointGroupCommand) | 50 Hz | ALOHA |
| ROS2 topics (JointTrajectory) | 60-90 Hz | teleop_xr, Bunny-VisionPro |
| ZMQ PUB/SUB (pickle) | 30-60 Hz | OPEN TEACH, BEAVR, Bunny-VisionPro |
| RTDE (UR5 servoL) | 500 Hz | UMI, GELLO |
| XArm TCP/IP API | 50-250 Hz | Bunny-VisionPro, BEAVR |
| Cartesian velocity (ROS) | 60 Hz | OPEN TEACH (Kinova) |

---

## System-by-System Breakdown

### ALOHA / Mobile ALOHA (Stanford, 2023-2024)

- **Input**: Backdrivable WidowX 250s leader arms (Dynamixel, torque off, PWM mode)
- **Output**: `np.float64[14]` — 6 joint angles + 1 gripper per arm
- **Calibration**: None for arm joints (same servo representation). Gripper: linear remap between master range `[-0.68, 0.31]` and puppet range `[-0.62, 1.49]`. Opening ceremony moves all arms to fixed start pose
- **Retargeting**: Direct 1:1 joint copy. `puppet.set_joint_positions(master.dxl.joint_states.position[:6])`
- **Comms**: ROS1, Dynamixel USB serial 1Mbaud, udev symlinks per serial number
- **Rate**: 50 Hz (min 42 Hz acceptable). `profile_velocity=0, profile_acceleration=0` on puppets for instant response
- **Recording**: HDF5 — `qpos[14]`, `qvel[14]`, `effort[14]`, `images[480x640x3]`, `action[14]`
- **Repos**: `tonyzhaozh/aloha`, `MarkFzp/mobile-aloha`

### Open-TeleVision (UCSD/MIT, CoRL 2024)

- **Input**: WebXR Hands API via Vuer: 4x4 wrist SE3 + 25x3 finger landmarks + 4x4 head pose, column-major
- **Calibration**: Hardcoded matrices. `grd_yup2grd_zup` applied as similarity transform: `result = M @ input @ inv(M)`
- **Arm Retargeting**: Direct position — wrist XYZ + fixed offset `[-0.6, 0, 1.6]` becomes robot position
- **Finger Retargeting**: `dex-retargeting` VectorOptimizer (NLopt SLSQP). Huber loss on inter-finger vectors. Pinocchio analytical Jacobians. Low-pass alpha=0.5
- **Comms**: WebSocket/HTTPS, Python multiprocessing shared memory. No ROS
- **Rate**: 72-90 Hz (WebXR), 60 Hz (IsaacGym sim)
- **Repo**: `OpenTeleVision/TeleVision`

### OPEN TEACH (NYU, CoRL 2024)

- **Input**: Meta Quest OVR SDK (Unity): 24 bone xyz per hand, pipe-delimited string
- **Calibration**: Runtime initial pose capture. Thumb gets separate `OculusThumbBoundCalibrator` (perspective transform)
- **Arm Retargeting**: Palm frame from knuckle cross-products → relative motion → conjugated through `H_R_V` → Cartesian velocity to Kinova (`displacement * 20`)
- **Finger Retargeting**: Geometric bone angle measurement, per-joint scaling. Thumb: KDL IK
- **Comms**: ZMQ PUSH/PULL NetMQ (Quest→PC), ZMQ PUB/SUB pickle (internal), ROS1 (→Kinova)
- **Rate**: 60 Hz single arm, 90 Hz bimanual
- **Repo**: `aadhithya14/Open-Teach`

### UMI (Stanford/Columbia, RSS 2024)

- **Input**: GoPro fisheye → ORB-SLAM3 monocular-inertial → `[t, x, y, z, qx, qy, qz, qw]`
- **Calibration**: Fixed cam-to-TCP offset (8.6cm + 20.5cm), ArUco table tag → `tx_slam_tag`, gripper width from ArUco finger tags
- **Retargeting**: Direct Cartesian — chain of rigid transforms: `tx_tag_tcp = tx_tag_slam @ tx_slam_cam @ tx_cam_tcp`
- **Comms**: RTDE (UR5, 500 Hz) via separate process + SharedMemoryQueue. `servoL(lookahead=0.1, gain=300)`
- **Rate**: 10-20 Hz policy loop, 500 Hz RTDE inner loop
- **Action format**: 10D — 3D position + 6D rotation (continuous repr) + 1D gripper. Zarr storage
- **Repo**: `real-stanford/universal_manipulation_interface`

### GELLO (UC Berkeley, 2023)

- **Input**: Dynamixel encoders on kinematic replica. `raw / 2048 * pi` radians. Exponential smoothing alpha=0.99
- **Calibration**: Joint offset table (multiples of pi/2) + joint signs (+1/-1), hardcoded per USB serial. Startup 2pi correction: `offset += 2pi * round((-target + current) / 2pi) * sign`
- **Retargeting**: Direct 1:1 joint copy. `robot.command_joint_state(gello.get_joint_state())`
- **Comms**: USB serial 57600 baud (Dynamixel SDK). ZMQ REQ/REP pickle, or direct API (RTDE/Polymetis/XArmAPI)
- **Rate**: 100 Hz control loop, ~1000 Hz servo read thread
- **Safety**: Refuses start if any joint delta > 0.8 rad. 25-step gradual alignment
- **Repo**: `wuphilipp/gello_software`

### AnyTeleop / dex-retargeting (UCSD/NVIDIA, RSS 2023)

- **Input**: MediaPipe 21 hand keypoints (3D). Transformed through SVD-based wrist frame + `operator2mano = [[0,0,-1],[-1,0,0],[0,1,0]]`
- **Calibration**: URDF + YAML config only. Scaling factor per robot (Allegro=1.6x). No per-user calibration
- **Retargeting**: NLopt SLSQP with three modes:
  - PositionOptimizer: absolute fingertip positions
  - VectorOptimizer: inter-joint displacement vectors (scale-invariant)
  - DexPilotOptimizer: snap-to-grasp (200-400x weight when tips < 3cm, hysteresis at 5cm)
- **Features**: Warm start from previous solution, regularization `||q - q_prev||^2`, low-pass alpha=0.1
- **Rate**: ~30 Hz (camera-limited), 1-5ms per NLopt solve
- **Repo**: `dexsuite/dex-retargeting`

### Bunny-VisionPro (IROS 2025)

- **Input**: Apple Vision Pro ARKit: 27 joint 4x4 SE3 per hand via gRPC protobuf (port 12345)
- **Calibration**: Alignment-based init — robot sends base poses + init joint config → server computes `init2base` from EE FK midpoint. No markers
- **Arm Retargeting**: Pinocchio damped least-squares IK: `J^T(JJ^T + lambda*I)^{-1} * err`, 100 iterations. Low-pass alpha=0.1 on wrist pos/rot before IK
- **Finger Retargeting**: `dex-retargeting` VectorOptimizer (NLopt SLSQP). 25→21 joints (drop 4 metacarpals)
- **Comms**: gRPC (AVP→PC), ZMQ PUB/SUB pickle port 5500 (PC→robot), ROS2 (internal)
- **Rate**: ~90 Hz ARKit, 60 Hz command output, 250 Hz arm internal PID
- **Coord transform**: Y-up→Z-up, then `OPERATOR2AVP = [[0,0,-1],[-1,0,0],[0,1,0]]`
- **Repo**: `Dingry/BunnyVisionPro`

### DexCap (Stanford, RSS 2024)

- **Input**: Rokoko gloves (21 joints, xyz+quat, UDP JSON port 14551) + 3x RealSense T265 SLAM + L515 depth
- **Calibration**: 6+ steps — rigid cam-to-body offsets, hand-to-tracker euler/translation calibration files, 45-degree mount correction, manual point cloud alignment to table, canonical frame drift correction
- **Retargeting**: Offline PyBullet `calculateInverseKinematics` with null-space. 4 fingertips (no pinky), 1000 max iterations. Multiple axis flips, joint swaps, +pi offsets for LEAP hand
- **Comms**: UDP/JSON (gloves), Redis localhost:6669 (IPC), file I/O
- **Rate**: ~30 fps recording. Retargeting is offline batch
- **Repo**: `j96w/DexCap`

### BEAVR (MIT ARCLab, 2025)

- **Input**: Meta Quest Unity app: 26 XRHandJointID keypoints, text-encoded `"<hand>:x,y,z|x,y,z|..."`
- **Calibration**: Pause/resume captures initial hand frame + robot EE. Fixed `H_R_V` and `H_T_V` 4x4 per arm
- **Arm Retargeting**: Palm frame from knuckle Gram-Schmidt → relative 4x4 → rotation conjugated through `H_R_V`, translation through `H_T_V` → Cartesian target to xArm
- **Hand Retargeting**: PyBullet IK for 16 LEAP hand joints. Keypoints scaled (thumb 1.7x, fingers 1.8x)
- **Comms**: ZMQ PUSH/PULL (Quest→PC), ZMQ PUB/SUB pickle (internal), multi-process
- **Rate**: 30 Hz all components
- **Recording**: LeRobot dataset format (Parquet + MP4)
- **Repo**: `ARCLab-MIT/beavr-bot`

### teleop_xr (qrafty-ai, 2025)

- **Input**: WebXR API (any browser): controller grip SE3 + gamepad + optional 25 hand joints, JSON over WSS
- **Calibration**: Deadman snapshot (both grips held) captures XR + robot FK. `TF_RUB2FLU = [[0,0,-1,0],[-1,0,0,0],[0,1,0,0],[0,0,0,1]]`. Per-robot `SO3` orientation
- **Arm Retargeting**: PyRoki/jaxls whole-body IK — JAX JIT-compiled, 15 iterations, costs: position(50), orientation(10), limits(100), rest(5), manipulability(0.01). Translation via `ros_to_base @ delta`, rotation via `ros_to_base @ delta_rot @ base_to_ros`
- **Comms**: WebSocket JSON/WSS (VR→server), ROS2 `JointTrajectory` (→robot), WebRTC (video→headset)
- **Rate**: 72-90 Hz (WebXR native), IK triggered per frame
- **Repo**: `qrafty-ai/teleop_xr`

---

## Finger/Hand Retargeting Methods

Three dominant approaches for mapping human finger motion to robot hand joints:

### 1. Optimization-Based (dex-retargeting)
Used by: Open-TeleVision, Bunny-VisionPro, AnyTeleop

- NLopt SLSQP nonlinear optimizer
- Pinocchio FK + analytical Jacobians for gradients
- Huber loss on inter-finger vectors (VectorOptimizer) or positions (PositionOptimizer)
- Warm start from previous solution + regularization toward previous
- DexPilot extension: snap-to-grasp with hysteresis (200-400x weight when tips < 3cm)
- Typical solve time: 1-5ms

### 2. Geometric Angle Computation
Used by: OPEN TEACH

- Measure angles between consecutive bone segments directly
- Per-joint scaling factors
- Thumb handled separately via KDL IK or perspective transform
- Simpler, faster, but less generalizable across hand morphologies

### 3. Physics-Engine IK
Used by: DexCap, BEAVR

- PyBullet `calculateInverseKinematics` with null-space projection
- Target fingertip positions as IK goals
- 1000 max iterations, 0.001 residual threshold
- Requires careful joint remapping and offset correction per robot hand

---

## Coordinate Frame Transforms (Reference)

### WebXR (RUB) to ROS (FLU)
```
RUB2FLU = [[0, 0, -1],
           [-1, 0,  0],
           [0,  1,  0]]
```
Maps: ROS_X = -WebXR_Z, ROS_Y = -WebXR_X, ROS_Z = WebXR_Y

### Unity (left-handed Y-up) to Robot (right-handed Z-up)
```
UNITY2ROBOT = [[0,  1, 0],
               [-1, 0, 0],
               [0,  0, 1]]
```

### Apple Vision Pro to Robot
```
AVP2ROBOT = [[0,  0, -1],
             [-1, 0,  0],
             [0,  1,  0]]
```

### Applying rotation to SE3 delta (similarity transform / conjugation)
```python
# For translation: simple rotation
t_robot = R @ t_input

# For orientation: conjugation preserves rotation axis semantics
R_robot = R @ R_input @ R.T
```

---

## Foundational Papers

### Classical Control Theory
| Paper | Authors | Year | Contribution |
|-------|---------|------|-------------|
| Force-reflecting positional servo mechanism | Goertz | 1952 | Origin of bilateral master-slave teleoperation |
| Impedance Control (Parts I-III) | Hogan | 1985 | Foundation for haptic teleoperation (~3,355 citations) |
| Bilateral Control with Time Delay | Anderson & Spong | 1989 | Passivity/scattering theory for stability under delay |
| Stable Adaptive Teleoperation | Niemeyer & Slotine | 1991 | Wave variables for delay-robust communication |
| Telerobotics, Automation, and Human Supervisory Control | Sheridan | 1992 | The textbook -- defined supervisory control and autonomy levels |
| Stability and Transparency in Bilateral Teleoperation | Lawrence | 1993 | Formalized stability-transparency tradeoff |
| Virtual Fixtures | Rosenberg | 1993 | Computer-generated motion guides for teleoperation |

### Surveys
| Paper | Authors | Year | Venue |
|-------|---------|------|-------|
| Bilateral Teleoperation: An Historical Survey | Hokayem & Spong | 2006 | Automatica |
| Robot Learning from Demonstration: A Survey | Argall et al. | 2009 | RAS |
| Teleoperation of Humanoid Robots: A Survey | Darvish et al. | 2023 | IEEE T-RO |

### Policy Learning from Teleoperation Data
| Paper | Year | Method | Key Result |
|-------|------|--------|------------|
| Diffusion Policy (Chi et al.) | RSS 2023 | DDPM action diffusion | 46.9% avg improvement, dominant architecture |
| ACT / ALOHA (Zhao et al.) | RSS 2023 | CVAE action chunking | 80-90% from 10 min demos |
| RT-1 (Google, 50+ authors) | RSS 2023 | Transformer on 130K episodes | Massive-scale generalization |
| pi0 (Physical Intelligence) | 2024 | VLA flow model | 68 tasks, 7 platforms, open-sourced |
| RDT-1B | ICLR 2025 | 1.2B diffusion transformer | New skills from 1-5 demos |

### Data Infrastructure
| Resource | Description |
|----------|-------------|
| LeRobot (Hugging Face) | Standardized dataset format, integrates ACT + Diffusion Policy |
| Open X-Embodiment | 1M+ trajectories, 22 embodiments, 60 datasets |
| Robomimic | Benchmark showing demo quality > quantity |

---

## Papers & Projects Demonstrating the VR → Robot Pipeline

### Tier 1: Most Popular & Widely Adopted (Open-Source, Top Venues)

| Paper | Authors | Year | Venue | VR Device | Robot | arXiv | Project Page |
|-------|---------|------|-------|-----------|-------|-------|-------------|
| Open-TeleVision | Cheng et al. (UCSD/MIT) | 2024 | CoRL | Apple Vision Pro / Quest | Unitree H1 humanoid | [2407.01512](https://arxiv.org/abs/2407.01512) | [robot-tv.github.io](https://robot-tv.github.io/) |
| OPEN TEACH | Iyer et al. (NYU/Meta) | 2024 | CoRL | Meta Quest 3 | Franka, xArm, Jaco, Allegro, Stretch | [2403.07870](https://arxiv.org/abs/2403.07870) | [open-teach.github.io](https://open-teach.github.io/) |
| Bunny-VisionPro | Ding et al. (UCSD/HKU) | 2024 | IROS 2025 | Apple Vision Pro | xArm7 + Ability Hand (bimanual) | [2407.03162](https://arxiv.org/abs/2407.03162) | [dingry.github.io](https://dingry.github.io/projects/bunny_visionpro.html) |
| ALOHA / Mobile ALOHA | Zhao/Fu et al. (Stanford) | 2023-24 | RSS / CoRL | Physical leader-follower | ViperX bimanual + mobile base | [2304.13705](https://arxiv.org/abs/2304.13705) | [mobile-aloha.github.io](https://mobile-aloha.github.io/) |
| OmniH2O | He et al. (CMU) | 2024 | CoRL | VR / RGB camera / voice | Full humanoid whole-body | [2406.08858](https://arxiv.org/abs/2406.08858) | [omni.human2humanoid.com](https://omni.human2humanoid.com/) |
| AnyTeleop | Qin et al. (UCSD/NVIDIA) | 2023 | RSS | RGB camera (no VR needed) | Multi-arm + dexterous hands | [2307.04577](https://arxiv.org/abs/2307.04577) | [anyteleop.com](https://anyteleop.com/) |
| BEAVR | Posadas-Nava et al. (MIT) | 2025 | ICCR | Meta Quest 3 | xArm + LEAP hand, RX-1 humanoid | [2508.09606](https://arxiv.org/abs/2508.09606) | [arclab-mit.github.io](https://arclab-mit.github.io/beavr-landing/) |

### Tier 2: Important Recent Systems

| Paper | Year | Venue | Key Innovation | arXiv |
|-------|------|-------|----------------|-------|
| ACE (Cross-Platform Visual-Exoskeletons) | 2024 | CoRL | Camera + exoskeleton, multi-platform, cheapest | [2408.11805](https://arxiv.org/abs/2408.11805) |
| Holo-Dex (NYU) | 2023 | ICRA | First commodity VR for dexterous hand IL | [2210.06463](https://arxiv.org/abs/2210.06463) |
| TeleMoMa (UT Austin) | 2024 | arXiv | Modular, supports VR+camera+keyboard+joystick | [2403.07869](https://arxiv.org/abs/2403.07869) |
| XRoboToolkit | 2025 | SII 2026 | OpenXR cross-platform standard, QP-based IK | [2508.00097](https://arxiv.org/abs/2508.00097) |
| LeVR (Northwestern) | 2025 | arXiv | Native LeRobot integration for dexterous teleop | [2509.14349](https://arxiv.org/abs/2509.14349) |
| CLONE (PKU) | 2025 | CoRL | Closed-loop whole-body, MoE policy, 12cm drift over 8.9m | [2506.08931](https://arxiv.org/abs/2506.08931) |
| Mobile-TeleVision (UCSD) | 2024 | arXiv | Predictive motion priors for locomotion + teleop | [2412.07773](https://arxiv.org/abs/2412.07773) |
| TelePreview (NUS) | 2024 | ICRA 2025 WS Best Paper | Virtual arm preview before execution, <$1K | [2412.13548](https://arxiv.org/abs/2412.13548) |
| SPARK-Remote (UMN) | 2025 | arXiv | <$600 bimanual remote teleop, 200+ miles | [2504.05488](https://arxiv.org/abs/2504.05488) |
| Quest2ROS / Quest2ROS2 | 2024 | HRI WS | 0.46mm accuracy, 82ms latency, standalone Quest app | [quest2ros.github.io](https://quest2ros.github.io/) |
| OpenVR (SoftwareX) | 2025 | SoftwareX | Minimal open-source VR->Franka, consumer hardware only | [2305.09765](https://arxiv.org/abs/2305.09765) |
| ARMADA (Apple) | 2024 | arXiv | AR robot-free data collection, 1.3%->71.1% replay success | [2412.10631](https://arxiv.org/abs/2412.10631) |

### Tier 3: Foundational / Classic Papers

| Paper | Authors | Year | Venue | Why It's Foundational |
|-------|---------|------|-------|-----------------------|
| Baxter's Homunculus | Lipton, Fay, Rus (MIT) | 2018 | RA-L/IROS | First modern consumer VR (HTC Vive) -> robot arm. 95% grasp, 57% faster |
| ROS Reality | Whitney et al. (Brown) | 2018 | IROS | First open-source ROS-to-Unity-to-VR framework. 66% faster than keyboard |
| DexPilot | Handa et al. (NVIDIA) | 2020 | ICRA | Vision-based bare-hand -> 23-DOF dexterous hand-arm |
| RoboTurk | Mandlekar et al. (Stanford) | 2018 | CoRL | Crowdsourced phone teleop, 2200+ demos, scalable data collection |
| Ramos & Kim | (MIT) | 2019 | Science Robotics | Bilateral dynamic locomotion teleop with balance feedback |
| Guthart & Salisbury (da Vinci) | (Intuitive Surgical) | 2000 | ICRA | The seminal surgical teleop paper, 9000+ systems deployed |
| Robonaut 2 | Diftler et al. (NASA) | 2011 | ICRA | First humanoid teleoperated in space (ISS) |
| Predictive Display | Bejczy et al. (JPL) | 1990 | ICRA | "Phantom robot" ghost overlay to compensate for delay |
| PHANToM Haptic | Massie & Salisbury | 1994 | ASME | The haptic device that enabled a generation of teleop research |
| Shared Autonomy | Dragan & Srinivasa (CMU) | 2013 | IJRR | Policy-blending formalism for human-robot shared control |
| Director (DARPA DRC) | Marion, Tedrake et al. (MIT) | 2017 | JFR | Team MIT's Atlas operator UI for DARPA Robotics Challenge |

### Tier 4: Domain-Specific Applications

| Domain | Notable Work | Link |
|--------|-------------|------|
| Surgical | Orbit-Surgical (sim for dVRK/STAR), MR teleop with HoloLens 2 | [arxiv.org/abs/2404.16027](https://arxiv.org/abs/2404.16027) |
| Space | NASA Valkyrie VR Cockpit (Unity, full-body humanoid on ISS) | [ntrs.nasa.gov/20220007587](https://ntrs.nasa.gov/citations/20220007587) |
| Underwater | MR-UBi (mixed reality bilateral underwater arm control) | [arxiv.org/abs/2510.20407](https://arxiv.org/abs/2510.20407) |
| Nuclear | UCL immersive VR for radiation monitoring robots | IEEE ICRA 2023 |
| Industrial | ABB IRB1200 via HTC Vive over global internet | Springer 2025 |
| BCI | BrainGate -> robotic arm (Nature 2012, person drank coffee after 15 years) | [nature.com/articles/nature11076](https://www.nature.com/articles/nature11076) |

### Tier 5: Foundation Models + Teleop

| Paper | Year | Key Innovation | arXiv |
|-------|------|----------------|-------|
| SONIC (NVIDIA) | 2025 | 42M param foundation model accepting teleop/video/text/VLA inputs for whole-body humanoid | [2511.07820](https://arxiv.org/abs/2511.07820) |
| ExtremControl | 2026 | 50ms end-to-end latency by direct extremity SE3 control (skip retargeting) | [2602.11321](https://arxiv.org/abs/2602.11321) |
| TWIST | 2025 | RL+BC teacher-student for whole-body imitation from mocap | [2505.02833](https://arxiv.org/abs/2505.02833) |

### The Canonical Survey

**"Teleoperation of Humanoid Robots: A Survey"** -- Darvish et al., IEEE T-RO 2023
[arXiv:2301.04317](https://arxiv.org/abs/2301.04317) -- Covers the entire pipeline from sensing through retargeting to control.

---

## Educational Resources for the Pipeline

### Full Pipeline Walkthroughs
| Resource | Type | URL |
|----------|------|-----|
| Jonas Metzger -- A Teleoperation Pipeline for Imitation Learning | Blog + code | [jonasmetzger.com](https://jonasmetzger.com/projects/imitation_learning/) |
| LeRobot Robot Learning Tutorial | Interactive tutorial | [huggingface.co](https://huggingface.co/spaces/lerobot/robot-learning-tutorial) |
| RSS 2024: Supervised Policy Learning for Real Robots | Conference tutorial (video) | [supervised-robot-learning.github.io](https://supervised-robot-learning.github.io/) |
| Humanoid Teleoperation Survey Website | Survey site with diagrams | [humanoid-teleoperation.github.io](https://humanoid-teleoperation.github.io/3-retargeting-control/) |
| Chris Paxton -- Remote Robotic Teleoperation | Blog post | [itcanthink.substack.com](https://itcanthink.substack.com/p/remote-robotic-teleoperation) |

### Coordinate Frames & SE(3) Math (Steps 3-5)
| Resource | Type | URL |
|----------|------|-----|
| Modern Robotics Ch. 3 (Lynch & Park) | Textbook (free PDF) | [hades.mech.northwestern.edu](https://hades.mech.northwestern.edu/images/7/7f/MR.pdf) |
| Kris Hauser -- Coordinate Transformations | Online textbook chapter | [motion.cs.illinois.edu](https://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html) |
| Articulated Robotics -- Coordinate Transforms | Blog + video | [articulatedrobotics.xyz](https://articulatedrobotics.xyz/tutorials/coordinate-transforms/coordinate-transforms/) |
| ROS2 Quaternion Fundamentals | Official docs | [docs.ros.org](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html) |
| ETH Zurich -- Introduction to Quaternions | Lecture notes | [ethz.ch](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/asl-dam/documents/lectures/robot_dynamics/RD2_Quaternions.pdf) |
| Columbia -- Coordinate Frames Lecture | Lecture notes | [cs.columbia.edu](https://www.cs.columbia.edu/~allen/F19/NOTES/frames2.pdf) |

### Inverse Kinematics (Step 6)
| Resource | Type | URL |
|----------|------|-----|
| CMU IK Survey (Buss) | Survey paper | [cs.cmu.edu](https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf) |
| Damped Least Squares (Buss & Kim) | Paper | [courses.cs.washington.edu](https://courses.cs.washington.edu/courses/cse599j/12sp/papers/DampedLeastSquares.pdf) |
| PyRoki Docs -- Basic IK Example | Code tutorial | [chungmin99.github.io](https://chungmin99.github.io/pyroki/examples/01_basic_ik/) |
| Robot Academy -- IK Masterclass (Peter Corke) | Video | [robotacademy.net.au](https://robotacademy.net.au/masterclass/inverse-kinematics-and-robot-motion/) |
| Automatic Addison -- Ultimate IK Guide | Blog tutorial | [automaticaddison.com](https://automaticaddison.com/the-ultimate-guide-to-inverse-kinematics-for-6dof-robot-arms/) |

### Retargeting (Steps 4-6 for hands)
| Resource | Type | URL |
|----------|------|-----|
| dex-retargeting library | Python library + examples | [github.com/dexsuite/dex-retargeting](https://github.com/dexsuite/dex-retargeting) |
| Ayusawa -- Motion Retargeting for Humanoid Robots (IEEE T-RO 2017) | Paper | [staff.aist.go.jp](https://staff.aist.go.jp/e.yoshida/papers/Ayusawa-Retargeting-2017TRO.pdf) |
| Design Space of Control Coordinates in Telemanipulation | Paper | [arxiv.org](https://arxiv.org/html/2403.05757v1) |

### Filtering (Step 7)
| Resource | Type | URL |
|----------|------|-----|
| WPILib -- Introduction to Filters | Docs | [docs.wpilib.org](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/introduction.html) |
| SimpleFOC -- Low Pass Filter | Docs | [docs.simplefoc.com](https://docs.simplefoc.com/low_pass_filter) |

### Teleoperation Theory (Classical)
| Resource | Type | URL |
|----------|------|-----|
| Stanford ME 328 -- Teleoperation Lecture (Okamura) | Lecture slides | [web.stanford.edu](https://web.stanford.edu/class/me328/lectures/lecture3-teleoperation.pdf) |
| Springer Handbook of Robotics -- Ch. 43: Telerobotics | Handbook chapter | [link.springer.com](https://link.springer.com/chapter/10.1007/978-3-319-32552-1_43) |
| Sheridan -- Telerobotics, Automation, and Human Supervisory Control | Textbook | [mitpress.mit.edu](https://mitpress.mit.edu/9780262515474/) |

### Imitation Learning from Teleop Data
| Resource | Type | URL |
|----------|------|-----|
| Russ Tedrake -- Underactuated Robotics Ch. 21: Imitation Learning | Online textbook | [underactuated.mit.edu](https://underactuated.mit.edu/imitation.html) |
| Diffusion Policy Explained (Medium) | Blog | [medium.com](https://medium.com/@ligerfotis/diffusion-policy-explained-14a3075ba26c) |
| Diving into Diffusion Policy with LeRobot | Blog | [radekosmulski.com](https://radekosmulski.com/diving-into-diffusion-policy-with-lerobot/) |

### University Courses
| Course | Institution | URL |
|--------|-------------|-----|
| Robotic Manipulation (6.421) | MIT (Russ Tedrake) | [manipulation.csail.mit.edu](https://manipulation.csail.mit.edu/) |
| ME 328 Medical Robotics | Stanford (Allison Okamura) | [web.stanford.edu](https://web.stanford.edu/class/me328/) |
| CS225A Experimental Robotics | Stanford | [cs225a.stanford.edu](https://cs225a.stanford.edu/) |
| Pupper Independent Study | Stanford | [pupper-independent-study.readthedocs.io](https://pupper-independent-study.readthedocs.io/) |

### Best Project Documentation (learn by reading real implementations)
| Project | Best For | URL |
|---------|----------|-----|
| Bunny-VisionPro docs | Full setup + architecture | [dingry.github.io/BunnyVisionPro](https://dingry.github.io/BunnyVisionPro/) |
| BEAVR docs | Modular architecture | [github.com/ARCLab-MIT/beavr-bot](https://github.com/ARCLab-MIT/beavr-bot) |
| K-Scale KBot VR Teleop | Directly relevant to KBot | [docs.kscale.dev](https://docs.kscale.dev/robots/k-bot/teleop/) |
| LeRobot getting started | End-to-end pipeline | [huggingface.co/docs/lerobot](https://huggingface.co/docs/lerobot/en/getting_started_real_world_robot) |
| MoveIt Servo tutorial | ROS2 standard approach | [moveit.picknik.ai](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html) |
| SpesRobotics phone teleop | Quickest hands-on (just needs a phone) | [github.com/SpesRobotics/teleop](https://github.com/SpesRobotics/teleop) |
| PyRoki docs + examples | IK solver for custom pipeline | [chungmin99.github.io/pyroki](https://chungmin99.github.io/pyroki/) |

### Recommended Learning Path

1. **Articulated Robotics** coordinate transforms tutorial (beginner, ROS2)
2. **Modern Robotics Ch. 3** (rigid body math, free PDF)
3. **CMU IK Survey** (Buss) + **PyRoki basic IK example** (practical)
4. **Jonas Metzger's blog** (full pipeline walkthrough)
5. **K-Scale KBot VR teleop docs** (directly relevant to KBot)
6. **LeRobot tutorial** (teleop -> train -> deploy)
7. **RSS 2024 SPL tutorial** (cutting-edge policy learning)

---

## SteveROS VR Teleoperation Pipeline (Chosen Architecture)

### Architecture: teleop_xr (steps 1-5) + MoveIt Servo (steps 6-7) + ros2_control (step 8)

teleop_xr handles VR input capture through target pose computation. Its built-in PyRoki IK is bypassed.
MoveIt Servo handles real-time IK with collision avoidance using the existing URDF/SRDF.
The existing ros2_control stack handles hardware communication unchanged.

### Data Flow

```
Meta Quest 3 (WebXR browser)
    |
    |  WebSocket (JSON over WSS)
    |
    v
teleop_xr (steps 1-5)
    |-- 1. Capture: WebXR controller grip poses + buttons + head pose
    |-- 2. Snapshot: Deadman grip -> captures XR + robot FK initial poses
    |-- 3. Delta: relative motion from snapshot
    |-- 4. Frame Transform: RUB -> FLU + per-robot SO3 conjugation
    |-- 5. Apply: target_pose = robot_init + transformed_delta
    |
    |  Publishes PoseStamped (target EE pose per arm)
    |
    v
MoveIt Servo (steps 6-7)
    |-- 6. Solve: real-time IK from URDF/SRDF (KDL or TRAC-IK backend)
    |      + collision avoidance (self-collision + scene)
    |      + singularity handling (velocity scaling near singularities)
    |-- 7. Filter: built-in smoothing + velocity/acceleration limits
    |
    |  Publishes JointTrajectory
    |
    v
joint_trajectory_controller (step 8)
    |-- 8. Send: spline-interpolated pos + vel at 100 Hz
    |
    |  Position + velocity command interfaces
    |
    v
steveros_hardware (ros2_control)
    |-- MIT command frame over SocketCAN (can0)
    |-- Robstride RS02/RS03/RS04 motors
    |-- K-Scale production gains (Kp/Kd per motor)
```

### ROS2 Topics

```
teleop_xr
  publishes:
    /left_ee_target   (geometry_msgs/PoseStamped)    <- left arm target
    /right_ee_target  (geometry_msgs/PoseStamped)    <- right arm target
    /head_pose        (geometry_msgs/PoseStamped)    <- head tracking
    /joy              (sensor_msgs/Joy)              <- buttons/triggers
    /tf               (geometry_msgs/TransformStamped) <- XR device frames
  subscribes:
    /joint_states     (sensor_msgs/JointState)       <- for FK in step 5

MoveIt Servo
  subscribes:
    /servo_target     (geometry_msgs/PoseStamped)    <- from teleop_xr
       or
    /delta_twist      (geometry_msgs/TwistStamped)   <- Cartesian velocity mode
  publishes:
    /joint_trajectory_controller/joint_trajectory
                      (trajectory_msgs/JointTrajectory)

joint_trajectory_controller
  subscribes:
    joint_trajectory  (trajectory_msgs/JointTrajectory)
  publishes:
    /joint_states     (sensor_msgs/JointState)       <- via joint_state_broadcaster
```

### What Already Exists in SteveROS
- `steveros_hardware/src/steveros_hardware.cpp` -- hardware interface with position + velocity command interfaces
- `steveros_description/urdf/steveros_ros2_control.xacro` -- full robot ros2_control config
- `joint_trajectory_controller` -- already configured and working
- `joint_state_broadcaster` -- already publishing `/joint_states`
- K-Scale production gains -- already tuned per motor (RS04: Kp=85/Kd=5, RS03: Kp=40/Kd=4, RS02: Kp=30/Kd=1)

### What Needs to Be Built/Configured
1. **KBot robot config for teleop_xr** -- end-effector frame names so steps 1-5 compute target poses (disable PyRoki, publish PoseStamped only)
2. **MoveIt2 SRDF + config** -- define planning groups (left_arm, right_arm), collision pairs, joint limits
3. **MoveIt Servo config YAML** -- control rate, velocity scaling, singularity thresholds, input topic remapping
4. **Topic remapping** -- connect teleop_xr's PoseStamped output to MoveIt Servo's input
5. **Launch file** -- orchestrate: teleop_xr -> MoveIt Servo -> joint_trajectory_controller -> hardware

### Why This Architecture

| Alternative | Why Not |
|-------------|---------|
| teleop_xr with PyRoki (built-in IK) | Works but adds a dependency outside the ROS2 ecosystem. KBot not a built-in robot model |
| K-Scale's own VR teleop | May not output ROS2 topics, unclear integration path |
| Pinocchio custom IK node | More control but more work. MoveIt Servo already handles collision avoidance and singularities |
| IK in hardware interface | Wrong layer. Mixing IK with hardware comms creates tight coupling |
| MoveIt Servo chosen | Already in the ROS2 ecosystem, collision-aware, singularity handling, uses existing URDF/SRDF, outputs to existing joint_trajectory_controller |
