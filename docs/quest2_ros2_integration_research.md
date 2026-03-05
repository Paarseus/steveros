# Quest 2 + ROS2 Integration Research for SteveROS

**Date:** 2026-03-05
**Workspace:** SteveROS (KBot 20-DOF Humanoid)
**Target ROS2 Distro:** Jazzy (primary), Humble (MediaPipe nodes)
**Goal:** Connect Meta Quest 2 to this ROS2 workspace for VR-based teleoperation

---

## Workspace Context

SteveROS is a full-stack ROS2 framework for a KBot 20-DOF humanoid robot featuring:
- `ros2_control` hardware interface (Robstride RS03 motors over CAN bus)
- MuJoCo simulation via `mujoco_ros2_control`
- MoveIt2 for arm motion planning (with `pick_ik`)
- MediaPipe-based vision nodes (hand tracking, pose estimation, gesture recognition)
- Default CycloneDDS middleware

The workspace already has MediaPipe hand tracking for camera-based teleoperation.
Adding Quest 2 VR would provide immersive 6-DoF controller/hand tracking with
significantly better spatial accuracy and an immersive visualization layer.

---

## Top Repositories & Projects Evaluated

### 1. teleop_xr (qrafty-ai) -- RECOMMENDED (EASIEST + MOST CAPABLE)

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/qrafty-ai/teleop_xr |
| **Stars** | 30 |
| **License** | Apache-2.0 |
| **ROS2 Support** | Yes (native, distro-agnostic) |
| **Quest 2 Compatible** | Yes (WebXR -- just open URL in Quest browser) |
| **Last Updated** | February 2026 (v1.2.3) |
| **Install** | `pip install teleop-xr` |
| **Ease of Setup** | VERY EASY |

**Why it's the #1 pick for SteveROS:**
- **ZERO installation on Quest** -- just open a URL in the Quest browser
- **WebRTC video streaming** -- low-latency camera feeds directly in headset
- **Passthrough AR mode** -- toggle between VR and AR
- **Whole-body IK teleoperation** -- perfect for a 20-DOF humanoid
- **Bimanual support** -- controls both arms simultaneously (Unitree H1, Franka, etc.)
- **Real-time 3D robot visualization** -- digital twin in the browser
- **Collision-aware IK** via Pyroki (differentiable solver)
- Built on FastAPI (Python) + TypeScript/WebXR frontend
- Also supports dora-rs interface besides ROS2

**Setup (3 commands):**
```bash
pip install teleop-xr
# Install IK deps (pyroki, ballpark) from GitHub
python -m teleop_xr.demo
```
Then open the displayed URL on Quest 2 browser. Done.

**Compatibility with SteveROS:**
- Python 3.10+ (matches ROS2 Jazzy)
- Publishes target poses to ROS2 topics
- IK solver can load your KBot URDF directly
- Can integrate with MoveIt2 servo for real-time control
- No Unity, no APK sideloading, no Android build pipeline

---

### 2. SpesRobotics/teleop -- SIMPLEST OPTION

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/SpesRobotics/teleop |
| **Stars** | 182 (most popular) |
| **License** | Apache-2.0 |
| **ROS2 Support** | Yes (native) |
| **Quest 2 Compatible** | Yes (WebXR -- just open URL in Quest browser) |
| **Last Updated** | December 2025 (v0.1.4) |
| **Install** | `pip install teleop` |
| **Ease of Setup** | EASIEST |

**Why it's great:**
- **The original WebXR teleop** that teleop_xr is forked from
- `pip install teleop` then `python -m teleop.ros2` -- that's it
- 90fps controller tracking from VR headset
- Built-in IK solver (Pinocchio-based) with velocity/acceleration limits
- ROS2 mode publishes target poses and subscribes to current positions
- More battle-tested than teleop_xr (182 stars vs 30)

**ROS2 usage:**
```bash
pip install teleop
# Simple pose publishing:
python -m teleop.ros2
# With IK servoing:
python -m teleop.ros2_ik --joint-names joint1 joint2 ... --ee-link end_effector
```

**Limitations vs teleop_xr:**
- No WebRTC video streaming
- No passthrough AR mode
- No 3D robot visualization in browser
- Single-arm only (no bimanual)

---

### 3. unity_ros_teleoperation (ETH Zurich / Legged Robotics) -- BEST FULL VR

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/leggedrobotics/unity_ros_teleoperation |
| **License** | BSD-3-Clause |
| **ROS2 Support** | Yes (use `main-ros2` branch of ROS-TCP-Endpoint) |
| **Quest 2 Compatible** | Yes (OpenXR-based, works with Quest 2/3/Pro) |
| **Unity Version** | 2022.3.12f1 |
| **Maintained** | Active (v0.1.1, 2024) |
| **Ease of Setup** | Medium |

**Why it's the best fit for SteveROS:**
- **28+ components** including hand tracking, pose publishing, LiDAR visualization, camera streaming, elevation maps, and bidirectional audio
- **OpenXR foundation** means it works with Quest 2 (not locked to Quest 3)
- **Supports multiple robot models** out of the box (Anymal, Franka Panda, etc.) -- easy to add KBot URDF
- **ROS2 native** via TCP Endpoint on the `main-ros2` branch
- **Hand tracking** with Ability model compatibility -- can complement existing MediaPipe setup
- **GPU-accelerated** point cloud and LiDAR rendering in VR
- Published at IROS 2024 (Wilder-Smith, Patil & Hutter - "Radiance Fields for Robotic Teleoperation")

**Setup overview:**
1. Clone repo and open in Unity 2022.3.12f1
2. Build for Android (Quest 2 target)
3. On ROS2 side: clone `ROS-TCP-Endpoint` (`main-ros2` branch) into colcon workspace
4. Run: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your_ip>`
5. Connect Quest to same network, configure endpoint IP in-app

**Compatibility with SteveROS:**
- Works with ROS2 Jazzy (TCP Endpoint is distro-agnostic)
- Can visualize MuJoCo sim state in VR
- Can feed controller poses to MoveIt2 servo for real-time arm control
- Hand tracking data can supplement/replace MediaPipe pipeline

---

### 4. PickNikRobotics/meta_quest_teleoperation -- MOVEIT-SPECIFIC

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/PickNikRobotics/meta_quest_teleoperation |
| **Stars** | 7 |
| **ROS2 Support** | Yes (designed for ROS2) |
| **Quest 2 Compatible** | Yes (Meta Quest headsets) |
| **Maintained** | Active (5 commits, PickNik team) |
| **Ease of Setup** | Easy-Medium |
| **Docs** | https://docs.picknik.ai/hardware_guides/setting_up_the_meta_quest_for_teleop/ |

**Why it's great for SteveROS:**
- **Built specifically for MoveIt** -- SteveROS already uses MoveIt2
- Publishes to `/right_controller_odom` and `/tf` -- standard ROS2 topics
- **Gripper control** via trigger, **clutch mechanism** via grip button
- **Frame rotation** via analog stick for aligning VR frame to robot frame
- Works as a generic ROS2 driver (MoveIt Pro recommended but not required)
- Open source Unity project

**Setup overview:**
1. Clone repo, open in Unity Hub (auto-fetches correct Unity version)
2. Enable Android build target, configure XR Plugin Management
3. Build and deploy to Quest 2
4. On ROS2 side: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your_ip>`
5. Enter PC IP in Quest app keyboard

**Published ROS2 Topics:**
- `/right_controller_odom` (nav_msgs/Odometry)
- `/tf` (geometry_msgs/TransformStamped)
- Gripper state commands

**Compatibility with SteveROS:**
- Direct integration with MoveIt2 servo for real-time arm control
- Works with ROS2 Jazzy via ROS-TCP-Endpoint
- Simple topic mapping to existing joint trajectory controllers

---

### 5. Hand Tracking Streamer (HTS) -- BEST FOR CONTROLLER-FREE TRACKING

| Field | Details |
|-------|---------|
| **Quest App** | https://www.meta.com/experiences/hand-tracking-streamer/26303946202523164/ |
| **SideQuest** | https://sidequestvr.com/app/46236/hand-tracking-streamer |
| **ROS2 Package** | `hand_tracking_sdk_ros2` (in development) |
| **Python SDK** | `pip install hand-tracking-sdk` |
| **Quest 2 Compatible** | Yes |
| **Price** | Free |
| **Posted** | February 2026 (very recent) |

**Key features:**
- **21 hand landmarks per hand** (MediaPipe-compatible format!)
- **6-DoF wrist pose** tracking
- **No controllers needed** -- bare hand tracking
- UDP (low latency) or TCP (reliable) streaming modes
- Left, right, or dual-hand modes
- In-headset live debug console and phantom hand visualization
- Built on Meta Interaction SDK

**Why it's interesting for SteveROS:**
- MediaPipe-compatible landmark format matches existing `mediapipe_ros2_py` nodes
- Could be a drop-in replacement for camera-based MediaPipe with much better accuracy
- ROS2 bridge node publishes wrist pose topics, landmark topics, and TF frames
- No Unity development required -- just install the app

**Caveats:**
- ROS2 package (`hand_tracking_sdk_ros2`) is still in active development
- Interface may change as specs finalize
- No established version compatibility matrix yet

---

### 6. Quest2ROS (KTH Royal Institute of Technology) -- ROS1 ONLY

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/Quest2ROS/quest2ros |
| **Website** | https://quest2ros.github.io/ |
| **Stars** | 36 |
| **Last Commit** | December 2023 |
| **ROS2 Support** | NO (ROS1 only, uses catkin) |
| **Quest 2 Compatible** | Yes |
| **Paper** | HRI 2024 Workshop |

**Performance metrics (published):**
- Tracking accuracy: **0.46 mm** mean error at ~1m distance
- Latency: **82 ms** average
- Update frequency: **71.86 Hz**

**NOT recommended for SteveROS** because:
- ROS1 only (`catkin build`, `rosrun`, `roslaunch`)
- Would require `ros1_bridge` which adds complexity and latency
- Not actively maintained (last commit Dec 2023)

---

### 7. vr-hand-tracking (Northwestern ME 495) -- QUEST 3 FOCUSED

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/NU-MECH-ENG-495/vr-hand-tracking |
| **Stars** | 26 |
| **ROS2 Support** | Yes |
| **Quest 2 Compatible** | Targets Quest 3/3s but may work with Quest 2 |
| **Unity Version** | Unity 6 |

**Features:**
- Captures hand joint angles via Meta OpenXR SDK
- Broadcasts via UDP to a C++ ROS2 node
- Provides RViz visualization
- Optional robotic hand remote control integration

**Caveats:**
- Primarily targets Quest 3s (newer hardware)
- Course project (may not be actively maintained long-term)
- Uses Unity 6 (newer than most other projects)

---

### 8. vr_ros2_bridge (UM ARM Lab) -- HTC VIVE FOCUSED

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/UM-ARM-Lab/vr_ros2_bridge |
| **Stars** | 11 |
| **ROS2 Support** | Yes |
| **Quest 2 Compatible** | No (HTC Vive only, requires SteamVR) |

**Not recommended** -- designed for HTC Vive with SteamVR on Windows. Quest 2 uses a different ecosystem.

---

## Architecture Comparison

### Approach A: WebXR + Python Server (RECOMMENDED -- Easiest)

```
Quest 2 (Browser/WebXR) --[WebSocket/WebRTC]--> Python Server --[ROS2 Topics]--> SteveROS
```

**Pros:**
- ZERO installation on Quest -- just open a URL
- pip install on host, 3 commands total
- 90fps controller tracking
- teleop_xr adds WebRTC video, bimanual IK, passthrough AR
- No Unity, no Android SDK, no APK sideloading
- Works with any WebXR-capable device (Quest 2/3/Pro, Pico, etc.)

**Cons:**
- Browser sandbox limits some hardware features
- Less rich 3D visualization than native Unity app (teleop_xr partially addresses this)
- Depends on Quest browser WebXR implementation

---

### Approach B: Unity + ROS-TCP-Connector (Most Feature-Rich)

```
Quest 2 (Unity App) --[TCP]--> ROS-TCP-Endpoint --[ROS2 Topics]--> SteveROS
```

**Pros:**
- Most mature approach, used by ETH, PickNik, and Unity Technologies
- Bidirectional communication (can send haptic feedback back)
- Rich visualization (render robot model, sensors in VR)
- Works across ROS2 distros (TCP is transport-layer agnostic)

**Cons:**
- Requires Unity development for customization
- Must build and deploy Android APK to Quest
- ROS-TCP-Connector has known connectivity issues on Quest 2 (GitHub Issues #353, #359, #280) -- workaround: ensure same subnet, disable firewall, use static IPs

### Approach C: Hand Tracking Streamer + Python SDK (Controller-Free)

```
Quest 2 (HTS App) --[UDP/TCP]--> Python SDK --[ROS2 bridge]--> SteveROS
```

**Pros:**
- No Unity development at all
- Free app from Meta Store
- MediaPipe-compatible output (matches existing SteveROS pipeline)
- Lowest barrier to entry

**Cons:**
- ROS2 package still in development
- No immersive visualization (Quest only used as input device)
- No controller button input (hand tracking only)

### Approach D: WebXR + rosbridge_suite (Legacy Alternative)

```
Quest 2 (Browser/WebXR) --[WebSocket]--> rosbridge_server --[ROS2 Topics]--> SteveROS
```

**Pros:**
- No app installation on Quest
- Web-based, easy to iterate
- rosbridge_suite is well-maintained

**Cons:**
- Higher latency (WebSocket overhead)
- Limited hand tracking quality in browser
- More complex web development required
- WebXR API limitations on Quest browser

### Approach E: MQTT Bridge (Fallback)

```
Quest 2 (Unity App) --[MQTT]--> Mosquitto Broker --[MQTT-ROS2 Bridge]--> SteveROS
```

**Pros:**
- Avoids ROS-TCP-Connector connectivity issues on Quest 2
- MQTT is lightweight and reliable

**Cons:**
- Additional infrastructure (MQTT broker)
- Less common pattern, fewer examples
- Still requires Unity development

---

## Final Recommendation for SteveROS

### PRIMARY: teleop_xr (qrafty-ai) -- Start Here

**Why this is the best fit:**
1. **Zero install on Quest** -- open a URL and go. No Unity, no APK, no sideloading
2. **Bimanual whole-body IK** -- perfect for a 20-DOF humanoid with two arms
3. **WebRTC video** -- see robot cameras in the headset with minimal latency
4. **Passthrough AR** -- toggle between VR and AR modes
5. **pip install** -- 3 commands to running demo
6. **Collision-aware IK** -- differentiable solver via Pyroki
7. **Active development** -- updated Feb 2026, Apache-2.0 license
8. Works with ROS2 Jazzy (Python 3.10+, distro-agnostic)

### FALLBACK: SpesRobotics/teleop -- Even Simpler

**Why:**
1. More mature (182 stars, v0.1.4)
2. `pip install teleop` + `python -m teleop.ros2` = done
3. Same WebXR architecture but simpler (single-arm focus)
4. Good stepping stone before teleop_xr

### FOR FULL IMMERSION: unity_ros_teleoperation (ETH Zurich)

**Why:**
1. Most feature-complete VR teleoperation framework available
2. Full 3D robot visualization, stereo cameras, point clouds in VR
3. OpenXR-based = Quest 2 compatible
4. ROS2 support via TCP Endpoint (works with Jazzy)
5. Published research backing (IROS 2024)
6. Best option if you need a rich immersive digital twin

### MOVEIT-SPECIFIC: PickNikRobotics/meta_quest_teleoperation

**Why:**
1. Designed specifically for MoveIt teleoperation (SteveROS uses MoveIt2)
2. Publishes standard odometry + TF topics
3. PickNik is the team behind MoveIt -- guaranteed compatibility

### FUTURE WATCH: Hand Tracking Streamer

**Why:**
1. Once the ROS2 package stabilizes, this becomes the best controller-free option
2. MediaPipe-compatible output = near drop-in for existing SteveROS pipeline
3. No Unity development needed
4. Free and available on Quest Store today

---

## Integration Roadmap for SteveROS

### Phase 1: Immediate Prototype (< 1 hour)
1. `pip install teleop` on ROS2 machine
2. Run `python -m teleop.ros2`
3. Open displayed URL on Quest 2 browser
4. Verify controller tracking works with ROS2 topics
5. Test with MuJoCo sim (subscribe to target pose, command joint trajectories)

### Phase 2: Bimanual Teleop (1-2 days)
1. `pip install teleop-xr` + install pyroki/ballpark deps
2. Configure KBot URDF for whole-body IK
3. Set up WebRTC camera streaming from robot/sim
4. Test bimanual arm control in passthrough AR mode
5. Map IK output to MoveIt2 servo or direct joint trajectory controllers

### Phase 3: Full Immersion (1-2 weeks, optional)
1. Set up `unity_ros_teleoperation` (ETH) for rich VR experience
2. Import KBot URDF into Unity project
3. Configure MuJoCo camera feeds for stereo VR rendering
4. Set up bidirectional communication (haptic feedback from contact forces)
5. Add hand tracking as alternative to controllers

### Phase 4: Production Pipeline
1. Evaluate Hand Tracking Streamer once ROS2 package is stable
2. Compare tracking quality: Quest hand tracking vs. MediaPipe camera
3. Integrate best option into `steveros_bringup` launch files
4. Add VR teleop mode to roadmap

---

## Key Links

| Resource | URL |
|----------|-----|
| **teleop_xr (RECOMMENDED)** | https://github.com/qrafty-ai/teleop_xr |
| SpesRobotics/teleop | https://github.com/SpesRobotics/teleop |
| unity_ros_teleoperation | https://github.com/leggedrobotics/unity_ros_teleoperation |
| meta_quest_teleoperation | https://github.com/PickNikRobotics/meta_quest_teleoperation |
| Hand Tracking Streamer | https://www.meta.com/experiences/hand-tracking-streamer/26303946202523164/ |
| Quest2ROS (ROS1 only) | https://github.com/Quest2ROS/quest2ros |
| vr-hand-tracking | https://github.com/NU-MECH-ENG-495/vr-hand-tracking |
| vr_ros2_bridge | https://github.com/UM-ARM-Lab/vr_ros2_bridge |
| ROS-TCP-Endpoint (ROS2) | https://github.com/Unity-Technologies/ROS-TCP-Endpoint (main-ros2 branch) |
| ROS-TCP-Connector | https://github.com/Unity-Technologies/ROS-TCP-Connector |
| MoveIt Pro Quest Guide | https://docs.picknik.ai/hardware_guides/setting_up_the_meta_quest_for_teleop/ |
| Quest2ROS Paper | https://openreview.net/forum?id=d3MTESE2e8 |
| HTS Discourse Post | https://discourse.openrobotics.org/t/hand-tracking-streamer-quest-hand-wrist-telemetry-streaming-tool/52511 |
